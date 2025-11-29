from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Optional
import uuid
from datetime import datetime

from ...core.logging import logger
from ...models.chat import ChatMessage, ChatSession, MessageRole, Citation
from ...models.errors import ErrorResponse
from ...services.llm.openai_client import generate_embeddings
from ...services.vectordb.qdrant_client import search_book_embeddings
from ...services.rag.rag_agent import generate_rag_response_with_agent, handle_out_of_scope_query

router = APIRouter(prefix="/api/chat", tags=["chat"])


# Request/Response models
class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    session_id: Optional[uuid.UUID] = Field(None, description="Chat session ID (optional)")
    highlighted_context: Optional[str] = Field(None, description="Highlighted text from page")
    current_page: Optional[str] = Field(None, description="Current documentation page URL")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    session_id: uuid.UUID
    message: ChatMessage
    citations: List[Citation]
    answer: str


@router.post("", response_model=ChatResponse, status_code=status.HTTP_200_OK)
async def chat(request: ChatRequest):
    """
    Handle chat interactions with RAG (Retrieval Augmented Generation).

    Flow:
    1. Generate embedding for user query
    2. Search Qdrant for relevant book chunks
    3. Optionally incorporate highlighted context
    4. Generate response using OpenAI Agent (via Agents SDK) with retrieved context
    5. Extract citations from retrieved chunks
    6. Return response with citations

    Args:
        request: ChatRequest containing query and optional context

    Returns:
        ChatResponse with answer, session info, and citations

    Raises:
        HTTPException: If any step fails
    """
    try:
        # Generate session ID if not provided
        session_id = request.session_id or uuid.uuid4()

        logger.info(
            "Chat request received",
            session_id=str(session_id),
            query_length=len(request.query),
            has_highlight=request.highlighted_context is not None
        )

        # Step 1: Generate query embedding
        query_embedding = await generate_embeddings(request.query)

        # Step 2: Search for relevant chunks
        search_results = await search_book_embeddings(
            query_vector=query_embedding,
            limit=5,
            score_threshold=0.7
        )

        if not search_results:
            logger.warning("No relevant content found", session_id=str(session_id))
            out_of_scope_response = await handle_out_of_scope_query(request.query)
            return ChatResponse(
                session_id=session_id,
                message=ChatMessage(
                    session_id=session_id,
                    role=MessageRole.ASSISTANT,
                    content=out_of_scope_response,
                    citations=[]
                ),
                citations=[],
                answer=out_of_scope_response
            )

        # Step 3: Prepare context chunks
        context_chunks = [result.content for result in search_results]

        # Add highlighted context if provided (higher priority)
        if request.highlighted_context:
            context_chunks.insert(0, f"[User highlighted text]: {request.highlighted_context}")
            logger.info("Added highlighted context", session_id=str(session_id))

        # Step 4: Generate RAG response using OpenAI Agent
        answer, tokens_used = await generate_rag_response_with_agent(
            user_query=request.query,
            context_chunks=context_chunks,
            chat_history=None  # TODO: Retrieve from database in future
        )

        # Step 5: Create citations from search results
        citations = [
            Citation(
                chunk_id=result.chunk_id,
                chapter_title=result.chapter_title,
                section=result.section_title,
                relevance_score=result.score,
                source_url=result.source_url or "#"
            )
            for result in search_results
        ]

        # Step 6: Create chat message
        message = ChatMessage(
            session_id=session_id,
            role=MessageRole.ASSISTANT,
            content=answer,
            citations=citations,
            highlighted_context=request.highlighted_context,
            tokens_used=tokens_used
        )

        logger.info(
            "Chat response generated",
            session_id=str(session_id),
            tokens=tokens_used,
            citations_count=len(citations),
            relevance_scores=[c.relevance_score for c in citations]
        )

        return ChatResponse(
            session_id=session_id,
            message=message,
            citations=citations,
            answer=answer
        )

    except Exception as e:
        logger.error(
            "Chat request failed",
            error=str(e),
            query=request.query[:100]
        )
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=ErrorResponse(
                code="CHAT_ERROR",
                message="Failed to process chat request. Please try again.",
                details={"error": str(e)}
            ).model_dump()
        )


@router.get("/sessions/{session_id}", status_code=status.HTTP_200_OK)
async def get_session(session_id: uuid.UUID):
    """
    Retrieve chat session history (placeholder for future implementation).

    Args:
        session_id: UUID of the chat session

    Returns:
        Chat session with message history
    """
    # TODO: Implement session retrieval from Postgres
    logger.info("Session retrieval requested", session_id=str(session_id))

    return {
        "session_id": session_id,
        "messages": [],
        "note": "Session history not yet implemented"
    }


@router.delete("/sessions/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_session(session_id: uuid.UUID):
    """
    Delete a chat session (placeholder for future implementation).

    Args:
        session_id: UUID of the session to delete

    Returns:
        No content (204)
    """
    # TODO: Implement session deletion in Postgres
    logger.info("Session deletion requested", session_id=str(session_id))
    return None
