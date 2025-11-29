from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Optional
import uuid
from datetime import datetime

from ...core.logging import logger
from ...models.chat import ChatMessage, ChatSession, MessageRole, Citation
from ...models.errors import ErrorResponse
from ...services.rag.rag_agent import ask_question

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
    1. Pass user query to RAG agent
    2. Agent automatically uses its search_book_content tool to find relevant information
    3. Agent generates response based on retrieved content with citations
    4. Return response to user

    The agent handles all RAG logic internally including:
    - Vector search via the search_book_content tool
    - Context retrieval from Qdrant
    - Grounded response generation
    - Citation of sources

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

        # Optionally prepend highlighted context to the query
        query = request.query
        if request.highlighted_context:
            query = (
                f"Context from the page I'm reading:\n\n{request.highlighted_context}\n\n"
                f"My question: {request.query}"
            )
            logger.info("Added highlighted context to query", session_id=str(session_id))

        # Ask the RAG agent - it will use its search tool internally
        answer, tokens_used = await ask_question(
            user_query=query,
            chat_history=None  # TODO: Retrieve from database in future
        )

        # Create chat message (citations are embedded in agent's response text)
        message = ChatMessage(
            session_id=session_id,
            role=MessageRole.ASSISTANT,
            content=answer,
            citations=[],  # Agent cites sources in its response text
            highlighted_context=request.highlighted_context,
            tokens_used=tokens_used
        )

        logger.info(
            "Chat response generated via agent",
            session_id=str(session_id),
            tokens=tokens_used,
            response_length=len(answer)
        )

        return ChatResponse(
            session_id=session_id,
            message=message,
            citations=[],  # Citations are in the response text
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
