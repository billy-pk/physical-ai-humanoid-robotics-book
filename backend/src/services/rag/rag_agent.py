"""
RAG Agent using OpenAI Agents SDK for answering questions about the Physical AI book.

This module defines an agent that:
- Uses the OpenAI Agents SDK (Runner class) with function tools
- Operates on GPT-4o-mini for cost efficiency
- Uses a vector search tool to retrieve relevant book content
- Answers questions based ONLY on retrieved context from book chapters
- Cites specific sections when providing answers
- Handles out-of-scope queries gracefully
"""

from agents import Agent, Runner, function_tool, set_default_openai_key
from ...core.config import settings
from ...core.logging import logger
from ...services.llm.openai_client import generate_embeddings
from ...services.vectordb.qdrant_client import search_book_embeddings as _search_qdrant
from typing import List, Optional, Dict

# Configure OpenAI API key for the agents SDK
set_default_openai_key(settings.OPENAI_API_KEY)


@function_tool
async def search_book_content(query: str, limit: int = 5) -> str:
    """
    Search the Physical AI and Humanoid Robotics book for relevant content.

    This tool searches through the book's chapters and sections to find
    content relevant to the user's question. Use this tool whenever you
    need to find information to answer a student's question.

    Args:
        query: The search query or question to find relevant content for
        limit: Maximum number of relevant sections to retrieve (default: 5)

    Returns:
        A formatted string containing relevant book sections with their
        chapter titles and content
    """
    try:
        logger.info(
            "Vector search tool called",
            query_preview=query[:100],
            limit=limit
        )

        # Generate embedding for the query
        query_vector = await generate_embeddings(query)

        # Search Qdrant for relevant chunks (sync function)
        search_results = _search_qdrant(
            query_vector=query_vector,
            limit=limit,
            score_threshold=0.5  # Lower threshold for better recall
        )

        if not search_results:
            return (
                "No relevant content found in the book for this query. "
                "The question may be outside the scope of the Physical AI and Humanoid Robotics book. "
                "Topics covered include: AI fundamentals, computer vision for robotics, "
                "machine learning, Python programming for AI, and mathematical foundations."
            )

        # Format results for the agent
        formatted_results = []
        for i, result in enumerate(search_results, 1):
            formatted_results.append(
                f"[Section {i}] {result.chapter_title}"
                + (f" - {result.section_title}" if result.section_title else "")
                + f"\n{result.content}"
                + f"\n(Relevance: {result.score:.2f})"
            )

        formatted_output = "\n\n---\n\n".join(formatted_results)

        logger.info(
            "Vector search completed",
            results_found=len(search_results),
            avg_score=sum(r.score for r in search_results) / len(search_results)
        )

        return formatted_output

    except Exception as e:
        logger.error(
            "Vector search tool failed",
            error=str(e),
            error_type=type(e).__name__
        )
        return f"Error searching book content: {str(e)}"


# Define the RAG agent with the search tool
rag_agent = Agent(
    name="Physical AI Book Assistant",
    model="gpt-4o-mini",
    instructions="""You are an expert AI teaching assistant for the "Physical AI and Humanoid Robotics" educational book.

Your responsibilities:
1. When a student asks a question, ALWAYS use the search_book_content tool to find relevant information
2. Answer questions based ONLY on the content retrieved by the search tool
3. Cite specific sections/chapters when providing answers (e.g., "According to the section on...")
4. If the search tool returns no relevant content, politely explain that the question is outside the book's scope
5. Be educational, encouraging, and clear in your explanations
6. Use examples from the retrieved context when available
7. Break down complex concepts into understandable parts

Guidelines:
- ALWAYS use the search_book_content tool before answering - never answer from general knowledge
- NEVER make up information not in the search results
- NEVER answer questions outside the book's scope
- Always prioritize accuracy over completeness
- Cite which sections you're referencing (use the [Section N] identifiers from search results)
- Encourage students to explore related chapters when relevant
- Maintain a friendly, supportive teaching tone
- Keep responses concise but thorough

Remember: You MUST use the search tool for every question. Do not answer without searching first.""",
    tools=[search_book_content],
)


async def ask_question(
    user_query: str,
    chat_history: Optional[List[Dict[str, str]]] = None,
) -> tuple[str, int]:
    """
    Ask the RAG agent a question about the Physical AI book.

    The agent will:
    1. Use its search_book_content tool to find relevant information
    2. Generate a response based on the retrieved content
    3. Cite specific sections when providing answers

    Args:
        user_query: The student's question
        chat_history: Previous conversation messages (optional, not currently used)

    Returns:
        Tuple of (agent_response_text, tokens_used)

    Raises:
        Exception: If agent execution fails
    """
    try:
        logger.info(
            "Running RAG agent with tool-based search",
            query_length=len(user_query),
            has_history=chat_history is not None
        )

        # Run the agent - it will automatically use the search_book_content tool
        result = await Runner.run(
            rag_agent,
            user_query,
        )

        # Extract the final output from the agent
        response_text = result.final_output

        # Estimate tokens based on query and response
        # (In production, extract from result.usage if available)
        estimated_tokens = int(
            (len(user_query.split()) + len(response_text.split())) * 1.3
        )

        logger.info(
            "RAG agent response generated successfully",
            response_length=len(response_text),
            estimated_tokens=estimated_tokens,
            tool_calls=len([item for item in result.new_items if hasattr(item, 'tool_call_id')])
        )

        return response_text, estimated_tokens

    except Exception as e:
        logger.error(
            "RAG agent execution failed",
            error=str(e),
            error_type=type(e).__name__,
            query_preview=user_query[:100]
        )
        raise


async def handle_out_of_scope_query(user_query: str) -> str:
    """
    Handle queries that are outside the book's scope.

    This is called when vector search returns no results or very low-confidence results.

    Args:
        user_query: The user's question

    Returns:
        A polite, helpful message explaining the limitation
    """
    query_preview = user_query[:80] + "..." if len(user_query) > 80 else user_query

    return (
        f"I cannot answer questions outside the Physical AI and Humanoid Robotics book content. "
        f"Your question '{query_preview}' doesn't appear to be covered in the available documentation.\n\n"
        f"Topics I can help with include:\n"
        f"• Foundations of AI and robotics\n"
        f"• Computer vision for robotics\n"
        f"• Machine learning fundamentals\n"
        f"• Python programming for AI\n"
        f"• Mathematical foundations for robotics\n\n"
        f"Please try rephrasing your question to focus on these areas!"
    )
