"""
RAG Agent using OpenAI Agents SDK for answering questions about the Physical AI book.

This module defines an agent that:
- Uses the OpenAI Agents SDK (Runner class)
- Operates on GPT-4o-mini for cost efficiency
- Answers questions based ONLY on provided context from book chapters
- Cites specific sections when providing answers
- Handles out-of-scope queries gracefully
"""

from agents import Agent, Runner
from ...core.config import settings
from ...core.logging import logger
from typing import List, Optional, Dict


# Define the RAG agent with instructions
rag_agent = Agent(
    name="Physical AI Book Assistant",
    model="gpt-4o-mini",
    instructions="""You are an expert AI teaching assistant for the "Physical AI and Humanoid Robotics" educational book.

Your responsibilities:
1. Answer student questions based ONLY on the provided context from the book chapters
2. Cite specific sections/chapters when providing answers (e.g., "According to Chapter 2.1...")
3. If the context doesn't contain the answer, clearly state: "I cannot answer this question based on the current book content. Please ask about topics covered in the documentation."
4. Be educational, encouraging, and clear in your explanations
5. Use examples from the context when available
6. Break down complex concepts into understandable parts

Guidelines:
- NEVER make up information not in the context
- NEVER answer questions outside the book's scope
- Always prioritize accuracy over completeness
- Encourage students to explore related chapters when relevant
- Maintain a friendly, supportive teaching tone
- Keep responses concise but thorough""",
)


async def generate_rag_response_with_agent(
    user_query: str,
    context_chunks: List[str],
    chat_history: Optional[List[Dict[str, str]]] = None,
) -> tuple[str, int]:
    """
    Generate a response using the OpenAI Agent with RAG context.

    This function:
    1. Takes the user's query and retrieved book content chunks
    2. Formats them into a structured prompt for the agent
    3. Uses the Runner class to execute the agent
    4. Returns the agent's response and estimated token count

    Args:
        user_query: The student's question
        context_chunks: Retrieved relevant passages from the book (from vector search)
        chat_history: Previous conversation messages (optional, not currently used)

    Returns:
        Tuple of (agent_response_text, tokens_used)

    Raises:
        Exception: If agent execution fails
    """
    try:
        # Format context into numbered sections for the agent
        formatted_context = "\n\n".join([
            f"[Book Context {i+1}]\n{chunk}"
            for i, chunk in enumerate(context_chunks)
        ])

        # Build the full prompt with context and query
        full_prompt = f"""Here is the relevant context from the Physical AI and Humanoid Robotics book:

{formatted_context}

---

Student Question: {user_query}

Please answer the student's question using ONLY the information provided in the context above. Cite which context section(s) you're using when relevant."""

        logger.info(
            "Calling RAG agent via Runner",
            query_length=len(user_query),
            context_chunks_count=len(context_chunks),
            total_context_length=len(formatted_context),
            has_history=chat_history is not None
        )

        # Run the agent asynchronously using Runner
        result = await Runner.run(
            rag_agent,
            full_prompt,
        )

        # Extract the final output from the agent
        response_text = result.final_output

        # Estimate tokens (Runner doesn't expose token count directly yet)
        # In production, you'd extract this from result metadata if available
        estimated_tokens = (
            len(full_prompt.split()) + len(response_text.split())
        ) * 1.3  # Rough token estimate (1.3 words per token)

        logger.info(
            "RAG agent response generated successfully",
            response_length=len(response_text),
            estimated_tokens=int(estimated_tokens)
        )

        return response_text, int(estimated_tokens)

    except Exception as e:
        logger.error(
            "RAG agent generation failed",
            error=str(e),
            error_type=type(e).__name__,
            query_preview=user_query[:100]
        )
        raise


def generate_rag_response_sync(
    user_query: str,
    context_chunks: List[str],
) -> tuple[str, int]:
    """
    Synchronous version of RAG response generation.

    Useful for non-async contexts or scripts.

    Args:
        user_query: The student's question
        context_chunks: Retrieved relevant passages from the book

    Returns:
        Tuple of (agent_response_text, tokens_used)
    """
    formatted_context = "\n\n".join([
        f"[Book Context {i+1}]\n{chunk}"
        for i, chunk in enumerate(context_chunks)
    ])

    full_prompt = f"""Here is the relevant context from the Physical AI and Humanoid Robotics book:

{formatted_context}

---

Student Question: {user_query}

Please answer the student's question using ONLY the information provided in the context above."""

    # Run synchronously using Runner.run_sync
    result = Runner.run_sync(rag_agent, full_prompt)

    response_text = result.final_output
    estimated_tokens = int((len(full_prompt.split()) + len(response_text.split())) * 1.3)

    return response_text, estimated_tokens


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
