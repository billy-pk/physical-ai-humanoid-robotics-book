from openai import AsyncOpenAI
from backend.src.core.config import settings
from backend.src.core.logging import logger
from typing import List, Optional


client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


async def generate_embeddings(text: str) -> List[float]:
    """
    Generate embeddings for text using OpenAI's text-embedding-3-large model.

    Args:
        text: Text content to embed

    Returns:
        List of 3072 floats representing the embedding vector
    """
    try:
        response = await client.embeddings.create(
            model="text-embedding-3-large",
            input=text,
            dimensions=3072
        )

        embedding = response.data[0].embedding

        logger.info(
            "Generated embedding",
            text_length=len(text),
            embedding_dim=len(embedding)
        )

        return embedding

    except Exception as e:
        logger.error("Embedding generation failed", error=str(e), text_preview=text[:100])
        raise


async def generate_chat_response(
    messages: List[dict],
    model: str = "gpt-4o-mini",
    temperature: float = 0.7,
    max_tokens: Optional[int] = None
) -> tuple[str, int]:
    """
    Generate a chat completion using OpenAI's API.

    Args:
        messages: List of message dicts with 'role' and 'content' keys
        model: OpenAI model to use (default: gpt-4o-mini for cost efficiency)
        temperature: Sampling temperature (0.0 to 2.0)
        max_tokens: Maximum tokens in response (None = model default)

    Returns:
        Tuple of (response_text, tokens_used)
    """
    try:
        response = await client.chat.completions.create(
            model=model,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens
        )

        content = response.choices[0].message.content
        tokens = response.usage.total_tokens

        logger.info(
            "Generated chat response",
            model=model,
            tokens=tokens,
            message_count=len(messages)
        )

        return content, tokens

    except Exception as e:
        logger.error("Chat completion failed", error=str(e), model=model)
        raise


async def generate_rag_response(
    user_query: str,
    context_chunks: List[str],
    chat_history: Optional[List[dict]] = None,
    model: str = "gpt-4o-mini"
) -> tuple[str, int]:
    """
    Generate a RAG (Retrieval Augmented Generation) response.

    Combines retrieved context with user query to generate accurate responses
    grounded in the book content.

    Args:
        user_query: User's question
        context_chunks: Retrieved relevant text chunks from the book
        chat_history: Previous messages in the conversation (optional)
        model: OpenAI model to use

    Returns:
        Tuple of (response_text, tokens_used)
    """
    # Build context string
    context = "\n\n".join([
        f"[Context {i+1}]\n{chunk}"
        for i, chunk in enumerate(context_chunks)
    ])

    # System prompt for RAG
    system_message = {
        "role": "system",
        "content": """You are a helpful AI assistant for the "Physical AI and Humanoid Robotics" educational book.

Your role:
- Answer questions based ONLY on the provided context from the book
- If the context doesn't contain the answer, say "I don't have enough information in the current context to answer that question."
- Cite specific sections when providing answers
- Be clear, educational, and encouraging
- Use examples from the context when available

Always prioritize accuracy over completeness. It's better to say you don't know than to guess."""
    }

    # Build messages
    messages = [system_message]

    # Add chat history if provided
    if chat_history:
        messages.extend(chat_history[-6:])  # Keep last 6 messages for context

    # Add context and user query
    messages.append({
        "role": "user",
        "content": f"""Context from the book:

{context}

---

User question: {user_query}"""
    })

    return await generate_chat_response(messages, model=model, temperature=0.7)
