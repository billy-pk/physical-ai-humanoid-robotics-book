from typing import AsyncGenerator
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from urllib.parse import urlparse, parse_qs, urlencode, urlunparse
from .core.config import settings
from .core.logging import logger

# Ensure the database URL is an async-compatible one (e.g., postgresql+asyncpg)
# Convert postgresql:// to postgresql+asyncpg:// if needed
# Also fix SSL parameters for asyncpg (asyncpg doesn't support sslmode parameter)
database_url = settings.NEON_DATABASE_URL

# Parse the URL to handle SSL parameters correctly
if database_url.startswith("postgresql://") and "+asyncpg" not in database_url:
    # Parse the URL
    parsed = urlparse(database_url)
    
    # Parse query parameters
    query_params = parse_qs(parsed.query)
    
    # Remove parameters that asyncpg doesn't support
    # asyncpg uses SSL automatically for secure connections
    removed_params = []
    if 'sslmode' in query_params:
        del query_params['sslmode']
        removed_params.append('sslmode')
    if 'channel_binding' in query_params:
        del query_params['channel_binding']
        removed_params.append('channel_binding')
    
    if removed_params:
        logger.info(f"Removed unsupported parameters for asyncpg: {', '.join(removed_params)}")
    
    # Rebuild query string
    new_query = urlencode(query_params, doseq=True)
    
    # Reconstruct URL with postgresql+asyncpg://
    new_parsed = parsed._replace(
        scheme="postgresql+asyncpg",
        query=new_query
    )
    database_url = urlunparse(new_parsed)
    logger.info("Converted database URL to async format")

# Async engine
# asyncpg automatically uses SSL for secure connections (like Neon Postgres)
# We just need to ensure the URL doesn't have sslmode parameter (which we removed above)
async_engine = create_async_engine(
    database_url,
    echo=False, # Set to True for SQL logging
    pool_size=settings.DATABASE_POOL_SIZE,
    max_overflow=settings.DATABASE_MAX_OVERFLOW,
)

# Async session maker
async_session_maker = async_sessionmaker(
    async_engine,
    expire_on_commit=False,
    class_=AsyncSession,
)

async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency to provide a database session.
    Manages session lifecycle: opens, yields, and closes.
    """
    session: AsyncSession = async_session_maker()
    try:
        yield session
    except Exception as e:
        logger.error(f"Database session error: {e}")
        await session.rollback()
        raise
    finally:
        await session.close()
