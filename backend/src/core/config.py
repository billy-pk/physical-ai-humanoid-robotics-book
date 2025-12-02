from pydantic_settings import BaseSettings, SettingsConfigDict # Import SettingsConfigDict for Pydantic v2
from typing import Optional
from pathlib import Path # Import Path

class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=Path(__file__).parent.parent.parent / ".env", # Look for .env in the backend directory
        env_file_encoding="utf-8"
    )

    OPENAI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
    BETTER_AUTH_SERVICE_URL: Optional[str] = None  # URL of Better Auth service (e.g., http://localhost:3001)
    DATABASE_POOL_SIZE: int = 5  # Connection pool size for async database connections
    DATABASE_MAX_OVERFLOW: int = 10  # Maximum overflow connections beyond pool_size

settings = Settings()