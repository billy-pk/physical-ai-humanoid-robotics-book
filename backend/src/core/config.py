from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    OPENAI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
    BETTER_AUTH_SERVICE_URL: Optional[str] = None  # URL of Better Auth service (e.g., http://localhost:3001)

    class Config:
        env_file = ".env"

settings = Settings()