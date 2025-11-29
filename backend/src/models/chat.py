import uuid
from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum
from pydantic import BaseModel, Field

class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"

class Citation(BaseModel):
    chunk_id: str
    chapter_title: str
    section: Optional[str] = None
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    source_url: str

class ChatMessage(BaseModel):
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    session_id: uuid.UUID
    role: MessageRole
    content: str = Field(..., min_length=1)
    citations: List[Citation] = []
    highlighted_context: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.now)
    tokens_used: Optional[int] = None

    class Config:
        use_enum_values = True

class ChatSession(BaseModel):
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    created_at: datetime = Field(default_factory=datetime.now)
    last_activity: datetime = Field(default_factory=datetime.now)
    user_context: Dict[str, Any] = {}
    metadata: Dict[str, Any] = {}