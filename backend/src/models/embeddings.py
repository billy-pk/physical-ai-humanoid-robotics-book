import uuid
from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field

class EmbeddingChunk(BaseModel):
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    content: str
    content_hash: str
    vector: List[float] = Field(..., min_length=3072, max_length=3072)
    chapter_id: str
    chapter_title: str
    section_title: Optional[str] = None
    chunk_index: int = Field(..., ge=0)
    token_count: int
    created_at: datetime = Field(default_factory=datetime.now)
    metadata: Dict[str, Any] = {}

class VectorSearchResult(BaseModel):
    chunk_id: str
    content: str
    score: float = Field(..., ge=0.0, le=1.0)
    chapter_title: str
    section_title: Optional[str] = None
    source_url: str
    metadata: Dict[str, Any] = {}