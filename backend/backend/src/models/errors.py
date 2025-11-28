from pydantic import BaseModel
from typing import Any, Dict, Optional

class ErrorResponse(BaseModel):
    code: str
    message: str
    details: Optional[Dict[str, Any]] = None