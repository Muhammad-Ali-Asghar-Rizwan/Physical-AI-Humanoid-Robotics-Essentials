from pydantic import BaseModel
from typing import Optional, List

class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    detailed_answer: Optional[str] = None
    source_references: Optional[List[str]] = None
