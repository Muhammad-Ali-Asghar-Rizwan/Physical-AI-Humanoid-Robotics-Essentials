from typing import List
from .loader import Document
import re

def chunk_documents(documents: List[Document], chunk_size: int = 800) -> List[Document]:
    chunks = []
    for doc in documents:
        # Split by sentences, using Python's re module
        sentences = re.split(r'(?<=[.?!])\s+', doc.text)
        
        current_chunk_text = ''
        for sentence in sentences:
            if len(current_chunk_text) + len(sentence) > chunk_size:
                chunks.append(Document(text=current_chunk_text.strip(), metadata=doc.metadata))
                current_chunk_text = ''
            current_chunk_text += sentence + ' '
        
        if current_chunk_text:
            chunks.append(Document(text=current_chunk_text.strip(), metadata=doc.metadata))
    return chunks