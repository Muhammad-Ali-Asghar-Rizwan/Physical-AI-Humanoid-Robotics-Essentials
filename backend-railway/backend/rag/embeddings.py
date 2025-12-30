from typing import List
from .loader import Document
from ..config import cohere_client

def generate_embeddings(documents: List[Document]) -> List[List[float]]:
    texts = [doc.text for doc in documents]
    
    # Cohere has a limit of 96 texts per request
    batch_size = 90
    all_embeddings = []
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        response = cohere_client.embed(
            texts=batch_texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        all_embeddings.extend(response.embeddings)
    return all_embeddings
