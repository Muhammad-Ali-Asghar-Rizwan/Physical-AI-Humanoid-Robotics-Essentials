from typing import List, Dict, Any
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from ..config import qdrant_client, COLLECTION_NAME
from .loader import Document

def recreate_qdrant_collection(vector_size: int = 1024):
    """Recreates the Qdrant collection."""
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
    )
    print(f"Collection '{COLLECTION_NAME}' recreated with vector size {vector_size}.")

def upsert_documents_to_qdrant(documents: List[Document], embeddings: List[List[float]]):
    """Upserts documents and their embeddings to Qdrant."""
    points = []
    for i, doc in enumerate(documents):
        points.append(
            PointStruct(
                id=i,
                vector=embeddings[i],
                payload={"text": doc.text, **doc.metadata},
            )
        )
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        wait=True,
        points=points,
    )
    print(f"Upserted {len(documents)} documents to '{COLLECTION_NAME}'.")

def search_qdrant(query_vector: List[float], limit: int = 5) -> List[Dict[str, Any]]:
    """Performs a similarity search in Qdrant."""
    try:
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=limit,
            with_payload=True,
        )
        documents = [
            {
                "text": hit.payload.get("text", ""),
                "source": hit.payload.get("source", "unknown")
            }
            for hit in search_result
            if hit.payload
        ]
        return documents
    except Exception as e:
        print(f"Error during Qdrant search: {e}")
        # If search fails, try the newer query_points method
        try:
            search_result = qdrant_client.query_points(
                collection_name=COLLECTION_NAME,
                query=query_vector,
                limit=limit,
                with_payload=True,
            )
            documents = [
                {
                    "text": hit.payload.get("text", ""),
                    "source": hit.payload.get("source", "unknown")
                }
                for hit in search_result.points
                if hit.payload
            ]
            return documents
        except Exception as e2:
            print(f"Error during fallback Qdrant search: {e2}")
            return []
