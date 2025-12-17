import os
from dotenv import load_dotenv
from cohere import AsyncClient as CohereClient
from qdrant_client import QdrantClient

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

cohere_client = CohereClient(api_key=COHERE_API_KEY)

qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    prefer_grpc=False,
)

COLLECTION_NAME = "humanoid-robotics-book"
