import os
from dotenv import load_dotenv

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Initialize clients only if required environment variables are present
cohere_client = None
qdrant_client = None

if COHERE_API_KEY:
    from cohere import Client as CohereClient
    cohere_client = CohereClient(api_key=COHERE_API_KEY)
else:
    print("Warning: COHERE_API_KEY not found. Cohere functionality will be disabled.")

if QDRANT_URL and QDRANT_API_KEY:
    from qdrant_client import QdrantClient
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )
else:
    print("Warning: QDRANT_URL or QDRANT_API_KEY not found. Qdrant functionality will be disabled.")

COLLECTION_NAME = "humanoid-robotics-book"
