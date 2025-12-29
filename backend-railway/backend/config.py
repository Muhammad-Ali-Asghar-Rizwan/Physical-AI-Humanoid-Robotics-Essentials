import os
from dotenv import load_dotenv
from cohere import Client as CohereClient
from qdrant_client import QdrantClient # Use QdrantClient for sync operations

# Load .env only for local development
# Railway will inject env vars via UI, so this is safe to call
try:
    load_dotenv()
except:
    pass

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Initialize clients only if keys are present (fail gracefully for debugging)
if not COHERE_API_KEY:
    print("WARNING: COHERE_API_KEY not set")
if not QDRANT_URL or not QDRANT_API_KEY:
    print("WARNING: QDRANT_URL or QDRANT_API_KEY not set")

cohere_client = CohereClient(api_key=COHERE_API_KEY) if COHERE_API_KEY else None

# Use AsyncQdrantClient for async operations
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
) if QDRANT_URL and QDRANT_API_KEY else None

COLLECTION_NAME = "humanoid-robotics-book"
