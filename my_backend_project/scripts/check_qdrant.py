import asyncio
import os
import sys
from dotenv import load_dotenv
from qdrant_client import AsyncQdrantClient

# Add parent dir to path to import config if needed, but we can just use env vars directly for this check
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "humanoid-robotics-book"

async def check_qdrant():
    try:
        client = AsyncQdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False,
        )

        # Check if collection exists
        collections = await client.get_collections()
        exists = any(c.name == COLLECTION_NAME for c in collections.collections)

        if not exists:
            print(f"Collection '{COLLECTION_NAME}' does NOT exist.")
            return False

        # Check count
        count = (await client.count(collection_name=COLLECTION_NAME)).count
        print(f"Collection '{COLLECTION_NAME}' has {count} documents.")

        if count == 0:
            print("Collection is empty.")
            return False

        print("Qdrant seems populated.")
        return True

    except Exception as e:
        print(f"Error connecting to Qdrant: {e}")
        return False

if __name__ == "__main__":
    asyncio.run(check_qdrant())
