import asyncio
import os
import sys

# Add the root of my_backend_project to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from backend.rag.loader import load_and_clean_documents
from backend.rag.chunker import chunk_documents
from backend.rag.embeddings import generate_embeddings
from backend.rag.retriever import recreate_qdrant_collection, upsert_documents_to_qdrant

async def main():
    print("Starting book ingestion process...")

    # Path to the actual textbook docs
    docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'physical-ai-humanoid-robotics-textbook', 'docs'))
    print(f"Loading documents from: {docs_path}")

    if not os.path.exists(docs_path):
        print(f"ERROR: Path not found: {docs_path}")
        return

    # Load and clean
    documents = load_and_clean_documents(docs_path)
    print(f"Loaded {len(documents)} raw documents.")

    if len(documents) == 0:
        print("ERROR: No documents found! Check the path.")
        return

    # Chunk
    chunks = chunk_documents(documents)
    print(f"Created {len(chunks)} chunks.")

    # Generate Embeddings
    print("Generating embeddings... (this may take a while)")
    embeddings = await generate_embeddings(chunks)
    print(f"Generated {len(embeddings)} embeddings.")

    # Qdrant Setup
    print("Creating Qdrant collection...")
    await recreate_qdrant_collection()
    print("Collection created successfully!")

    # Store Vectors
    print("Upserting documents to Qdrant...")
    await upsert_documents_to_qdrant(chunks, embeddings)
    print("✅ Book ingestion complete!")

if __name__ == "__main__":
    asyncio.run(main())
