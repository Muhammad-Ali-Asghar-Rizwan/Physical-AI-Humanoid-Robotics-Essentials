import os
import sys

# Add the root of my_backend_project to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from backend.rag.loader import load_and_clean_documents
from backend.rag.chunker import chunk_documents
from backend.rag.embeddings import generate_embeddings
from backend.rag.retriever import recreate_qdrant_collection, upsert_documents_to_qdrant

def main():
    print("Starting book ingestion process...")

    docs_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'backend'))
    print(f"Loading documents from: {docs_path}")

    # Load and clean
    documents = load_and_clean_documents(docs_path)
    print(f"Loaded {len(documents)} raw documents.")

    # Chunk
    chunks = chunk_documents(documents)
    print(f"Created {len(chunks)} chunks.")

    # Generate Embeddings
    embeddings = generate_embeddings(chunks)
    print(f"Generated {len(embeddings)} embeddings.")

    # Qdrant Setup
    recreate_qdrant_collection()

    # Store Vectors
    upsert_documents_to_qdrant(chunks, embeddings)
    print("Book ingestion complete.")

if __name__ == "__main__":
    main()
