from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import sys
import os
import asyncio
from concurrent.futures import ThreadPoolExecutor

# Add the backend directory to the path to enable imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Support running as a package (relative imports) or as a top-level module
try:
    # When running as part of a package
    from .schemas import QueryRequest, QueryResponse
    from .config import cohere_client
    from .rag.retriever import search_qdrant
    from .rag.agent import generate_rag_response
except ImportError:
    # When running directly as a script
    try:
        from backend.schemas import QueryRequest, QueryResponse
        from backend.config import cohere_client
        from backend.rag.retriever import search_qdrant
        from backend.rag.agent import generate_rag_response
    except ImportError:
        # If both ways fail, import directly from current directory
        from schemas import QueryRequest, QueryResponse
        from config import cohere_client
        from rag.retriever import search_qdrant
        from rag.agent import generate_rag_response

app = FastAPI()

# Thread pool for sync operations
executor = ThreadPoolExecutor(max_workers=4)

# Add CORS middleware to allow requests from the Docusaurus frontend
origins = [
    "http://localhost:3000", # Docusaurus development server
    "http://127.0.0.1:8000", # Backend local
    "*" # Allow all for development
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/health")
def read_health():
    return {"status": "ok"}

@app.post("/query", response_model=QueryResponse)
async def query_chatbot(request: QueryRequest):
    try:
        print(f"Received question: {request.question}")

        if not request.question:
            raise HTTPException(status_code=400, detail="Question cannot be empty.")

        query_text = request.question
        if request.selected_text:
            query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

        print(f"Generating embedding...")
        query_embedding_response = await asyncio.wait_for(
            cohere_client.embed(
                texts=[query_text],
                model="embed-english-v3.0",
                input_type="search_query"
            ),
            timeout=8.0
        )
        query_embedding = query_embedding_response.embeddings[0]
        print(f"Embedding generated, shape: {len(query_embedding)}")

        print(f"Searching Qdrant...")
        retrieved_chunks = await asyncio.wait_for(
            search_qdrant(query_embedding),
            timeout=5.0
        )
        print(f"Retrieved {len(retrieved_chunks)} chunks")

        context_chunks = [chunk["text"] for chunk in retrieved_chunks]
        source_references = [chunk["source"] for chunk in retrieved_chunks]

        print(f"Generating response...")
        llm_response = await asyncio.wait_for(
            generate_rag_response(request.question, context_chunks, source_references),
            timeout=12.0
        )

        print(f"Response generated: {llm_response['answer'][:100]}...")

        return QueryResponse(
            answer=llm_response["answer"],
            detailed_answer=llm_response["detailed_answer"],
            source_references=llm_response["sources"]
        )
    except Exception as e:
        print(f"Error in query_chatbot: {str(e)}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {str(e)}")



