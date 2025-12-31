from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from .schemas import QueryRequest, QueryResponse
from .config import cohere_client
from .rag.retriever import search_qdrant
from .rag.agent import generate_rag_response
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# Add CORS middleware to allow requests from the Docusaurus frontend
origins = [
    "http://localhost:3000",  # Docusaurus local development server
    "https://humanoid-robotics-book-lovat.vercel.app",  # Old Vercel deployment URL
    "https://physical-ai-humanoid-robotics-essen-opal.vercel.app",  # Current Vercel deployment URL
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],  # Be more specific about allowed methods
    allow_headers=["*"],
    # Allow credentials to be included in cross-origin requests
    allow_origin_regex=None,  # We're specifying origins explicitly
    # Expose headers that the frontend might need to access
    expose_headers=["Access-Control-Allow-Origin", "Access-Control-Allow-Credentials"]
)

# Add custom middleware to log requests
@app.middleware("http")
async def log_requests(request, call_next):
    logger.info(f"Request: {request.method} {request.url}")
    logger.info(f"Headers: {request.headers}")

    response = await call_next(request)
    return response

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/health")
def read_health():
    return {"status": "ok"}

# Handle preflight requests for the query endpoint
@app.options("/query")
def query_options():
    from fastapi.responses import Response
    return Response(
        headers={
            "Access-Control-Allow-Origin": "https://physical-ai-humanoid-robotics-essen-opal.vercel.app",
            "Access-Control-Allow-Methods": "POST, OPTIONS",
            "Access-Control-Allow-Headers": "Content-Type, Authorization",
        }
    )

# Handle preflight requests for the root endpoint
@app.options("/")
def root_options():
    from fastapi.responses import Response
    return Response(
        headers={
            "Access-Control-Allow-Origin": "https://physical-ai-humanoid-robotics-essen-opal.vercel.app",
            "Access-Control-Allow-Methods": "GET, OPTIONS",
            "Access-Control-Allow-Headers": "Content-Type",
        }
    )

# Handle preflight requests for the health endpoint
@app.options("/health")
def health_options():
    from fastapi.responses import Response
    return Response(
        headers={
            "Access-Control-Allow-Origin": "https://physical-ai-humanoid-robotics-essen-opal.vercel.app",
            "Access-Control-Allow-Methods": "GET, OPTIONS",
            "Access-Control-Allow-Headers": "Content-Type",
        }
    )

@app.post("/query", response_model=QueryResponse)
def query_chatbot(request: QueryRequest):
    try:
        if not request.question:
            raise HTTPException(status_code=400, detail="Question cannot be empty.")

        query_text = request.question
        if request.selected_text:
            query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

        # Generate embedding for the query
        query_embedding_response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = query_embedding_response.embeddings[0]

        # Perform similarity search in Qdrant
        retrieved_chunks = search_qdrant(query_embedding)
        
        context_chunks = [chunk["text"] for chunk in retrieved_chunks]
        source_references = [chunk["source"] for chunk in retrieved_chunks]

        # Generate answer using LLM agent
        llm_response = generate_rag_response(request.question, context_chunks, source_references)

        return QueryResponse(
            answer=llm_response["answer"],
            detailed_answer=llm_response["detailed_answer"],
            source_references=llm_response["sources"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")
