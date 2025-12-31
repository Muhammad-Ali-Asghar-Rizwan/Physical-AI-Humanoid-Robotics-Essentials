from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import modules with error handling to identify any import issues
try:
    from .schemas import QueryRequest, QueryResponse
    logger.info("Successfully imported QueryRequest and QueryResponse schemas")
except Exception as e:
    logger.error(f"Error importing schemas: {e}")
    raise e

try:
    from .config import cohere_client
    logger.info("Successfully imported cohere_client")
except Exception as e:
    logger.error(f"Error importing cohere_client: {e}")
    raise e

try:
    from .rag.retriever import search_qdrant
    logger.info("Successfully imported search_qdrant")
except Exception as e:
    logger.error(f"Error importing search_qdrant: {e}")
    raise e

try:
    from .rag.agent import generate_rag_response
    logger.info("Successfully imported generate_rag_response")
except Exception as e:
    logger.error(f"Error importing generate_rag_response: {e}")
    raise e

app = FastAPI()

# Add startup event to log when the application starts
@app.on_event("startup")
async def startup_event():
    logger.info("Application startup complete. All routes and dependencies loaded successfully.")

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

        # Check if required services are available
        if cohere_client is None:
            raise HTTPException(status_code=503, detail="Cohere service is not available. Check if COHERE_API_KEY is set.")

        query_text = request.question
        if request.selected_text:
            query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

        # Generate embedding for the query
        try:
            query_embedding_response = cohere_client.embed(
                texts=[query_text],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = query_embedding_response.embeddings[0]
        except Exception as embed_error:
            logger.error(f"Embedding generation failed: {embed_error}")
            raise HTTPException(status_code=500, detail=f"Embedding generation failed: {embed_error}")

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
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Unexpected error in query_chatbot: {e}")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")
