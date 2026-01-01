from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import logging
import hashlib

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

try:
    from .utils.cache import query_cache
    logger.info("Successfully imported query cache")
except Exception as e:
    logger.error(f"Error importing query cache: {e}")
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
    response = Response(
        content="OK",
        headers={
            "Access-Control-Allow-Origin": "https://physical-ai-humanoid-robotics-essen-opal.vercel.app",
            "Access-Control-Allow-Methods": "POST, OPTIONS",
            "Access-Control-Allow-Headers": "Content-Type, Authorization",
        }
    )
    return response

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

# Temporary GET endpoint for debugging - this should not be called by the frontend
@app.get("/query")
def query_debug():
    from fastapi.responses import JSONResponse
    return JSONResponse(
        status_code=405,
        content={
            "detail": "GET method not allowed. The frontend should make a POST request to this endpoint.",
            "error": "Make sure your frontend is sending a POST request with JSON body"
        }
    )

@app.post("/query", response_model=QueryResponse)
def query_chatbot(request: QueryRequest):
    try:
        if not request.question:
            raise HTTPException(status_code=400, detail="Question cannot be empty.")

        # Create a cache key based on the question and selected text
        query_text = request.question
        if request.selected_text:
            query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

        # Create a hash of the query text to use as cache key
        cache_key = hashlib.md5(query_text.encode()).hexdigest()

        # Check if result is already in cache
        cached_result = query_cache.get(cache_key)
        if cached_result:
            logger.info("Returning cached result for query")
            return QueryResponse(
                answer=cached_result["answer"],
                detailed_answer=cached_result["detailed_answer"],
                source_references=cached_result["source_references"]
            )

        # Check if required services are available
        if cohere_client is None:
            raise HTTPException(status_code=503, detail="Cohere service is not available. Check if COHERE_API_KEY is set.")

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
            # Check if it's a rate limit error
            error_str = str(embed_error)
            if "429" in error_str or "rate limit" in error_str.lower() or "Too Many Requests" in error_str:
                raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")
            else:
                raise HTTPException(status_code=500, detail=f"Embedding generation failed: {embed_error}")

        # Perform similarity search in Qdrant
        retrieved_chunks = search_qdrant(query_embedding)

        context_chunks = [chunk["text"] for chunk in retrieved_chunks]
        source_references = [chunk["source"] for chunk in retrieved_chunks]

        # Generate answer using LLM agent
        llm_response = generate_rag_response(request.question, context_chunks, source_references)

        response = QueryResponse(
            answer=llm_response["answer"],
            detailed_answer=llm_response["detailed_answer"],
            source_references=llm_response["sources"]
        )

        # Cache the result for future requests
        query_cache.set(cache_key, {
            "answer": llm_response["answer"],
            "detailed_answer": llm_response["detailed_answer"],
            "source_references": llm_response["sources"]
        })

        return response
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Unexpected error in query_chatbot: {e}")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")
