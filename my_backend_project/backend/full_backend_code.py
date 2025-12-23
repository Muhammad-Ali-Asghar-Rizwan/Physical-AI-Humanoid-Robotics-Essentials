# File: my_backend_project/backend/config.py
import os
from dotenv import load_dotenv
from cohere import Client as CohereClient
from qdrant_client import QdrantClient # Use QdrantClient for sync operations

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

cohere_client = CohereClient(api_key=COHERE_API_KEY)

# Use QdrantClient for sync operations
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

COLLECTION_NAME = "humanoid-robotics-book"

# =================================================================================================
# File: my_backend_project/backend/schemas.py
from pydantic import BaseModel
from typing import Optional, List

class QueryRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    detailed_answer: Optional[str] = None
    source_references: Optional[List[str]] = None

# =================================================================================================
# File: my_backend_project/backend/rag/loader.py
from typing import List
import os
import re
from markdown_it import MarkdownIt

class Document:
    def __init__(self, text: str, metadata: dict):
        self.text = text
        self.metadata = metadata

def _strip_markdown(content: str) -> str:
    md = MarkdownIt()
    html = md.render(content)
    # Simple regex to strip HTML tags for now, can be improved
    text = re.sub(r'<[^>]+>', '', html)
    # Further cleanup: remove excessive newlines, decode HTML entities
    text = re.sub(r'\n\s*\n', '\n', text).strip()
    return text

def load_and_clean_documents(docs_path: str) -> List[Document]:
    documents = []
    for root, _, files in os.walk(docs_path):
        for file_name in files:
            if file_name.endswith((".md", ".mdx")):
                file_path = os.path.join(root, file_name)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                
                cleaned_text = _strip_markdown(content)
                metadata = {
                    "source": file_path,
                    "filename": file_name,
                    # Add chapter/section parsing logic here if needed
                }
                documents.append(Document(text=cleaned_text, metadata=metadata))
    return documents

# =================================================================================================
# File: my_backend_project/backend/rag/chunker.py
from typing import List
#from .loader import Document # This is commented out because Document is defined in the same file
import re

def chunk_documents(documents: List[Document], chunk_size: int = 800) -> List[Document]:
    chunks = []
    for doc in documents:
        # Split by sentences, using Python's re module
        sentences = re.split(r'(?<=[.?!])\s+', doc.text)
        
        current_chunk_text = ''
        for sentence in sentences:
            if len(current_chunk_text) + len(sentence) > chunk_size:
                chunks.append(Document(text=current_chunk_text.strip(), metadata=doc.metadata))
                current_chunk_text = ''
            current_chunk_text += sentence + ' '
        
        if current_chunk_text:
            chunks.append(Document(text=current_chunk_text.strip(), metadata=doc.metadata))
    return chunks

# =================================================================================================
# File: my_backend_project/backend/rag/embeddings.py
from typing import List
# from .loader import Document # This is commented out because Document is defined in the same file
# from ..config import cohere_client # This is commented out because cohere_client is defined in the same file

def generate_embeddings(documents: List[Document]) -> List[List[float]]:
    texts = [doc.text for doc in documents]
    
    # Cohere has a limit of 96 texts per request
    batch_size = 90
    all_embeddings = []
    for i in range(0, len(texts), batch_size):
        batch_texts = texts[i:i + batch_size]
        response = cohere_client.embed(
            texts=batch_texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        all_embeddings.extend(response.embeddings)
    return all_embeddings

# =================================================================================================
# File: my_backend_project/backend/rag/retriever.py
from typing import List, Dict, Any
from qdrant_client.http.models import Distance, VectorParams, PointStruct
# from ..config import qdrant_client, COLLECTION_NAME # This is commented out because qdrant_client and COLLECTION_NAME are defined in the same file
# from .loader import Document # This is commented out because Document is defined in the same file

def recreate_qdrant_collection(vector_size: int = 1024):
    """Recreates the Qdrant collection."""
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
    )
    print(f"Collection '{COLLECTION_NAME}' recreated with vector size {vector_size}.")

def upsert_documents_to_qdrant(documents: List[Document], embeddings: List[List[float]]):
    """Upserts documents and their embeddings to Qdrant."""
    points = []
    for i, doc in enumerate(documents):
        points.append(
            PointStruct(
                id=i,
                vector=embeddings[i],
                payload={"text": doc.text, **doc.metadata},
            )
        )
    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        wait=True,
        points=points,
    )
    print(f"Upserted {len(documents)} documents to '{COLLECTION_NAME}'.")

def search_qdrant(query_vector: List[float], limit: int = 5) -> List[Dict[str, Any]]:
    """Performs a similarity search in Qdrant."""
    try:
        search_result = qdrant_client.search_points(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=limit,
            with_payload=True,  # Ensure payload is returned
        )
        return [
            {"text": point.payload["text"], "source": point.payload.get("source", "unknown")}
            for point in search_result.points
        ]
    except Exception as e:
        print(f"Error during Qdrant search: {e}")
        return []

# =================================================================================================
# File: my_backend_project/backend/rag/agent.py
import os
import google.generativeai as genai
from dotenv import load_dotenv
from typing import List, Dict, Any

load_dotenv()

# Initialize Gemini client, but handle potential missing API key
try:
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        print("Warning: GEMINI_API_KEY not found. LLM will be disabled.")
        genai.configure(api_key="DUMMY_KEY_FOR_INITIALIZATION") # Configure with a dummy key
        model_rag = None
    else:
        genai.configure(api_key=api_key)
        model_rag = genai.GenerativeModel('models/gemini-1.5-flash')
except Exception as e:
    print(f"Error initializing Gemini: {e}. LLM will be disabled.")
    model_rag = None

# Refusal messages
REFUSAL_EN = "I'm sorry, I can only answer questions related to the textbook content."
REFUSAL_UR = "Maaf kijiye, mein sirf textbook ke content se related sawalon ka jawab de sakta hoon."

def _format_rag_only_response(context_chunks: List[str], source_references: List[str]) -> Dict[str, Any]:
    """Formats the RAG context into a user-friendly response when the LLM fails."""
    if not context_chunks:
        return {
            "answer": "No relevant context found in the book.",
            "detailed_answer": "No relevant context was found to answer your question. Please try rephrasing or selecting different text.",
            "sources": []
        }
        
    # Combine and format the retrieved chunks as the main answer
    formatted_answer = "Here is the most relevant information found in the textbook:\n\n" + "\n\n---\n\n".join(context_chunks)
    
    unique_sources = sorted(list(set(source_references)))

    return {
        "answer": "Could not generate a summary, but here is the relevant context from the book.",
        "detailed_answer": formatted_answer,
        "sources": unique_sources
    }

def detect_language(text: str) -> str:
    """Detects if the text is English or Roman Urdu using the Gemini model. Falls back to English on error."""
    if not model_rag:
        return "en"
    try:
        prompt = f"Identify the language of the following text: '{text}'. Respond with 'en' for English or 'ur' for Roman Urdu."
        # Note: Using generate_content for sync context
        response = model_rag.generate_content(prompt)
        lang_code = response.text.strip().lower()
        return "ur" if "ur" in lang_code else "en"
    except Exception as e:
        print(f"Error detecting language, defaulting to 'en': {e}")
        return "en"

def check_relevance(question: str) -> bool:
    """Checks if the question is relevant. Defaults to True on error to avoid blocking valid questions."""
    if not model_rag:
        return True
    try:
        prompt = f"Is the following question relevant to a 'Humanoid Robotics' textbook? Answer with 'yes' or 'no'. Question: '{question}'"
        # Note: Using generate_content for sync context
        response = model_rag.generate_content(prompt)
        return "yes" in response.text.strip().lower()
    except Exception as e:
        print(f"Error checking relevance, defaulting to 'True': {e}")
        return True

def generate_rag_response(question: str, context_chunks: List[str], source_references: List[str]) -> Dict[str, Any]:
    """
    Generates a response using RAG, with an optional LLM fallback.
    Ensures that any failure in LLM communication is caught and handled gracefully.
    """
    if not context_chunks:
        # If no context is found from RAG, return a "not found" message immediately.
        return {
            "answer": "Answer not found in the book. Please try rephrasing your question.",
            "detailed_answer": "No relevant content was found in the textbook to answer your question.",
            "sources": []
        }

    # If the LLM is disabled via configuration, go directly to the RAG-only response.
    if not model_rag:
        print("LLM disabled. Falling back to RAG-only response.")
        return _format_rag_only_response(context_chunks, source_references)

    try:
        # --- Start of fully isolated LLM logic ---
        # All calls that depend on the LLM are now inside this single try block.

        lang = detect_language(question)
        is_relevant = check_relevance(question)

        if not is_relevant:
            refusal_message = REFUSAL_EN if lang == "en" else REFUSAL_UR
            return {"answer": refusal_message, "detailed_answer": refusal_message, "sources": []}

        # Format the prompt for the LLM
        formatted_context = "\n\n".join(context_chunks)
        system_message = (
            "You are a helpful assistant for a Humanoid Robotics textbook. Answer questions strictly from the provided context. "
            "If the answer is not in the context, say 'Answer not found in the book.' "
            "Provide a concise summary first, then a detailed answer with citations. "
            f"Respond in {'Roman Urdu' if lang == 'ur' else 'English'}."
        )
        user_message = f"Context from the book:\n{formatted_context}\n\nQuestion: {question}"
        full_prompt = f"{system_message}\n\n{user_message}"

        # Generate the response from the LLM
        response = model_rag.generate_content(
            full_prompt,
            generation_config={"temperature": 0.2}
        )
        llm_full_response = response.text.strip()
        
        # Process and return the successful LLM response
        concise_answer = llm_full_response.split('\n\n')[0]
        detailed_answer = llm_full_response
        unique_sources = sorted(list(set(source_references)))

        return {
            "answer": concise_answer,
            "detailed_answer": detailed_answer,
            "sources": unique_sources
        }
        # --- End of fully isolated LLM logic ---
        
    except Exception as e:
        # If any part of the LLM logic fails (network, API, etc.), catch it.
        print(f"Error generating LLM response: {e}. Falling back to RAG-only response.")
        # Fallback to returning the raw context if the LLM fails for any reason.
        return _format_rag_only_response(context_chunks, source_references)

# =================================================================================================
# File: my_backend_project/backend/main.py
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
# from .schemas import QueryRequest, QueryResponse # This is commented out because QueryRequest and QueryResponse are defined in the same file
# from .config import cohere_client # This is commented out because cohere_client is defined in the same file
# from .rag.retriever import search_qdrant # This is commented out because search_qdrant is defined in the same file
# from .rag.agent import generate_rag_response # This is commented out because generate_rag_response is defined in the same file

app = FastAPI()

# Add CORS middleware to allow requests from the Docusaurus frontend
origins = [
    "https://humanoid-robotics-book-lovat.vercel.app/", # Docusaurus development server
    # Add your Vercel deployment URL here in production
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
