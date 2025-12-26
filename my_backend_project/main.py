import sys
import os
# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse

# Import using absolute paths
import backend.schemas as schemas
import backend.config as config
import backend.rag.retriever as retriever
import backend.rag.agent as agent

# Access the imported objects
QueryRequest = schemas.QueryRequest
QueryResponse = schemas.QueryResponse
cohere_client = config.cohere_client
search_qdrant = retriever.search_qdrant
generate_rag_response = agent.generate_rag_response

import json
import time

app = FastAPI()

# Add CORS middleware to allow requests from the Docusaurus frontend
origins = [
    "http://localhost:3000",  # Docusaurus local development server
    "https://humanoid-robotics-book-lovat.vercel.app",  # Vercel deployment URL
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

@app.post("/query", response_model=schemas.QueryResponse)
def query_chatbot(request: schemas.QueryRequest):
    try:
        if not request.question:
            raise HTTPException(status_code=400, detail="Question cannot be empty.")

        query_text = request.question
        if request.selected_text:
            query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

        # Generate embedding for the query
        query_embedding_response = config.cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = query_embedding_response.embeddings[0]

        # Perform similarity search in Qdrant
        retrieved_chunks = retriever.search_qdrant(query_embedding)

        context_chunks = [chunk["text"] for chunk in retrieved_chunks]
        source_references = [chunk["source"] for chunk in retrieved_chunks]

        # Generate answer using LLM agent
        llm_response = agent.generate_rag_response(request.question, context_chunks, source_references)

        return schemas.QueryResponse(
            answer=llm_response["answer"],
            detailed_answer=llm_response["detailed_answer"],
            source_references=llm_response["sources"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")

@app.post("/stream-query")
def stream_query_chatbot(request: schemas.QueryRequest):
    def event_generator():
        try:
            if not request.question:
                yield f"event: error\n"
                yield f"data: {json.dumps({'detail': 'Question cannot be empty.'})}\n\n"
                return

            # Send thinking event first
            yield f"event: thinking\n"
            yield f"data: {json.dumps({'status': 'Processing your question...'})}\n\n"

            query_text = request.question
            if request.selected_text:
                query_text = f"{request.selected_text}\n\nQuestion: {request.question}"

            # Generate embedding for the query
            query_embedding_response = config.cohere_client.embed(
                texts=[query_text],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = query_embedding_response.embeddings[0]

            # Perform similarity search in Qdrant
            retrieved_chunks = retriever.search_qdrant(query_embedding)

            context_chunks = [chunk["text"] for chunk in retrieved_chunks]
            source_references = [chunk["source"] for chunk in retrieved_chunks]

            # Generate answer using LLM agent
            llm_response = agent.generate_rag_response(request.question, context_chunks, source_references)

            # Send the final response
            response_data = {
                'answer': llm_response['answer'],
                'detailed_answer': llm_response['detailed_answer'],
                'source_references': llm_response['sources']
            }
            yield f"event: response\n"
            yield f"data: {json.dumps(response_data)}\n\n"
        except Exception as e:
            yield f"event: error\n"
            yield f"data: {json.dumps({'detail': f'Internal Server Error: {e}'})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 8000)))