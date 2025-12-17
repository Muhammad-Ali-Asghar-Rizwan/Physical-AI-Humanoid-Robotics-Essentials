# Quickstart Guide: RAG Chatbot Integration

## Overview

This guide provides a quick setup for the RAG Chatbot Integration feature. It assumes you have the Physical AI & Humanoid Robotics textbook content in the Docusaurus project and want to add the chatbot functionality.

## Prerequisites

- Python 3.10+ installed
- Node.js 16+ installed (for Docusaurus frontend)
- Git installed
- Access to the following APIs:
  - OpenAI API key (for LLM generation)
  - Cohere API key (for embeddings)
  - Qdrant Cloud (for vector storage)
  - Neon Postgres (for chat history)
- Vercel account for backend deployment
- GitHub account for frontend deployment

## Environment Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up environment variables**:
   Create a `.env` file in the project root with the following variables:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_cluster_url
   DATABASE_URL=your_neon_postgres_connection_string
   ```

3. **Install backend dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

4. **Install frontend dependencies**:
   ```bash
   cd frontend  # or the root if Docusaurus is in the root
   npm install
   ```

## Backend Setup

1. **Start the FastAPI development server**:
   ```bash
   cd backend
   uvicorn src.api.main:app --reload
   ```
   The backend API will be available at `http://localhost:8000`

2. **Run database migrations** (if applicable):
   ```bash
   python -m src.db.migrations
   ```

## Frontend Setup

1. **Start the Docusaurus development server**:
   ```bash
   cd path/to/docusaurus
   npm start
   ```
   The frontend will be available at `http://localhost:3000`

## Initializing the Knowledge Base

1. **Extract and chunk textbook content**:
   ```bash
   cd backend
   python -m src.scripts.chunk_textbook --source path/to/textbook/mdx/files --output-dir data/chunks
   ```

2. **Generate embeddings for all text chunks**:
   ```bash
   cd backend
   python -m src.scripts.generate_embeddings --chunks-dir data/chunks --qdrant-config config/qdrant.json
   ```

3. **Verify embeddings are stored in Qdrant**:
   - Check your Qdrant dashboard to confirm vectors are stored
   - Test similarity search with sample queries

## Testing the Integration

1. **Test the backend API endpoints**:
   - `POST /query`: Test with a sample query
     ```bash
     curl -X POST http://localhost:8000/query \
       -H "Content-Type: application/json" \
       -d '{"query": "What is the main principle of humanoid robotics?", "selected_text": ""}'
     ```
   - `GET /history`: Test retrieving chat history
   - `POST /feedback`: Test providing feedback on responses

2. **Verify chatbot widget functionality**:
   - Open the frontend in a browser
   - Use the chatbot widget to ask questions about the textbook content
   - Verify that responses include proper citations
   - Test text selection functionality

## Deployment

### Backend (to Vercel)

1. **Prepare for Vercel deployment**:
   - Ensure all environment variables are configured in Vercel dashboard
   - Check that the project follows Vercel's Python deployment requirements

2. **Deploy to Vercel**:
   ```bash
   vercel --prod
   ```

### Frontend (to GitHub Pages via Docusaurus)

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Deploy to GitHub Pages** according to your Docusaurus configuration

## Troubleshooting

1. **API responses taking too long**:
   - Check your API keys and rate limits for Cohere, OpenAI, and Qdrant
   - Verify your network connection to these services

2. **Embeddings not found for queries**:
   - Verify that the textbook content was properly chunked and embedded
   - Check that Qdrant contains the expected vectors
   - Confirm your query is formatted correctly

3. **Frontend not communicating with backend**:
   - Verify CORS settings in the backend
   - Check that the backend is accessible from the frontend domain
   - Ensure API endpoints are correctly configured in the frontend

## Next Steps

1. Customize the chatbot UI to match your design preferences
2. Fine-tune the prompt engineering for better response quality
3. Implement additional analytics and monitoring
4. Add support for additional textbook content formats