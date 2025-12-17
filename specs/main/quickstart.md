# Quickstart Guide: RAG Chatbot for Physical AI Textbook

This guide will help you quickly set up and run the RAG chatbot system for the Physical AI textbook.

## Prerequisites

- Python 3.10+
- Node.js 16+ (for Docusaurus frontend)
- Access to OpenAI API (with GPT-4 and embedding-3-small models)
- Qdrant Cloud account (free tier sufficient)
- Neon Postgres account (free tier sufficient)

## Environment Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Set up the backend:
```bash
cd backend
pip install -r requirements.txt
```

3. Set up the frontend:
```bash
cd frontend  # or wherever Docusaurus is located
npm install
```

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```bash
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
API_KEY=your_api_key_for_authentication
```

## Initial Setup

1. Ingest the textbook content into Qdrant:
```bash
cd scripts
python ingest_docs.py
```

2. Start the backend server:
```bash
cd backend
python main.py
# or using uvicorn
uvicorn main:app --reload --port 8000
```

3. Build and serve the frontend:
```bash
cd frontend
npm run build
npm run serve
```

## API Usage

### Chat Endpoint
Send a message to the chatbot:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your_api_key" \
  -d '{
    "query": "What are the key principles of physical AI?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "selected_text": "Physical AI combines robotics and artificial intelligence..."
  }'
```

### Get Chat History
Retrieve conversation history:
```bash
curl -X GET http://localhost:8000/history/{session_id} \
  -H "X-API-Key: your_api_key"
```

### Health Check
Check API health:
```bash
curl -X GET http://localhost:8000/health
```

## Integration with Docusaurus

To add the chatbot to your Docusaurus site:

1. Add the React component to your Docusaurus layout:
```jsx
// Example in src/components/RAGChatbot/index.js
import RAGChatbot from './RAGChatbot';
// Use in your layout
<RAGChatbot />
```

2. Configure the API endpoint in the component:
```js
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

## Running Tests

Backend tests:
```bash
cd backend
pytest tests/
```

Frontend tests:
```bash
cd frontend
npm test
```

## Next Steps

- Configure authentication for production use
- Set up monitoring and logging
- Deploy to Vercel (backend) and GitHub Pages (frontend)
- Tune performance parameters based on usage