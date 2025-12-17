# RAG Chatbot for Physical AI & Humanoid Robotics Textbook

This project implements a Retrieval-Augmented Generation (RAG) chatbot that allows students to query the Physical AI & Humanoid Robotics textbook content.

## Architecture

The project is divided into two main components:

### Backend (Railway Deployable)
- Built with FastAPI in Python
- Handles RAG pipeline, embeddings, and vector storage
- Connects to Qdrant Cloud for vector search
- Stores chat history in Neon Postgres
- Uses OpenAI and Cohere APIs for generation and embeddings

### Frontend (Docusaurus Integration)
- React-based chat widget
- Embeds directly into Docusaurus textbook pages
- Supports text selection for focused Q&A
- Stores chat history in browser local storage

## Folder Structure

```
my_project/
├─ backend/                # Railway deployable backend
│  ├─ src/
│  │  ├─ api/
│  │  │  ├─ main.py        # FastAPI entrypoint
│  │  │  └─ endpoints/     # API endpoints
│  │  ├─ models/           # Pydantic models
│  │  ├─ services/         # Business logic
│  │  ├─ utils/            # Utility functions
│  │  └─ scripts/          # Utility scripts
│  ├─ requirements.txt     # Python dependencies
├─ frontend/               # Docusaurus integration files
│  ├─ src/
│  │  ├─ components/      # React components
│  │  └─ services/        # Frontend services
│  └─ package.json         # JavaScript dependencies
├─ .env                    # DO NOT TOUCH
└─ README.md
```

## Setup

### Backend Setup

1. Install Python dependencies:
```bash
cd backend
pip install -r requirements.txt
```

2. Set up environment variables in `.env` file:
```
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
DATABASE_URL=your_neon_postgres_connection_string
```

3. Run the backend server:
```bash
cd backend
uvicorn src.api.main:app --reload
```

### Textbook Ingestion

Before the chatbot can answer questions, you need to ingest the textbook content:

```bash
cd backend
python -m src.scripts.chunk_textbook --source /path/to/textbook/mdx/files
```

This will:
- Read all MD/MDX files from the specified directory
- Chunk the content (500-1000 tokens per chunk)
- Generate embeddings using Cohere
- Store embeddings in Qdrant with metadata

### Frontend Setup

1. Install JavaScript dependencies:
```bash
cd frontend
npm install
```

2. Run the frontend:
```bash
npm start
```

## API Endpoints

The backend exposes the following endpoints:

- `POST /api/query` - Process user queries and return responses
- `GET /api/history` - Retrieve chat history for a session
- `POST /api/feedback` - Submit feedback on a response
- `GET /health` - Health check endpoint

## Features

- **Textbook Querying**: Ask questions about the Physical AI & Humanoid Robotics textbook
- **Text Selection**: Select text in the document and ask specific questions about it
- **Source Citations**: Responses include references to textbook chapters/sections
- **Chat History**: Conversation history stored in browser local storage
- **Anonymous Usage**: No login required for basic functionality
- **Feedback System**: Rate responses to help improve the system

## Integration with Docusaurus

To integrate the chatbot with your Docusaurus site:

1. Copy the `ChatbotWidget.jsx` and `ChatbotWidget.css` files to your Docusaurus project's `src/components` directory.
2. Use the component in your pages or layouts:
```jsx
import ChatbotWidget from '@site/src/components/ChatbotWidget';

// Use it in your layout or MDX pages
<ChatbotWidget />
```

3. See `frontend/src/components/integration-guide.md` for detailed instructions.

## Deployment

### Backend (to Railway)

1. Create a new Railway project
2. Connect to your backend repository
3. Add environment variables in Railway dashboard
4. Deploy the project

### Frontend (to Vercel/GitHub Pages)

1. Build the frontend: `npm run build`
2. Deploy to your preferred static hosting provider
3. Update the API base URL in the frontend to point to your Railway backend

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License.