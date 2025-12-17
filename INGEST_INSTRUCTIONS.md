# Document Ingestion Guide

Your chatbot is working! But the database is empty. You need to ingest your textbook documents.

## Quick Start

### Option 1: Run the Ingest Script (Recommended)

```bash
cd c:\Users\user\Desktop\AI Hackathon\my_backend_project

# Make sure your backend environment is set up
python -m scripts.ingest_book
```

This will:
1. ✅ Load all markdown files from `physical-ai-humanoid-robotics-textbook/docs`
2. ✅ Split them into chunks
3. ✅ Generate embeddings using Cohere
4. ✅ Store vectors in Qdrant
5. ✅ Make them searchable in the chatbot

### What You'll See

```
Starting book ingestion process...
Loading documents from: c:\Users\user\Desktop\AI Hackathon\physical-ai-humanoid-robotics-textbook\docs
Loaded X raw documents.
Created Y chunks.
Generating embeddings... (this may take a while)
Generated Z embeddings.
Creating Qdrant collection...
Upserting documents to Qdrant...
✅ Book ingestion complete!
```

## Troubleshooting

### Error: "Path not found"
- Make sure the textbook docs folder exists at the correct path
- Verify: `c:\Users\user\Desktop\AI Hackathon\physical-ai-humanoid-robotics-textbook\docs`

### Error: "No documents found"
- Check that you have `.md` or `.mdx` files in the docs folder
- The loader only picks up markdown files

### Error: API Key issues
- Make sure `.env` file has valid:
  - `COHERE_API_KEY` (for embeddings)
  - `QDRANT_URL` (for vector database)
  - `QDRANT_API_KEY` (for vector database auth)

## After Ingestion

Once the script completes:
1. Stop your backend
2. Restart the backend server
3. Go to your chatbot and ask questions about the textbook
4. You should now get relevant answers! 🚀

## How It Works

1. **Documents** → Split into chunks (500 tokens each)
2. **Embeddings** → Generated using Cohere embed-english-v3.0
3. **Vector DB** → Stored in Qdrant cloud
4. **Search** → When you ask a question, it finds similar chunks
5. **Answer** → Gemini AI generates an answer based on those chunks

---

**Need help?** Check your `.env` file and make sure all API keys are valid!
