---
id: 2
title: rag-chatbot-backend-spec
stage: spec
date_iso: 2025-12-09
model: Qwen
feature: 002-rag-chatbot-backend
branch: 002-rag-chatbot-backend
user: user
command: /sp.specify Component: FastAPI RAG Chatbot Backend Objective: Build a production-ready FastAPI backend that implements RAG (Retrieval-Augmented Generation) for the Physical AI textbook, supporting both full-book queries and selected-text queries. Target Environment: - Python 3.10+ - Vercel Serverless Deployment - Free tier services (Qdrant, Neon, OpenAI) API Endpoints: 1. POST /chat Input: { "query": "What is ROS 2?", "session_id": "uuid-string" (optional), "selected_text": "text snippet" (optional) } Output: { "answer": "ROS 2 is...", "session_id": "uuid-string", "chat_id": 123, "context_source": "full_book" | "selected_text", "chunks_retrieved": 5, "sources": [{"module": "Module 1", "section": "Introduction"}] } 2. GET /history/{session_id} Output: { "session_id": "uuid", "history": [ {"user_message": "...", "bot_response": "...", "timestamp": "..."} ] } 3. GET /health Output: {"status": "healthy", "message": "API running"} RAG Implementation: - If selected_text provided: * Use selected text as primary context * Still retrieve 2-3 relevant chunks for additional context - If no selected_text: * Retrieve top 5 chunks from Qdrant * Rank by cosine similarity * Include module/section metadata Database Schema (Neon Postgres): Table: chat_history - id (SERIAL PRIMARY KEY) - session_id (VARCHAR(255)) - user_message (TEXT) - bot_response (TEXT) - context_used (TEXT) - selected_text (TEXT, nullable) - created_at (TIMESTAMP) Qdrant Collection: Name: physical_ai_book Vector Size: 1536 (OpenAI embedding dimension) Distance: Cosine Payload: - text (string): The actual chunk text - module (string): e.g., "Module 1" - section (string): e.g., "ROS 2 Basics" - page (string): File name - chunk_index (integer) LLM Prompt Template: System: "You are an expert assistant for the Physical AI & Humanoid Robotics textbook..." User: "Context: {retrieved_chunks}\n\nQuestion: {user_query}\n\nAnswer based on context:" Error Handling: - API key invalid → Return 401 - Qdrant connection failed → Fallback to GPT without RAG - Postgres connection failed → Continue but don't store history - Timeout (>30s) → Return error message Performance Requirements: - Response time: <3 seconds (p95) - Concurrent requests: Handle 10 simultaneous users - Rate limiting: 60 requests/minute per session Security: - CORS whitelist (GitHub Pages domain) - API key in environment variables - Input sanitization (prevent injection) - Rate limiting per IP Dependencies: - fastapi - uvicorn - openai - qdrant-client - psycopg2-binary - python-dotenv - pydantic Success Criteria: - All endpoints return correct responses - RAG retrieves relevant context (manual review) - Selected text feature works correctly - Chat history persists in Postgres - API deploys to Vercel successfully - Response time <3 seconds
labels: ["spec", "rag", "chatbot", "backend", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/002-rag-chatbot-backend/spec.md"
  - "specs/002-rag-chatbot-backend/checklists/requirements.md"
tests_yaml:
  - "specs/002-rag-chatbot-backend/spec.md"
---

# Prompt History Record: RAG Chatbot Backend Specification

## Original Prompt
```
/sp.specify Component: FastAPI RAG Chatbot Backend Objective: Build a production-ready FastAPI backend that implements RAG (Retrieval-Augmented Generation) for the Physical AI textbook, supporting both full-book queries and selected-text queries. Target Environment: - Python 3.10+ - Vercel Serverless Deployment - Free tier services (Qdrant, Neon, OpenAI) API Endpoints: 1. POST /chat Input: { "query": "What is ROS 2?", "session_id": "uuid-string" (optional), "selected_text": "text snippet" (optional) } Output: { "answer": "ROS 2 is...", "session_id": "uuid-string", "chat_id": 123, "context_source": "full_book" | "selected_text", "chunks_retrieved": 5, "sources": [{"module": "Module 1", "section": "Introduction"}] } 2. GET /history/{session_id} Output: { "session_id": "uuid", "history": [ {"user_message": "...", "bot_response": "...", "timestamp": "..."} ] } 3. GET /health Output: {"status": "healthy", "message": "API running"} RAG Implementation: - If selected_text provided: * Use selected text as primary context * Still retrieve 2-3 relevant chunks for additional context - If no selected_text: * Retrieve top 5 chunks from Qdrant * Rank by cosine similarity * Include module/section metadata Database Schema (Neon Postgres): Table: chat_history - id (SERIAL PRIMARY KEY) - session_id (VARCHAR(255)) - user_message (TEXT) - bot_response (TEXT) - context_used (TEXT) - selected_text (TEXT, nullable) - created_at (TIMESTAMP) Qdrant Collection: Name: physical_ai_book Vector Size: 1536 (OpenAI embedding dimension) Distance: Cosine Payload: - text (string): The actual chunk text - module (string): e.g., "Module 1" - section (string): e.g., "ROS 2 Basics" - page (string): File name - chunk_index (integer) LLM Prompt Template: System: "You are an expert assistant for the Physical AI & Humanoid Robotics textbook..." User: "Context: {retrieved_chunks}\n\nQuestion: {user_query}\n\nAnswer based on context:" Error Handling: - API key invalid → Return 401 - Qdrant connection failed → Fallback to GPT without RAG - Postgres connection failed → Continue but don't store history - Timeout (>30s) → Return error message Performance Requirements: - Response time: <3 seconds (p95) - Concurrent requests: Handle 10 simultaneous users - Rate limiting: 60 requests/minute per session Security: - CORS whitelist (GitHub Pages domain) - API key in environment variables - Input sanitization (prevent injection) - Rate limiting per IP Dependencies: - fastapi - uvicorn - openai - qdrant-client - psycopg2-binary - python-dotenv - pydantic Success Criteria: - All endpoints return correct responses - RAG retrieves relevant context (manual review) - Selected text feature works correctly - Chat history persists in Postgres - API deploys to Vercel successfully - Response time <3 seconds
```

## Response Summary
Successfully created a feature specification for the RAG Chatbot Backend. The following artifacts were generated:

1. `specs/002-rag-chatbot-backend/spec.md` - Feature specification with user stories, requirements, and success criteria
2. `specs/002-rag-chatbot-backend/checklists/requirements.md` - Specification quality checklist

The specification was created following these key steps:
1. Generated a short name "rag-chatbot-backend" and determined the next feature number (002)
2. Created a new feature branch and spec file
3. Developed a comprehensive specification that:
   - Focuses on user value and business needs
   - Removes implementation details (languages, frameworks, APIs)
   - Includes 3 prioritized user stories
   - Defines 12 functional requirements and 3 requiring clarification
   - Identifies key entities involved in the system
   - Establishes 9 measurable success criteria

The specification underwent quality validation and was refined to ensure it contained no implementation-specific details, focusing instead on business requirements and user needs.

## Outcome
The feature specification is complete and ready for the next phase of development. It successfully abstracts the technical implementation details while preserving the core functionality requirements. Three areas requiring clarification were identified (token limits, performance settings, and session policy) which will need to be addressed before implementation planning.

## Evaluation
The specification meets the required standards:
- Contains no implementation details (languages, frameworks, APIs)
- Focuses on user value and business needs
- Written for non-technical stakeholders
- All mandatory sections completed
- Requirements are testable and unambiguous
- Success criteria are measurable and technology-agnostic
- All acceptance scenarios are defined
- Edge cases are identified
- Scope is clearly bounded
- Dependencies and assumptions identified

The specification is ready for the planning phase once the three clarification questions are answered.