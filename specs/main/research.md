# Research Findings: Complete RAG chatbot system for Physical AI textbook

## Decision: Textbook Content Preprocessing
**Rationale**: The textbook content consists of markdown files in the docs/ directory. The optimal preprocessing approach is to:
- Parse the markdown files to extract content while preserving structure
- Split content into chunks of approximately 500 words with 50-word overlap to maintain context
- Convert to plain text while preserving key structural information (headings, sections) as metadata
- This approach ensures semantic coherence while optimizing for embedding quality
**Alternatives considered**: Alternative approaches included sentence-level chunking or section-level chunking. Sentence-level chunking was rejected for losing context, while section-level chunking was rejected for potentially being too large and losing focus.

## Decision: Text Selection Method/Detection
**Rationale**: For the selected text feature, the most effective approach is to implement a client-side JavaScript solution that:
- Detects when a user selects text on a page
- Captures the selected text content
- Sends the selected text along with the query to the backend as context
- Optionally highlights the selected text visually to confirm detection
This approach ensures the feature works reliably across different browsers and doesn't require complex server-side processing of page content.
**Alternatives considered**: Server-side text selection detection was considered but rejected for requiring complex DOM parsing and being less responsive. Plugin-based solutions were also considered but rejected for adding unnecessary dependencies.

## Decision: Architecture Components
**Rationale**: Based on the requirements and technology stack constraints, the system will be implemented with:
- Document Ingestion Pipeline: Python script to process markdown files from docs/
- FastAPI Backend: Asynchronous API server handling requests and orchestrating RAG flow
- Database Layer: Qdrant for vector storage of embeddings, Neon Postgres for chat history
- Frontend Widget: React component that integrates with Docusaurus
- Deployment: Serverless deployment on Vercel for backend, static integration in Docusaurus for frontend
**Alternatives considered**: Different frameworks like Flask were considered but FastAPI was chosen for its async support and automatic API documentation. Different database options like Pinecone were considered but Qdrant was chosen for the generous free tier.

## Decision: API Design
**Rationale**: The API will follow RESTful principles with:
- POST /chat for question/answer requests
- GET /history for retrieving conversation history
- GET /health for health checks
- This design follows the specified requirements and integrates well with frontend components.
**Alternatives considered**: GraphQL was considered but REST was chosen for its simplicity and widespread support in frontend frameworks.

## Decision: Error Handling and Fallbacks
**Rationale**: The system will implement multiple layers of error handling:
- At the API level: graceful degradation when external services (OpenAI, Qdrant) are unavailable
- At the frontend level: appropriate user messaging during errors
- At the ingestion level: validation of documents before embedding
- All with appropriate logging for debugging
**Alternatives considered**: More complex circuit breaker patterns were considered but rejected for adding unnecessary complexity for an educational application.

## Decision: Performance Optimization
**Rationale**: To meet the 3-second response time requirement:
- Implement caching for frequently asked questions
- Optimize vector search with appropriate indexing in Qdrant
- Use connection pooling for database operations
- Implement streaming responses if needed for long answers
**Alternatives considered**: Pre-computing answers was considered but rejected for being too static and unable to handle diverse queries effectively.

## Decision: Security and Rate Limiting
**Rationale**: Security will be implemented with:
- API key authentication for backend services
- Rate limiting to prevent abuse
- Input validation to prevent injection attacks
- Secure handling of user sessions
**Alternatives considered**: More complex authentication systems were considered but rejected for being overkill for an educational application.