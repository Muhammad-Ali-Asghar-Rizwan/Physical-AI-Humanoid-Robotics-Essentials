# Research: RAG Chatbot Integration for Physical AI & Humanoid Robotics Textbook

## Overview

This research document addresses the technical requirements and unknowns identified during the planning phase of the RAG chatbot integration. It covers key technology decisions, architecture patterns, and implementation details required for successfully implementing the feature.

## Text Chunking Strategy

**Decision**: Use semantic text splitting with 500-1000 token chunks
**Rationale**: Semantic splitting maintains context within text chunks while keeping them small enough for efficient embedding and retrieval. This size range is optimal for both the Cohere embedding model and for providing relevant context to the OpenAI LLM.
**Alternatives considered**: 
- Fixed character count splitting (rejected - doesn't preserve semantic boundaries)
- Sentence-based splitting (rejected - could result in chunks too large for embedding limits)

## Embedding Technology Choice

**Decision**: Use Cohere's embed-multilingual-v3.0 model
**Rationale**: Cohere embeddings are specifically optimized for retrieval tasks and provide excellent performance for semantic similarity search. The model handles technical content well, which is important for a textbook about AI and robotics.
**Alternatives considered**:
- OpenAI's text-embedding-3-small (rejected - already using OpenAI for LLM generation, Cohere is optimized for retrieval)
- Self-hosted embedding models (rejected - increases complexity beyond requirements)

## Vector Database Technology

**Decision**: Use Qdrant Cloud Free Tier
**Rationale**: Qdrant provides excellent performance for semantic search with efficient similarity matching algorithms. The cloud tier offers managed infrastructure with good API support and scaling capabilities.
**Alternatives considered**:
- Pinecone (rejected - prefer open source solution where possible)
- Weaviate (rejected - Qdrant has better documentation and Python support)
- Self-hosted vector databases (rejected - increases operational complexity)

## Backend Technology

**Decision**: FastAPI for backend API
**Rationale**: FastAPI provides automatic API documentation, excellent performance through ASGI, and strong type validation. It integrates well with the Python ecosystem and supports async operations needed for API calls to embedding and LLM services.
**Alternatives considered**:
- Flask (rejected - lacks automatic documentation and async support)
- Django (rejected - overkill for API-only service)
- Node.js/Express (rejected - team expertise is in Python, Python better for ML integrations)

## Database Technology

**Decision**: Neon Serverless Postgres for storing chat history and metadata
**Rationale**: Neon provides serverless PostgreSQL with excellent scaling characteristics and familiar SQL interface. It handles the structured data requirements for chat history, user info, and metadata efficiently.
**Alternatives considered**:
- MongoDB (rejected - prefer relational model for chat history and user data)
- SQLite (rejected - not suitable for web application at scale)
- DynamoDB (rejected - overkill for initial implementation)

## LLM Technology

**Decision**: OpenAI GPT-4 for response generation
**Rationale**: OpenAI models provide state-of-the-art language understanding and generation capabilities. GPT-4 offers excellent performance for generating educational content and is well-documented with good support.
**Alternatives considered**:
- Open-source LLMs (rejected - hosting and optimization complexity)
- Anthropic Claude (rejected - already using OpenAI for ecosystem consistency)
- Cohere Command (rejected - OpenAI better for generation, Cohere better for embeddings)

## Frontend Integration Strategy

**Decision**: React-based chat widget for Docusaurus integration
**Rationale**: Docusaurus is built with React, making it straightforward to integrate a React-based chat widget. This approach ensures consistency with the existing technology stack and allows easy text selection functionality.
**Alternatives considered**:
- Vanilla JavaScript widget (rejected - harder to maintain and less feature-rich)
- Web components (rejected - would add additional complexity)
- Iframe embedding (rejected - would limit integration and styling capabilities)

## Text Selection and Processing

**Decision**: Client-side text selection with metadata preservation
**Rationale**: Handling text selection on the frontend provides immediate user feedback and preserves context from the Docusaurus content. The selected text is sent to the backend with sufficient metadata to enable focused Q&A.
**Alternatives considered**:
- Server-side text extraction (rejected - increases latency and server load)
- Pre-processed text segments (rejected - less flexible for user-defined queries)

## Security and Rate Limiting

**Decision**: API key authentication with rate limiting
**Rationale**: Each service (Cohere, OpenAI, Qdrant) requires its own API key. Implementing proper authentication and rate limiting at the API level prevents abuse while maintaining security.
**Alternatives considered**:
- OAuth (rejected - overkill for this use case)
- IP-based rate limiting only (rejected - less secure than API key approach)

## Performance Optimization Strategies

**Decision**: Caching, async operations, and embedding pre-processing
**Rationale**: To ensure responses are delivered within 3 seconds, implement caching for frequently requested information, use async operations throughout the stack, and pre-process textbook embeddings to minimize query time.
**Alternatives considered**:
- Synchronous processing (rejected - would likely exceed performance targets)
- No caching (rejected - would increase response times and API costs)

## Deployment Architecture

**Decision**: Vercel for backend API, GitHub Pages for frontend
**Rationale**: Vercel provides excellent support for FastAPI deployments and integrates well with GitHub. The frontend will already be deployed on GitHub Pages via the Docusaurus setup.
**Alternatives considered**:
- AWS (rejected - adds unnecessary complexity)
- Docker containers (rejected - adds operational overhead)
- Railway (rejected - Vercel already used for documentation site)