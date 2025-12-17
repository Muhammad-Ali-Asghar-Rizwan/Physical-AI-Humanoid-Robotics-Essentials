# Data Model: Complete RAG chatbot system for Physical AI textbook

## Entities

### ChatSession
Represents a conversation session

**Fields:**
- `session_id`: UUID, unique identifier for the session
- `user_id`: UUID, identifier for the user (optional, for registered users)
- `anonymous_id`: UUID, identifier for anonymous users
- `created_at`: DateTime, timestamp of session creation
- `last_interaction_at`: DateTime, timestamp of the most recent interaction
- `is_active`: Boolean, indicates if session is currently active

**Relationships:**
- One ChatSession has many ChatMessages

### ChatMessage
Represents an individual message in the conversation

**Fields:**
- `message_id`: UUID, unique identifier for the message
- `session_id`: UUID, foreign key linking to ChatSession
- `sender_type`: Enum (USER | AI), indicates the sender
- `content`: Text, the actual message content
- `timestamp`: DateTime, when the message was created
- `source_citations`: JSON, references to textbook sections used in AI responses
- `confidence_score`: Float (0.0-1.0), confidence in AI response

**Relationships:**
- Many ChatMessages belong to one ChatSession

### DocumentChunk
Represents a chunk of processed textbook content for retrieval

**Fields:**
- `chunk_id`: UUID, unique identifier for the chunk
- `source_document_path`: String, path to original document in docs/
- `content`: Text, the chunk content
- `embedding_vector`: Vector, the OpenAI embedding vector
- `metadata`: JSON, additional info like headings, section names
- `created_at`: DateTime, timestamp of chunk creation

**Relationships:**
- One DocumentChunk has many ChatMessages associated with it through citations

### UserSession
Represents a user's browsing session for session management

**Fields:**
- `session_token`: String, unique token identifying the browser session
- `user_id`: UUID, identifier for the user (if registered)
- `anonymous_id`: UUID, identifier for anonymous users
- `created_at`: DateTime, timestamp of session creation
- `expires_at`: DateTime, when the session expires

**Relationships:**
- One UserSession maps to one ChatSession

## Validation Rules

### ChatSession Validation
- `session_id` must be unique
- `created_at` must be before `last_interaction_at`

### ChatMessage Validation
- `session_id` must reference an existing ChatSession
- `sender_type` must be either USER or AI
- `content` must not be empty
- `timestamp` must be after the session's `created_at`
- `confidence_score` must be between 0.0 and 1.0 if provided

### DocumentChunk Validation
- `source_document_path` must exist in the textbook
- `embedding_vector` must have correct dimensions for OpenAI embeddings
- `content` must not exceed token limits for OpenAI

### UserSession Validation
- `session_token` must be unique
- `expires_at` must be after `created_at`
- Either `user_id` or `anonymous_id` must be provided

## State Transitions

### ChatSession States
- `NEW`: Just created, no messages yet
- `ACTIVE`: Has exchanged at least one message
- `INACTIVE`: No activity for a period (still recoverable)
- `EXPIRED`: Session has timed out and is no longer recoverable

### UserSession States
- `ACTIVE`: Currently active in browser
- `EXPIRED`: TTL has passed, needs renewal
- `TERMINATED`: Explicitly ended by user