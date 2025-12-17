# Tasks: Document Ingestion Pipeline

**Input**: Design documents from `/specs/004-doc-ingestion-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Project Setup

**Purpose**: Establish project structure and dependencies for the ingestion pipeline

- [ ] T001 Create scripts/ directory if it doesn't exist
- [ ] T002 Create tests/ directory if it doesn't exist
- [ ] T003 [P] Set up requirements.txt with dependencies (openai, qdrant-client, python-dotenv, pathlib, re, tqdm)
- [ ] T004 Create .env file template for API keys

---

## Phase 2: Core Ingestion Components (Blocking Prerequisites)

**Purpose**: Core components that MUST be complete before processing can begin

**⚠️ CRITICAL**: No document processing can begin until this phase is complete

- [ ] T005 Create the main script file scripts/ingest_docs.py
- [ ] T006 Implement command line argument parsing for --docs-path, --collection-name, --batch-size, --chunk-size, --overlap, --reset
- [ ] T007 Implement file discovery function to recursively scan docs/ directory for .md and .mdx files
- [ ] T008 Implement markdown processing function to remove YAML frontmatter
- [ ] T009 Implement intelligent chunking algorithm based on headers and paragraph boundaries
- [ ] T010 Implement metadata extraction for module, section, page, word count, code presence
- [ ] T011 Set up Qdrant client connection and collection management
- [ ] T012 Implement embedding generation using OpenAI text-embedding-3-small
- [ ] T013 Implement progress tracking with logging and progress bar

---

## Phase 3: Processing Features (Priority: P1) 🎯 MVP

**Goal**: Core document processing functionality that ingests files and stores embeddings

**Independent Test**: Process a sample docs/ directory and verify all markdown content is processed into the vector database.

### Validation Tests for Processing (REQUIRED) ⚠️

> **NOTE: Write these tests FIRST to verify core functionality**

- [ ] T014 [P] [US1] Test file discovery finds all .md and .mdx files while excluding build directories
- [ ] T015 [P] [US1] Test YAML frontmatter is properly removed from markdown files
- [ ] T016 [P] [US2] Test chunking algorithm creates segments based on header boundaries
- [ ] T017 [P] [US2] Test long sections (>1000 words) are subdivided into paragraph-based chunks with overlap

### Implementation for Processing

- [ ] T018 [P] [US1] Implement recursive file scanning to find all markdown files in docs/
- [ ] T019 [P] [US1] Implement filtering to exclude build and cache directories (node_modules/, build/, .docusaurus/)
- [ ] T020 [US1] Implement YAML frontmatter removal during file processing
- [ ] T021 [US2] Implement header-based chunking logic (H1, H2, H3 boundaries)
- [ ] T022 [US2] Implement paragraph-based sub-chunking for sections >1000 words
- [ ] T023 [US2] Implement 500-word target with 50-word overlap between chunks
- [ ] T024 [US2] Implement header preservation in chunks for context maintenance

**Checkpoint**: At this point, the core processing should be able to read files, chunk them intelligently, and prepare them for storage

---

## Phase 4: Storage & Metadata (Priority: P2)

**Goal**: Store processed content with metadata in vector database

**Independent Test**: Verify that content and metadata are correctly stored in the vector database and can be retrieved later.

### Validation Tests for Storage (REQUIRED) ⚠️

- [ ] T025 [P] [US3] Test that content chunks are stored in vector database with accurate metadata
- [ ] T026 [P] [US3] Test that all metadata (module, section, page, word count, code presence) is preserved

### Implementation for Storage

- [ ] T027 [P] [US3] Implement metadata extraction for module, section, page, chunk_index, word_count, has_code
- [ ] T028 [US3] Implement storage of content and metadata in Qdrant vector database
- [ ] T029 [US3] Implement vector dimension handling (1536 for text-embedding-3-small)
- [ ] T030 [US3] Implement UUID generation for point IDs in Qdrant
- [ ] T031 [US3] Implement cosine distance metric configuration for Qdrant collection

**Checkpoint**: At this point, processed content should be stored in the vector database with complete metadata

---

## Phase 5: API Integration & Error Handling (Priority: P3)

**Goal**: Integrate with OpenAI API with proper error handling and rate limiting

**Independent Test**: Verify that embeddings are generated correctly and errors are handled gracefully.

### Validation Tests for API Integration (REQUIRED) ⚠️

- [ ] T032 [P] Test that API rate limits are properly handled
- [ ] T033 [P] Test that API errors are retried appropriately
- [ ] T034 [P] Test that connection issues are handled gracefully

### Implementation for API Integration

- [ ] T035 [P] Implement OpenAI client setup with text-embedding-3-small model
- [ ] T036 Batch processing implementation (10 chunks at a time)
- [ ] T037 Implement rate limit handling (20 requests/min)
- [ ] T038 Implement retry logic with exponential backoff for API errors
- [ ] T039 Implement error handling for OpenAI API error → Retry 3 times
- [ ] T040 Implement error handling for Qdrant connection error → Exit with message
- [ ] T041 Implement error handling for file read error → Skip file, log warning
- [ ] T042 Implement error handling for invalid markdown → Skip, log error

**Checkpoint**: At this point, the system should handle all API interactions and errors properly

---

## Phase 6: Progress Tracking & Checkpoints (Priority: P4)

**Goal**: Implement comprehensive progress tracking and resumable processing

**Independent Test**: Verify that processing can resume from a checkpoint if interrupted.

### Validation Tests for Progress Tracking (REQUIRED) ⚠️

- [ ] T043 Test that processing progress is accurately tracked and displayed
- [ ] T044 Test that processing can resume from a checkpoint after interruption

### Implementation for Progress Tracking

- [ ] T045 Implement logging of each file processed with timestamps
- [ ] T046 Implement progress bar using tqdm library
- [ ] T047 Implement total chunk counter as they're ingested
- [ ] T048 Implement time remaining estimation
- [ ] T049 Implement checkpoint saving with file position to resume from
- [ ] T050 Implement resume functionality from last checkpoint

**Checkpoint**: At this point, the system should provide comprehensive feedback during processing and support resumption

---

## Phase 7: Validation & Reporting (Priority: P5)

**Goal**: Implement validation and create summary reports

**Independent Test**: Verify that the system generates appropriate reports and can validate successful processing.

### Validation Tests for Validation & Reporting (REQUIRED) ⚠️

- [ ] T051 Test that summary report is generated with processing statistics
- [ ] T052 Test that all files have been successfully processed
- [ ] T053 Test that vector count in Qdrant matches expectations
- [ ] T054 Test retrieval with sample queries to verify content is properly stored

### Implementation for Validation & Reporting

- [ ] T055 Implement validation to check all files were ingested
- [ ] T056 Implement verification of vector count in Qdrant
- [ ] T057 Implement spot-check functionality for random chunks
- [ ] T058 Implement sample query testing to verify retrieval
- [ ] T059 Implement summary report generation (total files, chunks, vectors, time, errors)
- [ ] T060 Implement final verification that no data was lost during processing

**Checkpoint**: At this point, the system should provide comprehensive validation and reporting

---

## Phase N: Polish & Integration

**Purpose**: Improvements that enhance the system

- [ ] T061 [P] Performance optimization for large document sets
- [ ] T062 Documentation for the ingestion pipeline script
- [ ] T063 Integration testing with the RAG system
- [ ] T064 Run final validation to ensure all success criteria are met

---

## Dependencies & Execution Order

### Phase Dependencies

- **Project Setup (Phase 1)**: No dependencies - can start immediately
- **Core Ingestion Components (Phase 2)**: Depends on Setup completion - BLOCKS all processing
- **Processing Features (Phase 3)**: Depends on Core Components phase completion
- **Storage & Metadata (Phase 4)**: Depends on Processing Features phase completion
- **API Integration & Error Handling (Phase 5)**: Depends on Storage & Metadata phase completion
- **Progress Tracking & Checkpoints (Phase 6)**: Depends on API Integration phase completion
- **Validation & Reporting (Phase 7)**: Depends on Progress Tracking phase completion
- **Polish & Integration (Final Phase)**: Depends on all previous phases being complete

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Validation tests for a phase can run in parallel
- Within each phase, tasks marked [P] can run in parallel if they work on different components

---

## Implementation Strategy

### MVP First (Processing & Storage)

1. Complete Phase 1: Project Setup
2. Complete Phase 2: Core Ingestion Components
3. Complete Phase 3: Processing Features
4. Complete Phase 4: Storage & Metadata
5. **STOP and VALIDATE**: Test core functionality processes and stores content
6. Continue to subsequent phases

### Incremental Delivery

1. Project Setup → Core Components ready
2. Add Processing Features → Basic document processing works
3. Add Storage → Content stored in vector DB
4. Add API Integration → Embeddings generated and stored
5. Add Progress Tracking → Comprehensive processing feedback
6. Add Validation & Reporting → Complete verification
7. Add Polish → Final refinements

---

## Notes

- [P] tasks = different files, no dependencies
- [US#] label maps task to specific user story for traceability
- Each phase should be independently testable and valuable
- Commit after each task or logical group
- Stop at any checkpoint to validate functionality independently
- Avoid: vague tasks, same file conflicts, cross-phase dependencies that break independent validation