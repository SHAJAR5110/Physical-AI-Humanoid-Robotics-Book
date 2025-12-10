# RAG Chatbot Implementation Tasks

**Feature**: 002-rag-chatbot
**Status**: Task Breakdown
**Created**: 2025-12-09
**Total Tasks**: 45
**Target**: MVP with floating widget in bottom-left corner, answers from book content only

---

## Table of Contents

1. [Implementation Strategy](#implementation-strategy)
2. [Phase 1: Project Setup & Infrastructure](#phase-1-project-setup--infrastructure)
3. [Phase 2: Foundational Services (Backend Core)](#phase-2-foundational-services-backend-core)
4. [Phase 3: User Story 1 - Context-Aware Q&A](#phase-3-user-story-1--context-aware-qa)
5. [Phase 4: User Story 2 - Text Selection Query](#phase-4-user-story-2--text-selection-query)
6. [Phase 5: User Story 3 - Source Attribution](#phase-5-user-story-3--source-attribution)
7. [Phase 6: Frontend Widget Integration](#phase-6-frontend-widget-integration)
8. [Phase 7: Polish & Deployment](#phase-7-polish--deployment)
9. [Dependencies & Execution Order](#dependencies--execution-order)
10. [Test Strategy](#test-strategy)

---

## Implementation Strategy

**MVP Scope**: Deliver a fully functional RAG chatbot with floating widget that:
1. Accepts questions from students
2. Retrieves relevant passages from book content (Qdrant + sentence-transformers)
3. Generates answers using Groq LLM API
4. Returns answers with source attribution and confidence scores
5. Displays in a floating widget (bottom-left corner) embedded in Docusaurus

**Tech Stack**:
- **Backend**: FastAPI (Python 3.11) with UV package manager & venv
- **Vector DB**: Qdrant Cloud (free tier)
- **Embeddings**: Sentence-Transformers (local, CPU-friendly)
- **LLM**: Groq API (free tier, fast inference)
- **Database**: Neon Postgres (optional, for future use)
- **Frontend**: React/TypeScript (via Docusaurus theme wrapper)
- **Deployment**: GitHub Pages (backend API) + Vercel (frontend already deployed)

**Parallelization**:
- Phase 2 (foundational): All services can be developed in parallel after setup
- Phase 3-5 (user stories): Each US can start independently after Phase 2 complete
- Frontend tasks can begin once API contract is finalized (Phase 2)

**Testing Approach**:
- Unit tests for each service (embedding, retrieval, generation, confidence)
- Integration tests for the full Q&A pipeline
- E2E tests for the floating widget with mock API responses
- No TDD required; tests can be added after implementation

---

## Phase 1: Project Setup & Infrastructure

**Goal**: Initialize backend project with UV package manager, create project structure, configure environment.

**Independent Test Criteria**:
- Backend folder structure matches plan.md
- `uv venv` creates isolated environment
- All dependencies in `pyproject.toml` install without errors
- `.env.example` includes all required variables
- `main.py` starts and returns 200 on `/health` endpoint

### Tasks

- [X] T001 Create backend project structure with UV package manager
  - Create `backend/` directory with `pyproject.toml`, `src/`, `tests/`, `scripts/` subdirectories
  - Initialize with `uv init backend` and configure Python 3.11
  - File: `backend/pyproject.toml`

- [X] T002 Create Python virtual environment using UV
  - Run `uv venv backend/.venv` in the backend directory
  - Verify venv is created at `backend/.venv/`
  - Create `backend/.venv_activate.sh` helper script for activation

- [X] T003 Configure dependencies in pyproject.toml
  - Add core dependencies: fastapi, uvicorn, pydantic, python-dotenv
  - Add RAG dependencies: sentence-transformers, qdrant-client, groq
  - Add dev dependencies: pytest, pytest-cov, mypy, black, ruff
  - Add type stubs: types-requests (if needed)
  - File: `backend/pyproject.toml`

- [X] T004 Create environment configuration files
  - Create `backend/.env.example` with template variables:
    - QDRANT_URL, QDRANT_API_KEY
    - GROQ_API_KEY
    - ALLOWED_ORIGINS, ENVIRONMENT, LOG_LEVEL, PYTHON_VERSION=3.11
  - Create `backend/src/config.py` to load and validate env vars
  - File: `backend/.env.example`, `backend/src/config.py`

- [X] T005 Create FastAPI main application file
  - Initialize FastAPI app with title "RAG Chatbot API"
  - Add metadata and version
  - Configure CORS with allowed origins from .env
  - Add `/health` endpoint (returns 200 with status)
  - File: `backend/main.py`

- [X] T006 Create project README with setup instructions
  - Document UV setup steps: `uv venv`, `source .venv/bin/activate`, `uv sync`
  - Document environment variable setup
  - Document how to run dev server: `uvicorn backend.main:app --reload`
  - Document how to run tests: `pytest tests/ -v --cov=src`
  - File: `backend/README.md`

---

## Phase 2: Foundational Services (Backend Core)

**Goal**: Build core RAG services that power all user stories. These are blocking tasks—all must complete before user stories.

**Independent Test Criteria**:
- Embedding service loads model and generates 1536-dim vectors in <500ms
- Retrieval service connects to Qdrant and returns top-5 results
- Generation service calls Groq and returns text in <2000ms
- Confidence service calculates confidence scores (high/medium/low)
- Error handler returns user-friendly error messages (never technical details)
- All services pass type checking with mypy

### Tasks

- [X] T007 Implement Embedding Service with sentence-transformers
  - Load `all-minilm-l6-v2` model at startup (singleton pattern)
  - Implement `embed_text(text: str) -> List[float]` method
  - Handle edge cases: empty text, very long text (>512 tokens)
  - Add logging and error handling
  - File: `backend/src/services/embedding_service.py`

- [X] T008 [P] Implement Retrieval Service with Qdrant client
  - Connect to Qdrant Cloud using URL and API key from .env
  - Implement `search(embedding: List[float], top_k: int, threshold: float)` method
  - Apply similarity threshold (≥0.85) before returning results
  - Extract metadata (chapter, module, section, text) from Qdrant payload
  - Handle connection errors gracefully
  - File: `backend/src/services/retrieval_service.py`

- [X] T009 [P] Implement Generation Service with Groq LLM
  - Initialize Groq client with API key from .env
  - Implement `generate_answer(question: str, context: str, max_tokens: int)` method
  - Use system prompt that enforces answer is based ONLY on context
  - Return generated text with timeout handling
  - Log prompt/response for debugging (redact API keys)
  - File: `backend/src/services/generation_service.py`

- [X] T010 [P] Implement Confidence Service
  - Calculate confidence score from:
    - Max cosine similarity (40% weight)
    - Number of relevant passages (30% weight)
    - Key term matching in answer (30% weight)
  - Return confidence level: "high" (≥0.8), "medium" (0.5-0.8), "low" (<0.5)
  - Implement `calculate_confidence(sources, answer, question)` method
  - File: `backend/src/services/confidence_service.py`

- [X] T011 [P] Create Pydantic models for request/response validation
  - Implement `ChatRequest` model: question (str, 3-1000 chars), selected_text (optional str, max 5000)
  - Implement `SourceRef` model: chapter, module, section, excerpt (str)
  - Implement `ChatResponse` model: answer, sources (list), confidence, processing_time_ms
  - Add validators to prevent injection attacks and sanitize inputs
  - File: `backend/src/models.py`

- [X] T012 [P] Implement error handler with user-friendly messages
  - Create `ErrorHandler` class that converts exceptions to user-facing messages
  - Map service errors to HTTP responses:
    - RetrievalError (no results) → "I couldn't find information on this topic..."
    - GenerationError → "I'm having trouble generating a response..."
    - ConnectionError → "Chat service is temporarily unavailable..."
  - Never expose stack traces to user
  - Add structured error logging with redaction
  - File: `backend/src/utils/error_handler.py`

- [X] T013 Implement context window manager for token limits
  - Calculate token count for passages using tiktoken or rough estimation
  - Implement `prepare_context(sources: List[SourceRef], max_tokens: int)` method
  - Truncate passages gracefully if exceeding limit (≤6k tokens)
  - Return formatted context string for LLM
  - File: `backend/src/utils/context_manager.py`

- [X] T014 [P] Create middleware for rate limiting
  - Implement rate limiting: 10 requests per minute per IP
  - Use simple in-memory tracking (dictionary + timestamps)
  - Return 429 Too Many Requests when limit exceeded
  - Include `Retry-After` header
  - File: `backend/src/middleware/rate_limit.py`

- [X] T015 [P] Create middleware for structured logging
  - Log all requests with method, path, timestamp
  - Log all responses with status code and latency
  - Log errors with type, message (redacted of sensitive data)
  - Output to stdout as JSON for easy parsing
  - File: `backend/src/middleware/logging.py`

- [X] T016 Create response formatter utility
  - Implement `format_chat_response(answer, sources, confidence, processing_time)` method
  - Validate response against ChatResponse schema
  - Ensure JSON serialization works correctly
  - File: `backend/src/utils/response_formatter.py`

- [X] T017 [P] Create input validators
  - Validate question length, characters, injection attempts
  - Validate selected_text if provided
  - Return clear error messages for validation failures
  - File: `backend/src/utils/validators.py`

- [X] T018 Create initialization/startup logic
  - Load embedding model at startup (print progress)
  - Verify Qdrant connection at startup (warn if unavailable, continue)
  - Verify Groq API at startup (warn if unavailable, continue)
  - Print startup summary with model info and config
  - File: `backend/src/startup.py` (called from main.py)

---

## Phase 3: User Story 1 - Context-Aware Q&A

**User Story Goal**: Students can ask questions about book content and receive answers synthesized from relevant passages.

**Definition of Done**:
- Student types question in widget
- System retrieves ≥1 relevant passage (cosine similarity ≥0.85)
- LLM synthesizes answer from passage(s)
- Answer returns within 3 seconds (p95)
- Response includes sources, confidence, processing time

**Independent Test Criteria**:
- POST `/api/chat` endpoint accepts valid ChatRequest and returns ChatResponse
- Retrieval returns only passages with similarity ≥0.85
- Answer includes at least 1 source reference
- Processing time is <3000ms for typical questions
- Error handling returns user-friendly message if no relevant content

### Tasks

- [X] T019 [P] [US1] Create /api/chat router and endpoint
  - Accept POST requests with ChatRequest body
  - Parse question and optional selected_text
  - Route to chat pipeline service
  - Return ChatResponse (200) or error response (400, 503, 429)
  - File: `backend/src/routers/chat.py`

- [X] T020 [US1] Create chat pipeline service orchestrating all steps
  - Implement `ChatPipelineService` that:
    1. Validates input (ChatRequest)
    2. Embeds question using EmbeddingService
    3. Retrieves passages using RetrievalService
    4. Prepares context using ContextManager
    5. Generates answer using GenerationService
    6. Calculates confidence using ConfidenceService
    7. Formats response using ResponseFormatter
  - Measure total processing time
  - Handle and convert exceptions to user-friendly errors
  - File: `backend/src/services/chat_pipeline_service.py`

- [X] T021 [P] [US1] Add unit tests for embedding service
  - Test embed_text with normal text (returns 1536-dim vector)
  - Test with empty text (raises ValueError)
  - Test with very long text (handles gracefully)
  - Test vector dimensions and normalization
  - File: `backend/tests/test_embedding_service.py`

- [X] T022 [P] [US1] Add unit tests for retrieval service
  - Test search with valid embedding (returns filtered results)
  - Test similarity threshold enforcement (no results < 0.85)
  - Test with no matching results (raises RetrievalError)
  - Test metadata extraction (chapter, module, section)
  - File: `backend/tests/test_retrieval_service.py`

- [X] T023 [P] [US1] Add unit tests for generation service
  - Test generate_answer with context and question
  - Test that answer respects max_tokens limit
  - Test error handling for API failures
  - File: `backend/tests/test_generation_service.py`

- [X] T024 [P] [US1] Add unit tests for confidence service
  - Test confidence calculation with high similarity sources
  - Test confidence calculation with low similarity sources
  - Test with different numbers of sources (1-5)
  - Verify confidence levels: high, medium, low
  - File: `backend/tests/test_confidence_scoring.py`

- [X] T025 [US1] Add integration test for full Q&A pipeline
  - Mock Qdrant and Groq responses
  - Test end-to-end: question → retrieval → generation → response
  - Verify response schema matches ChatResponse
  - Verify processing time is tracked
  - File: `backend/tests/test_e2e_qa_pipeline.py`

- [X] T026 [US1] Test /api/chat endpoint with mock services
  - Test valid request returns 200 with ChatResponse
  - Test invalid request returns 400 with error details
  - Test service unavailable returns 503
  - Test rate limiting returns 429
  - File: `backend/tests/test_chat_endpoint.py`

---

## Phase 4: User Story 2 - Text Selection Query

**User Story Goal**: Students can select text from a chapter and ask questions specifically about that excerpt, receiving more focused answers.

**Definition of Done**:
- Frontend sends selected_text in ChatRequest
- Backend prioritizes selected_text in retrieval/generation
- Answer acknowledges selected context
- System still validates against similarity threshold

**Independent Test Criteria**:
- selected_text parameter is optional but processed correctly
- Backend prioritizes selected text in answer synthesis
- Error handling for selected_text (too long, empty, etc.)

### Tasks

- [X] T027 [P] [US2] Update retrieval service to prioritize selected text
  - If selected_text provided, boost similarity score for passages matching selected text
  - Still apply threshold (≥0.85) to filtered results
  - Log when selected_text is used for debugging
  - File: `backend/src/services/retrieval_service.py` (update)

- [X] T028 [P] [US2] Update generation service to acknowledge selected context
  - Modify system prompt: "User selected this text: [selected_text]. Answer specifically about this context."
  - Prioritize selected text in answer synthesis
  - File: `backend/src/services/generation_service.py` (update)

- [X] T029 [US2] Update chat pipeline to handle selected_text
  - Pass selected_text through pipeline stages
  - Handle validation (max 5000 chars, not empty if provided)
  - Include selected_text in logging/debugging
  - File: `backend/src/services/chat_pipeline_service.py` (update)

- [X] T030 [P] [US2] Add unit tests for selected_text handling
  - Test retrieval with selected_text (prioritizes relevant passages)
  - Test generation with selected_text (acknowledges context)
  - Test validation (rejects empty/too-long selected_text)
  - File: `backend/tests/test_selected_text.py`

- [X] T031 [US2] Add integration test for selected_text flow
  - Mock request with question + selected_text
  - Verify answer focuses on selected context
  - Verify sources align with selected text
  - File: `backend/tests/test_e2e_selected_text.py`

---

## Phase 5: User Story 3 - Source Attribution

**User Story Goal**: Every answer includes accurate source citations to book chapters/modules so students can verify and learn more.

**Definition of Done**:
- Every answer includes ≥1 source reference
- Sources include chapter name, module name, section anchor
- Frontend can click source link and jump to relevant section
- Sources are accurate (matched to retrieval results)

**Independent Test Criteria**:
- ChatResponse.sources list is non-empty for all answers
- Each source has chapter, module, section, excerpt fields
- Section anchors are valid (slugified chapter/module names)
- Sources are returned in order of relevance

### Tasks

- [X] T032 [P] [US3] Update Qdrant indexing to include metadata
  - Ensure Qdrant collection has metadata fields: chapter, module, section, text
  - Create indexing script to parse book chapters and extract sections
  - Store vector embeddings with corresponding metadata in Qdrant
  - File: `backend/scripts/index_chapters.py`

- [X] T033 [US3] Create chapter indexing script for initial data load
  - Read markdown files from `book-source/docs/docs/Chapter-*.md`
  - Parse H1 (chapter), H2 (module), H3 (section) headings
  - Split by H3 sections; each becomes a Qdrant document
  - Generate embeddings using sentence-transformers
  - Store in Qdrant with chapter/module/section metadata
  - File: `backend/scripts/index_chapters.py` (full implementation)

- [X] T034 [P] [US3] Update retrieval service to return source metadata
  - Retrieval already extracts metadata; ensure SourceRef is properly formatted
  - Verify chapter, module, section are returned correctly
  - Test that excerpt matches the original text (no truncation)
  - File: `backend/src/services/retrieval_service.py` (update)

- [X] T035 [US3] Create section anchor generator utility
  - Convert chapter/module/section names to URL-safe anchors (e.g., "ROS 2 Topics" → "ros-2-topics")
  - Ensure anchors match Docusaurus heading slugs
  - File: `backend/src/utils/slug_generator.py`

- [X] T036 [US3] Update response formatter to include source links
  - For each source, generate clickable link: `/docs/[chapter-slug]#[section-slug]`
  - Ensure links are valid and lead to correct sections
  - File: `backend/src/utils/response_formatter.py` (update)

- [X] T037 [P] [US3] Add unit tests for source attribution
  - Test that sources are extracted correctly from Qdrant
  - Test source link generation (slug generation)
  - Test that excerpt text matches retrieved passage
  - Test multiple sources returned in order
  - File: `backend/tests/test_source_attribution.py`

- [X] T038 [US3] Add integration test for end-to-end source verification
  - Test that answers include all sources with correct metadata
  - Test that links are clickable and accurate
  - Verify excerpt field contains relevant text from chapter
  - File: `backend/tests/test_e2e_sources.py`

---

## Phase 6: Frontend Widget Integration

**User Story Goal**: Floating chatbot widget in bottom-left corner sends questions to backend and displays answers with proper formatting.

**Definition of Done**:
- Widget button visible in bottom-left corner on all book pages
- Clicking button opens/closes floating card
- Text selection auto-opens widget with selected text
- Submitting question calls `/api/chat` endpoint
- Response displays with formatting, sources, confidence, feedback buttons

**Independent Test Criteria**:
- ChatBot component renders button and widget (when open)
- Widget form submits question to API
- Response displays correctly with sources and confidence
- Error messages are user-friendly
- Mobile responsive (≤640px width)
- Loading states and spinners work correctly

### Tasks

- [X] T039 Create Docusaurus Root theme wrapper
  - Create `book-source/src/theme/Root.tsx`
  - Wrap app with ChatBot component for global availability
  - Ensure widget renders on all pages (docs, homepage, etc.)
  - File: `book-source/src/theme/Root.tsx`

- [X] T040 [P] Update ChatBot component for floating widget UI
  - Implement toggle button (💬) fixed in bottom-left corner
  - Implement collapsible card widget with header and close button
  - Implement form input (question textarea + optional selected text)
  - Implement loading state with spinner
  - Implement error state with dismiss button
  - Implement response display with answer, sources, confidence, feedback buttons
  - File: `book-source/src/components/ChatBot.tsx` (update)

- [X] T041 [P] Create ChatBot CSS module for floating widget styling
  - Style toggle button: 56px circle, gradient, fixed positioning
  - Style widget: fixed positioning, animations (slide up), responsive widths
  - Style header: gradient background, close button
  - Style input: textarea, submit button, selected text notice
  - Style response: answer text, sources (inline links), confidence indicator, feedback buttons
  - Mobile breakpoints: ≤640px widget width, max-height adjustments
  - File: `book-source/src/components/ChatBot.module.css` (update)

- [X] T042 [P] Update ChatBot component API integration
  - Fetch endpoint: `/api/chat` on the backend URL (from env variable)
  - Submit question + selected_text to backend
  - Handle response parsing (ChatResponse schema)
  - Handle errors (display error message from API response)
  - Implement timeout handling (show "still loading" after 2s)
  - Show processing time in response
  - File: `book-source/src/components/ChatBot.tsx` (update)

- [X] T043 [P] Implement source link handling in ChatBot component
  - Parse source links from ChatResponse
  - Render source links as clickable elements
  - Ensure links navigate to correct chapter sections (use anchor)
  - Open links in new tab (target="_blank")
  - File: `book-source/src/components/ChatBot.tsx` (update)

- [X] T044 [P] Implement confidence indicator in ChatBot component
  - Display confidence level: high (✅), medium (⚠️), low (❓)
  - Style with appropriate colors/opacity
  - Show processing time (ms)
  - File: `book-source/src/components/ChatBot.tsx` (update)

- [X] T045 [P] Implement feedback buttons in ChatBot component
  - Add thumbs up/down buttons for answer feedback
  - Add refresh button to clear conversation
  - Implement click handlers (log feedback for future integration)
  - Style appropriately for compact widget
  - File: `book-source/src/components/ChatBot.tsx` (update)

---

## Phase 7: Polish & Deployment

**Goal**: Finalize code quality, add remaining tests, prepare for production deployment.

**Independent Test Criteria**:
- All mypy type checks pass (strict mode)
- All tests pass (pytest with coverage)
- No API keys or secrets in code/logs
- CORS is properly configured for Vercel domain
- Cold start handling works correctly

### Tasks

- [ ] T046 Add type hints to all Python functions
  - Ensure all function signatures have input/return type hints
  - Pass mypy in strict mode (mypy --strict backend/src/)
  - File: All files in `backend/src/`

- [ ] T047 Run code quality checks
  - Format code with Black (line length 100): `black backend/src/ --line-length 100`
  - Lint with Ruff: `ruff check backend/src/`
  - Run type checking: `mypy --strict backend/src/`
  - Fix all issues
  - File: All files in `backend/src/`

- [ ] T048 Generate test coverage report
  - Run pytest with coverage: `pytest tests/ -v --cov=src --cov-report=html`
  - Verify coverage ≥80%
  - Identify and test uncovered lines
  - File: Coverage report in `backend/htmlcov/`

- [ ] T049 Create .gitignore for backend
  - Exclude `.venv`, `.env`, `__pycache__`, `*.pyc`, `.pytest_cache`, `htmlcov/`
  - Ensure API keys and credentials are never committed
  - File: `backend/.gitignore`

- [ ] T050 Configure CORS for production deployment
  - Add Vercel domain to ALLOWED_ORIGINS in .env
  - Test CORS headers are correct (Access-Control-Allow-Origin)
  - Ensure credentials are allowed if needed
  - File: `backend/src/config.py` (update)

- [ ] T051 Create cold start handling
  - Implement health check endpoint: `GET /health`
  - Pre-warm embedding model at startup
  - Return startup status in health response
  - Document cold start behavior for users
  - File: `backend/src/startup.py` (update)

- [ ] T052 Create deployment configuration for GitHub Pages
  - Create `.github/workflows/deploy-backend.yml`
  - Build backend on push to main
  - Deploy API to GitHub Pages (or alternative: Railway/Render)
  - Set environment variables in GitHub Actions secrets
  - File: `.github/workflows/deploy-backend.yml`

- [ ] T053 Create API documentation with examples
  - Document all endpoints (POST /api/chat, GET /health)
  - Include request/response examples
  - Include error codes and meanings
  - Add curl examples for testing
  - File: `backend/API.md`

- [ ] T054 Create deployment checklist
  - Verify backend builds locally without errors
  - Verify all tests pass
  - Verify mypy strict mode passes
  - Verify coverage ≥80%
  - Verify no secrets in code/logs
  - Verify API keys are in environment variables
  - Verify CORS is configured correctly
  - File: `backend/DEPLOYMENT.md`

- [ ] T055 Create comprehensive README for backend
  - Document project structure
  - Document setup with UV: `uv venv`, `uv sync`
  - Document environment variables
  - Document how to run dev server
  - Document how to index chapters
  - Document how to run tests
  - Document how to deploy
  - File: `backend/README.md` (update)

---

## Dependencies & Execution Order

### Blocking Order (Must Complete in Sequence):

1. **Phase 1** (Setup) → Phase 2 (Foundations) → Phases 3-6 (Parallel after Phase 2)
2. Phase 2 must complete before any User Stories (all depend on core services)
3. Phase 3 (US1) must complete before Phase 4 (US2 depends on US1 pipeline)

### Parallel Opportunities:

- **Phase 2**: Tasks T007-T017 can run in parallel (different services/utilities)
- **Phase 3**: Tasks T021-T026 (tests) can run in parallel with T019-T020 (implementation)
- **Phase 4**: Tasks T027-T031 can run in parallel after Phase 3
- **Phase 5**: Tasks T032-T038 can run in parallel after Phase 3
- **Phase 6**: Tasks T039-T045 can run in parallel with Phases 4-5 (frontend independent)
- **Phase 7**: Polish tasks T046-T055 can start anytime but should complete before merge

### Recommended MVP Execution Path:

1. Complete Phase 1 (6 tasks) - 30 min
2. Complete Phase 2 (12 tasks) - 2-3 hours
3. Complete Phase 3 (8 tasks) - 1.5-2 hours (includes tests)
4. Test Phase 3 end-to-end with mock data
5. Complete Phase 6 (7 tasks) - 1.5-2 hours (widget integration)
6. Complete Phase 7 quality checks (10 tasks) - 1-1.5 hours

**Total estimated MVP time**: 8-10 hours (can be parallelized to 4-5 hours with team)

---

## Test Strategy

### Unit Tests (Provided)

Each service has isolated unit tests:
- Embedding Service: Model loading, vector generation, edge cases
- Retrieval Service: Qdrant queries, threshold enforcement, metadata extraction
- Generation Service: LLM calls, error handling, timeout management
- Confidence Service: Score calculation, level classification
- Models: Pydantic validation, injection prevention

### Integration Tests (Provided)

Full pipeline tests with mocked external services:
- E2E Q&A pipeline: question → embedding → retrieval → generation → response
- Selected text handling: question + selected_text flows through pipeline
- Source attribution: sources are extracted and formatted correctly
- Error scenarios: invalid input, no results, API failures

### Frontend Tests (Recommended but Optional)

- Component rendering (button, widget open/close)
- Form submission and API integration
- Response display and formatting
- Error handling and loading states
- Mobile responsiveness

### Manual Testing Checklist

Before deployment:
- [ ] Ask a simple question about a chapter → receive answer within 3 seconds
- [ ] Verify answer includes source links
- [ ] Click source link → jumps to correct chapter section
- [ ] Select text on a chapter → widget auto-opens with selection
- [ ] Verify confidence indicator shows (high/medium/low)
- [ ] Test error cases: rate limit, service unavailable, no results
- [ ] Test on mobile device (≤640px) → widget is usable
- [ ] Test loading state → spinner appears, estimated time shown if >2s

---

## Next Steps After Tasks

1. **Task Execution**: Developers choose parallelization strategy per phase
2. **Testing**: Run unit/integration tests as tasks complete
3. **Code Review**: Ensure mypy strict mode, Black formatting, test coverage ≥80%
4. **Deployment**: Use GitHub Actions to deploy backend to GitHub Pages
5. **Monitoring**: Monitor latency, error rates, uptime after deployment
6. **Iteration**: Gather user feedback and plan Phase 4 (Conversation History)

---

**Document Version**: 1.0
**Status**: Ready for Implementation
**Last Updated**: 2025-12-09
