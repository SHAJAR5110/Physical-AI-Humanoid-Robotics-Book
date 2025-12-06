# Tasks: Physical AI & Humanoid Robotics Book MVP

**Feature Branch**: `001-mvp-features` | **Date**: 2025-12-06 | **Spec**: spec.md | **Plan**: plan.md

**Task Total**: ~52 tasks across 7 user stories | **Sprint Duration**: 2-3 weeks (hackathon)

---

## Task Execution Strategy

**MVP Approach (Hackathon Optimized)**:
1. **Week 1 (P1 Stories)**: Book Content, Auth & Profiles, Deployment Foundation
2. **Week 2 (P2 Stories)**: Personalization, Translation, RAG Chatbot
3. **Week 3 (Polish)**: Integration testing, demo refinement, bug fixes

**Parallel Opportunities**: Backend services can be developed in parallel with frontend UI. Chapter content can be authored while auth/API development occurs.

---

## Phase 1: Setup & Initialization

Project initialization and shared infrastructure setup.

- [ ] T001 Create repository directory structure (backend/, frontend/, docs/, tests/, .github/)
- [ ] T002 Initialize Python backend project: `pip init`, requirements.txt (FastAPI, SQLAlchemy, Pydantic, python-dotenv, pytest)
- [ ] T003 Initialize Node.js/Docusaurus frontend project: `npx create-docusaurus@latest`, Tailwind CSS setup
- [ ] T004 Create .env.example template (API keys, database URLs, secrets placeholders)
- [ ] T005 Set up GitHub Actions CI/CD: backend-tests.yml, frontend-build.yml, deploy.yml
- [ ] T006 Initialize Git pre-commit hooks: Black (Python), Prettier (JS), ruff linting
- [ ] T007 Create docker-compose.yml for local development (PostgreSQL, Qdrant, optional Redis)

---

## Phase 2: Foundational Infrastructure (Blocking Dependencies)

Essential services and APIs required before user story implementation.

- [ ] T008 [P] Set up Neon Postgres connection in backend/src/db/connection.py (SQLAlchemy)
- [ ] T009 [P] Set up Qdrant client in backend/src/db/qdrant.py (vector store connection)
- [ ] T010 Create SQLAlchemy Base and session management in backend/src/db/__init__.py
- [ ] T011 Implement BetterAuth integration skeleton in backend/src/services/auth_service.py (interface stubs)
- [ ] T012 Create FastAPI app entry point with middleware in backend/src/main.py (CORS, error handling, logging)
- [ ] T013 Set up Docusaurus theme and navigation in frontend/docusaurus.config.js and frontend/sidebars.js
- [ ] T014 Create React layout component in frontend/src/components/Layout.tsx (header, sidebar, footer)
- [ ] T015 [P] Initialize Docusaurus docs structure in frontend/docs/ with placeholder chapters

---

## Phase 3: User Story 1 - Read Book Content Online (P1)

**Story Goal**: Users can read 6 chapters sequentially with proper formatting and navigation.

**Independent Test**: Load Docusaurus site, navigate all 6 chapters, verify content renders with syntax-highlighted code.

**Tasks**:

- [ ] T016 [US1] Author Chapter 1: Introduction to Physical AI (frontend/docs/intro.md, ~5K words)
- [ ] T017 [US1] Author Chapter 2: ROS 2 Fundamentals (frontend/docs/ros2.md, ~5K words, include code examples)
- [ ] T018 [US1] Author Chapter 3: Gazebo Simulation (frontend/docs/gazebo.md, ~5K words, URDF/SDF examples)
- [ ] T019 [US1] Author Chapter 4: NVIDIA Isaac Platform (frontend/docs/isaac.md, ~5K words, Isaac examples)
- [ ] T020 [US1] Author Chapter 5: Vision-Language-Action Models (frontend/docs/vla.md, ~5K words)
- [ ] T021 [US1] Author Chapter 6: Capstone Project (frontend/docs/capstone.md, ~5K words, full project example)
- [ ] T022 Create CodeBlock component with syntax highlighting in frontend/src/components/CodeBlock.tsx (Prism or highlight.js)
- [ ] T023 Configure Docusaurus Markdown plugins for code highlighting in frontend/docusaurus.config.js
- [ ] T024 Implement chapter navigation sidebar in frontend/src/components/ChapterNav.tsx (links to all 6 chapters)
- [ ] T025 [P] Implement "Next Chapter" footer link in frontend/src/components/ChapterFooter.tsx
- [ ] T026 Build responsive CSS for chapter layout in frontend/src/styles/chapters.css (mobile-friendly)
- [ ] T027 [P] Test chapter rendering: Verify all 6 chapters render correctly with formatting (manual E2E)
- [ ] T028 [P] Test code block syntax highlighting in all chapters (Python, YAML, Bash examples)
- [ ] T029 Deploy Docusaurus to GitHub Pages (GitHub Actions workflow in .github/workflows/frontend-deploy.yml)

---

## Phase 4: User Story 2 - Sign Up and Build User Profile (P1)

**Story Goal**: Users can register, sign in, and manage their profile (OS, GPU, experience, robotics background).

**Independent Test**: Complete signup flow, verify account created, log out, log in with new credentials, confirm profile restored.

**Tasks**:

- [ ] T030 Create User model in backend/src/models/user.py (id, email, name, os, gpu, experience_level, robotics_background, created_at, updated_at)
- [ ] T031 Create Session model in backend/src/models/session.py (session_id, user_id, token_hash, expires_at, created_at)
- [ ] T032 Create Alembic migrations in backend/src/db/migrations/ for User and Session tables
- [ ] T033 Implement UserService in backend/src/services/user_service.py (create, read, update, delete operations)
- [ ] T034 Implement SessionService in backend/src/services/session_service.py (create, validate, expire sessions)
- [ ] T035 Integrate BetterAuth in backend/src/services/auth_service.py (signup, signin, signout, token validation)
- [ ] T036 [P] Create Auth API routes in backend/src/api/routes/auth.py (POST /signup, POST /signin, POST /signout, GET /profile)
- [ ] T037 [P] Implement BetterAuth middleware in backend/src/api/middleware.py (protect endpoints, inject user context)
- [ ] T038 Create SignUp form component in frontend/src/components/SignUpForm.tsx (email, password, OS dropdown, GPU dropdown, experience dropdown, robotics background textarea)
- [ ] T039 Create SignIn form component in frontend/src/components/SignInForm.tsx (email, password fields)
- [ ] T040 Create UserProfile component in frontend/src/components/UserProfile.tsx (display user background, edit button)
- [ ] T041 Implement useAuth hook in frontend/src/hooks/useAuth.ts (login, logout, user state management)
- [ ] T042 Add auth pages to Docusaurus in frontend/src/pages/signup.tsx and frontend/src/pages/signin.tsx
- [ ] T043 [P] Implement session storage in frontend/localStorage) or cookies (secure token handling)
- [ ] T044 Test auth flow: Signup → auto-login → logout → signin → profile verification (manual E2E)
- [ ] T045 Test duplicate email rejection (attempt signup with existing email, verify error)
- [ ] T046 Test session expiry (wait 24 hours or mock time, verify re-auth required)

---

## Phase 5: User Story 7 - Deploy and Access Online (P1)

**Story Goal**: Frontend deployed to GitHub Pages, backend deployed to Render/Heroku, live and accessible.

**Independent Test**: Visit public GitHub Pages URL, see book rendered. Visit backend API health check endpoint, get 200 response.

**Tasks**:

- [ ] T047 Set up Render account and free-tier backend instance
- [ ] T048 Configure backend environment variables in Render (DATABASE_URL, QDRANT_URL, CLAUDE_API_KEY, etc.)
- [ ] T049 Create backend Dockerfile in backend/Dockerfile (Python 3.11, FastAPI, gunicorn)
- [ ] T050 Deploy backend to Render with auto-deploys from GitHub (via Render dashboard or GitHub Actions)
- [ ] T051 Create health check endpoint in backend/src/api/routes/health.py (GET /health returns {"status": "ok"})
- [ ] T052 Configure CORS in FastAPI to allow frontend domain requests in backend/src/main.py
- [ ] T053 Update frontend API base URL in frontend/src/constants/config.ts to point to deployed backend
- [ ] T054 Deploy frontend to GitHub Pages via GitHub Actions (.github/workflows/frontend-deploy.yml)
- [ ] T055 Test frontend loads from GitHub Pages public URL (manual browser test)
- [ ] T056 Test backend API is accessible from frontend (test /health endpoint call from browser)
- [ ] T057 Verify CORS headers allow cross-origin requests (browser dev tools check)
- [ ] T058 Document deployment URLs in README.md (Book URL, Backend API URL, GitHub repo link)

---

## Phase 6: User Story 3 - Personalize Chapter Content (P2)

**Story Goal**: Logged-in users click "Personalize" button, receive chapter adapted to their profile in 10–25 seconds.

**Independent Test**: Sign in → open chapter → click Personalize → verify adapted content differs from original based on profile attributes.

**Tasks**:

- [ ] T059 Create PersonalizedContent model in backend/src/models/cache.py (id, user_id, chapter_id, personalized_text, created_at, expires_at)
- [ ] T060 Create Alembic migration for PersonalizedContent table in backend/src/db/migrations/
- [ ] T061 Implement PersonalizationService in backend/src/services/personalization_service.py:
  - [ ] T061a Retrieve user profile from database
  - [ ] T061b Retrieve original chapter content
  - [ ] T061c Call Claude API with user context (experience level, background)
  - [ ] T061d Cache result in Neon Postgres with 30-day TTL
  - [ ] T061e Handle timeouts (return original if >25s)
- [ ] T062 Create Personalize API endpoint in backend/src/api/routes/personalize.py (POST /personalize, requires auth)
- [ ] T063 Implement PersonalizeButton component in frontend/src/components/PersonalizeButton.tsx:
  - [ ] T063a Show loading spinner while generating
  - [ ] T063b Call POST /personalize endpoint
  - [ ] T063c Display personalized content or fallback on timeout
  - [ ] T063d Track error state with user-friendly message
- [ ] T064 Add Personalize button to chapter layout (integrate into ChapterLayout or similar)
- [ ] T065 Test personalization for beginner profile (verify explanations are detailed, jargon reduced)
- [ ] T066 Test personalization for advanced profile (verify explanations concise, advanced topics expanded)
- [ ] T067 Test timeout behavior (simulate slow API, verify fallback to original at 25s)
- [ ] T068 Test cache hit (personalize same chapter twice, verify second call is instant)

---

## Phase 7: User Story 4 - Translate Chapter to Urdu (P2)

**Story Goal**: Users click "Translate to Urdu" button, entire chapter rendered in Urdu, code blocks remain English, toggle back to English.

**Independent Test**: Open chapter → click Translate to Urdu → verify Urdu text readable, code unchanged. Toggle back → see English.

**Tasks**:

- [ ] T069 Create TranslationCache model in backend/src/models/cache.py (id, chapter_id, language, translated_text, created_at, expires_at)
- [ ] T070 Create Alembic migration for TranslationCache table
- [ ] T071 Implement TranslationService in backend/src/services/translation_service.py:
  - [ ] T071a Parse chapter content to identify code blocks
  - [ ] T071b Call Claude API to translate non-code sections to Urdu
  - [ ] T071c Preserve code blocks in English
  - [ ] T071d Cache result with 30-day TTL
  - [ ] T071e Handle timeouts (return original if >25s)
- [ ] T072 Create Translation API endpoint in backend/src/api/routes/translate.py (POST /translate, requires auth, language parameter)
- [ ] T073 Implement TranslateButton component in frontend/src/components/TranslateButton.tsx:
  - [ ] T073a Toggle between English and Urdu
  - [ ] T073b Loading spinner during translation
  - [ ] T073c Fallback on timeout
  - [ ] T073d Display "Machine Translation" disclaimer
- [ ] T074 Add TranslateButton to chapter layout
- [ ] T075 Test translation accuracy (verify key terms translated correctly, no English bleeding through)
- [ ] T076 Test code preservation (verify Python/YAML snippets remain in English)
- [ ] T077 Test toggle functionality (Urdu → English → Urdu, verify content matches)
- [ ] T078 Test timeout (simulate slow API, verify fallback to English at 25s)
- [ ] T079 Test cache hit (translate same chapter twice, verify second is instant)
- [ ] T080 Configure Urdu font in frontend/src/styles/urdu.css (support right-to-left text if needed)

---

## Phase 8: User Story 5 - Query Book Content via RAG Chatbot (P2)

**Story Goal**: Users click floating chatbot, ask questions, receive answers grounded in book content with citations to chapters/sections.

**Independent Test**: Click chatbot → ask ROS2 question → verify answer cites Chapter 2. Ask off-topic question → verify polite decline.

**Tasks**:

- [ ] T081 Create ChaptterEmbedding model in backend/src/models/embedding.py (id, chapter_id, section, vector, metadata)
- [ ] T082 Create ChatMessage model in backend/src/models/chat.py (id, user_id, content, message_type, chapter_context, created_at)
- [ ] T083 Create Alembic migrations for ChapterEmbedding and ChatMessage tables
- [ ] T084 Implement EmbeddingService in backend/src/services/embedding_service.py:
  - [ ] T084a Batch embed all chapters using Claude/OpenAI embeddings API
  - [ ] T084b Store embeddings in Qdrant with metadata (chapter, section, title)
  - [ ] T084c Create search index for semantic retrieval
- [ ] T085 [P] Run embedding batch job: embed all 6 chapters and store in Qdrant (one-time setup)
- [ ] T086 Implement RAGService in backend/src/services/rag_service.py:
  - [ ] T086a Embed user question using same API
  - [ ] T086b Query Qdrant for top-K relevant passages (k=3)
  - [ ] T086c Format passages as context for Claude API
  - [ ] T086d Call Claude with system prompt (answer from book only, cite sources)
  - [ ] T086e Handle timeouts (return polite "I'm thinking..." message if >5s)
  - [ ] T086f Handle out-of-scope questions (detect, return "I can only answer about...")
- [ ] T087 Create RAG Chat API endpoint in backend/src/api/routes/chat.py (POST /chat, requires auth)
- [ ] T088 Implement Chatbot widget in frontend/src/components/Chatbot.tsx:
  - [ ] T088a Floating icon (bottom-right corner)
  - [ ] T088b Chat modal/sidebar on click
  - [ ] T088c Message input and send button
  - [ ] T088d Display bot responses with citations
  - [ ] T088d Loading spinner during response
  - [ ] T088e Conversation history in session
- [ ] T089 Implement useChat hook in frontend/src/hooks/useChat.ts (send message, track conversation)
- [ ] T090 Add Chatbot component to main layout in frontend/src/components/Layout.tsx
- [ ] T091 Style chatbot UI in frontend/src/styles/chatbot.css (floating position, responsive)
- [ ] T092 Test chatbot with ROS2 question (verify citation to Chapter 2)
- [ ] T093 Test chatbot with Gazebo question (verify citation to Chapter 3)
- [ ] T094 Test out-of-scope question (ask about unrelated topic, verify polite decline)
- [ ] T095 Test response time (measure latency, target <5s)
- [ ] T096 Test citation accuracy (verify all claims are in cited chapters)

---

## Phase 9: User Story 6 - Highlight and Chat on Selected Text (P3)

**Story Goal**: Highlight text → right-click → "Chat about this" → ask question with selected context included.

**Independent Test**: Select text → "Chat about this" → ask "Explain this" → verify response considers selection.

**Tasks**:

- [ ] T097 [US6] Add text selection listener in frontend chapter layout
- [ ] T098 [US6] Create context menu on right-click with "Chat about this" option
- [ ] T099 [US6] Modify Chatbot to accept selected text as initial context
- [ ] T100 [US6] Update RAGService to include selected text in prompt (prefix: "Here's selected text from the chapter: ...")
- [ ] T101 [US6] Test selection → chat → verify response elaborates on selected text
- [ ] T102 [US6] Test multiple selections in same session (verify each uses correct context)

---

## Phase 10: Polish & Cross-Cutting Concerns

Final integration, testing, documentation, and demo preparation.

- [ ] T103 Run full E2E test suite: signup → read chapters → personalize → translate → chat (manual)
- [ ] T104 Verify all success criteria: SC-001 through SC-012 (book chapters, signup time, latencies, concurrency, session, success rate, deployment, security, page load)
- [ ] T105 Run backend unit tests: `pytest tests/unit/` (target >80% coverage)
- [ ] T106 Run backend integration tests: `pytest tests/integration/` (auth flow, personalization, RAG)
- [ ] T107 Run frontend unit tests: `npm test` (Vitest, target >80% coverage)
- [ ] T108 Lint backend code: `black . && ruff check src/`
- [ ] T109 Lint frontend code: `prettier --write src/` && `eslint src/`
- [ ] T110 Run type checking: `mypy src/` (backend)
- [ ] T111 Audit dependencies: check for security vulnerabilities (`pip audit`, `npm audit`)
- [ ] T112 Create README.md with setup instructions, feature overview, deployment links, screenshots
- [ ] T113 Create CONTRIBUTING.md with development workflow and PR guidelines
- [ ] T114 Document API endpoints in backend/API.md or Swagger UI (OpenAPI schema)
- [ ] T115 Test on production URLs: verify all features work on GitHub Pages + Render
- [ ] T116 Create demo script / talking points for judges (5-minute walkthrough)
- [ ] T117 Performance test: measure book load time, personalization latency, chat response time (record metrics)
- [ ] T118 Accessibility audit: test with screen reader, verify Urdu text renders correctly
- [ ] T119 Mobile responsiveness test: verify Docusaurus and chatbot work on phone/tablet
- [ ] T120 Final code review: all PRs merged, no outstanding comments

---

## Dependencies & Execution Order

**Critical Path (Must Complete)**:
1. Phase 1 (Setup) → Phase 2 (Infrastructure) ← foundational
2. Phase 2 → Phase 3 (Book Content, US1) → Phase 5 (Deployment) [P1 stories in Week 1]
3. Phase 2 → Phase 4 (Auth, US2) → Phase 5 (Deployment) [P1 stories in Week 1]
4. Phase 3 + Phase 4 → Phase 6 (Personalization) [P2 in Week 2, depends on book + auth]
5. Phase 3 + Phase 4 → Phase 7 (Translation) [P2 in Week 2, depends on book + auth]
6. Phase 3 + Phase 4 → Phase 8 (RAG) [P2 in Week 2, depends on book + auth]
7. Phase 8 → Phase 9 (Text Selection Chat) [P3 in Week 3, depends on RAG]

**Parallel Opportunities**:
- T016-T021 (chapter authoring) can happen in parallel
- T030-T037 (backend auth models/services) can happen in parallel with T016-T021
- T038-T042 (frontend auth components) can happen in parallel with T030-T037
- T059-T068 (personalization) and T069-T080 (translation) can happen in parallel
- T081-T096 (RAG) can happen in parallel with personalization/translation

---

## Testing Checkpoints

| Checkpoint | Tasks | Verification |
|-----------|-------|--------------|
| **End of Week 1 (P1 Complete)** | T001-T058 | Book readable (T029), Auth working (T044-T046), Backend deployed (T051), Frontend deployed (T055) |
| **Mid Week 2 (P2 Started)** | T059-T096 | Personalize works (T065-T068), Translate works (T075-T080), RAG works (T092-T096) |
| **End of Week 2 (P2 Complete)** | T059-T096 | All P2 features tested, integrated, deployed |
| **End of Week 3 (Polish)** | T097-T120 | All E2E tests pass, success criteria met, demo ready, code reviewed |

---

## Success Metrics (from Spec)

Each task contributes to one or more success criteria:

- **SC-001**: Tasks T016-T021 (chapters), T027-T029 (rendering, deployment)
- **SC-002**: Tasks T030-T046 (signup flow)
- **SC-003**: Tasks T059-T068 (personalization, latency <25s)
- **SC-004**: Tasks T069-T080 (translation, latency <25s)
- **SC-005**: Tasks T081-T096 (RAG, latency <5s)
- **SC-006**: Tasks T086-T092 (citations, accuracy)
- **SC-007**: Load test under 100 concurrent users (backend scaling, Render free tier)
- **SC-008**: Tasks T033-T035 (24-hour session timeout)
- **SC-009**: Error handling in T061e, T071e, T086e (>90% success rate)
- **SC-010**: Tasks T047-T058 (deployment automation)
- **SC-011**: Tasks T031, T034-T035 (hashed tokens, no plain text)
- **SC-012**: Tasks T029, T055 (page load <3s, Docusaurus static + CDN)
