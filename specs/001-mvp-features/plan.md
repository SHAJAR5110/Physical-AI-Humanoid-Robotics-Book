# Implementation Plan: Physical AI & Humanoid Robotics Book MVP

**Branch**: `001-mvp-features` | **Date**: 2025-12-06 | **Spec**: specs/001-mvp-features/spec.md
**Input**: Feature specification from `specs/001-mvp-features/spec.md` + Curriculum guidance on hardware/architecture

## Summary

Build an AI-native educational platform for Physical AI and Humanoid Robotics covering 6 chapters (Intro, ROS2, Gazebo, Isaac, VLA, Capstone). Core MVP delivers: (1) Docusaurus-based book with chapters and syntax-highlighted code, (2) BetterAuth signup/signin with user profile (OS, GPU, experience, robotics background), (3) RAG chatbot using Qdrant + FastAPI + Neon Postgres with citations to chapter content, (4) Per-chapter "Personalize" button (Claude API rewrite based on user profile), (5) Per-chapter "Translate to Urdu" button (Claude API translation, cached), (6) Deployment to GitHub Pages (frontend) + Render/Heroku (backend).

Technical approach: Frontend is static Docusaurus site with client-side chatbot widget and personalization/translation buttons; backend is FastAPI microservice exposing REST endpoints for auth, personalization, translation, and RAG chat; vector embeddings pre-generated from chapter content and stored in Qdrant; user profiles and sessions stored in Neon Postgres with BetterAuth handling credentials.

## Technical Context

**Language/Version**: Python 3.11 (backend), Node.js 18+ (frontend/Docusaurus), TypeScript (UI logic)

**Primary Dependencies**:
- **Backend**: FastAPI, Pydantic, SQLAlchemy, python-dotenv
- **Frontend**: Docusaurus 3, React 18, Tailwind CSS
- **Auth**: BetterAuth (third-party service)
- **AI/ML**: Claude API (personalization, translation, RAG), OpenAI embeddings API or Claude embeddings
- **Vector DB**: Qdrant (self-hosted or managed instance)
- **SQL DB**: Neon Postgres (managed PostgreSQL)
- **Testing**: pytest (backend), Vitest (frontend)

**Storage**:
- Neon Postgres: User profiles, sessions, cached personalized content, chat history
- Qdrant: Chapter embeddings (vectors), metadata for RAG retrieval
- GitHub Pages: Static Docusaurus site (HTML/CSS/JS)

**Testing**: pytest (unit/integration on backend), Vitest (unit on frontend), manual E2E (auth flow, personalization, RAG)

**Target Platform**: Web (responsive), Linux for backend deployment, no mobile app in MVP

**Project Type**: Full-stack web application (frontend + backend)

**Performance Goals**:
- Book pages load in <3 seconds (SC-012)
- Chatbot responses in <5 seconds average (SC-005)
- Personalization returns in 10–25 seconds (SC-003)
- Translation returns in 10–25 seconds (SC-004)
- Support 100 concurrent users (SC-007)

**Constraints**:
- Hackathon timeline: 2-3 weeks
- Free/low-cost hosting (GitHub Pages, Render free tier, Heroku free tier)
- Latency budget 10–25s for LLM-driven features (acceptable for async operations)
- Free-tier vector and database services (Qdrant free, Neon free)

**Scale/Scope**:
- ~100K words of content across 6 chapters
- ~100 concurrent users during demo
- 6 key entities (User, Chapter, Session, Personalized Content, Chat Message, Embedding)
- 33 functional requirements, 12 success criteria
- 7 prioritized user stories (P1/P2/P3)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| **Content-First** | ✅ PASS | All chapter content authored by domain experts or peer-reviewed; spec requires runnable examples and pedagogically sound explanations |
| **Reusability (Modular Skills)** | ✅ PASS | Personalization, translation, RAG, and code generation implemented as reusable skills; no hardcoded feature-specific logic; clear input/output contracts |
| **Personalization** | ✅ PASS | BetterAuth captures user background; Personalize button triggers Claude API rewrite; non-destructive (original persists) |
| **Accessibility (Urdu + UX)** | ✅ PASS | Translate button outputs Urdu via Claude API; single-click discoverable UX; Docusaurus is mobile-responsive |
| **Security & Privacy** | ✅ PASS | BetterAuth handles auth (no custom password logic); Neon Postgres encrypted; minimal data collection (only profile + language); API keys in `.env` (never committed) |
| **Testing & Quality** | ✅ PASS | pytest for backend (>80% coverage target); Vitest for frontend; manual E2E for auth/personalization/RAG; pre-commit hooks for linting |
| **Code Review** | ✅ PASS | All PRs require review; constitution compliance verified; tests and coverage must pass before merge |

**Gate Status**: ✅ **PASS** — Plan aligns with all constitution principles. Proceeding to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/001-mvp-features/
├── spec.md                  # Feature specification (created, requirements & user stories)
├── plan.md                  # This file (architectural decisions & technical context)
├── research.md              # Phase 0 output (LLM choice, embedding strategy, hosting)
├── data-model.md            # Phase 1 output (entity definitions, relationships, schema)
├── contracts/               # Phase 1 output (OpenAPI/REST API definitions)
│   ├── auth.yaml
│   ├── personalization.yaml
│   ├── translation.yaml
│   ├── rag.yaml
│   └── admin.yaml
├── quickstart.md            # Phase 1 output (setup & first run guide)
├── checklists/
│   └── requirements.md       # Quality validation checklist
└── tasks.md                 # Phase 2 output (/sp.tasks command - TO BE CREATED)
```

### Source Code (repository root)

**Structure Decision**: Option 2 - Full-stack web application with separate frontend (Docusaurus) and backend (FastAPI microservice).

```text
# Repository root structure
.github/
├── workflows/               # CI/CD pipelines for testing & deployment
│   ├── backend-tests.yml
│   ├── frontend-build.yml
│   └── deploy.yml

backend/
├── src/
│   ├── main.py              # FastAPI application entry point
│   ├── config.py            # Configuration (env vars, settings)
│   ├── models/
│   │   ├── user.py          # User, Session, Profile models
│   │   ├── chapter.py       # Chapter, Embedding models
│   │   └── cache.py         # Personalized content cache model
│   ├── services/
│   │   ├── auth_service.py  # BetterAuth integration
│   │   ├── rag_service.py   # RAG with Qdrant + Claude
│   │   ├── personalization_service.py  # Claude API personalization
│   │   ├── translation_service.py      # Claude API translation (Urdu)
│   │   └── embedding_service.py        # Chapter embedding generation
│   ├── api/
│   │   ├── __init__.py
│   │   ├── routes/
│   │   │   ├── auth.py      # POST /signup, POST /signin, POST /signout
│   │   │   ├── chapters.py  # GET /chapters, GET /chapters/{id}
│   │   │   ├── personalize.py   # POST /personalize (chapter_id, user_id)
│   │   │   ├── translate.py     # POST /translate (chapter_id, language)
│   │   │   └── chat.py          # POST /chat (question, user_id, context)
│   │   └── middleware.py    # CORS, error handling, auth middleware
│   ├── db/
│   │   ├── connection.py    # SQLAlchemy + Neon Postgres
│   │   ├── migrations/      # Alembic for schema versioning
│   │   └── queries.py       # Reusable DB queries
│   └── utils/
│       ├── cache.py         # Redis or in-memory caching for personalized content
│       ├── logging.py       # Structured logging
│       └── validators.py    # Pydantic validators

tests/
├── unit/
│   ├── test_auth_service.py
│   ├── test_rag_service.py
│   ├── test_personalization.py
│   └── test_translation.py
├── integration/
│   ├── test_auth_flow.py
│   ├── test_personalization_flow.py
│   └── test_rag_flow.py
└── fixtures/
    ├── sample_chapters.py
    └── mock_responses.py

frontend/
├── docusaurus.config.js      # Docusaurus config (theme, plugins, sidebar)
├── src/
│   ├── pages/
│   │   ├── index.tsx        # Landing page
│   │   └── books.tsx        # Book catalog/navigation
│   ├── components/
│   │   ├── Chatbot.tsx      # Floating RAG chatbot widget
│   │   ├── PersonalizeButton.tsx
│   │   ├── TranslateButton.tsx
│   │   ├── UserProfile.tsx  # Display user background
│   │   └── CodeBlock.tsx    # Syntax-highlighted code blocks
│   ├── hooks/
│   │   ├── useAuth.ts       # Auth state management
│   │   ├── useChat.ts       # RAG chat hook
│   │   └── usePersonalization.ts
│   ├── services/
│   │   ├── api.ts           # HTTP client (axios/fetch)
│   │   ├── auth.ts          # BetterAuth integration
│   │   └── rag.ts           # Chat service
│   ├── styles/
│   │   └── custom.css       # Tailwind overrides
│   └── constants/
│       └── config.ts        # API URLs, feature flags

docs/
├── intro.md                 # Chapter 1: Introduction to Physical AI
├── ros2.md                  # Chapter 2: ROS 2 Fundamentals
├── gazebo.md                # Chapter 3: Gazebo Simulation
├── isaac.md                 # Chapter 4: NVIDIA Isaac Platform
├── vla.md                   # Chapter 5: Vision-Language-Action Models
└── capstone.md              # Chapter 6: Capstone Project

.env.example                 # Environment variables template
.gitignore
README.md
```

**Rationale**:
- **Backend**: Micro service pattern allows independent scaling and reuse of skills/services for future features
- **Frontend**: Docusaurus for static book content + React components for interactive widgets (chatbot, buttons)
- **Separation**: Enables parallel development, independent testing, and deployment to different hosting platforms


## Complexity Tracking

No constitution violations detected. All architectural decisions align with principles.

---

## Phase 0: Research & Unknowns (To be generated)

### Decisions to Document (research.md)

1. **LLM Choice for Personalization & Translation**: Claude API (Haiku/Sonnet for cost-efficiency)
2. **Embedding Strategy**: Claude embeddings API (consistency with personalization) or OpenAI embeddings (cost trade-off)
3. **Hosting Backend**: Render free tier (preferred) vs Heroku (alternative) vs AWS
4. **Caching Strategy**: In-memory cache (fast, simple) vs Redis (scalable, complex)
5. **RAG Retrieval**: Semantic search in Qdrant with top-K selection + re-ranking

**Output**: `specs/001-mvp-features/research.md` (Phase 0)

---

## Phase 1: Design & Contracts (To be generated)

### Data Model (data-model.md)

Key entities to define:
- **User**: id, email, name, os, gpu, experience_level, robotics_background, created_at, updated_at
- **Session**: session_id, user_id, token_hash, expires_at, created_at
- **Chapter**: id, title, content (markdown), metadata (author, version), embedding_id, created_at
- **Personalized Content**: id, user_id, chapter_id, personalized_text, created_at, expires_at (cache TTL)
- **Chat Message**: id, user_id, content, message_type (user/assistant), chapter_context, created_at
- **Chapter Embedding**: id, chapter_id, vector (dimensions: 1536 for OpenAI, variable for Claude), metadata

**Output**: `specs/001-mvp-features/data-model.md` (Phase 1)

### API Contracts (contracts/)

Endpoints:

| Method | Path | Purpose | Auth | Response |
|--------|------|---------|------|----------|
| POST | `/auth/signup` | Register new user | None | { user_id, session_token, profile } |
| POST | `/auth/signin` | Login | Email + password | { user_id, session_token } |
| GET | `/chapters` | List all chapters | Optional | [ { id, title, summary } ] |
| GET | `/chapters/{id}` | Get chapter content | Optional | { id, title, content, metadata } |
| POST | `/personalize` | Generate personalized content | Session | { personalized_text, cached_at } |
| POST | `/translate` | Translate to Urdu | Session | { translated_text, language } |
| POST | `/chat` | RAG chat | Session | { response, citations, sources } |

**Output**: `specs/001-mvp-features/contracts/` (Phase 1)

### Quick Start Guide (quickstart.md)

Setup instructions:
1. Clone repository
2. Backend: `pip install -r requirements.txt`, set `.env`, run migrations, start server
3. Frontend: `npm install`, `npm run build`, deploy to GitHub Pages
4. Configuration: Qdrant endpoint, Neon connection string, Claude API key, BetterAuth credentials

**Output**: `specs/001-mvp-features/quickstart.md` (Phase 1)

---

## Phase 2: Task Breakdown (To be generated by /sp.tasks)

Phase 2 will decompose the 7 user stories into:
- **P1 Stories** (Book Content, Auth, Deployment): Critical path, 8-10 tasks each
- **P2 Stories** (Personalization, Translation, RAG): Dependent path, 6-8 tasks each
- **P3 Stories** (Text Selection Chat): Enhancement, 4 tasks

Estimated task count: ~40-50 tasks total

**Output**: `specs/001-mvp-features/tasks.md` (Phase 2, via /sp.tasks command)

---

## Key Architectural Decisions

### 1. LLM for Personalization & Translation
**Decision**: Use Claude API (Haiku for cost, Sonnet for quality if budget allows)
**Rationale**: Familiar with Claude Code ecosystem; good cost/quality trade-off; consistent with RAG
**Alternative**: OpenAI (GPT-3.5-turbo) — rejected for now due to cost; can switch post-MVP

### 2. Vector Embeddings
**Decision**: Claude embeddings API (consistency) or OpenAI embeddings (if cost concerns)
**Rationale**: Embed chapters at publish time (batch job), store in Qdrant, query at runtime
**Alternative**: Generate embeddings on-the-fly — rejected due to latency

### 3. Backend Hosting
**Decision**: Render free tier (simpler onboarding)
**Rationale**: Free tier suitable for ~100 concurrent users; auto-deploys from GitHub
**Alternative**: Heroku (legacy platform) or AWS (more complex setup)

### 4. Caching Strategy
**Decision**: In-memory cache (Redis-like) for personalized content (30-day TTL)
**Rationale**: Avoid re-personalizing same user/chapter; Neon Postgres stores metadata
**Alternative**: Direct DB storage — rejected due to personalization latency

### 5. Frontend Deployment
**Decision**: GitHub Pages (static Docusaurus + JavaScript)
**Rationale**: Free, automatic builds from main branch, perfect for static site
**Alternative**: Vercel (simpler but less visibility)

---

## Success Metrics (from Spec)

This plan enables all 12 success criteria:

- SC-001: Book chapters fully authored (6 chapters) ✅
- SC-002: Signup in <2 minutes (simple form, BetterAuth) ✅
- SC-003: Personalization in 10–25s (Claude API latency budget) ✅
- SC-004: Translation in 10–25s (Claude API latency budget) ✅
- SC-005: Chatbot response in <5s (Qdrant retrieval + Claude) ✅
- SC-006: 100% citation accuracy (RAG grounding in spec + tests) ✅
- SC-007: 100 concurrent users (FastAPI + Neon scaling) ✅
- SC-008: 24-hour session timeout (BetterAuth + database) ✅
- SC-009: 90%+ success rate (error handling + retries) ✅
- SC-010: Deployment in <30 min (documented, automated) ✅
- SC-011: Zero plain-text passwords (BetterAuth hashing) ✅
- SC-012: 95% pages load in <3s (Docusaurus static + CDN) ✅

---

## Next Steps

1. **Phase 0**: Generate `research.md` resolving unknowns (LLM, embeddings, hosting)
2. **Phase 1**: Generate `data-model.md`, `contracts/`, `quickstart.md` with concrete schemas and APIs
3. **Phase 2**: Run `/sp.tasks` to break down 7 user stories into ~40-50 testable tasks
4. **Implementation**: Begin with P1 stories (Book Content, Auth, Deployment) for foundation
