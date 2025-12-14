# Physical AI Book Constitution

<!--
  Sync Impact Report (v1.1.2)
  Version: 1.1.1 → 1.1.2 (PATCH: Replaced Claude API with Groq API for embeddings and LLM inference to reduce costs and improve latency)
  Stage: Constitution amendment for AI service provider switch
  Modified Principles: None (existing principles preserved; principle VI still applies with Groq)
  Modified Sections:
    - Tech Stack Decision (Claude API → Groq API)
    - RAG Chatbot Architecture (Claude API → Groq for embeddings and inference; verified performance targets remain achievable)
  Removed Sections: None
  Templates Updated: None required (principles and SLOs remain unchanged; Groq API is drop-in replacement for Claude API calls)
  Follow-up TODOs:
    - Validate: Groq API latency targets meet p95 ≤ 3s chatbot SLO (benchmark before MVP launch)
    - Verify: Groq embedding dimensions match Qdrant Free Tier schema (typically 384 or 768 dims; Qdrant supports variable)
    - Plan: Cost comparison post-hackathon (Groq pay-as-you-go vs. Claude API; evaluate long-term cost per request)
-->

## Core Principles

### I. Content-First

High-quality, canonical educational material is the foundation of the Physical AI Book. Every chapter, example, and explanation must be technically accurate, pedagogically sound, and serve learner outcomes.

**Non-negotiable**:
- All content authored by domain experts or subject to peer review
- No placeholder or stub content shipped in production
- Examples must be runnable or clearly marked as pseudocode
- Explanations must connect concepts to learner goals

**Rationale**: The hackathon judges and long-term maintainers will evaluate educational quality above all. Weak content undermines the entire project regardless of technical polish.

---

### II. Reusability (Modular Skills & Agents)

All tools, agents, and components must be designed for modular reuse across the project and beyond. Skills and agents created for one feature should be applicable to other features without modification.

**Non-negotiable**:
- Each skill (content-writer, translator, personalizer, rag-helper, code-generator) is a standalone unit
- Skills expose clear input/output contracts documented in comments
- Agents compose skills; agents themselves should be reusable (tutoring, researching, content-generation)
- No skill or agent contains hardcoded feature-specific logic

**Rationale**: Reusability reduces duplication, speeds up development within the hackathon sprint, and leaves a maintainable codebase.

---

### III. Personalization (User-Centric Adaptation)

Content and guidance adapt to each learner's background, experience level, and language preference. Personalization is not optional; it is core to the vision.

**Non-negotiable**:
- BetterAuth signup/signin captures user background (hardware, experience, language)
- "Personalize" button per chapter triggers Claude Code agent to rewrite content for that user profile
- "Translate" button per chapter outputs Urdu text using translator skill
- RAG chatbot context includes user profile to tailor responses
- All personalization is non-destructive (original content persists; personalized versions are generated on-demand)

**Rationale**: Learners have vastly different needs. A roboticist asking about simulation differs from a student learning fundamentals. Personalization is the hackathon's differentiator and core to "AI-native" positioning.

---

### IV. Accessibility (Urdu Translation & Clear UX)

The book must be accessible to Urdu-speaking learners and all learners regardless of device or connectivity. UX must be simple and intuitive.

**Non-negotiable**:
- All core chapters have Urdu translation available via button (translation skill + Neon Postgres storage)
- UX for personalization and translation buttons is discoverable and single-click
- Mobile-responsive Docusaurus design
- Links to simulations external or via notebook runners (not hosted locally in MVP)

**Rationale**: Urdu accessibility opens the project regional learner base. Clear UX ensures adoption during the hackathon demo.

---

### V. Security & Privacy (BetterAuth, Minimal Telemetry)

User data is protected and collected minimally. Authentication is delegated to BetterAuth; no custom auth logic is built.

**Non-negotiable**:
- All user signup/signin flows use BetterAuth (no manual password handling)
- User profile stored in Neon Postgres; encrypted at rest and in transit
- Only background questions and language preference logged; no content views or behavior tracking
- No third-party analytics (except minimal error logging for ops)
- API keys and secrets stored in `.env`; never committed

**Rationale**: Judges expect privacy-first practices. Learner trust is paramount. BetterAuth is battle-tested and reduces surface area for security bugs.

---

### VI. RAG Chatbot Quality (Performance, Reliability, User Experience)

The RAG chatbot MUST deliver fast, accurate, and reliable answers to learner questions with graceful error handling and user-friendly feedback.

**Non-negotiable**:
- **Performance**: p95 response latency ≤ 3 seconds (includes embedding, retrieval, and LLM inference)
- **Relevance**: Retrieved context must achieve ≥ 85% cosine similarity to query (relevance scoring)
- **Reliability**: Graceful fallback when embeddings/retrieval fail; users receive clear error messages (e.g., "I couldn't find relevant material; try rephrasing your question")
- **Confidence Scoring**: Chatbot MUST indicate confidence level (high/medium/low) based on retrieval similarity and answer quality
- **Context Window Management**: Limit context to ≤ 6k tokens; prioritize most relevant chunks; truncate gracefully if exceeding limit
- **Error Logging**: All chatbot errors (retrieval failures, LLM errors, timeout) logged with user consent; never log user queries without explicit opt-in
- **Loading States**: UI MUST show loading indicator during embedding/retrieval; estimated time-to-response when latency exceeds 2 seconds
- **Code Quality**: Chatbot service (backend + frontend) MUST maintain >80% test coverage (unit + integration tests); type hints on all Python functions (mypy strict mode)

**Rationale**: Chatbot is the primary user-facing feature for the hackathon. Poor performance or unreliable answers damage credibility. Fast responses (3s) keep learners engaged. Confidence scoring and error handling build trust. High code quality ensures maintainability post-hackathon.

---

## Architecture & Technology

### Tech Stack Decision

- **Frontend**: Docusaurus (React-based static site with dynamic client-side features)
- **Backend**: FastAPI (Python, fast iteration, great docs, integrates with Groq SDK)
- **Search/Retrieval**: Qdrant (vector database for RAG embeddings)
- **Database**: Neon Postgres (managed, free tier, scales with usage)
- **Auth**: BetterAuth (third-party, reduces custom code)
- **AI Services**: Groq API (model calls for embeddings, personalization, and tutoring; fast inference via Groq's LPU architecture)
- **Hosting**: Vercel (frontend), Railway or Heroku (backend, free tier)
- **Deployment**: GitHub Actions (CI/CD for content builds and API)

**Rationale**: Stack balances ease-of-use, free/affordable hosting for hackathon, and low-cost inference via Groq. Groq API offers faster latency and competitive pricing compared to Claude API, reducing operational costs while maintaining performance SLOs.

### RAG Chatbot Architecture

**Tech Stack**:
- **Embedding & LLM**: Groq API (embeddings and chat inference via LPU-accelerated models)
- **Vector Database**: Qdrant Cloud (Free Tier for MVP; scales post-hackathon)
- **Backend**: FastAPI with Groq SDK (Python 3.9+)
- **Agents/Orchestration**: OpenAI Agents API with Groq Toolkit or native FastAPI task queues (TBD based on agent complexity)
- **Database**: Neon Postgres for metadata and user context
- **Frontend**: React component embedded in Docusaurus book

**Retrieval Flow**:
1. **Ingestion**: On chapter publish, extract text chunks → embed using Groq API → store embeddings in Qdrant Cloud + metadata in Postgres
2. **Query**: User question → embed (Groq API) → semantic search in Qdrant → retrieve top-k (≥ 0.85 cosine similarity) → Groq chat API with system prompt (include user profile, chapter context, retrieved chunks)
3. **Constraints**: Context window max ~6k tokens; prioritize chapter relevance over global search; apply exponential backoff for API retries
4. **Retrieval Quality**: Return top-3 results ranked by cosine similarity; enforce ≥ 0.85 threshold before passing to LLM
5. **Fallback Strategy**: If retrieval fails or similarity < threshold, return graceful error (e.g., "I couldn't find relevant material. Try rephrasing or explore the chapter directly."); never hallucinate answers
6. **Qdrant Cloud Free Tier Constraints**: 1 GB storage, 10k embeddings limit; optimize chunk sizes to stay within limit during MVP; plan migration post-hackathon

### Personalization Flow

1. **User Profile**: BetterAuth captures (background, experience, language)
2. **On-Demand Rewrite**: User clicks "Personalize" → Backend calls Groq API with (chapter text, user profile) via personalization skill → Stores result in cache (30-day TTL)
3. **Caching**: Rewritten versions cached to avoid re-running for same user/chapter pair
4. **Fallback**: If personalization fails, serve original content

---

## Development Workflow & Quality Gates

### Spec-Driven Development (SDD)

All work follows Spec-Driven Development methodology as outlined in CLAUDE.md:

1. **Specification First**: Every feature begins with a spec in `/specs/<feature>/spec.md` (user stories, requirements, success criteria)
2. **Planning**: Architecture and technical plan in `/specs/<feature>/plan.md` (tech stack, structure, decisions)
3. **Tasks**: Testable, independent tasks in `/specs/<feature>/tasks.md` (organized by user story)
4. **Implementation**: Code changes referenced to tasks; single small commits per task
5. **Prompt History**: Every user interaction recorded in `/history/prompts/<feature>/` for traceability and learning

### Testing & Quality

- **Unit Tests**: All Python services in FastAPI backend must have >80% coverage (pytest)
- **Integration Tests**: RAG, personalization, and auth flows tested end-to-end
- **Chatbot-Specific Tests**:
  - Retrieval latency benchmarks (p95 ≤ 3s)
  - Relevance scoring tests (verify ≥ 85% similarity threshold enforcement)
  - Error handling tests (simulate Qdrant/LLM failures; verify graceful fallbacks)
  - Confidence scoring validation (ensure scores reflect answer quality)
- **Content Review**: All educational content peer-reviewed by domain expert before merge
- **Linting**: Black (Python), Prettier (JS/JSON), ruff for linting
- **Type Checking**: mypy on all Python code (strict mode)
- **Pre-commit hooks**: Linting and format checks before commit

### Code Review & Approval

- All PRs require review from another team member
- Constitution compliance verified (e.g., reusable skills, no hardcoded secrets, chatbot quality gates)
- Tests must pass and coverage thresholds met before merge
- Content changes require subject-matter expert sign-off
- Chatbot changes require verification of performance benchmarks and error handling

---

## Scope (In & Out)

### In Scope (Hackathon MVP)

- ✅ Docusaurus book with 3–5 core chapters on Physical AI & Humanoid Robotics
- ✅ Embedded RAG chatbot (Qdrant + FastAPI + Neon Postgres)
- ✅ Chatbot performance: p95 ≤ 3 seconds response time
- ✅ Chatbot reliability: Graceful error handling + confidence scoring
- ✅ Chatbot code quality: >80% test coverage, type hints, mypy strict mode
- ✅ BetterAuth signup/signin with user profile (background, experience, language)
- ✅ "Personalize" button per chapter (Claude Code agent/skill rewrites for user profile)
- ✅ "Translate" button per chapter (translator skill outputs Urdu)
- ✅ Reusable skills: content-writer, translator, personalizer, rag-helper, code-generator
- ✅ Agents: tutoring, researching, content-generation (compose skills)
- ✅ Free-tier hosting (Vercel + Railway/Heroku)

### Out of Scope (Post-Hackathon)

- ❌ Full simulation hosting (MVP links to external notebook runners or demo repos)
- ❌ Large-scale production infrastructure (multi-region, CDN, auto-scaling)
- ❌ Offline-first client (future; requires service worker + sync)
- ❌ Advanced analytics (future; requires privacy-preserving aggregation)
- ❌ Mobile native apps (future; Docusaurus web app is mobile-responsive)
- ❌ Chatbot personalization by learning style (future; requires longer interaction history)
- ❌ Multi-language chatbot responses (MVP: chatbot responds in English; chapters translate separately)

---

## Governance

### Amendment Procedure

1. **Proposal**: Any team member or maintainer proposes a change to a principle or section
2. **Discussion**: Changes are debated on their technical/business merit
3. **Ratification**: Unanimous or majority approval (depending on severity) from maintainers
4. **Version Bump**: Constitution version incremented per semantic versioning
5. **Propagation**: Related templates (spec, plan, tasks, commands) updated; all PRs re-validated

### Compliance Review

- **Pre-merge**: All PRs checked against current constitution (reusability, security, quality gates, chatbot SLOs)
- **Quarterly**: Team reviews constitution against realized practices; amends if needed
- **Long-term**: Maintainers inherit constitution as part of project handoff

### Version Policy

Constitution follows semantic versioning:
- **MAJOR** (v2.0.0): Backward-incompatible principle removals or redefinitions
- **MINOR** (v1.1.0): New principle or materially expanded guidance
- **PATCH** (v1.0.1): Clarifications, wording, non-semantic refinements

---

**Version**: 1.1.2 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-14
