# Physical AI Book Constitution

<!--
  Sync Impact Report (v1.0.0)
  Version: 0.0.0 → 1.0.0 (MAJOR: Initial constitution)
  Stage: Project initialization
  Modified Principles: None (first version)
  Added Sections: Core Principles, Content & Accessibility, Architecture, Governance
  Removed Sections: None
  Templates Updated: ✅ All
  Follow-up TODOs: None
-->

## Core Principles

### I. Content-First

High-quality, canonical educational material is the foundation of the Physical AI Book. Every chapter, example, and explanation must be technically accurate, pedagogically sound, and serve learner outcomes.

**Non-negotiable**:
- All content authored by domain experts or subject to peer review
- No placeholder or stub content shipped in production
- Examples must be runnable or clearly marked as pseudocode
- Explanations must connect concepts to learner goals

**Rationale**: The hackathon judges and long-term maintainers (Panaversity) will evaluate educational quality above all. Weak content undermines the entire project regardless of technical polish.

---

### II. Reusability (Modular Skills & Agents)

All tools, agents, and components must be designed for modular reuse across the project and beyond. Skills and agents created for one feature should be applicable to other features without modification.

**Non-negotiable**:
- Each skill (content-writer, translator, personalizer, rag-helper, code-generator) is a standalone unit
- Skills expose clear input/output contracts documented in comments
- Agents compose skills; agents themselves should be reusable (tutoring, researching, content-generation)
- No skill or agent contains hardcoded feature-specific logic

**Rationale**: Reusability reduces duplication, speeds up development within the hackathon sprint, and leaves a maintainable codebase for Panaversity. Skills are Panaversity's long-term asset.

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

**Rationale**: Urdu accessibility opens the project to Panaversity's regional learner base. Clear UX ensures adoption during the hackathon demo.

---

### V. Security & Privacy (BetterAuth, Minimal Telemetry)

User data is protected and collected minimally. Authentication is delegated to BetterAuth; no custom auth logic is built.

**Non-negotiable**:
- All user signup/signin flows use BetterAuth (no manual password handling)
- User profile stored in Neon Postgres; encrypted at rest and in transit
- Only background questions and language preference logged; no content views or behavior tracking
- No third-party analytics (except minimal error logging for ops)
- API keys and secrets stored in `.env`; never committed

**Rationale**: Judges and Panaversity expect privacy-first practices. Learner trust is paramount. BetterAuth is battle-tested and reduces surface area for security bugs.

---

## Architecture & Technology

### Tech Stack Decision

- **Frontend**: Docusaurus (React-based static site with dynamic client-side features)
- **Backend**: FastAPI (Python, fast iteration, great docs, integrates with Claude SDK)
- **Search/Retrieval**: Qdrant (vector database for RAG embeddings)
- **Database**: Neon Postgres (managed, free tier, scales with usage)
- **Auth**: BetterAuth (third-party, reduces custom code)
- **AI Services**: Claude API (model calls for personalization and tutoring)
- **Hosting**: Vercel (frontend), Railway or Heroku (backend, free tier)
- **Deployment**: GitHub Actions (CI/CD for content builds and API)

**Rationale**: Stack balances ease-of-use, free/affordable hosting for hackathon, and alignment with Panaversity's future operations.

### RAG Chatbot Architecture

1. **Ingestion**: On chapter publish, extract text and embed using Claude embeddings API
2. **Storage**: Store embeddings in Qdrant; metadata (chapter, author) in Postgres
3. **Query**: User question → embed → Qdrant search → retrieve context → Claude API chat with system prompt (include user profile and selected text)
4. **Constraints**: Context window max ~8k tokens; prioritize chapter relevance over global search

### Personalization Flow

1. **User Profile**: BetterAuth captures (background, experience, language)
2. **On-Demand Rewrite**: User clicks "Personalize" → Backend calls Claude Code agent/skill with (chapter text, user profile) → Stores result in cache (30-day TTL)
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
- **Content Review**: All educational content peer-reviewed by domain expert before merge
- **Linting**: Black (Python), Prettier (JS/JSON), ruff for linting
- **Type Checking**: mypy on all Python code (strict mode)
- **Pre-commit hooks**: Linting and format checks before commit

### Code Review & Approval

- All PRs require review from another team member
- Constitution compliance verified (e.g., reusable skills, no hardcoded secrets)
- Tests must pass and coverage thresholds met before merge
- Content changes require subject-matter expert sign-off

---

## Scope (In & Out)

### In Scope (Hackathon MVP)

- ✅ Docusaurus book with 3–5 core chapters on Physical AI & Humanoid Robotics
- ✅ Embedded RAG chatbot (Qdrant + FastAPI + Neon Postgres)
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

---

## Governance

### Amendment Procedure

1. **Proposal**: Any team member or maintainer proposes a change to a principle or section
2. **Discussion**: Changes are debated on their technical/business merit
3. **Ratification**: Unanimous or majority approval (depending on severity) from maintainers
4. **Version Bump**: Constitution version incremented per semantic versioning
5. **Propagation**: Related templates (spec, plan, tasks, commands) updated; all PRs re-validated

### Compliance Review

- **Pre-merge**: All PRs checked against current constitution (reusability, security, quality gates)
- **Quarterly**: Team reviews constitution against realized practices; amends if needed
- **Long-term**: Panaversity maintainers inherit constitution as part of project handoff

### Version Policy

Constitution follows semantic versioning:
- **MAJOR** (v2.0.0): Backward-incompatible principle removals or redefinitions
- **MINOR** (v1.1.0): New principle or materially expanded guidance
- **PATCH** (v1.0.1): Clarifications, wording, non-semantic refinements

---

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
