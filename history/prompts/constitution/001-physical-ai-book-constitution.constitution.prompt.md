---
id: 001
title: Physical AI Book Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: team
command: /sp.constitution
labels: ["constitution","project-init","spec-driven-development"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```
Project name: Physical AI Book

Purpose / Mission:
Create an AI-native Docusaurus textbook for Physical AI & Humanoid Robotics that adapts content to a learner's background and language (Urdu) and embeds a RAG chatbot that answers using book content and selected text.

Vision:
An interactive, personalized learning portal where students with varied hardware and experience can learn Physical AI, run simulations and receive targeted guidance from agent skills.

Principles:

Content-first: high-quality, canonical educational material.

Reusability: skills and agents must be modular and reusable.

Personalization: adapt content to user profile.

Accessibility: Urdu translation & clear UX.

Security & privacy: use BetterAuth; store minimal user telemetry.

Stakeholders:

Hackathon team (you) — implementers

Judges / Panaversity — evaluators

Learners (students) — end users

Panaversity maintainers — long-term owners

Scope (what's in):

Docusaurus book with core chapters.

Embedded RAG chatbot (Qdrant + FastAPI + Neon Postgres).

BetterAuth signup/signin & user profile (background questions).

Personalize button per chapter (Claude Code agent / skill rewrites).

Urdu translation button per chapter.

Reusable skills (content-writer, translator, personalizer, rag-helper, code-generator).

Agents for tutoring, researching, and content generation.

Out of scope (for hackathon MVP):

Full simulation hosting (we will link to demo notebooks/cloud runs).

Large-scale production infra (we use free tiers).
```

## Response snapshot

Created Physical AI Book Constitution (v1.0.0) capturing project principles (Content-First, Reusability, Personalization, Accessibility, Security & Privacy), tech stack (Docusaurus, FastAPI, Qdrant, Neon Postgres, BetterAuth), and governance structure aligned with Spec-Driven Development methodology. Constitution includes in-scope hackathon MVP features and out-of-scope future work, with clear quality gates (>80% test coverage, peer review, content expert sign-off) and semantic versioning policy.

## Outcome

- ✅ Impact: Project governance established; constitution serves as authoritative source for all development decisions and compliance reviews
- 🧪 Tests: Constitution syntax validated; no placeholders remain; version policy defined (MAJOR/MINOR/PATCH)
- 📁 Files: `.specify/memory/constitution.md` (created v1.0.0), `history/prompts/constitution/001-physical-ai-book-constitution.constitution.prompt.md` (PHR)
- 🔁 Next prompts: `/sp.specify` (create feature specs), `/sp.plan` (architecture planning), `/sp.tasks` (task generation)
- 🧠 Reflection: Constitution balances aggressive hackathon timeline with long-term maintainability; principles are testable and enforceable via PR review; tech stack chosen for free tiers and integration with Claude API ecosystem

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS (no unresolved placeholders, version coherent, governance clear)
- Prompt variant (if applicable): null
- Next experiment: none required; constitution ready for implementation
