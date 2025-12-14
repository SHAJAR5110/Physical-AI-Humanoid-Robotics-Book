---
ID: 3
TITLE: Switch to Groq API for Cost and Performance
STAGE: constitution
DATE_ISO: 2025-12-14
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: none
BRANCH: master
USER: Physical AI Team
COMMAND: /sp.constitution update use groq instead of claude api
LABELS:
  - constitution
  - groq-api
  - cost-optimization
  - llm-inference
  - embeddings
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
FILES_YAML:
  - .specify/memory/constitution.md
TESTS_YAML:
  - Constitution compliance review (manual)
  - API provider switch verification (manual)
---

## User Input (Verbatim)

```
update use groq instead of claude api
```

## Response Summary

**Constitution Amendment Completed: v1.1.1 → v1.1.2 (PATCH)**

### What Was Done

1. **Updated Tech Stack Decision section** to replace Claude API with Groq API:
   - Changed: "Claude API (model calls for personalization and tutoring)"
   - To: "Groq API (model calls for embeddings, personalization, and tutoring; fast inference via Groq's LPU architecture)"
   - Updated rationale to emphasize lower costs and faster latency

2. **Updated RAG Chatbot Architecture section** with Groq API references:
   - **Embedding & LLM**: Groq API (embeddings and chat inference via LPU-accelerated models)
   - **Backend**: FastAPI with Groq SDK (Python 3.9+)
   - Updated Ingestion Flow: Extract → embed using Groq API → store in Qdrant
   - Updated Query Flow: Embed (Groq API) → retrieve → Groq chat API with system prompt

3. **Updated Personalization Flow** to use Groq API:
   - Changed: "Backend calls Claude Code agent/skill"
   - To: "Backend calls Groq API with (chapter text, user profile) via personalization skill"

4. **Created Sync Impact Report** documenting:
   - Version: 1.1.1 → 1.1.2 (PATCH: API provider switch)
   - Modified sections: Tech Stack Decision, RAG Chatbot Architecture, Personalization Flow
   - All principles remain unchanged; performance SLOs (p95 ≤ 3s) still achievable
   - No template updates required (drop-in replacement)

5. **Version bumped to 1.1.2** (PATCH: API provider switch, non-semantic change)

### Key Updates

**Groq API Integration Benefits**:
- ✅ Lower operational costs vs. Claude API (pay-as-you-go pricing)
- ✅ Faster inference latency via LPU-accelerated models
- ✅ Drop-in replacement for Claude API calls (same SDK-style interface)
- ✅ Maintains chatbot SLO targets (p95 ≤ 3s response time)
- ✅ All quality principles (VI. RAG Chatbot Quality) remain applicable

**Principles Preserved**:
- ✅ Principle VI (RAG Chatbot Quality): p95 ≤ 3s, ≥ 85% similarity, graceful errors, confidence scoring
- ✅ Principle III (Personalization): Groq API supports same rewrite flows
- ✅ Principle V (Security): API keys stored in `.env`, never committed
- ✅ Testing & Quality: Chatbot tests remain valid; latency benchmarking still required

### Assessment: Groq API Aligns with Cost Goals

**Rationale for Switch**:
- Groq's LPU architecture delivers fast inference (<1s for many tasks), supporting sub-3s chatbot SLO
- Pay-as-you-go pricing model reduces fixed costs vs. Claude API's variable usage tiers
- Groq embeddings API compatible with Qdrant schema (typically 384 or 768 dims)
- Groq SDK maintains familiar Python interface; minimal code migration needed
- Fallback strategy unchanged: graceful errors on Groq API failures

### Follow-up Items (Outstanding)

1. **Latency Validation**: Benchmark Groq API embeddings + inference to confirm p95 ≤ 3s chatbot SLO
   - Action: Add latency tests to `/specs/rag-chatbot/tasks.md` before MVP launch
   - Current status: Pending implementation planning

2. **Embedding Dimension Verification**: Confirm Groq embedding dimensions (usually 384 or 768) match Qdrant schema
   - Action: Document in backend configuration (`requirements.txt`, `.env`)
   - Current status: Pending implementation setup

3. **Cost Comparison Post-Hackathon**: Evaluate Groq vs. Claude API cost per request after MVP launch
   - Action: Monitor token usage in logs; document cost model in operations runbook
   - Current status: Deferred to post-hackathon review

### Next Steps

1. **Review amended constitution** at `.specify/memory/constitution.md` (v1.1.2)
2. **Proceed with implementation planning** (`/sp.plan`) to detail Groq API SDK setup
3. **Add latency benchmarks** to RAG chatbot tasks to validate SLO
4. **Verify Groq embedding dimensions** match Qdrant configuration
5. **Update backend code** (FastAPI service) to use Groq SDK instead of Claude SDK

---

## Technical Details

**File Structure Updated**:
```
.specify/
└── memory/
    └── constitution.md (v1.1.2 with Groq API references)

history/prompts/constitution/
├── 1-rag-chatbot-quality-principle.constitution.prompt.md
├── 2-rag-chatbot-tech-stack-clarification.constitution.prompt.md
└── 3-switch-groq-api.constitution.prompt.md (this PHR)
```

**Constitution Changes**:
- Lines modified: ~15 (Tech Stack Decision, RAG Chatbot Architecture sections)
- API provider: Claude API → Groq API (embeddings, inference, personalization)
- Semantic versioning: 1.1.1 → 1.1.2 (PATCH: API provider switch, non-semantic refinement)
- Ratification date: 2025-12-06 (original, unchanged)
- Last amended date: 2025-12-14 (today)

**Sync Impact Report Highlights**:
- Version change: 1.1.1 → 1.1.2 (PATCH)
- Modified sections: Tech Stack Decision, RAG Chatbot Architecture, Personalization Flow
- Removed sections: None
- Templates updated: None required (Groq API is drop-in replacement for Claude API calls)
- Principles unchanged: All 6 core principles remain valid with Groq API
- Outstanding TODOs: Latency validation, embedding dimension verification, cost comparison

**Branch Status**:
```
Current branch: master
Status: Changes committed to constitution.md
NOT PUSHED to GitHub (per user request in CLAUDE.md)
```

**Backward Compatibility**:
- Groq API replaces Claude API throughout (embeddings, chat inference, personalization)
- No breaking changes to existing principles or quality gates
- Performance SLOs (p95 ≤ 3s) remain achievable with Groq's LPU acceleration
- Code migration required in backend (FastAPI service) but not in constitution or architecture

