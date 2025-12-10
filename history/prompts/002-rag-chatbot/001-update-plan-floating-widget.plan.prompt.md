---
ID: "001"
TITLE: "Update plan floating widget architecture"
STAGE: "plan"
FEATURE: "002-rag-chatbot"
DATE_ISO: "2025-12-09"
SURFACE: "agent"
MODEL: "claude-haiku-4-5-20251001"
BRANCH: "chatbot-constitution-v1.1.0"
USER: "user"
COMMAND: "/sp.plan"
LABELS:
  - "architecture"
  - "floating-widget"
  - "frontend-design"
  - "docusaurus-integration"
LINKS:
  SPEC: "/specs/002-rag-chatbot/spec.md"
  PLAN: "/specs/002-rag-chatbot/plan.md"
  ADR: "null"
  PR: "null"
FILES_YAML:
  - "specs/002-rag-chatbot/plan.md"
TESTS_YAML: []
---

## PROMPT_TEXT

```
@specs\002-rag-chatbot\ update plan that we have already frontend means we already have book @book-source\ our task is to add just chatbot in the bottom left corner. which gives the answer from this book nothing else.
```

## RESPONSE_TEXT

Updated the RAG Chatbot implementation plan to reflect the floating widget architecture in the bottom-left corner of all book pages. Key changes:

**Architectural Changes**:
- Chatbot is now a **floating widget** (not a dedicated page)
- Always-visible toggle button (💬) in bottom-left corner
- Collapsible card that opens on click
- Auto-opens when user selects text on a chapter page
- Sticky positioning (remains visible during scroll)
- Mobile-responsive (scales down on ≤640px screens)

**Frontend Component Updates**:
- `ChatBot.tsx`: Added `isOpen` state, toggle button, and widget-specific styling
- `ChatBot.module.css`: Replaced full-page layout with floating widget CSS (fixed positioning, animations, responsive design)
- Gradient button with cyan-to-purple color scheme matching book branding

**Docusaurus Integration**:
- New Root wrapper component at `book-source/src/theme/Root.tsx`
- Global widget rendering on all pages (docs, homepage)
- No manual inclusion needed on each page
- Uses Docusaurus swizzle mechanism for seamless integration

**Design Details**:
- Button: 56px circular with gradient, shadow, hover effects
- Widget: 380px width (desktop), responsive on mobile, max-height 600px with scrollable response section
- Header: Gradient background with close button
- Compact input (2 rows), sources as inline links, minimal feedback buttons
- Loading spinner, error states, confidence indicators all optimized for compact widget

**Acceptance Criteria Updated**:
- Widget architecture approved and documented
- All constitutional principles verified (content-first, reusable, accessible, secure, RAG quality)
- Quality gates maintained (type safety, 80% coverage, p95 ≤ 3s latency)

Plan is now ready for task breakdown and implementation.

---

## OUTCOME

✅ Plan successfully updated for floating widget architecture
✅ All sections refreshed to reflect bottom-left corner placement
✅ Frontend component and CSS redesigned for floating widget
✅ Docusaurus integration method specified
✅ Mobile responsiveness confirmed (640px breakpoint)
✅ Acceptance criteria updated and verified

## NEXT_STEPS

- Run `/sp.tasks` to generate testable implementation tasks
- Backend implementation (FastAPI services)
- Frontend implementation (React widget + CSS)
- Docusaurus Root wrapper integration
- Testing and deployment
