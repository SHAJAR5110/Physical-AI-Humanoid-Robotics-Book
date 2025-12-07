# Claude Code Rules

This file is generated during init for the selected agent.

You are an expert AI assistant specializing in Spec-Driven Development (SDD). Your primary goal is to work with the architext to build products.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps. 

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.

---

## 🎨 Project Modernization Updates (December 2025)

### Overview
Complete modernization of the Physical AI & Humanoid Robotics book project including open-source restructuring, landing page redesign, and professional branding.

### Key Changes & Updates

#### 1. **Open-Source Repository Structure**
- **Renamed**: `Ai-and-Humanoid-Robotics-Book/` → `book-source/`
- **Added**: MIT License to `book-source/LICENSE`
- **Updated**: Root README with new directory structure
- **Documentation**: Professional README in book-source/
- **Files Changed**: 37 files with 31,817 insertions
- **Git Commit**: `802ed62` (refactor: reorganize book platform to open-source structure)

#### 2. **Landing Page Modernization**
- **Hero Section**: Professional gradient text with cyan-to-purple colors
- **Book Description**: 5-6 line description of content and target audience
- **Book Overview**: 6 feature cards highlighting key benefits
- **Chapter Cards**: Enhanced with animations and dark mode support
- **Responsive Design**: Full mobile, tablet, desktop support
- **Dark Mode**: Complete light/dark theme compatibility
- **Files Modified**:
  - `book-source/src/pages/index.tsx` (117 lines)
  - `book-source/src/pages/index.module.css` (301 lines)
  - `book-source/src/components/HomepageFeatures/styles.module.css` (89 lines)
- **Git Commit**: `b7178f5` (feat: landing-page modernize homepage)

#### 3. **Modern Logo Design**
- **Type**: Geometric design representing AI/Robotics interconnection
- **Design**: 4 interconnected nodes (AI, Robotics, Physical, Integration)
- **Colors**: Gradient cyan (#0ea5e9) → purple (#8b5cf6)
- **Format**: SVG (scalable at any resolution)
- **Files**:
  - `book-source/static/img/logo.svg` (200x200)
  - `book-source/static/img/favicon.svg` (favicon version)
- **Usage**: Navigation bar, browser tabs, all pages
- **Git Commits**:
  - `8b2e9e8` (design: replace logo with modern geometric)
  - `6205576` (design: add favicon matching modern logo)

#### 4. **Hero Section with Background Image**
- **Background Image**: `hero-image.png` (5.8 MB, high quality)
- **Height**: 95vh (95% of viewport) for full-screen impact
- **Overlay**: Semi-transparent gradient (80-90% opacity)
  - Light mode: White/blue gradient overlay
  - Dark mode: Dark gradient overlay
- **Features**:
  - Parallax scrolling effect
  - Full-width background coverage
  - Centered text content (horizontal & vertical)
  - Professional, immersive design
  - Responsive on all devices
- **Removed**: Animated gradient orb and robot emoji
- **Git Commits**:
  - `cec6bb9` (fix: resolve hero image not displaying)
  - `6aeeddb` (feat: redesign with background image)
  - `08bb1d0` (feat: increase height to 95vh)

#### 5. **Contribution Guidelines**
- **File**: `CONTRIBUTING.md` (407 lines)
- **Covers**: Issue reporting, chapter standards, code examples, frontend/backend guides, workflow, testing

### Color Scheme
- **Primary Gradient**: Cyan (#0ea5e9) → Purple (#8b5cf6)
- **Accent Gradient**: Teal (#06b6d4) → Cyan (#0ea5e9)
- **Text**: Dark on light mode, light on dark mode
- **Shadows**: Subtle drop shadows with opacity

### Recent Git Commits
```
08bb1d0 feat(hero-section): increase height to 95vh for full-screen impact
cec6bb9 fix: resolve hero image not displaying - rename file and fix path
6aeeddb feat(hero-section): redesign with background image and remove orb
6205576 design: add favicon matching modern logo design
8b2e9e8 design: replace logo with modern geometric design
b7178f5 feat(landing-page): modernize homepage with custom design
802ed62 refactor: reorganize book platform to open-source structure
```

### Files Structure After Updates
```
book-source/
├── static/img/
│   ├── logo.svg                (200x200, modern geometric)
│   ├── favicon.svg             (favicon version)
│   ├── hero-image.png          (5.8 MB, full background)
│   └── ...other images
├── src/pages/
│   ├── index.tsx               (hero section with background)
│   └── index.module.css        (95vh height, gradients, responsive)
├── src/components/HomepageFeatures/
│   └── styles.module.css       (chapter cards with animations)
└── ...other project files
```

### Future Documentation Protocol
All future updates should be added to this CLAUDE.md section rather than creating separate documentation files. Use this format:

```markdown
#### N. **Feature Name**
- **Description**: Brief overview
- **Files**: Files modified/created
- **Git Commit**: Commit hash
- **Key Details**: Important implementation notes
```

### Deployment Strategy
- **Frontend (book-source)**: Deploy to Vercel (static site hosting)
- **Backend (FastAPI)**: Deploy to Railway, Heroku, or similar
- **Database**: PostgreSQL (Heroku Postgres or external service)
- **No Docker**: Not needed for Vercel deployment
- **Frontend Build**: `npm run build` in book-source/
- **Backend Requirements**: `requirements.txt` for pip install

### Deployment Checklist
- [ ] Build frontend: `cd book-source && npm run build`
- [ ] Connect book-source/ to Vercel
- [ ] Deploy backend to Railway/Heroku
- [ ] Update `REACT_APP_API_BASE_URL` environment variable
- [ ] Set up PostgreSQL database
- [ ] Configure environment variables (.env)
- [ ] Test endpoints integration
- [ ] Monitor logs in Vercel dashboard

### Sidebar Navigation Structure
The book uses a hierarchical sidebar navigation with collapsed categories:
- **Chapters**: 6 main chapters (collapsed by default)
- **Modules**: Each chapter contains 6-9 modules (topic links)
- **Navigation**: Click chapter to expand → select module to jump to section
- **Anchor Links**: Modules use `#heading-anchor` links for direct navigation
- **User Experience**: Clean sidebar, users choose what to explore

**Implementation Details** (book-source/sidebars.ts):
- `type: 'category'` for chapters with `collapsed: true`
- `type: 'link'` for modules pointing to section anchors
- Each module links directly to markdown heading in chapter
- Example: `href: '/docs/intro#what-is-physical-ai'`

**Chapter Structure:**
1. Chapter 1: Introduction to Physical AI (6 modules)
2. Chapter 2: ROS 2 Fundamentals (7 modules)
3. Chapter 3: Gazebo Simulation (7 modules)
4. Chapter 4: NVIDIA Isaac Platform (7 modules)
5. Chapter 5: Vision-Language-Action Models (6 modules)
6. Chapter 6: Capstone Project (9 modules)

### Current Project Status
✅ Open-source structure (book-source/)
✅ Modern landing page with hero section
✅ Professional geometric logo
✅ Full favicon integration
✅ 95vh full-screen hero with background
✅ Responsive design (mobile, tablet, desktop)
✅ Dark mode support throughout
✅ Professional contribution guidelines
✅ MIT licensing
✅ No Docker (uses Vercel for deployment)
✅ Sidebar navigation with 6 chapters and 42+ module links
✅ Chapter modules collapsed by default for clean UI
✅ Direct anchor links to section content
✅ All builds successful with zero errors
