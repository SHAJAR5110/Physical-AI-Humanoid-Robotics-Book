# RAG Chatbot for Physical AI Textbook - Feature Specification

**Feature ID**: 002-rag-chatbot
**Status**: Specification (Updated for Constitution v1.1.1 Alignment)
**Last Updated**: 2025-12-13
**Owned By**: Physical AI Team
**Constitutional Alignment**: Constitution v1.1.1 (RAG Chatbot Architecture clarified with Claude API, Qdrant Cloud Free Tier, FastAPI + Claude SDK, OpenAI Agents API reference)

---

## Executive Summary

We are building a Retrieval-Augmented Generation (RAG) chatbot embedded in the Physical AI & Humanoid Robotics textbook that answers student questions about course content in real-time. The chatbot enables students to ask questions about specific chapters without reading the entire textbook, reducing cognitive load and improving learning outcomes.

---

## Why This Matters

- **Improves Learning Outcomes**: Students can ask follow-up questions instantly without leaving the book
- **Reduces Cognitive Load**: No need to search external resources or read entire chapters for clarity
- **Provides Instant Feedback**: Answers appear within 3 seconds with source citations powered by Claude API
- **Scales Affordably**: Free-tier services (Qdrant Cloud, Render) + Claude API support MVP without infrastructure overhead
- **Builds on Existing Platform**: Integrates seamlessly with Docusaurus-deployed textbook
- **Reliable & Resilient**: Claude API with exponential backoff ensures graceful error handling; Qdrant Cloud Free Tier sufficient for MVP with optimized chunking strategy

---

## Core Features

### Feature 1: Context-Aware Question Answering

**Description**: Students ask questions about course concepts; the system retrieves relevant sections and synthesizes answers.

**User Flow**:
1. Student reads chapter on "ROS 2 Nodes"
2. Student asks: "What is the difference between topics and services?"
3. System retrieves relevant passages from the chapter
4. LLM synthesizes answer from retrieved content
5. Answer appears with source references (e.g., "From Module 2.3: ROS 2 Topics and Services")

**Acceptance Criteria**:
- Answer appears within 3 seconds (p95 latency)
- Retrieved content has ≥ 85% relevance to query (cosine similarity > 0.85)
- Answer includes at least one source reference with chapter/module link
- Graceful error message if no relevant content found (e.g., "I couldn't find information on this topic")

---

### Feature 2: Text Selection Query

**Description**: Students select text from a chapter and ask questions specifically about that excerpt.

**User Flow**:
1. Student selects a paragraph about "ROS 2 Services"
2. Student clicks "Ask about this" button
3. Selection is sent with the question as context
4. System uses selected text as primary context for retrieval
5. Answer focuses on the selected content with minimal out-of-context information

**Acceptance Criteria**:
- Selected text is automatically included in API request
- System prioritizes selected text in answer synthesis
- UI clearly indicates that a selection is being used
- Works on mobile devices (text selection on 320px+ width)

---

### Feature 3: Conversation History

**Description**: Students can ask follow-up questions and maintain context across multiple exchanges.

**User Flow**:
1. Student asks: "What is a ROS 2 node?"
2. Student receives answer with sources
3. Student asks follow-up: "How do I create one?"
4. System maintains conversation context to understand the follow-up

**Acceptance Criteria**:
- Follow-up questions reference previous context (optional for MVP; marked as optional in scope)
- Conversation history is cleared when user navigates away from book
- No conversation history is stored long-term (privacy-first)

**Note**: This feature is optional for MVP; core focus is on single-turn Q&A.

---

### Feature 4: Source Attribution

**Description**: Every answer includes citations to the original book content.

**User Flow**:
1. Chatbot returns answer to student question
2. Answer includes inline source citations (e.g., "According to Chapter 2, Module 3...")
3. Student can click source link to jump to relevant chapter section
4. Builds trust and enables verification

**Acceptance Criteria**:
- Every answer includes at least one source reference
- Source references include chapter and module name
- Source references are clickable links to relevant section anchors
- Source references are accurate (matched to retrieval results)

---

## User Scenarios & Testing

### Scenario 1: Student Clarifies Unfamiliar Concept

**Actor**: First-year student learning ROS 2 for the first time

**Setup**: Student is reading Chapter 2 on "ROS 2 Fundamentals"

**Flow**:
1. Student reads "Nodes use topics to publish/subscribe messages"
2. Student is unfamiliar with pub/sub pattern
3. Student types: "What is publish-subscribe in ROS 2?"
4. System retrieves sections explaining pub/sub pattern
5. System returns: "In ROS 2, publish-subscribe (pub/sub) is a messaging pattern where nodes publish messages to topics and other nodes subscribe to receive those messages. This decouples node communication..."
6. Answer includes source: "From Chapter 2, Module 1: ROS 2 Nodes, Topics, and Services"
7. Student clicks source link and jumps to relevant section
8. Student's confusion is resolved; continues reading

**Acceptance**: Answer arrives within 3 seconds; student doesn't need external resources

---

### Scenario 2: Student Asks About Selected Text

**Actor**: Intermediate student reviewing advanced concepts

**Setup**: Student is reading Chapter 4 on "NVIDIA Isaac Platform"

**Flow**:
1. Student selects paragraph: "The IsaacLab simulator uses PyTorch for differentiable physics..."
2. Student asks: "Why use PyTorch instead of other physics engines?"
3. System receives selected text + question
4. System prioritizes selected text in retrieval
5. System returns contextualized answer about PyTorch in Isaac
6. Answer acknowledges the selected context: "Regarding the approach you selected..."

**Acceptance**: System clearly uses the selected text; answer is more focused than without selection

---

### Scenario 3: Student Provides Feedback on Answer

**Actor**: Student wanting to improve chatbot quality

**Setup**: Student received an answer to their question

**Flow**:
1. Student sees thumbs-up/thumbs-down buttons below answer
2. Student clicks thumbs-down (answer wasn't helpful)
3. System logs feedback with question and answer
4. Feedback is used to improve future iterations (future feature)

**Acceptance**: Feedback buttons are visible; clicks are logged (if user consents)

---

## Functional Requirements

### FR-1: Question Processing Pipeline

The system MUST:
- Accept text questions from students
- Embed questions using Claude API embeddings (or sentence-transformers as fallback)
- Query Qdrant Cloud vector database for semantic matches
- Return top-3 relevant passages ranked by cosine similarity
- Pass retrieved passages to Claude API for answer synthesis
- Apply exponential backoff for API retries on transient failures
- Generate responses within 3 seconds (p95)

**Testable**: Measure latency for 100 random questions; verify p95 ≤ 3 seconds; verify exponential backoff is applied

---

### FR-2: Relevance Filtering

The system MUST:
- Apply minimum cosine similarity threshold of 0.85 before passing to LLM
- Return "couldn't find relevant content" error if no passages exceed threshold
- Log rejected queries for analysis (with user consent)

**Testable**: Verify threshold enforcement by testing with off-topic queries

---

### FR-3: Source Attribution

The system MUST:
- Track source metadata (chapter name, module name, section anchor) for each passage
- Include source reference in every answer
- Format source as: "From [Chapter Name], [Module Name]: [Section]"
- Generate clickable links to chapter anchors (e.g., `/docs/chapter-2#ros-2-topics`)

**Testable**: Check that 100% of answers include formatted sources; verify links are valid

---

### FR-4: Error Handling

The system MUST:
- Return user-friendly error messages when:
  - No relevant content found: "I couldn't find information on this topic. Try rephrasing your question or browse the chapters."
  - Vector database unavailable: "Chat service is temporarily unavailable. Please try again in a moment."
  - LLM service unavailable: "I'm having trouble generating a response. Please try again."
- Never expose stack traces or technical errors to users
- Log all errors for debugging (with appropriate detail level)

**Testable**: Simulate service failures; verify error messages are user-friendly

---

### FR-5: Confidence Scoring

The system MUST:
- Calculate confidence as function of:
  - Maximum cosine similarity of retrieved passages (0-1)
  - Number of relevant passages found (1-5)
  - Answer quality checks (presence of key terms from query)
- Return confidence level as: high (≥0.8), medium (0.5-0.8), low (<0.5)
- Display confidence indicator in UI (e.g., "This answer has high confidence")

**Testable**: Verify confidence scores correlate with manual quality assessment

---

### FR-6: Context Window Management

The system MUST:
- Limit total context sent to LLM to ≤ 6k tokens
- Prioritize most relevant passages (ordered by cosine similarity)
- Truncate passages gracefully if exceeding limit
- Include system prompt explaining context and asking for concise answers

**Testable**: Count tokens in API requests; verify no requests exceed 6k

---

### FR-7: Response Format

The system MUST return JSON responses with structure:
```json
{
  "answer": "string (user-facing answer)",
  "sources": [
    {
      "chapter": "string (chapter name)",
      "module": "string (module name)",
      "section": "string (section anchor)",
      "excerpt": "string (relevant passage)"
    }
  ],
  "confidence": "high|medium|low",
  "processing_time_ms": number,
  "conversation_id": "string (optional, for future conversation threading)"
}
```

**Testable**: Validate JSON schema against all responses

---

### FR-8: Mobile Responsiveness

The system MUST:
- Display chatbot on screens ≥ 320px wide
- Support touch-friendly input (larger tap targets)
- Show loading indicators during processing
- Display answers in readable format on mobile (appropriate font size, line length)

**Testable**: Verify UI is usable on iPhone SE (375px), iPad (768px), desktop (1920px)

---

### FR-9: Loading States

The system MUST:
- Show loading indicator immediately after user submits question
- Display estimated time-to-response if latency exceeds 2 seconds
- Animate loading indicator (spinner, progress bar, or equivalent)
- Cancel request if user closes chat or navigates away

**Testable**: Verify loading indicator appears within 100ms; request cancels on navigation

---

### FR-10: Security & Privacy

The system MUST:
- Never log user questions without explicit opt-in consent
- Store API keys only in environment variables (never in code or logs)
- Validate and sanitize all user inputs
- Implement rate limiting (e.g., 10 requests per minute per user)
- Only log errors with redaction of user data

**Testable**: Code review; verify no API keys in logs; test rate limiting

---

### FR-11: Type Safety & Code Quality

The system MUST:
- Use type hints for all Python functions
- Achieve ≥ 80% test coverage (unit + integration tests)
- Follow PEP 8 style guide; use Black formatter (line length 100)
- Include docstrings with examples for all public functions
- Pass mypy type checking in strict mode

**Testable**: Run mypy, pytest, coverage analysis in CI/CD

---

### FR-12: Cold Start Reliability

The system MUST:
- Handle Render instance cold starts gracefully
- Return `/health` endpoint status within 5 seconds on first request
- Queue questions during startup; process after service is ready
- Display message to user: "Chat service is warming up. Please try again in a moment."

**Testable**: Deploy to fresh Render instance; verify first request succeeds

---

## Success Criteria

### Performance

- **Response Latency**: System responds to 95% of questions within 3 seconds (p95 ≤ 3s)
- **Relevance Accuracy**: Retrieved passages have average cosine similarity ≥ 0.85 to query
- **Uptime**: Service achieves ≥ 99% uptime (free tier baseline)

### User Experience

- **Answer Quality**: Manual evaluation shows ≥ 80% of answers are accurate and contextually relevant
- **Error Recovery**: 100% of error scenarios return user-friendly messages (no technical errors exposed)
- **Mobile Usability**: Chatbot is fully functional on screens ≥ 320px wide

### Code Quality & Security

- **Test Coverage**: ≥ 80% coverage for backend services (unit + integration tests)
- **Type Safety**: 100% of Python functions have type hints; mypy passes in strict mode
- **Security**: Zero API keys in logs; rate limiting prevents abuse; no sensitive data leaks

### Feature Completeness

- **Source Attribution**: 100% of answers include source references with chapter/module/section
- **Confidence Scoring**: System returns confidence level for all answers
- **Data Privacy**: Zero questions logged without user consent

---

## Key Entities & Data

### Document Entity
- **id**: Unique document identifier
- **chapter**: Chapter name (e.g., "Chapter 2: ROS 2 Fundamentals")
- **module**: Module name (e.g., "Module 2.1: Nodes and Topics")
- **section**: Section anchor (e.g., "ros-2-topics")
- **text**: Full text content (markdown)
- **tokens**: Token count for context window management
- **embedding**: Vector embedding (1536 dimensions from sentence-transformers)

### Question Entity
- **id**: Unique question identifier
- **text**: User's question text
- **embedding**: Vector embedding of question
- **selected_text**: Optional user-selected passage (if applicable)
- **timestamp**: When question was asked
- **conversation_id**: Optional ID for grouping related questions

### Answer Entity
- **id**: Unique answer identifier
- **question_id**: Reference to question
- **text**: LLM-generated answer
- **sources**: List of source references with chapter/module/section
- **confidence**: Calculated confidence (high|medium|low)
- **feedback**: Optional user feedback (thumbs up/down)
- **processing_time_ms**: Time taken to generate answer

---

## Scope: In & Out

### In Scope (MVP)

✅ Context-aware Q&A (Feature 1)
✅ Text selection queries (Feature 2)
✅ Source attribution (Feature 4)
✅ Performance: p95 ≤ 3 seconds
✅ Mobile responsive UI
✅ Error handling with user-friendly messages
✅ Confidence scoring
✅ Type safety & ≥80% test coverage
✅ Rate limiting
✅ Cold start handling

### Out of Scope (Post-MVP)

❌ Multi-turn conversation history (persisted across sessions)
❌ User accounts and personalized chat history
❌ Chatbot personalization by learning style
❌ Multi-language chatbot responses (chapters translate separately)
❌ Advanced analytics on chatbot usage
❌ Admin dashboard for monitoring
❌ Custom training on book content (uses off-the-shelf embeddings)

---

## Assumptions

1. **Embeddings Quality**: Claude API embeddings (or sentence-transformers fallback) are accurate enough for semantic matching (cosine similarity ≥ 0.85 is achievable for relevant passages)
2. **Book Content Structure**: Markdown chapters follow consistent structure with chapter/module/section headings for reliable source attribution
3. **LLM Behavior**: Claude API consistently generates concise, accurate answers from retrieved context
4. **Claude API Availability**: Claude API embeddings and chat endpoints are available and responsive within SLA; exponential backoff handles transient failures
5. **Qdrant Cloud Free Tier Capacity**: 10k embeddings limit is sufficient for MVP (3-5 chapters with optimized chunk sizes of 500-1000 tokens)
6. **User Expectations**: Students expect answers within 3 seconds (based on web application standards)
7. **Data Privacy**: No long-term conversation storage is required for MVP (privacy-first approach)
8. **Free-Tier Sufficiency**: Qdrant Cloud free tier and Render free tier can support expected MVP student load; post-MVP scaling documented for future migration
9. **No Custom Indexing**: Initial index uses extracted markdown content (no additional content creation needed)
10. **FastAPI Task Queues Sufficient**: MVP uses native FastAPI background tasks for orchestration; OpenAI Agents API reserved for future complexity (post-MVP decision point)

---

## Dependencies & Integrations

### External Services

#### LLM & Embeddings
- **Claude API** (Anthropic):
  - Used for: Answer generation via chat completions API
  - Also used for: Embeddings (via Claude embeddings endpoint)
  - Fallback: Sentence-Transformers (local, free) if Claude API unavailable
  - Rate limiting: Implement exponential backoff per API contract

#### Vector Database
- **Qdrant Cloud (Free Tier)**:
  - Capacity: 1 GB storage, 10k embeddings limit
  - Performance: Semantic search with cosine similarity threshold (≥ 0.85)
  - Constraints: Optimize chunk sizes (~500-1000 tokens per chunk) to fit within 10k embeddings across 3-5 chapters
  - Scaling: Plan post-MVP migration to paid Qdrant or self-hosted instance
  - Request rate: 50 requests/second (free tier sufficient for MVP student load)

#### Orchestration (TBD)
- **OpenAI Agents API** (optional): May be used for task orchestration if complexity requires
  - Alternative: Native FastAPI background tasks (recommended for MVP simplicity)
  - Decision point: To be confirmed during planning phase
  - Current implementation: Use FastAPI task queues for cold start handling and bulk indexing

#### Data Persistence
- **Neon Postgres** (serverless): Optional for conversation history (future, post-MVP)
- **Not used in MVP**: Chatbot is stateless; no long-term conversation storage

#### Frontend Platform
- **Docusaurus**: Existing book platform for chapter links/navigation
- **Vercel**: Auto-deployment of Docusaurus frontend with React chatbot widget

### Book Content Structure
- Expects markdown chapters in `/docs/docs/` directory
- Chapter files follow structure: `Chapter-N.md` with H2 headings for modules
- Section anchors auto-generated from H3 headings
- Estimated content: 3-5 chapters, ~40-80 MB total (fits within 10k embeddings limit with optimized chunking)

---

## Testing Strategy

### Unit Tests
- Test embedding pipeline (tokenization, vector generation)
- Test relevance threshold enforcement
- Test response formatting and JSON schema validation
- Test error handling for edge cases

### Integration Tests
- End-to-end Q&A pipeline (question → embedding → search → generation)
- Test with real Qdrant instance (free cloud tier)
- Test with real Claude API (via Groq)
- Test cold start scenarios

### User Acceptance Tests
- Manual evaluation of answer quality for diverse questions
- Test text selection on mobile devices
- Test error messages are user-friendly
- Test source attribution is accurate and clickable

---

## Open Questions & Notes

- **Multi-language Support**: Current spec assumes English questions and answers. Future enhancement could support Urdu (align with book's translation goals).
- **Conversation Threading**: Optional for MVP; consider for post-MVP based on user feedback.
- **Feedback Integration**: Thumbs up/down buttons are placeholders; feedback loop not automated in MVP.

---

## Next Steps

1. **Architecture Planning** (`/sp.plan`): Design backend/frontend structure, data flow, deployment strategy
2. **Task Breakdown** (`/sp.tasks`): Create testable, independent implementation tasks
3. **Implementation** (`/sp.implement`): Build backend API, embed in Docusaurus, deploy to Render + Vercel
4. **Testing**: Execute unit, integration, and UAT tests
5. **Deployment**: Deploy to production (Render + Vercel)

---

**Document Version**: 1.0
**Created**: 2025-12-08
**Status**: Ready for Planning
