# RAG Chatbot Backend API

**Part of**: Physical AI & Humanoid Robotics Book Project
**Purpose**: Backend API for the RAG chatbot embedded in the Docusaurus textbook
**Tech Stack**: FastAPI, Python 3.11, UV Package Manager, Qdrant, Groq

---

## Quick Start

### Prerequisites

- Python 3.11 or higher
- UV package manager ([install here](https://github.com/astral-sh/uv))
- Git

### Installation & Setup (UV + venv)

```bash
# 1. Navigate to backend directory
cd backend

# 2. Create virtual environment with UV
uv venv .venv

# 3. Activate virtual environment
# On Windows:
.venv\Scripts\activate
# On macOS/Linux:
source .venv/bin/activate

# 4. Install dependencies
uv sync

# 5. Copy environment template and configure
cp .env.example .env
# Edit .env with your API keys:
# - QDRANT_URL and QDRANT_API_KEY
# - GROQ_API_KEY
# - ALLOWED_ORIGINS (for local dev: http://localhost:3000)
```

### Run Development Server

```bash
# From backend/ directory with venv activated
uvicorn main:app --reload

# Server starts at: http://localhost:8000
# API Docs: http://localhost:8000/api/docs
# ReDoc: http://localhost:8000/api/redoc
```

### Environment Variables

Create a `.env` file from `.env.example`:

```env
# Qdrant Vector Database (free tier: https://qdrant.io/cloud)
QDRANT_URL=https://<your-cluster>.qdrant.io
QDRANT_API_KEY=<your-api-key>

# Groq LLM API (free tier: https://console.groq.com)
GROQ_API_KEY=<your-api-key>

# Application Settings
ENVIRONMENT=development
LOG_LEVEL=debug
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000

# Server
HOST=0.0.0.0
PORT=8000
```

**⚠️ Never commit `.env` file. It contains API keys.**

---

## Project Structure

```
backend/
├── main.py                     # FastAPI app, CORS, lifespan
├── pyproject.toml             # Dependencies (UV config)
├── .env.example               # Environment template
├── .gitignore                 # Git ignore patterns
├── README.md                  # This file
├── src/
│   ├── __init__.py
│   ├── config.py             # Settings, environment loading
│   ├── models.py             # Pydantic models (ChatRequest, ChatResponse)
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embedding_service.py    # Sentence-transformers
│   │   ├── retrieval_service.py    # Qdrant client
│   │   ├── generation_service.py   # Groq LLM
│   │   ├── confidence_service.py   # Confidence scoring
│   │   └── chat_pipeline_service.py # Orchestration
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── chat.py           # POST /api/chat endpoint
│   │   └── health.py         # GET /health endpoint
│   ├── middleware/
│   │   ├── __init__.py
│   │   ├── rate_limit.py     # Rate limiting (10 req/min)
│   │   └── logging.py        # Structured logging
│   └── utils/
│       ├── __init__.py
│       ├── error_handler.py      # User-friendly errors
│       ├── context_manager.py    # Token limit management
│       ├── response_formatter.py # JSON formatting
│       ├── validators.py         # Input validation
│       └── slug_generator.py     # URL-safe slugs
├── tests/
│   ├── __init__.py
│   ├── conftest.py           # Pytest fixtures
│   ├── test_embedding_service.py
│   ├── test_retrieval_service.py
│   ├── test_generation_service.py
│   ├── test_confidence_service.py
│   ├── test_models.py
│   ├── test_chat_endpoint.py
│   └── test_e2e_qa_pipeline.py
└── scripts/
    └── index_chapters.py     # Initial data indexing to Qdrant
```

---

## API Endpoints

### POST `/api/chat`

Ask a question and get an answer with sources.

**Request**:
```json
{
  "question": "What is ROS 2?",
  "selected_text": "Optional selected passage from book"
}
```

**Response** (200 OK):
```json
{
  "answer": "ROS 2 is a middleware for robotics...",
  "sources": [
    {
      "chapter": "Chapter 2: ROS 2 Fundamentals",
      "module": "Module 2.1: Nodes and Topics",
      "section": "ros-2-topics",
      "excerpt": "ROS 2 is built on a publish-subscribe architecture..."
    }
  ],
  "confidence": "high",
  "processing_time_ms": 1850
}
```

**Error Response** (400):
```json
{
  "error": "Validation failed",
  "details": "question must be at least 3 characters"
}
```

### GET `/health`

Health check endpoint for monitoring.

**Response** (200):
```json
{
  "status": "healthy",
  "app": "RAG Chatbot API",
  "version": "1.0.0",
  "environment": "development"
}
```

### GET `/api/docs`

Interactive API documentation (Swagger UI).

### GET `/api/redoc`

Alternative API documentation (ReDoc).

---

## Development Workflow

### Run Tests

```bash
# Run all tests with coverage
pytest tests/ -v --cov=src --cov-report=html

# Run specific test file
pytest tests/test_embedding_service.py -v

# Run with specific marker
pytest tests/ -v -k "embedding"
```

Coverage report will be generated in `htmlcov/index.html`.

### Code Quality Checks

```bash
# Format code
black src/ tests/ --line-length 100

# Lint code
ruff check src/ tests/

# Type checking (strict mode)
mypy --strict src/
```

### Development Checklist

- [ ] Virtual environment created and activated
- [ ] Dependencies installed with `uv sync`
- [ ] `.env` file configured with API keys
- [ ] Dev server runs: `uvicorn main:app --reload`
- [ ] Health check works: `curl http://localhost:8000/health`
- [ ] Tests pass: `pytest tests/ -v`
- [ ] Code passes linting: `black`, `ruff`, `mypy`

---

## Indexing Book Content

Before the chatbot can answer questions, you must index the book chapters.

### Initial Indexing

```bash
# From backend/ with venv activated
python scripts/index_chapters.py

# This:
# 1. Reads markdown files from book-source/docs/docs/Chapter-*.md
# 2. Extracts H1 (chapter), H2 (module), H3 (section) headings
# 3. Creates embeddings using sentence-transformers
# 4. Uploads to Qdrant collection "book_passages"
```

### What Gets Indexed

- **Chapter**: "Chapter 2: ROS 2 Fundamentals"
- **Module**: "Module 2.1: Nodes and Topics"
- **Section**: "ros-2-topics" (slugified for links)
- **Text**: Full markdown section content
- **Embedding**: 1536-dimensional vector from sentence-transformers

---

## Deployment

### Prerequisites

- Qdrant Cloud account (free tier: https://qdrant.io/cloud)
- Groq API account (free tier: https://console.groq.com)
- GitHub Actions secrets configured

### Deploy to GitHub Pages (Backend API)

1. **Create GitHub Actions workflow** `.github/workflows/deploy-backend.yml`:

```yaml
name: Deploy Backend

on:
  push:
    branches: [main, master]
  pull_request:
    branches: [main, master]

jobs:
  test-and-deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - uses: astral-sh/setup-uv@v1
        with:
          python-version: "3.11"

      - name: Install dependencies
        run: cd backend && uv sync

      - name: Run tests
        run: cd backend && pytest tests/ -v --cov=src

      - name: Type check
        run: cd backend && mypy --strict src/

      - name: Format check
        run: cd backend && black --check src/ tests/

      - name: Lint
        run: cd backend && ruff check src/ tests/
```

2. **Set GitHub Actions Secrets**:
   - `QDRANT_URL`: Qdrant Cloud URL
   - `QDRANT_API_KEY`: Qdrant API key
   - `GROQ_API_KEY`: Groq API key

3. **Push to GitHub**: Tests and type checks run automatically

### Alternative: Deploy to Render.com (Free)

1. Create account at https://render.com
2. Connect GitHub repository
3. Create new Web Service
4. Configure:
   - **Runtime**: Python 3.11
   - **Build**: `uv sync`
   - **Start**: `uvicorn main:app --host 0.0.0.0 --port $PORT`
   - **Env vars**: QDRANT_URL, QDRANT_API_KEY, GROQ_API_KEY, etc.

5. Deploy and get public API URL

---

## Performance Targets (from Constitution)

| Metric | Target | Status |
|--------|--------|--------|
| p95 response latency | ≤ 3 seconds | ✓ Achievable |
| Retrieval relevance | ≥ 85% similarity | ✓ Threshold enforced |
| Confidence accuracy | High/medium/low classification | ✓ Implemented |
| Type safety | 100% type hints, mypy strict | ✓ In progress |
| Test coverage | ≥ 80% | ✓ Target |
| Uptime | ≥ 99% | ✓ Free tier baseline |

---

## Troubleshooting

### ImportError: No module named 'fastapi'

**Solution**: Activate virtual environment and install dependencies:
```bash
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
uv sync
```

### CORS error when calling from frontend

**Solution**: Update `ALLOWED_ORIGINS` in `.env`:
```env
ALLOWED_ORIGINS=https://your-book-site.com,http://localhost:3000
```

### Qdrant connection refused

**Solution**: Verify credentials and URL in `.env`:
```bash
curl https://<your-cluster>.qdrant.io/health
# Should return 200 OK
```

### No relevant content found errors

**Solution**: Verify chapters are indexed:
```bash
python scripts/index_chapters.py
```

---

## Contributing

1. Create a feature branch: `git checkout -b feature/my-feature`
2. Make changes and add tests
3. Run quality checks:
   ```bash
   black src/ tests/
   ruff check src/ tests/
   mypy --strict src/
   pytest tests/ -v --cov=src
   ```
4. Commit and push: `git push origin feature/my-feature`
5. Create a Pull Request

---

## License

MIT License - See LICENSE file for details

---

## Support

- **Documentation**: See `/specs/002-rag-chatbot/`
- **Architecture Plan**: `/specs/002-rag-chatbot/plan.md`
- **Task Breakdown**: `/specs/002-rag-chatbot/tasks.md`
- **API Spec**: `/specs/002-rag-chatbot/contracts/chat-api.openapi.yaml`

---

**Last Updated**: 2025-12-09
