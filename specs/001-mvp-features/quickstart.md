# Phase 1 Setup: Quick Start Guide

**Branch**: `001-mvp-features` | **Date**: 2025-12-06 | **Target Audience**: Developers
**Duration**: 30–45 minutes (first-time setup, end-to-end)

---

## Overview

This guide walks you through:
1. **Repository cloning** and branch setup
2. **Backend initialization** (FastAPI, dependencies, database migrations)
3. **Frontend initialization** (Docusaurus, Node.js setup)
4. **Environment configuration** (.env files, API keys)
5. **Database setup** (Neon Postgres, migrations)
6. **Vector database setup** (Qdrant)
7. **First run** (start backend + frontend servers)
8. **Validation** (health checks, basic smoke tests)

**Success Criteria**:
- [ ] Backend server running on `localhost:8000`
- [ ] Frontend dev server running on `localhost:3000`
- [ ] `/health` endpoint returns all services OK
- [ ] Can sign up a new user via `/auth/signup`
- [ ] Can retrieve chapters via `/chapters`

---

## Prerequisites

### System Requirements
- **OS**: Linux (recommended for ROS 2), macOS, or Windows (WSL2)
- **Memory**: 4GB RAM minimum
- **Disk Space**: 2GB free

### Software Requirements
- Git (`git --version` ≥ 2.30)
- Python 3.11+ (`python --version`)
- Node.js 18+ (`node --version`)
- npm 9+ (`npm --version`)
- Docker or Docker Desktop (for Qdrant, optional; can use managed instance)

### Accounts Required
- **GitHub**: Access to `https://github.com/SHAJAR5110/Physical-AI-Humanoid-Robotics-Book`
- **Neon Postgres**: Free tier account (https://neon.tech) for managed PostgreSQL
- **Qdrant**: Free tier account (https://qdrant.tech) or self-hosted via Docker
- **Anthropic Claude API**: API key for Claude 3.5 Haiku (https://console.anthropic.com)
- **BetterAuth**: Service credentials (project ID, secret)

---

## Step 1: Clone Repository & Create Branch

```bash
# Clone the repository
git clone https://github.com/SHAJAR5110/Physical-AI-Humanoid-Robotics-Book.git
cd Physical-AI-Humanoid-Robotics-Book

# Verify you're on master branch
git branch
# Output: * master

# Switch to feature branch (or create if doesn't exist)
git checkout 001-mvp-features
# or: git checkout -b 001-mvp-features origin/001-mvp-features

# Verify current branch
git branch
# Output: * 001-mvp-features
```

---

## Step 2: Backend Setup (FastAPI)

### 2.1 Install Python Dependencies

```bash
# Create Python virtual environment
python3 -m venv backend/venv

# Activate virtual environment
# Linux/macOS:
source backend/venv/bin/activate
# Windows (PowerShell):
backend\venv\Scripts\Activate.ps1

# Verify activation (prompt should show (venv))
python --version
# Output: Python 3.11.x

# Install dependencies
pip install -r backend/requirements.txt
# This includes: fastapi, uvicorn, sqlalchemy, pydantic, python-dotenv, etc.

# Verify installation
python -c "import fastapi; print(fastapi.__version__)"
```

### 2.2 Create Backend `.env` File

Create `backend/.env` with the following variables:

```bash
cat > backend/.env << 'EOF'
# FastAPI Configuration
ENVIRONMENT=development
DEBUG=True
LOG_LEVEL=INFO

# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@ep-xxxxx.us-east-1.neon.tech/physical_ai_book
NEON_API_KEY=your_neon_api_key_here

# Vector Database (Qdrant)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Anthropic Claude API
ANTHROPIC_API_KEY=sk-ant-xxxxxxxxxxxxx
ANTHROPIC_MODEL=claude-3-5-haiku-20241022

# BetterAuth
BETTERAUTH_PROJECT_ID=your_betterauth_project_id
BETTERAUTH_SECRET=your_betterauth_secret

# CORS (for frontend dev server on localhost:3000)
CORS_ORIGINS=http://localhost:3000,http://localhost:5173

# API Base URL (for frontend to call backend)
API_BASE_URL=http://localhost:8000/api
EOF

# IMPORTANT: Never commit .env file (should be in .gitignore)
```

**Getting Credentials**:
- **DATABASE_URL**: From Neon dashboard → Connection string
- **ANTHROPIC_API_KEY**: From Anthropic console
- **QDRANT_API_KEY**: From Qdrant Cloud dashboard
- **BETTERAUTH**: From BetterAuth project settings

### 2.3 Initialize Database

```bash
# Navigate to backend directory
cd backend

# Run Alembic migrations (creates tables)
alembic upgrade head

# Verify tables created
psql "postgresql://user:password@ep-xxxxx.us-east-1.neon.tech/physical_ai_book" \
  -c "\dt"
# Output: Should show users, sessions, chapters, etc.

# Return to repo root
cd ..
```

### 2.4 Load Sample Data (Chapters)

```bash
# Create a Python script to load chapters
python << 'EOF'
import os
import sys
sys.path.insert(0, 'backend/src')

from sqlalchemy import create_engine
from models.chapter import Chapter
from db.connection import SessionLocal

# Sample chapters
chapters_data = [
  {
    "title": "Introduction to Physical AI",
    "slug": "intro",
    "content": "# Introduction to Physical AI\n\nPhysical AI combines robotics...",
    "summary": "Overview of Physical AI fundamentals",
    "author": "Dr. Pieter Abbeel",
    "version": "1.0",
    "page_count": 12
  },
  # ... add remaining 5 chapters
]

db = SessionLocal()
for ch in chapters_data:
  existing = db.query(Chapter).filter_by(slug=ch['slug']).first()
  if not existing:
    db.add(Chapter(**ch))
db.commit()
print("Chapters loaded successfully")
EOF
```

### 2.5 Start Backend Server

```bash
cd backend
source venv/bin/activate  # Activate venv if not active

# Start FastAPI server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Output:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Started server process [12345]
```

**In a new terminal**, verify backend is running:

```bash
curl http://localhost:8000/api/health

# Output:
# {
#   "status": "ok",
#   "services": {
#     "database": "ok",
#     "qdrant": "ok",
#     "claude_api": "ok"
#   }
# }
```

---

## Step 3: Frontend Setup (Docusaurus)

### 3.1 Install Node.js Dependencies

```bash
cd frontend

# Install dependencies
npm install

# Verify installation
npm list docusaurus
# Output: @docusaurus/core@3.x.x, etc.
```

### 3.2 Create Frontend `.env.local`

```bash
cat > frontend/.env.local << 'EOF'
REACT_APP_API_BASE_URL=http://localhost:8000/api
REACT_APP_ENVIRONMENT=development
EOF
```

### 3.3 Start Development Server

```bash
# From frontend directory
npm run start

# Output:
# ⚠ It seems you don't have a 'docusaurus.config.js' file yet.
#   ...
# Docusaurus is listening on http://localhost:3000
# Open http://localhost:3000 to view it in the browser.
```

**In browser**, navigate to `http://localhost:3000`

---

## Step 4: Vector Database Setup (Qdrant)

### Option A: Qdrant Cloud (Recommended for MVP)

```bash
# 1. Sign up at https://qdrant.tech
# 2. Create cluster (free tier)
# 3. Copy cluster URL and API key
# 4. Paste into backend/.env as QDRANT_URL and QDRANT_API_KEY

# Test connection from backend
python << 'EOF'
from qdrant_client import QdrantClient
import os

client = QdrantClient(
  url=os.getenv("QDRANT_URL"),
  api_key=os.getenv("QDRANT_API_KEY")
)

collections = client.get_collections()
print("Qdrant connection OK")
print(f"Collections: {collections}")
EOF
```

### Option B: Self-Hosted Qdrant (Docker)

```bash
# Start Qdrant container
docker run -d \
  --name qdrant \
  -p 6333:6333 \
  qdrant/qdrant:latest

# Wait for startup (10s)
sleep 10

# Test connection
curl http://localhost:6333/health
# Output: {"status":"ok"}

# Update backend/.env
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=""  # Local instance, no auth needed
```

### 4.1 Create Qdrant Collection

```bash
# From backend directory
python << 'EOF'
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os

client = QdrantClient(
  url=os.getenv("QDRANT_URL"),
  api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection for chapter embeddings
collection_name = "chapters"
vector_size = 1024  # Claude embeddings

try:
  client.create_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
  )
  print(f"Collection '{collection_name}' created")
except Exception as e:
  print(f"Collection may already exist: {e}")
EOF
```

---

## Step 5: Embedding Batch Job (Pre-compute Chapter Vectors)

This step converts chapter markdown to embeddings stored in Qdrant.

```bash
# From backend directory
python << 'EOF'
import sys
sys.path.insert(0, 'src')

from services.embedding_service import EmbeddingService
from db.connection import SessionLocal
from models.chapter import Chapter

db = SessionLocal()
embedding_service = EmbeddingService()

chapters = db.query(Chapter).all()

for chapter in chapters:
  print(f"Embedding Chapter {chapter.id}: {chapter.title}...")
  embedding_service.embed_chapter(chapter)
  print(f"  ✓ Embedded {chapter.id}")

print("All chapters embedded successfully")
EOF
```

---

## Step 6: Smoke Tests (Validation)

### 6.1 Health Check

```bash
curl -s http://localhost:8000/api/health | jq

# Expected output:
# {
#   "status": "ok",
#   "services": {
#     "database": "ok",
#     "qdrant": "ok",
#     "claude_api": "ok"
#   }
# }
```

### 6.2 Sign Up User

```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPassword123!",
    "name": "Test User",
    "os": "linux",
    "gpu": "nvidia-rtx-4090",
    "experience_level": "intermediate",
    "robotics_background": true
  }' | jq

# Expected output:
# {
#   "user": {
#     "id": "550e8400-...",
#     "email": "test@example.com",
#     "name": "Test User",
#     ...
#   },
#   "session_token": "eyJhbGc...",
#   "expires_at": "2025-12-07T14:30:00Z"
# }
```

Save the `session_token` for next test.

### 6.3 List Chapters

```bash
# No auth required
curl -s http://localhost:8000/api/chapters | jq

# Expected output:
# {
#   "chapters": [
#     {
#       "id": 1,
#       "title": "Introduction to Physical AI",
#       "slug": "intro",
#       "summary": "...",
#       "page_count": 12
#     },
#     ...
#   ]
# }
```

### 6.4 Get Single Chapter

```bash
curl -s http://localhost:8000/api/chapters/1 | jq .title

# Expected output:
# "Introduction to Physical AI"
```

### 6.5 RAG Chat Test

```bash
SESSION_TOKEN="eyJhbGc..."  # From signup response

curl -X POST http://localhost:8000/api/chat \
  -H "Authorization: Bearer $SESSION_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "chapter_id": 1,
    "user_message": "What is Physical AI?"
  }' | jq

# Expected output:
# {
#   "assistant_response": "Physical AI is the field of AI that...",
#   "citations": [
#     {
#       "chapter_id": 1,
#       "section_title": "Introduction",
#       "excerpt": "..."
#     }
#   ],
#   "tokens_used": 342
# }
```

---

## Step 7: Database Schema Visualization

To view the schema in your database:

```bash
# Connect to Neon Postgres
psql "postgresql://user:password@ep-xxxxx.us-east-1.neon.tech/physical_ai_book"

# List all tables
\dt

# Describe a table
\d users

# View indexes
\di

# Exit
\q
```

---

## Step 8: Useful Commands

### Backend Commands

```bash
# Activate virtual environment
source backend/venv/bin/activate

# Run tests
pytest backend/tests -v

# Generate test coverage
pytest --cov=backend/src backend/tests

# Format code
black backend/src

# Type checking
mypy backend/src

# Linting
pylint backend/src
```

### Frontend Commands

```bash
cd frontend

# Build static site
npm run build

# Start dev server
npm run start

# Run tests
npm test

# Lint and format
npm run lint
npm run format
```

### Database Commands

```bash
# Run migrations
cd backend
alembic upgrade head

# Create new migration (after schema change)
alembic revision --autogenerate -m "Add new column"

# Rollback one migration
alembic downgrade -1
```

---

## Step 9: Deployment Checklist (Before Going to Production)

- [ ] All tests pass: `pytest backend/tests -v`
- [ ] Frontend builds: `npm run build`
- [ ] All environment variables set (no `.env` file committed)
- [ ] Database migrations applied: `alembic upgrade head`
- [ ] Chapters embedded in Qdrant: verify via `/chapters` endpoint
- [ ] Auth working: signup → signin → signout cycle verified
- [ ] RAG chat tested: `/chat` returns citations matching chapter content
- [ ] Health check passing: `/health` returns all services OK
- [ ] Rate limiting configured: 10 personalization/10 translation per hour, 30 chat per hour
- [ ] Error handling tested: invalid tokens, missing chapters, API timeouts

---

## Step 10: Troubleshooting

### Backend won't start: `ModuleNotFoundError: No module named 'fastapi'`

```bash
# Re-activate virtual environment
source backend/venv/bin/activate
pip install -r backend/requirements.txt
```

### Database connection failed: `psycopg2.OperationalError`

```bash
# Verify DATABASE_URL is correct
grep DATABASE_URL backend/.env

# Test connection manually
psql "your_connection_string_here"

# If Neon, ensure IP is whitelisted
```

### Qdrant connection refused

```bash
# Check Qdrant is running
curl http://localhost:6333/health

# If using Qdrant Cloud, verify API key in .env
```

### Frontend can't reach backend API

```bash
# Verify backend is running on port 8000
curl http://localhost:8000/api/health

# Check CORS_ORIGINS in backend/.env includes http://localhost:3000
```

### Claude API authentication failed

```bash
# Verify API key is set in backend/.env
grep ANTHROPIC_API_KEY backend/.env

# If not set, get from https://console.anthropic.com
```

---

## Step 11: Next Steps After Setup

1. **Run P1 implementation tasks** (from `specs/001-mvp-features/tasks.md`)
   - T001-T058: Setup, foundational infrastructure, book content, auth, deployment
2. **Create PR** for Week 1 work
3. **Weekly checkpoint**: Verify success criteria against test checklist
4. **Deploy to Render + GitHub Pages** (Week 1 P1)

---

## Success Metrics

After completing this quickstart, you should have:

✅ **Backend**:
- FastAPI server running on `localhost:8000`
- All tables created in Neon Postgres
- 6 chapters loaded and embedded in Qdrant
- `/health` returning all services OK

✅ **Frontend**:
- Docusaurus dev server on `localhost:3000`
- Can navigate between chapters

✅ **Authentication**:
- Can sign up → receive session token
- Can sign in → receive session token
- Can sign out → token invalidated

✅ **RAG Chat**:
- Can ask questions about chapters
- Responses include citations from source text
- No hallucinations (100% citation accuracy)

✅ **Performance**:
- Chat response <5 seconds
- Page load <3 seconds
- Personalization <25 seconds (first run)

---

## Reflection

This quickstart enables developers to:
- **Onboard quickly** (30–45 minutes, end-to-end)
- **Test locally** before deployment
- **Understand architecture** (backend + frontend separation)
- **Validate MVP features** (auth, chapters, RAG, personalization, translation)

No blockers or external dependencies remain. Ready for P1 implementation.
