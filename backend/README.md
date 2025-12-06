# Physical AI Book Backend

FastAPI REST API for the Physical AI & Humanoid Robotics Book MVP.

## Quick Start

### Prerequisites

- Python 3.11+
- PostgreSQL (or Neon managed)
- Qdrant (vector database)
- Anthropic Claude API key

### Setup

1. **Create virtual environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\Activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Configure environment**:
   ```bash
   cp .env.example .env
   # Edit .env with your credentials
   ```

4. **Initialize database**:
   ```bash
   cd src
   python -c "from db.connection import init_db; init_db()"
   ```

5. **Start server**:
   ```bash
   uvicorn src.main:app --reload
   ```

Server runs on `http://localhost:8000`

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI application entry point
│   ├── config.py            # Configuration management
│   ├── models/              # SQLAlchemy ORM models
│   │   ├── user.py
│   │   ├── chapter.py
│   │   └── chat.py
│   ├── services/            # Business logic
│   │   ├── auth_service.py
│   │   ├── chapter_service.py
│   │   ├── personalization_service.py
│   │   ├── translation_service.py
│   │   └── rag_service.py
│   ├── api/                 # API routes
│   │   └── routes/
│   │       ├── auth.py
│   │       ├── chapters.py
│   │       ├── personalize.py
│   │       ├── translate.py
│   │       └── chat.py
│   ├── db/                  # Database connection
│   │   └── connection.py
│   └── utils/               # Utilities
│       ├── cache.py
│       ├── logging.py
│       └── validators.py
├── tests/                   # Test suite
│   ├── unit/
│   ├── integration/
│   └── fixtures/
├── requirements.txt
├── .env.example
├── pyproject.toml
└── README.md
```

## API Endpoints

See `../specs/001-mvp-features/contracts/api.yaml` for full OpenAPI specification.

### Authentication
- `POST /api/auth/signup` - Register new user
- `POST /api/auth/signin` - Login
- `POST /api/auth/signout` - Logout

### Chapters
- `GET /api/chapters` - List all chapters
- `GET /api/chapters/{id}` - Get chapter by ID

### Features
- `POST /api/personalize` - Rewrite chapter for user profile
- `POST /api/translate` - Translate chapter to Urdu
- `POST /api/chat` - RAG chatbot with citations

### Health
- `GET /api/health` - Service health check

## Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src

# Run specific test
pytest tests/unit/test_auth_service.py -v
```

## Code Quality

```bash
# Format code
black src/

# Check types
mypy src/

# Lint
pylint src/
```

## Database Migrations

Using Alembic for schema versioning:

```bash
# Create new migration
alembic revision --autogenerate -m "Add new column"

# Apply migrations
alembic upgrade head

# Rollback
alembic downgrade -1
```

## Environment Variables

See `.env.example` for all available options:

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@host/db` |
| `QDRANT_URL` | Qdrant vector database endpoint | `https://cluster.qdrant.io` |
| `ANTHROPIC_API_KEY` | Claude API key | `sk-ant-...` |
| `CORS_ORIGINS` | Allowed CORS origins | `http://localhost:3000` |
| `ENVIRONMENT` | Deployment environment | `development` or `production` |

## Deployment

### Render.com (Recommended)

1. Connect GitHub repository
2. Create new Web Service
3. Build command: `pip install -r backend/requirements.txt`
4. Start command: `cd backend && uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Set environment variables from dashboard
6. Deploy!

### Docker

```bash
docker build -t physical-ai-backend .
docker run -p 8000:8000 --env-file .env physical-ai-backend
```

## Contributing

- Follow PEP 8 style guide
- Add tests for new features
- Run `pytest` before committing
- Include docstrings for all functions

## License

MIT
