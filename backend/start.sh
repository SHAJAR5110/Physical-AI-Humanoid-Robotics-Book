#!/bin/bash

# Railway Startup Script for RAG Chatbot Backend
# This script installs dependencies and starts the FastAPI server

set -e

echo "🚀 Starting RAG Chatbot Backend on Railway..."

# Install Python dependencies
echo "📦 Installing dependencies from requirements.txt..."
pip install -r requirements.txt

# Run database migrations if needed (optional)
# python -m alembic upgrade head

# Start the FastAPI server with Uvicorn
echo "▶️  Starting FastAPI server..."
uvicorn main:app --host 0.0.0.0 --port $PORT

