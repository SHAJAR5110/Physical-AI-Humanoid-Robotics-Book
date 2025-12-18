import os
import asyncio
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import the existing RAG agent functionality
from agent import RAGAgent

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG Agent with document retrieval and question answering",
    version="1.0.0"
)

# Add CORS middleware
cors_origins = [
    "http://localhost:3000",  # Local development
    "http://localhost:8000",  # Local backend dev
    "https://hackathon-physical-ai-humanoid-text-sigma.vercel.app",  # Production frontend
    "https://localhost:3000",  # HTTPS local development
]

# Add environment-specific origins if provided
frontend_url = os.getenv("FRONTEND_URL")
if frontend_url:
    cors_origins.append(frontend_url)

# Allow all origins in development (for testing)
if os.getenv("ENV", "development") == "development":
    cors_origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)

# Pydantic models
class QueryRequest(BaseModel):
    query: str

class ChatRequest(BaseModel):
    query: str
    chat_history: List[Dict] = []

class MatchedChunk(BaseModel):
    content: str
    url: str
    position: int
    similarity_score: float

class QueryResponse(BaseModel):
    answer: str
    sources: List[str]
    matched_chunks: List[MatchedChunk]
    error: Optional[str] = None
    status: str  # "success", "error", "empty"
    query_time_ms: Optional[float] = None
    confidence: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    answer: str
    sources: List[Dict] = []
    confidence: str
    processing_time_ms: float
    status: str = "success"
    error: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    message: str

# Global RAG agent instance
rag_agent = None

@app.on_event("startup")
async def startup_event():
    """Initialize the RAG agent on startup"""
    global rag_agent
    logger.info("Initializing RAG Agent...")
    try:
        rag_agent = RAGAgent()
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise

@app.post("/ask", response_model=QueryResponse)
async def ask_rag(request: QueryRequest):
    """
    Process a user query through the RAG agent and return the response
    """
    logger.info(f"Processing query: {request.query[:50]}...")

    try:
        # Validate input
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(request.query) > 2000:
            raise HTTPException(status_code=400, detail="Query too long, maximum 2000 characters")

        # Process query through RAG agent
        response = rag_agent.query_agent(request.query)

        # Format response
        formatted_response = QueryResponse(
            answer=response.get("answer", ""),
            sources=response.get("sources", []),
            matched_chunks=[
                MatchedChunk(
                    content=chunk.get("content", ""),
                    url=chunk.get("url", ""),
                    position=chunk.get("position", 0),
                    similarity_score=chunk.get("similarity_score", 0.0)
                )
                for chunk in response.get("matched_chunks", [])
            ],
            error=response.get("error"),
            status="error" if response.get("error") else "success",
            query_time_ms=response.get("query_time_ms"),
            confidence=response.get("confidence")
        )

        logger.info(f"Query processed successfully in {response.get('query_time_ms', 0):.2f}ms")
        return formatted_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        return QueryResponse(
            answer="",
            sources=[],
            matched_chunks=[],
            error=str(e),
            status="error"
        )

@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint for frontend integration
    Matches the expected API contract from the Docusaurus frontend
    """
    logger.info(f"Chat request received: {request.query[:50]}...")

    try:
        # Validate input
        if not request.query or len(request.query.strip()) == 0:
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if len(request.query) > 2000:
            raise HTTPException(status_code=400, detail="Query too long, maximum 2000 characters")

        # Process query through RAG agent
        response = rag_agent.query_agent(request.query)

        # Format response for frontend
        sources = []
        if response.get("sources"):
            for source_url in response.get("sources", []):
                sources.append({
                    "url": source_url,
                    "chapter": extract_chapter_from_url(source_url),
                    "module": extract_module_from_url(source_url),
                    "section": "content"
                })

        # If no explicit sources, extract from matched_chunks
        if not sources and response.get("matched_chunks"):
            for chunk in response.get("matched_chunks", [])[:3]:
                sources.append({
                    "url": chunk.get("url", ""),
                    "chapter": extract_chapter_from_url(chunk.get("url", "")),
                    "module": "Content",
                    "section": "content"
                })

        formatted_response = ChatResponse(
            response=response.get("answer", ""),
            answer=response.get("answer", ""),
            sources=sources,
            confidence=response.get("confidence", "medium"),
            processing_time_ms=response.get("query_time_ms", 0),
            status="success" if not response.get("error") else "error",
            error=response.get("error")
        )

        logger.info(f"Chat processed successfully in {response.get('query_time_ms', 0):.2f}ms")
        return formatted_response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing chat: {e}")
        return ChatResponse(
            response=f"Error: {str(e)}",
            answer=f"Error: {str(e)}",
            sources=[],
            confidence="low",
            processing_time_ms=0,
            status="error",
            error=str(e)
        )

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint
    """
    return HealthResponse(
        status="healthy",
        message="RAG Agent API is running"
    )

def extract_chapter_from_url(url: str) -> str:
    """
    Extract chapter name from URL
    Example: https://example.com/docs/ros2 -> Chapter 2
    """
    chapter_map = {
        "intro": "Chapter 1",
        "ros2": "Chapter 2",
        "gazebo": "Chapter 3",
        "isaac": "Chapter 4",
        "vla": "Chapter 5",
        "capstone": "Chapter 6",
    }

    for key, chapter in chapter_map.items():
        if key in url.lower():
            return chapter

    return "Chapter 1"

def extract_module_from_url(url: str) -> str:
    """
    Extract module/section name from URL
    """
    # Try to get the last part of the URL as the module name
    parts = url.rstrip("/").split("/")
    if len(parts) > 0:
        module = parts[-1].replace("-", " ").title()
        return module

    return "Content"

# For running with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)