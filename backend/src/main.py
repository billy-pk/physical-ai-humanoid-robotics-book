from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .core.config import settings
from .core.logging import configure_logging, logger
from .core.monitoring import metrics
from .api.routes import chat, auth, session_proxy, content
from .database import async_engine # Import async_engine

# Configure structured logging at application startup
configure_logging()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    logger.info("Application starting up")
    # Initialize database connections
    yield
    # Shutdown logic
    logger.info("Application shutting down")
    await async_engine.dispose() # Dispose of the async engine pool

app = FastAPI(lifespan=lifespan,
              title="AI-Course-Book Backend",
              description="FastAPI backend for the AI-Course-Book platform with RAG chatbot.",
              version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Frontend origin (no path in origin)
    ],
    allow_credentials=True,  # Important: Allow cookies to be sent
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register API routers
app.include_router(chat.router)
app.include_router(auth.router)
app.include_router(session_proxy.router)
app.include_router(content.router, prefix="/api/content", tags=["content"]) # Include the new content router with prefix

@app.get("/health")
async def health_check():
    return {"status": "ok", "message": "Service is healthy"}
