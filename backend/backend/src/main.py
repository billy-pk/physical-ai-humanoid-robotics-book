from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from backend.src.core.config import settings
from backend.src.core.logging import configure_logging, logger
from backend.src.core.monitoring import metrics
from backend.src.api.routes import chat

# Configure structured logging at application startup
configure_logging()

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    logger.info("Application starting up")
    # In a real application, connect to DBs here
    yield
    # Shutdown logic
    logger.info("Application shutting down")

app = FastAPI(lifespan=lifespan,
              title="AI-Course-Book Backend",
              description="FastAPI backend for the AI-Course-Book platform with RAG chatbot.",
              version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # TODO: Configure dynamic origins from settings
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register API routers
app.include_router(chat.router)

@app.get("/health")
async def health_check():
    return {"status": "ok", "message": "Service is healthy"}
