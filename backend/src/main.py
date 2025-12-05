"""FastAPI application for RAG chatbot API."""

import json
import logging
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from src.config import config, logger

# Initialize FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="REST API for asking questions about robotics textbook content with vector search and LLM generation",
    version="0.1.0",
)

# Import and include routers
from src.api.v1.routes.chat import router as chat_router

app.include_router(chat_router)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "http://localhost:8080",  # Alternative local port
        "https://localhost",  # HTTPS local
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Custom exception handlers for error responses
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Any, exc: HTTPException) -> JSONResponse:
    """Handle HTTP exceptions with user-friendly messages."""
    error_response: Dict[str, Any] = {"detail": exc.detail}

    # Add context for specific status codes
    if exc.status_code == 400:
        error_response[
            "error_type"
        ] = "bad_request"  # Invalid input validation
    elif exc.status_code == 429:
        error_response["error_type"] = "rate_limit"  # Too many requests
        error_response["message"] = "Rate limit exceeded. Maximum 5 requests per second."
    elif exc.status_code == 503:
        error_response["error_type"] = "service_unavailable"  # Service down
        error_response[
            "message"
        ] = "Service temporarily unavailable. Please try again later."

    # Log the error
    logger.warning(
        f"HTTP exception: {exc.status_code}",
        extra={
            "status_code": exc.status_code,
            "detail": str(exc.detail),
            "path": request.url.path,
        },
    )

    return JSONResponse(
        status_code=exc.status_code,
        content=error_response,
    )


# Health check endpoint
@app.get("/health")
async def health() -> Dict[str, str]:
    """Health check endpoint."""
    return {"status": "ok"}


# Startup event
@app.on_event("startup")
async def startup_event() -> None:
    """Initialize on startup."""
    try:
        config.validate()
        logger.info(
            "FastAPI application started",
            extra={
                "operation": "startup",
                "status": "success",
                "gemini_model": config.gemini_model,
                "collection_name": config.collection_name,
            },
        )
    except ValueError as e:
        logger.error(
            f"Configuration validation failed: {str(e)}",
            extra={
                "operation": "startup",
                "status": "failed",
                "error": str(e),
            },
        )
        raise


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event() -> None:
    """Cleanup on shutdown."""
    logger.info(
        "FastAPI application shutdown",
        extra={"operation": "shutdown", "status": "success"},
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
