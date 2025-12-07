"""FastAPI application for RAG chatbot API."""

import json
import logging
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from src.config import config, logger
from src.services.health_service import check_all_dependencies, check_gemini_health, check_qdrant_health

# Initialize FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="REST API for asking questions about robotics textbook content with vector search and LLM generation",
    version="0.1.0",
)

# Configure CORS middleware FIRST (must be before including routers for preflight requests)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "http://127.0.0.1:3000",  # Local development (IP)
        "http://localhost:8080",  # Alternative local port
        "http://127.0.0.1:8080",  # Alternative local port (IP)
        "https://localhost",  # HTTPS local
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import and include routers
from src.api.v1.routes.chat import router as chat_router


app.include_router(chat_router)


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


# Health check endpoints
@app.get("/health")
async def health() -> Dict[str, str]:
    """Basic health check endpoint."""
    return {"status": "ok"}


@app.get("/health/full")
async def health_full() -> Dict[str, Any]:
    """
    Full health check including dependencies.

    Returns:
        {
            "status": "healthy|degraded|unhealthy",
            "components": {
                "qdrant": {"status": "...", "point_count": ...},
                "gemini": {"status": "...", "model_name": "..."}
            }
        }
    """
    return await check_all_dependencies()


@app.get("/health/qdrant")
async def health_qdrant() -> Dict[str, Any]:
    """Check Qdrant vector database connectivity and collection status."""
    return await check_qdrant_health()


@app.get("/health/gemini")
async def health_gemini() -> Dict[str, Any]:
    """Check Google Gemini API connectivity and model availability."""
    return await check_gemini_health()


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
                "gemini_model": config.gemini_chat_model,
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
