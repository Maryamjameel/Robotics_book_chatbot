"""Health check service for RAG chatbot dependencies."""

import asyncio
from typing import Any, Dict, Optional

import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.exceptions import UnexpectedResponse

from ..config import config, logger


async def check_qdrant_health() -> Dict[str, Any]:
    """
    Check Qdrant vector database health and collection status.

    Returns:
        Dict with status, collection_name, point_count, and error details if any

    Example:
        {
            "status": "healthy",
            "collection_name": "robotics_chapters",
            "point_count": 1523,
            "error": None
        }
    """
    health_status = {
        "status": "unknown",
        "collection_name": config.collection_name,
        "point_count": 0,
        "error": None,
    }

    try:
        loop = asyncio.get_event_loop()
        result = await asyncio.wait_for(
            loop.run_in_executor(None, _check_qdrant_sync),
            timeout=5.0,
        )
        return result

    except asyncio.TimeoutError:
        health_status["status"] = "unhealthy"
        health_status["error"] = "Qdrant connection timeout (5s)"
        logger.warning(
            "Qdrant health check timed out",
            extra={
                "operation": "check_qdrant_health",
                "status": "timeout",
            },
        )
        return health_status

    except Exception as e:
        health_status["status"] = "unhealthy"
        health_status["error"] = str(e)
        logger.error(
            f"Qdrant health check failed: {str(e)}",
            extra={
                "operation": "check_qdrant_health",
                "error": str(e),
                "status": "failed",
            },
        )
        return health_status


def _check_qdrant_sync() -> Dict[str, Any]:
    """Synchronous Qdrant health check implementation."""
    health_status = {
        "status": "healthy",
        "collection_name": config.collection_name,
        "point_count": 0,
        "error": None,
    }

    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=5.0,
        )

        # Get collection info
        collection_info = client.get_collection(config.collection_name)
        health_status["point_count"] = collection_info.points_count

        logger.info(
            "Qdrant health check passed",
            extra={
                "operation": "check_qdrant_health",
                "status": "healthy",
                "points": collection_info.points_count,
            },
        )

        return health_status

    except Exception as e:
        health_status["status"] = "unhealthy"
        health_status["error"] = str(e)
        raise


async def check_gemini_health() -> Dict[str, Any]:
    """
    Check Google Gemini API health and model availability.

    Returns:
        Dict with status, model_name, and error details if any

    Example:
        {
            "status": "healthy",
            "model_name": "gemini-1.5-flash",
            "error": None
        }
    """
    health_status = {
        "status": "unknown",
        "model_name": config.gemini_model,
        "error": None,
    }

    try:
        loop = asyncio.get_event_loop()
        result = await asyncio.wait_for(
            loop.run_in_executor(None, _check_gemini_sync),
            timeout=5.0,
        )
        return result

    except asyncio.TimeoutError:
        health_status["status"] = "unhealthy"
        health_status["error"] = "Gemini API timeout (5s)"
        logger.warning(
            "Gemini health check timed out",
            extra={
                "operation": "check_gemini_health",
                "status": "timeout",
            },
        )
        return health_status

    except Exception as e:
        health_status["status"] = "unhealthy"
        health_status["error"] = str(e)
        logger.error(
            f"Gemini health check failed: {str(e)}",
            extra={
                "operation": "check_gemini_health",
                "error": str(e),
                "status": "failed",
            },
        )
        return health_status


def _check_gemini_sync() -> Dict[str, Any]:
    """Synchronous Gemini API health check implementation."""
    health_status = {
        "status": "healthy",
        "model_name": config.gemini_model,
        "error": None,
    }

    try:
        # Configure API
        genai.configure(api_key=config.gemini_api_key)

        # List available models to verify API access
        models = genai.list_models()
        available_model_names = [model.name for model in models]

        # Check if configured model is available
        full_model_name = f"models/{config.gemini_model}"
        if full_model_name not in available_model_names:
            health_status["status"] = "unhealthy"
            health_status["error"] = (
                f"Model {config.gemini_model} not available in account"
            )
            return health_status

        logger.info(
            "Gemini health check passed",
            extra={
                "operation": "check_gemini_health",
                "status": "healthy",
                "model": config.gemini_model,
            },
        )

        return health_status

    except Exception as e:
        health_status["status"] = "unhealthy"
        health_status["error"] = str(e)
        raise


async def check_all_dependencies() -> Dict[str, Any]:
    """
    Check health of all system dependencies in parallel.

    Returns:
        Dict with overall status and individual component statuses

    Example:
        {
            "status": "healthy",
            "timestamp": "2025-12-05T10:30:45Z",
            "components": {
                "qdrant": {"status": "healthy", "point_count": 1523},
                "gemini": {"status": "healthy", "model_name": "gemini-1.5-flash"}
            }
        }
    """
    try:
        # Run health checks in parallel
        qdrant_status, gemini_status = await asyncio.gather(
            check_qdrant_health(),
            check_gemini_health(),
            return_exceptions=False,
        )

        # Determine overall status
        overall_status = "healthy"
        if qdrant_status["status"] != "healthy":
            overall_status = "degraded"
        if gemini_status["status"] != "healthy":
            overall_status = "degraded"

        result = {
            "status": overall_status,
            "components": {
                "qdrant": qdrant_status,
                "gemini": gemini_status,
            },
        }

        logger.info(
            "Health check completed",
            extra={
                "operation": "check_all_dependencies",
                "overall_status": overall_status,
                "qdrant": qdrant_status["status"],
                "gemini": gemini_status["status"],
            },
        )

        return result

    except Exception as e:
        logger.error(
            f"Health check failed: {str(e)}",
            extra={
                "operation": "check_all_dependencies",
                "error": str(e),
            },
        )
        return {
            "status": "unhealthy",
            "error": str(e),
            "components": {},
        }
