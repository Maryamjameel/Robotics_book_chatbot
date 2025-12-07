"""Configuration and logging setup for the embedding pipeline."""

import json
import logging
import logging.handlers
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv


class JSONFormatter(logging.Formatter):
    """Custom JSON formatter for structured logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "operation": getattr(record, "operation", None),
            "status": getattr(record, "status", None),
        }

        # Add exception info if present
        if record.exc_info:
            log_data["error_details"] = {
                "type": record.exc_info[0].__name__,
                "message": str(record.exc_info[1]),
            }

        # Add any custom attributes
        for key, value in record.__dict__.items():
            if key not in [
                "name",
                "msg",
                "args",
                "created",
                "filename",
                "funcName",
                "levelname",
                "levelno",
                "lineno",
                "module",
                "msecs",
                "message",
                "pathname",
                "process",
                "processName",
                "relativeCreated",
                "thread",
                "threadName",
                "exc_info",
                "exc_text",
                "stack_info",
                "operation",
                "status",
            ]:
                log_data[key] = value

        return json.dumps(log_data)


class Config:
    """Configuration for the embedding pipeline."""

    def __init__(self):
        """Initialize configuration from environment variables."""
        # Load .env file
        env_path = Path(__file__).parent.parent.parent / ".env"
        if env_path.exists():
            load_dotenv(env_path)

        # Google Gemini Configuration (for both embeddings and chat)
        self.gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
        self.gemini_embedding_model: str = os.getenv("GEMINI_EMBEDDING_MODEL", "text-embedding-004")
        self.gemini_chat_model: str = os.getenv("GEMINI_CHAT_MODEL", "gemini-2.0-flash")

        # Qdrant Configuration
        self.qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
        self.collection_name: str = os.getenv("COLLECTION_NAME", "robotics_chapters")

        # Batch Processing Configuration
        self.embedding_batch_size: int = int(os.getenv("EMBEDDING_BATCH_SIZE", "32"))
        self.max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
        self.initial_backoff_seconds: float = float(os.getenv("INITIAL_BACKOFF_SECONDS", "1"))

        # Text Chunking Configuration
        self.min_chunk_tokens: int = int(os.getenv("MIN_CHUNK_TOKENS", "50"))
        self.max_chunk_tokens: int = int(os.getenv("MAX_CHUNK_TOKENS", "512"))

        # Logging Configuration
        self.log_level: str = os.getenv("LOG_LEVEL", "INFO")
        self.log_file: str = os.getenv("LOG_FILE", "pipeline.log")
        self.log_to_console: bool = os.getenv("LOG_TO_CONSOLE", "true").lower() == "true"

        # Vector Configuration (Gemini text-embedding-004 produces 768-dim vectors)
        self.vector_size: int = 768
        self.distance_metric: str = "cosine"

    def validate(self) -> bool:
        """Validate critical configuration values."""
        if not self.gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable must be set")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable must be set")
        return True


def setup_logging(config: Config) -> logging.Logger:
    """Configure logging with JSON formatting."""
    logger = logging.getLogger("embedding_pipeline")
    logger.setLevel(getattr(logging, config.log_level.upper()))

    # Remove any existing handlers
    logger.handlers.clear()

    # Create formatters
    json_formatter = JSONFormatter()

    # File handler with rotation
    log_dir = Path(__file__).parent.parent.parent / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / config.log_file

    file_handler = logging.handlers.RotatingFileHandler(
        log_path, maxBytes=10 * 1024 * 1024, backupCount=5  # 10MB per file
    )
    file_handler.setFormatter(json_formatter)
    logger.addHandler(file_handler)

    # Console handler if enabled
    if config.log_to_console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(json_formatter)
        logger.addHandler(console_handler)

    return logger


# Global configuration and logger instances
config = Config()
logger = setup_logging(config)
