---
name: "environment-configuration"
description: "Manage environment variables, secrets, and configuration files for development, staging, and production environments. Set up .env files, configure services, and ensure secure secrets management."
version: "1.0.0"
---

# Environment Configuration Skill

## When to Use This Skill

- Setting up project environment variables
- Creating .env files for different environments
- Configuring database connections
- Managing API keys and secrets
- Setting up authentication credentials
- Configuring external services (OpenAI, Qdrant, Neon)
- Documenting environment setup

## How This Skill Works

1. **Identify Requirements**: List all required environment variables
2. **Create .env Template**: Provide example values with descriptions
3. **Document Variables**: Explain what each variable does
4. **Secure Secrets**: Ensure sensitive data is not committed
5. **Environment Separation**: Create configs for dev, staging, production

## Environment Variables Structure

### Backend (.env for FastAPI)

```env
# ==================================================
# DATABASE CONFIGURATION
# ==================================================

# Neon Serverless Postgres
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require
DATABASE_POOL_SIZE=20
DATABASE_MAX_OVERFLOW=10

# ==================================================
# VECTOR DATABASE CONFIGURATION
# ==================================================

# Qdrant Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=chapter_embeddings

# ==================================================
# AI SERVICES CONFIGURATION
# ==================================================

# OpenAI API
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Alternative: Azure OpenAI
# AZURE_OPENAI_ENDPOINT=https://your-resource.openai.azure.com/
# AZURE_OPENAI_API_KEY=your-azure-key
# AZURE_OPENAI_DEPLOYMENT_NAME=gpt-4

# ==================================================
# AUTHENTICATION CONFIGURATION
# ==================================================

# Better-Auth
BETTER_AUTH_SECRET=your-secret-key-min-32-characters-long-very-secure
BETTER_AUTH_URL=http://localhost:3000

# JWT Configuration
JWT_SECRET_KEY=your-jwt-secret-key-here
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
REFRESH_TOKEN_EXPIRE_DAYS=7

# ==================================================
# API CONFIGURATION
# ==================================================

# FastAPI Server
API_HOST=0.0.0.0
API_PORT=8000
API_RELOAD=True
DEBUG=True

# CORS Origins (comma-separated)
CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# API Rate Limiting
RATE_LIMIT_PER_MINUTE=60

# ==================================================
# EMAIL CONFIGURATION (Optional)
# ==================================================

# SMTP for email verification
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASSWORD=your-app-password
SMTP_FROM_EMAIL=noreply@robotics-book.com
SMTP_FROM_NAME=Robotics Textbook Platform

# ==================================================
# CACHE CONFIGURATION (Optional)
# ==================================================

# Redis for caching
REDIS_URL=redis://localhost:6379/0
CACHE_TTL_SECONDS=3600

# ==================================================
# MONITORING & LOGGING
# ==================================================

# Logging Level
LOG_LEVEL=INFO
LOG_FORMAT=json

# Sentry Error Tracking (Optional)
SENTRY_DSN=https://your-sentry-dsn@sentry.io/project-id

# ==================================================
# FEATURE FLAGS
# ==================================================

ENABLE_PERSONALIZATION=true
ENABLE_TRANSLATION=true
ENABLE_CHATBOT=true
```

### Frontend (.env for Docusaurus/React)

```env
# ==================================================
# API CONFIGURATION
# ==================================================

# Backend API URL
REACT_APP_API_URL=http://localhost:8000
# or for production:
# REACT_APP_API_URL=https://api.robotics-book.com

# ==================================================
# AUTHENTICATION
# ==================================================

REACT_APP_BETTER_AUTH_URL=http://localhost:3000

# ==================================================
# ANALYTICS (Optional)
# ==================================================

# Google Analytics
REACT_APP_GA_TRACKING_ID=G-XXXXXXXXXX

# ==================================================
# FEATURE FLAGS
# ==================================================

REACT_APP_ENABLE_CHATBOT=true
REACT_APP_ENABLE_PERSONALIZATION=true
REACT_APP_ENABLE_TRANSLATION=true
```

## Example .env Files by Environment

### Development (.env.development)

```env
# Development Environment

DATABASE_URL=postgresql://dev_user:dev_pass@localhost:5432/robotics_book_dev
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=dev-key

OPENAI_API_KEY=sk-dev-key-here

BETTER_AUTH_SECRET=dev-secret-at-least-32-chars
JWT_SECRET_KEY=dev-jwt-secret

API_HOST=0.0.0.0
API_PORT=8000
DEBUG=True
CORS_ORIGINS=http://localhost:3000

LOG_LEVEL=DEBUG
```

### Staging (.env.staging)

```env
# Staging Environment

DATABASE_URL=postgresql://staging_user:staging_pass@staging-db.neon.tech/robotics_book_staging?sslmode=require
QDRANT_URL=https://staging-qdrant.example.com
QDRANT_API_KEY=staging-qdrant-key

OPENAI_API_KEY=sk-staging-key-here

BETTER_AUTH_SECRET=staging-secret-very-long-and-secure-key
JWT_SECRET_KEY=staging-jwt-secret-key

API_HOST=0.0.0.0
API_PORT=8000
DEBUG=False
CORS_ORIGINS=https://staging.robotics-book.com

LOG_LEVEL=INFO
SENTRY_DSN=https://your-sentry-dsn@sentry.io/staging
```

### Production (.env.production)

```env
# Production Environment

DATABASE_URL=postgresql://prod_user:prod_pass@prod-db.neon.tech/robotics_book_prod?sslmode=require
QDRANT_URL=https://qdrant.robotics-book.com
QDRANT_API_KEY=prod-qdrant-key-keep-secret

OPENAI_API_KEY=sk-prod-key-here

BETTER_AUTH_SECRET=production-secret-super-long-secure-random-string
JWT_SECRET_KEY=production-jwt-secret-key-also-very-secure

API_HOST=0.0.0.0
API_PORT=8000
DEBUG=False
CORS_ORIGINS=https://robotics-book.com,https://www.robotics-book.com

LOG_LEVEL=WARNING
SENTRY_DSN=https://your-sentry-dsn@sentry.io/production

RATE_LIMIT_PER_MINUTE=30
```

## .env.example Template

```env
# ==================================================
# ENVIRONMENT CONFIGURATION TEMPLATE
# ==================================================
# Copy this file to .env and fill in your actual values
# DO NOT commit .env files with real secrets to git!

# Database
DATABASE_URL=postgresql://user:password@host/dbname
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key

# AI Services
OPENAI_API_KEY=sk-your-key-here

# Authentication
BETTER_AUTH_SECRET=your-secret-min-32-chars
JWT_SECRET_KEY=your-jwt-secret

# API
API_PORT=8000
DEBUG=True
CORS_ORIGINS=http://localhost:3000

# Optional
SMTP_HOST=smtp.gmail.com
SMTP_USER=your-email@gmail.com
SMTP_PASSWORD=your-app-password
```

## Configuration Loading (Python)

```python
# config.py
from pydantic_settings import BaseSettings
from typing import List
import os

class Settings(BaseSettings):
    # Database
    database_url: str
    database_pool_size: int = 20

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "chapter_embeddings"

    # OpenAI
    openai_api_key: str
    openai_model: str = "gpt-4-turbo-preview"
    openai_embedding_model: str = "text-embedding-3-small"

    # Authentication
    better_auth_secret: str
    jwt_secret_key: str
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # API
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    debug: bool = False
    cors_origins: List[str] = ["http://localhost:3000"]

    # Logging
    log_level: str = "INFO"

    class Config:
        env_file = ".env"
        case_sensitive = False

# Create global settings instance
settings = Settings()

# Usage in code:
# from config import settings
# database_url = settings.database_url
```

## Configuration Loading (TypeScript)

```typescript
// config.ts
interface AppConfig {
  api: {
    url: string;
    timeout: number;
  };
  auth: {
    betterAuthUrl: string;
  };
  features: {
    chatbot: boolean;
    personalization: boolean;
    translation: boolean;
  };
}

export const config: AppConfig = {
  api: {
    url: process.env.REACT_APP_API_URL || 'http://localhost:8000',
    timeout: 30000,
  },
  auth: {
    betterAuthUrl: process.env.REACT_APP_BETTER_AUTH_URL || 'http://localhost:3000',
  },
  features: {
    chatbot: process.env.REACT_APP_ENABLE_CHATBOT === 'true',
    personalization: process.env.REACT_APP_ENABLE_PERSONALIZATION === 'true',
    translation: process.env.REACT_APP_ENABLE_TRANSLATION === 'true',
  },
};

// Usage:
// import { config } from './config';
// const apiUrl = config.api.url;
```

## Secrets Management Best Practices

### 1. Never Commit Secrets

```gitignore
# .gitignore
.env
.env.local
.env.development.local
.env.test.local
.env.production.local
.env.*.local

# Only commit example file
!.env.example
```

### 2. Use Secret Management Services

**For Production:**
- **AWS Secrets Manager**: Store secrets in AWS
- **HashiCorp Vault**: Enterprise secret management
- **Doppler**: Modern secrets management platform
- **Environment Variables**: Set directly in hosting platform

### 3. Rotate Secrets Regularly

```bash
# Example: Rotate JWT secret
NEW_SECRET=$(openssl rand -base64 32)
echo "JWT_SECRET_KEY=$NEW_SECRET" >> .env
```

### 4. Validate Required Variables

```python
# validate_env.py
import os
import sys

REQUIRED_VARS = [
    "DATABASE_URL",
    "OPENAI_API_KEY",
    "BETTER_AUTH_SECRET",
    "JWT_SECRET_KEY"
]

def validate_environment():
    missing = []
    for var in REQUIRED_VARS:
        if not os.getenv(var):
            missing.append(var)

    if missing:
        print("ERROR: Missing required environment variables:")
        for var in missing:
            print(f"  - {var}")
        sys.exit(1)

    print("✓ All required environment variables are set")

if __name__ == "__main__":
    validate_environment()
```

## Docker Environment Configuration

```dockerfile
# Dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY . .

# Expose port
EXPOSE 8000

# Run validation before starting
CMD python validate_env.py && uvicorn app.main:app --host 0.0.0.0 --port 8000
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  backend:
    build: ./backend
    ports:
      - "8000:8000"
    env_file:
      - .env.development
    environment:
      - DATABASE_URL=${DATABASE_URL}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
    depends_on:
      - postgres
      - qdrant

  postgres:
    image: postgres:15
    environment:
      POSTGRES_USER: dev_user
      POSTGRES_PASSWORD: dev_pass
      POSTGRES_DB: robotics_book_dev
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
    volumes:
      - qdrant_data:/qdrant/storage

volumes:
  postgres_data:
  qdrant_data:
```

## Setup Scripts

### Backend Setup

```bash
#!/bin/bash
# setup-backend.sh

echo "Setting up backend environment..."

# Check if .env exists
if [ ! -f .env ]; then
    echo "Creating .env from template..."
    cp .env.example .env
    echo "Please edit .env and add your actual credentials"
    exit 1
fi

# Validate environment
python validate_env.py

# Install dependencies
pip install -r requirements.txt

# Run database migrations
python migrate.py

# Setup Qdrant collection
python setup_qdrant.py

echo "✓ Backend setup complete!"
```

### Frontend Setup

```bash
#!/bin/bash
# setup-frontend.sh

echo "Setting up frontend environment..."

# Check if .env exists
if [ ! -f .env ]; then
    echo "Creating .env from template..."
    cp .env.example .env
    echo "Please edit .env and add your API URL"
fi

# Install dependencies
npm install

echo "✓ Frontend setup complete!"
echo "Run 'npm start' to start the development server"
```

## Environment Variable Documentation

Create `ENVIRONMENT.md`:

```markdown
# Environment Variables Documentation

## Required Variables

### DATABASE_URL
- **Description**: PostgreSQL connection string for Neon database
- **Format**: `postgresql://user:password@host/dbname?sslmode=require`
- **Example**: `postgresql://user:pass@ep-cool-name-123456.neon.tech/robotics_book?sslmode=require`
- **Where to get**: Neon dashboard → Connection Details

### OPENAI_API_KEY
- **Description**: API key for OpenAI services (embeddings, chat)
- **Format**: `sk-...`
- **Example**: `sk-proj-abc123xyz...`
- **Where to get**: https://platform.openai.com/api-keys

### BETTER_AUTH_SECRET
- **Description**: Secret key for Better-Auth session encryption
- **Format**: Minimum 32 characters, random string
- **Example**: `super-secret-key-min-32-chars-long`
- **How to generate**: `openssl rand -base64 32`

## Optional Variables

### SENTRY_DSN
- **Description**: Sentry error tracking URL
- **Required**: No (production recommended)
- **Where to get**: Sentry project settings

### REDIS_URL
- **Description**: Redis connection for caching
- **Required**: No (improves performance)
- **Format**: `redis://host:port/db`
```

## Checklist for Environment Setup

- ✓ Copy `.env.example` to `.env`
- ✓ Fill in all required variables
- ✓ Generate secure secrets (min 32 chars)
- ✓ Set correct database URLs
- ✓ Configure CORS origins
- ✓ Add `.env` to `.gitignore`
- ✓ Validate environment variables
- ✓ Document variables in ENVIRONMENT.md
- ✓ Set production secrets in hosting platform
- ✓ Enable SSL for database connections

## Output Format

When creating environment configuration, provide:

1. **Complete .env.example**: All variables with example values
2. **Environment-specific configs**: Dev, staging, production
3. **Configuration loader**: Python/TypeScript code to load settings
4. **Validation script**: Check required variables are set
5. **Documentation**: Explain what each variable does and where to get it
6. **Setup scripts**: Automate environment setup
