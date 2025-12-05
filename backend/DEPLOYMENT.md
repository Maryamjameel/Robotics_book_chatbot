# RAG Chatbot API - Deployment Guide

## Pre-Deployment Checklist

### Code Quality
- [ ] All tests passing: `make test`
- [ ] No linting issues: `make lint`
- [ ] Code formatted: `make format`
- [ ] Type checking passing: `make type-check`
- [ ] Coverage >80%: `make coverage`
- [ ] No uncommitted changes: `git status`
- [ ] All commits pushed: `git push`

### Configuration
- [ ] `.env` file created with all required variables
- [ ] All secrets using environment variables (not hardcoded)
- [ ] API keys validated with quota available:
  - [ ] OpenAI API key (embeddings)
  - [ ] Gemini API key (generation)
  - [ ] Qdrant API key (vector search)
- [ ] Qdrant collection initialized with embeddings
- [ ] Log directory writable: `mkdir -p logs`

### Dependencies
- [ ] Python 3.11+ installed
- [ ] Poetry installed
- [ ] All dependencies resolve: `poetry install`
- [ ] No security vulnerabilities: `poetry audit`
- [ ] Lock file up-to-date: `poetry lock --no-update`

### External Services
- [ ] Qdrant instance accessible and running
  - [ ] URL reachable from deployment environment
  - [ ] API key authenticated and authorized
  - [ ] Collection exists: `GET /collections/{collection_name}`
  - [ ] Has embeddings: `point_count > 0`
- [ ] OpenAI API endpoint accessible
  - [ ] Rate limit quota available
  - [ ] Model available: `text-embedding-3-small`
- [ ] Gemini API endpoint accessible
  - [ ] Rate limit quota available
  - [ ] Model available: `gemini-1.5-flash`

## Local Development Setup

### 1. Install Dependencies
```bash
cd backend
make dev
```

### 2. Configure Environment
```bash
cp .env.example .env
# Edit .env with your credentials
nano .env
```

### 3. Run with Docker (Recommended)
```bash
# Start Qdrant + API
docker-compose up -d

# Check health
make health

# View logs
docker-compose logs -f api
```

### 4. Run Locally (Without Docker)
```bash
# Ensure Qdrant is running externally
# Update QDRANT_URL in .env

# Start API server
make run

# In another terminal, check health
make health
```

## Running Tests Before Deployment

```bash
# Unit tests (fast, no external dependencies)
make test-unit

# Integration tests (requires external services)
make test-int

# Full test suite with coverage
make test

# Full quality check (format, lint, type, test, coverage)
make check-full
```

## Docker Deployment

### Build Image
```bash
docker build -t rag-chatbot-api:latest .
```

### Run Container
```bash
docker run \
  --env-file .env \
  -p 8000:8000 \
  -v logs:/app/logs \
  rag-chatbot-api:latest
```

### Using Docker Compose
```bash
# Start all services
docker-compose up -d

# Check logs
docker-compose logs -f api

# Stop all services
docker-compose down
```

## Health Verification

After deployment, verify all systems are operational:

```bash
# Basic liveness check (should respond immediately)
curl http://localhost:8000/health
# Response: {"status": "ok"}

# Full dependency check
curl http://localhost:8000/health/full
# Response should show all components "healthy"

# Specific service checks
curl http://localhost:8000/health/qdrant
curl http://localhost:8000/health/gemini

# Check OpenAPI documentation
curl http://localhost:8000/docs

# Test chat endpoint
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is forward kinematics?"}'
```

## Monitoring and Logging

### Log Files
- Location: `logs/pipeline.log`
- Format: JSON (structured logging)
- Rotate: 10MB per file (configurable)

### Monitor Real-Time Logs
```bash
# From Docker
docker-compose logs -f api

# From local file
tail -f logs/pipeline.log

# Filter errors
tail -f logs/pipeline.log | grep -E "ERROR|WARNING"
```

### Key Metrics to Monitor
- API response latency (search_latency_ms + generation_latency_ms)
- Error rate (403, 429, 503 responses)
- Qdrant connectivity (health checks)
- Gemini API quota usage
- Rate limit hits (429 responses)

## Scaling Configuration

### For Development (Single Worker)
```bash
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
```

### For Production (Multiple Workers)
```bash
# 4 workers for quad-core CPU
python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4

# Behind a reverse proxy (nginx, etc.)
# - Set workers based on CPU cores
# - Enable keep-alive connections
# - Use health endpoint for load balancer checks
```

## Reverse Proxy Configuration (Nginx)

```nginx
upstream rag_api {
    least_conn;
    server localhost:8000 max_fails=3 fail_timeout=30s;
    server localhost:8001 max_fails=3 fail_timeout=30s;
    server localhost:8002 max_fails=3 fail_timeout=30s;
}

server {
    listen 80;
    server_name your-domain.com;

    # Health check endpoint
    location /health {
        access_log off;
        proxy_pass http://rag_api;
        proxy_http_version 1.1;
        proxy_set_header Connection "";
    }

    # API endpoints
    location /api/ {
        proxy_pass http://rag_api;
        proxy_http_version 1.1;
        proxy_set_header Connection "";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;

        # Timeouts for long-running requests
        proxy_connect_timeout 10s;
        proxy_send_timeout 30s;
        proxy_read_timeout 30s;
    }

    # Documentation
    location /docs {
        proxy_pass http://rag_api;
        proxy_http_version 1.1;
        proxy_set_header Connection "";
    }
}
```

## Kubernetes Deployment Example

### Deployment YAML
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: rag-chatbot-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: rag-chatbot-api
  template:
    metadata:
      labels:
        app: rag-chatbot-api
    spec:
      containers:
      - name: api
        image: rag-chatbot-api:latest
        ports:
        - containerPort: 8000
        env:
        - name: OPENAI_API_KEY
          valueFrom:
            secretKeyRef:
              name: openai-secrets
              key: api-key
        - name: GEMINI_API_KEY
          valueFrom:
            secretKeyRef:
              name: gemini-secrets
              key: api-key
        - name: QDRANT_URL
          value: "http://qdrant-service:6333"
        - name: QDRANT_API_KEY
          valueFrom:
            secretKeyRef:
              name: qdrant-secrets
              key: api-key
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health/full
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
```

### Service YAML
```yaml
apiVersion: v1
kind: Service
metadata:
  name: rag-chatbot-api
spec:
  type: LoadBalancer
  selector:
    app: rag-chatbot-api
  ports:
  - port: 80
    targetPort: 8000
    protocol: TCP
```

## Rollback Procedures

### Container Rollback
```bash
# Get previous image version
docker images | grep rag-chatbot-api

# Run previous version
docker run -d \
  --env-file .env \
  -p 8000:8000 \
  rag-chatbot-api:v1.0.0
```

### Docker Compose Rollback
```bash
# Edit docker-compose.yml to point to previous image
# Or use git to revert to previous version
git revert HEAD~1

# Restart with previous configuration
docker-compose down
docker-compose up -d
```

### Kubernetes Rollback
```bash
# List rollout history
kubectl rollout history deployment/rag-chatbot-api

# Rollback to previous version
kubectl rollout undo deployment/rag-chatbot-api

# Rollback to specific revision
kubectl rollout undo deployment/rag-chatbot-api --to-revision=2
```

## Troubleshooting

### API Not Responding
1. Check if service is running: `curl http://localhost:8000/health`
2. Check logs for errors: `tail logs/pipeline.log`
3. Verify ports are open: `netstat -tlnp | grep 8000`
4. Check environment variables: `echo $GEMINI_API_KEY`

### Qdrant Connection Failed
1. Verify URL: `curl http://qdrant-url:6333/health`
2. Check API key: Ensure `QDRANT_API_KEY` is correct
3. Test connectivity: `curl -H "api-key: your-key" http://qdrant-url:6333/health`

### Gemini API Errors
1. Verify API key is valid
2. Check quota: Login to Google Cloud Console
3. Verify model availability: `GEMINI_MODEL=gemini-1.5-flash`
4. Check rate limits in logs

### High Latency
1. Monitor latency metrics in logs
2. Check Qdrant response times
3. Check Gemini API response times
4. Increase worker count if CPU-bound
5. Optimize chunk size if I/O-bound

## Performance Tuning

### Request Timeout
- Default: 30 seconds
- Adjustable in chat endpoint or reverse proxy
- Consider increasing for large context windows

### Rate Limiting
- Current: 5 requests per second (asyncio.Semaphore)
- Adjustable in RAGService configuration
- Matches Gemini API quotas

### Batch Processing
- Embedding batch size: 32 (configurable)
- Qdrant search limit: 10 (top_k * 2 for threshold filtering)
- Confidence scoring: Real-time calculation

## Security Considerations

1. **API Keys**: Always use environment variables, never commit to git
2. **CORS**: Configure for your domain, not wildcard in production
3. **Rate Limiting**: Enforced per-second via asyncio.Semaphore
4. **Input Validation**: Pydantic validates all inputs (1-2000 char questions)
5. **Error Messages**: Generic messages in production, detailed logs for debugging
6. **HTTPS**: Use TLS/SSL in production (configure in reverse proxy)

## Support and Escalation

- **API Issues**: Check `/health/full` endpoint
- **Search Issues**: Verify Qdrant collection and embeddings
- **Generation Issues**: Check Gemini API quota and logs
- **Performance**: Monitor latency metrics in JSON logs
- **Production Incidents**: Check rollback procedures above
