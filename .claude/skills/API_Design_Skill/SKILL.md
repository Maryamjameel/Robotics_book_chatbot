---
name: "api-design"
description: "Design RESTful API endpoints with proper request/response schemas, HTTP methods, status codes, and OpenAPI documentation. Use when creating new backend routes or refactoring existing APIs."
version: "1.0.0"
---

# API Design Skill

## When to Use This Skill

- Designing new REST API endpoints
- Creating OpenAPI/Swagger documentation
- Defining Pydantic request/response schemas
- Standardizing API error responses
- Planning API versioning strategy
- Designing webhook endpoints
- Creating API authentication flows

## How This Skill Works

1. **Analyze Requirements**: Understand the feature and its data flow
2. **Design Endpoints**: Define HTTP methods, paths, and parameters
3. **Create Schemas**: Build Pydantic models for request/response validation
4. **Document API**: Generate OpenAPI specs with examples
5. **Error Handling**: Define error responses and status codes
6. **Security**: Add authentication and authorization requirements

## API Design Principles

### RESTful Conventions
- Use nouns for resources: `/users`, `/chapters`, `/chatbot`
- Use HTTP methods semantically:
  - GET: Retrieve data
  - POST: Create or complex operations
  - PUT: Full update
  - PATCH: Partial update
  - DELETE: Remove data
- Use plural nouns: `/chapters/5` not `/chapter/5`
- Version APIs: `/api/v1/chapters`

### Status Codes
- **200 OK**: Successful GET/PUT/PATCH
- **201 Created**: Successful POST (resource created)
- **204 No Content**: Successful DELETE
- **400 Bad Request**: Invalid input
- **401 Unauthorized**: Not authenticated
- **403 Forbidden**: Not authorized
- **404 Not Found**: Resource doesn't exist
- **422 Unprocessable Entity**: Validation error
- **500 Internal Server Error**: Server error

## Example API Designs

### 1. Chatbot Question Endpoint

**Endpoint Design**:
```
POST /api/v1/chatbot/ask
```

**Request Schema** (Pydantic):
```python
from pydantic import BaseModel, Field
from typing import Optional

class ChatbotRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=500, description="User's question")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Text selected by user")
    chapter_id: Optional[int] = Field(None, description="Specific chapter to search")

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is inverse kinematics?",
                "selected_text": "The robot arm has 6 degrees of freedom...",
                "chapter_id": 3
            }
        }
```

**Response Schema**:
```python
from typing import List, Dict

class ChatbotSource(BaseModel):
    chapter_id: int
    chapter_title: str
    section: str
    relevance_score: float

class ChatbotResponse(BaseModel):
    answer: str = Field(..., description="AI-generated answer")
    sources: List[ChatbotSource] = Field(default_factory=list)
    confidence: float = Field(..., ge=0.0, le=1.0)
    processing_time_ms: int

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "Inverse kinematics computes joint angles...",
                "sources": [
                    {
                        "chapter_id": 3,
                        "chapter_title": "Kinematics",
                        "section": "3.4 Inverse Kinematics",
                        "relevance_score": 0.92
                    }
                ],
                "confidence": 0.88,
                "processing_time_ms": 450
            }
        }
```

**FastAPI Route**:
```python
from fastapi import APIRouter, Depends, HTTPException

router = APIRouter(prefix="/chatbot", tags=["chatbot"])

@router.post("/ask", response_model=ChatbotResponse)
async def ask_question(
    request: ChatbotRequest,
    current_user = Depends(get_current_user)  # Optional auth
):
    """
    Ask a question about the robotics textbook.

    - **question**: Your question (1-500 characters)
    - **selected_text**: Optionally provide selected text for context
    - **chapter_id**: Optionally restrict search to specific chapter
    """
    try:
        result = await chatbot_service.answer_question(
            question=request.question,
            selected_text=request.selected_text,
            chapter_id=request.chapter_id
        )
        return result
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail="Internal server error")
```

### 2. Personalization Endpoint

**Endpoint Design**:
```
POST /api/v1/personalization/personalize
```

**Request Schema**:
```python
class PersonalizationRequest(BaseModel):
    chapter_id: int = Field(..., description="Chapter to personalize")
    content: Optional[str] = Field(None, description="Original content (if not using stored)")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": 5,
                "content": "# Chapter 5: Inverse Kinematics\n\nThis chapter covers..."
            }
        }
```

**Response Schema**:
```python
class PersonalizationResponse(BaseModel):
    chapter_id: int
    personalized_content: str
    personalization_factors: Dict[str, str] = Field(
        description="What aspects were personalized"
    )
    cached: bool = Field(description="Whether response was cached")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": 5,
                "personalized_content": "# Chapter 5: IK with Python Examples\n\n...",
                "personalization_factors": {
                    "experience_level": "intermediate",
                    "software_background": "Python",
                    "added_examples": "NumPy-based implementations"
                },
                "cached": False
            }
        }
```

### 3. Translation Endpoint

**Endpoint Design**:
```
POST /api/v1/translation/translate
```

**Request Schema**:
```python
class TranslationRequest(BaseModel):
    chapter_id: int
    target_language: str = Field(..., pattern="^(ur|en)$", description="Target language code")
    force_refresh: bool = Field(default=False, description="Bypass cache")

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_id": 3,
                "target_language": "ur",
                "force_refresh": False
            }
        }
```

**Response Schema**:
```python
class TranslationResponse(BaseModel):
    chapter_id: int
    source_language: str
    target_language: str
    translated_content: str
    cached: bool
    translation_version: int
```

### 4. User Profile Endpoints

**Get Profile**:
```
GET /api/v1/users/me/profile
Response: UserProfileResponse
```

**Update Profile**:
```
PATCH /api/v1/users/me/profile
Request: UpdateProfileRequest
Response: UserProfileResponse
```

**Schemas**:
```python
class UserProfileResponse(BaseModel):
    user_id: int
    email: str
    name: str
    software_background: Optional[str]
    hardware_background: Optional[str]
    experience_level: str
    preferred_topics: List[str]
    joined_at: str

class UpdateProfileRequest(BaseModel):
    name: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    experience_level: Optional[str] = None
    preferred_topics: Optional[List[str]] = None
```

## Error Response Standard

**Error Schema**:
```python
class ErrorDetail(BaseModel):
    field: Optional[str]
    message: str
    error_code: Optional[str]

class ErrorResponse(BaseModel):
    error: str
    message: str
    details: Optional[List[ErrorDetail]] = None
    request_id: Optional[str]

    class Config:
        json_schema_extra = {
            "example": {
                "error": "ValidationError",
                "message": "Request validation failed",
                "details": [
                    {
                        "field": "question",
                        "message": "Field required",
                        "error_code": "required"
                    }
                ],
                "request_id": "req_abc123"
            }
        }
```

## OpenAPI Documentation

### FastAPI Auto-Documentation
FastAPI automatically generates OpenAPI docs at:
- `/docs` - Swagger UI
- `/redoc` - ReDoc UI
- `/openapi.json` - OpenAPI schema

### Custom Metadata
```python
from fastapi import FastAPI

app = FastAPI(
    title="Robotics Textbook API",
    description="AI-native textbook platform with RAG chatbot, personalization, and translation",
    version="1.0.0",
    contact={
        "name": "Support Team",
        "email": "support@robotics-book.com",
    },
    license_info={
        "name": "MIT",
    },
)
```

### Tag Descriptions
```python
tags_metadata = [
    {
        "name": "chatbot",
        "description": "RAG-based question answering system",
    },
    {
        "name": "personalization",
        "description": "Content personalization based on user profile",
    },
    {
        "name": "translation",
        "description": "Chapter translation to multiple languages",
    },
    {
        "name": "auth",
        "description": "User authentication and authorization",
    },
]

app = FastAPI(openapi_tags=tags_metadata)
```

## API Versioning Strategy

### URL Versioning (Recommended)
```
/api/v1/chatbot/ask
/api/v2/chatbot/ask
```

### Implementation
```python
# v1 router
v1_router = APIRouter(prefix="/api/v1")
v1_router.include_router(chatbot_v1.router)

# v2 router
v2_router = APIRouter(prefix="/api/v2")
v2_router.include_router(chatbot_v2.router)

app.include_router(v1_router)
app.include_router(v2_router)
```

## Security Considerations

### Authentication
```python
from fastapi.security import HTTPBearer

security = HTTPBearer()

@router.post("/protected-endpoint")
async def protected_route(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    current_user = Depends(get_current_user)
):
    # Only authenticated users can access
    pass
```

### Rate Limiting
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@router.post("/chatbot/ask")
@limiter.limit("10/minute")
async def ask_question(request: Request, data: ChatbotRequest):
    # Limit to 10 requests per minute
    pass
```

### Input Validation
```python
from pydantic import validator

class ChatbotRequest(BaseModel):
    question: str

    @validator('question')
    def validate_question(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('Question cannot be empty')
        if any(char in v for char in ['<', '>', 'script']):
            raise ValueError('Invalid characters detected')
        return v.strip()
```

## Testing API Design

### Endpoint Tests
```python
from fastapi.testclient import TestClient

client = TestClient(app)

def test_chatbot_ask():
    response = client.post(
        "/api/v1/chatbot/ask",
        json={
            "question": "What is inverse kinematics?",
            "selected_text": None,
            "chapter_id": None
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "confidence" in data
```

## Best Practices Checklist

- ✓ Use consistent naming conventions
- ✓ Version your APIs
- ✓ Validate all inputs with Pydantic
- ✓ Return appropriate HTTP status codes
- ✓ Provide clear error messages
- ✓ Include request/response examples
- ✓ Implement authentication where needed
- ✓ Add rate limiting to prevent abuse
- ✓ Document all endpoints with OpenAPI
- ✓ Handle errors gracefully
- ✓ Test all endpoints thoroughly

## Output Format

When designing an API, provide:

1. **Endpoint Specification**:
   - HTTP method and path
   - Authentication requirements
   - Rate limiting rules

2. **Request Schema**:
   - Pydantic model with validation
   - Field descriptions and examples
   - Validation rules

3. **Response Schema**:
   - Success response model
   - Error response model
   - Status codes

4. **Implementation**:
   - FastAPI route handler
   - Error handling
   - Dependencies

5. **Documentation**:
   - Endpoint description
   - Example requests/responses
   - Usage notes
