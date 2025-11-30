<!--
  SYNC IMPACT REPORT
  Version Change: None → 1.0.0
  Rationale: Initial constitution creation for Physical AI & Humanoid Robotics Textbook project

  Modified Principles: N/A (initial creation)
  Added Sections: All core principles, technology standards, testing requirements, security standards, development workflow, governance
  Removed Sections: None

  Templates Requiring Updates:
  ✅ plan-template.md - Constitution Check section will reference these principles
  ✅ spec-template.md - Aligns with functional requirements and user story structure
  ✅ tasks-template.md - Aligns with testing requirements and task structure

  Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Production-Grade Quality (NON-NEGOTIABLE)

All code MUST meet production standards before merge:

- **Comprehensive Error Handling**: All API endpoints, database operations, and external service calls MUST handle errors gracefully with user-friendly messages and appropriate logging
- **Type Safety**: TypeScript strict mode enabled; Python type hints required for all functions and class methods
- **Testing Before Deployment**: All critical paths (authentication, RAG queries, personalization, translation) MUST have automated tests with 80%+ coverage
- **Graceful Degradation**: System MUST remain functional when non-critical services fail (e.g., translation service down does not break content reading)
- **Performance Monitoring**: All API endpoints MUST complete within defined SLAs (RAG < 3s, page load < 2s, auth < 1s)

**Rationale**: As a Panaversity educational product targeting students and professionals, reliability and user experience are paramount. Production quality ensures the textbook platform can scale and serve learners effectively.

### II. Privacy-First & GDPR-Compliant

User data collection and handling MUST follow privacy-first principles:

- **Minimal Collection**: Collect ONLY what enables core functionality: email, name, software background (beginner/intermediate/advanced), hardware background (no hardware/basic/advanced)
- **No PII Beyond Necessity**: No phone numbers, addresses, or sensitive personal information stored
- **Transparent Data Usage**: Signup flow MUST clearly explain how background data enables personalization
- **Right to be Forgotten**: Users MUST be able to export all their data (JSON format) and request complete deletion within 48 hours
- **Secure Storage**: All passwords hashed with bcrypt (cost factor 12+); sessions use secure HTTP-only cookies with SameSite=Strict
- **Data Retention**: Inactive accounts (no login for 2+ years) flagged for deletion; users notified 30 days before auto-deletion

**Rationale**: Trust is essential for educational platforms. Students must feel safe providing learning background information. GDPR compliance ensures global accessibility and ethical data practices.

### III. RAG Accuracy & Source Citation (HIGH PRIORITY)

The RAG chatbot MUST prioritize accuracy and transparency:

- **Mandatory Source Citations**: Every chatbot response MUST cite the specific book chapter and section (e.g., "Source: Chapter 3, Section 3.2 - Inverse Kinematics")
- **Confidence Scoring**: Responses with confidence < 80% MUST display "I'm not entirely certain, but based on [section]..." and suggest manual review
- **Selected Text Priority**: When users highlight text and ask questions, RAG MUST prioritize that selected text over full-book retrieval (boost selected chunk relevance by 2x)
- **Hallucination Detection**: Answers MUST be verified against retrieved chunks; if LLM generates content not present in chunks, flag as "uncertain" and show chunk sources for user verification
- **Relevance Threshold**: Minimum cosine similarity 0.7 for chunk retrieval; below threshold triggers "I couldn't find relevant information in the textbook" response
- **Context Window Management**: Include chapter title + section title + paragraph content in chunks; retrieve top-5 chunks, rerank by relevance

**Rationale**: Educational content requires accuracy. Incorrect information in robotics education can lead to failed projects or unsafe hardware implementations. Citations enable students to verify and deepen understanding.

### IV. Modular & Testable Architecture

System architecture MUST support independent development and testing:

- **Decoupled Services**: Frontend (Docusaurus), Backend API (FastAPI), RAG Service, Auth Service, Translation Service MUST operate independently via well-defined APIs
- **API-First Design**: All backend functionality exposed via versioned REST APIs (`/api/v1/...`); OpenAPI specs auto-generated and kept in sync
- **Dependency Injection**: Services MUST use dependency injection for database, vector store, LLM client connections to enable mocking in tests
- **Contract Testing**: All API contracts MUST have tests verifying request/response schemas match OpenAPI spec
- **Integration Testing**: Each service integration point (FastAPI ↔ Neon Postgres, FastAPI ↔ Qdrant, Frontend ↔ FastAPI) MUST have integration tests
- **Stateless Services**: Backend services MUST be stateless; all state in database or vector store to enable horizontal scaling

**Rationale**: Modular architecture enables parallel development (e.g., one developer works on translation while another improves RAG). Testability ensures changes don't break existing functionality. Stateless design enables scaling to thousands of concurrent learners.

### V. Content Quality & Accessibility

Educational content MUST be high-quality and accessible:

- **Content Review**: All chapter content MUST be technically accurate and reviewed before personalization layer is applied
- **Urdu Translation Quality**: Translations MUST be reviewed by native Urdu speakers with technical knowledge; machine-translated content flagged as "draft translation"
- **Personalization Transparency**: Personalized content MUST clearly indicate which sections are adapted vs original; users can toggle "show original content"
- **Progressive Disclosure**: Complex topics MUST follow beginner → intermediate → advanced progression based on user background
- **Responsive Design**: All content MUST be readable on mobile devices (320px width minimum); code snippets horizontally scrollable on small screens
- **Accessibility Standards**: WCAG 2.1 AA compliance for color contrast, keyboard navigation, screen reader support

**Rationale**: Panaversity's mission is inclusive education. Accessibility ensures students with disabilities can learn. Quality translations expand reach to Urdu-speaking students. Personalization makes advanced robotics accessible to beginners.

### VI. Observability & Debugging

All production systems MUST be observable:

- **Structured Logging**: All logs in JSON format with timestamp, service name, log level, user_id (if applicable), request_id (for tracing)
- **RAG Query Logging**: Log every RAG query with: user question, retrieved chunk IDs, relevance scores, final answer, confidence score, processing time
- **Authentication Event Logging**: Log all signup, login, logout, password reset, session expiration events (no passwords logged)
- **Performance Metrics**: Track and alert on: p50/p95/p99 latency for RAG queries, API response times, database query times, vector search times
- **Error Tracking**: All exceptions captured with full stack trace, context, and user impact assessment
- **Monitoring Dashboards**: Real-time visibility into active users, RAG query volume, error rates, service health

**Rationale**: When students encounter issues (e.g., RAG gives wrong answer), observability enables quick diagnosis and improvement. Logging RAG reasoning helps improve chunk retrieval and prompt engineering.

### VII. Spec-Driven Development (SDD)

All features MUST follow the spec-plan-tasks-implement workflow:

- **Specification First**: Every feature starts with `spec.md` defining user stories, acceptance criteria, functional requirements
- **Architecture Planning**: Complex features require `plan.md` documenting technical approach, data models, API contracts, dependencies
- **Task Breakdown**: `tasks.md` MUST break plan into testable, independent tasks with explicit file paths and acceptance criteria
- **Prompt History Records (PHRs)**: Create PHR after every significant development session; route to `history/prompts/constitution/`, `history/prompts/<feature>/`, or `history/prompts/general/`
- **Architectural Decision Records (ADRs)**: Significant architectural decisions (e.g., "Why Qdrant over Pinecone?", "Why FastAPI over Flask?") MUST be documented in `history/adr/` with context, options considered, rationale
- **No Implementation Without Spec**: Features without spec.md are blocked; ad-hoc code changes must be documented retroactively

**Rationale**: Spec-driven development ensures alignment with Panaversity's educational goals. PHRs and ADRs capture decision-making process, valuable for learning and future maintenance. Clear specs enable parallel development.

## Technology Standards

### Frontend (Docusaurus)

- **Version**: Docusaurus 3.x with React 18+
- **Language**: TypeScript with strict mode enabled
- **Styling**: Tailwind CSS v4.x for consistent design system
- **State Management**: React Context API for authentication state; React Query for server state (API calls)
- **Component Testing**: React Testing Library with 70%+ coverage for interactive components (chatbot UI, personalization controls)
- **Build Optimization**: Code splitting by route; lazy loading for chatbot and translation features; bundle size < 500KB initial load

### Backend (FastAPI)

- **Version**: FastAPI 0.110+ with Python 3.11+
- **Type Safety**: Pydantic v2 models for all request/response schemas; mypy strict mode enabled
- **Async/Await**: All I/O operations (database, vector search, LLM calls) MUST be async to prevent blocking
- **API Versioning**: All endpoints prefixed with `/api/v1/`; breaking changes require new version (`/api/v2/`)
- **Documentation**: OpenAPI spec auto-generated at `/api/v1/docs`; kept in sync with implementation via Pydantic models
- **CORS Configuration**: Production environment restricts CORS to `https://yourdomain.github.io`; development allows `localhost:3000`
- **Dependency Management**: Poetry for dependency locking and reproducible builds

### RAG System (OpenAI Agents + Qdrant)

- **LLM Provider**: OpenAI GPT-4 via ChatKit SDK for RAG responses
- **Vector Store**: Qdrant Cloud Free Tier (1GB storage, sufficient for ~200-300 textbook pages)
- **Embedding Model**: OpenAI text-embedding-3-small (1536 dimensions, cost-effective)
- **Chunking Strategy**:
  - Paragraph-level chunks with chapter and section metadata
  - Chunk overlap: 50 tokens to preserve context across boundaries
  - Max chunk size: 512 tokens to fit in retrieval context
- **Retrieval**: Retrieve top-5 chunks via cosine similarity; rerank by relevance to query
- **Context Window**: Include chunk text + chapter title + section title in LLM context
- **Prompt Engineering**: System prompt enforces citation requirement; few-shot examples demonstrate desired answer format

### Database (Neon Serverless Postgres)

- **Version**: PostgreSQL 16+ (Neon's latest stable)
- **ORM**: SQLAlchemy 2.x with async support (asyncpg driver)
- **Migrations**: Alembic for schema versioning; all schema changes MUST have migration scripts
- **Connection Pooling**: Use Neon's built-in pooling; configure max 10 connections for free tier
- **Indexes**: Create indexes on `user_id`, `chapter_id`, `created_at` for common query patterns
- **Backup Strategy**: Neon automatic daily backups; export full DB weekly to S3 for disaster recovery

### Authentication (better-auth.com)

- **Provider**: Better Auth with email/password authentication
- **Session Management**: HTTP-only secure cookies with 7-day expiration; refresh tokens for 30-day persistence
- **Email Verification**: Required for account activation; send verification email within 5 minutes of signup
- **Rate Limiting**: Max 5 login attempts per 15 minutes per IP; exponential backoff after failures
- **User Background Questionnaire**: Embedded in signup flow; fields: software_background (beginner/intermediate/advanced), hardware_background (none/basic/advanced)

## Testing Requirements

### Unit Tests

- **Coverage**: 80%+ for critical business logic (RAG prompt construction, personalization logic, translation routing)
- **Framework**: pytest for backend, Vitest for frontend
- **Mocking**: Mock external dependencies (OpenAI API, Qdrant, Neon) in unit tests using pytest-mock
- **Fast Execution**: Unit test suite MUST run in < 30 seconds for rapid feedback

### Integration Tests

- **RAG End-to-End**: Test full RAG pipeline (user query → vector search → LLM call → response with citations)
- **Authentication Flow**: Test signup → email verification → login → session management
- **Personalization Pipeline**: Test user background → content adaptation → personalized rendering
- **Translation Workflow**: Test translation request → Urdu rendering → toggle back to English
- **Database Operations**: Test CRUD operations against real Neon Postgres (use test database)

### End-to-End (E2E) Tests

- **Framework**: Playwright for browser automation
- **Core User Journeys**:
  1. New user signup → verify email → login → read personalized chapter
  2. Existing user → ask RAG question → receive cited answer
  3. User highlights text → ask question → RAG prioritizes selected text
  4. User clicks "Translate to Urdu" → chapter renders in Urdu
- **Test Environment**: Run against production build deployed to staging environment
- **Execution**: E2E tests run nightly and before production deployment

### Performance Tests

- **RAG Response Time**: p95 latency < 3 seconds for RAG queries (measure with 100 concurrent users)
- **Page Load Time**: p95 time-to-interactive < 2 seconds for chapter pages (Lighthouse CI)
- **API Throughput**: Backend handles 100 requests/second without degradation (load test with Locust)
- **Database Queries**: All queries < 100ms p95 (log slow queries for optimization)

## Security Standards

### Input Validation

- **API Endpoints**: Validate all request bodies against Pydantic schemas; reject invalid requests with 422 status
- **SQL Injection Prevention**: Use SQLAlchemy ORM with parameterized queries; NEVER construct raw SQL from user input
- **XSS Prevention**: Sanitize user inputs before rendering in React; use DOMPurify for any HTML content
- **NoSQL Injection**: Escape special characters in Qdrant queries; use Qdrant's built-in query builders

### Authentication & Authorization

- **Password Storage**: Hash passwords with bcrypt (cost factor 12); salts generated automatically
- **Session Security**: HTTP-only cookies with Secure flag (HTTPS only) and SameSite=Strict to prevent CSRF
- **API Authentication**: Require valid session cookie for all protected endpoints; return 401 for unauthorized requests
- **Role-Based Access**: Future-proof for admin roles; prepare user roles table for admin features (content management)

### Secrets Management

- **Environment Variables**: All API keys, database URLs, secrets MUST be in `.env` (never committed)
- **Production Secrets**: Use GitHub Secrets for CI/CD; rotate secrets quarterly
- **Local Development**: Provide `.env.example` template with placeholder values; developers create own `.env`

### Rate Limiting

- **RAG Queries**: 20 queries per minute per authenticated user; prevent abuse of OpenAI API costs
- **Authentication**: 5 login attempts per 15 minutes per IP; prevents brute force attacks
- **API Endpoints**: 100 requests per minute per user for general API endpoints

## Development Workflow

### Git Workflow

- **Branching Strategy**: Feature branches from `main` following pattern `###-feature-name` (e.g., `001-rag-chatbot`)
- **Commit Messages**: Conventional Commits format (`feat:`, `fix:`, `docs:`, `test:`, `refactor:`)
- **Pull Requests**: All changes via PRs; require passing CI checks before merge
- **Code Review**: At least one approval required for PRs; reviewer checks constitution compliance

### Continuous Integration (GitHub Actions)

- **On Push**: Run linters (ESLint, Pylint, mypy, Prettier), unit tests, type checks
- **On PR**: Run integration tests, E2E tests, performance tests, security scans
- **On Merge to Main**: Auto-deploy Docusaurus to GitHub Pages; deploy FastAPI to cloud platform
- **Test Coverage**: Fail PR if coverage drops below 80% for critical modules

### Deployment

- **Frontend**: GitHub Pages with automatic deployment from `main` branch
- **Backend**: Deploy to cloud platform (Railway/Render/Vercel) with zero-downtime deployment
- **Database**: Neon Serverless Postgres (managed service, no deployment needed)
- **Vector Store**: Qdrant Cloud (managed service, no deployment needed)
- **Environment Configuration**: Separate configs for dev, staging, prod; use environment variables

### Health Checks

- **Backend**: `/health` endpoint returns 200 if database and vector store accessible
- **Frontend**: Display "System Status" indicator in footer (green = all services healthy)
- **Monitoring**: Uptime monitoring with alerts via email if service down > 5 minutes

## Governance

### Constitution Authority

This constitution supersedes all other development practices and conventions. When in doubt, refer to this document.

### Amendment Process

1. **Proposal**: Submit amendment via PR to `.specify/memory/constitution.md` with rationale
2. **Discussion**: Discuss tradeoffs and implications with team
3. **Approval**: Require consensus from project leads (Zia, Rehan, Junaid, Wania if applicable)
4. **Migration Plan**: Document how existing code will be brought into compliance
5. **Version Bump**: Follow semantic versioning (MAJOR for breaking changes, MINOR for new principles, PATCH for clarifications)
6. **Propagation**: Update dependent templates (plan, spec, tasks) to reflect amendments

### Compliance Verification

- **All PRs**: Reviewer MUST verify changes comply with constitution principles
- **Complexity Justification**: Any violations (e.g., >3s RAG latency) MUST be documented in `plan.md` Complexity Tracking section
- **Quarterly Audits**: Review codebase quarterly for compliance; create remediation tasks for violations

### Living Document

- This constitution evolves as the project grows
- Principles should be measurable and testable
- Prefer explicit rules over vague guidelines
- When ambiguous, favor user experience and educational value

**Version**: 1.0.0 | **Ratified**: 2025-11-30 | **Last Amended**: 2025-11-30
