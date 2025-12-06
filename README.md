# Robotics Textbook Chatbot

Physical AI & Humanoid Robotics Textbook - Interactive documentation platform with integrated ChatKit RAG widget.

## Features

### Feature 008: Docusaurus Theme Integration & Production Configuration

- **ChatKit Theme Adaptation**: Widget styling matches Docusaurus theme with smooth 300ms transitions
- **Production API Configuration**: Environment-aware API endpoint selection
- **CSS Variable Integration**: Uses native Docusaurus CSS variables with fallback colors
- **Comprehensive E2E Tests**: Playwright test suite for theme, API, and user interactions

## Configuration

### API Configuration

Set the backend API endpoint using environment variables:

```bash
# Development (default)
REACT_APP_API_URL=http://localhost:8000/api npm start

# Production
NODE_ENV=production REACT_APP_API_URL=https://api.your-domain.com/api npm run build
```

**Environment Hierarchy:**
1. REACT_APP_API_URL (highest priority)
2. NODE_ENV defaults (development/production/staging)

### Theme Configuration

ChatKit automatically syncs with Docusaurus theme. No manual setup required.

## Development

```bash
cd frontend
npm install
npm start
```

## Testing

```bash
# Unit tests
npm run test:unit

# E2E tests
npm run test:e2e
```

## Documentation

- `specs/008-docusaurus-theme-config/spec.md` - Feature specification
- `specs/008-docusaurus-theme-config/plan.md` - Implementation plan
- `specs/008-docusaurus-theme-config/tasks.md` - Task breakdown
- `frontend/.env.example` - Environment variable template

## Architecture

**Theme Integration:** Docusaurus → useThemeContext Hook → CSS Variables → ChatKit Widget

**API Configuration:** process.env → apiConfig Module → apiService → Backend API

## File Structure

Key files created in Feature 008:

- `frontend/src/config/api.ts` - API configuration module
- `frontend/src/components/ChatKit/hooks/useThemeContext.ts` - Theme detection hook
- `frontend/src/components/ChatKit/styles/chatkit.css` - CSS variables
- `frontend/.env.example` - Environment template
- `playwright.config.ts` - E2E test configuration

## Constitution Compliance

✅ Production-Grade Quality
✅ Privacy-First  
✅ RAG Accuracy
✅ Modular Architecture
✅ Observability
✅ Spec-Driven Development

## Performance

- Theme transitions: 300ms
- API config resolution: <1ms
- E2E suite execution: <60s
