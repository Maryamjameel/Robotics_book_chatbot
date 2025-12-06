# QUICKSTART: Feature 008 Development Guide

Get started with Feature 008 (Docusaurus Theme Integration & Production API Configuration) in 5 minutes.

## 1. Setup (1 minute)

```bash
cd frontend
npm install
npm install --save-dev @playwright/test
```

## 2. Configure Environment (1 minute)

Copy environment template:
```bash
cp .env.example .env
```

Edit `.env` with your API endpoint:

**Development:**
```env
NODE_ENV=development
REACT_APP_API_URL=http://localhost:8000/api
```

**Production:**
```env
NODE_ENV=production
REACT_APP_API_URL=https://api.your-domain.com/api
```

## 3. Run Development Server (1 minute)

```bash
# Terminal 1: Start Docusaurus
npm start

# Terminal 2 (if needed): Start backend API
cd ../backend
python -m uvicorn src.main:app --reload --port 8000
```

Visit http://localhost:3000 - ChatKit widget should appear in bottom-right corner

## 4. Test Theme Integration (1 minute)

In browser DevTools console:

```javascript
// Switch to dark mode
document.documentElement.setAttribute('data-theme', 'dark');

// Switch back to light mode
document.documentElement.setAttribute('data-theme', 'light');

// Check theme config
console.log(document.documentElement.getAttribute('data-theme'));
```

ChatKit colors should change smoothly over 300ms.

## 5. Run Tests (1 minute)

### Unit Tests
```bash
npm run test:unit
```

Tests validate:
- ✅ Theme detection via useThemeContext hook
- ✅ CSS variable resolution with fallbacks
- ✅ API configuration loading
- ✅ URL validation (HTTPS in production)

### E2E Tests
```bash
npm run test:e2e
```

E2E tests validate:
- ✅ Theme switching (light ↔ dark)
- ✅ API endpoint configuration
- ✅ Selected text integration
- ✅ Mobile responsive design

### Run Specific Test
```bash
# Theme switching tests
npx playwright test theme-switching.spec.ts

# API endpoint tests
npx playwright test api-endpoint.spec.ts

# Selected text tests
npx playwright test selected-text.spec.ts
```

## Key Files

### Theme Integration
- `src/components/ChatKit/hooks/useThemeContext.ts` - Theme detection
- `src/components/ChatKit/styles/chatkit.css` - CSS variables
- `src/components/ChatKit/ChatKitWidget.tsx` - Uses theme hook

### API Configuration
- `src/config/api.ts` - API configuration module
- `src/config/validate-api.ts` - URL validation
- `src/config/api-logger.ts` - Logging utility
- `src/components/ChatKit/services/apiService.ts` - API requests

### Tests
- `src/components/ChatKit/__tests__/theme.test.ts` - Theme tests
- `src/components/ChatKit/__tests__/api-config.test.ts` - API config tests
- `src/__tests__/e2e/theme-switching.spec.ts` - Theme E2E tests
- `src/__tests__/e2e/api-endpoint.spec.ts` - API E2E tests
- `src/__tests__/e2e/selected-text.spec.ts` - Selected text E2E tests

### Configuration
- `.env.example` - Environment variable template
- `playwright.config.ts` - E2E test configuration

## Common Tasks

### Change API Endpoint

Edit `.env`:
```env
REACT_APP_API_URL=https://your-new-api.com/api
```

Then restart dev server: `npm start`

### Check API Configuration

In browser console:
```javascript
// Import and check config (if exposed)
import { apiConfig } from './src/config/api';
console.log(apiConfig);
// Output: { baseURL, environment, isDevelopment, isProduction, ... }
```

### Debug Theme Detection

In browser console:
```javascript
// Check current theme
const theme = document.documentElement.getAttribute('data-theme');
console.log('Current theme:', theme);

// Check CSS variables
const style = getComputedStyle(document.documentElement);
console.log('Primary color:', style.getPropertyValue('--ifm-color-primary'));
console.log('Background:', style.getPropertyValue('--ifm-background-color'));
```

### Run Full Test Suite

```bash
# Unit tests + E2E tests + lint
npm run test:all
```

## Troubleshooting

### Theme Not Changing?

1. Check that Docusaurus is setting `html[data-theme]` attribute
2. Open DevTools: right-click → Inspect → check `<html>` tag
3. Manually set theme: `document.documentElement.setAttribute('data-theme', 'dark')`
4. Check CSS: Open ChatKit widget, right-click → Inspect, look for `--chatkit-bg` variable

### API Requests Failing?

1. Check `.env` file has correct `REACT_APP_API_URL`
2. Verify backend is running: `curl http://localhost:8000/api`
3. Open DevTools Network tab, look for API requests
4. Check browser console for `[APIConfig]` logs

### E2E Tests Timing Out?

1. Ensure Docusaurus dev server is running: `npm start`
2. Ensure backend API is running: `http://localhost:8000`
3. Run single test with verbose output: `npx playwright test --debug`
4. Check Playwright report: `npx playwright show-report`

## Next Steps

- Read `spec.md` for complete feature specification
- Read `plan.md` for architectural decisions
- Read `tasks.md` for implementation task breakdown
- Review `../../README.md` for full documentation

## Quick Reference: npm Scripts

```bash
npm start              # Start Docusaurus dev server
npm run build          # Build for production
npm run test:unit      # Run unit tests
npm run test:e2e       # Run E2E tests
npm run test:all       # Run all tests
npm run typecheck      # TypeScript type checking
npm run lint           # Linting
```

## Support

Questions? Check:
1. `.env.example` - Environment variable documentation
2. Browser console - Look for `[APIConfig]` and `[useThemeContext]` logs
3. Playwright report - Visual debugging of E2E tests
4. Architecture diagrams in `plan.md`

Happy coding! 🚀
