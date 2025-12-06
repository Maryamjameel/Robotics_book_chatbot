# Feature Specification: Docusaurus Theme Integration & Production Configuration

**Feature Branch**: `008-docusaurus-theme-config`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Match Docusaurus theme and test: ChatKit styling configuration with Docusaurus variables support and dark mode toggle support, production API URL configuration via environment variables, and comprehensive end-to-end tests"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - ChatKit Adapts to Docusaurus Light/Dark Theme (Priority: P1)

Users reading the Robotics Book textbook expect the ChatKit widget to seamlessly match the current Docusaurus theme (light or dark mode). When the user toggles Docusaurus theme settings, ChatKit should immediately reflect those changes without requiring page reload.

**Why this priority**: Visual consistency is fundamental to user experience. Users notice theme mismatches immediately and perceive the widget as broken or low-quality if it doesn't follow the documentation site's design language. This is the primary visual integration point.

**Independent Test**: Can be fully tested by opening ChatKit in a Docusaurus page, toggling the site's dark mode, and verifying ChatKit updates colors accordingly. Delivers consistent visual experience that makes ChatKit feel native to the site.

**Acceptance Scenarios**:

1. **Given** user is on a Docusaurus page in light mode, **When** ChatKit widget opens, **Then** widget uses light theme colors matching Docusaurus primary colors
2. **Given** ChatKit is open in light mode, **When** user toggles Docusaurus theme to dark mode, **Then** ChatKit immediately transitions to dark colors without page reload
3. **Given** user is on a Docusaurus page in dark mode, **When** ChatKit widget opens, **Then** widget uses dark theme colors matching Docusaurus background and text colors
4. **Given** user toggles theme multiple times, **When** each toggle occurs, **Then** ChatKit transitions smoothly with 300ms animation without flickering or console errors

---

### User Story 2 - Production API Configuration (Priority: P1)

The ChatKit widget must connect to the correct API endpoint depending on the deployment environment. In development, it uses localhost; in staging/production, it uses the production server URL. Configuration should be environment-based with no hardcoded URLs.

**Why this priority**: Incorrectly configured API endpoints prevent the application from working entirely in production. This is a critical deployment blocker that directly affects whether users can interact with ChatKit on production servers.

**Independent Test**: Can be fully tested by building the application with different NODE_ENV values and verifying the API endpoint changes accordingly. Delivers correct backend connectivity in all environments.

**Acceptance Scenarios**:

1. **Given** application built with NODE_ENV=development, **When** ChatKit initializes, **Then** API calls target http://localhost:8000/api
2. **Given** application built with NODE_ENV=production, **When** ChatKit initializes, **Then** API calls target the production API URL from environment configuration
3. **Given** REACT_APP_API_URL environment variable is set to custom value, **When** application builds, **Then** that URL is used for all API calls
4. **Given** ChatKit sends a question to configured API endpoint, **When** request completes, **Then** response includes chapter context data showing end-to-end integration

---

### User Story 3 - End-to-End Test Coverage (Priority: P2)

Development and QA teams need automated tests validating that ChatKit works correctly across different scenarios: page detection, text selection, chapter context passing, result display, and responsiveness on mobile devices.

**Why this priority**: Automated tests prevent regressions and catch integration issues early. They enable confident deployments and reduce manual QA burden. Critical for maintaining quality as features evolve.

**Independent Test**: Can be fully tested by running the E2E test suite and verifying all scenarios pass. Delivers automated validation of key user workflows.

**Acceptance Scenarios**:

1. **Given** E2E test suite runs against Docusaurus page, **When** test loads page, **Then** ChatKit widget appears on page without console errors
2. **Given** user selects text on documentation page, **When** ChatKit opens, **Then** selected text is visible in the query input and chapter context is populated in backend API request
3. **Given** chapter context is passed to backend, **When** search results return, **Then** response includes chapter filtering metadata showing which chapter was used
4. **Given** browser window is resized to mobile dimensions (320px width), **When** ChatKit widget is used, **Then** interface remains fully functional and readable
5. **Given** user sends multiple messages in ChatKit, **When** conversation continues, **Then** all previous messages persist and chapter context is maintained across each message

---

### Edge Cases

- What happens when Docusaurus CSS variables are not defined (older version or custom setup)?
- How does ChatKit handle API timeouts or network failures in production?
- What happens when NODE_ENV is set to a non-standard value (staging, custom)?
- How does mobile experience handle very long chapter titles or customized Docusaurus layouts?
- What happens if REACT_APP_API_URL environment variable is malformed or invalid?
- How does ChatKit behave if Docusaurus theme changes after initial render?
- What happens if both NODE_ENV and REACT_APP_API_URL are set (which takes precedence)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: ChatKit styling MUST use Docusaurus CSS custom properties (`--ifm-color-primary`, `--ifm-background-color`, `--ifm-font-color-base`, etc.) instead of hardcoded colors
- **FR-002**: ChatKit MUST support Docusaurus dark mode by listening to theme changes and updating colors dynamically
- **FR-003**: ChatKit MUST apply a smooth color transition when theme changes (CSS transition duration 300ms or less)
- **FR-004**: System MUST create `frontend/config/api.ts` configuration module that exports API base URL
- **FR-005**: API endpoint selection MUST be based on NODE_ENV environment variable with clear fallbacks and priority
- **FR-006**: REACT_APP_API_URL environment variable MUST override the default endpoint if provided during build
- **FR-007**: All API calls from ChatKit MUST use the configured endpoint from api.ts configuration
- **FR-008**: E2E tests MUST validate ChatKit appears on documentation pages without console errors
- **FR-009**: E2E tests MUST validate selected text feature works correctly and passes text to ChatKit queries
- **FR-010**: E2E tests MUST validate chapter context is detected from page and passed in API requests
- **FR-011**: E2E tests MUST validate response sources are displayed correctly with chapter information metadata
- **FR-012**: E2E tests MUST validate ChatKit is responsive on mobile viewports (minimum 320px width)
- **FR-013**: E2E tests MUST validate message persistence across multiple user queries in same session
- **FR-014**: API configuration MUST be consumed by `useRAGAPI` hook without requiring component-level changes to existing code
- **FR-015**: Development builds MUST default to localhost API (http://localhost:8000/api), production builds MUST use production URL
- **FR-016**: Configuration MUST be loaded at application startup and available to all components without requiring dynamic reloading

### Key Entities *(include if feature involves data)*

- **APIConfiguration**: Holds API endpoint configuration (baseURL, environment, isDevelopment, isProduction)
- **ThemeContext**: Represents current theme state (light/dark) and resolved CSS variable values
- **TestEnvironment**: Test fixtures for E2E testing with mock Docusaurus pages, API responses, and theme toggling

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: ChatKit colors match Docusaurus theme within 100% accuracy - visual comparison shows no color mismatches in light and dark modes
- **SC-002**: Theme transitions happen within 300ms when user toggles dark/light mode - perceived by users as instantaneous
- **SC-003**: ChatKit in production connects to correct API endpoint 100% of the time - verified through production logs
- **SC-004**: E2E test suite covers 8+ distinct scenarios and all tests pass with 95%+ consistency across runs
- **SC-005**: Mobile viewport tests pass on widths from 320px to 768px covering 90% of real mobile device sizes
- **SC-006**: API configuration can be validated by building with different NODE_ENV values and checking actual network requests
- **SC-007**: Theme configuration adds zero additional bytes to production bundle size by using native CSS variables only
- **SC-008**: End-to-end test suite executes in under 60 seconds for full test coverage

---

## Assumptions

- Docusaurus version 2.x or higher is being used (supports CSS custom properties/variables)
- The application is using Create React App or similar build system with environment variable support
- Dark mode toggle is controlled by Docusaurus's native theme switcher (`html[data-theme="dark"]` selector)
- Production API URL will be provided as environment variable during build time or in `.env.production`
- ChatKit is embedded in Docusaurus via a custom theme component or document wrapper
- E2E tests will use industry-standard framework (Playwright, Cypress, or WebDriver)
- Docusaurus CSS variable names follow the `--ifm-*` naming convention
- The frontend build process can access and inject environment variables at build time

---

## Constraints

- No changes to Docusaurus core configuration files
- No additional npm dependencies if possible (use native CSS variables where available)
- API configuration must work with existing Create React App build pipeline
- E2E tests must not require authentication or external API calls (should use mocks or fixtures)
- Theme switching must not cause page reloads or loss of ChatKit state
- API URL configuration must follow 12-factor app principles (environment-based, not configuration files)
- CSS transitions must be performant (60fps without dropping frames)

---

## Out of Scope

- Creating a new Docusaurus theme from scratch
- Modifying Docusaurus CSS variables or documentation configuration
- Custom theme selector UI (use Docusaurus's built-in toggle)
- API rate limiting, caching, or retry logic
- Analytics or telemetry for theme usage patterns
- Accessibility audit beyond existing WCAG AA compliance
- Server-side rendering (SSR) configuration

---

## In Scope

- ChatKit CSS refactor to use Docusaurus CSS custom properties
- API configuration module (`frontend/config/api.ts`) creation and integration
- Theme change listener implementation using CSS media queries or MutationObserver
- Environment variable configuration for NODE_ENV and REACT_APP_API_URL
- Comprehensive E2E test suite covering all user stories
- Documentation for configuration and environment setup

---

## Dependencies

### Internal Dependencies
- Feature 007-chapter-context (ChatKit must pass chapter context through configured API)
- Feature 006-selected-text (Selected text feature must work with configured API endpoint)
- Feature 005-chatkit-integration (Baseline ChatKit widget integration)

### External Dependencies
- Docusaurus 2.x or higher (CSS variable support)
- Node.js/npm build system (environment variable support)
- E2E testing framework (Playwright, Cypress, or WebDriver)
- React 16.8+ (hooks support for theme detection)

---

## Non-Functional Requirements

### Performance
- CSS variable lookup: < 1ms (native browser functionality)
- Theme transition: 300ms smooth CSS transition (60fps animation)
- API endpoint resolution: < 1ms (synchronous operation at startup)
- API configuration initialization: < 10ms
- E2E test execution: < 60 seconds total suite

### Reliability
- API configuration must be available before any component renders
- API endpoint must not change after application startup
- Theme transitions must not drop frames (consistent 60fps)
- Configuration validation must run at build time
- Fallback colors must be provided if CSS variables unavailable

### Compatibility
- Light mode and dark mode fully supported
- Works on all modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- Mobile responsive from 320px viewport width to 2560px (all device sizes)
- Works without modification to existing Docusaurus installations
- Graceful degradation if CSS variables not supported (uses fallback colors)

### Security
- No secrets in frontend code or version control
- API URL from environment variables only (not from user input)
- HTTPS enforced in production configuration (validated at build time)
- No hardcoded development URLs in production builds
- Configuration validated before use to prevent injection attacks

---

## Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Docusaurus CSS variables not defined or named differently | Medium | High | Provide sensible CSS fallback colors and document compatibility requirements |
| Production API URL misconfiguration | Medium | Critical | Validate URL at startup, log configuration on app init, provide clear error messages |
| E2E test flakiness due to timing issues | Medium | Medium | Use explicit waits, Cypress/Playwright best practices, retry logic, mock API responses |
| Theme transition performance issues | Low | Medium | Profile transitions, use CSS GPU acceleration (transform), optimize paint areas |
| API config loaded after component render | Low | Critical | Ensure config.ts is imported at app root level, validate synchronously |
| REACT_APP_API_URL variable not injected at build time | Medium | High | Document build process, validate env vars during CI/CD, fail build if missing in production |

---

## Glossary

- **CSS Variables (Custom Properties)**: CSS feature allowing reusable color values (`--variable-name`) that can be changed dynamically
- **NODE_ENV**: Environment variable indicating development vs production vs staging mode
- **E2E (End-to-End)**: Testing approach validating entire user workflows from UI through backend
- **Docusaurus**: Static site generator used for documentation, supports CSS custom properties
- **REACT_APP_**: Prefix for Create React App environment variables exposed to frontend
- **CSS Fallback**: Default CSS value used if CSS variable is not defined
- **MutationObserver**: Browser API for detecting changes to DOM elements

---

## Next Phase

Run `/sp.clarify` to resolve any ambiguities, then `/sp.plan` to create architecture and detailed task breakdown.
