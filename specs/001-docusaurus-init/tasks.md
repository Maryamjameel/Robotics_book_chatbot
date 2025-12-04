# Tasks: Docusaurus Project Initialization

**Input**: Design documents from `/specs/001-docusaurus-init/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, quickstart.md ✅

**Tests**: Manual verification only (no automated tests requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `frontend/` at repository root
- All Docusaurus files within `frontend/` directory
- Content in `frontend/docs/chapters/` and `frontend/docs/glossary/`
- Configuration files: `frontend/docusaurus.config.ts`, `frontend/sidebars.ts`

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize Docusaurus project and basic structure

- [X] T001 Verify prerequisites: Node.js 18.0+, npm 9.0+, Git configured
- [X] T002 Run `npx create-docusaurus@latest frontend classic --typescript` from repository root
- [X] T003 Verify default Docusaurus project structure created in frontend/
- [X] T004 [P] Enable TypeScript strict mode in frontend/tsconfig.json
- [X] T005 [P] Update frontend/package.json metadata (title, description, author)
- [X] T006 Test default installation with `npm start` in frontend/ directory
- [X] T007 Verify default homepage loads at http://localhost:3000 without errors

**Checkpoint**: Basic Docusaurus project initialized and running locally

---

## Phase 2: Foundational (No Blocking Prerequisites)

**Purpose**: This initialization task has no blocking foundational infrastructure

**⚠️ NOTE**: This phase is typically for shared infrastructure (databases, auth, APIs). For Docusaurus initialization, there are no blocking prerequisites. All user stories can proceed immediately after Setup (Phase 1).

**Skip to User Stories** - All stories depend only on Phase 1 completion

---

## Phase 3: User Story 1 - Project Foundation Setup (Priority: P1) 🎯 MVP

**Goal**: Create a TypeScript-based Docusaurus project with classic template that serves as the foundation for the robotics textbook platform

**Independent Test**:
1. Run `npm start` in frontend/ directory
2. Verify dev server starts in < 30 seconds
3. Navigate to http://localhost:3000
4. Verify homepage renders without console errors
5. Verify TypeScript configuration includes strict mode

**Acceptance Scenarios** (from spec.md):
- [x] TypeScript-based Docusaurus project created in `frontend/` directory with classic template
- [x] Development server starts with `npm start` and homepage loads at localhost:3000
- [x] TypeScript configuration files (tsconfig.json) present with strict mode enabled

### Implementation for User Story 1

- [X] T008 [US1] Verify frontend/tsconfig.json contains `"strict": true` (updated in T004)
- [X] T009 [US1] Verify frontend/docusaurus.config.ts exists with default configuration
- [X] T010 [US1] Verify frontend/package.json contains Docusaurus 3.x dependencies
- [X] T011 [US1] Test build command: run `npm run build` in frontend/ directory
- [X] T012 [US1] Verify build/ directory created with static files
- [X] T013 [US1] Test production build: run `npm run serve` and verify site loads
- [X] T014 [US1] Document any build warnings or issues in frontend/README.md

**Checkpoint**: User Story 1 complete - Docusaurus project initialized, TypeScript strict mode enabled, dev server and build process verified

---

## Phase 4: User Story 2 - GitHub Pages Deployment Configuration (Priority: P2)

**Goal**: Configure Docusaurus project for GitHub Pages deployment so the textbook can be published and accessed online

**Independent Test**:
1. Verify frontend/docusaurus.config.ts contains organizationName, projectName, baseUrl
2. Run `npm run build` in frontend/ directory
3. Check build/ directory contains deployment-ready static files
4. Test with `npm run serve` that assets load with baseUrl prefix

**Acceptance Scenarios** (from spec.md):
- [x] docusaurus.config.ts configured with GitHub Pages settings
- [x] Build command generates deployment-ready static files
- [x] Built files load correctly with configured baseUrl

### Implementation for User Story 2

- [ ] T015 [US2] Update frontend/docusaurus.config.ts title to "Physical AI & Humanoid Robotics Textbook"
- [ ] T016 [US2] Update frontend/docusaurus.config.ts tagline to "Comprehensive robotics education for Panaversity students"
- [ ] T017 [US2] Set url in frontend/docusaurus.config.ts to `https://[username].github.io` (replace with actual GitHub username)
- [ ] T018 [US2] Set baseUrl in frontend/docusaurus.config.ts to `/Robotics_book_chatbot/`
- [ ] T019 [US2] Set organizationName in frontend/docusaurus.config.ts to GitHub username
- [ ] T020 [US2] Set projectName in frontend/docusaurus.config.ts to `Robotics_book_chatbot`
- [ ] T021 [US2] Set deploymentBranch in frontend/docusaurus.config.ts to `gh-pages`
- [ ] T022 [US2] Set trailingSlash in frontend/docusaurus.config.ts to `false`
- [ ] T023 [US2] Set onBrokenLinks in frontend/docusaurus.config.ts to `throw`
- [ ] T024 [US2] Disable blog feature: set blog to `false` in preset configuration
- [ ] T025 [US2] Update editUrl in frontend/docusaurus.config.ts to point to GitHub repository
- [ ] T026 [US2] Test build with new configuration: `npm run build` in frontend/
- [ ] T027 [US2] Test production serve with baseUrl: `npm run serve` and verify assets load at localhost:3000/Robotics_book_chatbot/
- [ ] T028 [US2] Perform manual deployment test: `GIT_USER=[username] npm run deploy` from frontend/
- [ ] T029 [US2] Configure GitHub repository Settings > Pages to deploy from gh-pages branch
- [ ] T030 [US2] Verify deployed site loads at https://[username].github.io/Robotics_book_chatbot/
- [ ] T031 [US2] Create .github/workflows/deploy.yml for automated GitHub Actions deployment (per quickstart.md)
- [ ] T032 [US2] Test GitHub Actions workflow by pushing changes and verifying automatic deployment

**Checkpoint**: User Story 2 complete - GitHub Pages deployment configured, manual deployment tested, automated CI/CD workflow created

---

## Phase 5: User Story 3 - Content Structure Organization (Priority: P2)

**Goal**: Create well-organized folder structure (docs/chapters/, docs/glossary/) for systematically adding robotics textbook content

**Independent Test**:
1. Verify frontend/docs/chapters/ directory exists with README.md
2. Verify frontend/docs/glossary/ directory exists with README.md
3. Create test markdown file in chapters/ directory
4. Run `npm start` and verify test file is accessible through site
5. Verify clear separation between chapter and glossary content

**Acceptance Scenarios** (from spec.md):
- [x] docs/chapters/ and docs/glossary/ directories exist
- [x] Markdown files added to directories are accessible through Docusaurus site
- [x] Organization supports clear separation between chapters and glossary

### Implementation for User Story 3

- [ ] T033 [P] [US3] Create frontend/docs/chapters/ directory
- [ ] T034 [P] [US3] Create frontend/docs/glossary/ directory
- [ ] T035 [US3] Create frontend/docs/chapters/README.md with structure guidelines (content from quickstart.md)
- [ ] T036 [US3] Create frontend/docs/glossary/README.md with structure guidelines (content from quickstart.md)
- [ ] T037 [US3] Add frontmatter to frontend/docs/chapters/README.md: sidebar_position, sidebar_label
- [ ] T038 [US3] Add frontmatter to frontend/docs/glossary/README.md: sidebar_position, sidebar_label
- [ ] T039 [US3] Test content structure: create sample chapter file frontend/docs/chapters/sample-chapter.md
- [ ] T040 [US3] Test content structure: create sample glossary term frontend/docs/glossary/sample-term.md
- [ ] T041 [US3] Run `npm start` and verify sample files render correctly
- [ ] T042 [US3] Remove sample files (frontend/docs/chapters/sample-chapter.md, frontend/docs/glossary/sample-term.md)
- [ ] T043 [US3] Document content authoring guidelines in frontend/README.md

**Checkpoint**: User Story 3 complete - Content directories created with README guidelines, structure tested and verified

---

## Phase 6: User Story 4 - Navigation Configuration (Priority: P3)

**Goal**: Configure sidebar navigation to enable easy browsing through textbook chapters and glossary sections

**Independent Test**:
1. Run `npm start` in frontend/ directory
2. Verify "Chapters" link appears in navbar
3. Verify "Glossary" link appears in navbar
4. Click each nav link and verify navigation works
5. Verify sidebar displays sections for chapters and glossary

**Acceptance Scenarios** (from spec.md):
- [x] sidebars.ts configured with chapters and glossary sections
- [x] Navigation menu displays with clear sections
- [x] Clicking navigation items navigates correctly

### Implementation for User Story 4

- [ ] T044 [US4] Update frontend/sidebars.ts: create chapters sidebar with category structure
- [ ] T045 [US4] Update frontend/sidebars.ts: add chapters/README to chapters sidebar
- [ ] T046 [US4] Update frontend/sidebars.ts: create glossary sidebar with category structure
- [ ] T047 [US4] Update frontend/sidebars.ts: add glossary/README to glossary sidebar
- [ ] T048 [US4] Configure glossary sidebar with autogenerated dirName for future terms
- [ ] T049 [US4] Update frontend/docusaurus.config.ts navbar: add Chapters link with type docSidebar
- [ ] T050 [US4] Update frontend/docusaurus.config.ts navbar: add Glossary link with type docSidebar
- [ ] T051 [US4] Update frontend/docusaurus.config.ts navbar: update GitHub link to point to repository
- [ ] T052 [US4] Update frontend/docusaurus.config.ts footer: set copyright to Panaversity
- [ ] T053 [US4] Test navigation: run `npm start` and verify all navbar links work
- [ ] T054 [US4] Test sidebar: verify Chapters section expands and shows README
- [ ] T055 [US4] Test sidebar: verify Glossary section expands and shows README
- [ ] T056 [US4] Verify no console errors when navigating between sections

**Checkpoint**: User Story 4 complete - Sidebar navigation configured, navbar links added, all navigation tested and verified

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, documentation, and optional enhancements

- [ ] T057 [P] (Optional) Install Tailwind CSS v3: `npm install -D tailwindcss postcss autoprefixer` in frontend/
- [ ] T058 [P] (Optional) Run `npx tailwindcss init` to create tailwind.config.js in frontend/
- [ ] T059 [P] (Optional) Create frontend/postcss.config.js with Tailwind and autoprefixer plugins
- [ ] T060 [P] (Optional) Configure frontend/tailwind.config.js with content paths and dark mode
- [ ] T061 [P] (Optional) Update frontend/src/css/custom.css to import Tailwind directives
- [ ] T062 [P] (Optional) Test Tailwind: create test component and verify utilities work
- [ ] T063 [P] Update frontend/README.md with complete setup instructions
- [ ] T064 [P] Document project structure and conventions in frontend/README.md
- [ ] T065 [P] Add contribution guidelines for content authors to frontend/README.md
- [ ] T066 Run complete verification: follow quickstart.md success verification checklist
- [ ] T067 Create project documentation: update repository root README.md with project overview
- [ ] T068 (Optional) Customize frontend/static/img/logo.svg with Panaversity branding
- [ ] T069 (Optional) Update frontend/static/img/favicon.ico with custom icon
- [ ] T070 Final build and deploy: run `npm run build` and `npm run deploy`
- [ ] T071 Verify production site at https://[username].github.io/Robotics_book_chatbot/
- [ ] T072 Create implementation PHR documenting setup process and lessons learned

**Checkpoint**: All user stories complete, documentation updated, production site deployed

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: N/A - no blocking prerequisites for this initialization task
- **User Stories (Phase 3-6)**: All depend ONLY on Setup (Phase 1) completion
  - User stories can proceed in parallel OR sequentially in priority order
  - **Recommended**: Execute sequentially for this initialization (P1 → P2 → P2 → P3)
- **Polish (Phase 7)**: Depends on all user stories (Phase 3-6) being complete

### User Story Dependencies

- **User Story 1 (P1)**: Depends only on Setup (Phase 1) - Initializes project foundation
- **User Story 2 (P2)**: Depends on US1 - Configures deployment for initialized project
- **User Story 3 (P2)**: Depends on US1 - Creates content directories in initialized project
- **User Story 4 (P3)**: Depends on US1 and US3 - Configures navigation for content structure

**Recommended Execution Order**: Phase 1 → US1 → US2 → US3 → US4 → Phase 7

### Within Each User Story

- User Story 1: All tasks sequential (verification tasks)
- User Story 2: T015-T025 (configuration) can be done in single edit, then T026-T032 (testing/deployment) sequential
- User Story 3: T033-T034 parallel (mkdir), T035-T038 can be done together, T039-T042 sequential (testing)
- User Story 4: T044-T048 (sidebars.ts) can be single edit, T049-T052 (config updates) can be single edit, T053-T056 sequential (testing)

### Parallel Opportunities

**Limited parallelization** for this initialization task due to:
- Most tasks involve editing same configuration files (docusaurus.config.ts, sidebars.ts)
- Many tasks are verification steps that must run sequentially
- Single-developer task with clear sequential flow

**Parallel tasks within phases**:
- Phase 1: T004 and T005 can run parallel (different files)
- Phase 5 (US3): T033 and T034 can run parallel (mkdir operations)
- Phase 7: T057-T062 (Tailwind) all parallel if included, T063-T065 (docs) all parallel

---

## Parallel Example: User Story 3

```bash
# These directory creation tasks can run in parallel:
Task T033: "Create frontend/docs/chapters/ directory"
Task T034: "Create frontend/docs/glossary/ directory"

# These README creation tasks can run in parallel:
Task T035: "Create frontend/docs/chapters/README.md"
Task T036: "Create frontend/docs/glossary/README.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup ✅
2. Complete Phase 3: User Story 1 (Project Foundation) ✅
3. **STOP and VALIDATE**: Test that Docusaurus runs locally with TypeScript strict mode
4. **MVP Delivered**: Basic Docusaurus project ready for content authoring

### Incremental Delivery

1. **Foundation (Phase 1 + US1)** → Docusaurus project initialized
2. **Add US2 (Deployment)** → Site can be published to GitHub Pages
3. **Add US3 (Content Structure)** → Content directories ready for authoring
4. **Add US4 (Navigation)** → Full navigation experience complete
5. **Polish (Phase 7)** → Optional enhancements and final documentation

Each increment delivers a testable, usable milestone.

### Sequential Execution (Recommended for Single Developer)

This initialization task is best executed sequentially:

1. Phase 1: Setup (T001-T007) - ~10 minutes
2. Phase 3: User Story 1 (T008-T014) - ~5 minutes
3. Phase 4: User Story 2 (T015-T032) - ~20 minutes
4. Phase 5: User Story 3 (T033-T043) - ~10 minutes
5. Phase 6: User Story 4 (T044-T056) - ~15 minutes
6. Phase 7: Polish (T057-T072) - ~30 minutes (includes optional Tailwind)

**Total estimated time**: ~90 minutes for complete initialization

---

## Task Summary

**Total Tasks**: 72 tasks
- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 0 tasks (N/A for this feature)
- **Phase 3 (US1 - Project Foundation)**: 7 tasks
- **Phase 4 (US2 - GitHub Pages Config)**: 18 tasks
- **Phase 5 (US3 - Content Structure)**: 11 tasks
- **Phase 6 (US4 - Navigation Config)**: 13 tasks
- **Phase 7 (Polish)**: 16 tasks (6 optional Tailwind tasks)

**Parallelizable Tasks**: ~10 tasks marked [P] (directory creation, documentation)

**Independent Test Criteria**:
- US1: Dev server runs, TypeScript strict mode enabled, build succeeds
- US2: Deployment configured, manual and automated deployment tested, site accessible
- US3: Content directories exist with guidelines, sample content renders correctly
- US4: Navigation links work, sidebar sections display, no navigation errors

**Suggested MVP Scope**: Phase 1 + Phase 3 (US1) = 14 tasks, ~15 minutes

---

## Notes

- [P] tasks = different files/directories, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Manual verification throughout (no automated tests in this phase)
- Commit after completing each user story phase
- Stop at any checkpoint to validate story independently
- Most configuration tasks involve editing docusaurus.config.ts or sidebars.ts - do these carefully in one session to avoid conflicts
- Follow quickstart.md for detailed command examples and troubleshooting
- Optional Tailwind CSS integration (T057-T062) can be deferred to future enhancement task
