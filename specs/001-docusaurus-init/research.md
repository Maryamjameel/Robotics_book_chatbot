# Research: Docusaurus Project Initialization

**Feature**: 001-docusaurus-init
**Date**: 2025-12-03
**Status**: Complete

## Overview

This document captures research findings for initializing a Docusaurus 3.x documentation platform with TypeScript, Tailwind CSS v4, and GitHub Pages deployment for the Physical AI & Humanoid Robotics textbook.

## Research Questions

### 1. Docusaurus 3.x Best Practices

**Question**: What is the optimal TypeScript configuration and project structure for Docusaurus 3.x educational content?

**Findings**:

**TypeScript Configuration**:
- Docusaurus 3.x fully supports TypeScript with `--typescript` flag during initialization
- Default tsconfig.json includes:
  - `"strict": false` by default - should enable for constitution compliance
  - `"jsx": "react"` for JSX support
  - `"moduleResolution": "node"` for npm package resolution
  - Paths configured for `@docusaurus/*` and `@site/*` aliases
- Recommendation: Enable `"strict": true` post-initialization for type safety

**Project Structure for Educational Content**:
- Use `docs/` directory for all markdown content (default Docusaurus pattern)
- Organize by topic/chapter in subdirectories: `docs/chapters/`, `docs/glossary/`
- Each chapter as separate markdown file: `docs/chapters/01-introduction.md`
- Use frontmatter for metadata: `title`, `sidebar_label`, `sidebar_position`
- Code examples in fenced code blocks with syntax highlighting

**Best Practices**:
- Keep docs/ focused on content; custom React components in src/components/
- Use `_category_.json` files for folder-level sidebar configuration
- Enable `onBrokenLinks: 'throw'` to catch broken internal links during build
- Disable blog feature if not needed: `blog: false` in preset config

**Source**: [Docusaurus 3.x Documentation](https://docusaurus.io/docs)

### 2. Tailwind CSS v4 Integration with Docusaurus 3

**Question**: How to integrate Tailwind CSS v4 with Docusaurus 3 for consistent styling per constitution?

**Findings**:

**Integration Method** (Tailwind CSS v4 beta):
1. Install dependencies:
   ```bash
   npm install -D tailwindcss@next @tailwindcss/postcss autoprefixer
   ```

2. Create `postcss.config.js`:
   ```javascript
   module.exports = {
     plugins: {
       '@tailwindcss/postcss': {},
       autoprefixer: {},
     },
   };
   ```

3. Create `tailwind.config.js`:
   ```javascript
   /** @type {import('tailwindcss').Config} */
   module.exports = {
     content: ['./src/**/*.{js,jsx,ts,tsx}', './docs/**/*.{md,mdx}'],
     darkMode: ['class', '[data-theme="dark"]'], // Docusaurus dark mode
     theme: {
       extend: {},
     },
     plugins: [],
   };
   ```

4. Update `src/css/custom.css`:
   ```css
   @tailwind base;
   @tailwind components;
   @tailwind utilities;

   /* Custom Docusaurus overrides */
   :root {
     /* Keep Docusaurus CSS variables for theme consistency */
   }
   ```

**Considerations**:
- Tailwind v4 is in beta; may have breaking changes before stable release
- Alternative: Use Tailwind v3.x (stable) with same integration pattern
- Docusaurus uses CSS modules; Tailwind utilities complement them
- Test dark mode toggle compatibility with Docusaurus theme switcher

**Recommendation**: Start with Tailwind v3.x for stability; migrate to v4 when stable

**Source**: [Tailwind CSS v4 Beta Docs](https://tailwindcss.com/docs/v4-beta), [Docusaurus Styling Guide](https://docusaurus.io/docs/styling-layout)

### 3. GitHub Pages Deployment Configuration

**Question**: What is the recommended GitHub Pages deployment workflow for Docusaurus?

**Findings**:

**Deployment Options**:

1. **GitHub Actions (Recommended)**:
   - Automated deployment on push to main branch
   - No need to commit build artifacts to repository
   - Supports custom build steps and environment variables

   Create `.github/workflows/deploy.yml`:
   ```yaml
   name: Deploy to GitHub Pages

   on:
     push:
       branches: [main]
     workflow_dispatch:

   jobs:
     deploy:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v4
         - uses: actions/setup-node@v4
           with:
             node-version: 18
         - name: Install dependencies
           run: npm ci
           working-directory: ./frontend
         - name: Build website
           run: npm run build
           working-directory: ./frontend
         - name: Deploy to GitHub Pages
           uses: peaceiris/actions-gh-pages@v3
           with:
             github_token: ${{ secrets.GITHUB_TOKEN }}
             publish_dir: ./frontend/build
   ```

2. **Manual Deployment** (`npm run deploy`):
   - Uses `gh-pages` npm package
   - Pushes build/ directory to gh-pages branch
   - Requires GIT_USER environment variable
   - Good for testing before CI/CD setup

**Required Configuration** in `docusaurus.config.ts`:
```typescript
const config: Config = {
  url: 'https://[username].github.io',
  baseUrl: '/Robotics_book_chatbot/',  // Repository name
  organizationName: '[username]',       // GitHub username or org
  projectName: 'Robotics_book_chatbot', // Repository name
  deploymentBranch: 'gh-pages',         // Target branch (default)
  trailingSlash: false,                 // GitHub Pages compatibility
};
```

**GitHub Repository Settings**:
- Go to Settings > Pages
- Source: Deploy from a branch
- Branch: gh-pages / (root)
- Save and wait 1-2 minutes for first deployment

**Recommendation**: Use GitHub Actions for automated deployment; configure after manual verification

**Source**: [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment#deploying-to-github-pages)

### 4. Content Organization Patterns for Technical Textbooks

**Question**: What are effective patterns for organizing 20-30 chapters and 200+ glossary terms?

**Findings**:

**Chapter Organization Pattern**:

**Option A: Flat Chapter Structure** (Recommended for initial setup):
```
docs/
├── intro.md
├── chapters/
│   ├── _category_.json
│   ├── 01-introduction.md
│   ├── 02-kinematics.md
│   ├── 03-dynamics.md
│   └── ...
└── glossary/
    ├── _category_.json
    ├── a-terms.md  (Actuator, Algorithm, ...)
    ├── b-terms.md  (Bandwidth, Bearing, ...)
    └── ...
```

**Option B: Nested Chapter Structure** (For complex topics):
```
docs/
├── chapters/
│   ├── 01-fundamentals/
│   │   ├── _category_.json
│   │   ├── 01-introduction.md
│   │   ├── 02-mathematics.md
│   │   └── 03-physics.md
│   ├── 02-kinematics/
│   │   ├── _category_.json
│   │   ├── 01-forward-kinematics.md
│   │   └── 02-inverse-kinematics.md
│   └── ...
```

**Glossary Organization**:

**Option A: Alphabetical Single Files** (Simple, good for < 50 terms):
- One markdown file per letter: `a-terms.md`, `b-terms.md`
- Each file contains ### headings for individual terms

**Option B: One File Per Term** (Recommended for 200+ terms):
- `glossary/actuator.md`, `glossary/algorithm.md`, etc.
- Enables autogenerated sidebar with search
- Easier to maintain and cross-reference

**Sidebar Configuration**:

**Manual Configuration** (explicit control):
```typescript
sidebars: {
  chapters: [
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'chapters/01-introduction',
        'chapters/02-kinematics',
        // ... manually list all chapters
      ],
    },
  ],
}
```

**Autogenerated Configuration** (scales better):
```typescript
sidebars: {
  chapters: [
    {
      type: 'autogenerated',
      dirName: 'chapters',
    },
  ],
  glossary: [
    {
      type: 'autogenerated',
      dirName: 'glossary',
    },
  ],
}
```

**Recommendation**:
- Use **flat chapter structure initially**; migrate to nested if complexity increases
- Use **one file per glossary term** with autogenerated sidebar
- Use **manual sidebar** for chapters to control ordering (educational progression)
- Use **autogenerated sidebar** for glossary (alphabetical ordering acceptable)

**Source**: [Docusaurus Sidebar Guide](https://docusaurus.io/docs/sidebar), [Content Organization Best Practices](https://docusaurus.io/docs/docs-introduction)

### 5. TypeScript + Docusaurus Integration

**Question**: Are there any known issues or limitations with TypeScript strict mode in Docusaurus 3.x?

**Findings**:

**Compatibility**:
- Docusaurus 3.x is built with TypeScript; full support guaranteed
- Default tsconfig.json has `"strict": false` for flexibility
- Enabling strict mode is safe but requires fixing type issues in custom components

**Known Considerations**:
1. **MDX Files**: `.mdx` files don't benefit from TypeScript checking; use `.md` for content
2. **Config Files**: `docusaurus.config.ts` and `sidebars.ts` get full type checking
3. **Custom Components**: Import types from `@docusaurus/types` for props validation
4. **Plugin Types**: Third-party plugins may lack TypeScript definitions; use `@ts-ignore` sparingly

**Type-Safe Patterns**:
```typescript
// docusaurus.config.ts
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  // Fully type-checked configuration
};

// Custom component
import type {Props} from '@theme/DocItem';

export default function CustomDocItem(props: Props): JSX.Element {
  // Type-safe component
}
```

**Strict Mode Issues**:
- May require `skipLibCheck: true` if third-party types conflict
- Use `any` only for truly dynamic content (e.g., MDX metadata)

**Recommendation**: Enable strict mode after initial setup; fix type errors incrementally

**Source**: [Docusaurus TypeScript Support](https://docusaurus.io/docs/typescript-support)

## Decisions

### Primary Technology Stack

| Technology | Version | Rationale |
|------------|---------|-----------|
| **Docusaurus** | 3.x (latest) | Modern React-based SSG with excellent TypeScript support, active community, optimized for documentation |
| **TypeScript** | 5.x | Type safety, better developer experience, constitution requirement |
| **Tailwind CSS** | v3.x (stable) | Utility-first styling, constitution standard, defer v4 until stable |
| **Node.js** | 18.0+ LTS | Docusaurus 3.x requirement, long-term support |
| **React** | 18+ | Bundled with Docusaurus 3.x, modern concurrent rendering |

### Deployment Strategy

**Choice**: GitHub Actions automated deployment to gh-pages branch

**Rationale**:
- Automated CI/CD reduces manual deployment errors
- No build artifacts committed to main branch (cleaner git history)
- Easy to add additional checks (linting, testing) before deployment
- GitHub Actions free for public repositories

**Implementation**: Create `.github/workflows/deploy.yml` after manual deployment verification

### Content Organization

**Choice**:
- Flat chapter structure with manual sidebar configuration
- One file per glossary term with autogenerated sidebar

**Rationale**:
- Manual chapter sidebar allows explicit educational progression ordering
- One file per term scales to 200+ glossary entries without unwieldy files
- Autogenerated glossary sidebar enables alphabetical browsing without maintenance
- Flat structure easier to navigate during initial content authoring

### Tailwind CSS Version

**Choice**: Tailwind CSS v3.x (stable) instead of v4 beta

**Rationale**:
- v4 is in beta; potential breaking changes before stable release
- v3.x proven integration pattern with Docusaurus
- Can migrate to v4 in future task when stable
- Constitution requires Tailwind but doesn't mandate v4 beta

**Migration Path**: Document Tailwind v4 migration as future enhancement when stable

## Alternatives Considered

### Alternative Documentation Frameworks

| Framework | Pros | Cons | Verdict |
|-----------|------|------|---------|
| **VitePress** | Lightweight, fast builds, Vue 3 | Smaller plugin ecosystem, less feature-rich | ❌ Rejected: Less extensible for future features (chatbot, personalization) |
| **Nextra** | Modern, Next.js based | Tightly coupled to Next.js, complex deployment | ❌ Rejected: Over-engineered for documentation use case |
| **MkDocs** | Python-based, simple | Different ecosystem, no React/TypeScript | ❌ Rejected: Team prefers TypeScript ecosystem |
| **GitBook** | Polished UI, hosted solution | Vendor lock-in, limited customization | ❌ Rejected: Need full control for RAG integration |
| **Docusaurus 3.x** | React-based, extensible, active community | Heavier than VitePress | ✅ **Selected**: Best balance of features and extensibility |

### Alternative Deployment Methods

| Method | Pros | Cons | Verdict |
|--------|------|------|---------|
| **GitHub Actions** | Automated, no manual steps | Requires YAML configuration | ✅ **Selected**: Industry standard for CI/CD |
| **Manual `npm run deploy`** | Simple, no config needed | Manual process, error-prone | ⚠️ Use for initial testing only |
| **Vercel/Netlify** | Fast CDN, preview deploys | Third-party dependency, cost for private repos | ❌ Rejected: GitHub Pages free and sufficient |

### Alternative Content Structures

| Structure | Pros | Cons | Verdict |
|-----------|------|------|---------|
| **Flat chapters + manual sidebar** | Explicit control, simple navigation | Doesn't scale to 100+ chapters | ✅ **Selected**: Sufficient for 20-30 chapters |
| **Nested chapters + autogenerated sidebar** | Scales well, self-organizing | Less control over ordering | ❌ Defer: Complexity not needed yet |
| **Single glossary file** | Simple, all terms in one place | Unwieldy for 200+ terms, hard to navigate | ❌ Rejected: Doesn't scale |
| **One file per term + autogenerated sidebar** | Scales infinitely, easy to maintain | Many files to manage | ✅ **Selected**: Best for large glossaries |

## Implementation Recommendations

### Phase 1: Initialize (Priority: P1)

1. Run `npx create-docusaurus@latest frontend classic --typescript`
2. Verify default structure and dependencies
3. Enable TypeScript strict mode in `tsconfig.json`
4. Verify build and dev server work

### Phase 2: Configure Deployment (Priority: P1)

1. Update `docusaurus.config.ts` with GitHub Pages settings
2. Test manual deployment with `GIT_USER=[username] npm run deploy`
3. Verify site loads at `https://[username].github.io/Robotics_book_chatbot/`
4. Create GitHub Actions workflow for automated deployment

### Phase 3: Set Up Content Structure (Priority: P2)

1. Create `docs/chapters/` directory with README.md
2. Create `docs/glossary/` directory with README.md
3. Update `sidebars.ts` with manual chapters sidebar and autogenerated glossary sidebar
4. Add sample chapter and glossary term for testing

### Phase 4: Integrate Tailwind CSS (Priority: P2)

1. Install `tailwindcss@latest` (v3.x stable)
2. Configure PostCSS and Tailwind config
3. Update `src/css/custom.css` with Tailwind directives
4. Test Tailwind utilities in custom component

### Phase 5: Documentation & Verification (Priority: P3)

1. Update `frontend/README.md` with setup instructions
2. Document content authoring guidelines
3. Verify all acceptance criteria from spec.md
4. Create PHR for implementation session

## Open Questions

✅ **Q1: Should we use Tailwind v4 beta or v3.x stable?**
- **Answer**: Use Tailwind v3.x stable. Migrate to v4 when stable release is available.

✅ **Q2: Manual or autogenerated sidebars for chapters?**
- **Answer**: Manual sidebar for chapters (explicit ordering), autogenerated for glossary (alphabetical).

✅ **Q3: Flat or nested chapter structure?**
- **Answer**: Start with flat structure. Migrate to nested if complexity increases beyond 30 chapters.

✅ **Q4: Should we disable the blog feature?**
- **Answer**: Yes, set `blog: false` in preset configuration. Not needed for textbook.

## References

- [Docusaurus 3.x Documentation](https://docusaurus.io/docs)
- [Tailwind CSS v3 Documentation](https://tailwindcss.com/docs)
- [GitHub Pages Deployment](https://docs.github.com/en/pages)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/)
- [React 18 Documentation](https://react.dev/)

## Next Steps

1. ✅ Research complete - all unknowns resolved
2. ➡️ Proceed to Phase 1: Generate `quickstart.md` with step-by-step setup instructions
3. ➡️ Run `/sp.tasks` to generate detailed task breakdown in `tasks.md`
4. ➡️ Execute implementation following task list
5. ➡️ Verify against acceptance criteria and success metrics
