# Quickstart Guide: Docusaurus Initialization

**Feature**: 001-docusaurus-init
**Date**: 2025-12-03
**For**: Developers setting up the robotics textbook platform

## Prerequisites

Before starting, ensure you have:

- ✅ **Node.js 18.0+** installed ([Download](https://nodejs.org/))
- ✅ **npm 9.0+** or yarn 3.0+ (comes with Node.js)
- ✅ **Git** configured with GitHub authentication
- ✅ **GitHub repository** created: `Robotics_book_chatbot`
- ✅ **Code editor** (VS Code recommended with TypeScript/React extensions)
- ✅ **Internet connection** for downloading dependencies

### Verify Prerequisites

```bash
# Check Node.js version (should be 18.0 or higher)
node --version

# Check npm version (should be 9.0 or higher)
npm --version

# Check Git version
git --version

# Verify GitHub authentication
git config --global user.name
git config --global user.email
```

## Step-by-Step Setup

### Step 1: Initialize Docusaurus Project

Navigate to your project root and create the Docusaurus project:

```bash
# From repository root (Robotics_book_chatbot/)
npx create-docusaurus@latest frontend classic --typescript

# Expected output:
# ✔ Created frontend
# ✔ Success! Created frontend at /path/to/Robotics_book_chatbot/frontend
```

**What this does**:
- Creates `frontend/` directory with Docusaurus classic template
- Installs all dependencies (React, Docusaurus, TypeScript, etc.)
- Sets up default configuration files
- Takes 2-4 minutes depending on internet speed

**Verify**:
```bash
cd frontend
ls -la

# You should see:
# - package.json
# - tsconfig.json
# - docusaurus.config.ts
# - sidebars.ts
# - docs/
# - src/
# - static/
```

### Step 2: Enable TypeScript Strict Mode

Edit `frontend/tsconfig.json` to enable strict mode for type safety:

```json
{
  "extends": "@docusaurus/tsconfig",
  "compilerOptions": {
    "strict": true,  // 👈 Add this line (default is false)
    "baseUrl": "."
  }
}
```

**Why**: Constitution requires TypeScript strict mode for production-grade quality.

### Step 3: Test Default Installation

Verify the default setup works:

```bash
# From frontend/ directory
npm start

# Expected output:
# [INFO] Starting the development server...
# [SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

**Verify**:
1. Open browser to `http://localhost:3000`
2. Default Docusaurus homepage should load
3. Navigation links should work
4. No console errors

**Stop the server**: Press `Ctrl+C` in terminal

### Step 4: Configure GitHub Pages Deployment

Edit `frontend/docusaurus.config.ts` to add GitHub Pages settings.

**Find these lines** (around line 10):
```typescript
const config: Config = {
  title: 'My Site',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',
```

**Replace with**:
```typescript
const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Comprehensive robotics education for Panaversity students',
  favicon: 'img/favicon.ico',

  // GitHub Pages configuration
  url: 'https://YOUR_GITHUB_USERNAME.github.io',  // 👈 Replace with your username
  baseUrl: '/Robotics_book_chatbot/',              // 👈 Repository name
  organizationName: 'YOUR_GITHUB_USERNAME',        // 👈 Replace with your username
  projectName: 'Robotics_book_chatbot',            // Repository name
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',  // Fail build on broken links (good practice)
  onBrokenMarkdownLinks: 'warn',
```

**Important**: Replace `YOUR_GITHUB_USERNAME` with your actual GitHub username!

**Also update** the `i18n` configuration (around line 30):
```typescript
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
    // Future: Add 'ur' for Urdu translation feature
  },
```

**Disable blog** (around line 40):
```typescript
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/YOUR_GITHUB_USERNAME/Robotics_book_chatbot/tree/main/frontend/',
        },
        blog: false,  // 👈 Change from default blog settings to false
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
```

### Step 5: Create Content Directories

Create organized folders for chapters and glossary:

```bash
# From frontend/ directory
mkdir -p docs/chapters
mkdir -p docs/glossary
```

**Create `docs/chapters/README.md`**:
```bash
cat > docs/chapters/README.md << 'EOF'
---
sidebar_position: 1
---

# Chapters

This directory contains all textbook chapters for the Physical AI & Humanoid Robotics course.

## How to Add a New Chapter

1. Create a new markdown file: `NN-chapter-title.md` (e.g., `01-introduction.md`)
2. Add frontmatter at the top:
   ```markdown
   ---
   sidebar_position: NN
   sidebar_label: "Chapter NN: Title"
   ---
   ```
3. Write your content using markdown and MDX (React components)
4. Update `sidebars.ts` to include the new chapter

## Chapter Naming Convention

- Use two-digit prefix: `01-`, `02-`, `03-`, etc.
- Use kebab-case for file names: `inverse-kinematics.md`
- Sidebar will display titles from frontmatter

## Example Chapter Structure

```markdown
---
sidebar_position: 1
sidebar_label: "Chapter 1: Introduction"
---

# Chapter 1: Introduction to Robotics

Brief chapter overview...

## 1.1 What is a Robot?

Content...

## 1.2 History of Robotics

Content...
\```
EOF
```

**Create `docs/glossary/README.md`**:
```bash
cat > docs/glossary/README.md << 'EOF'
---
sidebar_position: 1
---

# Glossary

Technical terms and definitions for robotics concepts.

## How to Add a Glossary Entry

1. Create a new markdown file named after the term: `actuator.md`, `kinematics.md`
2. Use this template:
   ```markdown
   ---
   sidebar_label: "Term Name"
   ---

   # Term Name

   **Definition**: [Clear, concise definition]

   **Context**: [Where this term is used in robotics]

   **Example**: [Practical example or usage]

   **Related Terms**: [Links to related glossary entries]
   \```

3. The sidebar will be automatically generated in alphabetical order

## Glossary Naming Convention

- Use lowercase, kebab-case file names: `inverse-kinematics.md`
- One term per file
- Cross-reference related terms using markdown links
EOF
```

### Step 6: Configure Sidebar Navigation

Edit `frontend/sidebars.ts` to set up chapters and glossary sections:

**Replace entire file** with:
```typescript
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics Textbook
 *
 * - Chapters: Manually configured for explicit educational ordering
 * - Glossary: Auto-generated for alphabetical browsing
 */
const sidebars: SidebarsConfig = {
  // Chapters sidebar - manual configuration for explicit ordering
  chapters: [
    {
      type: 'category',
      label: 'Chapters',
      collapsed: false,  // Keep expanded by default
      items: [
        'chapters/README',
        // Add chapters here as they are created:
        // 'chapters/01-introduction',
        // 'chapters/02-kinematics',
        // 'chapters/03-dynamics',
        // etc.
      ],
    },
  ],

  // Glossary sidebar - auto-generated for alphabetical ordering
  glossary: [
    {
      type: 'category',
      label: 'Glossary',
      collapsed: false,
      items: [
        'glossary/README',
        // Auto-generate all other glossary entries:
        {
          type: 'autogenerated',
          dirName: 'glossary',
        },
      ],
    },
  ],
};

export default sidebars;
```

**Update navbar** in `docusaurus.config.ts` (around line 60):
```typescript
  themeConfig: {
    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Panaversity Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'chapters',  // 👈 Reference chapters sidebar
          position: 'left',
          label: 'Chapters',
        },
        {
          type: 'docSidebar',
          sidebarId: 'glossary',  // 👈 Reference glossary sidebar
          position: 'left',
          label: 'Glossary',
        },
        {
          href: 'https://github.com/YOUR_GITHUB_USERNAME/Robotics_book_chatbot',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    // ... rest of theme config
  }
```

### Step 7: Test Updated Configuration

Verify all changes work:

```bash
# From frontend/ directory
npm start
```

**Verify**:
1. Homepage loads at `http://localhost:3000/`
2. **"Chapters"** link in navbar leads to chapters README
3. **"Glossary"** link in navbar leads to glossary README
4. No console errors
5. Navigation sidebar shows chapters and glossary sections

**Test build**:
```bash
# Stop dev server (Ctrl+C)
npm run build

# Expected output:
# [SUCCESS] Generated static files in "build".
# [INFO] Use `npm run serve` to test the build.
```

**Test production build**:
```bash
npm run serve

# Open http://localhost:3000/Robotics_book_chatbot/
# Verify everything works with baseUrl prefix
```

### Step 8: (Optional) Install Tailwind CSS v3

**Note**: This step is optional but recommended per constitution standards.

```bash
# From frontend/ directory
npm install -D tailwindcss@latest postcss autoprefixer
npx tailwindcss init
```

**Create `postcss.config.js`**:
```javascript
module.exports = {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
};
```

**Edit `tailwind.config.js`**:
```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
  ],
  darkMode: ['class', '[data-theme="dark"]'],  // Docusaurus dark mode integration
  theme: {
    extend: {},
  },
  plugins: [],
  corePlugins: {
    preflight: false,  // Disable Tailwind's base styles to avoid conflicts with Docusaurus
  },
};
```

**Edit `src/css/custom.css`** (add at the top):
```css
@tailwind base;
@tailwind components;
@tailwind utilities;

/* Existing Docusaurus CSS below... */
:root {
  /* ... */
}
```

**Test Tailwind**:
Create `src/components/TestTailwind.tsx`:
```tsx
export default function TestTailwind(): JSX.Element {
  return (
    <div className="bg-blue-500 text-white p-4 rounded-lg">
      <p className="text-xl font-bold">Tailwind CSS is working!</p>
    </div>
  );
}
```

Add to `src/pages/index.tsx` to verify it works.

### Step 9: Manual GitHub Pages Deployment (Test)

Before setting up automated deployment, test manual deployment:

```bash
# From frontend/ directory
GIT_USER=YOUR_GITHUB_USERNAME npm run deploy

# This will:
# 1. Build the site
# 2. Push build/ folder to gh-pages branch
# 3. Trigger GitHub Pages deployment
```

**Configure GitHub Pages**:
1. Go to repository Settings > Pages
2. Source: Deploy from a branch
3. Branch: `gh-pages` / `(root)`
4. Click Save

**Wait 2-3 minutes**, then visit:
```
https://YOUR_GITHUB_USERNAME.github.io/Robotics_book_chatbot/
```

**Troubleshooting**:
- If assets don't load, verify `baseUrl` in `docusaurus.config.ts`
- If 404 error, check GitHub Pages is enabled and branch is correct
- Check GitHub Actions tab for deployment logs

### Step 10: Set Up Automated Deployment (GitHub Actions)

Create `.github/workflows/deploy.yml` in repository root:

```bash
# From repository root (not frontend/)
mkdir -p .github/workflows
```

**Create `deploy.yml`**:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main
    paths:
      - 'frontend/**'  # Only trigger on frontend changes
  workflow_dispatch:  # Allow manual trigger

permissions:
  contents: write  # Required for gh-pages deployment

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0  # Fetch all history for git info

      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
          cache-dependency-path: frontend/package-lock.json

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
          cname: false  # No custom domain
```

**Commit and push**:
```bash
git add .github/workflows/deploy.yml
git commit -m "feat: add GitHub Actions deployment workflow"
git push origin main
```

**Verify**:
1. Go to repository > Actions tab
2. Watch "Deploy to GitHub Pages" workflow run
3. After completion (2-3 minutes), site should update automatically

## Success Verification Checklist

✅ **Project Initialization** (SC-001):
- [ ] Docusaurus project created in `frontend/` directory (< 5 minutes)
- [ ] TypeScript strict mode enabled in `tsconfig.json`
- [ ] Dependencies installed without errors

✅ **Local Development** (SC-002, FR-006):
- [ ] `npm start` completes in < 30 seconds
- [ ] Dev server loads at `http://localhost:3000/`
- [ ] Homepage renders without errors
- [ ] Navigation links work (Chapters, Glossary)

✅ **Build Process** (SC-003, FR-007):
- [ ] `npm run build` completes without errors
- [ ] Static files generated in `frontend/build/`
- [ ] `npm run serve` serves production build successfully

✅ **Content Structure** (SC-004, FR-003, FR-004):
- [ ] `docs/chapters/` directory exists with README.md
- [ ] `docs/glossary/` directory exists with README.md
- [ ] README files accessible through navigation

✅ **GitHub Pages Deployment** (SC-005, FR-002):
- [ ] `docusaurus.config.ts` has correct GitHub Pages settings
- [ ] Manual deployment (`npm run deploy`) succeeds
- [ ] Site accessible at `https://USERNAME.github.io/Robotics_book_chatbot/`
- [ ] All assets (CSS, JS, images) load correctly

✅ **Sidebar Navigation** (FR-005):
- [ ] `sidebars.ts` configured with chapters and glossary sections
- [ ] Navigation menu displays in site
- [ ] Clicking nav items navigates correctly

✅ **GitHub Actions** (Automation):
- [ ] `.github/workflows/deploy.yml` created
- [ ] Workflow runs on push to main
- [ ] Deployment succeeds automatically

## Common Issues & Solutions

### Issue: `npm start` fails with port 3000 already in use

**Solution**:
```bash
# Use a different port
npm start -- --port 3001
```

### Issue: TypeScript strict mode causes errors

**Solution**:
```bash
# Temporarily disable strict mode, fix errors incrementally
# In tsconfig.json, set "strict": false
# Fix type errors one file at a time, then re-enable
```

### Issue: GitHub Pages shows 404

**Solution**:
1. Verify `baseUrl` matches repository name: `/Robotics_book_chatbot/`
2. Check GitHub repository Settings > Pages is enabled
3. Ensure `gh-pages` branch exists and is selected as source

### Issue: Assets don't load on GitHub Pages

**Solution**:
- Check `url` and `baseUrl` in `docusaurus.config.ts`
- Should be: `url: 'https://USERNAME.github.io'`, `baseUrl: '/Robotics_book_chatbot/'`
- Rebuild and redeploy

### Issue: `npm run deploy` fails with authentication error

**Solution**:
```bash
# Set GIT_USER environment variable
GIT_USER=YOUR_GITHUB_USERNAME npm run deploy

# Or configure SSH authentication
git remote set-url origin git@github.com:USERNAME/Robotics_book_chatbot.git
```

## Next Steps

1. ✅ Complete initialization and verify all checkboxes above
2. ➡️ Run `/sp.tasks` to generate detailed implementation task breakdown
3. ➡️ Add actual chapter content to `docs/chapters/`
4. ➡️ Add glossary terms to `docs/glossary/`
5. ➡️ Customize theme and branding (logo, colors)
6. ➡️ Set up backend API for RAG chatbot (future feature)

## Additional Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [Markdown Guide](https://docusaurus.io/docs/markdown-features)
- [MDX Documentation](https://mdxjs.com/)
- [GitHub Pages Guide](https://docs.github.com/en/pages)
- [TypeScript Handbook](https://www.typescriptlang.org/docs/)

## Support

- **GitHub Issues**: [Report bugs or request features](https://github.com/YOUR_GITHUB_USERNAME/Robotics_book_chatbot/issues)
- **Panaversity Community**: [Join Discord for support](#)
- **Docusaurus Discord**: [Get help from Docusaurus community](https://discord.gg/docusaurus)
