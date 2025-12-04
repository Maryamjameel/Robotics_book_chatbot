# Physical AI & Humanoid Robotics Textbook Frontend

This directory contains the frontend for the Physical AI & Humanoid Robotics Textbook, powered by [Docusaurus](https://docusaurus.io/).

## Setup and Installation

1.  **Install dependencies**:
    ```bash
    npm install
    ```

2.  **Start the development server**:
    ```bash
    npm start
    ```
    (Access at `http://localhost:3000`)

3.  **Build for production**:
    ```bash
    npm run build
    ```

4.  **Serve production build locally**:
    ```bash
    npm run serve
    ```
    (Access at `http://localhost:3000/Robotics_book_chatbot/`)

## Content Organization

- **Chapters**: Located in `docs/chapters/`
- **Glossary**: Located in `docs/glossary/`

## Deployment

Configured for GitHub Pages deployment. Automated via GitHub Actions on push to `main` branch. Manual deployment:

```bash
# From frontend/ directory
GIT_USER=YOUR_GITHUB_USERNAME npm run deploy
```

## No build warnings or issues observed during initial setup.