# Robotics Textbook Chatbot

An interactive educational platform for Physical AI & Humanoid Robotics with an intelligent RAG-powered chatbot for real-time Q&A.

## 🎯 Overview

This project is a comprehensive educational platform combining a Docusaurus-based textbook with an AI-powered chatbot that answers questions using Retrieval-Augmented Generation (RAG). The system provides students with an interactive learning experience where they can select text from chapters and ask questions to get contextual, accurate answers from the course material.

## 🏗️ Architecture

The project consists of two main components:

### Frontend (Docusaurus + React + TypeScript)
- **Framework**: Docusaurus v3 for static site generation
- **UI**: React with TypeScript for type safety
- **Chatbot Widget**: ChatKit integration with custom theme adaptation
- **Text Selection**: Interactive tooltip for asking questions about selected text
- **Responsive Design**: Mobile-friendly with theme toggle (light/dark mode)

### Backend (FastAPI + Python)
- **API Framework**: FastAPI for high-performance REST endpoints
- **Vector Database**: Qdrant for semantic search
- **Embeddings**: Google Gemini for text embeddings
- **LLM**: Google Gemini for answer generation
- **RAG Pipeline**: Custom implementation with chapter-aware context filtering

## ✨ Features

### 1. Interactive Textbook (Docusaurus)
- 📚 Structured chapters covering Physical AI & Humanoid Robotics
- 📖 Searchable glossary with technical terms
- 🎨 Beautiful, responsive UI with light/dark theme
- 📱 Mobile-optimized reading experience

### 2. RAG-Powered Chatbot
- 💬 Real-time Q&A using course material
- 🔍 Semantic search across all chapters
- 🎯 Context-aware answers with source citations
- 🧠 Powered by Google Gemini LLM

### 3. Text Selection Feature
- ✨ Select any text in the textbook
- ❓ Click "Ask about this" to query the chatbot
- 🎯 Context-aware questions with selected text pre-filled
- 🔄 Persistent across multiple selections (no page refresh needed)

### 4. Chapter Context Awareness
- 📍 Automatically detects which chapter you're reading
- 🎯 Prioritizes answers from the current chapter
- 🔗 Cross-references related chapters when relevant

### 5. Theme Integration
- 🌓 Automatic theme sync (light/dark mode)
- 🎨 Seamless integration with Docusaurus theme
- ⚡ Smooth transitions (300ms)

### 6. Production-Ready API
- 🚀 Environment-aware configuration (dev/staging/prod)
- 🔒 CORS protection
- ⚡ Optimized response times
- 📊 Comprehensive error handling

## 🛠️ Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x
- **Language**: TypeScript 5.x
- **UI Library**: React 18.x
- **Styling**: CSS Modules + CSS Variables
- **Build Tool**: Webpack (via Docusaurus)
- **Testing**: Playwright (E2E), Jest (Unit)

### Backend
- **Framework**: FastAPI 0.104+
- **Language**: Python 3.11+
- **Vector DB**: Qdrant
- **Embeddings**: Google Gemini (text-embedding-004)
- **LLM**: Google Gemini (gemini-1.5-flash)
- **HTTP Client**: httpx (async)
- **CORS**: FastAPI middleware

### Infrastructure
- **Deployment**: GitHub Pages (frontend), Cloud hosting (backend)
- **Version Control**: Git + GitHub
- **Package Management**: npm (frontend), Poetry/pip (backend)

## 📦 Installation

### Prerequisites
- Node.js 18+ and npm
- Python 3.11+
- Git
- Qdrant instance (local or cloud)
- Google Gemini API key

### Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Configure API endpoint in .env
# REACT_APP_API_URL=http://localhost:8000/api

# Start development server
npm start
```

The frontend will be available at `http://localhost:3000`

### Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
# OR if using Poetry:
poetry install

# Create .env file with required variables
cat > .env << EOF
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key_if_cloud
EOF

# Run database migrations (if any)
# python -m alembic upgrade head

# Start development server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The backend API will be available at `http://localhost:8000`

### Qdrant Setup

```bash
# Option 1: Run Qdrant locally with Docker
docker run -p 6333:6333 qdrant/qdrant

# Option 2: Use Qdrant Cloud
# Sign up at https://cloud.qdrant.io and get your cluster URL and API key
```

## 🚀 Usage

### For Students

1. **Browse Chapters**: Navigate through the textbook chapters
2. **Select Text**: Highlight any text you want to learn more about
3. **Ask Questions**: Click the "Ask about this" tooltip or use the chatbot widget
4. **Get Answers**: Receive contextual answers from the course material
5. **Explore More**: Use the chatbot to ask follow-up questions

### For Developers

1. **Add New Chapters**: Create markdown files in `frontend/docs/chapters/`
2. **Update Glossary**: Add terms in `frontend/docs/glossary/terms/`
3. **Generate Embeddings**: Run the embedding script to index new content
4. **Test Changes**: Run unit and E2E tests
5. **Deploy**: Push to GitHub for automatic deployment

## 📁 Project Structure

```
Robotics_book_chatbot/
├── frontend/                      # Docusaurus frontend
│   ├── docs/                      # Markdown content
│   │   ├── chapters/              # Textbook chapters
│   │   ├── glossary/              # Technical glossary
│   │   └── intro.md               # Landing page
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatKit/           # Chatbot widget
│   │   │   └── SelectionTooltip/  # Text selection tooltip
│   │   ├── hooks/                 # Custom React hooks
│   │   │   ├── useTextSelection.ts
│   │   │   └── useSelectionTooltip.ts
│   │   ├── theme/                 # Docusaurus theme customization
│   │   │   └── Root.tsx           # Root layout with integrations
│   │   ├── config/                # Configuration modules
│   │   │   └── api.ts             # API endpoint configuration
│   │   └── utils/                 # Utility functions
│   ├── static/                    # Static assets
│   ├── docusaurus.config.js       # Docusaurus configuration
│   ├── sidebars.js                # Sidebar navigation
│   └── package.json
│
├── backend/                       # FastAPI backend
│   ├── src/
│   │   ├── api/
│   │   │   └── v1/
│   │   │       └── routes/
│   │   │           └── chat.py    # Chat endpoints
│   │   ├── services/
│   │   │   ├── rag_service.py     # RAG orchestration
│   │   │   ├── qdrant_service.py  # Vector DB operations
│   │   │   ├── embedding_service.py  # Text embeddings
│   │   │   ├── gemini_service.py  # LLM integration
│   │   │   └── utils/
│   │   │       └── chapter_filter.py  # Chapter context
│   │   ├── models/                # Pydantic models
│   │   ├── middleware/            # API middleware
│   │   ├── config.py              # Configuration
│   │   └── main.py                # FastAPI app entry
│   ├── pyproject.toml
│   └── README.md
│
├── specs/                         # Feature specifications
│   ├── 001-docusaurus-init/
│   ├── 002-textbook-glossary/
│   ├── 003-qdrant-embeddings/
│   ├── 004-rag-chatbot-api/
│   ├── 005-chatkit-integration/
│   ├── 006-selected-text/
│   ├── 007-chapter-context/
│   └── 008-docusaurus-theme-config/
│
├── Project_flow/                  # Development guides
│   └── Detailed_Phase_Breakdown.md
│
└── README.md                      # This file
```

## 🧪 Testing

### Frontend Tests

```bash
cd frontend

# Run unit tests
npm run test:unit

# Run E2E tests
npm run test:e2e

# Run E2E tests in headed mode (with browser UI)
npm run test:e2e:headed

# Run specific test file
npm run test:e2e -- tests/chatkit-theme.spec.ts
```

### Backend Tests

```bash
cd backend

# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test file
pytest tests/test_rag_service.py -v
```

## 🌐 Deployment

### Frontend Deployment (GitHub Pages)

```bash
cd frontend

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### Backend Deployment

The backend can be deployed to any cloud platform that supports Python applications:

- **Railway**: `railway up`
- **Render**: Connect GitHub repo and deploy
- **Google Cloud Run**: `gcloud run deploy`
- **AWS Lambda**: Use Mangum adapter
- **Heroku**: `git push heroku main`

**Environment Variables for Production:**
```bash
GEMINI_API_KEY=your_production_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
ALLOWED_ORIGINS=https://yourdomain.com
```

## 🔧 Configuration

### API Configuration

Set the backend API endpoint using environment variables:

```bash
# Development (default)
REACT_APP_API_URL=http://localhost:8000/api npm start

# Production
NODE_ENV=production REACT_APP_API_URL=https://api.your-domain.com/api npm run build
```

### Theme Configuration

ChatKit automatically syncs with Docusaurus theme. No manual setup required.

### Qdrant Collections

The system uses the following collections:
- `robotics_book_chapters` - Chapter embeddings with metadata

## 📚 Development Workflow

1. **Create Feature Spec**: Document requirements in `specs/`
2. **Implement Backend**: Add API endpoints and services
3. **Implement Frontend**: Create UI components
4. **Write Tests**: Add unit and E2E tests
5. **Update Documentation**: Keep README and specs updated
6. **Create PR**: Submit for review
7. **Deploy**: Merge and deploy to production

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'feat: add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- **Docusaurus** for the amazing documentation framework
- **FastAPI** for the high-performance backend framework
- **Qdrant** for semantic search capabilities
- **Google Gemini** for embeddings and LLM
- **ChatKit** for the chat widget integration

## 📞 Support

For questions or issues:
- Create an issue on GitHub
- Contact the development team
- Check the documentation in `specs/`

---

**Built with ❤️ for robotics education**
