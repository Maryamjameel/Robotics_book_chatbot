# Detailed Phase Breakdown - Hackathon Execution Guide

Complete execution guide with tech stack, agents/skills, prompts, deliverables, and folder structure for each phase.

---

## **PROJECT 1: DOCUSAURUS BOOK CREATION**

**Project Folder:** `/frontend`

---

### **Phase 1: Setup & Planning**

#### **📋 Description**
Initialize Docusaurus project with TypeScript, configure for GitHub Pages deployment, establish folder structure, define 3-chapter structure, and create detailed outlines with learning objectives.

#### **🛠️ Tech Stack**
- **Framework**: Docusaurus v3 (React-based)
- **Language**: TypeScript
- **Package Manager**: npm
- **Deployment**: GitHub Pages
- **Version Control**: Git
- **Markdown**: Chapter structure
- **YAML**: Frontmatter metadata

#### **🤖 Agents & Skills**
- **Skill**: `Environment_Configuration_Skill` - Setting up .env files and configurations
- **Skill**: `Chapter_Outline_Skill` ⭐ - Generate structured chapter outlines
- **Agent**: `textbook-author` - Validate chapter structure

#### **💬 Prompts to Use**

**Prompt 1: Initialize Project**
```
Initialize Docusaurus project for robotics textbook:
- Create project: npx create-docusaurus@latest frontend classic --typescript
- Configure docusaurus.config.js for GitHub Pages
- Set up folder structure: docs/chapters/, docs/glossary/
- Configure sidebar navigation
- Test local server

Use Environment_Configuration_Skill.
```

**Prompt 2: Create Chapter Outlines**
```
Create outlines for 3 robotics chapters based on the Physical AI & Humanoid Robotics course:

Chapter 1: Introduction to Physical AI & ROS 2
  - Covers: Foundations of Physical AI (Weeks 1-2) + ROS 2 Fundamentals (Weeks 3-5)
  - Topics: Embodied intelligence, ROS 2 architecture, nodes/topics/services, URDF

Chapter 2: Robot Simulation & AI Perception
  - Covers: Gazebo & Unity (Weeks 6-7) + NVIDIA Isaac Platform (Weeks 8-10)
  - Topics: Physics simulation, sensor simulation, Isaac Sim, VSLAM, Nav2

Chapter 3: Vision-Language-Action for Robotics
  - Covers: Humanoid Development (Weeks 11-12) + VLA (Week 13)
  - Topics: Kinematics, bipedal locomotion, Whisper, LLMs, cognitive planning

For each chapter:
- 3-5 measurable learning outcomes
- Section hierarchy (##, ###)
- Key concepts and terminology
- 2-3 worked examples planned
- Equations and algorithms to cover
- Code examples (Python/ROS 2)

Reference: Project_flow/Minimal_Chapter_Structure.md for detailed structure

Use Chapter_Outline_Skill to generate detailed outlines.
Save to: frontend/docs/chapters/chapter-0X-outline.md
```

#### **📦 Deliverables**
- ✅ Docusaurus project initialized at `/frontend`
- ✅ TypeScript configuration complete
- ✅ Folder structure created (`docs/chapters/`, `docs/glossary/`)
- ✅ `docusaurus.config.js` configured for GitHub Pages
- ✅ Local dev server running on `http://localhost:3000`
- ✅ Git repository initialized
- ✅ 3 detailed chapter outlines (markdown files)
- ✅ Learning outcomes defined (3-5 per chapter)
- ✅ Section hierarchy established
- ✅ Key concepts identified
- ✅ Sidebar navigation configured
- ✅ Landing page written

#### **📁 Folder Structure**
```
frontend/
├── docs/
│   ├── chapters/
│   │   ├── chapter-01-outline.md
│   │   ├── chapter-02-outline.md
│   │   └── chapter-03-outline.md
│   ├── glossary/          # (empty, for later)
│   └── intro.md           # Landing page
├── static/
│   └── img/               # Images and diagrams
├── src/
│   ├── components/        # React components (for later)
│   ├── css/               # Custom styling
│   └── pages/             # Custom pages
├── docusaurus.config.js   # Main configuration
├── sidebars.js            # Sidebar navigation
├── package.json
└── tsconfig.json
```

---

### **Phase 2: Content Creation**

#### **📋 Description**
Write complete, academically rigorous chapters (2500-3000 words each) with code examples, LaTeX equations, and exercises. Then extract technical terms, create comprehensive glossary with definitions and cross-references.

#### **🛠️ Tech Stack**
- **Markdown**: Content authoring and glossary entries
- **LaTeX**: Mathematical equations (`$...$`, `$$...$$`)
- **Code blocks**: Python with syntax highlighting
- **Mermaid**: Diagrams (optional)
- **Docusaurus**: Internal linking for glossary

#### **🤖 Agents & Skills**
- **Agent**: `textbook-author` ⭐ PRIMARY - Write complete chapter content
- **Skill**: `Example_Generator_Skill` - Create worked examples
- **Skill**: `Code_Explanation_Skill` - Explain code snippets
- **Skill**: `Markdown_formatting_Skill` - Ensure proper markdown syntax
- **Agent**: `qa-validation-reviewer` - Quality check after each chapter
- **Agent**: `glossary-manager` ⭐ PRIMARY - Manage glossary entries
- **Skill**: `Glossary_Expansion_Skill` - Create definitions
- **Skill**: `Context_Extraction_Skill` - Identify terms from chapters

#### **💬 Prompts to Use**

**Prompt 1: Write Chapter (repeat for each chapter)**
```
Write complete Chapter 1: Introduction to Physical AI & ROS 2

Requirements:
- Follow outline from chapter-01-outline.md and Project_flow/Minimal_Chapter_Structure.md
- 3000 words
- Academic tone, clear explanations
- Include Python/ROS 2 code examples
- Use LaTeX for equations
- 2-3 worked examples
- Summary and key takeaways

Sections:
- Introduction (motivate Physical AI and embodied intelligence)
- 1.1 Foundations of Physical AI
  - Definition and principles
  - Digital AI vs Physical AI vs Embodied Intelligence
  - Why humanoids excel in human environments
- 1.2 The Humanoid Robotics Landscape
  - Current state (Unitree, Tesla, Boston Dynamics)
  - Sensor systems (LIDAR, cameras, IMUs)
  - Degrees of freedom and applications
- 1.3 The Robotic Nervous System: ROS 2
  - ROS 2 architecture (nodes, topics, services, actions)
  - Building ROS 2 packages with Python (rclpy)
  - Publisher/Subscriber example code
  - URDF (robot structure definition)
  - Simple 2-link arm URDF example
- 1.4 Summary & Key Takeaways

Use textbook-author agent to generate content.

After writing, use qa-validation-reviewer agent to check:
- Technical correctness
- ROS 2 code examples are valid
- Markdown formatting
- Learning outcomes met
- LaTeX syntax valid
```


**Phase 2 - Prompt 1b: Write Chapter 2**

  Write complete Chapter 2: Robot Simulation & AI Perception

  Requirements:
  - Follow outline from chapter-02-outline.md and Project_flow/Minimal_Chapter_Structure.md
  - 3000 words
  - Academic tone, clear explanations
  - Include Python/ROS 2 code examples
  - 2-3 worked examples
  - Summary and key takeaways

  Sections:
  - Introduction (motivate simulation in robotics development)
  - 2.1 Physics Simulation Foundations: Gazebo
    - Gazebo architecture and physics engines
    - SDF (Simulation Description Format)
    - Sensor simulation with realistic noise
    - Integration with ROS 2
  - 2.2 Advanced Simulation: NVIDIA Isaac Sim
    - Isaac Sim ecosystem and capabilities
    - Creating synthetic data for AI training
    - ROS 2 integration with Isaac
  - 2.3 Visual Perception Foundations
    - Camera models and projections
    - Point clouds (RGB-D, LiDAR)
    - Feature detection algorithms
  - 2.4 SLAM (Simultaneous Localization and Mapping)
    - Visual SLAM fundamentals
    - VSLAM algorithms and implementations
    - Loop closure detection
  - 2.5 Autonomous Navigation with Nav2
    - Nav2 architecture (global planner, local planner)
    - Costmaps and inflation layers
    - Behavior trees for navigation
  - 2.6 Summary & Key Takeaways

  Use textbook-author agent to generate content.

  After writing, use qa-validation-reviewer agent to check:
  - Technical correctness
  - Gazebo and Isaac Sim code examples are valid
  - SLAM algorithm descriptions are accurate
  - Markdown formatting
  - Learning outcomes met


  ---


**Phase 2 - Prompt 1c: Write Chapter 3**

  Write complete Chapter 3: Vision-Language-Action for Robotics

  Requirements:
  - Follow outline from chapter-03-vision-language-action.md and Project_flow/Minimal_Chapter_Structure.md
  - 3000 words
  - Academic tone, clear explanations
  - Include Python/ROS 2 code examples
  - 2-3 worked examples
  - Summary and key takeaways

  Sections:
  - Introduction (motivate humanoid robots with VLA capabilities)
  - 3.1 Humanoid Arm Kinematics
    - Denavit-Hartenberg parameters
    - Forward kinematics derivation
    - Jacobian and workspace analysis
    - Inverse kinematics methods
  - 3.2 Bipedal Locomotion
    - Zero Moment Point (ZMP) theory
    - Inverted pendulum model
    - Gait patterns (walking, running)
    - Balance recovery strategies
  - 3.3 LLMs and Whisper for Robotics
    - Transformer architecture basics
    - OpenAI Whisper for speech recognition
    - LLM prompt engineering for robotics
    - Integrating with robot tasks
  - 3.4 Vision-Language-Action Integration
    - Vision-language models (CLIP, BLIP)
    - Grounding language to actions
    - Multi-modal learning for robotics
    - Task decomposition from natural language
  - 3.5 Cognitive Planning and Execution
    - Classical vs learning-based planning
    - Hierarchical task decomposition
    - Real-time constraint handling
    - Feedback and error recovery
  - 3.6 End-to-End VLA Pipelines
    - System architecture design
    - Latency budgets and optimization
    - Sim-to-real transfer
  - 3.7 Summary & Key Takeaways

  Use textbook-author agent to generate content.

  After writing, use qa-validation-reviewer agent to check:
  - Technical correctness of kinematics/dynamics
  - LLM and vision-language model descriptions accurate
  - Code examples for bipedal control are valid
  - Markdown formatting
  - Learning outcomes met

  ---



**Prompt 2: Create Glossary**
```
Create comprehensive glossary for Physical AI & Humanoid Robotics textbook:

1. Extract technical terms from all 3 chapters

2. Identify 60-80 terms needing definitions:

   Chapter 1 terms (~20-25):
   - Physical AI, embodied intelligence, ROS 2, node, topic, publisher/subscriber
   - Service, action, URDF, link, joint, DOF, end-effector
   - LIDAR, IMU, sensor fusion, rclpy, colcon, launch file

   Chapter 2 terms (~20-25):
   - Gazebo, SDF, physics engine, point cloud, RGB-D camera
   - SLAM, VSLAM, NVIDIA Isaac Sim, Omniverse USD, synthetic data
   - Sim-to-real, Nav2, costmap, path planning, A*, behavior tree

   Chapter 3 terms (~20-25):
   - Forward kinematics, inverse kinematics, Jacobian, ZMP
   - Bipedal locomotion, gait, grasp planning, HRI
   - OpenAI Whisper, LLM, cognitive planning, action primitive
   - VLA (Vision-Language-Action), YOLO, object detection, semantic segmentation

3. For each term create entry:
   - Term name (bold)
   - Concise definition (50-150 words)
   - Category/domain (e.g., ROS 2, Simulation, Perception, Control)
   - Cross-references to related terms
   - Link to chapter where introduced

4. Organize alphabetically with category tags

5. Link terms in chapters to glossary using markdown links

Reference: Project_flow/Minimal_Chapter_Structure.md for complete term list

Use glossary-manager agent to:
- Detect all technical terms
- Create precise definitions
- Establish cross-references
- Ensure terminology consistency

Use Glossary_Expansion_Skill for definitions.
```

#### **📦 Deliverables**
- ✅ Chapter 1: Introduction to Physical AI & ROS 2 (3000+ words)
- ✅ Chapter 2: Robot Simulation & AI Perception (3000+ words)
- ✅ Chapter 3: Vision-Language-Action for Robotics (3000+ words)
- ✅ All chapters include:
  - Learning outcomes
  - Code examples (tested)
  - LaTeX equations (validated)
  - Worked examples
  - Summary sections
- ✅ QA review completed for each
- ✅ `glossary.md` with 60-80 term definitions
- ✅ Alphabetically organized entries
- ✅ Cross-references between related terms
- ✅ Category tags (ROS 2, Simulation, Perception, Control, etc.)
- ✅ Inline links from chapters to glossary
- ✅ Glossary added to sidebar

#### **📁 Folder Structure**
```
frontend/docs/
├── chapters/
│   ├── chapter-01-intro-physical-ai-ros2.md      (3000 words, ROS 2 code)
│   ├── chapter-02-simulation-ai-perception.md    (3000 words, Gazebo/Isaac)
│   └── chapter-03-vision-language-action.md      (3000 words, VLA pipeline)
└── glossary.md              # 60-80 terms with cross-references
```

---

### **Phase 1.5: Deployment & Testing**

#### **📋 Description**
Build production bundle, configure GitHub Actions, deploy to GitHub Pages, and test live site.

#### **🛠️ Tech Stack**
- **Build Tool**: Docusaurus build system
- **CI/CD**: GitHub Actions
- **Hosting**: GitHub Pages

#### **🤖 Agents & Skills**
- No agents needed (deployment task)
- Manual testing and configuration

#### **💬 Prompts to Use**

**Prompt 1: Deploy to GitHub Pages**
```
Deploy Docusaurus book to GitHub Pages:

1. Update docusaurus.config.js:
   - Set url, baseUrl, organizationName, projectName
   - Set deploymentBranch: 'gh-pages'

2. Create GitHub Actions workflow (.github/workflows/deploy.yml):
   - Trigger on push to main
   - Install dependencies, build, deploy

3. Build locally first: npm run build (fix errors)

4. Test build: npm run serve

5. Push to GitHub

6. Enable GitHub Pages in repository settings:
   - Source: Deploy from branch
   - Branch: gh-pages

7. Test live site:
   - Verify all navigation
   - Check glossary links
   - Test mobile responsiveness
   - Verify code blocks and LaTeX render
```

#### **📦 Deliverables**
- ✅ Production build successful
- ✅ GitHub Actions workflow configured
- ✅ Book deployed to GitHub Pages
- ✅ Live URL accessible
- ✅ All navigation working
- ✅ Mobile responsive
- ✅ Code blocks and LaTeX rendering correctly

#### **📁 Folder Structure**
```
.github/
└── workflows/
    └── deploy.yml

frontend/
├── build/                   # Production build (generated)
└── .gitignore
```

---

## **PROJECT 2: CHATBOT UI + RAG + URDU TRANSLATION**

**Project Folder:** `/backend`

---

### **Phase 2.1: Database & Vector Store Setup**

#### **📋 Description**
Design database schema, set up Qdrant vector database, generate embeddings from chapters using Gemini, and prepare for RAG.

#### **🛠️ Tech Stack**
- **Database**: PostgreSQL (Neon - free tier)
- **Vector DB**: Qdrant (Docker or Cloud)
- **Embedding Model**: Google Gemini `embedding-001` (768 dimensions, FREE)
- **ORM**: SQLAlchemy with asyncpg
- **Migration**: Alembic

#### **🤖 Agents & Skills**
- **Agent**: `database-schema` ⭐ PRIMARY - Design database schema
- **Skill**: `Database_Migration_Skill` - Create migration scripts
- **Skill**: `Vector_Embedding_Skill` ⭐ PRIMARY - Generate embeddings and Qdrant setup
- **Agent**: `code-reviewer` - Review database models

#### **💬 Prompts to Use**

**Prompt 1: Database Schema Design**
```
Create database schema for chatbot:

Tables needed:
- chapters (id, title, slug, chapter_number, content, summary, timestamps)
- chatbot_conversations (id, session_id, created_at)
- chatbot_messages (id, conversation_id, role, content, selected_text, sources, created_at)
- translations (id, chapter_id, language_code, translated_content, created_at)

Include:
- Indexes for performance
- Foreign key constraints
- SQLAlchemy ORM models
- Migration scripts

Use database-schema agent.
Use Database_Migration_Skill for migrations.
```

**Prompt 2: Qdrant Setup & Embeddings**
```
Set up vector embeddings with Qdrant:

1. Setup Qdrant collection:
   - Name: "chapter_embeddings"
   - Vector size: 768 (Gemini embedding-001)
   - Distance: COSINE
   - Payload indexes: chapter_id, section_number

2. Text chunking script:
   - Read markdown files from frontend/docs/chapters/
   - Split by ## headers (sections)
   - Extract metadata: chapter_id, title, section_number, content

3. Generate embeddings using Gemini:
   - Model: "models/embedding-001" (FREE)
   - Batch processing with rate limiting
   - Save embeddings with metadata

4. Insert into Qdrant with payload

5. Seed chapter metadata to Postgres

6. Create end-to-end pipeline script

Use Vector_Embedding_Skill.
Review with code-reviewer agent.
```

#### **📦 Deliverables**
- ✅ PostgreSQL schema created (4 tables)
- ✅ SQLAlchemy models defined
- ✅ Migration scripts created
- ✅ Qdrant collection configured
- ✅ Text chunking script (by sections)
- ✅ Embedding generation script (Gemini)
- ✅ All chapters embedded in Qdrant
- ✅ Chapter metadata in Postgres
- ✅ Test query returns relevant chunks

#### **📁 Folder Structure**
```
backend/
├── app/
│   └── models/
│       └── database.py          # SQLAlchemy models
├── migrations/
│   ├── 001_initial_schema.sql
│   └── migrate.py
├── scripts/
│   ├── chunk_chapters.py
│   ├── generate_embeddings.py
│   ├── insert_qdrant.py
│   ├── seed_chapters.py
│   ├── process_all.py
│   └── setup_qdrant.py
├── data/
│   └── chunks.json
├── .env
└── requirements.txt
```

---

### **Phase 2.2: RAG Backend API**

#### **📋 Description**
Build FastAPI backend with RAG pipeline: embed questions, retrieve from Qdrant, generate answers with Gemini, return with source citations.

#### **🛠️ Tech Stack**
- **Framework**: FastAPI (async)
- **LLM**: Google Gemini Pro (FREE)
- **Database**: PostgreSQL (async with asyncpg)
- **Vector DB**: Qdrant Python client
- **Validation**: Pydantic v2

#### **🤖 Agents & Skills**
- **Agent**: `backend-development` ⭐ PRIMARY - Build FastAPI services
- **Skill**: `API_Design_Skill` ⭐ PRIMARY - Design REST endpoints
- **Agent**: `code-reviewer` - Code quality review
- **Agent**: `test-runner` - Test API endpoints

#### **💬 Prompts to Use**

**Prompt 1: Build RAG Backend**
```
Build FastAPI RAG chatbot backend with Gemini:

Project structure:
- app/main.py (FastAPI app)
- app/config.py (settings with pydantic-settings)
- app/api/v1/routes/chat.py (chat endpoints)
- app/models/schemas.py (Pydantic schemas)
- app/services/gemini_service.py (LLM + embeddings)
- app/services/qdrant_service.py (vector search)
- app/services/rag_service.py (RAG orchestration)
- app/services/db_service.py (database operations)

Requirements:
1. Pydantic schemas: ChatRequest, ChatResponse, Source
2. Gemini service: generate_embedding(), generate_answer()
3. Qdrant service: search_similar() with filters
4. RAG service: answer_question() pipeline
5. Chat endpoint: POST /api/v1/chat/ask
6. CORS middleware for frontend
7. Error handling and logging

Use backend-development agent and API_Design_Skill.
Review with code-reviewer agent.
Test with test-runner agent.
```

#### **📦 Deliverables**
- ✅ FastAPI app running on port 8000
- ✅ RAG pipeline implemented
- ✅ Gemini integration (embeddings + generation)
- ✅ Qdrant search with filtering
- ✅ Selected-text context boosting
- ✅ Chat history saved to Postgres
- ✅ API documentation at `/docs`
- ✅ Error handling and logging
- ✅ CORS configured
- ✅ Tests passing

#### **📁 Folder Structure**
```
backend/app/
├── main.py
├── config.py
├── api/v1/routes/
│   └── chat.py
├── models/
│   ├── database.py
│   └── schemas.py
└── services/
    ├── gemini_service.py
    ├── qdrant_service.py
    ├── rag_service.py
    └── db_service.py

backend/tests/
└── test_chat.py
```

---

### **Phase 2.3: React Chatbot Widget - Core UI**

#### **📋 Description**
Build floating chatbot widget with message bubbles, input field, source citations, selected-text capture, and collapsible interface.

#### **🛠️ Tech Stack**
- **Framework**: React (TypeScript)
- **Styling**: CSS Modules
- **State**: React hooks
- **HTTP**: Fetch API
- **Integration**: Docusaurus theme swizzling

#### **🤖 Agents & Skills**
- **Agent**: `frontend-integration` ⭐ PRIMARY - Build React components
- **Skill**: `Component_Generation_Skill` ⭐ PRIMARY - Generate chatbot widget
- **Agent**: `code-reviewer` - Code quality review

#### **💬 Prompts to Use**

**Prompt 1: Create Chatbot Widget**
```
Create React chatbot widget for Docusaurus:

Component: frontend/src/components/ChatbotWidget/ChatbotWidget.tsx

Requirements:
- Floating button (bottom-right, 60px diameter)
- Expandable chat window (400x600px)
- Message list (scrollable, user/assistant bubbles)
- Input field with send button
- Capture selected text from page (window.getSelection())
- Display sources below answers
- Loading indicator ("Thinking...")
- Error handling
- Auto-scroll to latest message

Styling: ChatbotWidget.module.css
- Purple gradient button
- White chat window with shadow
- Message bubbles (user: blue, assistant: gray)
- Smooth animations
- Mobile responsive

Integration: frontend/src/theme/Root.tsx
- Swizzle Docusaurus Root
- Render ChatbotWidget globally

TypeScript types: frontend/src/types/chat.ts

Use frontend-integration agent and Component_Generation_Skill.
Review with code-reviewer agent.
```

#### **📦 Deliverables**
- ✅ ChatbotWidget component created
- ✅ Floating button with toggle
- ✅ Chat UI with message bubbles
- ✅ Selected text capture
- ✅ Source citations displayed
- ✅ Loading state indicator
- ✅ Error handling
- ✅ Mobile responsive
- ✅ Integrated via Root.tsx
- ✅ TypeScript types defined

#### **📁 Folder Structure**
```
frontend/src/
├── components/
│   └── ChatbotWidget/
│       ├── ChatbotWidget.tsx
│       └── ChatbotWidget.module.css
├── theme/
│   └── Root.tsx
└── types/
    └── chat.ts
```

---

### **Phase 2.4: Selected-Text Question Feature**

#### **📋 Description**
Implement text selection detection, show "Ask about this" tooltip on selection, pre-fill chatbot with context, filter RAG to prioritize selected section.

#### **🛠️ Tech Stack**
- **Browser API**: `window.getSelection()`
- **React**: Event listeners and state
- **CSS**: Tooltip positioning

#### **🤖 Agents & Skills**
- **Agent**: `frontend-integration` - Implement selection detection
- **Agent**: `code-reviewer` - Review selection logic

#### **💬 Prompts to Use**

**Prompt 1: Implement Text Selection**
```
Implement selected-text question feature:

1. Create hook: frontend/src/hooks/useTextSelection.ts
   - Listen for mouseup/touchend events
   - Get selected text via window.getSelection()
   - Return { text, x, y } for tooltip positioning

2. Create tooltip: frontend/src/components/SelectionTooltip/SelectionTooltip.tsx
   - Position at selection coordinates
   - Show "💬 Ask about this" button
   - Dismiss button

3. Update ChatbotWidget:
   - Accept initialQuestion and initialSelectedText props
   - Auto-open when props provided

4. Integrate in Root.tsx:
   - Use useTextSelection hook
   - Show SelectionTooltip when text selected
   - Pass selection to ChatbotWidget

5. Backend enhancement:
   - Boost Qdrant search results containing selected text
   - Update qdrant_service.py

Use frontend-integration agent.
Review with code-reviewer agent.
```

#### **📦 Deliverables**
- ✅ Text selection hook
- ✅ Tooltip on text selection
- ✅ "Ask about this" button functional
- ✅ Selected text pre-fills chatbot
- ✅ Backend boosts relevant results
- ✅ Tooltip dismisses after use
- ✅ Works on mobile (touch events)
- ✅ Smooth animations

#### **📁 Folder Structure**
```
frontend/src/
├── hooks/
│   └── useTextSelection.ts
├── components/
│   ├── ChatbotWidget/
│   │   └── ChatbotWidget.tsx    (updated)
│   └── SelectionTooltip/
│       ├── SelectionTooltip.tsx
│       └── SelectionTooltip.module.css
└── theme/
    └── Root.tsx                 (updated)

backend/app/services/
└── qdrant_service.py            (updated)
```

---

### **Phase 2.5: Urdu Translation Integration**

#### **📋 Description**
Add "Translate to Urdu" button, implement translation with Gemini, cache in database, handle RTL text, preserve code blocks/equations.

#### **🛠️ Tech Stack**
- **Translation**: Google Gemini Pro (FREE)
- **Database**: PostgreSQL (caching)
- **RTL Support**: CSS `direction: rtl`
- **React**: Translation button component

#### **🤖 Agents & Skills**
- **Agent**: `urdu-academic-translator` ⭐ PRIMARY - High-quality Urdu translation
- **Skill**: `Translation_Skill` ⭐ PRIMARY - Translation logic
- **Agent**: `backend-development` - Translation API endpoint
- **Skill**: `API_Design_Skill` - API design

#### **💬 Prompts to Use**

**Prompt 1: Backend Translation Service**
```
Implement Urdu translation feature:

Backend: backend/app/services/translation_service.py
- translate_to_urdu() method
- Use Gemini Pro with specialized prompt:
  * Keep code blocks unchanged
  * Preserve LaTeX equations
  * Maintain markdown structure
  * Transliterate technical terms
- Check cache first (translations table)
- Cache results after translation

Endpoint: backend/app/api/v1/routes/translation.py
- POST /api/v1/translation/translate
- Accept chapter_id
- Return translated_content and cached flag

Use urdu-academic-translator agent and Translation_Skill.
```

**Prompt 2: Frontend Translation Button**
```
Create translation UI:

Component: frontend/src/components/TranslateButton/TranslateButton.tsx
- Button: "🌐 Translate to Urdu"
- Loading state: "⏳ Translating..."
- POST to translation endpoint
- Pass urduContent to parent

Swizzle DocItem: frontend/src/theme/DocItem/index.tsx
- Add TranslateButton
- Toggle between English/Urdu
- RTL support: <div dir="rtl">
- Preserve code blocks in LTR

Styling:
- Add Urdu font (Noto Nastaliq Urdu)
- RTL text direction
- Code blocks stay LTR in RTL context

Use frontend-integration agent.
Validate with urdu-academic-translator agent.
```

#### **📦 Deliverables**
- ✅ Translation service with Gemini
- ✅ Translation endpoint
- ✅ Translation caching
- ✅ "Translate to Urdu" button on chapters
- ✅ RTL text rendering
- ✅ Code blocks preserved
- ✅ LaTeX equations preserved
- ✅ Toggle English/Urdu
- ✅ Urdu font loaded
- ✅ Loading state indicator

#### **📁 Folder Structure**
```
backend/app/
├── api/v1/routes/
│   └── translation.py
└── services/
    └── translation_service.py

frontend/src/
├── components/
│   └── TranslateButton/
│       ├── TranslateButton.tsx
│       └── TranslateButton.module.css
├── theme/
│   └── DocItem/
│       ├── index.tsx            (swizzled)
│       └── styles.module.css
└── css/
    └── custom.css               (Urdu font)
```

---

### **Phase 2.6: Chatbot-Docusaurus Integration**

#### **📋 Description**
Final integration: ensure chatbot appears on all pages, passes chapter context, matches Docusaurus theme, works on deployed site.

#### **🛠️ Tech Stack**
- **Integration**: Docusaurus theme system
- **Context**: React hooks
- **Styling**: CSS variables for theme matching

#### **🤖 Agents & Skills**
- **Agent**: `frontend-integration` - Final integration
- **Agent**: `test-runner` - End-to-end testing
- **Agent**: `code-reviewer` - Final code review

#### **💬 Prompts to Use**

**Prompt 1: Chapter Context Detection**
```
Implement chapter context awareness:

Hook: frontend/src/hooks/useChapterContext.ts
- Extract chapter_id from URL
- Extract chapter_title from page h1
- Return { chapterId, chapterTitle, chapterSlug }

Update ChatbotWidget:
- Accept chapterContext prop
- Show chapter in header badge
- Pass to backend in chat request

Update Backend:
- Accept chapter_context in ChatRequest schema
- Filter Qdrant search by chapter_id if provided

Use frontend-integration agent.
```

**Prompt 2: Theme Matching & Testing**
```
Match Docusaurus theme and test:

1. Update CSS to use Docusaurus variables:
   - --ifm-color-primary
   - --ifm-background-color
   - --ifm-font-color-base
   - Support dark mode

2. Production API URL:
   - Create config/api.ts
   - Use environment variables
   - Switch based on NODE_ENV

3. End-to-end tests:
   - Chatbot appears on all pages
   - Selected text opens chatbot
   - Chapter context passed correctly
   - Sources displayed
   - Translation works
   - Mobile responsive

Use test-runner agent for E2E tests.
Use code-reviewer agent for final review.
```

#### **📦 Deliverables**
- ✅ Chatbot on all Docusaurus pages
- ✅ Chapter context auto-detected
- ✅ Chapter filtering in RAG
- ✅ Theme matches Docusaurus
- ✅ Production API URL configured
- ✅ End-to-end tests passing
- ✅ Mobile responsive
- ✅ Works on GitHub Pages

#### **📁 Folder Structure**
```
frontend/src/
├── hooks/
│   ├── useTextSelection.ts
│   └── useChapterContext.ts     # NEW
├── config/
│   └── api.ts                   # API URL config
├── components/
│   └── ChatbotWidget/
│       ├── ChatbotWidget.tsx    (updated)
│       └── ChatbotWidget.module.css (theme variables)
└── theme/
    └── Root.tsx                 (final integration)

frontend/tests/
└── e2e/
    └── chatbot.spec.ts
```

---

## **BONUS FEATURES**

---

### **Bonus Phase 1: Better-Auth Signup/Signin**

#### **📋 Description**
Implement user authentication with Better-Auth, collect user profiles (software/hardware background, experience level), manage JWT sessions.

#### **🛠️ Tech Stack**
- **Auth**: Better-Auth (https://www.better-auth.com/)
- **Frontend**: React auth components
- **Backend**: JWT token validation
- **Database**: User and session tables

#### **🤖 Agents & Skills**
- **Agent**: `authentication` ⭐ PRIMARY - Complete auth implementation
- **Agent**: `database-schema` - User/session tables
- **Skill**: `API_Design_Skill` - Auth endpoints

#### **💬 Prompts to Use**

**Prompt: Implement Authentication**
```
Implement Better-Auth authentication:

Database schema (use database-schema agent):
- users (id, email, password_hash, name, timestamps)
- user_profiles (user_id, software_background, hardware_background, experience_level)
- sessions (user_id, session_token, expires_at)

Backend (use authentication agent):
- Auth service: hash_password, verify_password, create_access_token, verify_token
- Auth endpoints: POST /auth/signup, POST /auth/signin
- Collect profile data: software background, hardware background, experience level
- JWT token management

Frontend:
- SignupForm component (email, password, name, profile fields)
- SigninForm component
- Auth context for React
- Store token in localStorage

Use authentication agent for complete implementation.
```

#### **📦 Deliverables**
- ✅ User/profile/session tables
- ✅ Auth endpoints (`/signup`, `/signin`)
- ✅ JWT token generation/validation
- ✅ SignupForm and SigninForm components
- ✅ User profile collection
- ✅ Protected routes (optional)
- ✅ Auth context

#### **📁 Folder Structure**
```
backend/app/
├── api/v1/routes/
│   └── auth.py
└── services/
    └── auth_service.py

frontend/src/
├── components/
│   └── Auth/
│       ├── SignupForm.tsx
│       ├── SigninForm.tsx
│       └── Auth.module.css
└── context/
    └── AuthContext.tsx
```

---

### **Bonus Phase 2: Content Personalization**

#### **📋 Description**
Personalize chapter content based on user profile (software/hardware background, experience level) using Gemini to adapt explanations and code examples.

#### **🛠️ Tech Stack**
- **LLM**: Google Gemini Pro
- **Database**: Personalized content caching
- **React**: Personalize button component

#### **🤖 Agents & Skills**
- **Agent**: `content-personalizer` ⭐ PRIMARY - Content adaptation
- **Skill**: `Personalization_Rewrite_Skill` ⭐ PRIMARY - Rewriting logic
- **Agent**: `backend-development` - Personalization API

#### **💬 Prompts to Use**

**Prompt: Implement Personalization**
```
Implement content personalization:

Backend (use content-personalizer agent):
- Personalization service: personalize_chapter()
- Use Gemini Pro with user profile:
  * Adjust complexity based on experience_level
  * Adapt code examples to software_background
  * Reference hardware_background platforms
  * Keep core concepts intact
- Cache personalized content per user
- Endpoint: POST /personalization/personalize

Frontend:
- PersonalizeButton component (only for logged-in users)
- Button: "✨ Personalize for Me"
- Loading state
- Update DocItem to show personalized view
- Toggle: Original / Urdu / Personalized

Requirements:
- Verify JWT token
- Load user profile
- Check cache first
- Generate personalized version
- Cache result

Use content-personalizer agent and Personalization_Rewrite_Skill.
```

#### **📦 Deliverables**
- ✅ Personalization service with Gemini
- ✅ Personalization endpoint
- ✅ User profile-based adaptation
- ✅ Personalize button (logged-in only)
- ✅ Content caching per user
- ✅ View mode toggle
- ✅ Complexity adjustment
- ✅ Code examples adapted to tech stack

#### **📁 Folder Structure**
```
backend/app/
├── api/v1/routes/
│   └── personalization.py
└── services/
    └── personalization_service.py

frontend/src/
├── components/
│   └── PersonalizeButton/
│       ├── PersonalizeButton.tsx
│       └── PersonalizeButton.module.css
└── theme/
    └── DocItem/
        └── index.tsx             (updated)
```

---

## **FINAL FOLDER STRUCTURE**

```
Robotics_book_chatbot/
├── frontend/                     # PROJECT 1: Docusaurus Book
│   ├── docs/
│   │   ├── chapters/             (3 chapter markdown files)
│   │   ├── glossary.md
│   │   └── intro.md
│   ├── src/
│   │   ├── components/           (ChatbotWidget, TranslateButton, etc.)
│   │   ├── hooks/                (useTextSelection, useChapterContext)
│   │   ├── theme/                (Root, DocItem - swizzled)
│   │   ├── types/                (TypeScript interfaces)
│   │   ├── config/               (API URL config)
│   │   └── css/
│   ├── static/
│   │   └── img/
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   └── package.json
│
├── backend/                      # PROJECT 2: RAG + Chatbot + Translation
│   ├── app/
│   │   ├── main.py
│   │   ├── config.py
│   │   ├── api/v1/routes/        (chat, translation, auth, personalization)
│   │   ├── models/               (database, schemas)
│   │   └── services/             (gemini, qdrant, rag, db, translation, auth, personalization)
│   ├── migrations/
│   ├── scripts/                  (chunking, embeddings, seeding)
│   ├── tests/
│   ├── .env
│   └── requirements.txt
│
├── .github/
│   └── workflows/
│       └── deploy.yml
│
├── Project_flow/
│   └── Detailed_Phase_Breakdown.md
│
└── README.md
```

---

## **EXECUTION CHECKLIST**

### **Day 1: Foundation (4-5 hours)**
- [ ] Phase 1.1: Docusaurus setup
- [ ] Phase 1.2: Chapter outlines
- [ ] Phase 2.1: Database + Qdrant setup

### **Day 2: Content (4-5 hours)**
- [ ] Phase 1.3: Write 3 chapters
- [ ] Phase 1.4: Create glossary

### **Day 3: Backend (4-5 hours)**
- [ ] Phase 2.2: RAG backend API
- [ ] Phase 2.1 (cont): Generate embeddings

### **Day 4: Frontend (4-5 hours)**
- [ ] Phase 2.3: Chatbot widget UI
- [ ] Phase 2.4: Selected-text feature
- [ ] Phase 2.5: Urdu translation

### **Day 5: Integration & Deploy (3-4 hours)**
- [ ] Phase 2.6: Final integration
- [ ] Phase 1.5: Deploy to GitHub Pages
- [ ] Testing and polish

### **Bonus (if time permits)**
- [ ] Bonus Phase 1: Authentication
- [ ] Bonus Phase 2: Personalization

---

## **QUICK REFERENCE: AGENTS & SKILLS BY PHASE**

| Phase | Primary Agent/Skill | Supporting Tools |
|-------|-------------------|------------------|
| 1.1 | Environment_Configuration_Skill | - |
| 1.2 | Chapter_Outline_Skill | textbook-author agent |
| 1.3 | textbook-author agent | Example_Generator_Skill, Code_Explanation_Skill, Markdown_formatting_Skill, qa-validation-reviewer |
| 1.4 | glossary-manager agent | Glossary_Expansion_Skill, Context_Extraction_Skill |
| 1.5 | Manual | - |
| 2.1 | database-schema agent, Vector_Embedding_Skill | Database_Migration_Skill, code-reviewer |
| 2.2 | backend-development agent, API_Design_Skill | code-reviewer, test-runner |
| 2.3 | frontend-integration agent, Component_Generation_Skill | code-reviewer |
| 2.4 | frontend-integration agent | code-reviewer |
| 2.5 | urdu-academic-translator agent, Translation_Skill | backend-development, API_Design_Skill |
| 2.6 | frontend-integration agent | test-runner, code-reviewer |
| Bonus 1 | authentication agent | database-schema, API_Design_Skill |
| Bonus 2 | content-personalizer agent, Personalization_Rewrite_Skill | backend-development |

---

**Total Estimated Time:**
- **Core MVP**: 20-25 hours
- **Bonus Features**: +6-8 hours

---

**Ready to start building! 🚀**
