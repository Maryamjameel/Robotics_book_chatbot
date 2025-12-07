# 🚀 ROBOTICS TEXTBOOK CHATBOT - COMPLETE MANUAL TESTING GUIDE

## **QUICK START (Copy-Paste Commands)**

### **Step 1: Open Terminal 1 - Start Frontend**

```bash
cd "C:\Users\Musa Computer\Desktop\Robotics_book_chatbot\frontend"
npm start
```

⏳ Wait for: `Docusaurus website is running at: http://localhost:3000`

---

### **Step 2: Open Terminal 2 - Start Backend**

```bash
cd "C:\Users\Musa Computer\Desktop\Robotics_book_chatbot\backend"
python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

⏳ Wait for: `Uvicorn running on http://0.0.0.0:8000`

---

## **IMMEDIATE TESTING (No Terminal Needed)**

### **Test 1: Frontend - Open Browser**
```
http://localhost:3000
```

**Expected Results:**
- ✅ Page loads with title "Humanoid Robotics"
- ✅ Dark theme applied
- ✅ Sidebar with navigation visible
- ✅ No console errors (F12)

---

### **Test 2: Backend Health Check**

**Open new terminal and run:**
```bash
curl http://localhost:8000/health
```

**Expected Response:**
```json
{"status": "ok"}
```

---

## **COMPLETE PROJECT TEST WALKTHROUGH**

### **PROJECT 1: INTERACTIVE TEXTBOOK** ✅

**1. Homepage Test**
- Open: http://localhost:3000
- ✅ See title "Humanoid Robotics"
- ✅ Dark/Light toggle visible (top right)
- ✅ Sidebar navigation on left

**2. Navigate to Chapter 2**
- Click "Docs" in sidebar
- Click "Chapter 02"
- ✅ Page loads with content
- ✅ Sections 2.1, 2.2, 2.3, 2.4, 2.5 visible

**3. Check Content Quality**
- ✅ Math symbols render: α, β, γ, τ, θ, φ
- ✅ Code examples display with colors
- ✅ Practice problems at bottom
- ✅ Further reading section

**4. Test Responsiveness**
- Press F12 (DevTools)
- Click device icon (top-left)
- Set width to 375px (mobile)
- ✅ Content stacks vertically
- ✅ Sidebar becomes hamburger menu
- ✅ Text still readable

**5. Test Theme Toggle**
- Click dark/light icon
- ✅ Page changes to light theme
- Click again
- ✅ Page changes back to dark theme

**6. Accessibility Test**
- Press Tab repeatedly
- ✅ Focus ring visible on elements
- ✅ Can navigate using keyboard only

---

### **PROJECT 2: RAG CHATBOT** ⚠️

**Note:** The Gemini package issue requires a workaround. See "Workaround" section below.

**1. Backend Health Check**

Terminal 3:
```bash
curl http://localhost:8000/health
```

Expected:
```json
{"status":"ok"}
```

**2. Full Dependency Check**

```bash
curl http://localhost:8000/health/full
```

Expected (even with Gemini issue):
```json
{
  "status": "degraded",
  "components": {
    "qdrant": {"status": "healthy"},
    "gemini": {"status": "error"}
  }
}
```

**3. Test Chat Endpoint**

If Gemini is working:
```bash
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{"question":"What is kinematics?"}'
```

Expected Response:
```json
{
  "id": "uuid-here",
  "answer": "Kinematics is the study of...",
  "confidence": 0.95,
  "sources": [
    {
      "chapter": "Chapter 02",
      "section": "2.1",
      "content": "...",
      "relevance": 0.98
    }
  ],
  "metadata": {
    "processing_time_ms": 1234,
    "query_vector_dim": 1536,
    "results_retrieved": 3
  }
}
```

**4. Test Error Handling**

Send empty question:
```bash
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{"question":""}'
```

Expected: 400 error with "Question cannot be empty"

**5. Test with Chapter Context**

```bash
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question":"Explain the key concepts",
    "chapter_context":"ch02"
  }'
```

**6. Test Selected Text Boosting**

```bash
curl -X POST http://localhost:8000/api/v1/chat/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question":"What is this?",
    "selected_text":"Visual SLAM uses camera images to simultaneously build a map and localize the robot"
  }'
```

---
