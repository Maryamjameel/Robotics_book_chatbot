
---

# **Project Breakdown in Phases**

---

## **Phase 1: Setup & Basic Book Structure**

**Goal:** Get your book skeleton ready in Docusaurus + Spec-Kit Plus.

**Steps:**

1. **Create Book Project:**

   ```bash
   claude create book "Physical AI & Humanoid Robotics"
   ```
2. **Define Chapters & Outline using Spec-Kit Plus:**

   * Use **Chapter Outline Skill** to generate structured outlines.
   * Chapters examples:

     * Introduction to Physical AI
     * Robotics Fundamentals
     * Sensors & Actuators
     * Humanoid Motion Control
     * AI Agents in Robotics
     * Exercises & Case Studies
3. **Use Book Authoring Agent** to write **chapter drafts** in Markdown.
4. **Preview Docusaurus website locally** to confirm structure.

**Agents Used:**

* Book Authoring Agent

**Skills Used:**

* Chapter Outline Skill
* Markdown Formatting Skill

**Deliverable:**

* Full chapter skeleton in Markdown ready for content expansion.

---

## **Phase 2: Generate Chapter Content & Examples**

**Goal:** Fill each chapter with high-quality content.

**Steps:**

1. Use **Book Authoring Agent** + **Outline Expand Skill** for each chapter.
2. Use **Example Generator Skill** to create real-world examples.
3. Use **Code Explanation Skill** for robotics algorithms/code snippets.
4. Format all content using **Markdown Formatting Skill**.
5. Save final Markdown in your Docusaurus structure.

**Agents Used:**

* Book Authoring Agent
* Code Explainer Agent
* Glossary Agent

**Skills Used:**

* Example Generator Skill
* Code Explanation Skill
* Markdown Formatting Skill

**Deliverable:**

* Fully written chapters with examples and code explanations.

---

## **Phase 3: Create Glossary & Summaries**

**Goal:** Make the textbook navigable and easy to learn.

**Steps:**

1. Use **Glossary Expansion Skill** to extract technical terms from chapters.
2. Populate **Glossary Agent**.
3. Use **Summary Builder Skill** to generate chapter summaries.
4. Add summaries at the start/end of each chapter in Markdown.

**Agents Used:**

* Glossary Agent

**Skills Used:**

* Glossary Expansion Skill
* Summary Builder Skill

**Deliverable:**

* Glossary file & chapter summaries in place.

---

## **Phase 4: Exercises & Quizzes**

**Goal:** Add learning exercises and quizzes at the end of each chapter.

**Steps:**

1. Use **Exercise Generator Agent** to create exercises.
2. Use **Quiz Builder Skill** to generate MCQs and coding tasks.
3. Save exercises & quizzes as part of chapter Markdown.

**Agents Used:**

* Exercise Generator Agent

**Skills Used:**

* Generate Quiz Skill
* Generate Coding Task Skill

**Deliverable:**

* Chapters now have exercises + quizzes.

---

## **Phase 5: Translation (Urdu) Feature**

**Goal:** Allow chapters to be translated into Urdu.

**Steps:**

1. Use **Translation Agent** to generate Urdu versions.
2. Use **Translation Skill (Urdu)** for paragraph-level translation.
3. Add a toggle button in Docusaurus for **English ↔ Urdu** chapters.

**Agents Used:**

* Translation Agent

**Skills Used:**

* Translation Skill (Urdu)

**Deliverable:**

* Each chapter now has bilingual support.

---

## **Phase 6: Personalization Feature**

**Goal:** Adapt chapters based on user background.

**Steps:**

1. Collect user info at signup using **Better-Auth.com**.

   * Software/Hardware experience
   * Beginner/Intermediate/Advanced
2. Use **Personalization Agent** to rewrite chapters according to user profile.
3. Use **Personalization Rewrite Skill** to adjust tone and complexity.
4. Add a **“Personalize Chapter” button** in Docusaurus for logged-in users.

**Agents Used:**

* Personalization Agent

**Skills Used:**

* Personalization Rewrite Skill

**Deliverable:**

* Chapters dynamically personalized per user.

---

## **Phase 7: RAG Chatbot Integration**

**Goal:** Build AI chatbot that answers questions based on textbook content.

**Steps:**

1. Use **RAG Answer Agent** to answer questions.
2. Use **Context Extraction Skill** to get relevant text chunks.
3. Use **Retrieval Query Skill** to fetch from Qdrant vectors.
4. Use **Selected Text Answer Skill** for “Answer from selected text only.”
5. Deploy chatbot in FastAPI backend, integrate into book site.

**Agents Used:**

* RAG Answer Agent

**Skills Used:**

* Context Extraction Skill
* Retrieval Query Skill
* Selected Text Answer Skill

**Deliverable:**

* Fully working textbook AI chatbot.

---

## **Phase 8: Content Quality & Review**

**Goal:** Ensure the textbook is polished, error-free, and consistent.

**Steps:**

1. Run **Content Quality Agent** on all chapters.
2. Use **Consistency & Quality Check Skill** to detect formatting, hallucination, or logic errors.
3. Correct using Book Authoring Agent + Markdown Formatting Skill.

**Agents Used:**

* Content Quality Agent

**Skills Used:**

* Consistency & Quality Check Skill
* Markdown Formatting Skill

**Deliverable:**

* Final clean, consistent, publication-ready textbook.

---

## **Phase 9: Deploy to GitHub Pages**

**Goal:** Make the book publicly accessible.

**Steps:**

1. Use Docusaurus build command to generate static site.
2. Push to GitHub Pages.
3. Verify all features:

   * English/Urdu toggle
   * Personalization button
   * RAG chatbot

**Deliverable:**

* Fully deployed AI-native textbook site.

---

# **Summary Table — Phase vs Agents/Skills**

| Phase                   | Main Agents                              | Main Skills                                               |
| ----------------------- | ---------------------------------------- | --------------------------------------------------------- |
| 1. Book Skeleton        | Book Authoring Agent                     | Chapter Outline, Markdown Formatting                      |
| 2. Content & Examples   | Book Authoring, Code Explainer, Glossary | Example Generator, Code Explanation, Markdown Formatting  |
| 3. Glossary & Summaries | Glossary Agent                           | Glossary Expansion, Summary Builder                       |
| 4. Exercises & Quizzes  | Exercise Generator Agent                 | Generate Quiz, Generate Coding Task                       |
| 5. Translation          | Translation Agent                        | Translation Skill (Urdu)                                  |
| 6. Personalization      | Personalization Agent                    | Personalization Rewrite                                   |
| 7. RAG Chatbot          | RAG Answer Agent                         | Context Extraction, Retrieval Query, Selected Text Answer |
| 8. Quality Review       | Content Quality Agent                    | Consistency & Quality Check, Markdown Formatting          |
| 9. Deployment           | —                                        | —                                                         |

---

This phased approach will let you **build, test, and expand** the project gradually using **Claude CLI + Spec-Kit Plus**, without breaking the system.

---

