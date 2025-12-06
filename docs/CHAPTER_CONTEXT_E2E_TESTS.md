# Chapter Context Awareness - Manual E2E Test Scenarios

## Test Suite Overview

This document defines manual end-to-end test scenarios for the Chapter Context Awareness feature. These tests verify the complete flow from chapter detection through result ranking.

---

## Test Setup

### Prerequisites
- [ ] Robotics textbook deployed with chapters in /docs/chapter-* paths
- [ ] Docusaurus site running on local dev server
- [ ] Backend RAG API running and accessible
- [ ] Qdrant vector database populated with chapter embeddings
- [ ] Browser DevTools open for monitoring network requests

### Test Data
- Chapter 3: Kinematics (ch03)
- Chapter 4: Dynamics (ch04)
- Chapter 5: Control Systems (ch05)

---

## Scenario 1: Chapter Detection from URL

### Test Case 1.1: Detect Chapter from Standard URL Pattern

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Observe ChatKit widget renders
3. Look for chapter badge in widget

**Expected Results:**
- [ ] Chapter badge displays: "Chapter: Kinematics ✓ (high)"
- [ ] Badge color is green (high confidence)
- [ ] Network request includes `chapter_context: { chapter_id: "ch03", chapter_title: "Kinematics" }`

**Pass Criteria:** All three assertions pass

---

### Test Case 1.2: Detect Chapter from Alternate URL Format

**Steps:**
1. Navigate to `/docs/chapter_4_dynamics`
2. Observe ChatKit widget
3. Check chapter badge

**Expected Results:**
- [ ] Chapter badge displays: "Chapter: Dynamics (medium)"
- [ ] Badge color is amber (medium confidence)
- [ ] Chapter ID extracted as "chapter_4_dynamics" or "chapter_4"

**Pass Criteria:** Badge displayed with correct confidence level

---

### Test Case 1.3: Fallback to DOM when URL Missing

**Steps:**
1. Navigate to `/docs/some-section/` (no chapter pattern in URL)
2. Page displays h1 heading: "Advanced Kinematics"
3. Wait 1 second for hook to execute
4. Check chapter badge

**Expected Results:**
- [ ] Chapter badge displays: "Chapter: Advanced Kinematics (medium)"
- [ ] Badge color is amber (medium confidence)
- [ ] Chapter detected from DOM heading

**Pass Criteria:** Badge displayed from h1 extraction

---

## Scenario 2: Search Result Filtering

### Test Case 2.1: Results Filtered by Current Chapter

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Type question: "What is forward kinematics?"
3. Submit question
4. Wait for response
5. Check response metadata in browser console

**Expected Results:**
- [ ] Chat displays answer about kinematics
- [ ] Response includes 2-3 sources from Chapter 3
- [ ] Console shows `metadata.chapter_filtered: true`
- [ ] Console shows `metadata.chapter_id: "ch03"`
- [ ] Sources listed include Chapter 3 sections first

**Pass Criteria:** Results prioritize chapter 3, metadata confirms filtering

---

### Test Case 2.2: Cross-Chapter Search with Chapter Filtering

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Ask question: "What are the differences between kinematics and dynamics?"
3. Submit question
4. Review sources

**Expected Results:**
- [ ] Answer discusses both topics
- [ ] First source is from Chapter 3 (Kinematics)
- [ ] Second source is from Chapter 4 (Dynamics) with lower relevance
- [ ] Metadata shows `filtered_count > 0`

**Pass Criteria:** Ch3 results ranked higher than Ch4

---

### Test Case 2.3: No Chapter Context (No Filtering)

**Steps:**
1. Use browser DevTools to mock `useChapterContext` returning null
2. Ask question: "What is kinematics?"
3. Check metadata

**Expected Results:**
- [ ] Chat works normally
- [ ] Metadata shows `chapter_filtered: false`
- [ ] Sources not prioritized by chapter
- [ ] No chapter badge visible

**Pass Criteria:** Search works without chapter context

---

## Scenario 3: Selected Text Boosting

### Test Case 3.1: TF-IDF Boosting with Chapter Context

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Select text: "Forward kinematics matrix transformation"
3. Click "Ask about this" (selection tooltip)
4. Observe ChatKit input pre-filled
5. Submit question
6. Check metadata

**Expected Results:**
- [ ] Input field contains selected text
- [ ] Response shows high confidence
- [ ] Metadata shows `selected_text_boosted: true`
- [ ] Extracted terms: ["forward", "kinematics", "matrix", "transformation"]
- [ ] Sources include content matching selected terms first

**Pass Criteria:** Selected text boosting with chapter filtering both active

---

### Test Case 3.2: Selected Text Boosting Without Chapter Context

**Steps:**
1. Mock useChapterContext to return null
2. Select text: "Denavit-Hartenberg"
3. Click "Ask about this"
4. Submit question
5. Check metadata

**Expected Results:**
- [ ] Response shows boosted relevance
- [ ] Metadata shows `selected_text_boosted: true`
- [ ] No chapter filtering applied
- [ ] Results include DH-related content

**Pass Criteria:** TF-IDF boosting works independently

---

## Scenario 4: Badge Confidence Indicators

### Test Case 4.1: High Confidence Badge

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Page displays h1: "Kinematics"
3. Wait for hook execution
4. Check badge

**Expected Results:**
- [ ] Badge shows checkmark icon (✓)
- [ ] Text: "Chapter: Kinematics (high)"
- [ ] Badge color: green (#10b981)

**Pass Criteria:** Green badge with checkmark

---

### Test Case 4.2: Medium Confidence Badge

**Steps:**
1. Navigate to `/docs/docs/chapter-5-control/some-page`
2. Page displays h1: "Something Different"
3. Check badge

**Expected Results:**
- [ ] Badge shows half-circle icon (◐)
- [ ] Text: "Chapter: Something Different (medium)"
- [ ] Badge color: amber (#f59e0b)

**Pass Criteria:** Amber badge with half-circle

---

### Test Case 4.3: Low Confidence Fallback

**Steps:**
1. Navigate to page without chapter pattern or h1
2. Check for badge

**Expected Results:**
- [ ] No badge displayed (returns null)
- [ ] OR badge shown with "?" icon and "Unknown Chapter (low)"
- [ ] Badge color: red (#ef4444)

**Pass Criteria:** No badge OR low confidence badge displayed

---

## Scenario 5: Edge Cases

### Test Case 5.1: Special Characters in Chapter Title

**Steps:**
1. Navigate to page with h1: "Chapter 3.5: Dynamics & Control (v2)"
2. Ask question about this topic
3. Check slug generation

**Expected Results:**
- [ ] Slug generated: "chapter-35-dynamics-control-v2"
- [ ] Chapter filtering works with special chars stripped
- [ ] Results match the chapter correctly

**Pass Criteria:** Special chars handled gracefully

---

### Test Case 5.2: Very Long Chapter Title

**Steps:**
1. Create page with h1: "A Very Long Chapter Title About Advanced Robotics Kinematics for Humanoid Robots with Multiple Joint Types and Constraints"
2. Ask question
3. Check badge rendering

**Expected Results:**
- [ ] Badge wraps text appropriately on mobile
- [ ] No text overflow or layout issues
- [ ] All text visible and readable

**Pass Criteria:** Badge renders correctly on mobile

---

### Test Case 5.3: URL without Chapter Pattern

**Steps:**
1. Navigate to `/docs/introduction/` (no chapter-* pattern)
2. Page has no h1
3. Ask question

**Expected Results:**
- [ ] No chapter badge
- [ ] Search works normally
- [ ] No filtering applied
- [ ] Response includes results from all chapters

**Pass Criteria:** Graceful fallback to non-chapter mode

---

## Scenario 6: API Payload Validation

### Test Case 6.1: Inspect Network Request Payload

**Steps:**
1. Open DevTools Network tab
2. Navigate to `/docs/chapter-3-kinematics`
3. Ask question: "What is forward kinematics?"
4. Find POST request to `/api/v1/chat/ask`
5. Inspect request body

**Expected Results:**
- [ ] Request payload includes:
  ```json
  {
    "question": "What is forward kinematics?",
    "chapter_context": {
      "chapter_id": "ch03",
      "chapter_title": "Kinematics"
    }
  }
  ```
- [ ] No extra fields in chapter_context
- [ ] Question field present and valid

**Pass Criteria:** Payload matches schema

---

### Test Case 6.2: Inspect Response Metadata

**Steps:**
1. Same as 6.1
2. Inspect response body

**Expected Results:**
- [ ] Response includes metadata:
  ```json
  {
    "metadata": {
      "chapter_filtered": true,
      "chapter_id": "ch03",
      "boost_applied": true,
      "selected_text_boosted": false
    }
  }
  ```
- [ ] All metadata fields present

**Pass Criteria:** Response metadata complete

---

## Scenario 7: Performance Validation

### Test Case 7.1: Chapter Detection Latency

**Steps:**
1. Open DevTools Performance tab
2. Navigate to chapter page
3. Record performance
4. Check hook execution time

**Expected Results:**
- [ ] Chapter detection completes in < 10ms
- [ ] No layout shift when badge renders
- [ ] No jank in animations

**Pass Criteria:** Latency < 10ms, smooth rendering

---

### Test Case 7.2: Search with Filtering Latency

**Steps:**
1. Ask question on chapter page
2. Monitor backend response time
3. Check total latency in response

**Expected Results:**
- [ ] Backend response time: < 2000ms
- [ ] Chapter filtering adds < 50ms overhead
- [ ] Total latency: < 2000ms

**Pass Criteria:** Performance within budget

---

## Scenario 8: Regression Tests

### Test Case 8.1: Existing Functionality Still Works

**Steps:**
1. Test basic chat without chapter context
2. Test selected text feature alone
3. Test page context feature alone

**Expected Results:**
- [ ] All features work independently
- [ ] No regression in existing functionality
- [ ] Chat responds correctly in all modes

**Pass Criteria:** No regressions detected

---

### Test Case 8.2: Multiple Simultaneous Questions

**Steps:**
1. Navigate to `/docs/chapter-3-kinematics`
2. Ask 3 questions in rapid succession
3. Wait for all responses

**Expected Results:**
- [ ] All questions answered correctly
- [ ] All responses include chapter filtering
- [ ] No race conditions or data corruption

**Pass Criteria:** All requests handled correctly

---

## Scenario 9: Mobile Responsiveness

### Test Case 9.1: Chapter Badge on Mobile

**Steps:**
1. Open DevTools with mobile view (320px width)
2. Navigate to chapter page
3. Inspect chapter badge

**Expected Results:**
- [ ] Badge fits within mobile width
- [ ] Text wraps appropriately
- [ ] Icons visible and clickable
- [ ] No horizontal scroll

**Pass Criteria:** Badge responsive on 320px width

---

### Test Case 9.2: Chat on Mobile with Chapter Context

**Steps:**
1. Mobile view (375px - iPhone SE)
2. Ask question on chapter page
3. Wait for response

**Expected Results:**
- [ ] Chat interface fully functional
- [ ] Chapter context applied correctly
- [ ] Response renders properly
- [ ] Touch targets are 48px minimum

**Pass Criteria:** Full functionality on mobile

---

## Test Execution Checklist

- [ ] All scenarios executed
- [ ] No critical issues found
- [ ] No regressions detected
- [ ] Performance within budget
- [ ] Mobile responsiveness verified
- [ ] API payloads validated
- [ ] Edge cases handled
- [ ] Documentation updated

---

## Issue Reporting Template

If issues are found:

```
**Issue:** [Brief description]
**Scenario:** [Scenario number]
**Test Case:** [Test case number]
**Steps to Reproduce:** [Steps]
**Expected:** [What should happen]
**Actual:** [What actually happened]
**Severity:** [Critical/High/Medium/Low]
**Environment:** [Browser, OS, Viewport]
**Screenshots:** [Attach if applicable]
```

---

## Sign-Off

- **Tester:** _______________
- **Date:** _______________
- **Result:** [ ] PASS [ ] FAIL
- **Notes:** _______________
