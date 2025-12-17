---
id: 3
title: rag-chatbot-widget-spec
stage: spec
date_iso: 2025-12-09
model: Qwen
feature: 003-rag-chatbot-widget
branch: 003-rag-chatbot-widget
user: user
command: /sp.specify Component: React RAG Chatbot Widget for Docusaurus Objective: Create a floating chatbot widget that appears on every page of the Docusaurus book, supports text selection queries, and provides smooth user experience. UI/UX Requirements: 1. Floating Chat Button - Position: Bottom-right corner (24px from edges) - Size: 60x60px circle - Icon: 💬 when closed, ✕ when open - Color: Purple gradient (#667eea to #764ba2) - Hover effect: Slight lift + shadow - Always visible (z-index: 9999) 2. Chat Window - Size: 400px width, 600px height (desktop) - Size: 100vw x 70vh (mobile) - Position: Bottom-right, above button - Border-radius: 16px - Box-shadow: Large shadow for depth - Background: White (light mode), #151937 (dark mode) 3. Chat Header - Title: "🤖 AI Assistant" - Subtitle: "Ask me about the book" - Close button (X) - Minimize button (−) 4. Messages Area - Auto-scroll to latest message - User messages: Right-aligned, purple background - Bot messages: Left-aligned, gray background - Timestamp below each message - Loading indicator (3 dots animation) - Show "Context: Selected Text" or "Context: Full Book" 5. Selected Text Banner - Appears when text is selected - Shows: "📝 Text selected: [first 50 chars]..." - Clear button (X) to remove selection - Purple background, white text 6. Input Area - Textarea (auto-expand, max 4 lines) - Placeholder: "Ask a question about the book..." - Send button (paper plane icon) - Character counter (max 500 chars) - Disable when loading Features: 1. Text Selection Detection - Listen for mouseup event on document - Capture selected text if >10 characters - Show banner when text selected - Send selected text with query 2. Session Management - Generate UUID on first load - Store in localStorage - Persist across page navigation - Load previous chat history 3. Chat History - Fetch history on open (if session exists) - Display last 20 messages - Scroll to bottom on new message - Clear history button in settings 4. API Integration - POST request to /chat endpoint - Handle loading states - Handle errors gracefully - Retry on failure (max 3 attempts) - Timeout after 30 seconds 5. Responsive Design - Desktop: Fixed size window - Tablet: Slightly smaller - Mobile: Full-screen overlay - Touch-friendly buttons (min 44px) 6. Accessibility - Keyboard navigation (Tab, Enter, Esc) - ARIA labels on all interactive elements - Focus management - Screen reader support Technical Implementation: Component Structure: ```jsx <RAGChatbot> <ChatButton onClick={toggleChat} /> {isOpen && ( <ChatWindow> <ChatHeader /> {selectedText && <SelectedTextBanner />} <MessagesContainer> {messages.map(msg => <Message />)} </MessagesContainer> <InputArea /> </ChatWindow> )} </RAGChatbot> ``` State Management: - isOpen (boolean) - messages (array) - input (string) - isLoading (boolean) - sessionId (string) - selectedText (string) - error (string | null) API Client: ```javascript async function sendMessage(query, sessionId, selectedText) { const response = await fetch(API_URL + '/chat', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ query, session_id: sessionId, selected_text: selectedText }) }); return response.json(); } ``` Integration with Docusaurus: - Import in docusaurus.config.js as global component - Or add to theme wrapper - Ensure it appears on all pages - Doesn't interfere with page scrolling Styling: - CSS Modules for component styles - Use Docusaurus theme colors - Dark mode support - Smooth animations (0.3s transitions) - Mobile-first approach Error States: - API error → Show "Try again" message - Network error → Show "Check connection" - Timeout → Show "Response took too long" - Invalid response → Show generic error Success Criteria: - Chatbot appears on all book pages - Text selection feature works - Messages send and display correctly - Smooth animations and transitions - Works on mobile devices - Chat history persists - No layout shift or jank
labels: ["spec", "rag", "chatbot", "widget", "docusaurus", "frontend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/003-rag-chatbot-widget/spec.md"
  - "specs/003-rag-chatbot-widget/checklists/requirements.md"
tests_yaml:
  - "specs/003-rag-chatbot-widget/spec.md"
---

# Prompt History Record: RAG Chatbot Widget Specification

## Original Prompt
```
/sp.specify Component: React RAG Chatbot Widget for Docusaurus Objective: Create a floating chatbot widget that appears on every page of the Docusaurus book, supports text selection queries, and provides smooth user experience. UI/UX Requirements: 1. Floating Chat Button - Position: Bottom-right corner (24px from edges) - Size: 60x60px circle - Icon: 💬 when closed, ✕ when open - Color: Purple gradient (#667eea to #764ba2) - Hover effect: Slight lift + shadow - Always visible (z-index: 9999) 2. Chat Window - Size: 400px width, 600px height (desktop) - Size: 100vw x 70vh (mobile) - Position: Bottom-right, above button - Border-radius: 16px - Box-shadow: Large shadow for depth - Background: White (light mode), #151937 (dark mode) 3. Chat Header - Title: "🤖 AI Assistant" - Subtitle: "Ask me about the book" - Close button (X) - Minimize button (−) 4. Messages Area - Auto-scroll to latest message - User messages: Right-aligned, purple background - Bot messages: Left-aligned, gray background - Timestamp below each message - Loading indicator (3 dots animation) - Show "Context: Selected Text" or "Context: Full Book" 5. Selected Text Banner - Appears when text is selected - Shows: "📝 Text selected: [first 50 chars]..." - Clear button (X) to remove selection - Purple background, white text 6. Input Area - Textarea (auto-expand, max 4 lines) - Placeholder: "Ask a question about the book..." - Send button (paper plane icon) - Character counter (max 500 chars) - Disable when loading Features: 1. Text Selection Detection - Listen for mouseup event on document - Capture selected text if >10 characters - Show banner when text selected - Send selected text with query 2. Session Management - Generate UUID on first load - Store in localStorage - Persist across page navigation - Load previous chat history 3. Chat History - Fetch history on open (if session exists) - Display last 20 messages - Scroll to bottom on new message - Clear history button in settings 4. API Integration - POST request to /chat endpoint - Handle loading states - Handle errors gracefully - Retry on failure (max 3 attempts) - Timeout after 30 seconds 5. Responsive Design - Desktop: Fixed size window - Tablet: Slightly smaller - Mobile: Full-screen overlay - Touch-friendly buttons (min 44px) 6. Accessibility - Keyboard navigation (Tab, Enter, Esc) - ARIA labels on all interactive elements - Focus management - Screen reader support Technical Implementation: Component Structure: ```jsx <RAGChatbot> <ChatButton onClick={toggleChat} /> {isOpen && ( <ChatWindow> <ChatHeader /> {selectedText && <SelectedTextBanner />} <MessagesContainer> {messages.map(msg => <Message />)} </MessagesContainer> <InputArea /> </ChatWindow> )} </RAGChatbot> ``` State Management: - isOpen (boolean) - messages (array) - input (string) - isLoading (boolean) - sessionId (string) - selectedText (string) - error (string | null) API Client: ```javascript async function sendMessage(query, sessionId, selectedText) { const response = await fetch(API_URL + '/chat', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ query, session_id: sessionId, selected_text: selectedText }) }); return response.json(); } ``` Integration with Docusaurus: - Import in docusaurus.config.js as global component - Or add to theme wrapper - Ensure it appears on all pages - Doesn't interfere with page scrolling Styling: - CSS Modules for component styles - Use Docusaurus theme colors - Dark mode support - Smooth animations (0.3s transitions) - Mobile-first approach Error States: - API error → Show "Try again" message - Network error → Show "Check connection" - Timeout → Show "Response took too long" - Invalid response → Show generic error Success Criteria: - Chatbot appears on all book pages - Text selection feature works - Messages send and display correctly - Smooth animations and transitions - Works on mobile devices - Chat history persists - No layout shift or jank
```

## Response Summary
Successfully created a feature specification for the React RAG Chatbot Widget. The following artifacts were generated:

1. `specs/003-rag-chatbot-widget/spec.md` - Feature specification with user stories, requirements, and success criteria
2. `specs/003-rag-chatbot-widget/checklists/requirements.md` - Specification quality checklist

The specification was created following these key steps:
1. Generated a short name "rag-chatbot-widget" and determined the next feature number (003)
2. Created a new feature branch and spec file
3. Developed a comprehensive specification that:
   - Focuses on user value and business needs
   - Removes implementation details (languages, frameworks, APIs)
   - Includes 3 prioritized user stories
   - Defines 11 functional requirements and 3 requiring clarification
   - Identifies key entities involved in the system
   - Establishes 9 measurable success criteria

The specification underwent quality validation and was refined to ensure it contained no implementation-specific details, focusing instead on business requirements and user needs.

## Outcome
The feature specification is complete and ready for the next phase of development. It successfully abstracts the technical implementation details while preserving the core functionality requirements. Three areas requiring clarification were identified (accessibility contrast ratios, message history retention, and session expiration) which will need to be addressed before implementation planning.

## Evaluation
The specification meets the required standards:
- Contains no implementation details (languages, frameworks, APIs)
- Focuses on user value and business needs
- Written for non-technical stakeholders
- All mandatory sections completed
- Requirements are testable and unambiguous
- Success criteria are measurable and technology-agnostic
- All acceptance scenarios are defined
- Edge cases are identified
- Scope is clearly bounded
- Dependencies and assumptions identified

The specification is ready for the planning phase once the three clarification questions are answered.