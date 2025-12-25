# RAG Chatbot Constitution

## 1. Purpose & Mission

The purpose of this project is to design, build, and embed a Retrieval-Augmented Generation (RAG) Chatbot inside a published book platform.
The chatbot's mission is to:

Help readers understand, explore, and query the book's content

Provide accurate, citation-aware answers

Answer questions based on:

The entire book

Only the text selected by the user

The chatbot must never hallucinate or answer beyond the allowed context.

## 2. Core Principles

### Context-Bound Answers Only

The chatbot MUST answer strictly from:

- Retrieved book content
- OR user-selected text

If the answer is not found, it MUST say:

"The answer is not available in the provided content."

### No External Knowledge Injection

The chatbot MUST NOT use general internet knowledge.

OpenAI models are used ONLY for reasoning over retrieved text.

### Explainability & Trust

Answers should be:

- Clear
- Concise
- Referenced (chapter/section/page when available)

### Reader-First UX

Chat experience must feel natural, fast, and helpful.

Embedded directly within the book interface.

## 3. Functional Scope

### 3.1 Chatbot Capabilities

The chatbot MUST support:

✅ Natural language Q&A

✅ Book-wide semantic search

✅ Selected-text-only mode

✅ Follow-up questions (conversation memory)

✅ Source-aware responses

### 3.2 Question Modes

#### A. Full Book Mode

Queries answered using the entire indexed book content

#### B. Selected Text Mode

- User selects a portion of text
- Chatbot answers only from that selection
- No other context is allowed

## 4. System Architecture

### 4.1 Frontend

Embedded chatbot UI inside the book website

Features:

- Chat panel
- Text selection detection
- Mode toggle (Full Book / Selected Text)
- Streaming responses

### 4.2 Backend (FastAPI)

FastAPI acts as the orchestration layer:

- Authentication
- Query routing
- Context enforcement
- Tool execution

### 4.3 AI Layer

OpenAI Agents / ChatKit SDK used for:

- Tool calling
- Reasoning
- Controlled response generation

### 4.4 Vector Database (Qdrant Cloud – Free Tier)

Stores:

- Book chunks
- Metadata (chapter, section, page)

Used for:

- Semantic search
- Context retrieval

### 4.5 Relational Database (Neon Serverless Postgres)

Stores:

- Users
- Chat sessions
- Messages
- Selected text snapshots
- Feedback & analytics

## 5. Retrieval & RAG Rules

All book content MUST be:

- Chunked
- Embedded
- Stored in Qdrant

Retrieval pipeline:

User query → vector search → top-K chunks

Chunks injected into LLM context

### Selected Text Override Rule

If user provides selected text:

- Vector search is DISABLED
- Only selected text is used

## 6. Answer Generation Rules

The chatbot MUST:

- Answer clearly
- Avoid speculation
- Use simple language

If information is incomplete:

- Ask for clarification
- OR state limitation clearly