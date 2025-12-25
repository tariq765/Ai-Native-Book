# Plan: Book Creation Workflow

## Phase 1: Project Setup
- Initialize Docusaurus project
- Configure sidebar and navigation
- Setup docs folder structure

## Phase 2: Core Content Writing
- Write Introduction & Quarter Overview
- Write Module 1 (ROS 2)
- Write Module 2 (Digital Twin)
- Write Module 3 (NVIDIA Isaac)
- Write Module 4 (Vision-Language-Action)

## Phase 3: Refinement
- Simplify explanations
- Ensure consistent terminology
- Add summaries to each module

## Phase 4: Build & Deploy
- Build Docusaurus locally
- Fix markdown errors
- Deploy to GitHub Pages

## Output
- Fully published AI-native textbook
- Ready for future RAG chatbot integration

# Plan: RAG Chatbot Development Roadmap

## Phase 1: Setup
- Initialize backend with FastAPI
- Configure environment variables
- Connect to Qdrant Cloud
- Connect to Neon Postgres

## Phase 2: Ingestion Pipeline
- Read markdown files from `/docs`
- Chunk content
- Generate embeddings
- Store vectors with metadata

## Phase 3: Retrieval Engine
- Implement similarity search
- Add selected-text filtering logic
- Rank and clean retrieved chunks

## Phase 4: LLM Integration
- Integrate OpenAI Agents/ChatKit or Gemini
- Prompt engineering for grounded answers
- Implement fallback responses

## Phase 5: API Layer
- Implement /chat endpoint
- Implement /chat-with-selection endpoint
- Add logging and error handling

## Phase 6: Frontend Embedding
- Build chat UI component
- Embed in Docusaurus
- Enable text selection interaction

## Phase 7: Testing & Validation
- Test with multiple chapters
- Validate hallucination control
- Demo preparation for hackathon

Claude Code should execute each phase sequentially.