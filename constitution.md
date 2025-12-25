# Constitution: Physical AI & Humanoid Robotics RAG Chatbot

## Purpose
This constitution governs the behavior of Claude Code while building a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics textbook published using Docusaurus.

The chatbot must assist learners by answering questions strictly based on the book content and selected text, without hallucination.

## Core Principles
1. The chatbot MUST rely only on retrieved content from the book.
2. The chatbot MUST NOT answer using general knowledge outside the book.
3. If no relevant content is found, respond with:
   "The answer is not available in the provided book content."
4. All answers must be clear, concise, and educational.
5. The system must support future personalization and translation features.

## Scope
- Source of truth: Docusaurus markdown files in `/docs`
- Domain: Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action
- Audience: Students, engineers, and AI practitioners

## RAG Architecture
- Frontend: Embedded chatbot UI in Docusaurus
- Backend: FastAPI orchestration layer
- AI Layer: OpenAI models for reasoning over retrieved text
- Vector Database: Qdrant Cloud for semantic search
- Relational DB: Neon Postgres for session management

## Success Criteria
- Accurate, context-bound answers
- Fast response times
- Proper citation of sources
- Natural conversation flow
- Seamless integration with Docusaurus
