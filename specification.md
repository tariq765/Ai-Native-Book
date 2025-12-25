# Specification: Physical AI & Humanoid Robotics Book

## Book Title
Physical AI & Humanoid Robotics
_Subtitle: From Digital Intelligence to Embodied Machines_

## Platform
- Docusaurus v3
- Markdown (.md / .mdx)
- GitHub Pages deployment

## Folder Structure
```
physical-ai-humanoid-robotics/
├── docs/
│   ├── intro.md
│   ├── getting-started.md
│   ├── chapters/
│   │   ├── quarter-1/
│   │   │   ├── chapter-1-introduction-to-physical-ai.md
│   │   │   ├── chapter-2-robotics-fundamentals.md
│   │   │   ├── chapter-3-sensors-and-perception.md
│   │   │   └── chapter-4-motion-and-control.md
│   │   │   ├── quarter-2/
│   │   │   │   ├── chapter-5-learning-in-physical-world.md
│   │   │   │   ├── chapter-6-humanoid-kinematics.md
│   │   │   │   ├── chapter-7-balance-and-locomotion.md
│   │   │   │   └── chapter-8-embodied-intelligence.md
│   │   │   ├── quarter-3/
│   │   │   │   ├── chapter-9-ai-for-robotics.md
│   │   │   │   ├── chapter-10-planning-and-navigation.md
│   │   │   │   ├── chapter-11-human-robot-interaction.md
│   │   │   │   └── chapter-12-ethical-considerations.md
│   │   │   └── quarter-4/
│   │   │       ├── chapter-13-advanced-control-systems.md
│   │   │       ├── chapter-14-multi-robot-systems.md
│   │   │       ├── chapter-15-future-of-humanoids.md
│   │   │       └── chapter-16-conclusion.md
│   │   └── appendices/
│   │       ├── appendix-a-mathematical-foundations.md
│   │       ├── appendix-b-robotics-software-tools.md
│   │       └── appendix-c-project-ideas.md
│   └── blog/
├── src/
│   ├── components/
│   └── css/
├── static/
│   └── img/
├── docusaurus.config.js
├── package.json
├── README.md
└── sidebars.js
```

## Content Guidelines
- Each chapter should be 2000-4000 words
- Include conceptual diagrams (described in text)
- Provide code examples where relevant (Python)
- Include exercises at the end of each chapter
- Use consistent terminology throughout

## Writing Style
- Begin each chapter with learning objectives
- Use analogies to explain complex concepts
- Include "Robotics in Action" sidebars for real-world examples
- End chapters with key takeaways and further reading

## Alternative Folder Structure (Modules-based)
```
docs/
├── intro.md
├── quarter-overview.md
├── module-1-ros2/
│   ├── intro.md
│   ├── ros2-basics.md
│   ├── nodes-topics-services.md
│   └── urdf-humanoids.md
├── module-2-digital-twin/
│   ├── intro.md
│   ├── gazebo-simulation.md
│   ├── unity-hri.md
│   └── sensors.md
├── module-3-isaac/
│   ├── intro.md
│   ├── isaac-sim.md
│   ├── isaac-ros.md
│   └── nav2.md
├── module-4-vla/
│   ├── intro.md
│   ├── voice-to-action.md
│   ├── llm-planning.md
│   └── capstone.md
```

## Chapter Requirements
Each chapter must include:
- Concept Overview
- Why it matters in Physical AI
- How it connects to humanoid robots
- Future industry relevance

## Writing Tone
- Teacher explaining to a motivated beginner
- Clear, structured, no jargon overload

## Images & Media
- Textual placeholders for diagrams
- Example: "Diagram: ROS 2 Node communication flow"

## Build Requirements
- No broken links
- Sidebar navigation configured
- Clean markdown formatting

# Specification: RAG Chatbot for Physical AI & Humanoid Robotics Book

## Overview
This document specifies the architecture and behavior of the RAG chatbot embedded within the Physical AI & Humanoid Robotics textbook.

## Functional Requirements

### 1. Knowledge Ingestion
- Parse all markdown files from `/docs`
- Split content into semantic chunks
- Generate embeddings for each chunk
- Store embeddings in Qdrant Cloud

### 2. Query Handling
The chatbot must support:
- Standard questions about the book
- Questions based on user-selected text only

### 3. Retrieval
- Perform vector similarity search
- Retrieve top-k relevant chunks
- Filter by chapter if metadata is available

### 4. Answer Generation
- Send retrieved chunks + user query to LLM
- Generate grounded answers only
- Return structured response

### 5. API Endpoints (FastAPI)
- POST /chat
- POST /chat-with-selection
- POST /ingest

### 6. Frontend Integration
- Floating chat widget in Docusaurus
- Support text selection → ask question
- Display responses with source citations