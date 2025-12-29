========================================
Physical AI & Humanoid Robotics RAG Chatbot
========================================

QUICK START:
------------
1. Double-click "start_all.bat" file
2. Wait for both servers to start (2 terminal windows will open)
3. Browser will open automatically at: http://localhost:3000/physical-ai-humanoid-robotics/
4. Click the chat button (💬) in bottom-right corner
5. Ask questions about Physical AI and Humanoid Robotics!

MANUAL START (if batch file doesn't work):
------------------------------------------

Terminal 1 - Backend:
  cd "C:\Users\Hp\OneDrive\Desktop\Book2\rag_backend"
  python -m uvicorn main:app --host 0.0.0.0 --port 8000

Terminal 2 - Frontend:
  cd "C:\Users\Hp\OneDrive\Desktop\Book2\physical-ai-humanoid-robotics-ts"
  npm start

URLS:
-----
Backend API: http://localhost:8000
Frontend Website: http://localhost:3000/physical-ai-humanoid-robotics/

TROUBLESHOOTING:
----------------
- If port 8000 or 3000 is busy, close other apps using those ports
- Make sure Python and Node.js are installed
- Check that all API keys are set in rag_backend\.env file

API KEYS REQUIRED (in .env file):
---------------------------------
✓ COHERE_API_KEY - For embeddings
✓ QDRANT_URL & QDRANT_API_KEY - For vector database
✓ OPENROUTER_API_KEY - For LLM responses

All keys are already configured in your .env file!

========================================
Enjoy your AI-powered book chatbot! 🚀
========================================
