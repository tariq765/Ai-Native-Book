@echo off
echo ============================================
echo Starting Physical AI RAG Chatbot System
echo ============================================
echo.

REM Start Backend Server
echo [1/2] Starting Backend Server (Port 8000)...
start "RAG Backend Server" cmd /k "cd /d C:\Users\Hp\OneDrive\Desktop\Book2\rag_backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000"

REM Wait for backend to initialize
timeout /t 5 /nobreak >nul

REM Start Frontend Server
echo [2/2] Starting Frontend Server (Port 3000)...
start "Docusaurus Frontend" cmd /k "cd /d C:\Users\Hp\OneDrive\Desktop\Book2\physical-ai-humanoid-robotics-ts && npm start"

echo.
echo ============================================
echo Both servers are starting...
echo Backend: http://localhost:8000
echo Frontend: http://localhost:3000/physical-ai-humanoid-robotics/
echo ============================================
echo.
echo Press any key to close this window...
pause >nul
