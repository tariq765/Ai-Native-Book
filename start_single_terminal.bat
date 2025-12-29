@echo off
title Physical AI RAG Chatbot - Both Servers
color 0A

echo ============================================
echo   Physical AI RAG Chatbot System
echo ============================================
echo.
echo Starting Backend Server (Port 8000)...
echo.

REM Start backend in a new minimized window
start /MIN "Backend Server" cmd /c "cd /d C:\Users\Hp\OneDrive\Desktop\Book2\rag_backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000"

REM Wait 8 seconds for backend to fully start
echo Waiting for backend to initialize...
timeout /t 8 /nobreak >nul

REM Check if backend is running
echo Checking backend status...
curl -s http://localhost:8000 >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Backend failed to start!
    echo Please check the Backend Server window for errors.
    pause
    exit /b 1
)

echo [OK] Backend is running at http://localhost:8000
echo.
echo Starting Frontend Server (Port 3000)...
echo This will open in your browser automatically...
echo.
echo ============================================
echo   Both servers are running!
echo   Backend:  http://localhost:8000
echo   Frontend: http://localhost:3000/physical-ai-humanoid-robotics/
echo ============================================
echo.
echo IMPORTANT:
echo - Do NOT close this window while using the chatbot
echo - Backend is running in a minimized window
echo - Frontend logs will appear below
echo.
echo To stop: Close this window or press Ctrl+C
echo ============================================
echo.

REM Start frontend in THIS terminal (so you can see logs)
cd /d "C:\Users\Hp\OneDrive\Desktop\Book2\physical-ai-humanoid-robotics-ts"
npm start
