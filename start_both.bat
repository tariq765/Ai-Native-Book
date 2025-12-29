@echo off
echo ============================================
echo Starting Both Servers in One Terminal
echo ============================================
echo.

cd /d "C:\Users\Hp\OneDrive\Desktop\Book2\rag_backend"

echo Starting Backend and Frontend together...
echo Backend: http://localhost:8000
echo Frontend: http://localhost:3000/physical-ai-humanoid-robotics/
echo.
echo Press Ctrl+C to stop both servers
echo ============================================
echo.

REM Start both servers using && to run sequentially in background
start /B python -m uvicorn main:app --host 0.0.0.0 --port 8000 && cd /d "C:\Users\Hp\OneDrive\Desktop\Book2\physical-ai-humanoid-robotics-ts" && npm start
