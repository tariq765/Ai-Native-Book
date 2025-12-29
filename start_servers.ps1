# PowerShell script to run both servers in one terminal
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "Starting Physical AI RAG Chatbot System" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# Start Backend in background job
Write-Host "[1/2] Starting Backend Server..." -ForegroundColor Yellow
$backend = Start-Job -ScriptBlock {
    Set-Location "C:\Users\Hp\OneDrive\Desktop\Book2\rag_backend"
    python -m uvicorn main:app --host 0.0.0.0 --port 8000
}

Start-Sleep -Seconds 5

# Start Frontend in background job
Write-Host "[2/2] Starting Frontend Server..." -ForegroundColor Yellow
$frontend = Start-Job -ScriptBlock {
    Set-Location "C:\Users\Hp\OneDrive\Desktop\Book2\physical-ai-humanoid-robotics-ts"
    npm start
}

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "Both servers are running!" -ForegroundColor Green
Write-Host "Backend:  http://localhost:8000" -ForegroundColor White
Write-Host "Frontend: http://localhost:3000/physical-ai-humanoid-robotics/" -ForegroundColor White
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host "Monitoring server output..." -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop all servers" -ForegroundColor Red
Write-Host ""

# Monitor both jobs
try {
    while ($true) {
        Receive-Job -Job $backend -ErrorAction SilentlyContinue
        Receive-Job -Job $frontend -ErrorAction SilentlyContinue
        Start-Sleep -Seconds 1
    }
}
finally {
    Write-Host ""
    Write-Host "Stopping servers..." -ForegroundColor Yellow
    Stop-Job -Job $backend, $frontend
    Remove-Job -Job $backend, $frontend
    Write-Host "All servers stopped." -ForegroundColor Green
}
