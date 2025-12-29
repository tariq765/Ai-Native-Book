@echo off
echo ============================================
echo Starting Both Servers (One Terminal)
echo ============================================
echo.
echo Backend: http://localhost:8000
echo Frontend: http://localhost:3000/physical-ai-humanoid-robotics/
echo.
echo Press Ctrl+C to stop all servers
echo ============================================
echo.

REM Run PowerShell script
powershell -ExecutionPolicy Bypass -File "%~dp0start_servers.ps1"
