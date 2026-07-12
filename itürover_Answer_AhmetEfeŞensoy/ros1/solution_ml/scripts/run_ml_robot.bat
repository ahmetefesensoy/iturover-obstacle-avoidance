@echo off
echo WSL kapatiliyor (Gazebo blank bug icin)...
wsl --shutdown
timeout /t 3 /nobreak >nul

echo ML Robot baslatiliyor...
wsl -d Ubuntu-20.04 bash ~/run_ml_robot.sh
pause
