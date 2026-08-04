@echo off
title Stop Coaxial UAV Simulation
echo Stopping coaxial_uav simulation...
wsl -d Ubuntu-22.04 bash /home/cxj/coaxial_uav/scripts/stop_all.sh
echo.
echo Done. You can close this window.
pause
