@echo off
title Coaxial UAV Launcher

echo [1/2] Opening Gazebo + Dashboard in one terminal window...
wt -w new new-tab --title "Gazebo" wsl -d Ubuntu-22.04 bash -ic "cd ~/coaxial_uav && ./scripts/run_static_water_gui_ogre2.sh" ; new-tab --title "Dashboard" wsl -d Ubuntu-22.04 bash -ic "sleep 5 && cd ~/coaxial_uav && ./scripts/run_dashboard.sh"

echo [2/2] Waiting 9s for services, then opening the web console...
ping -n 10 127.0.0.1 >nul
start http://127.0.0.1:5223

echo.
echo Done. Web console: http://127.0.0.1:5223
echo To stop everything, double-click "Stop Simulation" on the desktop.
ping -n 5 127.0.0.1 >nul
