@echo off
rem Sets the wind in the running Gazebo simulation from Windows (PowerShell or cmd):
rem   wind SPEED FROM_DEG     e.g.  wind 6 270  = 6 m/s from the west,  wind 0 0 = calm
rem The direction is where the wind comes from (0 = north, 90 = east), like ArduPilot shows it.
rem Runs setup/gz_set_wind.sh inside Ubuntu; the simulated airspeed sensor follows by itself.
if "%~2"=="" (
  echo usage: wind SPEED FROM_DEG    e.g.  wind 6 270  = 6 m/s from the west,  wind 0 0 = calm
  exit /b 1
)
wsl -d Ubuntu-22.04 -- bash /mnt/c/Users/funfo/vio/setup/gz_set_wind.sh %1 %2
