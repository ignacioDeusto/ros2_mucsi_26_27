@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0launcher.ps1" %*
if "%~1"=="" pause
