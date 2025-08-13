@echo off
set SCR=%1
if "%SCR%"=="" set SCR=ALL
python tools\run_testcases.py %SCR%
