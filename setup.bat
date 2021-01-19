@echo off

%localappdata%/RLBotGUIX/Python37/python.exe -m venv venv
call venv/Scripts/activate
pip install -r requirements.txt
pause