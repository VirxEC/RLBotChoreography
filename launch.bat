@echo off

call venv/Scripts/activate
pip install -U -r requirements.txt
python ChoreographyHive --bot-folder=bots
pause