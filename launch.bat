@echo off

call venv/Scripts/activate
pip install -U -r requirements.txt
python ChoreographyHive --bot-folder=C:\Users\ericm\AppData\Local\RLBotGUIX\RLBotPackDeletable\RLBotPack-master\RLBotPack

pause