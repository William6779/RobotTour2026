#!/usr/bin/bash 

cd /home/pi/projects/rt
source /home/pi/projects/rt/.venv/bin/activate
/home/pi/projects/rt/.venv/bin/python3 gpio-buttons.py
