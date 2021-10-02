#!/bin/bash

password=field123

sshpass -p $password scp -r frc-uav@frc-uav.local:/home/frc-uav/agbot_data /home/vrushali/Desktop/Agbot_Data

