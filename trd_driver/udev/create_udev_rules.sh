#!/bin/bash

echo "Creating /etc/udev/rules.d/motor_trd.rules"
cp motor_trd.rules  /etc/udev/rules.d
echo "Restarting udev"
service udev reload
sleep 2
service udev restart
echo "OK!"

