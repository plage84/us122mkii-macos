#!/bin/bash
# Retire le driver et redémarre Core Audio. / Remove the driver.
sudo rm -rf /Library/Audio/Plug-Ins/HAL/US122MKII.driver
sudo killall coreaudiod 2>/dev/null || true
echo "Driver retiré."
