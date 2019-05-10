#!/bin/bash

# Get current time since epoch


DATE_PC=`date "+%s"`

DATE_PI="@"$DATE_PC

# Save date in shell script
echo sudo date -s \'"$DATE_PI"\'> pi_updateclock.sh
echo acro >> pi_updateclock.sh

# Set date of pi over SSH with shell script
sshpass -p 'groupRBP' ssh rosberrypi@192.168.42.1 -tt 'bash -s' < pi_updateclock.sh


