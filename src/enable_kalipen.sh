#!/bin/bash
sudo killall /dev/rfcomm0
sudo rfcomm connect /dev/rfcomm0 80:64:6F:C4:78:BE 1 &
sudo chmod -R 777 /dev/rfcomm0
