#!/bin/bash
# Start XLeRobot host on Jetson
# Usage: ssh into Jetson, then run this script
#   cd /home/jetson/Desktop/lerobot && bash scripts/start_host.sh

set -e
cd "$(dirname "$0")/.."

echo "=== XLeRobot Host Startup ==="

# Check which ports have which motors
echo "Scanning USB ports..."
PYTHONPATH=src python -c "
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
    b = FeetechMotorsBus(port=p, motors={'x': Motor(1, 'sts3215', MotorNormMode.RANGE_M100_100)})
    b.port_handler.openPort(); b.port_handler.setBaudRate(1000000)
    ids = [i for i in range(1,10) if b.packet_handler.ping(b.port_handler, i)[1]==0]
    print(f'  {p}: motor IDs {ids}')
    b.port_handler.closePort()
"

echo ""
echo "Config expects:"
echo "  port1 (head, IDs 7-8):  $(grep 'port1' src/lerobot/robots/xlerobot/config_xlerobot.py | head -1)"
echo "  port2 (arm+base, IDs 1-9): $(grep 'port2' src/lerobot/robots/xlerobot/config_xlerobot.py | head -1)"
echo ""
echo "If ports don't match, update config and git pull."
echo "Press ENTER to start host, or Ctrl+C to abort."
read

echo "Starting host..."
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
