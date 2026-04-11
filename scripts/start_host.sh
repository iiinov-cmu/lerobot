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
from scservo_sdk import PortHandler, PacketHandler
for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
    ph = PortHandler(p)
    if not ph.openPort():
        print(f'  {p}: could not open')
        continue
    ph.setBaudRate(1000000)
    pkt = PacketHandler(0)
    ids = []
    for i in range(1, 10):
        _, result, _ = pkt.ping(ph, i)
        if result == 0:
            ids.append(i)
    ph.closePort()
    print(f'  {p}: motor IDs {ids}')
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
