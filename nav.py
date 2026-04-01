import serial
import requests
import time

COM_PORT = 'COM5'
BAUDRATE = 115200

origin = "-86.91211,40.42831"        # lon,lat
destination = "-86.91149,40.42681"   # lon,lat

# Request route
url = f"http://router.project-osrm.org/route/v1/foot/{origin};{destination}?overview=full&steps=true"
response = requests.get(url).json()

steps = response['routes'][0]['legs'][0]['steps']

waypoints = []

for step in steps:

    maneuver = step['maneuver']
    modifier = maneuver.get('modifier')
    distance = int(step.get('distance', 0))
    road = step.get('name', '').strip()

    lon, lat = maneuver['location']

    if not modifier or distance == 0:
        continue

    if road:
        instr = f"{modifier},{distance}m,{road}"
    else:
        instr = f"{modifier},{distance}m"

    line = f"{lat},{lon},{instr}"
    waypoints.append(line)

# Send to STM32
with serial.Serial(COM_PORT, BAUDRATE, timeout=2) as ser:

    time.sleep(1)

    for line in waypoints:
        ser.write((line + "\n").encode("ascii", errors="ignore"))
        time.sleep(0.05)

    ser.write(b"<END>\n")

print("\n".join(waypoints))