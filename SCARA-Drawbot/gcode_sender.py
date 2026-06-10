#!/usr/bin/env python3
"""
SCARA GCode Sender
Sends a GCode file to the drawing robot over serial.

Usage:
    python3 gcode_sender.py <gcode_file> [port] [baud]

Examples:
    python3 gcode_sender.py hello.gcode
    python3 gcode_sender.py hello.gcode /dev/cu.usbmodem101
    python3 gcode_sender.py hello.gcode COM3 115200

The script:
  - Skips blank lines and comments (;)
  - Waits for 'OK' response before sending next line
  - Shows progress and any error responses
  - Ctrl+C to abort
"""

import sys
import time
import serial
import serial.tools.list_ports

def find_port():
    """Auto-detect ESP32 serial port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(x in p.description.lower() for x in ['usb', 'uart', 'cp210', 'ch340', 'ftdi']):
            return p.device
    if ports:
        return ports[0].device
    return None

def send_gcode(filename, port=None, baud=115200):
    # Load file
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Error: file '{filename}' not found")
        sys.exit(1)

    # Filter to commands only
    commands = []
    for line in lines:
        line = line.strip()
        if ';' in line:
            line = line[:line.index(';')].strip()
        if line:
            commands.append(line)

    print(f"Loaded {len(commands)} commands from '{filename}'")

    # Find port
    if port is None:
        port = find_port()
        if port is None:
            print("Error: no serial port found. Specify port as argument.")
            sys.exit(1)
    print(f"Connecting to {port} at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=30)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    time.sleep(2)  # wait for ESP32 reset
    ser.reset_input_buffer()

    # Read startup banner
    print("Waiting for robot ready...")
    deadline = time.time() + 5
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            print(f"  Robot: {line}")
            if 'ready' in line.lower():
                break

    print(f"\nSending {len(commands)} commands...\n")

    try:
        for i, cmd in enumerate(commands):
            print(f"[{i+1}/{len(commands)}] >> {cmd}")
            ser.write((cmd + '\n').encode())

            # Wait for response
            deadline = time.time() + 60  # 60s timeout per command
            while time.time() < deadline:
                if ser.in_waiting:
                    response = ser.readline().decode('utf-8', errors='replace').strip()
                    if response:
                        print(f"           << {response}")
                    if response.startswith('OK') or response.startswith('ok'):
                        break
                    if response.startswith('ERR'):
                        print(f"\n*** ERROR: {response} ***")
                        print("Aborting.")
                        ser.close()
                        sys.exit(1)
                    if response.startswith('IK:'):
                        continue  # debug output, keep waiting
                time.sleep(0.01)
            else:
                print(f"\nTimeout waiting for response to: {cmd}")
                ser.close()
                sys.exit(1)

    except KeyboardInterrupt:
        print("\n\nAborted by user.")

    ser.close()
    print("\nDone!")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    filename = sys.argv[1]
    port     = sys.argv[2] if len(sys.argv) > 2 else None
    baud     = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    send_gcode(filename, port, baud)
