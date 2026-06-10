#!/usr/bin/env python3
"""
Inkscape GCode Post-Processor for SCARA Drawing Robot
======================================================
Converts GCode exported from Inkscape to the format expected by the robot.

Usage:
    python3 inkscape_to_robot.py input.gcode [output.gcode] [options]

Options:
    --scale FACTOR      Scale factor (default: 1.0)
    --offset-x MM       X offset in mm (default: 0)
    --offset-y MM       Y offset in mm (default: 0)
    --flip-y            Flip Y axis (Inkscape Y is inverted vs robot Y)
    --rotate DEG        Rotate drawing by degrees (default: 0)
    --speed MM          Drawing speed step interval us (default: don't set)
    --preview           Print bounding box and stats without writing file

Examples:
    # Basic conversion with Y flip (almost always needed)
    python3 inkscape_to_robot.py drawing.gcode --flip-y

    # Scale down 50%, flip Y, centre on canvas
    python3 inkscape_to_robot.py drawing.gcode --scale 0.5 --flip-y

    # Check bounding box first
    python3 inkscape_to_robot.py drawing.gcode --flip-y --preview
"""

import sys
import re
import math
import argparse

def parse_args():
    p = argparse.ArgumentParser(description='Inkscape GCode post-processor')
    p.add_argument('input',  help='Input .gcode file from Inkscape')
    p.add_argument('output', nargs='?', help='Output .gcode file (default: input_robot.gcode)')
    p.add_argument('--scale',    type=float, default=1.0,  help='Scale factor')
    p.add_argument('--offset-x', type=float, default=0.0,  help='X offset mm')
    p.add_argument('--offset-y', type=float, default=0.0,  help='Y offset mm')
    p.add_argument('--flip-y',   action='store_true',      help='Flip Y axis')
    p.add_argument('--rotate',   type=float, default=0.0,  help='Rotation degrees CCW')
    p.add_argument('--skew',     type=float, default=-5.0, help='Skew correction degrees (default: -5)')
    p.add_argument('--speed',    type=int,   default=None, help='Step interval us')
    p.add_argument('--preview',  action='store_true',      help='Preview only, no output')
    return p.parse_args()

def parse_coord(line, axis):
    """Extract a coordinate value from a GCode line."""
    m = re.search(rf'{axis}([+-]?\d+\.?\d*)', line, re.IGNORECASE)
    return float(m.group(1)) if m else None

def transform(x, y, args, doc_height=None):
    """Apply all transformations to a coordinate pair."""
    # Flip Y (Inkscape Y=0 is top, robot Y increases upward)
    if args.flip_y and doc_height is not None:
        y = doc_height - y

    # Scale
    x *= args.scale
    y *= args.scale

    # Rotate
    if args.rotate != 0:
        r = math.radians(args.rotate)
        x, y = x*math.cos(r) - y*math.sin(r), x*math.sin(r) + y*math.cos(r)

    # Offset
    x += args.offset_x
    y += args.offset_y

    # Skew correction (robot canvas correction)
    if args.skew != 0:
        shear = math.tan(math.radians(args.skew))
        x = x + y * shear

    return round(x, 3), round(y, 3)

def detect_doc_height(lines):
    """Try to detect document height from GCode comments."""
    for line in lines:
        # Some exporters write dimensions in comments
        m = re.search(r'height[:\s=]+([0-9.]+)', line, re.IGNORECASE)
        if m:
            return float(m.group(1))
    return None

def process(lines, args):
    """Process GCode lines and return robot-compatible lines."""
    output = []
    pen_down = False
    current_x = None
    current_y = None
    xs = []; ys = []

    # Try to detect document size for Y flip
    doc_height = detect_doc_height(lines)
    if args.flip_y and doc_height is None:
        # Common Inkscape default or try to infer from coordinates
        all_ys = []
        for line in lines:
            y = parse_coord(line, 'Y')
            if y is not None: all_ys.append(y)
        if all_ys:
            doc_height = max(all_ys)
            print(f"  Auto-detected doc height: {doc_height:.1f}mm")

    # Header
    output.append("; GCode converted from Inkscape")
    output.append(f"; Scale: {args.scale}  FlipY: {args.flip_y}  Rotate: {args.rotate}°")
    output.append(f"; Skew correction: {args.skew}°")
    output.append("G28")
    output.append("GOHOME")
    output.append("M5  ; pen up")
    if args.speed:
        output.append(f"SPEED {args.speed}")
    output.append("")

    for line in lines:
        line = line.strip()
        if not line or line.startswith(';'):
            continue

        line_upper = line.upper()

        # Pen down equivalents from various exporters
        if any(x in line_upper for x in ['M3', 'M03', 'PENDOWN', 'PEN_DOWN',
                                           'M300', 'G1 Z-', 'G01 Z-',
                                           '; PEN DOWN', ';PEN DOWN']):
            if not pen_down:
                output.append("M3")
                pen_down = True
            continue

        # Pen up equivalents
        if any(x in line_upper for x in ['M5', 'M05', 'PENUP', 'PEN_UP',
                                          'M301', 'G1 Z', 'G0 Z', 'G00 Z',
                                          '; PEN UP', ';PEN UP']):
            if pen_down:
                output.append("M5")
                pen_down = False
            continue

        # G0 / G1 moves
        if line_upper.startswith(('G0 ', 'G00 ', 'G1 ', 'G01 ',
                                   'G0\t', 'G00\t', 'G1\t', 'G01\t')):
            x = parse_coord(line, 'X')
            y = parse_coord(line, 'Y')

            if x is None and y is None:
                continue  # Z-only move, skip

            # Use current position for missing axis
            if x is None: x = current_x if current_x else 0
            if y is None: y = current_y if current_y else 0

            tx, ty = transform(x, y, args, doc_height)
            current_x = x; current_y = y

            xs.append(tx); ys.append(ty)

            is_rapid = line_upper.startswith(('G0 ', 'G00 ', 'G0\t', 'G00\t'))
            cmd = 'G0' if is_rapid else 'G1'
            output.append(f"{cmd} X{tx} Y{ty}")
            continue

        # Pass through other lines we don't recognise
        # (but filter out Z, F, S parameters we don't use)
        # Skip: F (feedrate), S (spindle), Z (depth)
        if re.match(r'^[MFST]', line_upper) and not line_upper.startswith(('M3', 'M5')):
            continue

    # Footer
    if pen_down:
        output.append("M5  ; ensure pen up")
    output.append("G0 X0 Y0  ; return to centre")

    # Stats
    if xs and ys:
        print(f"  X range: {min(xs):.1f} to {max(xs):.1f} mm  (width {max(xs)-min(xs):.1f}mm)")
        print(f"  Y range: {min(ys):.1f} to {max(ys):.1f} mm  (height {max(ys)-min(ys):.1f}mm)")
        cx = (min(xs)+max(xs))/2; cy = (min(ys)+max(ys))/2
        print(f"  Centre:  ({cx:.1f}, {cy:.1f})")
        if abs(cx) > 5 or abs(cy) > 5:
            print(f"  WARNING: Drawing is off-centre. Consider:")
            print(f"    --offset-x {-cx:.1f} --offset-y {-cy:.1f}")
        if max(xs)-min(xs) > 180:
            print(f"  WARNING: Drawing width {max(xs)-min(xs):.1f}mm exceeds recommended 180mm")
        if max(ys)-min(ys) > 130:
            print(f"  WARNING: Drawing height {max(ys)-min(ys):.1f}mm exceeds recommended 130mm")

    return output

def main():
    args = parse_args()

    # Read input
    try:
        with open(args.input, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Error: '{args.input}' not found")
        sys.exit(1)

    print(f"Processing '{args.input}'...")
    print(f"  {len(lines)} input lines")

    output = process(lines, args)

    move_count = sum(1 for l in output if l.startswith(('G0','G1')))
    pen_ops    = sum(1 for l in output if l.strip() in ('M3','M5'))
    print(f"  {move_count} move commands, {pen_ops} pen operations")

    if args.preview:
        print("\nPreview mode — no file written.")
        print("First 20 output lines:")
        for l in output[:20]: print(f"  {l}")
        return

    # Write output
    outfile = args.output or args.input.replace('.gcode', '_robot.gcode')
    with open(outfile, 'w') as f:
        f.write('\n'.join(output) + '\n')
    print(f"Written to '{outfile}'")
    print(f"\nTo send to robot:")
    print(f"  python3 gcode_sender.py {outfile} /dev/cu.usbmodem201301")

if __name__ == '__main__':
    main()
