#!/usr/bin/env python3
"""Example host UART client for Nexys board.
Requires `pyserial`.
"""
import serial
import time

def send_command(port, baud, cmd):
    with serial.Serial(port, baud, timeout=1) as s:
        s.write((cmd + "\n").encode('ascii'))
        time.sleep(0.1)
        resp = s.read_all().decode('ascii', errors='ignore')
        return resp

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--port', required=True)
    p.add_argument('--baud', default=115200)
    p.add_argument('cmd')
    args = p.parse_args()
    print(send_command(args.port, int(args.baud), args.cmd))
