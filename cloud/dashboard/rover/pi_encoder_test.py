#!/usr/bin/env python3
"""Encoder swap verification test for FPMS V3.0 board."""

from Rosmaster_Lib import Rosmaster
import time

PORT = "/dev/ttyUSB0"

bot = Rosmaster(com=PORT)
bot.create_receive_threading()
time.sleep(0.5)

fw = bot.get_version()
print(f"Firmware version: {fw}")
if fw == 0:
    print("ERROR: not getting responses from board. Check port / power / cable.")
    bot.del_thread()
    exit(1)

e_before = bot.get_motor_encoder()
print(f"\nEncoders at rest: M1={e_before[0]}  M2={e_before[1]}  M3={e_before[2]}  M4={e_before[3]}")

SPEED = 50
DURATION = 1.5

for motor_idx in range(4):
    speeds = [0, 0, 0, 0]
    speeds[motor_idx] = SPEED

    print(f"\n--- Driving M{motor_idx+1} forward at {SPEED}% for {DURATION}s ---")
    bot.set_motor(*speeds)
    time.sleep(DURATION)
    bot.set_motor(0, 0, 0, 0)
    time.sleep(0.3)

    e_now = bot.get_motor_encoder()
    delta = [e_now[i] - e_before[i] for i in range(4)]
    print(f"Encoder deltas: M1={delta[0]:+d}  M2={delta[1]:+d}  M3={delta[2]:+d}  M4={delta[3]:+d}")
    e_before = e_now

print(f"\n--- Driving M1 BACKWARD at {SPEED}% for {DURATION}s ---")
bot.set_motor(-SPEED, 0, 0, 0)
time.sleep(DURATION)
bot.set_motor(0, 0, 0, 0)
time.sleep(0.3)
e_now = bot.get_motor_encoder()
delta = [e_now[i] - e_before[i] for i in range(4)]
print(f"Encoder deltas: M1={delta[0]:+d}  M2={delta[1]:+d}  M3={delta[2]:+d}  M4={delta[3]:+d}")

bot.del_thread()
print("\nTest complete.")
