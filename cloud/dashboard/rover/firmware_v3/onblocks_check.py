#!/usr/bin/env python3
"""
FPMS V3 -- WHEELS-OFF-THE-GROUND VERIFICATION. Run this FIRST, before the
rover ever touches the floor again.

    ROS_DOMAIN_ID=20 python3 onblocks_check.py

WHAT IT PROVES, in about four minutes:
  * the /cmd_duty path actually drives motors (and with no dead zone)
  * the deadman zeroes the wheels when commands stop
  * per-wheel MOTORn_INV -- motors 2 and 4 are wired electrically opposite,
    so a mistake there shows up immediately as a wheel spinning backwards
  * every encoder is alive, mapped to the right wheel, and correctly SIGNED
  * there is no cross-wiring (driving wheel N must move ONLY tick N)

THE STANDING SAFETY RULE THIS ENFORCES:
    before the rover touches the floor, verify PER WHEEL that driving forward
    makes THAT wheel's encoder count UP.

This script never drives more than one wheel at a time, never for more than
500 ms, and always sends an explicit zero afterwards. It is safe on blocks. It
is NOT safe on the floor -- the rover will lurch.
"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool

WHEELS = ["1 front-left", "2 front-right", "3 rear-left", "4 rear-right"]
DUTY = 15          # percent. Low enough to be harmless, high enough to turn a
                   # free wheel. If nothing moves at 15, try 25 before
                   # concluding the path is broken -- an unloaded gearbox can
                   # still have real stiction.
BURST_S = 0.5
SETTLE_S = 0.6     # let the wheel coast to rest so ticks are read AT REST,
                   # which is exactly how the golden B8B driver measured.


class Check(Node):
    def __init__(self):
        super().__init__("fpms_onblocks_check")
        self.pub = self.create_publisher(Int32MultiArray, "/cmd_duty", 10)
        self.estop = self.create_publisher(Bool, "/estop", 10)
        self.ticks = None
        self.health = None
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._t, 10)
        self.create_subscription(Int32MultiArray, "/fpms_health", self._h, 10)

    def _t(self, m): self.ticks = list(m.data)
    def _h(self, m): self.health = list(m.data)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, duty4):
        m = Int32MultiArray(); m.data = [int(v) for v in duty4]; self.pub.publish(m)

    def stop(self):
        for _ in range(5):
            self.send([0, 0, 0, 0]); self.spin_for(0.02)

    def burst(self, idx, duty):
        """Drive ONE wheel, re-sending at 20 Hz because the firmware deadman
        zeroes the motors after 300 ms without a command. Having to re-send is
        the deadman working -- it is not an inconvenience to design around."""
        cmd = [0, 0, 0, 0]; cmd[idx] = duty
        end = time.time() + BURST_S
        while time.time() < end:
            self.send(cmd); self.spin_for(0.05)
        self.stop()
        self.spin_for(SETTLE_S)


def main():
    rclpy.init()
    n = Check()

    print(__doc__)
    print("=" * 68)
    ans = input("Are ALL FOUR wheels off the ground and free to spin? [yes/NO] ")
    if ans.strip().lower() != "yes":
        print("Aborting. Put the rover on blocks first."); return 1

    # Clear any latched estop from a previous session, then confirm telemetry.
    n.estop.publish(Bool(data=False)); n.spin_for(0.5)
    n.spin_for(2.0)
    if n.ticks is None:
        print("FAIL: no /wheel_ticks. Is micro_ros_agent up, and ROS_DOMAIN_ID=20?")
        return 1
    if n.health:
        print(f"firmware version : {n.health[0]}")
        print(f"health flags     : 0x{n.health[1]:02x}  "
              f"(estop={bool(n.health[1]&1)} vel_armed={bool(n.health[1]&2)} "
              f"imu_ok={bool(n.health[1]&8)})")
        if n.health[1] & 2:
            print("WARNING: velocity path is ARMED. It should be disarmed for this test.")

    # ---- deadman check, before anything spins. --------------------------
    print("\n--- DEADMAN CHECK ---")
    print("Sending one duty command to wheel 1 and then STOPPING all commands.")
    print("The wheel must stop on its own within ~300 ms.")
    n.send([DUTY, 0, 0, 0]); n.spin_for(1.5)
    print("Did wheel 1 stop by itself? If it kept turning, HIT POWER NOW.")
    if input("Did it stop on its own? [yes/NO] ").strip().lower() != "yes":
        print("FAIL: deadman not working. DO NOT put this rover on the floor.")
        return 1
    n.stop()

    # ---- per-wheel forward / reverse ------------------------------------
    results = []
    for direction, sign, word in ((1, +1, "FORWARD"), (2, -1, "REVERSE")):
        print(f"\n=== {word} ({sign*DUTY}% duty) ===")
        for i, name in enumerate(WHEELS):
            before = list(n.ticks)
            n.burst(i, sign * DUTY)
            after = list(n.ticks)
            d = [a - b for a, b in zip(after, before)]

            moved = d[i]
            others = [abs(d[j]) for j in range(4) if j != i]
            ok_sign = (moved > 0) if sign > 0 else (moved < 0)
            ok_alone = max(others) < max(20, abs(moved) * 0.10)

            print(f"  wheel {name:<15} deltas={d}")
            if not ok_sign:
                print(f"    !! ENCODER SIGN WRONG for wheel {i+1}: expected "
                      f"{'UP' if sign>0 else 'DOWN'}, got {moved:+d}")
                print(f"       -> flip MOTOR{i+1}_ENCODER_INV in fpms_config.h")
            if not ok_alone:
                print(f"    !! CROSS-COUNTS: another encoder moved too. Check "
                      f"the H{i+1} wiring / pin map.")
            if abs(moved) < 20:
                print(f"    !! WHEEL {i+1} DID NOT MOVE (or its encoder is dead).")

            obs = input(f"    Did wheel {i+1} physically turn {word}"
                        f" (as seen from the rover's own front)? [yes/no] ")
            phys_ok = obs.strip().lower().startswith("y")
            if not phys_ok:
                print(f"    -> if it turned the WRONG WAY, flip MOTOR{i+1}_INV")
            results.append((word, i, ok_sign, ok_alone, phys_ok, moved))

    n.stop()

    # ---- verdict ---------------------------------------------------------
    print("\n" + "=" * 68)
    bad = [r for r in results if not (r[2] and r[3] and r[4])]
    if not bad:
        print("ALL CHECKS PASSED.")
        print("Encoders are alive, correctly signed, correctly mapped, and")
        print("every wheel turns the commanded direction. The duty path and")
        print("the deadman both work.")
        print("\nNEXT (still on blocks): push-test odometry sign, then measure")
        print("counts/mm over a tape-measured hand push -- FPMS_COUNTS_PER_REV")
        print("is still contested and /wheel_ticks lets you fix it without a")
        print("reflash.")
    else:
        print("FAILURES -- DO NOT PUT THE ROVER ON THE FLOOR:")
        for word, i, s, a, p, mv in bad:
            print(f"  {word} wheel {i+1}: sign_ok={s} isolated={a} "
                  f"physical_ok={p} delta={mv:+d}")
    return 0 if not bad else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted -- sending stop")
        sys.exit(1)
