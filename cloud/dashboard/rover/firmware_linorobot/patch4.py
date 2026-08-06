"""Put the board's DDS participant in ROS domain 20.

In micro-ROS the CLIENT declares its domain in the CREATE_PARTICIPANT request --
the agent's own ROS_DOMAIN_ID does not decide it. Upstream calls plain
rclc_support_init(), which leaves the participant in domain 0, so the board
published perfectly into a domain nothing else on this rover listens to:
verified with `ROS_DOMAIN_ID=0` showing /fpms_drive with odom_raw, imu and
battery all at pub=1 while domain 20 saw pub=0.

Fixing it here rather than moving the rest of the stack to domain 0, which is
the unnamespaced default and would collide with anything else on the network.
"""
import pathlib
import sys

LINO = pathlib.Path("/home/ubuntu/lino")
fw = LINO / "firmware/src/firmware.cpp"
txt = fw.read_text()

if "rclc_support_init_with_options" in txt:
    print("firmware.cpp: already sets a domain")
    sys.exit(0)

old = "RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));"
if old not in txt:
    sys.exit("firmware.cpp: rclc_support_init call not found")

new = """rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, FPMS_ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));"""

txt = txt.replace(old, new, 1)
fw.write_text(txt)
print("firmware.cpp: participant now created in domain FPMS_ROS_DOMAIN_ID")

cfg = LINO / "config/custom/fpms_config.h"
ctxt = cfg.read_text()
if "FPMS_ROS_DOMAIN_ID" not in ctxt:
    anchor = "#define BAUDRATE 921600"
    ctxt = ctxt.replace(
        anchor,
        "/* The whole FPMS stack runs on domain 20; the board must join it\n"
        " * explicitly because the micro-ROS client, not the agent, picks the\n"
        " * domain. */\n"
        "#define FPMS_ROS_DOMAIN_ID 20\n\n" + anchor,
        1,
    )
    cfg.write_text(ctxt)
    print("fpms_config.h: FPMS_ROS_DOMAIN_ID 20")

print("PATCH4 OK")
