========================================================================
 FPMS-OS  -  set up your rover before you switch it on
========================================================================

You are looking at the boot partition of an FPMS rover's memory card. You can
edit these files from Windows, Mac or Linux with any text editor (Notepad is
fine). Everything here is read ONCE, the first time the rover boots.

If you do nothing at all, the rover still boots. It just will not be able to
join your WiFi, so you would need to plug in an Ethernet cable or connect to
the rover's own fallback network. Two minutes now saves that.


------------------------------------------------------------------------
 1. WIFI  ->  edit  fpms-wifi.conf
------------------------------------------------------------------------

Rename  fpms-wifi.conf.example  to  fpms-wifi.conf  and put your networks in
it, one per line, like this:

    MyHotspot:mypassword123
    HomeWiFi:anotherpassword

ORDER MATTERS. The first line is tried first.

Put your WINDOWS MOBILE HOTSPOT FIRST if you use one. That is what the rover
expects the operator laptop to be on, and the whole telemetry link is
configured around it.

If a network has no password, leave the part after the colon empty:

    OpenNetwork:

After the rover reads this file it renames it to fpms-wifi.conf.applied and
locks it down, so your password is not sitting in plain text on a card that
any computer will happily mount. To change networks later, rename it back and
reboot.


------------------------------------------------------------------------
 2. OPTIONAL FILES  -  only if you need them
------------------------------------------------------------------------

fpms-hostname
    One line, the rover's name. Defaults to  fpms-pi , which means you reach
    it at  http://fpms-pi.local:8090/
    Change this ONLY if you are running two rovers at once, otherwise both
    will answer to the same name and confuse each other.

fpms-broker-host
    One line, the IP address of the operator laptop, e.g.  192.168.137.1
    The rover sends all its telemetry there. The default is already
    192.168.137.1, which is what a Windows Mobile Hotspot uses, so most
    people do not need this file.

fpms-mqtt-password
    One line. Only if you want this rover to use a password you already have.
    Leave it out and the rover generates its own strong one and prints it on
    screen during the first boot (and stores it on the rover, readable by an
    administrator).


------------------------------------------------------------------------
 3. WHAT HAPPENS WHEN YOU POWER ON
------------------------------------------------------------------------

First boot takes a few minutes, because the rover is resizing its own storage,
generating its security keys, and connecting to WiFi. Be patient - do not pull
the power.

After that, every boot is fast.

Then, from a laptop on the same network, open:

    http://fpms-pi.local:8090/

ALWAYS TYPE THE NAME, NOT AN IP ADDRESS. The rover's IP address changes; the
name does not. This project has had that bite it more than seven times.

The rover checks itself about 90 seconds after boot. To see the result:

    ssh ubuntu@fpms-pi.local
    fpms-selftest

If something is wrong, it tells you what and what to do about it.


------------------------------------------------------------------------
 4. IF IT DOES NOT COME UP ON WIFI
------------------------------------------------------------------------

The rover starts its own network so you can still reach it:

    network:  FPMS-Rover-Setup
    password: fpmsrover

Join that from a laptop or phone, then open  http://fpms-pi.local:8090/
or  ssh ubuntu@10.42.0.1

An Ethernet cable also always works.


------------------------------------------------------------------------
 5. A SAFETY NOTE, BECAUSE IT MATTERS
------------------------------------------------------------------------

This rover will not drive itself the moment it boots. Missions must be armed
deliberately from the console, and the STOP button is handled by a separate
process with its own connection so it cannot be queued behind anything else.

Do not go looking for a way to make it start driving automatically at boot.
That is deliberately not a feature.
