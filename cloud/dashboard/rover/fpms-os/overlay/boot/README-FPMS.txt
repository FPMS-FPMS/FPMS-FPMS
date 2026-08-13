========================================================================
 FPMS-OS  -  set up your rover before you switch it on
========================================================================

            THIS CARD IS  ROVER 1.   Its name is  fpms-rover1
            You reach it at  http://fpms-rover1.local:8090/

If you have a second rover, it is a DIFFERENT card with a different name, and
the two must never be given the same one. See section 2.

You are looking at the boot partition of an FPMS rover's memory card. You can
edit these files from Windows, Mac or Linux with any text editor (Notepad is
fine). Everything here is read ONCE, the first time the rover boots.

If you do nothing at all, this card is already set up for the competition
network  FPMS_Net  and should come up on it by itself. You only need the
files below if you are somewhere else, or something is wrong.


------------------------------------------------------------------------
 1. WIFI  ->  already set up; edit  fpms-wifi.conf  to change it
------------------------------------------------------------------------

This rover ALREADY KNOWS one network:

    FPMS_Net

You do not have to do anything to use it. Nobody has watched this particular
card join that network yet, though, so if it comes up on its own fallback
network instead (section 4), the first thing to check is that FPMS_Net is
actually on the air and spelled that way.

TO ADD OR PREFER A DIFFERENT NETWORK - for example at home, or on a laptop
hotspot - rename  fpms-wifi.conf.example  to  fpms-wifi.conf  and put your
networks in it, one per line, like this:

    MyHotspot:mypassword123
    HomeWiFi:anotherpassword

ORDER MATTERS. The first line is tried first. Anything you write here is
tried BEFORE the built-in FPMS_Net, so this file always wins - it adds to the
built-in network, it does not have to fight it.

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
    One line, the rover's name. This card is  fpms-rover1 , which means you
    reach it at  http://fpms-rover1.local:8090/
    Leave this file out unless you are deliberately renaming the rover.
    NEVER give two rovers the same name. They do not simply share it - one of
    them gets quietly renamed to something like fpms-rover1-2 by the network
    itself, and from then on you cannot tell which rover you are driving.
    The rover's identity ALSO includes the name its telemetry travels under,
    which for this card is  rover1 . That one is not on this partition; it is
    set inside the image and the dashboard has to be pointed at the same word
    or it will show you an empty screen and no error at all.

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

    http://fpms-rover1.local:8090/

ALWAYS TYPE THE NAME, NOT AN IP ADDRESS. The rover's IP address changes; the
name does not. This project has had that bite it more than seven times.

If that address does not open, older printouts, shortcuts and notes for this
project all say  fpms-rover1.local . That was the single-rover name and it is no
longer this rover. Nothing forwards the old name to the new one.

The rover checks itself about 90 seconds after boot. To see the result:

    ssh ubuntu@fpms-rover1.local
    fpms-selftest

If something is wrong, it tells you what and what to do about it.


------------------------------------------------------------------------
 4. IF IT DOES NOT COME UP ON WIFI
------------------------------------------------------------------------

If FPMS_Net is not in range - or is not spelled the way this card expects -
the rover starts its own network so you can still reach it:

    network:  FPMS-Rover-Setup
    password: fpmsrover

Join that from a laptop or phone, then open  http://fpms-rover1.local:8090/
or  ssh ubuntu@10.42.0.1

Seeing this network at a venue that HAS working WiFi is the symptom of a
wrong SSID or password, not of broken WiFi hardware.

An Ethernet cable also always works.


------------------------------------------------------------------------
 5. A SAFETY NOTE, BECAUSE IT MATTERS
------------------------------------------------------------------------

This rover will not drive itself the moment it boots. Missions must be armed
deliberately from the console, and the STOP button is handled by a separate
process with its own connection so it cannot be queued behind anything else.

Do not go looking for a way to make it start driving automatically at boot.
That is deliberately not a feature.
