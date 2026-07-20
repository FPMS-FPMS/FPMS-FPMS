# FPMS — Fire Prevention & Management System

> **Protecting the past with the power of the present.**

Two small autonomous rovers keeping quiet watch over the land where cultural heritage sites meet the growing risk of wildfire.

Built by **Aryan Wadhawan** and **Alex Tang** — Grade 7, David Leeder Middle School, Toronto. Gold medal at WRO Canada Nationals 2026. Advancing to WRO International Finals, San Juan, Puerto Rico — December 2026.

---

## Table of contents

- [What FPMS is](#what-fpms-is)
- [The two halves of the mission](#the-two-halves-of-the-mission)
- [Why this matters — the cultural heart](#why-this-matters--the-cultural-heart)
- [How the rover sees](#how-the-rover-sees)
- [System architecture](#system-architecture)
- [Hardware](#hardware)
- [Software stack](#software-stack)
- [Honest limits](#honest-limits)
- [The team](#the-team)
- [Recognition & competition](#recognition--competition)
- [Repository structure](#repository-structure)
- [Acknowledgments](#acknowledgments)

---

## What FPMS is

Wildfires threaten more than trees. They threaten places that carry stories, ceremonies, and history that cannot be rebuilt if they burn.

FPMS is a system of two small autonomous rovers, a set of ground sensors, and a cloud-connected public dashboard. It watches a piece of land, notices when something is changing, and acts before a small problem becomes a large one — in two ways at once.

## The two halves of the mission

**🔥 Reactive** — *When something is already burning*

The rover's thermal and vision AI detect the hotspot. Both cameras must agree before it acts. It drives to the source, releases water from a small onboard pump, and logs every decision to the cloud.

**🌱 Proactive** — *Before anything is burning*

The same rover scans vegetation for dry brush, ground temperature, and fuel buildup — building a live risk map that can support Indigenous cultural burning practices. The community always makes the burn decisions.

## Why this matters — the cultural heart

Cultural heritage sites — sacred grounds, ancestral markers, places of ceremony — are irreplaceable. Once one burns, no amount of rebuilding brings back what it meant. FPMS is designed so the technology serves those places, not the other way around.

The rover:

- Documents every heritage marker it encounters — a photo, a written description, precise coordinates, a timestamp
- Builds a digital record that survives even if the physical site does not
- Does *not* decide what a site means — interpretation belongs to the communities whose sites they are
- Approaches to learn, not to extract — first-cup outreach to Indigenous leadership is part of the project, not an appendix

> *Indigenous fire stewardship is one of the oldest land management practices in North America. For thousands of years, small planned fires kept forests healthy and prevented the huge uncontrolled fires we see today. FPMS is a small attempt to build technology that **supports** those practices, not replaces them.*

## How the rover sees

The rover carries two cameras that see the world in fundamentally different ways:

| Camera | What it sees | What it's good at |
|---|---|---|
| **RGB camera** | Visible light, like human eyes | Recognizing shapes — trees, fire, ground features via YOLO vision AI |
| **Thermal camera** | Infrared heat directly | Detecting embers under leaves, heat behind smoke, hot ground before flames appear |

**Only when both cameras agree does the rover act.** This one requirement — cross-validated perception — is how a system built by two Grade 7 students avoids the false positives (shadows, sun-warmed rocks) that would otherwise waste water and undermine trust.

## System architecture

Four tiers, all designed to be inspectable and transparent:

```
┌─────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  I · FIELD  │───▶│ II·TRANSPORT │───▶│  III · CLOUD │───▶│  IV · PUBLIC │
├─────────────┤    ├──────────────┤    ├──────────────┤    ├──────────────┤
│ Rover 1     │    │ WiFi 6       │    │ AWS IoT Core │    │ Live map     │
│ Rover 2     │    │ MQTT/TLS     │    │ Lambda       │    │ Event log    │
│ Sensors     │    │ Batched      │    │ S3 archive   │    │ Heritage db  │
│ Refill dock │    │ State only   │    │ SNS alerts   │    │ Email/SMS    │
└─────────────┘    └──────────────┘    └──────────────┘    └──────────────┘
```

**Key design principle:** the system is event-based, not streaming. Video does not go to the cloud — only state changes do (a fire detected, a heritage marker documented, a mission completed). This keeps bandwidth minimal, storage predictable, and the public dashboard readable.

Everything the rover does is public by default. Trust in a system like this only works when what it does is visible.

## Hardware

### Rover 1 — Reactive suppression

| Component | Purpose |
|---|---|
| Radxa ROCK 5B+ (RK3588) | Main compute — Nav2, SLAM, YOLO26 inference on 6 TOPS NPU |
| Yahboom V3.0 motor board (STM32F103) | Motor control, encoder/IMU fusion |
| 4× Yahboom 520 motors, 86mm off-road wheels | Locomotion — carpet traction |
| LDROBOT D500 LiDAR | 2D SLAM mapping |
| HBV RGB camera | YOLO vision |
| Waveshare thermal camera (LWIR) | Heat detection |
| MG90S servo | Shared camera pan for heritage + thermal scanning |
| R385 pump | Water suppression |
| Yahboom 9600mAh 12V Li-ion battery | Main power |

### Rover 2 — Proactive documentation

Details in `/hardware/rover2/` — architecture in progress.

### Water refill station

XIAO ESP32-S3 + PCA9685 + 2× MG996R servos + 6V NiMH pack — autonomous refill with ArUco docking alignment.

## Software stack

- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS2 Humble + Cyclone DDS
- **Navigation:** Nav2 (composed mode), SLAM Toolbox, `ldlidar_ros2`
- **Perception:** YOLO26 via Rockchip RKNN toolkit (FP16, 15+ FPS measured)
- **Sensor fusion:** EKF via `robot_localization` (encoders + IMU)
- **Cloud:** AWS IoT Core (MQTT/TLS), S3 (archive), Lambda + SNS (alerts)
- **Dashboard:** FastAPI + SQLite + Foxglove Studio + Chromium kiosk
- **Voice announcements:** Piper TTS (offline)

Full deployment details in `/docs/`.

## Honest limits

FPMS is a competition prototype built by two Grade 7 students. A small robot cannot replace a fire crew, community fire knowledge, or the professional systems that already exist to protect land and life.

What we are trying to show is a smaller thing — a working example of a bigger idea. That technology, when it is built respectfully, kept transparent to the people it affects, and offered without any claim to authority, can contribute in modest ways to protecting places that matter.

Whether this idea grows into anything larger depends not on us. It depends on the communities whose land it would serve, and whether they see anything worth building on here. Our part, right now, is to listen.

## The team

**Aryan Wadhawan** — Systems, programming, cultural outreach lead. Nav2 integration, cloud/AWS pipeline, first-cup outreach with Indigenous leadership.

**Alex Tang** — Hardware, mechanical design, integration lead. Chassis CAD, wiring architecture, servo/actuator subsystems, assembly.

Both team members are cross-trained on the full stack for judge Q&A. Both speak during the pitch. Both are credited on every deliverable.

## Recognition & competition

🥇 **Gold, WRO Canada Nationals 2026** — Montreal, May 30, 2026

🌎 **Advancing to WRO International Finals** — San Juan, Puerto Rico, December 2026

📚 **Category:** Future Innovators, Junior · Theme: *Robots Meet Culture*

## Repository structure

```
fpms/
├── README.md              ← you are here
├── docs/                  ← engineering documentation, design decisions
│   ├── architecture.md
│   ├── hardware-bom.md
│   └── outreach-notes.md
├── rover1/                ← reactive suppression rover
│   ├── firmware/          ← ESP32 bridge, V3.0 motor firmware
│   ├── ros2_ws/           ← ROS2 packages, Nav2 config, behavior tree
│   ├── perception/        ← YOLO26 RKNN deployment, thermal fusion
│   └── cad/               ← chassis STL files
├── rover2/                ← proactive documentation rover (in progress)
├── station/               ← water refill station
│   └── firmware/          ← ESP32-S3 pour sequence controller
├── cloud/                 ← AWS deployment
│   ├── iot-core/          ← MQTT topic setup, certificates
│   ├── lambda/            ← event routing, alert dispatch
│   └── dashboard/         ← FastAPI + public frontend
└── media/                 ← photos, video, explainer materials
```

## Acknowledgments

To the Indigenous fire stewards whose thousands of years of practice inform the proactive half of this project, and to the communities we hope to learn from.

To Deputy Grand Chief Mike Metatawabin of Nishnawbe Aski Nation, for engaging with a first-cup outreach from a Grade 7 student.

To our teachers, mentors, and families at David Leeder Middle School.

To the open-source robotics community — Nav2, ROS2, RKNN, Ultralytics, `ldlidar_ros2` — for the work we build on.

---

*Think globally. Act locally. Save culture.*

**Contact:** open an issue on this repo, or reach out via the [WRO Canada team directory](https://wrocanada.ca).

