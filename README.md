# Ember

Ember is an autonomous first-response fire suppression robot that detects flames in real time and neutralizes them before they escalate. It uses local machine learning for fire detection and a servo-actuated extinguisher mechanism for immediate response.

## Inspiration

Every year, structural fires cause devastating damage before emergency services can arrive. We wanted to build a proactive on-site first responder that could bridge the gap between detection and professional intervention.

One major inspiration for Ember was the Wang Fuk Court fire in Tai Po, Hong Kong. In dense urban environments, fires can spread through scaffolding and construction materials faster than human responders can act. Ember is designed to serve as a zero-hour responder: a device that lives on-site and reacts the moment a flame appears.

## What It Does

Ember autonomously patrols high-risk areas and monitors its surroundings for signs of fire.

Using an Intel RealSense camera and an on-device machine learning pipeline, it detects visual signatures of flames. Once a fire is identified, Ember:

- stops its patrol
- moves toward the target area
- positions itself at an effective distance
- triggers its onboard fire extinguisher using a servo-based actuation system

The goal is to suppress a flame at the moment of detection, before it becomes a larger emergency.

## How We Built It

### NVIDIA Jetson Nano
The Jetson Nano serves as Ember’s main compute unit. It handles:
- machine learning inference
- fire detection logic
- high-level control decisions

All inference runs locally, allowing low-latency performance without relying on the cloud.

### Intel RealSense Camera
The RealSense camera provides:
- RGB visual input
- depth sensing
- better environmental awareness for positioning and targeting

### Arduino
The Arduino is responsible for:
- motor control
- servo actuation
- low-level hardware coordination

This setup lets the Jetson focus on vision and decision-making, while the Arduino handles real-time physical control.

### Fire Detection Model
We trained and optimized a machine learning model on fire and smoke data so that Ember could detect flames directly on edge hardware.

## Challenges We Ran Into

One of the biggest challenges was the suppression mechanism itself. Pressing down a real fire extinguisher handle requires substantial force, so we had to iterate on the lever-arm design and servo setup to generate enough torque without damaging the hardware.

We also had to improve serial communication between the Jetson and Arduino to ensure movement and suppression commands were executed reliably during detection.

## Accomplishments We’re Proud Of

- Fully local fire detection and decision-making
- Edge ML running directly on the Jetson Nano
- Real-time hardware response through Arduino and servos
- A working autonomous system that combines vision, movement, and suppression

One of the most important outcomes was making Ember independent of cloud connectivity. Even if a building’s internet infrastructure fails, Ember can still operate on-site.

## What We Learned

Building Ember taught us a lot about:
- hardware-in-the-loop testing
- optimizing computer vision for edge devices
- coordinating ML systems with real-world robotics
- torque, leverage, and mechanical reliability in actuation systems

## What’s Next

We plan to expand Ember with:

- **SLAM** for navigation in unfamiliar environments
- **thermal imaging** to detect heat signatures through smoke or behind obstacles
- improved targeting and mobility
- better response planning for more complex indoor settings

## Tech Stack

- Arduino
- NVIDIA Jetson Nano
- Python
- Intel RealSense
- Servo Motors
- Motor Drivers
- Embedded / Robotics Control Systems

## Built For

HackPrinceton Spring 2026

## Team

- Sai Guvvala
- Julie Yiu
- Adhyann Singal
- Satchit Seth

## Vision

Ember is built around a simple idea: fire response should begin the instant danger appears, not minutes later. By combining edge AI, robotics, and autonomous suppression, we hope to make buildings safer with an always-present first responder.
