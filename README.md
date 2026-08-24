# Automated IoT Pill Dispenser
> Metropolia UAS | Embedded IoT Systems Project (Milla Juote and Matleena Vaara, 2025)

An automated, IoT-enabled medical pill dispenser built to schedule daily medication, physically verify dosage drops, and log device state over a LoRaWAN network.

## What We Did & Built

* **Hardware Assembly:** Integrated a custom PCB, stepper motor driver, and 3D-printed 8-compartment rotating carousel.
* **Optical Position Calibration:** Programmed an optical sensor (opto fork) to detect the calibration slot and home the wheel on startup.
* **Physical Dispense Verification:** Connected a piezoelectric sensor on the drop chute to capture falling-edge signals when a pill drops, triggering LED alerts if a pill is missing.
* **State Memory (I²C EEPROM):** Saved device state to non-volatile memory so remaining pill counts and logs persist across power losses.
* **LoRaWAN Telemetry:** Implemented UART AT-command routines to send wireless event logs (boot, dispense success/fail, dispenser empty) to a remote server.

## Hardware Pinout (RP2040)

* `GP28` -> Opto Fork Sensor (Input, Pull-Up)
* `GP27` -> Piezo Electric Sensor (Input, Pull-Up)
* `GP2, GP3, GP6, GP13` -> Stepper Motor Driver (IN1–IN4)
* `GP20, GP21, GP22` -> Status LEDs
* `GP4, GP5` -> LoRaWAN Module (UART1)
