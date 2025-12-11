# Turnstile-Drive-Control-Unit Based on STM32F103C8T6
A complete motor controller designed to drive a 12 V DC brushed motor for industrial-grade control, including electrical protection, noise filtering, and reliable communication through a differential interface

## Description
This project implements a DC brushed motor controller for a 12 V / 24 W motor with nominal current up to 4 A and speed up to 3000 rpm. 
The controller is built around an STM32F103C8T6 microcontroller and a full H-bridge power stage with MOSFET drivers, current sensing, RS-485 communication, and external EEPROM for statistics logging.  

This repository contains the hardware design of a DC motor controller for a turnstile application
(based on STM32F103 and an H-bridge power stage). The focus is on the schematic, components
selection, and interfaces.

The controller is intended for use in industrial or automation systems where the motor may be started, stopped, reversed and speed-controlled via PWM and an RS-485 interface. Additional discrete inputs support an open-collector signal and a dry contact (e.g. limit switch).

---

## Project Status

- Hardware schematic: **complete (v1.0)**
- Firmware: **not implemented yet**
- Turnstile control algorithm: **high-level concept only, no production code yet**
  
---

## Table of Contents

- [Description](#description)
- [Features](#features)
- [Hardware](#hardware)
  - [Components Overview](#components-overview)
  - [Power Stage (H-Bridge)](#power-stage-h-bridge)
  - [Interfaces](#interfaces)
    - [RS-485](#rs-485)
    - [I2C EEPROM](#i2c-eeprom)
    - [Discrete Inputs](#discrete-inputs)
- [Control Algorithm](#control-algorithm)
  - [1. System Initialization](#1-system-initialization)
  - [2. Motor Command Handling](#2-motor-command-handling)
  - [3. PWM and Direction Control](#3-pwm-and-direction-control)
  - [4. Current Measurement and Protection](#4-current-measurement-and-protection)
  - [5. RS-485 Communication](#5-rs-485-communication)
  - [6. EEPROM Logging](#6-eeprom-logging)
  - [7. Status Monitoring and Fault Handling](#7-status-monitoring-and-fault-handling)
- [Future Improvements](#future-improvements)
- [Conclusions](#conclusions)

---

## Features

- 12 V DC supply, up to 4 A motor current
- Full H-bridge power stage with high-side/low-side MOSFETs
- Direction and speed control via PWM from STM32
- Current measurement on shunt resistor with ADC feedback
- Over-current protection and fault handling in firmware
- RS-485 half-duplex interface for networking multiple controllers
- Open-collector digital input (optically clean, via 2N7002)
- Dry-contact digital input with debouncing and protection
- External I²C EEPROM (AT24C512) for logging statistics and events
- Status LEDs for power, communication and fault indication
- On-board 3.3 V buck regulator (TPS563201) from 12 V bus supply

---
Main components used on the board: 


## Hardware

### EasyEDA Full Scheme

<img width="1200" height="825" alt="SCH_Schematic1_1-P1_2025-12-11" src="https://github.com/user-attachments/assets/a4b28d90-d203-407c-b6be-16cea79da516" />


### Components Overview

Main components used on the board: 

- **MCU:** STM32F103C8T6  
  - 72 MHz ARM Cortex-M3  
  - ADC, timers with PWM, USART, I²C, GPIO  
- **Motor Power Stage:**
  - 2 × IR2104 high/low-side gate drivers
  - 4 × N-channel MOSFETs (IRF540 or logic-level equivalent) forming H-bridge
  - Shunt resistor 0.2 Ω / 5 W in low side for current measurement
  - Flyback/commutation diodes (Schottky 1N5819) across MOSFETs and motor
- **Power Supply:**
  - TPS563201 step-down converter from +12 V to +3.3 V
  - Input/output LC filtering and bulk capacitors
- **Interfaces & Logic:**
  - RS-485 transceiver with termination and bias resistors
  - I²C EEPROM AT24C512
  - 2N7002 transistor stage for open-collector input
  - Discrete circuit for dry-contact input
- **Clock & Misc:**
  - 8 MHz crystal + load capacitors for STM32
  - Multiple decoupling capacitors on all supply rails
  - Status LEDs for supply and controller state

---

### Power Stage (H-Bridge)

<img width="655" height="373" alt="image" src="https://github.com/user-attachments/assets/b7892c23-f511-475d-98af-78485c5598b2" />


The motor is driven by a **full H-bridge** built from four N-channel MOSFETs (Q1–Q4). Each side of the bridge is controlled by an **IR2104** driver:

- **High-side MOSFETs** are driven via bootstrap circuitry (VB, VS, HO pins of IR2104) with bootstrap capacitors and diodes.
- **Low-side MOSFETs** are driven via LO outputs referenced to COM (system ground).
- PWM is applied to the driver inputs from a dedicated STM32 timer channel:
  - One PWM channel (PWM1) controls one leg of the bridge.
  - The second PWM channel (PWM2) controls the opposite leg (for direction control and synchronous rectification if required).
- A **shunt resistor (0.2 Ω, 5 W)** in series with the motor return path generates a voltage proportional to motor current.
  - The shunt voltage is filtered and routed to an MCU ADC input.

Protective elements:

- **Schottky diodes (1N5819)** clamp inductive voltage spikes during commutation.
- Bulk electrolytic capacitor (≥470 µF) across the 12 V bus stabilizes the supply under dynamic load.
- Optional RC snubbers can be added across MOSFETs / motor terminals to reduce EMI.

---

### Interfaces

#### RS-485

<img width="346" height="188" alt="image" src="https://github.com/user-attachments/assets/cc10441b-a016-43d9-a1de-c144dfb0084f" />


- RS-485 transceiver provides half-duplex communication over differential pair A/B.
- Signals:
  - `RS485_TX` / `RS485_RX` connected to USART of STM32
  - `RS485_MD` (DE/RE) controls direction (transmit/receive)
- Hardware features:
  - 120 Ω termination resistor between A and B
  - Bias resistors to define bus idle state
  - Local decoupling capacitor and optional TVS protection

This interface is intended for connection to a supervisory controller, PLC, or PC via RS-485 dongle.

#### I²C EEPROM

<img width="327" height="180" alt="image" src="https://github.com/user-attachments/assets/f1844497-8af8-400b-81f8-86d14afbf3b8" />


- AT24C512 (U4) is connected to the MCU via `MEM_SCL` / `MEM_SDA` lines (I²C bus).
- Pull-up resistors on SCL and SDA ensure correct bus operation.
- Used to store:
  - Lifetime statistics (run time, number of starts, fault counters)
  - Configuration parameters (current limit, acceleration ramp, etc.)

#### Discrete Inputs

<img width="307" height="300" alt="image" src="https://github.com/user-attachments/assets/6d7379fe-f342-4554-9f9b-217ef16ed854" />

- **Open-collector output (IC_OC / OPEN_C):**
  - Implemented via 2N7002 transistor, pulled up to 3.3 V with series/current-limit resistor.
  - Can drive external IC, open-collector devices, optocouplers, or NPN outputs.
- **Dry-contact input (DRY_C / IC_DRY_C):**
  - Intended for switching with mechanical contacts (limit switches, emergency stop, etc.).
  - Includes pull-up and protection diode; can be extended with RC debouncing.

---

## Proposed Control Algorithm

Below is the high-level structure of the firmware algorithm, broken into functional blocks.

### 1. System Initialization

1. Configure system clock:
   - HSE 8 MHz + PLL to 72 MHz core.
2. Initialize peripherals:
   - GPIO directions and default states for all control pins (PWM, enables, direction, DE/RE, inputs).
   - Timer(s) for PWM generation (center-aligned or edge-aligned, e.g., 20–25 kHz).
   - ADC channel for current sense (shunt).
   - USART for RS-485 (baud rate, frame format).
   - I²C peripheral for EEPROM access.
3. Enable interrupts:
   - Timer update and/or capture-compare interrupts if required for dead-time/step control.
   - USART RX interrupt for receiving RS-485 frames.
   - Optional external interrupts for discrete inputs.

### 2. Motor Command Handling

Motor operation is controlled via **command state machine** driven mainly by RS-485 messages and discrete inputs.

**States:**
- `IDLE` – motor off, PWM = 0.
- `RUN_FWD` – motor running forward.
- `RUN_REV` – motor running reverse.
- `FAULT` – motor disabled due to error (over-current, communication timeout, etc.).

**Logic:**

1. On reception of a valid command frame (e.g., `SET_SPEED`, `SET_DIRECTION`, `START`, `STOP`):
   - Validate CRC/checksum.
   - Update desired direction and speed setpoint.
2. Discrete inputs:
   - **Dry contact** can be mapped to `Emergency Stop` or `Limit Switch`.
   - **Open-collector** input may be used for external enable, direction override, or mode selection.

### 3. PWM and Direction Control

1. Direction is typically determined by which diagonal of the H-bridge is active.
   - For **forward**:
     - High-side of left leg + low-side of right leg are PWM-driven.
   - For **reverse**:
     - High-side of right leg + low-side of left leg are PWM-driven.
2. To avoid shoot-through:
   - Use timer features (dead-time insertion) or firmware sequencing when switching direction:
     - Disable PWM outputs.
     - Wait for MOSFETs to turn off (few µs).
     - Reconfigure active legs.
     - Re-enable PWM.

3. Speed control:
   - Speed setpoint is mapped to PWM duty cycle (0–100%).
   - Optional soft-start:
     - Ramp duty cycle from 0 to target over configurable time (e.g., 100–500 ms).
   - Optional soft-stop:
     - Gradually decrease duty cycle to 0.

### 4. Current Measurement and Protection

1. ADC samples shunt voltage periodically (e.g., in PWM timer interrupt or at fixed rate).
2. Conversion to current:
   - `I_motor = V_shunt / R_shunt`.
3. Protection logic:
   - If current > **warning threshold**:
     - Limit PWM duty (current limiting mode).
   - If current > **trip threshold** for N consecutive samples:
     - Enter `FAULT` state:
       - Disable all PWM outputs / IR2104 enables.
       - Set fault LED.
       - Optionally send fault frame over RS-485.

4. Optional averaging / low-pass filtering is implemented in software to reduce noise.

### 5. RS-485 Communication

1. **Receive path:**
   - USART RX interrupt collects bytes into a buffer.
   - On end of frame (timeout or special byte):
     - Parse message, validate CRC.
     - Update configuration/state accordingly.
2. **Transmit path:**
   - To send a response:
     - Set RS485 driver to transmit (DE=1, RE=1).
     - Send bytes via USART.
     - On TX complete interrupt, return to receive mode (DE=0, RE=0).

3. Example functions:
   - `CMD_GET_STATUS` – returns current, state, error flags.
   - `CMD_SET_SPEED` – sets PWM duty.
   - `CMD_SET_LIMITS` – configures current limits, ramps, etc.

### 6. EEPROM Logging

1. On significant events (start, stop, fault, parameter change):
   - Append record to EEPROM:
     - Timestamp / counter
     - Event code
     - Context (e.g., measured current)
2. On power-up:
   - Read configuration block from EEPROM.
   - Restore previous parameters and counters.
3. Simple wear-leveling can be applied by marking active configuration blocks with version numbers or flags.

### 7. Status Monitoring and Fault Handling

1. Main loop periodically:
   - Checks for communication timeout.
   - Reads discrete inputs and debounces them.
   - Updates status LEDs:
     - Different blink patterns for `RUN_FWD`, `RUN_REV`, `IDLE`, `FAULT`.
2. When a fault is cleared (e.g., via RS-485 command or input reset):
   - Re-initialize power stage if necessary.
   - Return to `IDLE` or commanded state.

---

## Future Improvements

Potential enhancements for future revisions:

- Replace IRF540 with **logic-level MOSFETs** with lower Rds(on) to reduce heating and losses.
- Add **galvanic isolation** for RS-485 and/or control inputs for harsh industrial environments.
- Implement **closed-loop speed control** using encoder feedback (PI controller).
- Add **temperature sensing** on MOSFETs or PCB to protect against thermal overload.
- Support **multi-drop addressing** and more complex RS-485 protocol (Modbus RTU, custom binary protocol, etc.).
- Extend EEPROM usage for:
  - Detailed runtime logs,
  - Service diagnostics (number of over-current trips, hours in each direction).
- Create a **PC configuration tool** for parameter tuning and firmware updates.
- Design a dedicated **enclosure + connector pinout** and document mechanical integration.

---

## Conclusions

This project demonstrates a complete DC motor control solution based on an STM32F103C8T6 microcontroller and a discrete H-bridge power stage. The hardware integrates power conversion, current measurement, RS-485 communication, and non-volatile logging, making it suitable for use in industrial or automation systems where robust control and monitoring are required.  

The modular structure of the firmware (separate blocks for PWM, direction, protection, communication and logging) makes the controller adaptable to different motors and system requirements, and the GitHub repository layout is intended to support future hardware and software evolution.
