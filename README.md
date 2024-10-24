# Embedded Systems Lab 4: BLE Communication with STM32 and RPi

This repository contains the solution to **Lab 4: BLE Communication between STM32 and Raspberry Pi**. The project demonstrates a **BLE-based GATT service** with real-time accelerator data exchange between an **STM32 IoT node (BLE Server)** and a **Raspberry Pi 3 (BLE Client)**.

---

## Problem Statement

In this lab, the **STM32 IoT node** acts as a **BLE Server** and provides a **GATT Accelerator Service** with two main characteristics:  

1. **3-axis acceleration values** (GATT characteristic_a)  
2. **Accelerator sampling frequency** (GATT characteristic_b)  

The **Raspberry Pi 3** acts as a **BLE Client** and performs the following operations:  
- **Assign a sampling frequency** to the STM32 by writing to **GATT characteristic_b**.
- **Receive notifications** from the STM32 whenever the 3-axis acceleration values are updated in **GATT characteristic_a**.

### STM32 Setup Requirements:
1. **Real acceleration data** must be sampled using the **LSM6DSM sensor** (no simulated values allowed).  
2. Use **CMSIS-RTOS v2** to manage BLE and sensor tasks. Implement the following two tasks:
   - **TASK_BLE**:  
     Handles BLE operations, including:
     - GAP connection and disconnection  
     - GATT write requests from the RPi  
     - Event handling for BLE notifications  
     
   - **TASK_ACC**:  
     Manages the accelerator, including:
     - Sampling acceleration data from the LSM6DSM with the configured frequency  
     - Sending notifications with updated data to the RPi client
