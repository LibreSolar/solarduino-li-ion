# Solarduino Li-Ion
Solar charge controller with integrated 12 V Li-Ion BMS

**Remark:** This charge controller is not maintained anymore and firmware + hardware might contain serious bugs. Use at your own risk!

Features so far: 

- 12 V LiFePO4 BMS (4 cells)
- 8A MPPT charger
- 50V max PV input
- 16V max. DCDC output = battery voltage
- Arduino-compatible (ATmega 328P used)
- Expandable via break-out of unused ports to standard 2.54 pin-pitch headers (e.g. used for display, communication like CAN/RS485, etc.)

Built-in protection:
- Overvoltage
- Undervoltage
- Overcurrent
- Overtemperature (via software)
- PV short circuit
- PV reverse polarity (via diode)
- Battery reverse polarity (destructive, fuse is blown)

# Firmware
The last version of the firmware based on Arduino can be found in the firmware directory.

# Known issues
- The current measurement using the INA168 does not work properly. The output should not be directly connected to the ADC, but should be buffed using an Op-amp in between.

# Roadmap
- My plan for the future is to separate the BMS system and the charge controller. Separate modules make the system more flexible. For a new, extended version of the BMS system see my other project here on github.
- A new version of the Solarduino charge controller will be capable of 12-24V and 12 A, thus just perfect for one large 60-cell PV panel in a 24V system.
