# Todo

## Firmware

### Shared
- Test basic hardware
    - [x] Debug LED on PCB near MCU
    - [x] Status LED to indicate ready, error, etc.
    - [x] Status LED to indicate SSR status. Typically same state as SSR_EN
    - [x] SSR output
        - [x] One shot timer to trigger fixed length pulse under hardware control
    - [x] Misc output (fans etc)
    - [x] Rotary encoder
    - [x] NTC thermister
    - [x] Zero crossing detection circuit
    - [x] Trigger
    - [x] I2C display
    - [ ] USB
    - [x] Thermocouple

## Hardware
- [x] Add hardware debouncing to trigger
- [x] Add pull-up to SCL
- [x] Make I2C pull ups optional since adafruit module already includes them
- [x] Make I2C pull ups 2.2k since 3.3v not 5v