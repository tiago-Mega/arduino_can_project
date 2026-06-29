# CAN Bus PlatformIO Commands

## Upload Commands

### Upload Transmitter to a Specific Port
```bash
pio run -e transmitter --target upload --upload-port COM8
```

### Upload Receiver to a Specific Port
```bash
pio run -e receiver --target upload --upload-port COM8
```

### Upload Using Ports Defined in platformio.ini
```bash
# Transmitter (uses upload_port from platformio.ini)
pio run -e transmitter --target upload

# Receiver (uses upload_port from platformio.ini)
pio run -e receiver --target upload
```

---

## Monitor Commands

### Monitor by Environment (uses monitor_port from platformio.ini)
```bash
# Transmitter (COM8 by default)
pio device monitor -e transmitter

# Receiver (COM9 by default)
pio device monitor -e receiver
```

### Monitor with Explicit Port & Baud
```bash
# Transmitter
pio device monitor --port COM8 --baud 9600

# Receiver
pio device monitor --port COM9 --baud 9600
```

> **Tip:** Open two separate terminals to monitor both nodes simultaneously.

---

## List Available COM Ports
```bash
pio device list
```
Plug in one Arduino at a time and run this command to identify which COM port belongs to which board.

---

## platformio.ini Configuration (with explicit upload ports)

```ini
[env:transmitter]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 9600
monitor_port = COM8
upload_port = COM8
build_flags = -DNODE_MODE=TRANSMITTER

[env:receiver]
platform = atmelavr
board = nanoatmega328new
framework = arduino
monitor_speed = 9600
monitor_port = COM9
upload_port = COM9
build_flags = -DNODE_MODE=RECEIVER
```

---

## Quick Reference

| Action | Command |
|--------|---------|
| Upload transmitter | `pio run -e transmitter --target upload` |
| Upload receiver | `pio run -e receiver --target upload` |
| Upload to specific port | `pio run -e <env> --target upload --upload-port <PORT>` |
| Monitor transmitter | `pio device monitor -e transmitter` |
| Monitor receiver | `pio device monitor -e receiver` |
| List COM ports | `pio device list` |

> **Linux/macOS note:** Replace `COMx` with `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.
