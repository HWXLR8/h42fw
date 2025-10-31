Custom FW for Haute42 leverless gamepads. The goals of this project are:
1. Minimal latency.
2. Support for all HW features on Haute42 (all buttons/LEDs/OLED).
3. Simple user configuration.

### Feature list

| Feature          | State | Notes                                  |
|:-----------------|:-----:|:---------------------------------------|
| HID Gamepad      | ✅    | Tested on Linux.                       |
| HID Keyboard     | ❌    |                                        |
| XInput           | ❌    |                                        |
| 1000Hz Polling   | ✅    |                                        |
| Button Remap     | ✅    |                                        |
| Button Debounce  | ✅    | Leading-edge, configurable per-button. |
| SOCD Cleaning    | ⚠️    | Only Neutral/Last-input.               |
| LED Support      | ✅    | Per-button LED config.                 |
| LED Brightness   | ⚠️    | Only on/off toggle.                    |
| LED Animations   | ❌    |                                        |
| OLED Support     | ❌    |                                        |
| Console Support  | ❌    |                                        |
