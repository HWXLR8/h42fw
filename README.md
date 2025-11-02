Custom FW for Haute42 leverless gamepads. The goals of this project are:
1. Minimal latency.
2. Support for all HW features on Haute42 (all buttons/LEDs/OLED).
3. Simple user configuration.

### Feature list

| Feature          | State | Notes                                       |
|:-----------------|:-----:|:--------------------------------------------|
| HID Gamepad      | ✅    | Tested on Linux and Windows (DirectInput).  |
| HID Keyboard     | ❌    |                                             |
| XInput           | ❌    |                                             |
| Console Support  | ❌    |                                             |
| 1000Hz Polling   | ✅    |                                             |
| Button Remap     | ✅    |                                             |
| Button Debounce  | ✅    | Leading-edge, configurable per-button.      |
| SOCD Cleaning    | ⚠️    | Neutral/Last-input only.                    |
| LED Support      | ✅    | Per-button LED config for idle/press color. |
| LED Brightness   | ⚠️    | Only on/off toggle.                         |
| LED Animations   | ⚠️    | Button illumination on press.               |
| OLED Support     | ✅    | Text rendering only.                        |
| OLED Images      | ❌    |                                             |
| OLED Animations  | ❌    |                                             |
