Custom FW for Haute42 S16.

### Input Latency
Average latency is `0.814 ms` measured using the methodology described on [inputlag.science](http://inputlag.science)

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
| LED Animations   | ⚠️    | Button illumination on press only.          |
| OLED Support     | ✅    |                                             |
| OLED Images      | ✅    |                                             |
| OLED Animations  | ✅    |                                             |
| Live config      | ⚠️    | LED/OLED toggle only.                       |
| Turbo            | ❌    |                                             |
