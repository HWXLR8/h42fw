Custom FW for Haute42 S16.

### Input Latency
Average latency is `0.814 ms` measured using the methodology described on [inputlag.science](http://inputlag.science)

### Feature list

| Feature              | State | Notes                                       |
|:---------------------|:-----:|:--------------------------------------------|
| HID Gamepad          | ✅    | Tested on Linux and Windows (DirectInput).  |
| HID Keyboard         | ❌    |                                             |
| XInput               | ❌    |                                             |
| 1000Hz Polling       | ✅    |                                             |
| Button Remap         | ✅    |                                             |
| Button Debounce      | ✅    | Leading-edge, configurable per-button.      |
| SOCD Cleaning        | ⚠️    | Neutral/Last-input only.                    |
| LED Support          | ✅    | Per-button LED config for idle/press color. |
| LED Brightness       | ⚠️    | On/off toggle only.                         |
| LED Animations       | ⚠️    | Button illumination on press only.          |
| OLED Support         | ✅    |                                             |
| OLED Images          | ✅    |                                             |
| OLED Animations      | ✅    |                                             |
| Live config          | ⚠️    | Turbo, LED/OLED toggle only.                |
| Persistent Settings  | ⚠️    | Does not save turbo settings.               |
| Turbo                | ✅    | 15.6/31.25/62.5Hz configurable per button.  |

| Platfrom Support     | State | Notes                                       |
|:---------------------|:-----:|:--------------------------------------------|
| Linux                | ✅    |                                             |
| Windows              | ⚠️    | DirectInput only.                           |
| MacOS                | ⚠️    | Untested, likely works.                     |
| MiSTer FPGA          | ✅    |                                             |
| Nintendo Switch      | ❌    |                                             |
| Nintendo Switch 2    | ❌    |                                             |
| Xbox 360             | ❌    |                                             |
| PS1                  | ❌    |                                             |
| PS2                  | ❌    |                                             |
| PS3                  | ❌    |                                             |
| PS4                  | ❌    |                                             |
| PS5                  | ❌    |                                             |
