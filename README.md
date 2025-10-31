Custom FW for Haute42 leverless gamepads. The goals of this project are:
1. Minimal latency.
2. Support for all HW features on Haute42 (all buttons/LEDs/OLED).
3. Simple user configuration.

### Feature list

| Feature          | State | Notes                    |                                                     
|------------------|:-----:|--------------------------|                                                     
| HID Gamepad      | ✅    | Tested on Linux.         |                                                     
| HID Keyboard     | ❌    |                          |                                              
| XInput           | ❌    |                          |
| 1000Hz Polling   | ✅    |                          |                                                     
| Button Remap     | ✅    |                          |  
| Button Debounce  | ✅    | Leading-edge debounce.   |  
| SOCD Cleaning    | ⚠️    | Only Neutral/Last-input. |                                                    
| LED Support      | ✅    | Per-button LED config.   |                                                     
| LED Animations   | ❌    |                          |                                                     
| LED Brightness   | ⚠️    | Only on/off toggle.      |                                                     
| OLED Support     | ❌    |                          |                                                     
| Console Support  | ❌    |                          |     
