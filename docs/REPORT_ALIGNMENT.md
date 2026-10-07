
# Report alignment

Source: the 16-page `project-report.pdf` supplied by Omar Sadat. The PDF is preserved unchanged.

| Report evidence | Repository implementation |
| --- | --- |
| Raspberry Pi 5, USB webcam, four BO DC motors and 4WD chassis | Four GPIO motor channels; USB camera index configurable |
| Python, OpenCV, gpiozero on Raspberry Pi OS | Python modules and requirements |
| Initial marker-center alignment | `--mode basic`, horizontal offset and apparent-size stop |
| Tracking screenshot displays marker ID 1 | Default `marker_id` and marker generator use 1 |
| Chessboard camera calibration | `calibrate.py`, camera matrix and distortion coefficients |
| X/Y/Z pose estimation | `vision.py` returns all three coordinates; following uses X/Z |
| Variable PWM and smoother steering | Proportional differential steering with command slew limit |
| Stop when marker too close | Calibrated following-distance threshold or basic apparent-size threshold |
| SSH access and lgpio pin factory | `--headless` and explicit `LGPIOFactory` |
| Separate 2S motor battery and Pi power bank | Wiring documentation keeps motor and Pi supplies separate |

## Driver count correction

The report's hardware table lists one L298N. Omar clarified in chat that the car uses **two L298N drivers**. The repository follows that clarification: one channel per motor, with front and rear motors grouped by side. Example front/rear driver assignment and GPIO pins are configurable because the report does not provide a pin schematic.

## Parameters requiring measurement

The report does not contain the original program, exact GPIO assignments, marker dictionary/physical size, calibration matrices, gains, target distance, PWM duty limits, or numerical tracking benchmarks. The tracking screenshot shows marker ID 1 and center X = 640, but it does not establish the final camera calibration resolution. Other supplied defaults are reconstruction choices. Calibrate and tune them on the actual car. Software tests verify logic and synthetic vision, not physical tracking performance.

## Corrections and scope

- The report's cost rows sum to BDT 32,600, rather than its stated approximately BDT 27,000. The document also lists only one driver, so neither total establishes the corrected two-driver cost. The README avoids claiming a verified build cost.
- A 2S Li-ion pack's voltage varies with charge (up to 8.4 V for conventional 4.2 V cells). Check the actual motors, battery protection and driver configuration rather than treating the report's 6-7.4 V description as a fixed output.
- ORB tracking, human following, obstacle avoidance and AI object recognition are future work in the report. They are not implemented features here.
- Original team membership is credited without reproducing student ID numbers in the README.
