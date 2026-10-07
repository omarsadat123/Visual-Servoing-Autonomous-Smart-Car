"""Calibrated or image-centered ArUco following; GPIO is opt-in."""
import argparse
import json
from pathlib import Path
import signal
import time
import cv2
import numpy as np
from control import Controller, ImageController
from motors import Motors
from vision import estimate_pose, load_calibration


def stop_signal(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    parser.add_argument('--drive', action='store_true', help='Enable all four motors')
    parser.add_argument('--mode', choices=('pose', 'basic'), default='pose')
    parser.add_argument('--headless', action='store_true', help='SSH operation; Ctrl+C stops')
    args = parser.parse_args()
    with args.config.open(encoding='utf8') as file:
        config = json.load(file)
    if not 0 <= config['marker_id'] < 50 or not 0 < config['basic_stop_width_fraction'] <= 1:
        parser.error('Invalid marker ID or basic-mode stop threshold')
    if not np.isfinite(config['marker_size_m']) or config['marker_size_m'] <= 0:
        parser.error('marker_size_m must be finite and positive')
    if args.mode == 'pose':
        matrix, distortion, size = load_calibration(args.config.resolve().parent / config['calibration'])
        controller = Controller(config)
    else:
        size = (config['frame_width'], config['frame_height'])
        controller = ImageController(config)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    cap = cv2.VideoCapture(config['camera'])
    motors = None
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(size[0]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(size[1]))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Backend may ignore this request.
        motors = Motors(config['pins'], args.drive)
        previous = time.monotonic()
        next_log = previous
        print(f'{args.mode}: ' + ('DRIVE enabled' if args.drive else 'DRY RUN: GPIO disabled'))
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera frame capture failed')
            if args.mode == 'pose' and (frame.shape[1], frame.shape[0]) != size:
                raise RuntimeError('Camera resolution differs from calibration')
            corners, ids, _ = detector.detectMarkers(frame)
            first = second = None
            if ids is not None:
                matches = np.flatnonzero(ids.flatten() == config['marker_id'])
                if len(matches) == 1:
                    selected = corners[int(matches[0])].reshape(4, 2)
                    if args.mode == 'pose':
                        pose = estimate_pose(selected, config['marker_size_m'], matrix, distortion)
                        if pose is not None:
                            first, _, second = pose
                    else:
                        first = (selected[:, 0].mean() - frame.shape[1] / 2) / (frame.shape[1] / 2)
                        second = np.linalg.norm(np.roll(selected, -1, axis=0) - selected, axis=1).mean() / frame.shape[1]
                if not args.headless:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            now = time.monotonic()
            left, right = controller.step(first, second, now - previous)
            previous = now
            motors.set(left, right)
            if args.headless:
                if now >= next_log:
                    print(f'target={first is not None} left={left:.2f} right={right:.2f}', flush=True)
                    next_log = now + 1
            else:
                cv2.putText(frame, f'L {left:.2f} R {right:.2f} | q: stop', (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
                cv2.imshow('ArUco follower', frame)
                if cv2.waitKey(1) & 255 == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if motors:
            motors.close()
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
