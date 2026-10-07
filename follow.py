"""Run dry by default; --drive explicitly enables GPIO outputs."""
import argparse
import json
import signal
import time
import cv2
import numpy as np
from control import Controller
from motors import Motors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--drive', action='store_true')
    args = parser.parse_args()
    with open(args.config, encoding='utf8') as file:
        config = json.load(file)
    calibration = np.load(config['calibration'], allow_pickle=False)
    matrix, distortion = calibration['camera_matrix'], calibration['dist_coeffs']
    size = tuple(calibration['image_size'].astype(int))
    half = config['marker_size_m'] / 2
    if half <= 0:
        raise ValueError('Measure marker_size_m before running')
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    controller = Controller(config)
    cap = cv2.VideoCapture(config['camera'])
    motors = None
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(size[0]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(size[1]))
        motors = Motors(config['pins'], args.drive)
        previous = time.monotonic()
        print('DRIVE enabled' if args.drive else 'DRY RUN: GPIO disabled')
        while True:
            ok, frame = cap.read()
            if not ok:
                motors.set(0, 0)
                break
            if (frame.shape[1], frame.shape[0]) != size:
                raise RuntimeError('Camera resolution differs from calibration')
            corners, ids, _ = detector.detectMarkers(frame)
            x = z = None
            if ids is not None:
                matches = np.flatnonzero(ids.flatten() == config['marker_id'])
                if len(matches) == 1:
                    image_points = corners[int(matches[0])].reshape(4, 2)
                    found, rotation, translation = cv2.solvePnP(
                        points, image_points, matrix, distortion,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if found and np.all(np.isfinite(translation)):
                        projected, _ = cv2.projectPoints(points, rotation, translation, matrix, distortion)
                        error = np.linalg.norm(projected.reshape(4, 2)-image_points, axis=1).mean()
                        if error < 3:
                            x, z = float(translation[0, 0]), float(translation[2, 0])
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            now = time.monotonic()
            left, right = controller.step(x, z, now-previous)
            previous = now
            motors.set(left, right)
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
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
