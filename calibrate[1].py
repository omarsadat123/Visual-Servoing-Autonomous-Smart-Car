"""Calibrate from varied, in-focus chessboard photographs captured at one resolution."""
import argparse
import glob
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('images', help='Quoted glob, e.g. "photos/*.jpg"')
    parser.add_argument('--columns', type=int, default=9, help='Inner corners')
    parser.add_argument('--rows', type=int, default=6, help='Inner corners')
    parser.add_argument('--square-m', type=float, default=0.025)
    parser.add_argument('--output', default='calibration.npz')
    args = parser.parse_args()
    if min(args.columns, args.rows) < 3 or args.square_m <= 0:
        parser.error('Invalid chessboard dimensions')
    pattern = (args.columns, args.rows)
    obj = np.zeros((args.columns*args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)*args.square_m
    objects, images, size = [], [], None
    for path in sorted(glob.glob(args.images)):
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        current = (image.shape[1], image.shape[0])
        if size is not None and current != size:
            raise ValueError('Mixed image resolutions')
        size = current
        ok, corners = cv2.findChessboardCornersSB(image, pattern)
        if ok:
            objects.append(obj.copy())
            images.append(corners)
    if len(images) < 12:
        raise ValueError('Need at least 12 usable, varied chessboard images')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, size, None, None)
    np.savez(args.output, camera_matrix=matrix, dist_coeffs=distortion, image_size=size, rms=rms)
    print(f'Saved {args.output}: {len(images)} views, RMS {rms:.3f} px. Inspect quality before driving.')


if __name__ == '__main__':
    main()
