"""Create a printable DICT_4X4_50 marker with a white margin."""
import argparse
import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--id', type=int, default=1)
    parser.add_argument('--output', default='marker.png')
    args = parser.parse_args()
    if not 0 <= args.id < 50:
        parser.error('ID must be between 0 and 49')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, args.id, 600)
    marker = cv2.copyMakeBorder(marker, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
    if not cv2.imwrite(args.output, marker):
        raise RuntimeError('Could not save marker')
    print(f'Saved {args.output}. Measure the printed black square and update marker_size_m.')


if __name__ == '__main__':
    main()
