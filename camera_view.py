"""OpenCV camera viewer shared by the three original camera scripts."""
import time


def show(name, url):
    import cv2
    camera = cv2.VideoCapture(url)
    if not camera.isOpened():
        raise ConnectionError(f'Камера {name} недоступна: {url}')
    zoom = 1.0
    failures = 0
    try:
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 20:
                    raise ConnectionError(f'{name}: нет кадров от {url}')
                time.sleep(0.05)
                continue
            failures = 0
            height, width = frame.shape[:2]
            crop_h, crop_w = max(1, int(height / zoom)), max(1, int(width / zoom))
            top, left = (height - crop_h) // 2, (width - crop_w) // 2
            cv2.imshow(name, cv2.resize(frame[top:top+crop_h, left:left+crop_w], (width, height)))
            key = cv2.waitKey(1) & 0xff
            if key == 27:
                break
            if key in (ord('+'), ord('=')):
                zoom = min(10, zoom + 0.25)
            if key in (ord('-'), ord('_')):
                zoom = max(1, zoom - 0.25)
    finally:
        camera.release()
        cv2.destroyAllWindows()
