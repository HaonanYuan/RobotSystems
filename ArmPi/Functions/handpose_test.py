import cv2
import numpy as np
import math
import time


def get_euclidean_distance(a, b):
    c = (float(a[0] - b[0]), float(a[1] - b[1]))
    return np.sqrt(c[0] ** 2 + c[1] ** 2)


def remove_background(frame):
    fgbg = cv2.createBackgroundSubtractorMOG2()
    fgmask = fgbg.apply(frame)
    kernel = np.ones((3, 3), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=fgmask)
    return res


def skin_color(img):
    # Transform to TCrCb space
    YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    # Split the Y, Cr, Cb value
    (y, cr, cb) = cv2.split(YCrCb)
    cr1 = cv2.GaussianBlur(cr, (5, 5), 0)
    _, skin = cv2.threshold(cr1, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    res = cv2.bitwise_and(img, img, mask=skin)
    return res


def get_contour(img):
    # binaryimg = cv2.Canny(Laplacian, 50, 200) #二值化，canny检测
    h = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 寻找轮廓
    contour = h[0]
    contour = sorted(contour, key=cv2.contourArea, reverse=True)  # 已轮廓区域面积进行排序
    # contourmax = contour[0][:, 0, :]#保留区域面积最大的轮廓点坐标
    bg = np.ones(dst.shape, np.uint8) * 255  # 创建白色幕布
    ret = cv2.drawContours(bg, contour[0], -1, (0, 0, 0), 3)  # 绘制黑色轮廓
    return ret, contour


def get_defect_count(frame, contour, defects, verbose=False):
    ndefects = 0
    for i in range(defects.shape[0]):
        s, e, f, _ = defects[i, 0]
        beg = tuple(contour[s][0])
        end = tuple(contour[e][0])
        far = tuple(contour[f][0])
        a = get_euclidean_distance(beg, end)
        b = get_euclidean_distance(beg, far)
        c = get_euclidean_distance(end, far)
        angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # * 57
        if angle <= math.pi / 2:  # 90:
            ndefects = ndefects + 1
        if verbose:
            cv2.circle(frame, far, 3, (255, 0, 0), -1)
        if verbose:
            cv2.line(frame, beg, end, (255, 0, 0), 1)
    return frame, ndefects


def get_convex(frame, contours):
    hull = cv2.convexHull(contours, returnPoints=False)
    defects = cv2.convexityDefects(contours, hull)
    if defects is not None:
        frame, number = get_defect_count(frame, contours, defects)
        return frame, number
    return None, 0


if __name__ == '__main__':
    path = None
    if path is None:
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(path)

    while True:
        ret, frame = cap.read()
        frame = cv2.bilateralFilter(frame, 5, 50, 100)

        # 下面三行可以根据自己的电脑进行调节
        src = cv2.resize(frame, (800, 600), interpolation=cv2.INTER_CUBIC)  # 窗口大小
        cv2.rectangle(src, (200, 150), (600, 450), (0, 0, 255))  # 框出截取位置
        cv2.imshow("the original image", src)

        roi = src[150:450, 200:600]  # the area you want to use identify the hand pose
        resb = remove_background(roi)
        res = skin_color(resb)  # detect the skin
        cv2.imshow("skin detection", res)

        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        dst = cv2.Laplacian(gray, cv2.CV_16S, ksize=3)
        Laplacian = cv2.convertScaleAbs(dst)

        contour, array = get_contour(Laplacian)  # Get contour and draw it
        cv2.imshow("draw finger contour", contour)

        largecont = max(array, key=lambda contour: cv2.contourArea(contour))
        frame_res, finger_num = get_convex(contour, largecont)
        print('The number of finger: {}'.format(finger_num))
        cv2.imshow("draw the point of fingertips", frame_res)
        # time.sleep(0.1)
        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()