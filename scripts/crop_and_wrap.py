import numpy as np

def crop_and_wrap(frame, bbox, pad_ratio=0.6):
    h_img, w_img = frame.shape[:2]
    x, y, w, h = [int(v) for v in bbox]

    pad_w = int(w * pad_ratio)
    pad_h = int(h * pad_ratio)

    x1 = x - pad_w
    x2 = x + w + pad_w
    y1 = max(y - pad_h, 0)
    y2 = min(y + h + pad_h, h_img)

    if x1 < 0 or x2 > w_img:
        x1_mod = x1 % w_img
        x2_mod = x2 % w_img

        if x1 < 0 and x2 <= w_img:
            left = frame[y1:y2, x1_mod:]
            right = frame[y1:y2, :x2]
        elif x1 >= 0 and x2 > w_img:
            left = frame[y1:y2, x1:]
            right = frame[y1:y2, :x2_mod]
        else:  # full wrap
            left = frame[y1:y2, x1_mod:]
            right = frame[y1:y2, :x2_mod]

        cropped = np.hstack([left, right])
        x1_draw, x2_draw = 0, cropped.shape[1]
    else:
        cropped = frame[y1:y2, x1:x2]
        x1_draw, x2_draw = x1, x2

    return cropped, (x1_draw, y1, x2_draw, y2)
