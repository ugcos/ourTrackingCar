import cv2
import numpy as np
from rknnlite.api import RKNNLite
from periphery import Serial
import time

# ==================== 配置参数 ====================
RKNN_MODEL = 'best.rknn'

# 降低检测阈值
OBJ_THRESH = 0.15
NMS_THRESH = 0.45

IMG_SIZE = (640, 640)

CLASSES = (
    "Stop",
    "Turn_right",
    "Turn_left",
    "Go_straight"
)

# 串口设置
UART_DEV = "/dev/ttyS0"
BAUDRATE = 115200

# 摄像头
CAMERA_ID = 0

# 稳定检测参数
TOTAL_FRAMES = 10
STABLE_CONSECUTIVE = 6


# ==================== 后处理函数 ====================

def filter_boxes(boxes, box_confidences, box_class_probs):

    box_confidences = box_confidences.reshape(-1)

    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(
        class_max_score * box_confidences >= OBJ_THRESH
    )

    scores = (class_max_score * box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):

    x = boxes[:, 0]
    y = boxes[:, 1]

    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h

    order = scores.argsort()[::-1]

    keep = []

    while order.size > 0:

        i = order[0]

        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])

        xx2 = np.minimum(
            x[i] + w[i],
            x[order[1:]] + w[order[1:]]
        )

        yy2 = np.minimum(
            y[i] + h[i],
            y[order[1:]] + h[order[1:]]
        )

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)

        inter = w1 * h1

        ovr = inter / (
            areas[i] + areas[order[1:]] - inter
        )

        inds = np.where(ovr <= NMS_THRESH)[0]

        order = order[inds + 1]

    keep = np.array(keep)

    return keep


def box_process(position, anchors):

    grid_h, grid_w = position.shape[2:4]

    col, row = np.meshgrid(
        np.arange(0, grid_w),
        np.arange(0, grid_h)
    )

    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)

    grid = np.concatenate((col, row), axis=1)

    stride = np.array([
        IMG_SIZE[1] // grid_h,
        IMG_SIZE[0] // grid_w
    ]).reshape(1, 2, 1, 1)

    col = col.repeat(len(anchors), axis=0)
    row = row.repeat(len(anchors), axis=0)

    anchors = np.array(anchors)
    anchors = anchors.reshape(*anchors.shape, 1, 1)

    box_xy = position[:, :2, :, :] * 2 - 0.5
    box_wh = pow(position[:, 2:4, :, :] * 2, 2) * anchors

    box_xy += grid
    box_xy *= stride

    box = np.concatenate((box_xy, box_wh), axis=1)

    xyxy = np.copy(box)

    xyxy[:, 0, :, :] = (
        box[:, 0, :, :] - box[:, 2, :, :] / 2
    )

    xyxy[:, 1, :, :] = (
        box[:, 1, :, :] - box[:, 3, :, :] / 2
    )

    xyxy[:, 2, :, :] = (
        box[:, 0, :, :] + box[:, 2, :, :] / 2
    )

    xyxy[:, 3, :, :] = (
        box[:, 1, :, :] + box[:, 3, :, :] / 2
    )

    return xyxy


def yolov5_post_process(input_data):

    anchors = [
        [[10.0, 13.0], [16.0, 30.0], [33.0, 23.0]],
        [[30.0, 61.0], [62.0, 45.0], [59.0, 119.0]],
        [[116.0, 90.0], [156.0, 198.0], [373.0, 326.0]]
    ]

    boxes = []
    scores = []
    classes_conf = []

    input_data = [
        _in.reshape(
            [len(anchors[0]), -1] + list(_in.shape[-2:])
        )
        for _in in input_data
    ]

    for i in range(len(input_data)):

        boxes.append(
            box_process(
                input_data[i][:, :4, :, :],
                anchors[i]
            )
        )

        scores.append(
            input_data[i][:, 4:5, :, :]
        )

        classes_conf.append(
            input_data[i][:, 5:, :, :]
        )

    def sp_flatten(_in):

        ch = _in.shape[1]

        _in = _in.transpose(0, 2, 3, 1)

        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    boxes, classes, scores = filter_boxes(
        boxes,
        scores,
        classes_conf
    )

    nboxes = []
    nclasses = []
    nscores = []

    for c in set(classes):

        inds = np.where(classes == c)

        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


# ==================== 修复后的 draw ====================

def draw(image, boxes, scores, classes):

    for box, score, cl in zip(boxes, scores, classes):

        # YOLO输出格式
        left, top, right, bottom = [
            int(_b) for _b in box
        ]

        # 防止越界
        left = max(0, left)
        top = max(0, top)

        right = min(image.shape[1], right)
        bottom = min(image.shape[0], bottom)

        print(
            f'class: {CLASSES[int(cl)]}, '
            f'score: {score:.2f}'
        )

        cv2.rectangle(
            image,
            (left, top),
            (right, bottom),
            (255, 0, 0),
            2
        )

        cv2.putText(
            image,
            f'{CLASSES[int(cl)]} {score:.2f}',
            (left, max(top - 10, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):

    shape = im.shape[:2]

    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(
        new_shape[0] / shape[0],
        new_shape[1] / shape[1]
    )

    ratio = r, r

    new_unpad = (
        int(round(shape[1] * r)),
        int(round(shape[0] * r))
    )

    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:

        im = cv2.resize(
            im,
            new_unpad,
            interpolation=cv2.INTER_LINEAR
        )

    top, bottom = (
        int(round(dh - 0.1)),
        int(round(dh + 0.1))
    )

    left, right = (
        int(round(dw - 0.1)),
        int(round(dw + 0.1))
    )

    im = cv2.copyMakeBorder(
        im,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=color
    )

    return im, ratio, (dw, dh)


# ==================== 主程序 ====================

if __name__ == '__main__':

    # 1. 加载 RKNN 模型
    print('--> Load RKNN model')

    rknn = RKNNLite()

    ret = rknn.load_rknn(RKNN_MODEL)

    if ret != 0:
        print('Load RKNN model failed')
        exit(ret)

    print('done')

    # 2. 初始化运行环境
    print('--> Init runtime environment')

    ret = rknn.init_runtime()

    if ret != 0:
        print('Init runtime environment failed!')
        exit(ret)

    print('done')

    # 3. 打开串口
    print('--> Open serial port')

    serial = Serial(
        UART_DEV,
        baudrate=BAUDRATE,
        databits=8,
        stopbits=1,
        parity="none",
        xonxoff=False,
        rtscts=False
    )

    print('Serial port opened')

    # 4. 打开摄像头
    print('--> Open camera')

    cap = cv2.VideoCapture(
        CAMERA_ID,
        cv2.CAP_V4L2
    )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print('Failed to open camera')
        exit(-1)

    print('Camera ready')

    detected_labels = []

    # ==================== 主循环 ====================

    while True:

        print("\nWaiting for 'begin' command from STM32...")

        try:

            data = serial.read(5)

            try:
                data_str = data.decode().strip()
            except:
                continue

            if data_str == "begin":
                print(
                    "Received 'begin', "
                    "starting detection."
                )
            else:
                continue

        except Exception:
            continue

        # 清空检测记录
        detected_labels.clear()

        stable_label = None

        # ==================== 检测10帧 ====================

        for i in range(TOTAL_FRAMES):

            ret, frame = cap.read()

            if not ret:
                print("Failed to grab frame")
                break

            # ==================== 预处理 ====================

            img, ratio, (dw, dh) = letterbox(
                frame,
                new_shape=IMG_SIZE
            )

            img_rgb = cv2.cvtColor(
                img,
                cv2.COLOR_BGR2RGB
            )

            # RKNN 推荐 NHWC 输入
            input_data = img_rgb
            # ==================== 推理 ====================

            outputs = rknn.inference(
                inputs=[input_data]
            )

            # ==================== 后处理 ====================

            boxes, classes, scores = yolov5_post_process(
                outputs
            )

            print("classes:", classes)
            print("scores:", scores)

            # ==================== 可视化 ====================

            if boxes is not None:
                draw(
                    frame,
                    boxes,
                    scores,
                    classes
                )

            cv2.imshow("Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # ==================== 记录结果 ====================

            if boxes is not None and len(classes) > 0:

                max_idx = np.argmax(scores)

                label = CLASSES[
                    int(classes[max_idx])
                ]

                detected_labels.append(label)

                print("Current label:", label)

            else:

                detected_labels.append(None)

                print("Current label: None")

            # ==================== 连续稳定检测 ====================

            if len(detected_labels) >= STABLE_CONSECUTIVE:

                last_n = detected_labels[
                    -STABLE_CONSECUTIVE:
                ]

                print(
                    "Last stable check:",
                    last_n
                )

                # 连续6帧完全一致
                if (
                    all(l is not None for l in last_n)
                    and len(set(last_n)) == 1
                ):

                    stable_label = last_n[0]

                    print(
                        f"Stable label found: "
                        f"{stable_label}"
                    )

                    break

        # ==================== 发送结果 ====================

        if stable_label:
            cmd = stable_label.lower()
        else:
            # 默认直行
            cmd = "go_straight"

        print(f"Sending command: {cmd}")

        try:

            serial.write(
                (cmd + '\n').encode()
            )

        except Exception as e:

            print(f"Serial write error: {e}")

            serial.write(
                b"python-periphery!\n"
            )

    # ==================== 释放资源 ====================

    cap.release()

    cv2.destroyAllWindows()

    rknn.release()

    serial.close()