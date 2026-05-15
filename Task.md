# 项目需求说明文档：基于RK3588与STM32的交通标志识别与自动导航小车

## 1. 项目概述
本项目旨在开发一个基于 **RK3588 开发板** 和 **STM32 单片机** 的智能小车导航系统。
RK3588 作为上位机，负责通过 USB 摄像头采集图像，并利用内置 NPU 运行 YOLO 算法识别交通标志。STM32 作为下位机，负责底盘运动控制（直行、左转、右转、停车等）以及路口黑线检测。两者通过 UART 串口进行指令和结果的交互，最终实现小车按指示导航至目标终点。

## 2. 硬件与环境配置要求

### 2.1 硬件连接
* **核心板**：RK3588 (LuBanCat4)
* **下位机**：STM32 单片机
* **传感器**：USB 摄像头、四路红外传感器（用于寻线和路口检测）
* **串口连接 (UART0)**：
    * RK3588 `TX` 连接 STM32 `RX`
    * RK3588 `RX` 连接 STM32 `TX`
    * RK3588 `GND` 连接 STM32 `GND`

### 2.2 系统环境配置 (RK3588端)
* **开启 UART0**：需修改 `/boot/uEnv/uEnv.txt`，取消注释 `dtoverlay=/dtb/overlay/rk3588-lubancat-uart0-m2-overlay.dtbo` 并重启。
* **依赖库安装**：
    * OpenCV: `sudo apt-get install python3-opencv`
    * 串口通信库: `sudo pip3 install python-periphery`

## 3. 核心功能模块开发指南

### 3.1 串口通信模块 (UART)
* **库**：`from periphery import Serial`
* **配置参数**：设备节点 `/dev/ttyS0`，波特率 `115200`，数据位 `8`，停止位 `1`，无校验 `parity="none"`。
* **通信逻辑**：
    * **接收**：持续监听，接收 STM32 发送的识别触发指令（如 `begin`，长度约5字节）。
    * **发送**：将最终确认的识别结果（如 `turn_right`）编码为字节流发送给 STM32。

### 3.2 摄像头图像采集与预处理模块
* **设备调用**：必须使用 `cv2.VideoCapture(0, cv2.CAP_V4L2)` 开启摄像头。
* **预处理流**：
    1.  `ret, frame = cap.read()` 获取图像帧。
    2.  `img, ratio, (dw, dh) = letterbox(frame, new_shape=(640, 640))` 进行尺寸调整。
    3.  `img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)` 转换颜色空间以适配模型。

### 3.3 NPU 目标检测模块 (YOLO)
* **推理引擎**：使用 `rknn_lite` 进行实时推理。
* **核心调用**：`outputs = rknn_lite.inference(inputs=[img])`
* **后处理**：`boxes, classes, scores = yolov5_post_process(outputs)`
* **标签映射**：将识别到的 `classes[0]` 映射为具体标签，并存入 `detected_labels` 列表。

## 4. 主程序控制流 (Main Loop Logic)

请根据以下业务逻辑流编写主程序 `main.py`：

**初始化阶段：**
1. 创建 RKNNLite 对象并加载训练好的交通标志 RKNN 模型。
2. 初始化 RKNN 运行时环境。
3. 打开串口与 STM32 连接。
4. 打开 USB 摄像头。

**循环监听阶段 (while True)：**
1. 监听串口获取 STM32 命令。STM32 在检测到十字路口（四个红外传感器全为黑线）时会停车并发送检测指令。
2. 当接收到特定检测命令（如 `begin`）时：
    * 清空之前的检测标签列表 `detected_labels = []`。
    * 进入图像处理子循环，连续捕获并处理 **10帧** 图像。
3. **图像处理子循环（针对每一帧）：**
    * 图像预处理（Resize, Pad, Color Convert）。
    * 调用 RKNNLite 执行推理。
    * 解析推理结果（边界框、类别、置信度）。
    * 在图像帧上绘制 Bounding Box 并实时显示（可选）。
    * 记录当前帧检测到的类别标签。
4. **结果决断与反馈：**
    * 评估这 10 帧的检测结果，判断是否有**连续 6 帧**中稳定出现同一类别的标签。
    * 如果满足连续 6 帧一致的条件，将该稳定标签确认最终结果。
    * 通过 UART 串口将该结果（如直行、左转、右转、停车等指令）发送给 STM32 单片机。

## 5. 待开发任务 (给 AI 助手的 Prompt)

请根据以上需求，帮我编写 RK3588 端的完整 Python 主控程序。
**要求：**
1. 包含必要的类/函数封装（如串口通信类、摄像头预处理函数）。
2. 提供清晰的代码注释。
3. 针对 `rknn_lite` 和 `yolov5_post_process` 可以先使用占位函数或基础框架，重点完善**串口通信监听**和**连续多帧结果稳定判断（10帧内连续6帧一致）**的业务逻辑。