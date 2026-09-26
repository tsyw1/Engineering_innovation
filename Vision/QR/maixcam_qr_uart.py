"""MaixCAM-Pro QR-code scanner with UART1 output."""

from maix import app, camera, display, image, pinmap, time, uart, err


UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
# The hardware QR detector is optimized for 320x224 and is much faster than
# running the general-purpose decoder on a 640x480 frame.
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 224
CAMERA_FPS = 60
# Open the live camera preview on the MaixCAM-Pro display.
SHOW_PREVIEW = True
USE_HARDWARE_DETECTOR = True
RESEND_INTERVAL_MS = 500
RESET_AFTER_LOST_MS = 1000


def setup_uart():
    """Configure MaixCAM-Pro UART1 pins and open the serial port."""
    err.check_raise(
        pinmap.set_pin_function("A19", "UART1_TX"),
        "Failed to set A19 as UART1_TX",
    )
    err.check_raise(
        pinmap.set_pin_function("A18", "UART1_RX"),
        "Failed to set A18 as UART1_RX",
    )
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def payload_text(payload):
    """Convert a QR payload to one line of UTF-8-safe text."""
    if isinstance(payload, bytes):
        text = payload.decode("utf-8", "replace")
    else:
        text = str(payload)
    return text.replace("\r", "").replace("\n", "")


def draw_qr_result(img, qr):
    corners = qr.corners()
    for index in range(4):
        start = corners[index]
        end = corners[(index + 1) % 4]
        img.draw_line(
            start[0], start[1], end[0], end[1], image.COLOR_RED
        )

    img.draw_string(
        0, 0, "QR: " + payload_text(qr.payload()), image.COLOR_BLUE
    )


def create_camera():
    """Create the fastest supported camera mode, with a safe fallback."""
    try:
        return camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, fps=CAMERA_FPS)
    except Exception as exc:
        print("60 FPS camera mode unavailable: {}".format(exc))
        return camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, fps=30)


def detect_qrcodes(img, detector):
    """Use NPU-accelerated detection when available, otherwise fall back."""
    if detector is not None:
        return detector.detect(img)
    return img.find_qrcodes()


def main():
    serial = setup_uart()
    cam = create_camera()
    disp = display.Display() if SHOW_PREVIEW else None
    detector = None

    if USE_HARDWARE_DETECTOR:
        try:
            detector = image.QRCodeDetector()
            print("Using hardware QRCodeDetector")
        except Exception as exc:
            print("QRCodeDetector unavailable: {}".format(exc))
            print("Using find_qrcodes() fallback")

    last_payload = None
    last_send_ms = 0
    last_seen_ms = 0

    print("MaixCAM QR scanner started")
    print("UART: {} @ {} 8N1".format(UART_DEVICE, UART_BAUDRATE))

    while not app.need_exit():
        img = cam.read()
        qrcodes = detect_qrcodes(img, detector)
        now_ms = time.ticks_ms()

        if qrcodes:
            if SHOW_PREVIEW:
                for qr in qrcodes:
                    draw_qr_result(img, qr)

            payload = payload_text(qrcodes[0].payload())
            last_seen_ms = now_ms

            if payload and (
                payload != last_payload
                or (now_ms - last_send_ms) >= RESEND_INTERVAL_MS
            ):
                # STM32 receives the raw task code and adds the serial-screen
                # command wrapper: t0.txt="..." + FF FF FF.
                serial.write_str(payload + "\r\n")
                print("QR -> STM32: {}".format(payload))
                last_payload = payload
                last_send_ms = now_ms
        elif last_payload is not None and (
            now_ms - last_seen_ms
        ) >= RESET_AFTER_LOST_MS:
            last_payload = None

        if disp is not None:
            disp.show(img)
        time.sleep_ms(1)


if __name__ == "__main__":
    main()
