# MaixCAM-Pro QR UART

`maixcam_qr_uart.py` uses MaixPy v4 to recognize QR codes and send each
payload to the STM32 over MaixCAM-Pro UART1. It uses the 320x224 hardware
`QRCodeDetector` path when available, which is faster than scanning a
640x480 frame with the general-purpose decoder.

## Wiring

| MaixCAM-Pro | STM32F407 project |
| --- | --- |
| A19 / UART1_TX | PD9 / USART3_RX |
| A18 / UART1_RX | PD8 / USART3_TX |
| GND | GND |

Both sides use `115200 8N1`. TX and RX must be crossed. Use 3.3 V logic and
connect the grounds.

## Output format

The MaixCAM script sends the raw task code to the STM32:

```text
<QR payload>\r\n
```

The STM32 then sends the serial-screen command through UART5:

```text
t0.txt="156+123+516+231" FF FF FF
```

The final three values are raw bytes `0xFF 0xFF 0xFF`, not ASCII text.

For the current STM32 task-code parser, a valid payload is 15 characters in
the form `XXX+XXX+XXX+XXX`, for example `123+231+312+132`.

## Running

Copy `maixcam_qr_uart.py` to the MaixCAM-Pro, then run it with MaixPy v4 from
MaixVision or the device terminal. Preview drawing is disabled by default to
maximize frame rate; set `SHOW_PREVIEW = True` when visual debugging is
needed.
