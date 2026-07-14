from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from PIL import Image, ImageDraw
import time

serial = i2c(port=7, address=0x3C)
device = sh1106(serial)

while True:
    img = Image.new("1", (128, 64), 0)
    draw = ImageDraw.Draw(img)
    draw.rectangle((0,0,127,63), outline=255)
    draw.text((20, 25), "OLED TEST", fill=255)

    device.display(img)
    time.sleep(0.1)