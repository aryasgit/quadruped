import smbus2
import time

bus = smbus2.SMBus(7)
ADDR = 0x3C

def cmd(c):
    bus.write_byte_data(ADDR, 0x00, c)

def init():
    for c in [
        0xAE,       # display off
        0xD5, 0x80, # clock div
        0xA8, 0x3F, # multiplex 64
        0xD3, 0x00, # display offset
        0x40,       # start line
        0x8D, 0x14, # charge pump on
        0x20, 0x00, # horizontal addressing
        0xA1,       # seg remap
        0xC8,       # com scan dec
        0xDA, 0x12, # com pins
        0x81, 0xCF, # contrast
        0xD9, 0xF1, # precharge
        0xDB, 0x40, # vcomh
        0xA4,       # all on resume
        0xA6,       # normal display
        0xAF,       # display on
    ]: cmd(c)

def clear():
    for page in range(8):
        cmd(0xB0 + page)
        cmd(0x00)
        cmd(0x10)
        for _ in range(128):
            bus.write_byte_data(ADDR, 0x40, 0x00)

def write_text(text, page, col=0):
    # 5x7 font subset — A-Z, 0-9, space
    FONT = {
        'A':b'\x7e\x11\x11\x11\x7e','R':b'\x7f\x09\x19\x29\x46',
        'B':b'\x7f\x49\x49\x49\x36','Q':b'\x3e\x41\x51\x21\x5e',
        'O':b'\x3e\x41\x41\x41\x3e','N':b'\x7f\x04\x08\x10\x7f',
        'L':b'\x7f\x40\x40\x40\x40','I':b'\x00\x41\x7f\x41\x00',
        'E':b'\x7f\x49\x49\x49\x41','S':b'\x46\x49\x49\x49\x31',
        'U':b'\x3f\x40\x40\x40\x3f','C':b'\x3e\x41\x41\x41\x22',
        'T':b'\x01\x01\x7f\x01\x01','W':b'\x3f\x40\x38\x40\x3f',
        'K':b'\x7f\x08\x14\x22\x41','G':b'\x3e\x41\x49\x49\x3a',
        'H':b'\x7f\x08\x08\x08\x7f','P':b'\x7f\x09\x09\x09\x06',
        'M':b'\x7f\x02\x04\x02\x7f','D':b'\x7f\x41\x41\x41\x3e',
        'Y':b'\x07\x08\x70\x08\x07','X':b'\x63\x14\x08\x14\x63',
        'Z':b'\x61\x51\x49\x45\x43','F':b'\x7f\x09\x09\x09\x01',
        'V':b'\x1f\x20\x40\x20\x1f','J':b'\x20\x40\x41\x3f\x01',
        '0':b'\x3e\x51\x49\x45\x3e','1':b'\x00\x42\x7f\x40\x00',
        '2':b'\x42\x61\x51\x49\x46','3':b'\x21\x41\x45\x4b\x31',
        '4':b'\x18\x14\x12\x7f\x10','5':b'\x27\x45\x45\x45\x39',
        '6':b'\x3c\x4a\x49\x49\x30','7':b'\x01\x71\x09\x05\x03',
        '8':b'\x36\x49\x49\x49\x36','9':b'\x06\x49\x49\x29\x1e',
        ' ':b'\x00\x00\x00\x00\x00','-':b'\x08\x08\x08\x08\x08',
    }
    cmd(0xB0 + page)
    cmd(col & 0x0F)
    cmd(0x10 | (col >> 4))
    for ch in text.upper():
        for byte in FONT.get(ch, FONT[' ']):
            bus.write_byte_data(ADDR, 0x40, byte)
        bus.write_byte_data(ADDR, 0x40, 0x00)  # char spacing

init()
clear()
write_text("BARQ", 2, 40)
write_text("ONLINE", 4, 26)
print("Done.")