from __future__ import annotations

import math
import random
import time

from PIL import Image, ImageDraw
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106


WIDTH = 128
HEIGHT = 64
ADDR = 0x3C
I2C_PORT = 7

EMOTIONS = (
	"happy",
	"excited",
	"neutral",
	"sleepy",
	"sad",
	"angry",
	"surprised",
	"wink",
)


def _draw_top_eyebrows(draw: ImageDraw.ImageDraw, emotion: str, phase: float) -> None:
	"""Draw the eyebrows in the top band only, with no other decoration."""
	left_x = 31 + int(math.sin(phase * 0.6) * 1)
	right_x = 97 + int(math.sin(phase * 0.6) * 1)
	base_y = 5

	if emotion == "happy":
		draw.arc((left_x - 11, base_y - 3, left_x + 9, base_y + 8), start=200, end=350, fill=255)
		draw.arc((right_x - 9, base_y - 3, right_x + 11, base_y + 8), start=190, end=340, fill=255)
	elif emotion == "sad":
		draw.line((left_x - 11, base_y + 6, left_x + 5, base_y + 1), fill=255)
		draw.line((right_x - 5, base_y + 1, right_x + 11, base_y + 6), fill=255)
	elif emotion == "angry":
		draw.line((left_x - 11, base_y + 1, left_x + 5, base_y + 6), fill=255)
		draw.line((right_x - 5, base_y + 6, right_x + 11, base_y + 1), fill=255)
	elif emotion == "surprised":
		draw.arc((left_x - 12, base_y - 1, left_x + 10, base_y + 10), start=200, end=340, fill=255)
		draw.arc((right_x - 10, base_y - 1, right_x + 12, base_y + 10), start=200, end=340, fill=255)
	elif emotion == "sleepy":
		draw.line((left_x - 10, base_y + 4, left_x + 6, base_y + 4), fill=255)
		draw.line((right_x - 6, base_y + 4, right_x + 10, base_y + 4), fill=255)
	elif emotion == "wink":
		draw.line((left_x - 10, base_y + 2, left_x + 6, base_y + 0), fill=255)
		draw.line((right_x - 6, base_y + 0, right_x + 10, base_y + 2), fill=255)
	else:
		draw.line((left_x - 10, base_y + 2, left_x + 6, base_y + 2), fill=255)
		draw.line((right_x - 6, base_y + 2, right_x + 10, base_y + 2), fill=255)


def _draw_face_outline(draw: ImageDraw.ImageDraw, cx: int, cy: int, emotion: str, bob: float) -> None:
	eye_y = cy - 2 + int(bob * 0.5)
	eye_gap = 28
	left_eye_x = cx - eye_gap
	right_eye_x = cx + eye_gap
	eye_w = 18
	eye_h = 18

	if emotion == "surprised":
		eye_w = 20
		eye_h = 20
	elif emotion == "sleepy":
		eye_h = 7
	elif emotion == "angry":
		eye_w = 16
		eye_h = 12
	elif emotion == "wink":
		eye_h = 12

	def draw_eye(center_x: int, center_y: int, open_w: int, open_h: int, pupil_scale: float = 0.45) -> None:
		box = (center_x - open_w // 2, center_y - open_h // 2, center_x + open_w // 2, center_y + open_h // 2)
		if open_h <= 4:
			draw.line((box[0], center_y, box[2], center_y), fill=255)
			return
		draw.ellipse(box, outline=255)
		pupil_w = max(2, int(open_w * pupil_scale))
		pupil_h = max(2, int(open_h * pupil_scale))
		pupil_box = (
			center_x - pupil_w // 2,
			center_y - pupil_h // 2,
			center_x + pupil_w // 2,
			center_y + pupil_h // 2,
		)
		draw.ellipse(pupil_box, fill=255)

	if emotion == "wink":
		draw_eye(left_eye_x, eye_y, eye_w, eye_h)
		draw.line((right_eye_x - 5, eye_y, right_eye_x + 5, eye_y), fill=255)
	elif emotion == "sleepy":
		draw.line((left_eye_x - 6, eye_y, left_eye_x + 6, eye_y), fill=255)
		draw.line((right_eye_x - 6, eye_y, right_eye_x + 6, eye_y), fill=255)
		draw.line((left_eye_x - 2, eye_y + 1, left_eye_x + 2, eye_y + 1), fill=255)
		draw.line((right_eye_x - 2, eye_y + 1, right_eye_x + 2, eye_y + 1), fill=255)
	else:
		draw_eye(left_eye_x, eye_y, eye_w, eye_h, 0.4 if emotion != "surprised" else 0.55)
		draw_eye(right_eye_x, eye_y, eye_w, eye_h, 0.4 if emotion != "surprised" else 0.55)

	mouth_y = cy + 9
	mouth_y = cy + 14
	mouth_w = 28
	if emotion == "happy":
		draw.arc((cx - mouth_w, mouth_y - 8, cx + mouth_w, mouth_y + 10), start=20, end=160, fill=255)
	elif emotion == "excited":
		draw.arc((cx - mouth_w, mouth_y - 9, cx + mouth_w, mouth_y + 11), start=10, end=170, fill=255)
		draw.line((cx - 12, mouth_y + 1, cx + 12, mouth_y + 1), fill=255)
	elif emotion == "sad":
		draw.arc((cx - mouth_w, mouth_y - 1, cx + mouth_w, mouth_y + 14), start=200, end=340, fill=255)
	elif emotion == "angry":
		draw.line((cx - 14, mouth_y + 2, cx + 14, mouth_y + 2), fill=255)
		draw.line((cx - 11, mouth_y + 4, cx + 11, mouth_y + 4), fill=255)
	elif emotion == "surprised":
		draw.ellipse((cx - 8, mouth_y - 1, cx + 8, mouth_y + 12), outline=255)
	elif emotion == "sleepy":
		draw.line((cx - 12, mouth_y + 1, cx + 6, mouth_y + 1), fill=255)
	elif emotion == "wink":
		draw.arc((cx - 16, mouth_y - 6, cx + 16, mouth_y + 9), start=25, end=155, fill=255)
	else:
		draw.arc((cx - mouth_w, mouth_y - 2, cx + mouth_w, mouth_y + 8), start=25, end=155, fill=255)

	if emotion in {"happy", "excited"}:
		draw.point((cx - 22, cy + 8), fill=255)
		draw.point((cx + 22, cy + 8), fill=255)


def _pick_next_emotion(current: str) -> str:
	choices = [emotion for emotion in EMOTIONS if emotion != current]
	weights = {
		"happy": 3,
		"excited": 2,
		"neutral": 2,
		"sleepy": 1,
		"sad": 1,
		"angry": 1,
		"surprised": 2,
		"wink": 2,
	}
	return random.choices(choices, weights=[weights[emotion] for emotion in choices], k=1)[0]


def render_frame(frame: int, emotion: str, phase: float) -> Image.Image:
	img = Image.new("1", (WIDTH, HEIGHT), 0)
	draw = ImageDraw.Draw(img)

	_draw_top_eyebrows(draw, emotion, phase)

	bob = math.sin(phase) * 1.2
	sway = math.sin(phase * 0.63) * 0.8
	cx = 64 + int(sway)
	cy = 40 + int(bob)
	_draw_face_outline(draw, cx, cy, emotion, bob)

	return img


def main() -> None:
	serial = i2c(port=I2C_PORT, address=ADDR)
	device = None
	device = sh1106(serial)

	current_emotion = random.choice(EMOTIONS)
	next_change = time.monotonic() + random.uniform(1.4, 3.0)
	frame = 0
	last_emotion = None

	try:
		while True:
			now = time.monotonic()
			if now >= next_change:
				current_emotion = _pick_next_emotion(current_emotion)
				next_change = now + random.uniform(1.4, 3.0)

			phase = now * 2.2 + frame * 0.08
			img = render_frame(frame, current_emotion, phase)
			device.display(img)

			if current_emotion != last_emotion:
				print(f"[emote] {current_emotion}")
				last_emotion = current_emotion

			frame += 1
			time.sleep(0.08)
	except KeyboardInterrupt:
		pass
	finally:
		if device is not None:
			device.display(Image.new("1", (WIDTH, HEIGHT), 0))


if __name__ == "__main__":
	main()
