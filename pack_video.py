import argparse
import struct
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("--input", required=True, help="Path to phone .mp4/.mov clip")
parser.add_argument("--output", default="desk_test.bin")
parser.add_argument("--width", type=int, default=320)
parser.add_argument("--height", type=int, default=240)
parser.add_argument("--max-frames", type=int, default=450)
args = parser.parse_args()

cap = cv2.VideoCapture(args.input)
frames = []
while len(frames) < args.max_frames:
    ok, frame = cap.read()
    if not ok:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (args.width, args.height), interpolation=cv2.INTER_AREA)
    frames.append(resized.tobytes())
cap.release()

with open(args.output, "wb") as f:
    f.write(struct.pack("<III", args.width, args.height, len(frames)))
    for fr in frames:
        f.write(fr)

print(f"Packed {len(frames)} frames ({args.width}x{args.height}) -> {args.output}")