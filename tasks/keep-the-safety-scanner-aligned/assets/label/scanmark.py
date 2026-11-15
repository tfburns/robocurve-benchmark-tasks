"""Draw the scanner's printed floor mark: footprint outline, +-5 deg outlines, facing arrow with 5 deg ticks."""
import json, math, sys
from PIL import Image, ImageDraw, ImageFont
out, P = sys.argv[1], json.loads(sys.argv[2])
S = P["size"]; cx, cy = P["center"]; fx, fy = P["foot"]; yaw = math.radians(P["yaw_deg"])
W = H = 1024
img = Image.new("RGB", (W, H), (244, 240, 226)); d = ImageDraw.Draw(img)
def px(x, y):  # world -> pixel (image left = +y, top = +x)
    return ((cy + S / 2 - y) / S * W, (cx + S / 2 - x) / S * H)
def rect(a, grow=0.0):
    c, s = math.cos(a), math.sin(a)
    pts = [(-fx / 2 - grow, -fy / 2 - grow), (fx / 2 + grow, -fy / 2 - grow), (fx / 2 + grow, fy / 2 + grow), (-fx / 2 - grow, fy / 2 + grow)]
    return [px(cx + c * u - s * v, cy + s * u + c * v) for u, v in pts]
for da in (-5, 5):
    d.polygon(rect(yaw + math.radians(da), 0.002), outline=(150, 150, 150), width=3)
d.polygon(rect(yaw, 0.002), outline=(15, 15, 15), width=7)
R0, R1 = fx / 2 + 0.008, S / 2 - 0.012
ux, uy = math.cos(yaw), math.sin(yaw)
d.line([px(cx + ux * R0, cy + uy * R0), px(cx + ux * R1, cy + uy * R1)], fill=(200, 20, 20), width=10)
hx, hy = cx + ux * (R1 + 0.006), cy + uy * (R1 + 0.006)
for side in (-1, 1):
    a = yaw + side * math.radians(150)
    d.line([px(hx, hy), px(hx + 0.02 * math.cos(a), hy + 0.02 * math.sin(a))], fill=(200, 20, 20), width=10)
for da in (-5, 5):
    a = yaw + math.radians(da)
    d.line([px(cx + math.cos(a) * (R1 - 0.02), cy + math.sin(a) * (R1 - 0.02)),
            px(cx + math.cos(a) * (R1 + 0.004), cy + math.sin(a) * (R1 + 0.004))], fill=(15, 15, 15), width=6)
f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 44)
for i, t in enumerate(P.get("lines", [])):
    w = d.textlength(t, font=f); d.text(((W - w) / 2, H - 120 + i * 52), t, font=f, fill=(15, 15, 15))
d.rectangle([3, 3, W - 4, H - 4], outline=(15, 15, 15), width=6)
img.save(out); print(out)
