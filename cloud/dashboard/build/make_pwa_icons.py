"""Generate PWA + Apple touch icons at the resolutions iOS / Android look for.

Writes into frontend/public/ so Vite copies them into the production bundle.
"""
from pathlib import Path

from PIL import Image, ImageDraw

OUT = Path(__file__).parent.parent / "frontend" / "public"
OUT.mkdir(parents=True, exist_ok=True)

SIZES = [
    ("pwa-192.png",         192, False),
    ("pwa-512.png",         512, False),
    ("pwa-512-maskable.png",512, True),   # safe zone for adaptive icons
    ("apple-touch-icon.png",180, False),
]

BG = (11, 15, 22, 255)
FIRE_TOP = (251, 191, 36, 255)
FIRE_BOT = (194, 65, 12, 255)


def flame(size: int, safe_pad: bool) -> list[tuple[float, float]]:
    s = size / 32
    pad = 0.15 * size if safe_pad else 0  # maskable safe zone
    scale = (size - 2 * pad) / size
    pts = [
        (16, 4), (10, 12), (22, 14), (16, 20),
        (22, 24), (8, 26), (16, 28), (8, 24),
        (12, 18), (10, 14), (12, 16), (14, 12),
    ]
    return [(pad + x * s * scale, pad + y * s * scale) for x, y in pts]


def gradient(w: int, h: int, top, bot) -> Image.Image:
    img = Image.new("RGBA", (w, h))
    for y in range(h):
        t = y / max(1, h - 1)
        r = int(top[0] + (bot[0] - top[0]) * t)
        g = int(top[1] + (bot[1] - top[1]) * t)
        b = int(top[2] + (bot[2] - top[2]) * t)
        for x in range(w):
            img.putpixel((x, y), (r, g, b, 255))
    return img


def make(size: int, maskable: bool) -> Image.Image:
    img = Image.new("RGBA", (size, size), BG)
    d = ImageDraw.Draw(img)
    corner = 0 if maskable else max(2, size // 5)
    d.rounded_rectangle([0, 0, size - 1, size - 1], radius=corner, fill=BG)
    mask = Image.new("L", (size, size), 0)
    ImageDraw.Draw(mask).polygon(flame(size, maskable), fill=255)
    img.paste(gradient(size, size, FIRE_TOP, FIRE_BOT), (0, 0), mask=mask)
    return img


def main() -> None:
    for name, size, maskable in SIZES:
        p = OUT / name
        make(size, maskable).save(p, "PNG")
        print("wrote", p)


if __name__ == "__main__":
    main()
