"""Generate the FPMS app icon (multi-resolution .ico).

Draws the same fire-emblem the UI uses, at every resolution Windows Explorer
and the taskbar look for. Output: cloud/dashboard/build/fpms.ico
"""
from pathlib import Path

from PIL import Image, ImageDraw

OUT = Path(__file__).parent / "fpms.ico"
SIZES = [16, 20, 24, 32, 40, 48, 64, 96, 128, 256]

BG = (11, 15, 22, 255)          # ink-900
FIRE_TOP = (251, 191, 36, 255)  # amber
FIRE_BOT = (194, 65, 12, 255)   # ember-700


def flame_polygon(size: int) -> list[tuple[float, float]]:
    """Rough flame path scaled to `size`. Same silhouette as the SVG in the UI."""
    s = size / 32
    pts = [
        (16, 4), (10, 12), (22, 14), (16, 20),
        (22, 24), (8, 26), (16, 28), (8, 24),
        (12, 18), (10, 14), (12, 16), (14, 12),
    ]
    return [(x * s, y * s) for x, y in pts]


def draw_gradient(img: Image.Image, top: tuple, bot: tuple) -> Image.Image:
    """Vertical gradient mask applied to the flame silhouette."""
    w, h = img.size
    grad = Image.new("RGBA", (w, h))
    for y in range(h):
        t = y / max(1, h - 1)
        r = int(top[0] + (bot[0] - top[0]) * t)
        g = int(top[1] + (bot[1] - top[1]) * t)
        b = int(top[2] + (bot[2] - top[2]) * t)
        for x in range(w):
            grad.putpixel((x, y), (r, g, b, 255))
    return grad


def make_frame(size: int) -> Image.Image:
    img = Image.new("RGBA", (size, size), BG)
    d = ImageDraw.Draw(img)
    # Rounded square background (subtle)
    d.rounded_rectangle([0, 0, size - 1, size - 1], radius=max(2, size // 5), fill=BG)
    # Flame
    poly = flame_polygon(size)
    mask = Image.new("L", (size, size), 0)
    ImageDraw.Draw(mask).polygon(poly, fill=255)
    grad = draw_gradient(Image.new("RGBA", (size, size)), FIRE_TOP, FIRE_BOT)
    img.paste(grad, (0, 0), mask=mask)
    return img


def main() -> None:
    frames = [make_frame(s) for s in SIZES]
    frames[0].save(OUT, format="ICO", sizes=[(s, s) for s in SIZES], append_images=frames[1:])
    print(f"wrote {OUT}  ({len(SIZES)} sizes)")


if __name__ == "__main__":
    main()
