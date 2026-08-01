"""Generate the FPMS logo: Windows .ico plus the web/PWA icons.

Design notes, so a future change keeps what matters:

  * A FLAME for what the rover looks for, sitting inside a SCAN ARC for how it
    looks. Those two ideas are the whole product, so the mark is those two
    shapes and nothing else.
  * It has to survive 16x16 in a taskbar and a browser tab. That rules out
    anything with fine detail, text, or a thin outline -- at that size a flame
    silhouette is roughly nine pixels of solid colour, so the silhouette is
    what gets designed and everything else is drawn around it.
  * Ember orange (#f97316) on near-black, matching the dashboard accent, so the
    app icon and the UI read as one product.

Run:  python build/make_logo.py
"""
from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
PUBLIC = ROOT / "frontend" / "public"

BG = (11, 15, 22, 255)        # #0b0f16 - dashboard floor colour
EMBER = (249, 115, 22, 255)   # #f97316 - dashboard accent
EMBER_HOT = (253, 186, 116, 255)
ARC = (56, 189, 248, 255)     # #38bdf8 - scan/measured cyan

# Supersample, then downsample once at the end. Drawing straight to 32px gives
# jagged diagonals; drawing at 8x and reducing gives clean ones for free.
SS = 8


def _flame(d: ImageDraw.ImageDraw, cx: float, cy: float, h: float, colour) -> None:
    """A teardrop flame, centred on (cx, cy), h tall.

    Built from a polygon rather than arcs because the silhouette is the thing
    that has to read at 16px, and a polygon is exactly controllable.
    """
    w = h * 0.66

    def half(sign: int):
        pts = []
        for i in range(61):
            t = i / 60                      # 0 at the TIP (top), 1 at the base
            y = cy - h / 2 + h * t
            # The exponent decides where the widest point sits, and it is the
            # whole difference between a flame and a map pin: an exponent below
            # 1 bulges near the top and tapers to a point at the BOTTOM, which
            # reads as a teardrop hanging down. Above 1 the bulge is pushed
            # late, giving a sharp tip and a broad base -- a flame.
            bulge = math.sin((t ** 1.7) * math.pi) ** 0.7
            # Round the base off instead of letting it pinch back to a point.
            if t > 0.82:
                # Clamped: t reaches 1.0 exactly, and float division can push k
                # a hair past it, which makes 1 - k*k negative and the sqrt
                # complex rather than raising anything useful.
                k = min(1.0, (t - 0.82) / 0.18)
                bulge = max(bulge, math.sqrt(max(0.0, 1 - k * k)) * 0.92)
            x = cx + sign * (w / 2) * bulge
            x += (1 - t) ** 2 * h * 0.10    # lean, so it looks like it is moving
            pts.append((x, y))
        return pts

    d.polygon(half(1) + list(reversed(half(-1))), fill=colour)


def render(px: int, *, rounded: bool = True, bg: bool = True) -> Image.Image:
    s = px * SS
    img = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    if bg:
        r = int(s * 0.22)
        if rounded:
            d.rounded_rectangle([0, 0, s - 1, s - 1], radius=r, fill=BG)
        else:
            d.rectangle([0, 0, s - 1, s - 1], fill=BG)

    # Scan arc: two sweeps opening upward, echoing a LiDAR fan.
    for frac, width, colour in ((0.80, 0.055, ARC), (0.62, 0.045, ARC)):
        box = [s * (0.5 - frac / 2), s * (0.5 - frac / 2) + s * 0.06,
               s * (0.5 + frac / 2), s * (0.5 + frac / 2) + s * 0.06]
        d.arc(box, start=205, end=335, fill=colour, width=max(1, int(s * width)))

    _flame(d, s * 0.5, s * 0.545, s * 0.54, EMBER)
    _flame(d, s * 0.5, s * 0.655, s * 0.26, EMBER_HOT)

    return img.resize((px, px), Image.LANCZOS)


def main() -> None:
    # .ico carries every size Windows asks for; letting it generate them from
    # one bitmap produces mush at 16px, so each size is rendered at its own
    # resolution and only then packed.
    sizes = [16, 24, 32, 48, 64, 128, 256]
    frames = [render(n) for n in sizes]
    ico = BUILD / "fpms.ico"
    frames[-1].save(ico, format="ICO",
                    sizes=[(n, n) for n in sizes],
                    append_images=frames[:-1])
    print(f"  {ico}  ({ico.stat().st_size} bytes, {len(sizes)} sizes)")

    PUBLIC.mkdir(parents=True, exist_ok=True)
    for name, px, rounded in (("pwa-192.png", 192, True),
                              ("pwa-512.png", 512, True),
                              ("pwa-512-maskable.png", 512, False),
                              ("apple-touch-icon.png", 180, True)):
        p = PUBLIC / name
        render(px, rounded=rounded).save(p, format="PNG")
        print(f"  {p}  ({p.stat().st_size} bytes)")

    # SVG is hand-written rather than traced: it stays crisp at any size and
    # small enough to inline, and the flame is the same silhouette as above.
    svg = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" role="img" aria-label="FPMS">
  <rect width="64" height="64" rx="14" fill="#0b0f16"/>
  <g fill="none" stroke="#38bdf8" stroke-linecap="round">
    <path d="M10 39a22 22 0 0 1 44 0" stroke-width="3" opacity=".95"/>
    <path d="M17 40a15 15 0 0 1 30 0" stroke-width="2.4" opacity=".7"/>
  </g>
  <path d="M33 13c-1 7-5 10-8 14-3 4-5 7-5 11a12 12 0 0 0 24 0c0-6-5-9-8-13-2-3-3-6-3-12z" fill="#f97316"/>
  <path d="M32.5 34c-.6 3.4-2.6 5-3.8 6.9-1 1.6-1.5 2.8-1.5 4.3a5.3 5.3 0 0 0 10.6 0c0-2.4-1.8-3.9-3.2-5.6-1.2-1.5-2-2.9-2.1-5.6z" fill="#fdba76"/>
</svg>
"""
    p = PUBLIC / "favicon.svg"
    p.write_text(svg, encoding="utf-8")
    print(f"  {p}  ({p.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
