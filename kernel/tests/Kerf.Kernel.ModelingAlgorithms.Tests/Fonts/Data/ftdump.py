# Dumps every glyph's outline as FreeType decomposes it, in ems, unhinted, at a large pixel size.
# Usage: ftdump.py font face_index out.txt [max_glyphs]
import sys
import freetype

path, face_index, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
limit = int(sys.argv[4]) if len(sys.argv) > 4 else 1 << 30
face = freetype.Face(path, face_index)
upem = face.units_per_EM
ppem = 65535 if upem >= 256 else 16384
face.set_pixel_sizes(0, ppem)
scale = 1.0 / (64.0 * ppem)  # 26.6 pixels to ems
flags = freetype.FT_LOAD_NO_HINTING | freetype.FT_LOAD_NO_BITMAP
n = min(face.num_glyphs, limit)
latin = len(sys.argv) > 5 and sys.argv[5] == "latin"
indices = sorted({face.get_char_index(c) for c in list(range(0x20, 0x7F)) + list(range(0xA0, 0x100))} - {0}) if latin else range(n)
with open(out, "w") as f:
    f.write(f"font {upem} {face.num_glyphs} {face.ascender} {face.descender} {face.height}\n")
    for g in indices:
        try:
            face.load_glyph(g, flags)
        except freetype.FT_Exception as e:
            f.write(f"glyph {g} error {e}\n")
            continue
        o = face.glyph.outline
        adv = face.glyph.linearHoriAdvance / 65536.0 / ppem
        f.write(f"glyph {g} {face.glyph.metrics.horiAdvance} \n")
        state = {"cur": None, "start": None, "segs": []}
        contours = []

        def P(v):
            return (v.x * scale, v.y * scale)

        def flush():
            if state["start"] is not None:
                if state["cur"] != state["start"]:
                    state["segs"].append(("L", state["cur"], state["start"]))
                if state["segs"]:
                    contours.append(state["segs"])
            state["segs"] = []

        def move_to(a, ctx):
            flush()
            state["start"] = state["cur"] = P(a)
            return 0

        def line_to(a, ctx):
            p = P(a)
            if p != state["cur"]:
                state["segs"].append(("L", state["cur"], p))
            state["cur"] = p
            return 0

        def conic_to(a, b, ctx):
            p = P(b)
            state["segs"].append(("Q", state["cur"], P(a), p))
            state["cur"] = p
            return 0

        def cubic_to(a, b, c, ctx):
            p = P(c)
            state["segs"].append(("C", state["cur"], P(a), P(b), p))
            state["cur"] = p
            return 0

        if o.n_contours > 0:
            o.decompose(None, move_to=move_to, line_to=line_to, conic_to=conic_to, cubic_to=cubic_to)
            flush()
        for c in contours:
            f.write("contour\n")
            for s in c:
                f.write(s[0] + " " + " ".join(f"{x:.9f} {y:.9f}" for (x, y) in s[1:]) + "\n")
