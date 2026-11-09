# Makes the small fonts Kerf's font tests read (unit 13D): written by this script, so they are the project's own.
#   KerfTestCff.otf      CFF outlines: subroutines (local and global), hint masks, every curve operator, flex, GPOS kerning
#                        (a pair and a class pair)
#   KerfTestGlyf.ttf     TrueType: composites scaled, flipped, with scaled offsets, matched points, nested; a kern table
#   KerfTestStrokes.ttf  a single-stroke font: open strokes, as engraving fonts draw them
# Usage: maketestfonts.py out_folder
import sys
from fontTools.fontBuilder import FontBuilder
from fontTools.pens.t2CharStringPen import T2CharStringPen
from fontTools.pens.ttGlyphPen import TTGlyphPen
from fontTools.misc.psCharStrings import T2CharString
from fontTools.ttLib.tables._g_l_y_f import Glyph, GlyphComponent
from fontTools.feaLib.builder import addOpenTypeFeaturesFromString

out = sys.argv[1]


def names(family):
    return {"familyName": family, "styleName": "Regular", "uniqueFontIdentifier": family + " 1.0",
            "fullName": family + " Regular", "psName": family.replace(" ", "") + "-Regular", "version": "1.0"}


# ---- CFF ----
order = [".notdef", "space", "A", "B", "C", "D", "E", "V"]
fb = FontBuilder(1000, isTTF=False)
fb.setupGlyphOrder(order)
fb.setupCharacterMap({0x20: "space", 0x41: "A", 0x42: "B", 0x43: "C", 0x44: "D", 0x45: "E", 0x56: "V"})


def cs(program, private=None, globalSubrs=None):
    return T2CharString(program=program, private=private, globalSubrs=globalSubrs)


# The local subroutine draws a triangle's sides from the current point; the global one closes nothing, it moves
local_subrs = [
    [100, 600, "rlineto", 100, -600, "rlineto", "return"],  # bias 107: index -107
]
global_subrs = [
    [50, 50, "rmoveto", "return"],  # bias 107: index -107
]
pen_notdef = T2CharStringPen(500, None)
pen_notdef.moveTo((50, 0)); pen_notdef.lineTo((450, 0)); pen_notdef.lineTo((450, 700)); pen_notdef.lineTo((50, 700)); pen_notdef.closePath()
pen_notdef.moveTo((100, 50)); pen_notdef.lineTo((100, 650)); pen_notdef.lineTo((400, 650)); pen_notdef.lineTo((400, 50)); pen_notdef.closePath()
charstrings = {
    ".notdef": pen_notdef.getCharString().program,
    "space": [250, "endchar"],
    # A: width, two stems and a hint mask, a move through a global subroutine, a triangle through a local one, a hole
    "A": [600, 0, 50, 600, 50, "hstemhm", 100, 60, 400, 60, "vstemhm", "hintmask", bytes([0b11110000]),
          -107, "callgsubr", -107, "callsubr", "endchar"],
    # B: the curve operators
    "B": [100, 0, "rmoveto", 300, "hlineto", 100, 50, 50, 100, 0, 100, "rrcurveto",
          50, 100, 0, 100, "vvcurveto", -50, -100, -100, -50, "hhcurveto",
          -100, -50, -50, -100, "hvcurveto", -100, -50, 50, 100, "vhcurveto", "endchar"],
    # C: flex and its short forms
    "C": [100, 100, "rmoveto", 50, 20, 50, 20, 50, 0, 50, -20, 50, -20, 50, 0, 50, "flex",
          30, 40, 20, 30, 40, 30, 30, "hflex", 20, 20, 30, 20, 30, 30, -20, -20, 20, "hflex1",
          -50, 50, -50, 50, -50, 0, -50, -50, -50, -50, 40, "flex1", "endchar"],
    # D: lines and curves together, and the arithmetic operators putting a number on the stack
    "D": [50, 0, "rmoveto", 200, 100, "vlineto", 100, 100, 50, 50, 100, 100, 50, 50, "rcurveline",
          100, 0, 0, 100, 50, 50, 50, 50, 0, 100, "rlinecurve", 3, 4, "add", 2, "mul", -14, "exch", "rlineto", "endchar"],
    # E: two contours, the second through a hint mask after implicit stems
    "E": [50, 0, "rmoveto", 400, 0, 0, 600, -400, 0, "rlineto", 100, 100, "rmoveto", 200, 0, 0, 400, -200, 0, "rlineto", "endchar"],
    "V": [0, 700, "rmoveto", 300, -700, 300, 700, -100, 0, -200, -500, -200, 500, "rlineto", "endchar"],
}
fb.setupCFF(names("Kerf Test CFF")["psName"], {"FullName": "Kerf Test CFF"}, {k: T2CharString(program=list(v)) for k, v in charstrings.items()}, {})
cff = fb.font["CFF "].cff
top = cff.topDictIndex[0]
priv = top.Private
from fontTools.cffLib import SubrsIndex, GlobalSubrsIndex
subrs = SubrsIndex()
for s in local_subrs:
    subrs.append(cs(s, private=priv, globalSubrs=cff.GlobalSubrs))
priv.Subrs = subrs
for s in global_subrs:
    cff.GlobalSubrs.append(cs(s, private=priv, globalSubrs=cff.GlobalSubrs))
for name, program in charstrings.items():
    top.CharStrings[name].program = program
    top.CharStrings[name].private = priv
    top.CharStrings[name].globalSubrs = cff.GlobalSubrs
fb.setupHorizontalMetrics({g: (600 if g not in ("space",) else 250, 0) for g in order})
fb.setupHorizontalHeader(ascent=800, descent=-200)
fb.setupNameTable(names("Kerf Test CFF"))
fb.setupOS2(sTypoAscender=800, sTypoDescender=-200, usWinAscent=800, usWinDescent=200)
fb.setupPost()
addOpenTypeFeaturesFromString(fb.font, """
@LEFT = [A V];
@RIGHT = [V B];
feature kern {
  pos A V -80;
  pos @LEFT @RIGHT -30;
} kern;
""")
fb.font.recalcBBoxes = False
fb.save(f"{out}/KerfTestCff.otf")

# The same with a subroutine that calls itself, and one glyph calling it
loop = cff.topDictIndex[0].Private.Subrs[0]
loop.program = [-107, "callsubr", "return"]
loop.bytecode = None
fb.save(f"{out}/KerfTestCffLoop.otf")

# ---- TrueType with composites ----
order = [".notdef", "space", "o", "bar", "O", "Q", "W", "M", "Z"]
fb = FontBuilder(2048, isTTF=True)
fb.setupGlyphOrder(order)
fb.setupCharacterMap({0x20: "space", 0x6F: "o", 0x7C: "bar", 0x4F: "O", 0x51: "Q", 0x57: "W", 0x4D: "M", 0x5A: "Z"})


def ring(pen, cx, cy, r1, r2):
    k = 0.5523 * 2  # a quadratic's control at the corner of the square
    for r, flip in ((r1, False), (r2, True)):
        pts = [(cx + r, cy), (cx + r, cy + r), (cx, cy + r), (cx - r, cy + r), (cx - r, cy), (cx - r, cy - r), (cx, cy - r), (cx + r, cy - r)]
        if flip:
            pts = list(reversed(pts))
            pts = pts[-1:] + pts[:-1]
        pen.moveTo(pts[0])
        for i in range(0, 8, 2):
            pen.qCurveTo(pts[(i + 1) % 8], pts[(i + 2) % 8])
        pen.closePath()


glyphs = {}
p = TTGlyphPen(None); p.moveTo((100, 0)); p.lineTo((100, 700)); p.lineTo((500, 700)); p.lineTo((500, 0)); p.closePath(); glyphs[".notdef"] = p.glyph()
glyphs["space"] = Glyph()
p = TTGlyphPen(None); ring(p, 500, 500, 400, 250); glyphs["o"] = p.glyph()
p = TTGlyphPen(None); p.moveTo((100, -200)); p.lineTo((100, 1500)); p.lineTo((250, 1500)); p.lineTo((250, -200)); p.closePath(); glyphs["bar"] = p.glyph()
# Z: a contour that crosses itself, a bow tie, which makes no valid face
p = TTGlyphPen(None); p.moveTo((100, 0)); p.lineTo((900, 1400)); p.lineTo((900, 0)); p.lineTo((300, 1000)); p.closePath(); glyphs["Z"] = p.glyph()


def composite(parts):
    g = Glyph()
    g.numberOfContours = -1
    g.components = []
    for (name, x, y, flags, transform) in parts:
        c = GlyphComponent()
        c.glyphName = name
        c.x, c.y = x, y
        c.flags = flags
        if transform is not None:
            c.transform = transform
        g.components.append(c)
    return g


# O: o scaled 1.5, its offset scaled too (SCALED_COMPONENT_OFFSET 0x800), and a bar flipped by a 2x2
glyphs["O"] = composite([("o", 100, 0, 0x800, [[1.5, 0], [0, 1.5]]), ("bar", 1800, 0, 0, [[-1, 0], [0, 1]])])
# W: two o's with separate x and y scales and a 2x2 with shear; USE_MY_METRICS (0x200) on the first
glyphs["W"] = composite([("o", 0, 0, 0x200, [[0.75, 0], [0, 1.25]]), ("o", 900, 0, 0, [[1, 0.25], [0, 1]])])
# M: nested composites: W, then O
glyphs["M"] = composite([("W", 0, 0, 0, None), ("O", 1200, 0, 0, None)])
fb.setupGlyf(glyphs)
glyf = fb.font["glyf"]
# Q: o, then a bar whose point 0 meets o's point 2 (matched points, ARGS_ARE_XY_VALUES clear)
q = composite([("o", 0, 0, 0, None), ("bar", 2, 0, 0, None)])
for c in q.components:
    if c.glyphName == "bar":
        c.firstPt, c.secondPt = 2, 0
        del c.x, c.y
glyf["Q"] = q
fb.setupHorizontalMetrics({g: (1200, glyf[g].xMin if hasattr(glyf[g], "xMin") else 0) for g in order})
glyf.recalcBounds = True
fb.setupHorizontalHeader(ascent=1800, descent=-400)
fb.setupNameTable(dict(names("Kerf Test Glyf"), typographicFamily="Kerf Test", typographicSubfamily="Wide"))
fb.setupOS2(sTypoAscender=1800, sTypoDescender=-400, usWinAscent=1800, usWinDescent=400)
fb.setupPost()
from fontTools.ttLib import newTable
from fontTools.ttLib.tables._k_e_r_n import KernTable_format_0
kern = newTable("kern"); kern.version = 0
st = KernTable_format_0(); st.version = 0; st.coverage = 1; st.format = 0
st.kernTable = {("o", "bar"): -100, ("O", "o"): -40}
kern.kernTables = [st]
fb.font["kern"] = kern
fb.save(f"{out}/KerfTestGlyf.ttf")

# ---- single stroke ----
order = [".notdef", "space", "I", "L", "N"]
fb = FontBuilder(1000, isTTF=True)
fb.setupGlyphOrder(order)
fb.setupCharacterMap({0x20: "space", 0x49: "I", 0x4C: "L", 0x4E: "N"})
glyphs = {}
p = TTGlyphPen(None); p.moveTo((100, 0)); p.lineTo((100, 700)); p.lineTo((400, 700)); p.closePath(); glyphs[".notdef"] = p.glyph()
glyphs["space"] = Glyph()
# Open strokes: a TrueType contour is closed, so each stroke is drawn as the font draws it, its close left out
p = TTGlyphPen(None); p.moveTo((300, 0)); p.lineTo((300, 700)); p.closePath(); glyphs["I"] = p.glyph()
p = TTGlyphPen(None); p.moveTo((100, 700)); p.lineTo((100, 0)); p.lineTo((450, 0)); p.closePath(); glyphs["L"] = p.glyph()
p = TTGlyphPen(None); p.moveTo((100, 0)); p.lineTo((100, 700)); p.qCurveTo((300, 700), (300, 350)); p.lineTo((500, 0)); p.lineTo((500, 700)); p.closePath(); glyphs["N"] = p.glyph()
fb.setupGlyf(glyphs)
fb.setupHorizontalMetrics({g: (600 if g != "space" else 300, 0) for g in order})
fb.setupHorizontalHeader(ascent=800, descent=-200)
fb.setupNameTable(names("Kerf Test SingleLine"))
fb.setupOS2(sTypoAscender=800, sTypoDescender=-200, usWinAscent=800, usWinDescent=200)
fb.setupPost()
fb.save(f"{out}/KerfTestStrokes.ttf")
print("ok")
