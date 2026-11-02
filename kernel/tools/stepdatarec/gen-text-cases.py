# Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
#
# Writes text-cases.txt: the quoted STEP texts (one per line, as hex bytes) on which tools/stepdatarec's "text" mode
# and the port's StepDataNativeTests run StepData_StepReaderData's text decoding (cleanText), under every code page.
#   python -I tools/stepdatarec/gen-text-cases.py <out>
# Texts are written with "/" for the backslash, then converted.
import random, sys

cases = []
def add(b):
    if isinstance(b, str):
        b = b.replace("/", "\\").encode('latin-1')
    cases.append(b"'" + b + b"'")

for t in ["", "a", "abc", "it''s", "a//b", "x/N/y", "x/T/y", "a/Nb", "/N/", "/T//N/",
          "/X/E9/", "/X/41/", "/X/zz/", "/X/e9/x", "end/X/", "/X/00/", "/X/7f/",
          "/X2/00E9/X0/", "/X2/00E900E8/X0/", "/X2/00E9", "/X2/00E/X0/", "/X2/0E9/X0/",
          "/X2/00e9/X0/", "/X2/D83DDE00/X0/", "/X2/D83D/X0/", "/X2/DE00D83D/X0/", "/X2/0000/X0/",
          "a/X2/00E9/X0/b/X2/00E8/X0/c", "/X2//X0/", "/X2/00E9/X0", "/X2/04100411/X0/",
          "/X4/0001F600/X0/", "/X4/00110000/X0/", "/X4/000000E9000000E8/X0/", "/X4/00E9/X0/",
          "/X4/0000D800/X0/", "/X4/00000000/X0/", "/X4/FFFFFFFF/X0/",
          "/S/a", "/S/z", "/PA//S/a", "/PB//S/a", "/PE//S/a", "/PI//S/i", "/PZ/x", "/pa//S/a",
          "/S/", "/P", "a/", "//X2/00E9/X0/", "''/X/E9/''", "x/X0/y", "/X3/00E9/X0/",
          "line1\nline2", "cr\rlf", "/N//N//T/", "//N//", "q''''q", "////"]:
    add(t)

for b in [b"\xe9", b"caf\xe9", b"caf\xc3\xa9", b"\xc3", b"\xe9AA", b"\xe9A", b"\xf8\x88\x80\x80\x80", b"\xff",
          b"\xc0\x80x", b"\xf0\x9f\x98\x80", b"\xf4\x90\x80\x80", b"\xed\xa0\x80", b"\xed\xa0\x80\xed\xb0\x80",
          b"\x82\xa0", b"\x93\xfa\x96\x7b", b"\xa4\xa2\xa4\xa4", b"\xb0\xa1\xb0\xa2", b"\x81\x40", b"\x81\x30\x81\x30",
          b"\x81\x30\x81\x30\x31", b"\xa4\x40\xa4\x41", b"\x88\x62", b"\x88\x64", b"\x88\xa3", b"\x88\xa5",
          b"\xa4\x40a", b"\x81a", b"\x80", b"\x80\x81", b"\xa1\xa1", b"\x8e\xb1", b"\x82", b"x\x82",
          b"\xe9\x5cX2\x5c00E9\x5cX0\x5c\xe9", b"\x5cS\x5c\xe9", b"\xc3\xa9''\x5cN\x5c", b"\x8f\xa1\xa1", b"\xfe\xfe",
          b"\xa0\xa0"]:
    add(b)

rnd = random.Random(8)
alphabet = [b"a", b"'", b"''", b"\x5c", b"\x5c\x5c", b"\x5cX\x5c", b"\x5cX2\x5c", b"\x5cX4\x5c", b"\x5cX0\x5c",
            b"\x5cS\x5c", b"\x5cPA\x5c", b"\x5cN\x5c", b"\x5cT\x5c", b"0", b"9", b"E", b"f", b"\xe9", b"\xc3\xa9",
            b"\x82\xa0", b"\x81\x40", b"\xa4\x40", b"\n", b" "]
for i in range(150):
    add(b"".join(rnd.choice(alphabet) for _ in range(rnd.randint(1, 12))))

with open(sys.argv[1], "w", newline="\n") as out:
    for c in cases:
        out.write(c.hex() + "\n")
