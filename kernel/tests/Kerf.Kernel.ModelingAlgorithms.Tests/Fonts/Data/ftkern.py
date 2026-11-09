# FreeType's kerning (FT_Get_Kerning, unscaled: the kern table) for every pair of Latin-1 characters with a non-zero value.
# freetype-py's get_kerning takes characters and finds their glyphs itself.
import sys
import freetype
face = freetype.Face(sys.argv[1])
chars = list(range(0x20, 0x7F)) + list(range(0xA0, 0x100))
with open(sys.argv[2], "w", newline="\n") as f:
    for a in chars:
        for b in chars:
            k = face.get_kerning(chr(a), chr(b), freetype.FT_KERNING_UNSCALED)
            if k.x:
                f.write(f"{a:04X} {b:04X} {k.x}\n")
