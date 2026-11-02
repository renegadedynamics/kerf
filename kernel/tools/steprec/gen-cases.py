# Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
#
# Writes the hostile and mutated STEP inputs for unit 8P's parser tests, as a gzipped pack (entries
# "<name>\n<length>\n<bytes>"): tests/OcctNet.DataExchange.Tests/StepFile/Recordings/cases.pack.gz, which
# tools/steprec/build.ps1 records natively. Deterministic (fixed seeds).
#
#   python -I tools/steprec/gen-cases.py <OCCT source dir> <out cases.pack.gz>
#
# The cases: the scanner's rules and their ties one by one; structures and syntax errors (the parser's recovery);
# tokens around the 16,383 bytes OCCT's scanner can hold, at several offsets in its 8,192-byte reads; deep nesting;
# binary junk; truncations and token-level mutations of a synthetic file and of the start of OCCT's data/step/screw.step.
import gzip
import os
import random
import sys

src_dir, out_path = sys.argv[1], sys.argv[2]
cases = []


def add(name, data):
    if isinstance(data, str):
        data = data.encode('latin-1')
    cases.append((name, data))


HEAD = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('d'),'2;1');\nFILE_NAME('n','t',('a'),('o'),'p','s','z');\n"
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n")
TAIL = "ENDSEC;\nEND-ISO-10303-21;\n"


def wrap(body):
    return HEAD + body + TAIL


# 1. The scanner's rules, each in an argument list
tokens = ["1", "-1", "+12", "-", "+", ".", "..", "...", "1.", "1.5", "-.5", "+.5E-3", "1.5E", "1.5E+", "1.5e3", "1E3",
          "12E3", "1.E3", "1..2", "1.5.", ".T.", ".t.", "T.", "ABC.", "..X.", ".1.", "1A.", "_.", "\"0F\"",
          "\"0f\"", "\"\"", "\"0G\"", "\"12", "$", "*", "?", "#", "#12", "#012", "!", "!abc", "!_9", "&SCOPE",
          "&scope", "&SCOP", "&", "endscope", "ENDSCOPEX", "FOO", "foo_bar9", "_", "9abc", "a-b", "@", "\\", "%",
          "{", "}", "[", "]", "|", "~", "^", "`", "<", ">", ":", "\f", "\v", "\x80", "\xe9", "\xff", "\x01", "'a'",
          "''", "'a''b'", "'it''s'", "'a'\"\"", "'a\nb'", "'a\rb'", "'a\x00b'", "'\\X2\\00E9\\X0\\'", "'\\S\\a'",
          "STEP", "DATA", "HEADER", "ENDSEC", "ENDSTEP", "ISO", "SCOPE", "ENDSEC.", "DATA.", "e", "E", "1e", "0x1F"]
for i, t in enumerate(tokens):
    add(f"rule/{i:03d}", wrap(f"#1=FOO({t});\n#2=BAR(1,{t},2);\n#3=BAZ(({t}),{t});\n"))
# Strings' endings: the trailing context [" "\n\r]*[),] (with the double quote, without the tab)
for i, t in enumerate(["'a' )", "'a'\n)", "'a'\r\n,1)", "'a' \t,1)", "'a'\"),", "'a' ' )", "'a'b'c' )", "'a';#5=X(1)",
                       "'a'' )", "'a''', 1)", "'/* x */')", "'a'/* c */)", "'a\n\n\nb' , 1)"]):
    add(f"string/{i:03d}", wrap(f"#1=FOO({t};\n#2=BAR(2);\n"))
# Comments
for i, t in enumerate(["/* c */#1=A(1);", "#1=A(/* c */1);", "#1=A(1/**/);", "#1=A(1);/***/", "#1=A(1);/* * / */",
                       "#1=A(1);/* ** */#2=B(2);", "#1=A(1);/*\n\n*/#2=B(2);", "#1=A('/* x */');",
                       "#1=A(1);/* unterminated", "#1=A(1);/*/ #2=B(2); */", "#1=A(1);/ * */", "#1=A(1);//#2=B(2);"]):
    add(f"comment/{i:03d}", wrap(t + "\n"))
# Keywords, labels, blanks, line ends
for i, t in enumerate([
        "iso-10303-21;\nheader;\nfile_name('x');\nendsec;\ndata;\n#1=a(1);\nendsec;\nend-iso-10303-21;\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO-10303-21;\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;junk\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;\njunk\n",
        "ISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;\njunk\n",
        "ISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\n",
        "ISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n",
        "ISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\n#2=B(\n",
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nfoo bar\n",
        "ISO-10303-21;\r\nHEADER;\r\nX('a');\r\nENDSEC;\r\nDATA;\r\n#1 = A ( 1 , 2 ) ;\r\nENDSEC;\r\nEND-ISO-10303-21;\r\n",
        "ISO-10303-21;HEADER;X(1);ENDSEC;DATA;#1=A(1);#2\t=\tB(#1);ENDSEC;END-ISO-10303-21;",
        "ISO-10303-21;\nHEADER ;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO-10303-21;\n",
        "ISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1\n=A(1);\nENDSEC;\nEND-ISO-10303-21;\n",
        "ISO;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO;\n",
        "STEP;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO-10303-21;garbage\nmore",
        "\x00\x00ISO-10303-21;\nHEADER;\nX(1)\x00;\nENDSEC;\nDATA;\n#1=A(1);\x00\x00\x00\nENDSEC;\nEND-ISO-10303-21;\n",
        "\xef\xbb\xbfISO-10303-21;\nHEADER;\nX(1);\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO-10303-21;\n",
        "", "ISO-10303-21;", "ISO-10303-21;\nHEADER;", "HEADER;\nENDSEC;", "garbage",
]):
    add(f"file/{i:03d}", t)
# Structures: complex entities, sub-lists, typed lists, scopes, export lists, and their errors
for i, t in enumerate([
        "#1=(A(1)B(2,(3,4))C());", "#1=(A(1));", "#1=();", "#1=(A(1)B(2);", "#1=(A(1)2);", "#1=(1);", "#1=((A(1)));",
        "#1=A(B(1),C((2)),D(E(3)));", "#1=A(((((1)))));", "#1=A((),(()),((),()));", "#1=A(1,,2);", "#1=A(,);",
        "#1=A(1,);", "#1=A(1 2);", "#1=A(1)(2);", "#1=A(1));", "#1=A((1);", "#1=A;", "#1=;", "#1 A(1);", "#1==A(1);",
        "=A(1);", "A(1);", "#1=A(1)\n#2=B(2);", "#1=A(1);;#2=B(2);", "#1=A(#2=B(1));", "#1=A(B);", "#1=A(B C(1));",
        "#1=A(?x y z,2);", "#1=A(?x,y);", "#1=A(1,?x)#2=B(1);", "#1=A(CARTESIAN_POINT,?q CARTESIAN_POINT r);",
        "#1=CARTESIAN_POINT('',(0.,0.,0.));#2=CARTESIAN_POINT('',(1.,1.,1.));",
        "#1=&SCOPE\n#2=A(1);\n#3=B(#2);\nENDSCOPE\nC(#3);",
        "#1=&SCOPE\nENDSCOPE\nC(1);",
        "#1=&SCOPE\n#2=A(1);\nENDSCOPE/#2/C(#2);",
        "#1=&SCOPE\n#2=A(1);\n#3=A(2);\nENDSCOPE/#2,#3/C(#2);",
        "#1=&SCOPE\n#2=&SCOPE\n#3=A(1);\nENDSCOPE\nB(#3);\nENDSCOPE\nC(#2);",
        "#1=&SCOPE\n#2=A(1);\nENDSCOPE\n(C(1)D(2));",
        "#1=&SCOPE\n#2=A(1);\nENDSCOPE\n1;\n#4=E(1);ENDSCOPE C(1);",
        "#1=&SCOPE\n#2=A(1);\nENDSCOPE/#2 #3/C(1);",
        "#1=&SCOPE\n#2=A(1);\nENDSCOPE/ /C(1);",
        "#1=&SCOPE\n#2=A(1;\nENDSCOPE\nC(1);",
        "#1=&SCOPE\n#2=A(1);\n",
        "#1=&SCOPE\nfoo\nENDSCOPE\nC(1);\n#9=Z(9);",
        "ENDSCOPE\n#1=A(1);",
        "#1=A(1);\nENDSCOPE\nX(1);",
        "#1=A(1);\n#2=&SCOPE ENDSCOPE / #1 / B(2);",
        "#1=A('x');\n#2=B(\"0F\",.E.,$,*,#1,1.5,-2);",
]):
    add(f"struct/{i:03d}", wrap(t + "\n"))
    add(f"struct-nohead/{i:03d}", "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n" + t + "\n" + TAIL)
# The header's errors
for i, t in enumerate(["X(1)", "X(1);;", "X 1;", "(X(1));", "#1=X(1);", "X(1);ENDSEC;", "X(1); ENDSEC ; DATA",
                       "X(1);ENDSEC;#1=A(1);", "X((1),'a',$);Y();", "X(1);DATA;", "'a';X(1);"]):
    add(f"header/{i:03d}", f"ISO-10303-21;\nHEADER;\n{t}\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nEND-ISO-10303-21;\n")

# 2. Long tokens around the 16,383 bytes of OCCT's buffer, at offsets around its 8,192-byte reads
for pad in [0, 1, 100, 8000, 8191, 8192, 8193, 16380, 16384, 20000]:
    padding = "\n" * (pad % 7) + " " * (pad - pad % 7)
    for n in [16378, 16379, 16380, 16381, 16382, 16383, 16384]:
        s = "x" * (n - 2)  # a string of n bytes with its quotes
        add(f"long/str-{pad}-{n}", HEAD + padding + f"#1=A('{s}',1);\n#2=B(2);\n" + TAIL)
    for n in [16381, 16382, 16383, 16384]:
        add(f"long/type-{pad}-{n}", HEAD + padding + "#1=" + "T" * n + "(1);\n#2=B(2);\n" + TAIL)
for n in [16380, 16381, 16382, 16383, 16384, 16385, 40000]:
    add(f"long/str-eof-{n}", HEAD + "#1=A('" + "y" * (n - 1))
    add(f"long/str-nl-{n}", HEAD + "#1=A('" + ("abcdefghi\n" * (n // 10)) + "',1);\n" + TAIL)
    add(f"long/str-quotes-{n}", HEAD + "#1=A('" + ("ab''" * (n // 4)) + "',1);\n" + TAIL)
    add(f"long/int-{n}", wrap("#1=A(" + "1" * n + ");\n"))
    add(f"long/real-{n}", wrap("#1=A(1." + "5" * n + ");\n"))
    add(f"long/enum-{n}", wrap("#1=A(." + "E" * n + ".);\n"))
    add(f"long/user-{n}", wrap("#1=!" + "U" * n + "(1);\n"))
    add(f"long/hex-{n}", wrap("#1=A(\"" + "F" * n + "\");\n"))
    add(f"long/comment-{n}", wrap("/*" + "c" * n + "*/#1=A(1);\n"))
    add(f"long/comment-stars-{n}", wrap("/*" + "*" * n + "/#1=A(1);\n"))
    add(f"long/nul-{n}", wrap("#1=A(1);" + "\x00" * n + "#2=B(2);\n"))
    add(f"long/label-blanks-{n}", wrap("#1" + " " * n + "=A(1);\n"))
    add(f"long/string-trail-{n}", wrap("#1=A('a'" + " " * n + ",1);\n"))
    add(f"long/endstep-{n}", "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=A(1);\nENDSEC;\nENDSTEP;" + "j" * n + "\n")
    add(f"long/after-end-{n}", wrap("#1=A(1);\n") + "z" * n)
    add(f"long/blanks-{n}", wrap("#1=A(1," + " " * n + "2);\n"))

# 3. Deep nesting
for depth in [100, 1000, 3000]:
    add(f"deep/list-{depth}", wrap("#1=A(" + "(" * depth + "1" + ")" * depth + ");\n"))
    add(f"deep/open-{depth}", wrap("#1=A(" + "(" * depth + "1);\n"))
    add(f"deep/typed-{depth}", wrap("#1=A(" + "B(" * depth + "1" + ")" * depth + ");\n"))
for depth in [10, 100]:
    add(f"deep/scope-{depth}", wrap("".join(f"#{i + 1}=&SCOPE\n" for i in range(depth)) + "#999=A(1);\n" +
                                    "ENDSCOPE\nC(1);\n" * depth))

# 4. Binary junk
rng = random.Random(8)
for i in range(40):
    n = rng.choice([1, 2, 10, 100, 1000, 5000])
    add(f"junk/raw-{i:02d}", bytes(rng.randrange(256) for _ in range(n)))
    add(f"junk/body-{i:02d}", HEAD.encode() + bytes(rng.randrange(256) for _ in range(n)) + TAIL.encode())
    add(f"junk/ascii-{i:02d}", HEAD + "".join(chr(rng.randrange(32, 127)) for _ in range(n)) + TAIL)

# 5. Truncations and token-level mutations of a synthetic file and of the start of OCCT's screw.step
SYNTH = (HEAD.replace("DATA;\n", "DATA;\n/* a comment */\n") +
         "#1=CARTESIAN_POINT('',(0.,1.5E-3,-2.));\n#2=DIRECTION('',(0.,0.,1.));\n"
         "#3=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#4,#5))REPRESENTATION_CONTEXT('',''));\n"
         "#4=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n#5=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n"
         "#6=B_SPLINE_CURVE_WITH_KNOTS('',3,(#1,#1,#1,#1),.UNSPECIFIED.,.F.,.F.,(4,4),(0.,1.),.UNSPECIFIED.);\n"
         "#7=SHAPE_REPRESENTATION('it''s',(#1,#2),#3);\n#8=&SCOPE\n#9=A(1);\nENDSCOPE/#9/C(#9);\n"
         "#10=PRODUCT('p','\\X2\\00E9\\X0\\','',(#11));\n#11=!USER_DEFINED(\"0F\",*,$);\n" + TAIL)
with open(os.path.join(src_dir, "data", "step", "screw.step"), "rb") as f:
    screw = f.read()
cut = screw.index(b"#40 =")
SCREW = (screw[:cut] + TAIL.encode()).decode("latin-1")
for base_name, base in [("synth", SYNTH), ("screw", SCREW)]:
    for i in range(60):
        n = len(base) * i // 60 + rng.randrange(max(1, len(base) // 60))
        add(f"trunc/{base_name}-{i:02d}", base[:n])
VOCAB = ["(", ")", ",", ";", "=", "#5", "#5=", "FOO", "$", "/", "&SCOPE", "ENDSCOPE", "ENDSEC;", "DATA;", "HEADER;",
         "'x'", "'", ".T.", "1.5", "*", "?", "\n", "/*", "*/", "!U", "\"0F\"", "END-ISO-10303-21;", "ENDSTEP;", "\x00"]


def lex_rough(text):
    # Splits into rough tokens: words, strings, and single characters
    out, i = [], 0
    while i < len(text):
        c = text[i]
        if c.isalnum() or c in "_#.-+!":
            j = i
            while j < len(text) and (text[j].isalnum() or text[j] in "_#.-+!"):
                j += 1
        elif c == "'":
            j = text.find("'", i + 1)
            j = len(text) if j < 0 else j + 1
        else:
            j = i + 1
        out.append(text[i:j])
        i = j
    return out


for base_name, base in [("synth", SYNTH), ("screw", SCREW)]:
    toks = lex_rough(base)
    for i in range(400 if base_name == "synth" else 200):
        t = list(toks)
        for _ in range(rng.choice([1, 1, 1, 2, 3, 5])):
            k = rng.randrange(len(t))
            op = rng.randrange(4)
            if op == 0:
                del t[k]
            elif op == 1:
                t.insert(k, rng.choice(VOCAB))
            elif op == 2:
                t[k] = rng.choice(VOCAB)
            else:
                t.insert(k, t[rng.randrange(len(t))])
        add(f"mutate/{base_name}-{i:03d}", "".join(t))

with open(out_path, "wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, compresslevel=9, mtime=0) as out:
    for name, data in cases:
        out.write(name.encode() + b"\n" + str(len(data)).encode() + b"\n" + data)
print(f"{len(cases)} cases")
