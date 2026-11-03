"""The state survey (Kerf unit 10I, docs/state.md): every static field that isn't const, every static auto-property
with a setter, every [ThreadStatic] and every lock statement in the kernel's and the modelling layer's sources.

Usage: python -I tools/state-survey.py <repository root> <out.tsv>   (from kernel/: python -I tools/state-survey.py .. out.tsv)
Writes one row per site (kind, file, line, type, name, text) and prints the counts by kind: mutable (static, neither
readonly nor const), readonly (static readonly), threadstatic, property (a static auto-property with a setter), lock.
Found by regular expressions over the C# text with comments stripped: a declaration of several variables is one row.
"""
import os, re, sys

root = sys.argv[1]
out = sys.argv[2]
dirs = ["kernel/src/Kerf.Kernel.Foundation", "kernel/src/Kerf.Kernel.ModelingData",
        "kernel/src/Kerf.Kernel.ModelingAlgorithms", "kernel/src/Kerf.Kernel.DataExchange", "modeling/src/Kerf.Modeling"]

MODS = r"(?:public|private|internal|protected|static|readonly|volatile|new|unsafe|required)"
FIELD = re.compile(r"^\s*((?:" + MODS + r"\s+)*)static\s+((?:" + MODS + r"\s+)*)"
                   r"(?!class\b|struct\b|record\b|interface\b|enum\b|delegate\b|extern\b|partial\b|void\b|async\b|implicit\b|explicit\b|operator\b|event\b|abstract\b|override\b|virtual\b)"
                   r"([\w<>\[\],.?() ]+?)\s+(\w+)\s*(=(?![>=])|;|\{)")
LOCK = re.compile(r"\block\s*\(")
LOCKFIELD = re.compile(r"\b(?:Lock|object)\s+\w+\s*=\s*new\s*\(\s*\)|new\s+(?:Lock|object)\s*\(\s*\)")

def strip_comment(line):
    i = line.find("//")
    return line if i < 0 else line[:i]

rows = []
for d in dirs:
    for dp, dn, fn in os.walk(os.path.join(root, d)):
        if "\\obj" in dp or "/obj" in dp or "\\bin" in dp or "/bin" in dp:
            continue
        for f in fn:
            if not f.endswith(".cs"):
                continue
            p = os.path.join(dp, f)
            rel = os.path.relpath(p, root).replace("\\", "/")
            lines = open(p, encoding="utf-8-sig").read().split("\n")
            prev_attr = ""
            for i, raw in enumerate(lines):
                line = strip_comment(raw)
                s = line.strip()
                if s.startswith("[") and "ThreadStatic" in s and not FIELD.match(re.sub(r"^\s*\[[^\]]*\]\s*", "", line)):
                    prev_attr = "ThreadStatic"
                    continue
                m = FIELD.match(re.sub(r"^\s*\[[^\]]*\]\s*", "", line))
                if m:
                    mods = (m.group(1) + " " + m.group(2)).split()
                    typ = m.group(3).strip()
                    name = m.group(4)
                    term = m.group(5)
                    if typ in ("const",) or "const" in mods:
                        prev_attr = ""
                        continue
                    if "(" in typ:  # a method
                        prev_attr = ""
                        continue
                    ts = "ThreadStatic" in line or prev_attr == "ThreadStatic"
                    kind = None
                    if term == "{":
                        # property: mutable only with a setter
                        rest = " ".join(lines[i:i + 3])
                        if re.search(r"\bset\s*;|\binit\s*;", rest) and "=>" not in line.split("{")[0]:
                            kind = "property"
                        else:
                            prev_attr = ""
                            continue
                    elif ts:
                        kind = "threadstatic"
                    elif "readonly" in mods:
                        kind = "readonly"
                    else:
                        kind = "mutable"
                    rows.append((kind, rel, i + 1, typ, name, s[:200]))
                    prev_attr = ""
                    continue
                if LOCK.search(line):
                    rows.append(("lock", rel, i + 1, "", "", s[:200]))
                if s and not s.startswith("["):
                    prev_attr = ""

with open(out, "w", encoding="utf-8") as fo:
    for r in rows:
        fo.write("\t".join(str(x) for x in r) + "\n")
from collections import Counter
c = Counter(r[0] for r in rows)
print(c)
