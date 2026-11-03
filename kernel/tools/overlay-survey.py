"""The overlay survey (Kerf unit 10H1, docs/overlay-survey.md): copy-at-entry sites, frozen-shape checks and in-place
topology edits, counted per module, for units 10H2 to 10H6 to recount as they convert their modules.

Usage: python -I tools/overlay-survey.py src [--files]   (from kernel/)
Counts are of source sites (calls), found by regular expressions over the C# text with comments stripped: an
estimate to plan by, not a proof. The in-place edit kinds count every call, including the many into topology the
algorithm built itself.
"""
import os
import re
import sys
from collections import defaultdict

SRC = sys.argv[1]
SHOW_FILES = "--files" in sys.argv

BOOLEANS = {"BOPAlgo", "BOPDS", "BOPTools", "BRepAlgo", "BRepAlgoAPI", "IntTools", "TopOpeBRepBuild",
            "TopOpeBRepDS", "TopOpeBRepTool"}
CONSTRUCTION = {"BRepBuilderAPI", "BRepLib", "BRepPrim", "BRepPrimAPI", "BRepSweep", "BRepFill", "GeomFill",
                "HelixBRep", "BRepProj"}
HEALING = {"ShapeFix", "ShapeBuild", "ShapeUpgrade", "ShapeAnalysis", "ShapeConstruct", "ShapeCustom",
           "ShapeProcess", "ShapeProcessAPI", "ShapeAlgo", "ShapeExtend"}
OFFSETS = {"BRepOffset", "BiTgte"}
FILLETS = {"ChFi2d", "ChFi3d", "ChFiDS", "ChFiKPart", "FilletSurf", "BRepFilletAPI", "BRepBlend", "Blend",
           "BlendFunc"}
FEATURES = {"BRepFeat", "LocOpe", "Draft"}
# BRepOffsetAPI holds sweeps, offsets and drafts: by file
OFFSETAPI = {"ApiMakePipe.cs": "construction", "MakePipeShell.cs": "construction", "ThruSections.cs": "construction",
             "MakeEvolved.cs": "construction", "MakeFilling.cs": "construction", "MiddlePath.cs": "construction",
             "DraftAngle.cs": "features", "MakeDraft.cs": "features"}
BYFILE = {"Sewing.cs": "healing", "FastSewing.cs": "healing"}


def module(path):
    rel = os.path.relpath(path, SRC).replace("\\", "/")
    parts = rel.split("/")
    asm, pkg, name = parts[0], parts[1] if len(parts) > 2 else "", parts[-1]
    if asm.endswith("ModelingData") or asm.endswith("Foundation"):
        return "other"  # the topology layer itself (ModelingData's own sites are listed by file)
    if asm.endswith("DataExchange"):
        return "other"
    if name in BYFILE:
        return BYFILE[name]
    if pkg == "BRepOffsetAPI":
        return OFFSETAPI.get(name, "offsets")
    for mod, pkgs in (("booleans", BOOLEANS), ("construction", CONSTRUCTION), ("healing", HEALING),
                      ("offsets", OFFSETS), ("fillets", FILLETS), ("features", FEATURES)):
        if pkg in pkgs:
            return mod
    return "other"


MODULES = ["booleans", "construction", "healing", "offsets", "fillets", "features", "other"]

# --- copy at entry and the frozen-shape checks ---
COPY = [
    ("CopyAtEntry", r"\bnew\s+CopyAtEntry\s*\("),
    ("ExactCopyForWrite", r"\bExactCopyForWrite\s*\("),
    ("KeepUnchanged/KeepUnchangedIn", r"\bKeepUnchanged(?:In)?\s*\("),
    ("KeepUnchangedGenerating", r"\bKeepUnchangedGenerating\s*\("),
    ("CopyFrozenForWrite", r"\bCopyFrozenForWrite\s*\("),
    ("HoldsFrozen", r"\bHoldsFrozen\s*\("),
    ("frozen workarounds (*OfFrozen, *Unfrozen, UnfreezeResult)", r"\b\w*(?:OfFrozen|Unfrozen|UnfreezeResult)\s*\("),
]
CHECKS = [
    ("IsFrozen tests", r"\.IsFrozen\b"),
    ("Locked() tests", r"\.Locked\(\s*\)"),
    ("catch of LockedShape/ImmutableObject", r"catch\s*\(\s*(?:LockedShapeException|ImmutableObjectException|FrozenShapeException)"),
]
MARKERS = [("'OCCT.NET: frozen' markers", r"OCCT\.NET:\s*frozen")]

# --- in-place edits: kinds, and the members they reach ---
EDITS = [
    ("vertex point/tolerance/parameter (UpdateVertex)", r"\.UpdateVertex\s*\("),
    ("edge parameters on curves (Transfert)", r"\.Transfert\s*\("),
    ("edge curve, pcurve or tolerance (UpdateEdge)", r"\.UpdateEdge\s*\("),
    ("edge range (Range with bounds)", r"\.Range\s*\(\s*[^,()]+\s*,\s*[^()]+,[^()]*\)"),
    ("edge flags (SameParameter/SameRange/Degenerated set)", r"\.(?:SameParameter|SameRange|Degenerated)\s*\(\s*[^,()]+\s*,\s*[^()]+\)"),
    ("edge continuity (Continuity)", r"[Bb]\w*\.Continuity\s*\("),
    ("face surface/tolerance/mesh (UpdateFace)", r"\.UpdateFace\s*\("),
    ("face natural restriction", r"\.NaturalRestriction\s*\(\s*[^,()]+\s*,"),
    ("BRepLib's in-place statics (SameParameter, UpdateTolerances, BuildCurves3d, EncodeRegularity, OrientClosedSolid...)",
     r"\bBrepLibrary\.(?:SameParameter|UpdateTolerances|UpdateInnerTolerances|BuildCurves3d|BuildCurve3d|EncodeRegularity|OrientClosedSolid|SameRange|BuildPCurveForEdgeOnPlane|BuildPCurveForEdgesOnPlane|UpdateDeflection|UpdateEdgeTol|EnsureNormalConsistency)\s*\("),
    ("sub-shapes added (Builder.Add)", None),
    ("sub-shapes removed (Builder.Remove)", None),
    ("shape flags set (Free/Closed/Orientable/Modified/Checked/Convex/Infinite)",
     r"\.(?:Free|Closed|Orientable|Modified|Checked|Convex|Infinite)\s*\(\s*(?:true|false|![\w.()]+|is\w+|the\w+|\w+\.\w+\(\))\s*\)"),
    ("TShape data written directly (Change* lists, BRep TShape setters)",
     r"\.(?:ChangeCurves|ChangePoints)\s*\(\s*\)|\b(?:t[vef]|T[VEF]|a?T[VEF]\w*|te\w*|tv\w*|tf\w*)\.(?:Tolerance|UpdateTolerance|Pnt|Surface|Location|Triangulation|Triangulations)\s*\(\s*[^)\s]"),
    ("child lists edited directly (_shapes)", r"\._shapes\b"),
    ("representations edited (Range/PCurve/UVPoints/Parameter set on a representation)",
     r"\b(?:gc|GC|aGC|cr|itcr\.Value\(\)|pr|aPR|aCR|cos|COS)\w*\.(?:SetRange|Range|PCurve|PCurve2|SetUVPoints|SetUVPoints2|Parameter|Curve3D|Continuity)\s*\(\s*[^)\s]"),
    ("orientation/location of a value (no TShape edit)", r"\.(?:Orientation|Location)\s*\(\s*[^)\s][^)]*\)\s*;|\.(?:Reverse|Complement)\s*\(\s*\)\s*;|\.Move\s*\("),
]

BUILDER_DECL = re.compile(r"\b(?:BrepBuilder|Builder)\s+(\w+)\s*=|\b(\w+)\s*=\s*new\s+(?:BrepBuilder|Builder)\s*\(")
COMMENT_LINE = re.compile(r"//.*?$", re.M)
COMMENT_BLOCK = re.compile(r"/\*.*?\*/", re.S)


def strip(text):
    return COMMENT_LINE.sub("", COMMENT_BLOCK.sub("", text))


counts = {kind: defaultdict(int) for kind, _ in COPY + CHECKS + MARKERS + EDITS}
copy_files = defaultdict(lambda: defaultdict(int))  # module -> file -> copy sites
edit_files = defaultdict(int)

for root, dirs, files in os.walk(SRC):
    dirs[:] = [d for d in dirs if d not in ("bin", "obj")]
    for f in files:
        if not f.endswith(".cs"):
            continue
        path = os.path.join(root, f)
        raw = open(path, encoding="utf-8-sig", errors="replace").read()
        code = strip(raw)
        mod = module(path)
        rel = os.path.relpath(path, SRC).replace("\\", "/")
        # the helpers' own definitions don't count as sites
        is_helper = rel.endswith(("BRepTools/CopyAtEntry.cs", "BRepTools/BRepTools.ExactCopy.cs",
                                  "BRepTools/BRepTools.CopyFrozen.cs", "BRepTools/WorkingOverlay.cs"))
        for kind, rx in COPY:
            if is_helper:
                continue
            n = len(re.findall(rx, code))
            counts[kind][mod] += n
            if n:
                copy_files[mod][rel] += n
        for kind, rx in CHECKS:
            counts[kind][mod] += len(re.findall(rx, code))
        for kind, rx in MARKERS:
            counts[kind][mod] += len(re.findall(rx, raw))
        builders = {m.group(1) or m.group(2) for m in BUILDER_DECL.finditer(code)}
        builders |= {"B", "BB", "aBB", "aB", "theBB", "myBuilder", "_builder", "bb", "b", "aBuilder", "builder"}
        for kind, rx in EDITS:
            if kind.startswith("sub-shapes added"):
                n = sum(len(re.findall(r"\b%s\.Add\s*\(" % re.escape(b), code)) for b in builders) \
                    + len(re.findall(r"new\s+(?:Brep)?Builder\s*\(\s*\)\s*\.Add\s*\(", code))
            elif kind.startswith("sub-shapes removed"):
                n = sum(len(re.findall(r"\b%s\.Remove\s*\(" % re.escape(b), code)) for b in builders)
            else:
                n = len(re.findall(rx, code))
            counts[kind][mod] += n
            edit_files[rel] += n


def table(title, kinds):
    print(f"\n### {title}\n")
    print("| Kind | " + " | ".join(m.capitalize() for m in MODULES) + " | Total |")
    print("|---|" + "---:|" * (len(MODULES) + 1))
    totals = defaultdict(int)
    for kind, _ in kinds:
        row = [counts[kind][m] for m in MODULES]
        for m, n in zip(MODULES, row):
            totals[m] += n
        print(f"| {kind} | " + " | ".join(str(n) for n in row) + f" | {sum(row)} |")
    print("| **Total** | " + " | ".join(f"**{totals[m]}**" for m in MODULES) + f" | **{sum(totals.values())}** |")


table("Copy at entry", COPY)
table("Frozen-shape checks", CHECKS + MARKERS)
table("In-place edit sites (all topology, the algorithms' own included)", EDITS)

if SHOW_FILES:
    print("\n### Copy-at-entry sites by file\n")
    for m in MODULES:
        files = copy_files.get(m)
        if not files:
            continue
        print(f"- **{m.capitalize()}** ({sum(files.values())} sites, {len(files)} files): "
              + ", ".join(f"`{os.path.basename(k)}` {v}" for k, v in sorted(files.items())))
