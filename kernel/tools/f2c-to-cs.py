# Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
#
# A porting aid for AdvApp2Var's f2c code (decision 905): translates the functions of an f2c-generated .cxx into C#
# with the conventions of decision 901, as a first draft that is then fixed by hand until it compiles. It is not a
# build step; the C# files are the source of truth once committed.
#
#   python tools/f2c-to-cs.py <file.cxx> <ClassName> <out.cs> [extra-signatures.json]
#
# What it does:
#   - infers, for every pointer parameter, whether it is an array or a scalar and whether the function writes it
#     (directly or through the functions it calls, to a fixpoint), and emits F2cPtr<T> / F2cConstPtr<T> for arrays,
#     `ref T` for written scalars and `T` for read-only ones;
#   - rewrites the bodies: `*scalar` -> `scalar`; call arguments by the callee's parameter modes (`&x` -> `x`,
#     `ref x`, `F2c.Ref(ref x)`; `&a[k]` -> `a + (k)` or `a[k]` or `ref a[k]`); local declarations (zero-initialised,
#     arrays on the stack); `std::` functions; literals such as `1.`; the f2c comma expressions; unreferenced labels.
#   - keeps the comments.
# Signatures of functions defined elsewhere (AdvApp2Var_SysBase, or MathBase when converting ApproxF2var) are given
# in KNOWN below or in the JSON file: {"name": "vvcpr", ...} with one letter per parameter:
#   v = scalar by value, r = scalar by ref, c = read-only array, p = array, R = `ref` array pointer (mcrrqst_),
#   s = string, x = other (passed through unchanged).

import json
import re
import sys

KNOWN = {
    # AdvApp2Var_SysBase
    "mcrfill_": "vcp", "mvriraz_": "vp", "miraz_": "vp", "msrfill_": "vcp", "msifill_": "vcp",
    "maermsg_": "svv", "mgenmsg_": "sv", "mgsomsg_": "sv", "mswrdbg_": "sv", "mnfndeb_": "",
    "mcrrqst_": "vvRrr", "mcrdelt_": "vvprr", "macrar8_": "vvRrr", "macrdr8_": "vvprr",
    "macrai4_": "vvRrr", "macrdi4_": "vvprr", "maitbr8_": "vpv", "maovsr8_": "r", "macinit_": "vv",
}

CTYPES = {"int": "int", "double": "double", "float": "float", "short": "short", "char": "char"}


def strip_comments(theText, keep_strings=False):
    """The text with comments (and string contents unless keep_strings) blanked, at the same positions."""
    out = []
    i = 0
    n = len(theText)
    while i < n:
        if theText.startswith("/*", i):
            j = theText.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", theText[i:j]))
            i = j
        elif theText.startswith("//", i):
            j = theText.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif theText[i] == '"':
            j = i + 1
            while j < n and theText[j] != '"':
                j += 2 if theText[j] == "\\" else 1
            out.append(theText[i:j + 1] if keep_strings else '"' + " " * (j - i - 1) + '"')
            i = j + 1
        elif theText[i] == "'":
            j = theText.find("'", i + 1)
            out.append(" " * (j - i + 1))
            i = j + 1
        else:
            out.append(theText[i])
            i += 1
    return "".join(out)


def match_paren(theCode, theOpen):
    """Index of the bracket closing the one at theOpen."""
    pairs = {"(": ")", "[": "]", "{": "}"}
    stack = []
    for i in range(theOpen, len(theCode)):
        c = theCode[i]
        if c in pairs:
            stack.append(pairs[c])
        elif c in ")]}":
            if not stack or stack[-1] != c:
                raise ValueError(f"unbalanced at {i}")
            stack.pop()
            if not stack:
                return i
    raise ValueError("unclosed")


def split_top(theText, theSep=","):
    """Split at top-level separators (outside brackets)."""
    parts, depth, start = [], 0, 0
    for i, c in enumerate(theText):
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        elif c == theSep and depth == 0:
            parts.append(theText[start:i])
            start = i + 1
    parts.append(theText[start:])
    return parts


class Func:
    def __init__(self, name, ret, params, body, start, end, is_static, header_text):
        self.name = name
        self.ret = ret
        self.params = params  # list of dicts: name, ctype, kind ('ptr', 'val', 'arr1', 'fptr'), const
        self.body = body  # text between the braces (with comments)
        self.start, self.end = start, end
        self.is_static = is_static
        self.header_text = header_text
        self.modes = {}


FUNC_RE = re.compile(
    r"^(static\s+)?(int|double|void)\s+(?:(\w+)::)?(\w+)\s*\(", re.M)


def parse_params(theText):
    params = []
    stripped = strip_comments(theText)
    pos = 0
    for part in split_top(stripped):
        start, pos = pos, pos + len(part) + 1
        code = part.strip()
        # the name of an unnamed parameter is in the comment that follows it ("int*, // mxcoef,")
        eol = theText.find("\n", start + len(part))
        comment = re.search(r"//\s*(\w+)", theText[start:eol if eol >= 0 else len(theText)])
        if not code:
            continue
        p = {"const": code.startswith("const ")}
        code = re.sub(r"^const\s+", "", code)
        code = re.sub(r"^(\w+)\s+const\s*\*", r"\1*", code)  # `int const* x`, as `const int* x`
        p["const"] = p["const"] or "const" in part
        m = re.match(r"int\s*\(\*\s*(\w+)\)\s*\((.*)\)$", code, re.S)
        if m:
            p.update(kind="fptr", name=m.group(1), ctype="int", sig=m.group(2))
            params.append(p)
            continue
        m = re.match(r"AdvApp2Var_EvaluatorFunc2Var\s*&\s*(\w+)$", code)
        if m:
            p.update(kind="eval", name=m.group(1), ctype="AdvApp2Var_EvaluatorFunc2Var")
            params.append(p)
            continue
        m = re.match(r"NCollection_Array1<(\w+)>\s*&\s*(\w+)$", code)
        if m:
            p.update(kind="arr1", name=m.group(2), ctype=m.group(1))
            params.append(p)
            continue
        m = re.match(r"(\w+)\s*\*\s*(\w+)?$", code)
        if not m:
            raise ValueError(f"param {code!r}")
        name = m.group(2) or (comment.group(1) if comment else None)
        p.update(kind="ptr", name=name, ctype=m.group(1), unnamed=m.group(2) is None)
        params.append(p)
    return params


def parse_functions(theText, theCode):
    funcs = []
    for m in FUNC_RE.finditer(theCode):
        po = m.end() - 1
        pc = match_paren(theCode, po)
        k = pc + 1
        while k < len(theCode) and theCode[k] in " \t\r\n":
            k += 1
        if k >= len(theCode) or theCode[k] != "{":
            continue  # a prototype
        bc = match_paren(theCode, k)
        params = parse_params(theText[po + 1:pc])
        funcs.append(Func(m.group(4), m.group(2), params, theText[k + 1:bc], m.start(), bc + 1,
                          m.group(1) is not None or m.group(3) is None, theText[m.start():k]))
    return funcs


CALL_RE = re.compile(r"(?:\b(?:AdvApp2Var_\w+::|anAdvApp2Var_SysBase\.)|(?<![\w.:]))(\w+)\s*\(")


def find_calls(theCode, theKnown):
    """(name, [arg texts], start, end) of the calls to known functions in code (comment-free)."""
    calls = []
    for m in CALL_RE.finditer(theCode):
        name = m.group(1)
        if name not in theKnown:
            continue
        po = m.end() - 1
        pc = match_paren(theCode, po)
        args = [a.strip() for a in split_top(theCode[po + 1:pc])] if theCode[po + 1:pc].strip() else []
        calls.append((name, args, m.start(), pc + 1))
    return calls


def infer(funcs):
    """Fills f.modes: param name -> mode letter, to a fixpoint over the calls between the functions."""
    sigs = dict(KNOWN)
    byname = {f.name: f for f in funcs}
    for f in funcs:
        f.code = strip_comments(f.body)
        f.isarr, f.written = {}, {}
        for p in f.params:
            n = p["name"]
            if p["kind"] != "ptr":
                continue
            c = f.code
            arr = bool(re.search(rf"(?<![\w.]){n}\s*\[|--{n}\b|(?<![*\w])(?<!\* ){n}\s*[-+]=|(?<![*\w]){n}\+\+|\+\+{n}\b", c))
            f.isarr[n] = arr
            if arr:
                w = bool(re.search(rf"(?<![\w.]){n}\s*\[[^\]]*(?:\[[^\]]*\][^\]]*)*\]\s*([-+*/]?=)(?!=)", c)) \
                    or bool(re.search(rf"(\+\+|--)\s*{n}\s*\[", c)) \
                    or bool(re.search(rf"(?<![\w.]){n}\s*\[[^\]]*(?:\[[^\]]*\][^\]]*)*\]\s*(\+\+|--)", c))
            else:
                w = bool(re.search(rf"\*\s*{n}\b\s*([-+*/]?=)(?!=)", c)) or \
                    bool(re.search(rf"(\+\+|--)\s*\(?\s*\*\s*{n}\b|\(\s*\*\s*{n}\s*\)\s*(\+\+|--)", c))
            f.written[n] = w
        for p in f.params:
            if p["kind"] == "ptr":
                sigs[f.name] = None
    changed = True
    while changed:
        changed = False
        for f in funcs:
            sigs[f.name] = "".join(mode_of(f, p) for p in f.params)
        for f in funcs:
            for name, args, _, _ in find_calls(f.code, sigs):
                sig = sigs[name]
                if sig is None:
                    continue
                for a, mode in zip(args, sig):
                    m = re.match(r"^&?\s*(\w+)\s*(\[.*\])?$", a, re.S)
                    if not m or m.group(1) not in f.isarr:
                        continue
                    n = m.group(1)
                    if mode in "pR" and not f.isarr[n] and not a.startswith("&"):
                        pass
                    if mode in "pcR" and not a.startswith("&") and not f.isarr[n]:
                        # a pointer parameter passed on as an array
                        f.isarr[n] = True
                        changed = True
                    if mode in "prR" and not f.written[n]:
                        f.written[n] = True
                        changed = True
    for f in funcs:
        f.modes = {p["name"]: m for p, m in zip(f.params, sigs[f.name])}
    return sigs


def mode_of(f, p):
    if p["kind"] == "fptr":
        return "x"
    if p["kind"] in ("arr1", "eval"):
        return "x"
    n = p["name"]
    if f.isarr.get(n):
        return "p" if f.written[n] else "c"
    return "r" if f.written.get(n) else "v"


def cs_param(f, p):
    n = p["name"]
    t = CTYPES.get(p["ctype"], p["ctype"])
    m = f.modes[n]
    if p["kind"] == "fptr":
        return f"AdvApp2Var_MathBase_bfunx {n}"
    if p["kind"] == "arr1":
        return f"NCollection_Array1<{t}> {n}"
    if p["kind"] == "eval":
        return f"AdvApp2Var_EvaluatorFunc2Var {n}"
    return {"v": f"{t} {n}", "r": f"ref {t} {n}", "c": f"F2cConstPtr<{t}> {n}",
            "p": f"F2cPtr<{t}> {n}"}[m]


def convert_arg(a, mode, f, locals_arr, locals_scalar):
    a = a.strip()
    m_amp_idx = re.match(r"^&\s*(.+?)\s*\[(.*)\]$", a, re.S)
    m_amp = re.match(r"^&\s*([\w.]+)$", a)
    m_name = re.match(r"^(\w+)$", a)
    is_ptr_name = m_name and (f.isarr.get(a) or a in locals_arr)
    if mode == "x" or mode == "s":
        return a
    if mode == "v":
        if m_amp_idx:
            return f"{m_amp_idx.group(1)}[{m_amp_idx.group(2)}]"
        if m_amp:
            return m_amp.group(1)
        if is_ptr_name:
            return f"{a}[0]"
        return a
    if mode == "r":
        if m_amp_idx:
            return f"ref {m_amp_idx.group(1)}[{m_amp_idx.group(2)}]"
        if m_amp:
            return f"ref {m_amp.group(1)}"
        if is_ptr_name:
            return f"ref {a}[0]"
        return f"ref {a}"
    if mode == "R":
        return f"ref {a}"
    # p / c: pointers
    if m_amp_idx:
        base, idx = m_amp_idx.group(1), m_amp_idx.group(2).strip()
        return f"{base} + ({idx})" if not re.fullmatch(r"\w+|\d+", idx) else f"{base} + {idx}"
    if m_amp:
        return f"F2c.Ref(ref {m_amp.group(1)})" if mode == "p" else f"F2c.In(in {m_amp.group(1)})"
    if m_name and not is_ptr_name and (a in f.modes or a in locals_scalar):
        return f"F2c.Ref(ref {a})" if mode == "p" else f"F2c.In(in {a})"
    return a


def literals(theCode):
    # 1. -> 1.0, 1.e5 -> 1.0e5 (not inside identifiers)
    return re.sub(r"(?<![\w.])(\d+)\.(?![\d])", r"\1.0", theCode)


DECL_RE = re.compile(r"^([ \t]*)(static\s+)?(const\s+)?(int|double|float|long|intptr_t|char|short|bool)\b([^;(]*(?:\([^;]*\))?[^;]*);",
                     re.M)


def convert_decl(m, f, locals_arr, locals_scalar):
    indent, static, _, ctype, rest = m.group(1), m.group(2), m.group(3), m.group(4), m.group(5)
    if re.match(r"\s*\*?\s*\w+\s*\(", rest) and "=" not in rest.split("(")[0]:
        return m.group(0)  # a prototype or a cast, not a declaration
    t = {"intptr_t": "int", "long": "bool"}.get(ctype, ctype)
    out = []
    for d in split_top(rest):
        d = d.strip()
        if not d:
            continue
        mm = re.match(r"^(\*\s*)?(\w+)\s*(\[([^\]]*)\])?\s*(=\s*(.*))?$", d, re.S)
        if not mm:
            out.append(f"{indent}/* TODO */ {t} {d};")
            continue
        ptr, name, dim, init = mm.group(1), mm.group(2), mm.group(4), mm.group(6)
        if ctype == "char" and dim is not None:
            out.append(f"{indent}string {name} = {init.strip() if init else chr(34) + chr(34)};")
            locals_scalar.add(name)
            continue
        if ptr:
            locals_arr.add(name)
            out.append(f"{indent}F2cPtr<{t}> {name} = default;")
            continue
        if dim is not None:
            locals_arr.add(name)
            vals = []
            if init:
                inner = init.strip()[1:-1]
                vals = [v.strip() for v in split_top(inner) if v.strip()]
                if all(re.fullmatch(r"0\.?0*", v) for v in vals):
                    vals = []
            if static and vals:
                # a Fortran DATA table: constant data in the assembly, two values a line as OCCT writes them
                lines = [", ".join(literals(v) for v in vals[i:i + 2]) for i in range(0, len(vals), 2)]
                body = (",\n" + indent + "  ").join(lines)
                out.append(f"{indent}ReadOnlySpan<{t}> {name} =\n{indent}[\n{indent}  {body}\n{indent}];")
            elif vals:
                out.append(f"{indent}F2cPtr<{t}> {name} = new F2cPtr<{t}>(stackalloc {t}[{dim}] {{ {', '.join(vals)} }});")
            else:
                out.append(f"{indent}F2cPtr<{t}> {name} = new F2cPtr<{t}>(stackalloc {t}[{dim}]);")
            continue
        locals_scalar.add(name)
        if init is None:
            init = {"bool": "false", "double": "0.0", "float": "0.0f"}.get(t, "0")
        out.append(f"{indent}{t} {name} = {init.strip()};")
    return "\n".join(out)


def convert_body(f, sigs):
    body = f.body
    code = strip_comments(body, keep_strings=True)
    locals_arr, locals_scalar = set(), set()

    # Work on code segments only: we edit `body` but find positions in `code`; do edits from the end.
    edits = []  # (start, end, replacement)

    # 1. declarations
    for m in DECL_RE.finditer(code):
        txt = body[m.start():m.end()]
        mm = DECL_RE.match(txt)
        if mm is None:
            continue
        # Map the match on the comment-free text, but keep the original text for initializers with comments.
        rep = convert_decl(DECL_RE.match(code[m.start():m.end()]), f, locals_arr, locals_scalar)
        if rep != code[m.start():m.end()]:
            edits.append((m.start(), m.end(), rep))
    for m in re.finditer(r"^(\s*)AdvApp2Var_SysBase\s+(\w+)\s*;", code, re.M):
        edits.append((m.start(), m.end(), f"{m.group(1)}AdvApp2Var_SysBase {m.group(2)} = new();"))
    for m in re.finditer(r"^(\s*)NCollection_Array1<(\w+)>\s+(\w+)\s*\(([^;]*)\)\s*;", code, re.M):
        edits.append((m.start(), m.end(), f"{m.group(1)}NCollection_Array1<{m.group(2)}> {m.group(3)} = new({m.group(4)});"))

    # 2. calls (outermost handled with inner text converted recursively)
    def conv_expr(text):
        """Converts an expression or statement text (comment-free)."""
        res = []
        pos = 0
        for name, args, s, e in find_calls(text, sigs):
            if s < pos:
                continue
            sig = sigs[name]
            head = text[s:text.index("(", s)]
            head = head.replace("::", ".")
            new_args = []
            for i, a in enumerate(args):
                mode = sig[i] if sig and i < len(sig) else "x"
                new_args.append(convert_arg(conv_expr(a), mode, f, locals_arr, locals_scalar))
            res.append(text[pos:s])
            res.append(f"{head}({', '.join(new_args)})")
            pos = e
        res.append(text[pos:])
        return "".join(res)

    for name, args, s, e in find_calls(code, sigs):
        if any(es <= s < ee for es, ee, _ in edits):
            continue
        edits.append((s, e, conv_expr(code[s:e])))

    edits.sort(key=lambda x: x[0])
    # drop nested edits (calls inside calls are converted by conv_expr)
    flat = []
    for ed in edits:
        if flat and ed[0] < flat[-1][1]:
            continue
        flat.append(ed)
    out = []
    pos = 0
    for s, e, rep in flat:
        out.append(post(body[pos:s], f, locals_arr))
        out.append(post_code(rep, f, locals_arr))
        pos = e
    out.append(post(body[pos:], f, locals_arr))
    text = "".join(out)
    return labels(text)


def post(text, f, locals_arr):
    """Rewrites a body segment outside calls and declarations, leaving its comments alone."""
    code = strip_comments(text)
    pieces = []
    pos = 0
    # find code runs between comments
    for m in re.finditer(r"/\*.*?\*/|//[^\n]*", text, re.S):
        pieces.append(post_code(text[pos:m.start()], f, locals_arr))
        pieces.append(text[m.start():m.end()])
        pos = m.end()
    pieces.append(post_code(text[pos:], f, locals_arr))
    return "".join(pieces)


def post_code(text, f, locals_arr):
    # *scalar -> scalar
    for p in f.params:
        n = p["name"]
        if p["kind"] == "ptr" and not f.isarr.get(n):
            text = re.sub(rf"\*\s*{n}\b", n, text)
            text = re.sub(rf"\(\s*{n}\s*\)", n, text) if False else text
        elif p["kind"] == "ptr" and f.isarr.get(n):
            # a dereference, not a multiplication: the `*` follows an operator, a bracket or the start
            text = re.sub(rf"((?:^|[=(,+\-*/<>!&|?:;{{])\s*)\*\s*{n}\b(?!\s*\[)", rf"\g<1>{n}[0]", text)
    text = text.replace("AdvApp2Var_SysBase::", "AdvApp2Var_SysBase.")
    text = text.replace("AdvApp2Var_MathBase::", "AdvApp2Var_MathBase.")
    text = text.replace("AdvApp2Var_Data::", "AdvApp2Var_Data.")
    text = text.replace("AdvApp2Var_ApproxF2var::", "AdvApp2Var_ApproxF2var.")
    text = re.sub(r"\b(\w+)_Evaluate\(", r"\1.Evaluate(", text)
    text = re.sub(r"\bstd::max\(", "Max(", text)
    text = re.sub(r"\bstd::min\(", "Min(", text)
    text = re.sub(r"\bstd::abs\(", "Math.Abs(", text)
    text = re.sub(r"(?<![\w.:])(?:std::)?sqrt\(", "Math.Sqrt(", text)
    text = re.sub(r"(?<![\w.:])(?:std::)?log\(", "Math.Log(", text)
    text = re.sub(r"(?<![\w.:])(?:std::)?exp\(", "Math.Exp(", text)
    text = re.sub(r"\(void\)\s*(\w+)\s*;", r"_ = \1;", text)
    text = re.sub(r"\bnullptr\b", "default", text)
    text = literals(text)
    # (d__1 = X, Math.Abs(d__1)) -> Math.Abs(d__1 = X)
    text = re.sub(r"\((d__\d+) = ([^,;]+?), Math\.Abs\(\1\)\)", r"Math.Abs(\1 = \2)", text)
    # statement-level comma: a = x, b = y;
    def split_stmt(m):
        parts = split_top(m.group(2))
        if len(parts) > 1 and all(re.match(r"\s*\w+\s*=", p) for p in parts):
            return m.group(1) + ("; ").join(p.strip() for p in parts) + ";"
        return m.group(0)
    text = re.sub(r"^(\s*)(\w+\s*=[^;{}]*);", split_stmt, text, flags=re.M)
    return text


def labels(text):
    code = strip_comments(text)
    used = set(re.findall(r"goto\s+(L\d+)\s*;", code))
    def fix(m):
        lab = m.group(2)
        if lab not in used:
            return f"{m.group(1)}/* {lab}: */"
        return m.group(0)
    text = re.sub(r"^(\s*)(L\d+):(?!:)", fix, text, flags=re.M)
    # a used label directly before a closing brace needs an empty statement
    text = re.sub(r"(\bL\d+:)(\s*\})", r"\1;\2", text)
    return text


def main():
    src, cls, outp = sys.argv[1], sys.argv[2], sys.argv[3]
    if len(sys.argv) > 4:
        KNOWN.update(json.load(open(sys.argv[4])))
    text = open(src, encoding="utf-8").read()
    # ApproxF2var calls its evaluator through a const_cast: a pseudo-function with the evaluator's modes, which the
    # output then writes as foncnp.Evaluate(...) (its spans fixed by hand).
    text = re.sub(r"\(\*const_cast<AdvApp2Var_EvaluatorFunc2Var\*>\(&(\w+)\)\)\s*\.Evaluate\(", r"\1_Evaluate(", text)
    KNOWN.update({n + "_Evaluate": "vccvvvcvvpr" for n in set(re.findall(r"(\w+)_Evaluate\(", text))})
    code = strip_comments(text)
    funcs = parse_functions(text, code)
    sigs = infer(funcs)
    out = []
    for f in funcs:
        if f.name.startswith("init"):
            continue  # the anonymous namespace's initialisation: ported by hand
        params = ", ".join(cs_param(f, p) for p in f.params)
        vis = "private" if f.is_static else "public"
        out.append("    //=================================================================================================\n\n")
        out.append(f"    {vis} static {f.ret} {f.name}({params})\n    {{")
        unnamed = [p["name"] for p in f.params if p.get("unnamed")]
        if unnamed:
            out.append("\n" + "".join(f"        _ = {n};\n" for n in unnamed))
        body = convert_body(f, sigs)
        # OCCT indents by 2, OCCT.NET by 4, inside a class
        body = "\n".join((" " * (4 + 2 * (len(l) - len(l.lstrip(" ")))) + l.lstrip(" ") if l.strip() else "")
                         for l in body.split("\n"))
        out.append(body.rstrip())
        out.append("\n    }\n\n")
    with open(outp, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("".join(out))
    print(json.dumps({k: v for k, v in sigs.items() if k not in KNOWN}, indent=0))


if __name__ == "__main__":
    main()
