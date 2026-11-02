using System.Text;
using System.Text.RegularExpressions;

namespace OcctGen;

/// <summary>
/// An instantiation of an NCollection template, bound under its legacy OCCT name (e.g.
/// TopTools_ListOfShape for NCollection_List&lt;TopoDS_Shape&gt;). OCCT 8 deprecates the typedefs,
/// so the shim spells the template out and only the C# side uses the alias.
/// </summary>
sealed class CollectionInfo(string kind, string alias, string spelling)
{
    public string Kind { get; } = kind;
    public string Alias { get; } = alias;
    /// <summary>The template instantiation, normalised as clang spells canonical types.</summary>
    public string Spelling { get; } = Normalize(spelling);
    public string[] Args { get; } = SplitArgs(spelling);
    public Mapped Item { get; set; } = Mapped.Void;
    public Mapped Value { get; set; } = Mapped.Void;   // the mapped-to type of a data map
    public bool IsTransient => Kind is "harray1" or "harray2" or "hsequence";

    /// <summary>The non-transient collection the operations run on (an HArray1 is an Array1).</summary>
    public string Base => Kind switch
    {
        "harray1" => $"NCollection_Array1<{Args[0]}>",
        "harray2" => $"NCollection_Array2<{Args[0]}>",
        "hsequence" => $"NCollection_Sequence<{Args[0]}>",
        _ => Spelling,
    };

    public static readonly string[] Kinds =
        ["list", "sequence", "hsequence", "array1", "harray1", "array2", "harray2", "map", "indexedmap", "datamap", "indexeddatamap"];

    public static string Normalize(string s) =>
        Regex.Replace(Regex.Replace(s, @"\b(const|class|struct)\s+", ""), @"\s+", "")
            .Replace("opencascade::", "");

    /// <summary>Headers declaring the template and every class named in its arguments.</summary>
    public IEnumerable<string> Headers() =>
        Regex.Matches(Spelling, "[A-Za-z_][A-Za-z0-9_]*").Select(m => m.Value)
            .Where(n => n is not ("double" or "int" or "bool" or "float"))
            .Append(Spelling[..Spelling.IndexOf('<')]).Distinct();

    static string[] SplitArgs(string spelling)
    {
        var s = Normalize(spelling);
        var inner = s[(s.IndexOf('<') + 1)..s.LastIndexOf('>')];
        var args = new List<string>();
        int depth = 0, start = 0;
        for (int i = 0; i < inner.Length; i++)
        {
            if (inner[i] == '<') depth++;
            else if (inner[i] == '>') depth--;
            else if (inner[i] == ',' && depth == 0)
            {
                args.Add(inner[start..i]);
                start = i + 1;
            }
        }
        args.Add(inner[start..]);
        return args.ToArray();
    }
}

/// <summary>
/// Emits collections from a table of operations shared by the C++ and C# sides. In an operation,
/// "E" is the item (or key) type, "V" a data map's value type; C++ bodies call the collection "s".
/// </summary>
static class CollectionEmitter
{
    sealed record Op(string Name, string Ret, (string Type, string Name)[] Params, string Body, string? Doc = null);

    static readonly (string, string)[] None = [];
    static (string, string)[] P(params (string, string)[] ps) => ps;

    static IEnumerable<Op> Ops(CollectionInfo c) => c.Kind switch
    {
        "list" =>
        [
            new("Extent", "int", None, "s.Extent()"),
            new("Append", "void", P(("E", "item")), "s.Append(item)"),
            new("Prepend", "void", P(("E", "item")), "s.Prepend(item)"),
            new("First", "E", None, "s.First()"),
            new("Last", "E", None, "s.Last()"),
            new("RemoveFirst", "void", None, "s.RemoveFirst()"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        "sequence" or "hsequence" =>
        [
            new("Length", "int", None, "s.Length()"),
            new("Append", "void", P(("E", "item")), "s.Append(item)"),
            new("Prepend", "void", P(("E", "item")), "s.Prepend(item)"),
            new("Value", "E", P(("int", "index")), "s.Value(index)"),
            new("SetValue", "void", P(("int", "index"), ("E", "item")), "s.SetValue(index, item)"),
            new("Remove", "void", P(("int", "index")), "s.Remove(index)"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        "array1" or "harray1" =>
        [
            new("Lower", "int", None, "s.Lower()"),
            new("Upper", "int", None, "s.Upper()"),
            new("Length", "int", None, "s.Length()"),
            new("Value", "E", P(("int", "index")), "s.Value(index)"),
            new("SetValue", "void", P(("int", "index"), ("E", "item")), "s.SetValue(index, item)"),
        ],
        "array2" or "harray2" =>
        [
            new("LowerRow", "int", None, "s.LowerRow()"),
            new("UpperRow", "int", None, "s.UpperRow()"),
            new("LowerCol", "int", None, "s.LowerCol()"),
            new("UpperCol", "int", None, "s.UpperCol()"),
            new("ColLength", "int", None, "s.ColLength()"),
            new("RowLength", "int", None, "s.RowLength()"),
            new("Value", "E", P(("int", "row"), ("int", "col")), "s.Value(row, col)"),
            new("SetValue", "void", P(("int", "row"), ("int", "col"), ("E", "item")), "s.SetValue(row, col, item)"),
        ],
        "map" =>
        [
            new("Extent", "int", None, "s.Extent()"),
            new("Add", "bool", P(("E", "key")), "s.Add(key)"),
            new("Contains", "bool", P(("E", "key")), "s.Contains(key)"),
            new("Remove", "bool", P(("E", "key")), "s.Remove(key)"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        "indexedmap" =>
        [
            new("Extent", "int", None, "s.Extent()"),
            new("Add", "int", P(("E", "key")), "s.Add(key)"),
            new("Contains", "bool", P(("E", "key")), "s.Contains(key)"),
            new("FindIndex", "int", P(("E", "key")), "s.FindIndex(key)"),
            new("FindKey", "E", P(("int", "index")), "s.FindKey(index)"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        "datamap" =>
        [
            new("Extent", "int", None, "s.Extent()"),
            new("Bind", "bool", P(("E", "key"), ("V", "item")), "s.Bind(key, item)"),
            new("IsBound", "bool", P(("E", "key")), "s.IsBound(key)"),
            new("Find", "V", P(("E", "key")), "s.Find(key)"),
            new("UnBind", "bool", P(("E", "key")), "s.UnBind(key)"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        "indexeddatamap" =>
        [
            new("Extent", "int", None, "s.Extent()"),
            new("Add", "int", P(("E", "key"), ("V", "item")), "s.Add(key, item)"),
            new("Contains", "bool", P(("E", "key")), "s.Contains(key)"),
            new("FindIndex", "int", P(("E", "key")), "s.FindIndex(key)"),
            new("FindKey", "E", P(("int", "index")), "s.FindKey(index)"),
            new("FindFromIndex", "V", P(("int", "index")), "s.FindFromIndex(index)"),
            new("FindFromKey", "V", P(("E", "key")), "s.FindFromKey(key)"),
            new("Clear", "void", None, "s.Clear()"),
        ],
        _ => throw new InvalidOperationException(c.Kind),
    };

    static (string Params, string Args)[] Ctors(CollectionInfo c) => c.Kind switch
    {
        "array1" or "harray1" => [("int lower, int upper", "lower, upper")],
        "array2" or "harray2" => [("int rowLower, int rowUpper, int colLower, int colUpper", "rowLower, rowUpper, colLower, colUpper")],
        _ => [("", "")],
    };

    /// <summary>How to walk the items (keys) into the Fill buffer: a C++ loop header and the item at each step; null if not enumerable.</summary>
    static (string Count, string Loop, string Item)? Walk(CollectionInfo c) => c.Kind switch
    {
        "list" => ("s.Extent()", $"for ({c.Base}::Iterator it(s); it.More(); it.Next())", "it.Value()"),
        "sequence" or "hsequence" => ("s.Length()", "for (int i = 1; i <= s.Length(); i++)", "s.Value(i)"),
        "array1" or "harray1" => ("s.Length()", "for (int i = s.Lower(); i <= s.Upper(); i++)", "s.Value(i)"),
        "map" or "datamap" => ("s.Extent()", $"for ({c.Base}::Iterator it(s); it.More(); it.Next())", "it.Key()"),
        "indexedmap" or "indexeddatamap" => ("s.Extent()", "for (int i = 1; i <= s.Extent(); i++)", "s.FindKey(i)"),
        _ => null,
    };

    static Mapped Resolve(CollectionInfo c, string sym) => sym switch
    {
        "E" => c.Item,
        "V" => c.Value,
        "int" => new Mapped(Kind.Prim, "int32_t"),
        "bool" => new Mapped(Kind.Bool, "bool"),
        "void" => Mapped.Void,
        _ => throw new ArgumentException(sym),
    };

    // ---- C++ ----

    public static string EmitCpp(CollectionInfo c, Func<string, IEnumerable<string>> headersOf)
    {
        var sb = new StringBuilder();
        sb.AppendLine("// <auto-generated> by tools/OcctGen from bindings/occt.txt. Do not edit. </auto-generated>");
        sb.AppendLine("#include \"b3d_shim.h\"");
        var headers = new SortedSet<string>(c.Headers().Select(h => h + ".hxx"));
        foreach (var t in new[] { c.Item, c.Value }.Where(t => t.Kind == Kind.Value))
            headers.UnionWith(headersOf(t.Name));
        if (c.IsTransient)
            headers.Add(c.Spelling[..c.Spelling.IndexOf('<')] + ".hxx");
        foreach (var h in headers)
            sb.AppendLine($"#include <{h}>");
        sb.AppendLine();

        var self = c.Spelling;
        var a = c.Alias;
        sb.AppendLine(c.IsTransient
            ? $"B3D_API void b3d_{a}_delete(void* p) {{ b3d_release(static_cast<{self}*>(p)); }}"
            : $"B3D_API void b3d_{a}_delete(void* p) {{ delete static_cast<{self}*>(p); }}");
        if (c.IsTransient)
        {
            sb.AppendLine($"B3D_API void* b3d_{a}_as_Standard_Transient(void* p) {{ return static_cast<Standard_Transient*>(static_cast<{self}*>(p)); }}");
            sb.AppendLine($"B3D_API void* b3d_{a}_downcast(void* t) {{ return b3d_retain(dynamic_cast<{self}*>(static_cast<Standard_Transient*>(t))); }}");
        }
        else
            Fn(sb, "void*", $"b3d_{a}_copy", "const void* p", $"return new {self}(*static_cast<const {self}*>(p));");

        int i = 0;
        foreach (var (ps, args) in Ctors(c))
        {
            var make = $"new {self}({args})";
            Fn(sb, "void*", $"b3d_{a}_new{i++}", ps, c.IsTransient ? $"return b3d_retain({make});" : $"return {make};");
        }

        string selfRef = $"{c.Base}& s = *static_cast<{self}*>(self);";
        foreach (var op in Ops(c))
        {
            var ret = Resolve(c, op.Ret);
            var ps = string.Join(", ", op.Params.Select(p => $"{CParam(Resolve(c, p.Type))} {p.Name}").Prepend("void* self"));
            var body = op.Body;
            foreach (var p in op.Params)
                body = Regex.Replace(body, $@"\b{p.Name}\b", CArg(Resolve(c, p.Type), p.Name));
            Fn(sb, CRet(ret), $"b3d_{a}_{op.Name}", ps,
               selfRef + (ret.Kind == Kind.Void ? $" {body};" : $" return {CConv(ret, body)};"));
        }

        if (Walk(c) is (var count, var loop, var at))
        {
            var emit = c.Item.Kind switch
            {
                Kind.Value => $"out[n++] = new {c.Item.CppName}(x)",
                Kind.Bool => "out[n++] = (x) ? 1 : 0",
                _ => "out[n++] = x",
            };
            var outType = c.Item.Kind switch { Kind.Value => "void**", Kind.Bool => "uint8_t*", _ => c.Item.Name + "*" };
            // The caller sizes the buffer from Count
            Fn(sb, "int32_t", $"b3d_{a}_Count", "void* self", $"{selfRef} return {count};");
            Fn(sb, "void", $"b3d_{a}_Fill", $"void* self, {outType} out",
               $"{selfRef} int n = 0; {loop} {{ auto&& x = {at}; {emit}; }}");
        }
        return sb.ToString();
    }

    static void Fn(StringBuilder sb, string ret, string name, string ps, string body)
    {
        sb.AppendLine();
        sb.AppendLine($"B3D_API {ret} {name}({(ps.Length > 0 ? ps + ", " : "")}b3d_error** err)");
        sb.AppendLine("{");
        sb.AppendLine("  *err = nullptr;");
        sb.AppendLine($"  B3D_TRY {{ {body} }}");
        sb.AppendLine("  B3D_CATCH(err)");
        if (ret != "void")
            sb.AppendLine("  return {};");
        sb.AppendLine("}");
    }

    static string CParam(Mapped t) => t.Kind switch
    {
        Kind.Value => $"const {t.CppName}*",
        Kind.Bool => "uint8_t",
        Kind.Prim => t.Name,
        _ => throw new ArgumentException(t.ToString()),
    };

    static string CArg(Mapped t, string n) => t.Kind switch
    {
        Kind.Value => $"*{n}",
        Kind.Bool => $"({n} != 0)",
        _ => n,
    };

    static string CRet(Mapped t) => t.Kind switch
    {
        Kind.Void => "void",
        Kind.Value => "void*",
        Kind.Bool => "uint8_t",
        _ => t.Name,
    };

    static string CConv(Mapped t, string e) => t.Kind switch
    {
        Kind.Value => $"new {t.CppName}({e})",
        Kind.Bool => $"({e}) ? 1 : 0",
        _ => e,
    };

    // ---- C# ----

    public static string EmitCs(CollectionInfo c)
    {
        var sb = new StringBuilder();
        var a = c.Alias;
        var walk = Walk(c);
        var item = CsType(c.Item);
        sb.AppendLine("// <auto-generated> by tools/OcctGen from bindings/occt.txt. Do not edit. </auto-generated>");
        sb.AppendLine("#nullable enable");
        sb.AppendLine("using System.Runtime.InteropServices;");
        sb.AppendLine();
        sb.AppendLine("namespace Build123d.Occt;");
        sb.AppendLine();
        sb.AppendLine($"/// <summary><c>{c.Spelling}</c>.</summary>");
        var bases = (c.IsTransient ? "OcctTransient" : "OcctObject")
                    + (walk is null ? "" : $", IEnumerable<{item}>");
        sb.AppendLine($"public sealed partial class {a} : {bases}");
        sb.AppendLine("{");
        sb.AppendLine($"    internal {a}(IntPtr ptr) : base(ptr) {{ }}");
        sb.AppendLine();
        sb.AppendLine($"    protected override void ReleaseNative(IntPtr ptr) => __N.b3d_{a}_delete(ptr);");
        sb.AppendLine($"    internal IntPtr PtrAs_{a}() => Ptr;");
        if (c.IsTransient)
        {
            sb.AppendLine("    internal override IntPtr PtrAs_Standard_Transient() => __N.b3d_" + a + "_as_Standard_Transient(Ptr);");
            sb.AppendLine();
            sb.AppendLine($"    /// <summary>The object as a {a}, or null if it is not one.</summary>");
            sb.AppendLine($"    public static {a}? DownCast(OcctTransient? obj)");
            sb.AppendLine("    {");
            sb.AppendLine("        if (obj is null) return null;");
            sb.AppendLine($"        var p = __N.b3d_{a}_downcast(obj.PtrAs_Standard_Transient());");
            sb.AppendLine("        GC.KeepAlive(obj);");
            sb.AppendLine($"        return p == IntPtr.Zero ? null : new {a}(p);");
            sb.AppendLine("    }");
        }
        else
        {
            sb.AppendLine();
            sb.AppendLine($"    /// <summary>A copy of another {a}.</summary>");
            sb.AppendLine($"    public {a}({a} other) : this(__Copy(other)) {{ }}");
            sb.AppendLine();
            sb.AppendLine($"    private static IntPtr __Copy({a} other)");
            sb.AppendLine("    {");
            sb.AppendLine("        ArgumentNullException.ThrowIfNull(other);");
            sb.AppendLine($"        var __r = __N.b3d_{a}_copy(other.PtrAs_{a}(), out var __err);");
            sb.AppendLine("        GC.KeepAlive(other);");
            sb.AppendLine("        OcctException.ThrowIfError(__err);");
            sb.AppendLine("        return __r;");
            sb.AppendLine("    }");
        }

        var natives = new List<(string Ret, string Name, string Params)>
        {
            ("void", $"b3d_{a}_delete", "IntPtr p"),
        };
        if (c.IsTransient)
        {
            natives.Add(("IntPtr", $"b3d_{a}_as_Standard_Transient", "IntPtr p"));
            natives.Add(("IntPtr", $"b3d_{a}_downcast", "IntPtr t"));
        }
        else
            natives.Add(("IntPtr", $"b3d_{a}_copy", "IntPtr p, out IntPtr err"));

        int i = 0;
        foreach (var (ps, args) in Ctors(c))
        {
            sb.AppendLine();
            sb.AppendLine($"    public {a}({ps}) : this(__New{i}({args})) {{ }}");
            sb.AppendLine();
            sb.AppendLine($"    private static IntPtr __New{i}({ps})");
            sb.AppendLine("    {");
            sb.AppendLine($"        var __r = __N.b3d_{a}_new{i}({(args.Length > 0 ? args + ", " : "")}out var __err);");
            sb.AppendLine("        OcctException.ThrowIfError(__err);");
            sb.AppendLine("        return __r;");
            sb.AppendLine("    }");
            natives.Add(("IntPtr", $"b3d_{a}_new{i}", (ps.Length > 0 ? ps + ", " : "") + "out IntPtr err"));
            i++;
        }

        foreach (var op in Ops(c))
        {
            var ret = Resolve(c, op.Ret);
            var ps = op.Params.Select(p => (T: Resolve(c, p.Type), p.Name)).ToList();
            sb.AppendLine();
            sb.AppendLine($"    public {(ret.Kind == Kind.Void ? "void" : CsType(ret))} {op.Name}({string.Join(", ", ps.Select(p => $"{CsType(p.T)} {p.Name}"))})");
            sb.AppendLine("    {");
            foreach (var p in ps.Where(p => p.T.Kind == Kind.Value))
                sb.AppendLine($"        ArgumentNullException.ThrowIfNull({p.Name});");
            var args = ps.Select(p => p.T.Kind switch
            {
                Kind.Value => $"{p.Name}.PtrAs_{p.T.Name}()",
                Kind.Bool => $"{p.Name} ? (byte)1 : (byte)0",
                _ => p.Name,
            }).Prepend("Ptr").Append("out var __err");
            sb.AppendLine($"        {(ret.Kind == Kind.Void ? "" : "var __r = ")}__N.b3d_{a}_{op.Name}({string.Join(", ", args)});");
            sb.AppendLine("        GC.KeepAlive(this);");
            foreach (var p in ps.Where(p => p.T.Kind == Kind.Value))
                sb.AppendLine($"        GC.KeepAlive({p.Name});");
            sb.AppendLine("        OcctException.ThrowIfError(__err);");
            if (ret.Kind != Kind.Void)
                sb.AppendLine($"        return {ret.Kind switch { Kind.Value => $"new {ret.Name}(__r)", Kind.Bool => "__r != 0", _ => "__r" }};");
            sb.AppendLine("    }");
            natives.Add((NativeType(ret), $"b3d_{a}_{op.Name}",
                         string.Join(", ", ps.Select(p => $"{NativeType(p.T)} {p.Name}").Prepend("IntPtr self").Append("out IntPtr err"))));
        }

        if (walk is not null)
        {
            var buf = c.Item.Kind switch { Kind.Value => "IntPtr", Kind.Bool => "byte", _ => item };
            var conv = c.Item.Kind switch { Kind.Value => $"new {c.Item.Name}(x)", Kind.Bool => "x != 0", _ => "x" };
            sb.AppendLine();
            sb.AppendLine("    /// <summary>The items (or keys), in the collection's order.</summary>");
            sb.AppendLine($"    public {item}[] ToArray()");
            sb.AppendLine("    {");
            sb.AppendLine($"        int n = __N.b3d_{a}_Count(Ptr, out var __err);");
            sb.AppendLine("        OcctException.ThrowIfError(__err);");
            sb.AppendLine($"        var buf = new {buf}[n];");
            sb.AppendLine($"        __N.b3d_{a}_Fill(Ptr, buf, out __err);");
            sb.AppendLine("        GC.KeepAlive(this);");
            sb.AppendLine("        OcctException.ThrowIfError(__err);");
            sb.AppendLine(buf == item ? "        return buf;" : $"        return Array.ConvertAll(buf, x => {conv});");
            sb.AppendLine("    }");
            sb.AppendLine();
            sb.AppendLine($"    public IEnumerator<{item}> GetEnumerator() => ((IEnumerable<{item}>)ToArray()).GetEnumerator();");
            sb.AppendLine("    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();");
            natives.Add(("int", $"b3d_{a}_Count", "IntPtr self, out IntPtr err"));
            natives.Add(("void", $"b3d_{a}_Fill", $"IntPtr self, [Out] {buf}[] buf, out IntPtr err"));
        }

        sb.AppendLine();
        sb.AppendLine("    private static partial class __N");
        sb.AppendLine("    {");
        foreach (var (ret, name, ps) in natives)
        {
            sb.AppendLine($"        [LibraryImport(\"b3docct\")]");
            sb.AppendLine($"        internal static partial {ret} {name}({ps});");
        }
        sb.AppendLine("    }");
        sb.AppendLine("}");
        return sb.ToString();
    }

    static string CsType(Mapped t) => t.Kind switch
    {
        Kind.Value => t.Name,
        Kind.Bool => "bool",
        Kind.Prim => Mapped.CsPrim(t.Name),
        _ => throw new ArgumentException(t.ToString()),
    };

    static string NativeType(Mapped t) => t.Kind switch
    {
        Kind.Void => "void",
        Kind.Value => "IntPtr",
        Kind.Bool => "byte",
        Kind.Prim => Mapped.CsPrim(t.Name),
        _ => throw new ArgumentException(t.ToString()),
    };
}
