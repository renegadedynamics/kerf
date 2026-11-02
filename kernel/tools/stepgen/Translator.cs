// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Translates C++ function bodies to C# by docs/translation-rules.md: statements re-laid out one per line with
// OCCT's comments kept, expressions parsed and printed with the C++ types known (handles become references, a
// non-const reference to a value becomes ref, SELECT types are copied where C++ copies them).

using System.Text;
using System.Text.RegularExpressions;

namespace StepGen;

internal sealed class FnCtx
{
    public CppClass Cls = null!;
    public CppMethod? Method;
    public Dictionary<string, CppType> Locals = new(StringComparer.Ordinal);
    public HashSet<string> TemplateParams = [];
    public List<string> Problems = [];
    /// <summary>File-level helpers of the class's source (anonymous namespace), callable unqualified.</summary>
    public List<CppMethod> Helpers = [];
    public List<CppField> FileConstants = [];
}

internal sealed class Translator
{
    private readonly Db myDb;
    private readonly FnCtx c;
    private readonly List<string> myLines = [];
    private int myIndent;

    // scopes, for C#'s rule that a nested local can't take an enclosing local's name (C++ shadows)
    private sealed class Scope
    {
        public int Id;
        public Dictionary<string, string> Renames = new(StringComparer.Ordinal);
    }

    private readonly List<Scope> myScopes = [new Scope { Id = -1 }];
    /// <summary>Renames decided after a first pass: (scope id, name) to the new name.</summary>
    public Dictionary<(int, string), string> PlannedRenames = [];
    /// <summary>The declarations seen: the scope path (outermost first), and the name.</summary>
    public readonly List<(List<int> Path, string Name)> Decls = [];
    /// <summary>How many times each declared local is read after its declaration.</summary>
    public readonly Dictionary<string, int> Uses = new(StringComparer.Ordinal);
    public readonly HashSet<string> Declared = new(StringComparer.Ordinal);

    private void PushScope(int theId) => myScopes.Add(new Scope { Id = theId });

    private void PopScope() => myScopes.RemoveAt(myScopes.Count - 1);

    /// <summary>Declares a local in the current scope; returns its C# name.</summary>
    public string DeclareLocal(string theName)
    {
        var aScope = myScopes[^1];
        Decls.Add((myScopes.Select(x => x.Id).ToList(), theName));
        Declared.Add(theName);
        if (PlannedRenames.TryGetValue((aScope.Id, theName), out string? aNew))
        {
            aScope.Renames[theName] = aNew;
            return aNew;
        }

        return theName;
    }

    /// <summary>Renames for the declarations that shadow an enclosing one (none if C# accepts them all).</summary>
    public Dictionary<(int, string), string> ShadowRenames(IEnumerable<string> theParams)
    {
        var aResult = new Dictionary<(int, string), string>();
        var aParamList = theParams.ToList();
        var aAll = new HashSet<string>(Decls.Select(d => d.Name).Concat(aParamList), StringComparer.Ordinal);
        var aParams = new HashSet<string>(aParamList, StringComparer.Ordinal);
        foreach (var d in Decls)
        {
            int aScope = d.Path[^1];
            bool aShadows = (aParams.Contains(d.Name) && aScope != -1)
                            || Decls.Any(e => e.Name == d.Name && e.Path.Count < d.Path.Count && d.Path[e.Path.Count - 1] == e.Path[^1]);
            if (aShadows && !aResult.ContainsKey((aScope, d.Name)))
            {
                int n = 1;
                while (aAll.Contains(d.Name + n))
                {
                    n++;
                }

                aAll.Add(d.Name + n);
                aResult[(aScope, d.Name)] = d.Name + n;
            }
        }

        return aResult;
    }

    private string? RenamedLocal(string theName)
    {
        for (int k = myScopes.Count - 1; k >= 0; k--)
        {
            if (myScopes[k].Renames.TryGetValue(theName, out string? aNew))
            {
                return aNew;
            }
        }

        return null;
    }

    public Translator(Db theDb, FnCtx theCtx, int theIndent)
    {
        myDb = theDb;
        c = theCtx;
        myIndent = theIndent;
    }

    public List<string> Lines => myLines;

    // ------------------------------------------------------------------ types

    public bool IsTypeName(string theName)
    {
        if (Parser.IsBuiltin(theName) || c.TemplateParams.Contains(theName))
        {
            return true;
        }

        if (theName is "occ::handle" or "Handle" or "std::array" or "std::string")
        {
            return true;
        }

        if (theName.StartsWith("NCollection_", StringComparison.Ordinal) || theName.StartsWith("TCollection_", StringComparison.Ordinal)
            || theName.StartsWith("gp_", StringComparison.Ordinal))
        {
            return true;
        }

        return myDb.Classes.ContainsKey(theName) || myDb.Enums.ContainsKey(theName)
               || theName is "Standard_Transient" or "Standard_Type" or "StepData_Logical" or "Interface_ParamType";
    }

    private CppType? LookupVar(string theName)
    {
        var t = LookupVar0(theName);
        return t == null ? null : myDb.Resolve(t);
    }

    private CppType? LookupVar0(string theName)
    {
        if (c.Locals.TryGetValue(theName, out var t))
        {
            return t;
        }

        foreach (var cls in myDb.Ancestors(c.Cls.Name))
        {
            foreach (var f in cls.Fields)
            {
                if (f.Name == theName)
                {
                    if (f.ArraySize != null && f.Type.Name == "char" && f.IsConstexpr)
                    {
                        return new CppType { Name = "char", Pointers = 1, IsConst = true };
                    }

                    if (f.ArraySize != null)
                    {
                        return new CppType { Name = "std::array", Args = [f.Type.Bare()], ValueArgs = [f.ArraySize] };
                    }

                    return f.Type;
                }
            }
        }

        foreach (var f in c.FileConstants)
        {
            if (f.Name == theName)
            {
                return f.Type;
            }
        }

        if (myDb.EnumOfMember.TryGetValue(theName, out string? anEnum))
        {
            return new CppType { Name = anEnum };
        }

        return null;
    }

    private static CppType Simple(string theName) => new() { Name = theName };

    private CppType? ReturnOf(CppMethod m, CppType? theRecv)
    {
        if (m.Return == null)
        {
            return null;
        }

        // a template method returning occ::handle<T>: unknown T
        return m.Return;
    }

    /// <summary>The methods a call may reach, and its receiver's class.</summary>
    private List<CppMethod> Candidates(CallExpr theCall, out bool theIsBaseCall, out string? theQualifier)
    {
        theIsBaseCall = false;
        theQualifier = null;
        var aResult = new List<CppMethod>();
        switch (theCall.Fn)
        {
            case MemberExpr m when m.Name.Contains("::", StringComparison.Ordinal):
            {
                int k = m.Name.LastIndexOf("::", StringComparison.Ordinal);
                aResult.AddRange(myDb.FindMethods(m.Name[..k], m.Name[(k + 2)..]));
                break;
            }
            case MemberExpr m:
            {
                var aRecv = InferType(m.Obj);
                if (aRecv != null)
                {
                    aResult.AddRange(myDb.FindMethods(aRecv.Element.Name, m.Name));
                }

                break;
            }
            case NameExpr n when !n.Name.Contains("::", StringComparison.Ordinal):
                aResult.AddRange(c.Helpers.Where(h => h.Name == n.Name));
                aResult.AddRange(myDb.FindMethods(c.Cls.Name, n.Name));
                break;
            case NameExpr n:
            {
                int k = n.Name.LastIndexOf("::", StringComparison.Ordinal);
                string aClass = n.Name[..k], aName = n.Name[(k + 2)..];
                theQualifier = aClass;
                aResult.AddRange(myDb.FindMethods(aClass, aName));
                theIsBaseCall = !c.Cls.IsNamespace && aClass != c.Cls.Name && myDb.DerivesFrom(c.Cls.Name, aClass)
                                && aResult.Any(x => !x.IsStatic);
                break;
            }
        }

        return aResult;
    }

    private static List<CppMethod> ByArity(List<CppMethod> theCandidates, int theCount) =>
        theCandidates.Where(m => m.Params.Count >= theCount && m.Params.Count(p => p.Default == null) <= theCount).ToList();

    public CppType? InferType(Expr e)
    {
        switch (e)
        {
            case NameExpr n:
                if (n.Name == "this")
                {
                    return new CppType { Name = "occ::handle", Args = [Simple(c.Cls.Name)] };
                }

                if (n.Name is "true" or "false" or "Standard_True" or "Standard_False")
                {
                    return Simple("bool");
                }

                if (n.Name is "nullptr" or "NULL")
                {
                    return Simple("nullptr");
                }

                if (!n.Name.Contains("::", StringComparison.Ordinal))
                {
                    return LookupVar(n.Name);
                }

                {
                    int k = n.Name.LastIndexOf("::", StringComparison.Ordinal);
                    string aLast = n.Name[(k + 2)..];
                    if (myDb.EnumOfMember.TryGetValue(aLast, out string? anEnum))
                    {
                        return Simple(anEnum);
                    }

                    // a constant of a namespace (RW enum tools)
                    var aCls = myDb.Class(n.Name[..k]);
                    var f = aCls?.Fields.FirstOrDefault(x => x.Name == aLast);
                    if (f != null)
                    {
                        return f.ArraySize != null && f.Type.Name == "char" ? new CppType { Name = "char", Pointers = 1, IsConst = true } : f.Type;
                    }
                }

                return null;
            case LitExpr l:
                return l.Kind switch
                {
                    TokKind.String => new CppType { Name = "char", Pointers = 1, IsConst = true },
                    TokKind.Char => Simple("char"),
                    _ => IsRealLiteral(l.Text) ? Simple("double") : Simple("int"),
                };
            case ParenExpr p:
                return InferType(p.X);
            case CastExpr ce:
                return ce.Type;
            case NewExpr ne:
                return ne.ArraySize != null ? null : new CppType { Name = "occ::handle", Args = [ne.Type] };
            case TypeCallExpr tc:
                return tc.Type;
            case TernaryExpr t:
                return InferType(t.A) ?? InferType(t.B);
            case UnaryExpr u:
                return u.Op == "!" ? Simple("bool") : u.Op == "*" ? DerefType(InferType(u.X)) : InferType(u.X);
            case BinaryExpr b:
                if (b.Op is "==" or "!=" or "<" or ">" or "<=" or ">=" or "&&" or "||")
                {
                    return Simple("bool");
                }

                if (b.Op is "=" or "+=" or "-=" or "*=" or "/=" or ",")
                {
                    return b.Op == "," ? InferType(b.R) : InferType(b.L);
                }

                {
                    var l = InferType(b.L);
                    var r = InferType(b.R);
                    if (l?.Name == "double" || r?.Name == "double")
                    {
                        return Simple("double");
                    }

                    return l ?? r;
                }
            case IndexExpr ix:
            {
                var o = InferType(ix.Obj);
                if (o == null)
                {
                    return null;
                }

                if (o.IsCString)
                {
                    return Simple("char");
                }

                if (o.Name == "std::array" || o.Name.StartsWith("NCollection_", StringComparison.Ordinal))
                {
                    return o.Args.Count > 0 ? o.Args[0] : null;
                }

                if (o.IsHandle && o.Args[0].Name.StartsWith("NCollection_", StringComparison.Ordinal))
                {
                    return o.Args[0].Args.Count > 0 ? o.Args[0].Args[0] : null;
                }

                return null;
            }
            case CallExpr call:
                return InferCall(call);
            case MemberExpr me:
            {
                var o = InferType(me.Obj);
                var cls = myDb.Class(o?.Element.Name);
                var f = cls?.Fields.FirstOrDefault(x => x.Name == me.Name);
                return f?.Type;
            }
        }

        return null;
    }

    private static CppType? DerefType(CppType? t)
    {
        if (t == null)
        {
            return null;
        }

        if (t.Pointers > 0)
        {
            var a = t.Clone();
            a.Pointers--;
            return a;
        }

        return t.IsHandle ? t.Args[0] : t;
    }

    /// <summary>A collection held by value and called like a function (<c>types(i)</c>, C++'s operator()).</summary>
    private CppType? CalledCollection(CallExpr call)
    {
        if (call.Fn is NameExpr { TemplateArgs: null } cn && !cn.Name.Contains("::", StringComparison.Ordinal) && call.Args.Count == 1
            && LookupVar(cn.Name) is { IsHandle: false, Pointers: 0 } t && t.Name.StartsWith("NCollection_", StringComparison.Ordinal))
        {
            return t;
        }

        return null;
    }

    private bool IsStringValue(CppType? t) => t is { IsHandle: false } && (t.IsCString || myDb.Map(t) == "string");

    private CppType? InferCall(CallExpr call)
    {
        if (CalledCollection(call) is { } aColl)
        {
            return aColl.Args.Count > 0 ? aColl.Args[0] : null;
        }

        if (call.Fn is NameExpr n)
        {
            switch (n.Name)
            {
                case "STANDARD_TYPE":
                    return new CppType { Name = "occ::handle", Args = [Simple("Standard_Type")] };
                case "occ::down_cast":
                case "occ::handle::DownCast":
                    return new CppType { Name = "occ::handle", Args = [n.TemplateArgs![0]] };
                case "GetCasted":
                    return call.Args[0] is NameExpr tn ? new CppType { Name = "occ::handle", Args = [Simple(tn.Name)] } : null;
                case "strcmp":
                case "strlen":
                    return Simple("int");
                case "std::abs":
                case "std::sqrt":
                case "Abs":
                case "Sqrt":
                    return Simple("double");
                case "std::min":
                case "std::max":
                case "Min":
                case "Max":
                    return call.Args.Count > 0 ? InferType(call.Args[0]) : null;
            }
        }

        if (call.Fn is MemberExpr me)
        {
            var aRecv = InferType(me.Obj);
            if (aRecv != null)
            {
                var anElem = aRecv.Element;
                if (anElem.Name.StartsWith("NCollection_", StringComparison.Ordinal))
                {
                    switch (me.Name)
                    {
                        case "Value":
                        case "ChangeValue":
                        case "First":
                        case "Last":
                            return anElem.Args.Count > 0 ? anElem.Args[0] : null;
                        case "Length":
                        case "Size":
                        case "Upper":
                        case "Lower":
                        case "RowLength":
                        case "ColLength":
                        case "UpperRow":
                        case "LowerRow":
                        case "UpperCol":
                        case "LowerCol":
                        case "NbRows":
                        case "NbColumns":
                        case "Extent":
                            return Simple("int");
                        case "IsEmpty":
                            return Simple("bool");
                    }
                }

                if (me.Name == "IsNull" && (aRecv.IsHandle || !myDb.IsValueClass(aRecv)))
                {
                    return Simple("bool");
                }
            }
        }

        var aCands = Candidates(call, out _, out _);
        var aFit = ByArity(aCands, call.Args.Count);
        var m = aFit.FirstOrDefault() ?? aCands.FirstOrDefault();
        if (m == null)
        {
            return null;
        }

        var r = ReturnOf(m, null);
        // template methods returning their parameter (ReadMember<T>): unknown
        if (r != null && m.TemplateParams.Contains(r.Element.Name))
        {
            if (call.Fn is NameExpr tn && tn.TemplateArgs is { Count: > 0 })
            {
                return r.IsHandle ? new CppType { Name = "occ::handle", Args = [tn.TemplateArgs[0]] } : tn.TemplateArgs[0];
            }

            return null;
        }

        return r;
    }

    private static bool IsRealLiteral(string theText) =>
        !theText.StartsWith("0x", StringComparison.OrdinalIgnoreCase) && (theText.Contains('.') || theText.Contains('e') || theText.Contains('E'));

    /// <summary>A handle, or a type whose truth C++ tests as "not null".</summary>
    private bool IsHandleLike(CppType? t) => t != null && (t.IsHandle || (t.Pointers > 0 && !t.IsCString));

    // ------------------------------------------------------------------ expressions

    private static string Number(string theText)
    {
        string s = theText;
        if (s.StartsWith('.'))
        {
            s = "0" + s;
        }

        s = Regex.Replace(s, @"^(\d+)\.(?=[eE]|$|[fFlL]$)", "$1.0");
        if (s.EndsWith('L') || s.EndsWith('l'))
        {
            if (IsRealLiteral(s))
            {
                s = s[..^1];
            }
        }

        if (s.EndsWith("U", StringComparison.Ordinal) || s.EndsWith("u", StringComparison.Ordinal))
        {
            s = s[..^1];
        }

        return s;
    }

    private bool IsSelectish(CppType? t) => t != null && !t.IsHandle && t.Pointers == 0 && myDb.IsValueClass(t);

    private string Problem(string theWhat)
    {
        c.Problems.Add(theWhat);
        return "/* stepgen: " + theWhat.Replace("*/", "* /", StringComparison.Ordinal) + " */";
    }

    /// <summary>Prints an expression; <paramref name="theTop"/> when no operator surrounds it.</summary>
    public string Emit(Expr e, bool theTop = false)
    {
        switch (e)
        {
            case LitExpr l:
                return l.Kind == TokKind.Number ? Number(l.Text) : l.Text;
            case NameExpr n:
                return EmitName(n);
            case ParenExpr p:
                return "(" + Emit(p.X, true) + ")";
            case TernaryExpr t:
            {
                string s = Cond(t.C, false) + " ? " + Emit(t.A) + " : " + Emit(t.B);
                return theTop ? s : "(" + s + ")";
            }
            case UnaryExpr u:
                return EmitUnary(u, theTop);
            case BinaryExpr b:
                return EmitBinary(b, theTop);
            case CastExpr ce:
            {
                string aType = myDb.Map(ce.Type);
                if (ce.Type.IsHandle || ce.Kind == "dynamic_cast")
                {
                    string s = Emit(ce.X) + " as " + aType;
                    return theTop ? s : "(" + s + ")";
                }

                string x = Emit(ce.X);
                return "(" + aType + ")" + (IsSimple(ce.X) ? x : "(" + x + ")");
            }
            case NewExpr ne:
                if (ne.ArraySize != null)
                {
                    return "new " + myDb.Map(ne.Type) + "[" + Emit(ne.ArraySize, true) + "]";
                }

                return "new " + myDb.Map(ne.Type) + "(" + string.Join(", ", (ne.Args ?? []).Select(a => Emit(a, true))) + ")";
            case TypeCallExpr tc:
                return EmitTypeCall(tc);
            case InitListExpr il:
                return "{ " + string.Join(", ", il.Items.Select(a => Emit(a, true))) + " }";
            case IndexExpr ix:
            {
                var o = InferType(ix.Obj);
                if (o != null && o.IsCString)
                {
                    return "CStringChar(" + Emit(ix.Obj, true) + ", " + Emit(ix.Index, true) + ")";
                }

                return Emit(ix.Obj) + "[" + Emit(ix.Index, true) + "]";
            }
            case MemberExpr me:
                return EmitObj(me.Obj) + "." + Db.Id(me.Name) + TemplateSuffix(me.TemplateArgs);
            case CallExpr call:
                return EmitCall(call, theTop);
        }

        return Problem("unknown expression " + e.GetType().Name);
    }

    private static bool IsSimple(Expr e) => e is NameExpr or LitExpr or ParenExpr or CallExpr or MemberExpr or IndexExpr;

    private string EmitObj(Expr theObj)
    {
        // "*this" and "this" as receivers
        if (theObj is UnaryExpr { Op: "*" } u)
        {
            return EmitObj(u.X);
        }

        string s = Emit(theObj);
        return theObj is CastExpr or TernaryExpr or BinaryExpr or UnaryExpr ? "(" + s + ")" : s;
    }

    private string TemplateSuffix(List<CppType>? theArgs) =>
        theArgs is { Count: > 0 } ? "<" + string.Join(", ", theArgs.Select(myDb.Map)) + ">" : "";

    private string EmitName(NameExpr n)
    {
        switch (n.Name)
        {
            case "nullptr":
            case "NULL":
                return "null";
            case "Standard_True":
                return "true";
            case "Standard_False":
                return "false";
            case "std::abs":
            case "Abs":
                return "Math.Abs";
            case "std::sqrt":
                return "Math.Sqrt";
            case "std::min":
                return "Min";
            case "std::max":
                return "Max";
            case "std::endl":
                return "\"\\n\"";
        }

        if (n.Name.Contains("::", StringComparison.Ordinal))
        {
            string[] aParts = n.Name.Split("::");
            // Enum::Member, Class::Static
            return string.Join(".", aParts.Select(Db.Id)) + TemplateSuffix(n.TemplateArgs);
        }

        if (Declared.Contains(n.Name))
        {
            Uses[n.Name] = Uses.GetValueOrDefault(n.Name) + 1;
        }

        return Db.Id(RenamedLocal(n.Name) ?? n.Name) + TemplateSuffix(n.TemplateArgs);
    }

    private string EmitTypeCall(TypeCallExpr tc)
    {
        var t = tc.Type;
        if (t.IsHandle)
        {
            if (tc.Args.Count == 0)
            {
                return "null";
            }

            return Emit(tc.Args[0]);
        }

        string aMapped = myDb.Map(t);
        if (aMapped == "string")
        {
            if (tc.Args.Count == 0)
            {
                return "\"\"";
            }

            var a = InferType(tc.Args[0]);
            string s = Emit(tc.Args[0], true);
            return a != null && IsStringValue(a) ? s : "Standard_AsciiString(" + s + ")";
        }

        if (Db.BuiltinMap.ContainsKey(t.Name) || myDb.IsEnum(t.Name))
        {
            if (tc.Args.Count == 0)
            {
                return "default(" + aMapped + ")";
            }

            return "(" + aMapped + ")(" + Emit(tc.Args[0], true) + ")";
        }

        return "new " + aMapped + "(" + string.Join(", ", tc.Args.Select(a => Emit(a, true))) + ")";
    }

    private string EmitUnary(UnaryExpr u, bool theTop)
    {
        if (u.Postfix)
        {
            return Emit(u.X) + u.Op;
        }

        switch (u.Op)
        {
            case "!":
            {
                // !x.IsNull() -> x is not null; !handle -> handle is null; !int -> == 0
                if (u.X is CallExpr { Fn: MemberExpr { Name: "IsNull" } m } call && call.Args.Count == 0 && !IsSelectish(InferType(m.Obj)))
                {
                    string s = EmitObj(m.Obj) + " is not null";
                    return theTop ? s : "(" + s + ")";
                }

                if (u.X is CallExpr { Fn: NameExpr { Name: "strcmp" } } sc)
                {
                    string s = "string.CompareOrdinal(" + Emit(sc.Args[0], true) + ", " + Emit(sc.Args[1], true) + ") == 0";
                    return theTop ? s : "(" + s + ")";
                }

                var t = InferType(u.X);
                if (IsHandleLike(t) || t?.IsCString == true)
                {
                    string s = EmitObj(u.X) + " is null";
                    return theTop ? s : "(" + s + ")";
                }

                if (t != null && t.Pointers == 0 && t.Name is "int" or "Standard_Integer" or "unsigned" or "long" or "size_t" or "char")
                {
                    string s = EmitObj(u.X) + " == 0";
                    return theTop ? s : "(" + s + ")";
                }

                return "!" + EmitObj(u.X);
            }
            case "*":
                return Emit(u.X);
            case "&":
                return Problem("address-of: &" + Emit(u.X));
            case "sizeof":
                return Problem("sizeof");
        }

        return u.Op + Emit(u.X);
    }

    private string EmitBinary(BinaryExpr b, bool theTop)
    {
        string s;
        switch (b.Op)
        {
            case "=":
            {
                var lt = InferType(b.L);
                string r = Emit(b.R, true);
                if (IsSelectish(lt) && b.R is not (TypeCallExpr or NewExpr))
                {
                    r = "new " + myDb.Map(lt!.Bare()) + "(" + r + ")";
                }
                else if (lt is { IsHandle: true } && b.R is InitListExpr { Items.Count: 0 })
                {
                    r = "null";
                }
                else if (lt is { Name: "std::array" } && b.R is not InitListExpr && b.L is NameExpr)
                {
                    // std::array is a value: copied
                    r = r + ".ToArray()";
                }

                s = Emit(b.L) + " = " + r;
                return theTop ? s : "(" + s + ")";
            }
            case "&&":
            case "||":
                s = Cond(b.L, false) + " " + b.Op + " " + Cond(b.R, false);
                break;
            case "<<":
                if (IsStream(b))
                {
                    // std::cout << a << b: written to the console, as OCCT does
                    var aParts = new List<Expr>();
                    Expr x = b;
                    while (x is BinaryExpr { Op: "<<" } bb)
                    {
                        aParts.Insert(0, bb.R);
                        x = bb.L;
                    }

                    string aText = string.Join(" + ", aParts.Select(p => Emit(p)));
                    if (aParts.Count == 0 || InferType(aParts[0]) is not { IsCString: true })
                    {
                        aText = "\"\" + " + aText;
                    }

                    return "Console.Write(" + aText + ")";
                }

                s = Emit(b.L) + " << " + Emit(b.R);
                break;
            case ",":
                s = Emit(b.L, true) + ", " + Emit(b.R, true);
                break;
            case "<" or ">" or "<=" or ">=" when IsStringValue(InferType(b.L)) && IsStringValue(InferType(b.R)):
                // TCollection_AsciiString's comparison operators: ordinal
                s = "string.CompareOrdinal(" + Emit(b.L, true) + ", " + Emit(b.R, true) + ") " + b.Op + " 0";
                break;
            default:
                s = Emit(b.L) + " " + b.Op + " " + Emit(b.R);
                break;
        }

        return s; // the source's own parentheses are kept as ParenExpr
    }

    private static bool IsStream(BinaryExpr b)
    {
        Expr x = b;
        while (x is BinaryExpr { Op: "<<" } bb)
        {
            x = bb.L;
        }

        return x is NameExpr { Name: "std::cout" or "std::cerr" } || (x is CallExpr { Fn: NameExpr nn } && nn.Name.Contains("Send", StringComparison.Ordinal));
    }

    private static string Flatten(Expr e) => e switch
    {
        BinaryExpr b => Flatten(b.L) + " " + b.Op + " " + Flatten(b.R),
        NameExpr n => n.Name,
        LitExpr l => l.Text,
        _ => e.GetType().Name,
    };

    /// <summary>An expression used as a condition: C++'s implicit conversions to bool written out.</summary>
    public string Cond(Expr e, bool theTop = true)
    {
        var t = InferType(e);
        string s;
        if (IsHandleLike(t) || t?.IsCString == true)
        {
            s = EmitObj(e) + " is not null";
        }
        else if (t != null && t.Pointers == 0 && t.Name is "int" or "Standard_Integer" or "unsigned" or "long" or "size_t")
        {
            s = Emit(e) + " != 0";
        }
        else
        {
            return Emit(e, theTop);
        }

        return theTop ? s : "(" + s + ")";
    }

    private string EmitCall(CallExpr call, bool theTop)
    {
        // a collection called like a function: its indexer (an lvalue, as C++'s operator() returns a reference)
        if (CalledCollection(call) is not null)
        {
            return Emit(call.Fn) + "[" + Emit(call.Args[0], true) + "]";
        }

        if (call.Fn is MemberExpr sm)
        {
            var aRecvType = InferType(sm.Obj);
            // TCollection_AsciiString::IsEqual on a string: ==
            if (sm.Name == "IsEqual" && call.Args.Count == 1 && IsStringValue(aRecvType))
            {
                string s = EmitObj(sm.Obj) + " == " + Emit(call.Args[0]);
                return theTop ? s : "(" + s + ")";
            }

            if (aRecvType is { } && aRecvType.Element.Name.StartsWith("NCollection_", StringComparison.Ordinal) && aRecvType.Element.Name.Contains("Map", StringComparison.Ordinal))
            {
                // NCollection_DataMap::Find(key, item): the item is an out parameter
                if (sm.Name == "Find" && call.Args.Count == 2)
                {
                    return EmitObj(sm.Obj) + ".Find(" + Emit(call.Args[0], true) + ", out " + Emit(call.Args[1], true) + ")";
                }
            }

            // allocators are dropped (the port's collections have none)
            if (sm.Name == "Clear" && call.Args.Count == 1 && aRecvType is { } && aRecvType.Element.Name.StartsWith("NCollection_", StringComparison.Ordinal))
            {
                return EmitObj(sm.Obj) + ".Clear()";
            }
        }

        // special functions
        if (call.Fn is NameExpr n)
        {
            switch (n.Name)
            {
                case "STANDARD_TYPE":
                    return "typeof(" + EmitTypeArg(call.Args[0]) + ")";
                case "occ::down_cast":
                case "occ::handle::DownCast":
                {
                    string s = Emit(call.Args[0]) + " as " + myDb.Map(n.TemplateArgs![0]);
                    return theTop ? s : "(" + s + ")";
                }
                case "GetCasted":
                {
                    string s = Emit(call.Args[1]) + " as " + EmitTypeArg(call.Args[0]);
                    return theTop ? s : "(" + s + ")";
                }
                case "strcmp":
                    return "string.CompareOrdinal(" + Emit(call.Args[0], true) + ", " + Emit(call.Args[1], true) + ")";
                case "strlen":
                    return Emit(call.Args[0]) + ".Length";
                case "IsEqual" when call.Args.Count == 2 && (InferType(call.Args[0])?.IsCString == true || InferType(call.Args[1])?.IsCString == true):
                {
                    string s = Emit(call.Args[0]) + " == " + Emit(call.Args[1]);
                    return theTop ? s : "(" + s + ")";
                }
            }
        }

        if (call.Fn is MemberExpr me && call.Args.Count == 0)
        {
            var aRecv = InferType(me.Obj);
            bool isSelect = IsSelectish(aRecv) || (aRecv == null && false);
            if (me.Name == "IsNull" && !isSelect)
            {
                string s = EmitObj(me.Obj) + " is null";
                return theTop ? s : "(" + s + ")";
            }

            if (me.Name == "Nullify" && !isSelect)
            {
                return Emit(me.Obj) + " = null";
            }

            if (me.Name == "get")
            {
                return Emit(me.Obj);
            }

            // TCollection_AsciiString is a string: its C string is itself
            if (me.Name == "ToCString" && aRecv is { IsHandle: false, Pointers: 0 } && myDb.Map(aRecv) == "string")
            {
                return Emit(me.Obj);
            }

            if (aRecv == null && me.Name is "IsNull" or "Nullify")
            {
                c.Problems.Add($"{me.Name} on an expression of unknown type: assumed a handle");
            }
        }

        var aCands = Candidates(call, out bool isBaseCall, out string? aQualifier);
        var aFit = ByArity(aCands, call.Args.Count);
        var aMethod = PickOverload(aFit, call);

        // the callee
        string aFn;
        if (isBaseCall && call.Fn is NameExpr bn)
        {
            string aName = bn.Name[(bn.Name.LastIndexOf("::", StringComparison.Ordinal) + 2)..];
            aFn = BaseCall(aQualifier!, aName, aMethod) + "." + Db.Id(aName);
        }
        else if (call.Fn is MemberExpr q && q.Name.Contains("::", StringComparison.Ordinal))
        {
            // obj->Base::Method(): non-virtual, so the static type picks the version
            int k = q.Name.LastIndexOf("::", StringComparison.Ordinal);
            string aBase = q.Name[..k];
            string aName = q.Name[(k + 2)..];
            if (aMethod != null && (aMethod.IsVirtual || aMethod.IsOverride) && myDb.QualifiedCalls.Contains(q.Name))
            {
                aFn = EmitObj(q.Obj) + "." + aBase + "_" + aName;
            }
            else if (!HiddenBetween(InferType(q.Obj)?.Element.Name, aBase, aMethod))
            {
                aFn = EmitObj(q.Obj) + "." + Db.Id(aName);
            }
            else
            {
                aFn = "((" + aBase.Replace("::", ".", StringComparison.Ordinal) + ")" + Emit(q.Obj) + ")." + Db.Id(aName);
            }
        }
        else if (call.Fn is MemberExpr m2)
        {
            aFn = EmitObj(m2.Obj) + "." + Db.Id(m2.Name) + TemplateSuffix(m2.TemplateArgs);
        }
        else
        {
            aFn = Emit(call.Fn);
        }

        // the arguments, with ref where C++ binds a non-const reference to a value
        var aArgs = new List<string>();
        for (int k = 0; k < call.Args.Count; k++)
        {
            string a = Emit(call.Args[k], true);
            if (aMethod != null && k < aMethod.Params.Count)
            {
                var p = aMethod.Params[k];
                // C++ promotes bool to int (Send(int) for a bool): written out
                if (p.Type.Pointers == 0 && p.Type.Name is "int" or "Standard_Integer" or "double" or "Standard_Real"
                    && InferType(call.Args[k]) is { Name: "bool" or "Standard_Boolean", Pointers: 0 })
                {
                    a = "(" + a + " ? 1 : 0)";
                    c.Problems.Add($"bool passed as {p.Type.Name} to {aMethod.Name} (C++'s promotion, kept)");
                }

                string aMod = myDb.ParamModifier(p.Type);
                if (aMod.Length > 0)
                {
                    a = aMod + a;
                }
            }

            aArgs.Add(a);
        }

        // SetValue/Append of a SELECT type into an array or sequence: C++ copies the value
        if (call.Fn is MemberExpr sv && sv.Name is "SetValue" or "Append" or "Prepend" && aArgs.Count > 0)
        {
            var aRecv = InferType(sv.Obj);
            var anElem = aRecv?.Element;
            if (anElem != null && anElem.Name.StartsWith("NCollection_", StringComparison.Ordinal) && anElem.Args.Count > 0
                && IsSelectish(anElem.Args[0]) && call.Args[^1] is not (TypeCallExpr or NewExpr))
            {
                aArgs[^1] = "new " + myDb.Map(anElem.Args[0]) + "(" + aArgs[^1] + ")";
            }
        }

        return aFn + "(" + string.Join(", ", aArgs) + ")";
    }

    /// <summary>True when a class from <paramref name="theFrom"/> up to (not including) <paramref name="theBase"/> redeclares the method.</summary>
    private bool HiddenBetween(string? theFrom, string theBase, CppMethod? theMethod)
    {
        if (theFrom == null || theMethod == null)
        {
            return true;
        }

        string aKey = theMethod.SignatureKey(myDb.Map);
        foreach (var cls in myDb.Ancestors(theFrom))
        {
            if (cls.Name == theBase)
            {
                return false;
            }

            if (cls.Methods.Any(m => !m.IsCtor && m.SignatureKey(myDb.Map) == aKey))
            {
                return true;
            }
        }

        return true;
    }

    private CppMethod? PickOverload(List<CppMethod> theFit, CallExpr theCall)
    {
        if (theFit.Count <= 1)
        {
            return theFit.FirstOrDefault();
        }

        // the overloads agree on the ref arguments: any will do
        string Flags(CppMethod m) => string.Join(",", m.Params.Take(theCall.Args.Count).Select(p => myDb.ParamModifier(p.Type)));
        if (theFit.Select(Flags).Distinct().Count() == 1)
        {
            return theFit[0];
        }

        // pick by the arguments' types where they are known
        var aTypes = theCall.Args.Select(InferType).ToList();
        foreach (var m in theFit)
        {
            bool ok = true;
            for (int k = 0; k < aTypes.Count && ok; k++)
            {
                var a = aTypes[k];
                if (a == null)
                {
                    continue;
                }

                var p = m.Params[k].Type;
                bool aIsRefKind = myDb.IsRefKind(a);
                if (myDb.IsValueClass(a) != myDb.IsValueClass(p) && !p.IsHandle)
                {
                    ok = false;
                }

                _ = aIsRefKind;
            }

            if (ok)
            {
                return m;
            }
        }

        c.Problems.Add($"ambiguous overloads of {theFit[0].Name} with {theCall.Args.Count} arguments");
        return theFit[0];
    }

    /// <summary>
    /// A call to an ancestor's version (Ancestor::Method): base.Method when no class in between declares the same
    /// signature, otherwise a cast to the ancestor (non-virtual methods are resolved by the static type).
    /// </summary>
    private string BaseCall(string theAncestor, string theName, CppMethod? theMethod)
    {
        var aBase = myDb.Class(c.Cls.Base);
        if (aBase == null || theMethod == null)
        {
            return "base";
        }

        string aKey = theMethod.SignatureKey(myDb.Map);
        foreach (var cls in myDb.Ancestors(aBase.Name))
        {
            if (cls.Name == theAncestor)
            {
                return "base";
            }

            if (cls.Methods.Any(m => m.SignatureKey(myDb.Map) == aKey && !m.IsCtor))
            {
                if (theMethod.IsVirtual || theMethod.IsOverride)
                {
                    c.Problems.Add($"{theAncestor}::{theName} skips an override in {cls.Name}");
                }

                return "((" + theAncestor + ")this)";
            }
        }

        return "base";
    }

    private string EmitTypeArg(Expr e) => e switch
    {
        NameExpr n => n.Name.Replace("::", ".", StringComparison.Ordinal) + TemplateSuffix(n.TemplateArgs),
        TypeCallExpr tc => myDb.Map(tc.Type),
        _ => Emit(e),
    };

    // ------------------------------------------------------------------ statements

    private void Line(string theText) => myLines.Add(new string(' ', 4 * myIndent) + theText);

    private void Comment(Token k)
    {
        if (k.Kind == TokKind.Dropped)
        {
            Line(k.Text);
            return;
        }

        string aText = k.Text;
        if (aText.StartsWith("/*", StringComparison.Ordinal) && aText.Contains('\n'))
        {
            foreach (string l in aText.Split('\n'))
            {
                Line(l.Trim());
            }

            return;
        }

        if (!k.NewlineBefore && myLines.Count > 0 && !myLines[^1].TrimEnd().EndsWith('{'))
        {
            myLines[^1] += " " + aText;
            return;
        }

        Line(aText);
    }

    private int End(List<Token> t, int theFrom, int theTo)
    {
        // the ';' that ends a simple statement (outside brackets)
        int aDepth = 0;
        for (int j = theFrom; j < theTo; j++)
        {
            var k = t[j];
            if (k.IsComment || k.Kind is TokKind.String or TokKind.Char)
            {
                continue;
            }

            if (k.Is("(") || k.Is("[") || k.Is("{"))
            {
                aDepth++;
            }
            else if (k.Is(")") || k.Is("]") || k.Is("}"))
            {
                aDepth--;
            }
            else if (aDepth == 0 && k.Is(";"))
            {
                return j;
            }
        }

        throw new FormatException($"no ';' after line {t[theFrom].Line}");
    }

    public void Block(List<Token> t, int theFrom, int theTo)
    {
        int i = theFrom;
        bool aFirst = true;
        while (i < theTo)
        {
            var k = t[i];
            if (k.BlankBefore && !aFirst && myLines.Count > 0 && myLines[^1].Trim().Length > 0)
            {
                myLines.Add("");
            }

            aFirst = false;
            if (k.IsComment)
            {
                Comment(k);
                i++;
                continue;
            }

            i = Statement(t, i, theTo);
        }
    }

    /// <summary>Translates one statement starting at <paramref name="i"/>; returns the index after it.</summary>
    private int Statement(List<Token> t, int i, int theTo)
    {
        var k = t[i];
        try
        {
            if (k.Is("{"))
            {
                int anEnd = Parser.Match(t, i);
                Line("{");
                myIndent++;
                PushScope(i);
                Block(t, i + 1, anEnd);
                PopScope();
                myIndent--;
                Line("}");
                return anEnd + 1;
            }

            if (k.Is(";"))
            {
                return i + 1;
            }

            if (k.Is("if"))
            {
                int aClose = Parser.Match(t, i + 1);
                Line("if (" + Cond(ParseExpr(t, i + 2, aClose)) + ")");
                int j = Body(t, aClose + 1, theTo);
                j = SkipCommentsEmitting(t, j, theTo);
                while (j < theTo && t[j].Is("else"))
                {
                    if (t[j + 1].Is("if"))
                    {
                        int aClose2 = Parser.Match(t, j + 2);
                        Line("else if (" + Cond(ParseExpr(t, j + 3, aClose2)) + ")");
                        j = Body(t, aClose2 + 1, theTo);
                        j = SkipCommentsEmitting(t, j, theTo);
                        continue;
                    }

                    Line("else");
                    j = Body(t, j + 1, theTo);
                    break;
                }

                return j;
            }

            if (k.Is("for"))
            {
                PushScope(i);
                try
                {
                    return For(t, i, theTo);
                }
                finally
                {
                    PopScope();
                }
            }

            if (k.Is("while"))
            {
                int aClose = Parser.Match(t, i + 1);
                Line("while (" + Cond(ParseExpr(t, i + 2, aClose)) + ")");
                return Body(t, aClose + 1, theTo);
            }

            if (k.Is("do"))
            {
                Line("do");
                int j = Body(t, i + 1, theTo);
                // while (cond);
                int aClose = Parser.Match(t, j + 1);
                myLines[^1] += " while (" + Cond(ParseExpr(t, j + 2, aClose)) + ");";
                return aClose + 2;
            }

            if (k.Is("switch"))
            {
                int aClose = Parser.Match(t, i + 1);
                Line("switch (" + Emit(ParseExpr(t, i + 2, aClose), true) + ")");
                int anOpen = aClose + 1;
                int anEnd = Parser.Match(t, anOpen);
                Line("{");
                myIndent++;
                PushScope(anOpen);
                int j = anOpen + 1;
                while (j < anEnd)
                {
                    if (t[j].IsComment)
                    {
                        Comment(t[j]);
                        j++;
                        continue;
                    }

                    if (t[j].Is("case"))
                    {
                        int aColon = j + 1;
                        while (!t[aColon].Is(":"))
                        {
                            aColon++;
                        }

                        Line("case " + Emit(ParseExpr(t, j + 1, aColon), true) + ":");
                        j = aColon + 1;
                        continue;
                    }

                    if (t[j].Is("default") && t[j + 1].Is(":"))
                    {
                        Line("default:");
                        j += 2;
                        continue;
                    }

                    // "return x; break;": C# refuses the unreachable break
                    if (t[j].Is("break") && myLines.Count > 0
                        && myLines[^1].TrimStart().StartsWith("return", StringComparison.Ordinal) && myLines[^1].TrimEnd().EndsWith(';'))
                    {
                        j = End(t, j, anEnd) + 1;
                        continue;
                    }

                    myIndent++;
                    j = Statement(t, j, anEnd);
                    myIndent--;
                }

                // C++ lets the last section run off the switch's end; C# wants a break
                // (a block's last statement counts: "case 1: { ...; break; }")
                string aLast = myLines.LastOrDefault(l => l.Trim().Length > 0 && l.Trim() != "}" && !l.TrimStart().StartsWith("//", StringComparison.Ordinal))?.Trim() ?? "";
                if (!(aLast.StartsWith("break;", StringComparison.Ordinal) || aLast.StartsWith("return", StringComparison.Ordinal)
                      || aLast.StartsWith("throw", StringComparison.Ordinal) || aLast.StartsWith("continue;", StringComparison.Ordinal)
                      || aLast.StartsWith("switch (", StringComparison.Ordinal) || aLast == "{"))
                {
                    myIndent++;
                    Line("break;");
                    myIndent--;
                }

                PopScope();
                myIndent--;
                Line("}");
                return anEnd + 1;
            }

            if (k.Is("return"))
            {
                int anEnd = End(t, i, theTo);
                if (anEnd == i + 1)
                {
                    Line("return;");
                }
                else
                {
                    var e = ParseExpr(t, i + 1, anEnd);
                    string r = Emit(e, true);
                    var aRet = c.Method?.Return;
                    if (aRet != null && aRet.Name == "bool" && aRet.Pointers == 0)
                    {
                        r = Cond(e);
                    }
                    else if (aRet != null && aRet.IsHandle && e is InitListExpr { Items.Count: 0 })
                    {
                        r = "null";
                    }

                    if (aRet != null && aRet.IsRef && !aRet.IsConst && myDb.IsRefKind(aRet.Bare()))
                    {
                        r = "ref " + r;
                    }

                    Line("return " + r + ";");
                }

                return anEnd + 1;
            }

            if (k.Is("break") || k.Is("continue"))
            {
                Line(k.Text + ";");
                return End(t, i, theTo) + 1;
            }

            if (k.Is("throw"))
            {
                int anEnd = End(t, i, theTo);
                var e = ParseExpr(t, i + 1, anEnd);
                string s = Emit(e, true);
                if (!s.StartsWith("new ", StringComparison.Ordinal))
                {
                    s = "new " + s;
                }

                Line("throw " + s + ";");
                return anEnd + 1;
            }

            if (k.Is("try"))
            {
                Line("try");
                int j = Body(t, i + 1, theTo);
                while (j < theTo && t[j].Is("catch"))
                {
                    int aClose = Parser.Match(t, j + 1);
                    int p = j + 2;
                    string aCatch;
                    if (t[p].Is("..."))
                    {
                        aCatch = "catch";
                    }
                    else
                    {
                        var aType = Parser.ParseType(t, ref p)!;
                        string aName = p < aClose && t[p].IsIdent ? " " + t[p].Text : "";
                        aCatch = "catch (" + myDb.Map(aType) + aName + ")";
                    }

                    Line(aCatch);
                    j = Body(t, aClose + 1, theTo);
                }

                return j;
            }

            if (k.IsIdent && k.Text == "DeclareAndCast" && t[i + 1].Is("("))
            {
                // MoniTool's DeclareAndCast(Type, name, value): occ::handle<Type> name = occ::down_cast<Type>(value)
                int aClose = Parser.Match(t, i + 1);
                int p = i + 2;
                var aType = Parser.ParseType(t, ref p, IsTypeName) ?? throw new FormatException("DeclareAndCast's type");
                string aName = t[p + 1].Text;
                var e = ParseExpr(t, p + 3, aClose);
                string aMapped = myDb.Map(aType);
                string aValue = Emit(e);
                c.Locals[aName] = new CppType { Name = "occ::handle", Args = [aType] };
                Line(aMapped + " " + Db.Id(DeclareLocal(aName)) + " = " + aValue + " as " + aMapped + ";");
                return End(t, i, theTo) + 1;
            }

            if (k.IsIdent && k.Text is "OCC_CATCH_SIGNALS")
            {
                return i + 1 + (t[i + 1].Is(";") ? 1 : 0);
            }

            // a declaration or an expression
            int aStmtEnd = End(t, i, theTo);
            string aText = Simple(t, i, aStmtEnd);
            if (aText.Length > 0)
            {
                Line(aText + ";");
            }

            // a trailing comment on the same line
            return aStmtEnd + 1;
        }
        catch (FormatException ex)
        {
            int anEnd = i;
            try
            {
                anEnd = End(t, i, theTo);
            }
            catch (FormatException)
            {
                anEnd = theTo - 1;
            }

            string aSrc = string.Join(" ", t.GetRange(i, anEnd - i + 1).Where(x => !x.IsComment).Select(x => x.Text));
            Line(Problem("cannot translate (" + ex.Message + "): " + aSrc));
            return anEnd + 1;
        }
    }

    private int For(List<Token> t, int i, int theTo)
    {
        int aClose = Parser.Match(t, i + 1);
        // split "init; cond; step" at the top-level ';'
        var aSemis = new List<int>();
        int aDepth = 0;
        for (int j = i + 2; j < aClose; j++)
        {
            if (t[j].Is("(") || t[j].Is("["))
            {
                aDepth++;
            }
            else if (t[j].Is(")") || t[j].Is("]"))
            {
                aDepth--;
            }
            else if (aDepth == 0 && t[j].Is(";"))
            {
                aSemis.Add(j);
            }
        }

        string anInit = aSemis[0] > i + 2 ? Simple(t, i + 2, aSemis[0]) : "";
        string aCond = aSemis[1] > aSemis[0] + 1 ? Cond(ParseExpr(t, aSemis[0] + 1, aSemis[1])) : "";
        string aStep = aClose > aSemis[1] + 1 ? Emit(ParseExpr(t, aSemis[1] + 1, aClose), true) : "";
        Line("for (" + anInit + "; " + aCond + "; " + aStep + ")");
        return Body(t, aClose + 1, theTo);
        }

    private int SkipCommentsEmitting(List<Token> t, int j, int theTo)
    {
        // comments between "}" and "else" stay where they are only if no else follows
        int k = j;
        while (k < theTo && t[k].IsComment)
        {
        k++;
        }

        if (k < theTo && t[k].Is("else"))
        {
        for (int m = j; m < k; m++)
            {
                Comment(t[m]);
            }

            return k;
        }

        return j;
    }

    /// <summary>A controlled statement: a block, or a single statement indented.</summary>
    private int Body(List<Token> t, int i, int theTo)
    {
        while (i < theTo && t[i].IsComment)
        {
            Comment(t[i]);
            i++;
        }

        if (t[i].Is("{"))
        {
            return Statement(t, i, theTo);
        }

        myIndent++;
        int j = Statement(t, i, theTo);
        myIndent--;
        return j;
    }

    private Expr ParseExpr(List<Token> t, int theFrom, int theTo)
    {
        var p = new ExprParser(t, theFrom, theTo, IsTypeName);
        var e = p.ParseFull();
        foreach (string aComment in p.Comments)
        {
            Line(aComment);
        }

        return e;
    }

    /// <summary>A declaration or an expression statement, without its ';'.</summary>
    public string Simple(List<Token> t, int theFrom, int theTo)
    {
        int i = theFrom;
        while (i < theTo && t[i].IsComment)
        {
            Comment(t[i]);
            i++;
        }

        if (i >= theTo)
        {
            return "";
        }

        // declaration: [static] [const] Type name ...
        int j = i;
        bool isStatic = false;
        while (j < theTo && (t[j].Is("static") || t[j].Is("constexpr") || t[j].Is("register")))
        {
            isStatic |= t[j].Is("static");
            j++;
        }

        int aTypeEnd = j;
        var aType = Parser.ParseType(t, ref aTypeEnd, IsTypeName);
        if (aType != null && aTypeEnd < theTo && (t[aTypeEnd].IsIdent || t[aTypeEnd].Is("*") || t[aTypeEnd].Is("&"))
            && !(aType.Name.Contains("::", StringComparison.Ordinal) && !IsTypeName(aType.Name)))
        {
            if (isStatic)
            {
                c.Problems.Add("static local variable");
            }

            return Declaration(t, aType, aTypeEnd, theTo);
        }

        // (void)x;
        if (t[i].Is("(") && t[i + 1].Is("void") && t[i + 2].Is(")"))
        {
            return "_ = " + Emit(ParseExpr(t, i + 3, theTo), true);
        }

        var e = ParseExpr(t, i, theTo);
        string s = Emit(e, true);
        // expression statements C# doesn't allow (a bare comparison, a name)
        if (e is NameExpr or LitExpr || e is BinaryExpr { Op: "==" or "!=" or "<" or ">" })
        {
            s = "_ = " + s;
        }

        return s;
    }

    private string Declaration(List<Token> t, CppType theType, int i, int theTo)
    {
        var aParts = new List<string>();
        string? aCsType = null;
        var aResult = new StringBuilder();
        while (i < theTo)
        {
            var aDeclType = theType.Bare();
            aDeclType.IsConst = theType.IsConst;
            while (t[i].Is("*") || t[i].Is("&") || t[i].Is("const"))
            {
                if (t[i].Is("*"))
                {
                    aDeclType.Pointers++;
                }
                else if (t[i].Is("&"))
                {
                    aDeclType.IsRef = true;
                }

                i++;
            }

            string aName = t[i].Text;
            i++;
            string? anArraySize = null;
            if (i < theTo && t[i].Is("["))
            {
                int anEnd = Parser.Match(t, i);
                anArraySize = anEnd > i + 1 ? Emit(ParseExpr(t, i + 1, anEnd), true) : null;
                i = anEnd + 1;
                aDeclType = new CppType { Name = "std::array", Args = [theType.Bare()] };
            }

            string aMapped = myDb.Map(aDeclType);
            if (aDeclType.IsRef && !aDeclType.IsConst && myDb.IsRefKind(aDeclType.Bare()))
            {
                c.Problems.Add($"local reference {aName} (a copy in C#)");
            }

            // the initialiser: up to ',' at depth 0
            int aStop = i;
            int aDepth = 0;
            while (aStop < theTo)
            {
                if (t[aStop].Is("(") || t[aStop].Is("[") || t[aStop].Is("{"))
                {
                    aDepth++;
                }
                else if (t[aStop].Is(")") || t[aStop].Is("]") || t[aStop].Is("}"))
                {
                    aDepth--;
                }
                else if (aDepth == 0 && t[aStop].Is(","))
                {
                    // a ',' inside "(args)" is at depth 1; at depth 0 it separates declarators
                    break;
                }

                aStop++;
            }

            string anInit;
            if (i < aStop && t[i].Is("="))
            {
                var e = ParseExpr(t, i + 1, aStop);
                c.Locals[aName] = aDeclType.Bare();
                anInit = InitFrom(aDeclType, e);
            }
            else if (i < aStop && (t[i].Is("(") || t[i].Is("{")))
            {
                int anEnd = Parser.Match(t, i);
                var p = new ExprParser(t, i, anEnd + 1, IsTypeName);
                var aCall = t[i].Is("(") ? null : (Expr?)null;
                _ = aCall;
                var aArgs = new List<Expr>();
                if (anEnd > i + 1)
                {
                    var e = ParseExpr(t, i + 1, anEnd);
                    aArgs = e is BinaryExpr { Op: "," } ? SplitComma(e) : [e];
                }

                _ = p;
                c.Locals[aName] = aDeclType.Bare();
                anInit = ConstructFrom(aDeclType, aArgs);
            }
            else
            {
                c.Locals[aName] = aDeclType.Bare();
                anInit = anArraySize != null ? "new " + myDb.Map(theType.Bare()) + "[" + anArraySize + "]" : DefaultValue(aDeclType);
            }

            if (anArraySize != null && anInit.StartsWith("{", StringComparison.Ordinal))
            {
                // C array with initialiser: "double x[3] = {..}" -> "double[] x = { .. }"
            }

            if (aCsType == null)
            {
                aCsType = aMapped;
                aResult.Append(aMapped).Append(' ');
            }
            else if (aCsType != aMapped)
            {
                // a declarator of another C# type: a separate declaration
                aResult.Append("; ").Append(aMapped).Append(' ');
                aCsType = aMapped;
            }
            else
            {
                aResult.Append(", ");
            }

            aResult.Append(Db.Id(DeclareLocal(aName))).Append(" = ").Append(anInit);
            i = aStop;
            if (i < theTo && t[i].Is(","))
            {
                i++;
            }
        }

        _ = aParts;
        return aResult.ToString();
    }

    private static List<Expr> SplitComma(Expr e)
    {
        var l = new List<Expr>();
        while (e is BinaryExpr { Op: "," } b)
        {
            l.Insert(0, b.R);
            e = b.L;
        }

        l.Insert(0, e);
        return l;
    }

    /// <summary>The value of "T x = e".</summary>
    public string InitFrom(CppType t, Expr e)
    {
        if (e is InitListExpr il)
        {
            if (t.Name == "std::array")
            {
                return Emit(il, true);
            }

            if (t.IsHandle && il.Items.Count == 0)
            {
                return "null";
            }

            return ConstructFrom(t, il.Items);
        }

        string s = t.Name == "bool" && t.Pointers == 0 ? Cond(e) : Emit(e, true);
        if (IsSelectish(t) && e is not (TypeCallExpr or NewExpr))
        {
            return "new " + myDb.Map(t.Bare()) + "(" + s + ")";
        }

        return s;
    }

    /// <summary>The value of "T x(args)" or "T x{args}".</summary>
    private string ConstructFrom(CppType t, List<Expr> theArgs)
    {
        if (t.IsHandle)
        {
            return theArgs.Count == 0 ? "null" : Emit(theArgs[0], true);
        }

        string aMapped = myDb.Map(t);
        if (aMapped == "string")
        {
            return theArgs.Count == 0 ? "\"\"" : Emit(theArgs[0], true);
        }

        if (Db.BuiltinMap.ContainsKey(t.Name) || myDb.IsEnum(t.Name) || t.Pointers > 0)
        {
            return theArgs.Count == 0 ? DefaultValue(t) : Emit(theArgs[0], true);
        }

        if (theArgs.Count == 1 && IsSelectish(t) && InferType(theArgs[0]) is { } a && a.Name == t.Name)
        {
            return "new " + aMapped + "(" + Emit(theArgs[0], true) + ")";
        }

        return "new " + aMapped + "(" + string.Join(", ", theArgs.Select(a => Emit(a, true))) + ")";
    }

    /// <summary>The C# value of a C++ variable declared without an initialiser.</summary>
    public string DefaultValue(CppType t)
    {
        if (t.IsHandle || t.Pointers > 0)
        {
            return t.IsCString ? "null" : "null";
        }

        string aMapped = myDb.Map(t);
        switch (aMapped)
        {
            case "int":
            case "long":
            case "short":
            case "uint":
            case "byte":
                return "0";
            case "double":
            case "float":
                return "0.0";
            case "bool":
                return "false";
            case "char":
                return "'\\0'";
            case "string":
                return t.Name is "TCollection_AsciiString" or "std::string" ? "\"\"" : "null";
        }

        if (myDb.IsEnum(t.Name) || t.Name is "StepData_Logical" or "Interface_ParamType" || t.Name.StartsWith("gp_", StringComparison.Ordinal))
        {
            return "default";
        }

        if (c.TemplateParams.Contains(t.Name))
        {
            return "default";
        }

        if (t.Name == "std::array")
        {
            return "new " + myDb.Map(t.Args[0]) + "[" + (t.ValueArgs.Count > 0 ? t.ValueArgs[0] : "0") + "]";
        }

        return "new " + aMapped + "()";
    }
}

