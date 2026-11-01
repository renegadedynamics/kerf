using System.Reflection;

namespace OcctNetGen;

/// <summary>How a Build123d.Occt type is backed by OCCT.NET.</summary>
internal enum Kind
{
    /// <summary>An enum: Build123d.Occt's own file is compiled in, and values convert by cast.</summary>
    Enum,
    /// <summary>A namespace of free functions (TopoDS, gp…): a static class forwarding to OCCT.NET's.</summary>
    Static,
    /// <summary>A class around an OCCT.NET struct (gp_Pnt, Bnd_Box, TopoDS_Shape…), held in a field <c>V</c>.</summary>
    StructRoot,
    /// <summary>A subclass of a struct-backed class (TopoDS_Edge…): stored as its base's struct, narrowed on use.</summary>
    StructSub,
    /// <summary>A class around an OCCT.NET class, held by reference.</summary>
    Class,
    /// <summary>A class with only static functions, around an OCCT.NET static class (BRep_Tool, BRepGProp…).</summary>
    StaticOnly,
    /// <summary>Not in OCCT.NET: every member throws "not yet ported".</summary>
    Missing,
}

internal sealed class TypeInfo(Type native, Type? occt, Kind kind)
{
    public Type Native { get; } = native;
    public Type? Occt { get; } = occt;
    public Kind Kind { get; } = kind;
    public string Name => Native.Name;
    public bool IsCollection { get; init; }
}

/// <summary>An argument or result conversion: statements before and after the call, and the expression.</summary>
internal sealed record Conversion(string Expr, string? Pre = null, string? Post = null, int Score = 2);

/// <summary>A native member matched to an OCCT.NET member, with the conversions of its call.</summary>
/// <remarks>
/// A <paramref name="Twin"/> match is OCCT.NET's functional twin of an in-place form on a struct (<c>SetX</c> as
/// <c>WithX</c>, <c>Normalize</c> as <c>Normalized</c>): the wrapper's struct is replaced by the twin's result.
/// </remarks>
internal sealed record Match(MemberInfo Target, IReadOnlyList<Conversion> Args, string? Result, int Score, int Extra, bool Twin = false);

internal sealed class Model
{
    // Build123d.Occt's runtime, written by hand for the backend (src/Build123d.OcctNet/Runtime)
    private static readonly HashSet<string> RuntimeTypes =
        ["OcctObject", "OcctTransient", "OcctException", "OcctContext", "Extras", "MeshBuffers", "Occt", "DoingScope"];

    // OCCT's collection typedefs, which OCCT.NET writes as the NCollection templates
    private static readonly Dictionary<string, (string Template, string[] Args)> Collections = new()
    {
        ["TopTools_ListOfShape"] = ("NCollection_List`1", ["TopoDS_Shape"]),
        ["TopTools_SequenceOfShape"] = ("NCollection_Sequence`1", ["TopoDS_Shape"]),
        ["TopTools_HSequenceOfShape"] = ("NCollection_HSequence`1", ["TopoDS_Shape"]),
        ["TopTools_MapOfShape"] = ("NCollection_Map`1", ["TopoDS_Shape"]),
        ["TopTools_IndexedMapOfShape"] = ("NCollection_IndexedMap`1", ["TopoDS_Shape"]),
        ["TopTools_DataMapOfShapeShape"] = ("NCollection_DataMap`2", ["TopoDS_Shape", "TopoDS_Shape"]),
        ["TopTools_IndexedDataMapOfShapeListOfShape"] = ("NCollection_IndexedDataMap`2", ["TopoDS_Shape", "TopTools_ListOfShape"]),
        ["TColgp_Array1OfPnt"] = ("NCollection_Array1`1", ["gp_Pnt"]),
        ["TColgp_Array1OfPnt2d"] = ("NCollection_Array1`1", ["gp_Pnt2d"]),
        ["TColgp_Array1OfVec"] = ("NCollection_Array1`1", ["gp_Vec"]),
        ["TColgp_Array2OfPnt"] = ("NCollection_Array2`1", ["gp_Pnt"]),
        ["TColgp_HArray1OfPnt"] = ("NCollection_HArray1`1", ["gp_Pnt"]),
        ["TColgp_HArray2OfPnt"] = ("NCollection_HArray2`1", ["gp_Pnt"]),
        ["TColStd_Array1OfInteger"] = ("NCollection_Array1`1", ["System.Int32"]),
        ["TColStd_Array1OfReal"] = ("NCollection_Array1`1", ["System.Double"]),
        ["TColStd_Array2OfReal"] = ("NCollection_Array2`1", ["System.Double"]),
        ["TColStd_HArray1OfBoolean"] = ("NCollection_HArray1`1", ["System.Boolean"]),
        ["TColStd_HArray1OfReal"] = ("NCollection_HArray1`1", ["System.Double"]),
        ["TColStd_HArray2OfReal"] = ("NCollection_HArray2`1", ["System.Double"]),
    };

    private readonly Dictionary<string, Type> _occt;
    private readonly Handwritten _hand;

    public Dictionary<string, TypeInfo> Types { get; } = [];
    public List<string> Report { get; } = [];
    public int Forwarded { get; set; }
    public Handwritten Hand => _hand;
    public Type OcctShape { get; }

    public Model(Assembly native, IEnumerable<Assembly> occt, Handwritten hand)
    {
        _hand = hand;
        _occt = occt.SelectMany(a => a.GetExportedTypes()).Where(t => t.Namespace == "Occt" && !t.IsNested)
                    .ToDictionary(t => t.Name);
        OcctShape = _occt["TopoDS_Shape"];

        var natives = native.GetExportedTypes().Where(t => !t.IsNested && !RuntimeTypes.Contains(t.Name)).ToList();
        // Collections need their element types' OCCT counterparts, so plain types first
        foreach (var t in natives.Where(t => !Collections.ContainsKey(t.Name)))
            Types[t.Name] = Classify(t);
        // and collections of collections after the collections they hold
        foreach (var t in natives.Where(t => Collections.ContainsKey(t.Name))
                                 .OrderBy(t => Collections[t.Name].Args.Any(Collections.ContainsKey)))
        {
            var (template, args) = Collections[t.Name];
            var elements = args.Select(a => a.StartsWith("System.") ? typeof(int).Assembly.GetType(a)!
                                          : Types.TryGetValue(a, out var e) ? e.Occt! : _occt[a]).ToArray();
            Types[t.Name] = new TypeInfo(t, _occt[template].MakeGenericType(elements), Kind.Class) { IsCollection = true };
        }
    }

    private TypeInfo Classify(Type t)
    {
        if (t.IsEnum)
            return new TypeInfo(t, _occt.GetValueOrDefault(t.Name), Kind.Enum);
        if (t.IsAbstract && t.IsSealed)
            return new TypeInfo(t, _occt.GetValueOrDefault(t.Name), _occt.ContainsKey(t.Name) ? Kind.Static : Kind.Missing);
        // OCCT.NET keeps mutating builders internal; those are hand-written (BRep_Builder)
        if (!_occt.TryGetValue(t.Name, out var o) || o.IsNotPublic)
            return new TypeInfo(t, null, Kind.Missing);
        if (o.IsAbstract && o.IsSealed)
            return new TypeInfo(t, o, Kind.StaticOnly);
        if (o.IsValueType)
            return new TypeInfo(t, o, t.BaseType!.Name is "OcctObject" or "OcctTransient" ? Kind.StructRoot : Kind.StructSub);
        return new TypeInfo(t, o, Kind.Class);
    }

    public TypeInfo? Info(Type nativeType) =>
        nativeType.Namespace == "Build123d.Occt" && Types.TryGetValue(nativeType.Name, out var i) ? i : null;

    /// <summary>The C# type the wrapper keeps its OCCT.NET object in.</summary>
    public string Storage(TypeInfo info) => info.Kind == Kind.StructSub ? Cs.Occt(OcctShape) : Cs.Occt(info.Occt!);

    /// <summary>The OCCT.NET object of <paramref name="x"/>, a wrapper of <paramref name="info"/>'s type.</summary>
    public static string Receiver(TypeInfo info, string x) => info.Kind switch
    {
        Kind.Class => $"{x}.Impl",
        Kind.StructRoot => $"{x}.V",
        Kind.StructSub => $"{x}.S",
        _ => throw new InvalidOperationException(info.Name),
    };

    // ---- Matching

    private static bool IsNullable(ParameterInfo p) =>
        !p.ParameterType.IsValueType && new NullabilityInfoContext().Create(p).ReadState == NullabilityState.Nullable;

    private static readonly Dictionary<Type, Type[]> Widening = new()
    {
        [typeof(int)] = [typeof(long), typeof(double), typeof(float)],
        [typeof(float)] = [typeof(double)],
        [typeof(uint)] = [typeof(ulong), typeof(long)],
    };

    private static bool Same(Type a, Type b) => a.FullName == b.FullName && a.FullName is not null;

    private static bool Assignable(Type to, Type from) => to.IsAssignableFrom(from) || HasImplicit(from, to);

    private static bool HasImplicit(Type from, Type to) =>
        from.GetMethods(BindingFlags.Public | BindingFlags.Static).Concat(to.GetMethods(BindingFlags.Public | BindingFlags.Static))
            .Any(m => m.Name == "op_Implicit" && m.ReturnType == to && m.GetParameters() is [var p] && p.ParameterType == from);

    /// <summary>The argument for OCCT.NET parameter <paramref name="o"/> from native parameter <paramref name="n"/>.</summary>
    private Conversion? Arg(ParameterInfo n, ParameterInfo o, int i)
    {
        var name = Cs.Id(n.Name!);
        var nt = n.ParameterType;
        var ot = o.ParameterType;
        bool oIn = ot.IsByRef && (o.IsIn || o.GetRequiredCustomModifiers().Any(m => m.Name == "InAttribute")
                                  || o.GetCustomAttributes().Any(a => a.GetType().Name == "IsReadOnlyAttribute"));
        bool oOut = ot.IsByRef && o.IsOut && !oIn;
        bool oRef = ot.IsByRef && !oIn && !oOut;
        var oe = ot.IsByRef ? ot.GetElementType()! : ot;

        if (nt.IsByRef)
        {
            var ne = nt.GetElementType()!;
            if (!(oOut || oRef) || !Same(ne, oe))
                return null;
            return new Conversion($"{(oOut ? "out" : "ref")} {name}");
        }
        if (nt.IsPrimitive || nt == typeof(string))
        {
            if (oOut || oRef)
                return null;
            if (Same(nt, oe))
                return new Conversion(name);
            if (Widening.TryGetValue(nt, out var wider) && wider.Any(w => Same(w, oe)))
                return new Conversion(name, Score: 1);
            if (nt == typeof(string) && HasImplicit(typeof(string), oe))
                return new Conversion(name, Score: 1);
            return null;
        }
        var info = Info(nt);
        if (info is null)
            return null;
        if (info.Kind == Kind.Enum)
        {
            if (oOut || oRef || !oe.IsEnum || oe.Name != nt.Name)
                return null;
            return new Conversion($"({Cs.Occt(oe)}){name}");
        }
        if (info.Occt is null || info.Kind is Kind.Static or Kind.StaticOnly or Kind.Missing)
            return null;
        var nullable = IsNullable(n);
        if (oOut || oRef)
        {
            // A wrapper the native call fills in: a struct here, through a temporary
            if (info.Kind is not (Kind.StructRoot or Kind.StructSub) || !(Same(oe, info.Occt) || (info.Kind == Kind.StructSub && Same(oe, OcctShape))))
                return null;
            var t = $"__t{i}";
            var source = info.Kind == Kind.StructSub && Same(oe, OcctShape) ? $"{name}.V" : Receiver(info, name);
            var pre = oOut ? $"{Cs.Occt(oe)} {t};" : $"var {t} = {source};";
            return new Conversion($"{(oOut ? "out" : "ref")} {t}", pre, $"{name}.V = {t};");
        }
        switch (info.Kind)
        {
            case Kind.Class:
                if (!oe.IsAssignableFrom(info.Occt))
                    return null;
                return new Conversion(nullable ? $"{name}?.Impl!" : $"{name}.Impl", Score: Same(oe, info.Occt) ? 2 : 1);
            case Kind.StructRoot:
                if (Same(oe, info.Occt))
                    return new Conversion($"{name}.V");
                return Assignable(oe, info.Occt) ? new Conversion($"{name}.V", Score: 1) : null;
            case Kind.StructSub:
                if (Same(oe, info.Occt))
                    return new Conversion($"{name}.S");
                return Same(oe, OcctShape) ? new Conversion($"{name}.V", Score: 1) : null;
        }
        return null;
    }

    /// <summary>The native result from OCCT.NET's, held in <c>__r</c>; null if they don't convert.</summary>
    private string? Result(Type nt, ParameterInfo nativeReturn, Type ot)
    {
        if (ot.IsByRef)
            ot = ot.GetElementType()!;
        if (nt == typeof(void))
            return "";
        if (nt.IsPrimitive || nt == typeof(string))
        {
            if (Same(nt, ot))
                return "__r";
            if (Widening.TryGetValue(ot, out var wider) && wider.Any(w => Same(w, nt)))
                return "__r";
            return null;
        }
        var info = Info(nt);
        if (info is null)
            return null;
        if (info.Kind == Kind.Enum)
            return ot.IsEnum && ot.Name == nt.Name ? $"({Cs.Native(nt)})__r" : null;
        if (info.Occt is null || info.Kind is Kind.Static or Kind.StaticOnly or Kind.Missing)
            return null;
        var nullable = new NullabilityInfoContext().Create(nativeReturn).ReadState == NullabilityState.Nullable;
        switch (info.Kind)
        {
            case Kind.Class when info.IsCollection:
                // OCCT's const references to its collections are copied, as the native binding does
                return info.Occt.IsAssignableFrom(ot) || ot.GetInterfaces().Any(i => i.IsGenericType && i.GetGenericTypeDefinition() == typeof(IEnumerable<>))
                    ? (nullable && !ot.IsValueType ? $"__r is null ? null : {info.Name}.__Wrap(__r)" : $"{info.Name}.__Wrap(__r)") : null;
            case Kind.Class:
                if (!info.Occt.IsAssignableFrom(ot) || ot.IsValueType)
                    return null;
                return nullable ? $"__r is null ? null : new {info.Name}(__r)" : $"new {info.Name}(__r)";
            case Kind.StructRoot:
                return Same(ot, info.Occt) ? $"new {info.Name}(__r)" : null;
            case Kind.StructSub:
                return Same(ot, info.Occt) || Same(ot, OcctShape) ? $"new {info.Name}(__r)" : null;
        }
        return null;
    }

    /// <summary>The best OCCT.NET counterpart of a native method or constructor, or null with the reason.</summary>
    public Match? Find(TypeInfo owner, MethodBase m, out string why)
    {
        var match = FindNamed(owner, m, m.Name, out why);
        // OCCT 8 renamed GetType to GetType_ in the bindings, where it would hide object.GetType; OCCT.NET keeps GetType
        if (match is null && m.Name.EndsWith('_'))
            match = FindNamed(owner, m, m.Name.TrimEnd('_'), out _);
        // OCCT.NET makes a struct's in-place forms internal, with public functional twins that return the result
        if (match is null && owner.Kind == Kind.StructRoot && m is MethodInfo { IsStatic: false, ReturnType: var rt } && rt == typeof(void))
            foreach (var twin in Twins(m.Name))
                if (FindTwin(owner, (MethodInfo)m, twin) is { } t)
                    return t;
        return match;
    }

    private static IEnumerable<string> Twins(string name)
    {
        if (name.StartsWith("Set", StringComparison.Ordinal) && name.Length > 3)
            yield return "With" + name[3..];
        yield return name + "ed";
        yield return name + "d";
        if (name.EndsWith('y'))
            yield return name[..^1] + "ied";
    }

    private Match? FindTwin(TypeInfo owner, MethodInfo m, string twin)
    {
        var nps = m.GetParameters();
        Match? best = null;
        foreach (var c in owner.Occt!.GetMethods(BindingFlags.Public | BindingFlags.Instance).Where(c => c.Name == twin && Same(c.ReturnType, owner.Occt)))
        {
            var ops = c.GetParameters();
            if (ops.Length < nps.Length || ops.Skip(nps.Length).Any(p => !p.HasDefaultValue && !p.IsOptional))
                continue;
            var convs = new List<Conversion>();
            for (int i = 0; i < nps.Length && convs.Count == i; i++)
                if (Arg(nps[i], ops[i], i) is { } conv)
                    convs.Add(conv);
            if (convs.Count != nps.Length || convs.Any(a => a.Pre is not null))
                continue;
            var score = convs.Sum(a => a.Score) * 4 - (ops.Length - nps.Length);
            if (best is null || score > best.Score)
                best = new Match(c, convs, "", score, ops.Length - nps.Length, Twin: true);
        }
        return best;
    }

    private Match? FindNamed(TypeInfo owner, MethodBase m, string name, out string why)
    {
        why = "";
        if (owner.Occt is null)
        {
            why = "class not in OCCT.NET";
            return null;
        }
        var nps = m.GetParameters();
        var candidates = new List<MethodBase>();
        var flags = BindingFlags.Public | BindingFlags.FlattenHierarchy | (m.IsStatic || m is ConstructorInfo ? BindingFlags.Static : BindingFlags.Instance);
        var target = owner.Occt;
        if (m is ConstructorInfo)
        {
            if (target.IsAbstract)
            {
                why = "abstract in OCCT.NET";
                return null;
            }
            candidates.AddRange(target.GetConstructors(BindingFlags.Public | BindingFlags.Instance));
        }
        else
        {
            candidates.AddRange(AllMethods(target, flags).Where(c => c.Name == name && !c.IsGenericMethodDefinition));
        }

        Match? best = null;
        foreach (var c in candidates)
        {
            var ops = c.GetParameters();
            if (ops.Length < nps.Length || ops.Skip(nps.Length).Any(p => !p.HasDefaultValue && !p.IsOptional))
                continue;
            var convs = new List<Conversion>();
            for (int i = 0; i < nps.Length && convs.Count == i; i++)
                if (Arg(nps[i], ops[i], i) is { } conv)
                    convs.Add(conv);
            if (convs.Count != nps.Length)
                continue;
            string? result = "";
            if (m is MethodInfo nm && c is MethodInfo cm)
            {
                result = Result(nm.ReturnType, nm.ReturnParameter, cm.ReturnType);
                if (result is null)
                    continue;
            }
            var score = convs.Sum(a => a.Score) * 4 - (ops.Length - nps.Length);
            if (best is null || score > best.Score)
                best = new Match(c, convs, result, score, ops.Length - nps.Length);
        }
        // A getter for a method without arguments (OCCT.NET has a few properties where OCCT has functions)
        if (best is null && m is MethodInfo mi && nps.Length == 0 && mi.ReturnType != typeof(void)
            && target.GetProperty(name, flags) is { GetMethod.IsPublic: true } prop
            && Result(mi.ReturnType, mi.ReturnParameter, prop.PropertyType) is { } r)
            best = new Match(prop, [], r, 0, 0);
        if (best is null)
            why = candidates.Count == 0 ? "no public counterpart in OCCT.NET" : "signatures differ in OCCT.NET";
        return best;
    }

    private static IEnumerable<MethodInfo> AllMethods(Type t, BindingFlags flags)
    {
        // Interfaces' members for an interface; a class's own and inherited ones otherwise
        if (t.IsInterface)
            return t.GetMethods(flags).Concat(t.GetInterfaces().SelectMany(i => i.GetMethods(flags)));
        return t.GetMethods(flags);
    }
}
