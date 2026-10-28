using ClangSharp;
using ClangSharp.Interop;

namespace OcctGen;

sealed record Param(string Name, Mapped Type);

/// <summary>One bound function: a constructor, a method or a static function, at one arity.</summary>
sealed class Member
{
    public required ClassInfo Owner { get; init; }
    public required string DeclaringClass { get; init; }   // C++ class that declares it
    public required string Name { get; init; }
    public required bool IsCtor { get; init; }
    public required bool IsStatic { get; init; }
    public required bool IsConst { get; init; }
    public required List<Param> Params { get; init; }
    public required Mapped Return { get; init; }
    public string Export { get; set; } = "";
    public bool HidesBase { get; set; }
    /// <summary>A default constructor the class doesn't declare: emitted only if the type is default-constructible.</summary>
    public bool IsImplicitDefault { get; init; }

    public string Signature => $"{Name}({string.Join(",", Params.Select(p => p.Type.Key))})";

    /// <summary>The signature as C# sees it, where a handle and a reference to the same transient are one type.</summary>
    public string CsSignature =>
        $"{Name}({string.Join(",", Params.Select(p => p.Type.Kind == Kind.TransRef ? p.Type.Name : p.Type.Key))})";
}

/// <summary>A bound class, or a namespace of free functions (bound as a static class).</summary>
sealed class ClassInfo(string name, CXXRecordDecl? decl, List<FunctionDecl> nsFunctions)
{
    /// <summary>The class definition; null for a namespace.</summary>
    public CXXRecordDecl? Decl { get; } = decl;
    public List<FunctionDecl> NamespaceFunctions { get; } = nsFunctions;
    public string Name { get; } = name;
    /// <summary>Set for a collection bound under an alias: its C++ spelling.</summary>
    public CollectionInfo? Collection { get; init; }
    public string CppName => Collection?.Spelling ?? Name;
    public bool IsTransient { get; set; }
    public bool IsStatic { get; set; }
    public bool IsAbstract => Decl?.IsAbstract ?? false;
    public bool CanDelete { get; set; }
    /// <summary>A concrete value class whose copy constructor isn't explicitly deleted or hidden: gets a C# copy constructor.</summary>
    public bool CanCopy { get; set; }
    /// <summary>Nearest whitelisted ancestor along the primary base chain: the C# base class.</summary>
    public ClassInfo? Base { get; set; }
    public List<Member> Members { get; } = [];
    public HashSet<string> Referenced { get; } = [];

    /// <summary>The C# base chain, nearest first.</summary>
    public IEnumerable<ClassInfo> Ancestors()
    {
        for (var b = Base; b != null; b = b.Base)
            yield return b;
    }
}

sealed class Model
{
    public Dictionary<string, ClassInfo> Classes { get; } = [];
    /// <summary>Bound collections, by alias and by C++ spelling.</summary>
    public Dictionary<string, ClassInfo> Collections { get; } = [];
    /// <summary>Classes bound under an alias of a template instantiation, by the instantiation's spelling.</summary>
    public Dictionary<string, ClassInfo> Aliases { get; } = [];

    /// <summary>
    /// Whitelisted names that alias a template instantiation (BRepLProp_SLProps), with the instantiation's
    /// spelling and its definition if clang has instantiated it.
    /// </summary>
    public static Dictionary<string, (string Spelling, CXXRecordDecl? Def)> TemplateAliases(TranslationUnit tu, Config config)
    {
        var wanted = config.Classes.ToHashSet();
        var found = new Dictionary<string, (string, CXXRecordDecl?)>();
        foreach (var d in tu.TranslationUnitDecl.Decls)
            if (d is TypedefNameDecl td && wanted.Contains(td.Name)
                && td.UnderlyingType.CanonicalType is RecordType { Decl: ClassTemplateSpecializationDecl spec } rt)
                found.TryAdd(td.Name, (CollectionInfo.Normalize(rt.AsString), spec.Definition as CXXRecordDecl));
        return found;
    }
    public SortedDictionary<string, EnumDecl> Enums { get; } = [];
    public List<string> Skipped { get; } = [];

    static readonly HashSet<string> IgnoredMethods =
        ["DumpJson", "Dump", "get_type_name", "get_type_descriptor", "DynamicType", "This", "Delete"];

    public static Model Build(TranslationUnit tu, Config config)
    {
        var model = new Model();
        var defs = new Dictionary<string, CXXRecordDecl>();
        var namespaces = new Dictionary<string, List<FunctionDecl>>();
        foreach (var d in tu.TranslationUnitDecl.Decls)
        {
            if (d is CXXRecordDecl { IsThisDeclarationADefinition: true } r && r.Name.Length > 0)
                defs.TryAdd(r.Name, r);
            else if (d is NamespaceDecl ns && ns.Name.Length > 0)
            {
                // A namespace may be reopened; gather the free functions of every part
                if (!namespaces.TryGetValue(ns.Name, out var fns))
                    namespaces[ns.Name] = fns = [];
                fns.AddRange(ns.Decls.OfType<FunctionDecl>().Where(f => f is not CXXMethodDecl));
            }
        }

        var aliases = TemplateAliases(tu, config);
        foreach (var (alias, (_, def)) in aliases)
            if (def is not null)
                defs.TryAdd(alias, def);

        foreach (var name in config.Classes)
        {
            if (name == "Standard_Transient")
                throw new InvalidOperationException("Standard_Transient is bound implicitly (OcctTransient); don't list it");
            if (defs.TryGetValue(name, out var decl))
                model.Classes[name] = new ClassInfo(name, decl, []);
            else if (namespaces.TryGetValue(name, out var fns))
                model.Classes[name] = new ClassInfo(name, null, fns) { IsStatic = true };
            else
                throw new InvalidOperationException($"class or namespace {name} not found in the parsed headers");
        }

        foreach (var cls in model.Classes.Values)
        {
            if (cls.Decl is null)
                continue;
            cls.IsTransient = AllBases(cls.Decl).Any(b => b.Name == "Standard_Transient")
                              || cls.Name == "Standard_Transient";
            cls.IsStatic = !cls.IsTransient
                           && !cls.Decl.Ctors.Any(c => (c.IsUserProvided || c.IsExplicitlyDefaulted) && IsPublic(c))
                           && cls.Decl.Methods.Where(m => IsPublic(m) && m.Kind == CX_DeclKind.CX_DeclKind_CXXMethod)
                                  .All(m => m.IsStatic);
            cls.CanDelete = !cls.IsStatic && (cls.Decl.Destructor is null || IsPublic(cls.Decl.Destructor));
            // Implicit copy constructors aren't reliably declared in the parsed AST, so only an explicit
            // deleted or non-public one rules copying out; the shim checks the rest at compile time
            cls.CanCopy = cls.CanDelete && !cls.IsTransient && !cls.IsAbstract
                          && !cls.Decl.Ctors.Any(c => c.IsCopyConstructor && c.IsUserProvided && !IsPublic(c))
                          && !cls.Decl.Ctors.Any(c => c.IsCopyConstructor && c.IsDeleted);
            cls.Base = PrimaryWhitelistedBase(cls.Decl, model.Classes);
        }

        foreach (var (alias, (spelling, _)) in aliases)
            if (model.Classes.TryGetValue(alias, out var ac))
                model.Aliases[spelling] = ac;

        foreach (var c in config.Collections)
        {
            var info = new ClassInfo(c.Alias, null, []) { Collection = c, IsTransient = c.IsTransient };
            model.Collections[c.Alias] = info;
            model.Collections[c.Spelling] = info;
        }
        foreach (var c in config.Collections)
        {
            c.Item = model.ItemType(c, c.Args[0]);
            if (c.Kind is "datamap" or "indexeddatamap")
                c.Value = model.ItemType(c, c.Args[1]);
        }

        var mapper = new TypeMapper(n => model.Classes.GetValueOrDefault(n) ?? model.Collections.GetValueOrDefault(n)
                                         ?? model.Aliases.GetValueOrDefault(n),
                                    e => model.Enums.TryAdd(e.Name, e));
        // Base classes first, so each class knows which signatures C# already inherits
        foreach (var cls in model.Classes.Values.OrderBy(c => c.Ancestors().Count()))
            model.CollectMembers(cls, mapper, config);
        return model;
    }

    Mapped ItemType(CollectionInfo c, string arg)
    {
        switch (arg)
        {
            case "double": return new Mapped(Kind.Prim, "double");
            case "int": return new Mapped(Kind.Prim, "int32_t");
            case "bool": return new Mapped(Kind.Bool, "bool");
        }
        var cls = Classes.GetValueOrDefault(arg) ?? Collections.GetValueOrDefault(CollectionInfo.Normalize(arg));
        if (cls is { IsTransient: false, IsStatic: false })
            return new Mapped(Kind.Value, cls.Name, cls.CppName);
        throw new InvalidOperationException($"collection {c.Alias}: item type {arg} isn't a bound value class");
    }

    /// <summary>The headers that declare a bound class or collection.</summary>
    public IEnumerable<string> HeadersOf(string name) =>
        Collections.TryGetValue(name, out var c)
            ? c.Collection!.Headers().Select(h => h + ".hxx")
                .Concat(c.Collection.Args.SelectMany(a => Collections.TryGetValue(CollectionInfo.Normalize(a), out var inner) ? HeadersOf(inner.Name) : []))
            : [name + ".hxx"];

    void CollectMembers(ClassInfo cls, TypeMapper mapper, Config config)
    {
        if (cls.Decl is null)
        {
            var nsSeen = new HashSet<string>();
            foreach (var fn in cls.NamespaceFunctions)
                if (!fn.IsDeleted && !fn.IsVariadic && !fn.Name.StartsWith("operator"))
                    AddVariants(cls, cls.Name, fn, isCtor: false, mapper, config, nsSeen, []);
            return;
        }

        // Signatures C# already inherits from the base chain
        var inherited = cls.Ancestors().SelectMany(a => a.Members).Where(m => !m.IsCtor)
                           .Select(m => m.Signature).ToHashSet();
        var covered = cls.Base?.Decl is { } baseDecl ? AllBases(baseDecl).Append(baseDecl).ToHashSet() : [];
        var seen = new HashSet<string>();

        var trace = Environment.GetEnvironmentVariable("OCCTGEN_TRACE") == cls.Name;
        if (trace)
            Console.WriteLine($"{cls.Name}: abstract={cls.IsAbstract} static={cls.IsStatic} transient={cls.IsTransient} "
                              + $"base={cls.Base?.Name} ctors={string.Join("; ", cls.Decl.Ctors.Select(c => $"{c.Name}({c.Parameters.Count}) public={IsPublic(c)} userProvided={c.IsUserProvided} copy={c.IsCopyConstructor}"))}");

        if (!cls.IsAbstract && !cls.IsStatic)
        {
            foreach (var ctor in cls.Decl.Ctors)
            {
                if (!IsPublic(ctor) || !(ctor.IsUserProvided || ctor.IsExplicitlyDefaulted) || ctor.IsDeleted || ctor.IsCopyConstructor || ctor.IsMoveConstructor)
                    continue;
                AddVariants(cls, cls.Name, ctor, isCtor: true, mapper, config, seen, inherited);
            }
            // A class that declares no constructor of its own (BRep_Builder) has the implicit default one
            if (cls.CanDelete && !cls.Decl.Ctors.Any(c => c.IsUserProvided || c.IsExplicitlyDefaulted || c.IsDeleted))
            {
                var member = new Member
                {
                    Owner = cls, DeclaringClass = cls.Name, Name = "new", IsCtor = true, IsStatic = false, IsConst = false,
                    Params = [], Return = Mapped.Void, IsImplicitDefault = true,
                };
                if (seen.Add(member.Signature))
                {
                    member.Export = "b3d_" + cls.Name + "_new";
                    cls.Members.Add(member);
                }
            }
        }

        // Own methods first, then those of ancestors outside the C# base chain (flattened)
        var sources = new List<CXXRecordDecl> { cls.Decl };
        sources.AddRange(AllBases(cls.Decl).Where(b => !covered.Contains(b)));
        foreach (var src in sources)
            foreach (var m in src.Methods)
            {
                if (!IsPublic(m) || m.IsDeleted || m.IsVariadic
                    || m.Kind != CX_DeclKind.CX_DeclKind_CXXMethod || m.Name.StartsWith("operator")
                    || IgnoredMethods.Contains(m.Name))
                    continue;
                if (cls.IsStatic && !m.IsStatic)
                    continue;
                AddVariants(cls, src == cls.Decl ? cls.Name : src.Name, m, isCtor: false, mapper, config, seen, inherited);
            }

        // Methods of a non-public base that a class makes public with "using Base::Method;"
        // (BRepAlgoAPI_Algo re-exports SetRunParallel and friends from BOPAlgo_Options)
        foreach (var src in sources)
            foreach (var m in src.Decls.OfType<UsingShadowDecl>().Where(IsPublic).Select(s => s.UnderlyingDecl).OfType<CXXMethodDecl>())
                {
                    if (m.IsDeleted || m.IsVariadic || m.Kind != CX_DeclKind.CX_DeclKind_CXXMethod
                        || m.Name.StartsWith("operator") || IgnoredMethods.Contains(m.Name)
                        || (cls.IsStatic && !m.IsStatic))
                        continue;
                    AddVariants(cls, src == cls.Decl ? cls.Name : src.Name, m, isCtor: false, mapper, config, seen, inherited);
                }
    }

    void AddVariants(ClassInfo cls, string src, FunctionDecl fn, bool isCtor, TypeMapper mapper,
                     Config config, HashSet<string> seen, HashSet<string> inherited)
    {
        var name = isCtor ? "new" : fn.Name;
        var where = $"{cls.Name}::{(isCtor ? cls.Name : fn.Name)}";
        if (config.IsExcluded(cls.Name, isCtor ? cls.Name : fn.Name))
            return;
        if (fn.HasAttrs && fn.Attrs.Any(a => a.Kind == CX_AttrKind.CX_AttrKind_Deprecated))
        {
            Skipped.Add($"{where}: deprecated");
            return;
        }

        Mapped ret = Mapped.Void;
        if (!isCtor)
        {
            var r = mapper.MapReturn(fn.ReturnType, out var why);
            if (r is null)
            {
                Skipped.Add($"{where}: {why}");
                return;
            }
            ret = r;
        }

        // One variant per arity from the first defaulted parameter up to the full list
        var ps = fn.Parameters;
        int firstDefault = ps.Count;
        for (int i = 0; i < ps.Count; i++)
            if (ps[i].HasDefaultArg) { firstDefault = i; break; }

        var mapped = new List<Param>();
        for (int i = 0; i <= ps.Count; i++)
        {
            if (i >= firstDefault)
                Add(mapped.ToList());
            if (i == ps.Count)
                break;
            var t = mapper.MapParam(ps[i].Type, out var why);
            if (t is null)
            {
                if (i < firstDefault)
                    Skipped.Add($"{where}: {why}");
                return;
            }
            var pname = ps[i].Name.Length > 0 ? ps[i].Name : $"arg{i}";
            mapped.Add(new Param(pname, t));
        }

        void Add(List<Param> args)
        {
            var member = new Member
            {
                Owner = cls, DeclaringClass = src, Name = name, IsCtor = isCtor,
                IsStatic = !isCtor && fn is not CXXMethodDecl { IsStatic: false },
                IsConst = fn is CXXMethodDecl { IsConst: true }, Params = args, Return = ret,
            };
            if (!seen.Add(member.Signature))
                return;   // an override or a more-derived declaration already bound it
            if (!seen.Add("cs:" + member.CsSignature))
            {
                Skipped.Add($"{where}: same C# signature as another overload");
                return;
            }
            member.HidesBase = !isCtor && inherited.Contains(member.Signature);
            member.Export = "b3d_" + cls.Name + "_" + name
                            + string.Concat(args.Select(a => "__" + a.Type.Key.Replace("*", "").Replace(" ", "")));
            foreach (var t in args.Select(a => a.Type).Append(ret))
                if (t.Kind is Kind.Value or Kind.MutValue or Kind.Handle or Kind.TransRef)
                    cls.Referenced.Add(t.Name);
            cls.Members.Add(member);
        }
    }

    static bool IsPublic(Decl d) => d.Access == CX_CXXAccessSpecifier.CX_CXXPublic;

    static CXXRecordDecl? BaseDecl(CXXBaseSpecifier b)
    {
        var canon = b.Type.CanonicalType;
        return canon is RecordType rt ? (rt.Decl as CXXRecordDecl)?.Definition as CXXRecordDecl : null;
    }

    static IEnumerable<CXXBaseSpecifier> PublicBases(CXXRecordDecl decl) =>
        decl.Bases.Where(b => b.AccessSpecifier == CX_CXXAccessSpecifier.CX_CXXPublic);

    /// <summary>All publicly inherited ancestors, nearest first, each once.</summary>
    public static IEnumerable<CXXRecordDecl> AllBases(CXXRecordDecl decl)
    {
        var seen = new HashSet<string>();
        var queue = new Queue<CXXRecordDecl>([decl]);
        while (queue.Count > 0)
            foreach (var b in PublicBases(queue.Dequeue()))
                if (BaseDecl(b) is { } bd && seen.Add(bd.Name))
                {
                    yield return bd;
                    queue.Enqueue(bd);
                }
    }

    static ClassInfo? PrimaryWhitelistedBase(CXXRecordDecl decl, Dictionary<string, ClassInfo> classes)
    {
        for (var d = decl; ;)
        {
            var first = PublicBases(d).FirstOrDefault(b => !b.IsVirtual);
            if (first is null || BaseDecl(first) is not { } bd)
                return null;
            if (classes.TryGetValue(bd.Name, out var info))
                return info;
            d = bd;
        }
    }
}
