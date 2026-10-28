using ClangSharp;
using ClangSharp.Interop;
using Type = ClangSharp.Type;

namespace OcctGen;

enum Kind
{
    Void,
    Prim,       // fixed-size arithmetic type, passed as is
    Bool,       // uint8_t across the ABI
    Enum,       // int32_t across the ABI
    CString,    // const char*, UTF-8
    Value,      // whitelisted value class, by pointer
    MutValue,   // whitelisted value class through a non-const reference
    Handle,     // occ::handle<T> of a whitelisted transient class, by raw pointer
    RefPrim,    // double& / int&, by pointer
    RefBool,    // bool&, by uint8_t pointer
}

/// <summary>How one parameter or return type crosses the C ABI.</summary>
sealed record Mapped(Kind Kind, string Name)
{
    // Name is the C type for Prim, the enum name for Enum, the class name for Value/MutValue/Handle

    public static readonly Mapped Void = new(Kind.Void, "void");

    /// <summary>Part of the exported function name, distinguishing overloads.</summary>
    public string Key => Kind switch
    {
        Kind.Prim or Kind.RefPrim => (Kind == Kind.RefPrim ? "ref_" : "") + CsPrim(Name),
        Kind.Bool => "bool",
        Kind.RefBool => "ref_bool",
        Kind.CString => "string",
        Kind.MutValue => "mut_" + Name,
        _ => Name,
    };

    public static string CsPrim(string c) => c switch
    {
        "double" => "double",
        "float" => "float",
        "int32_t" => "int",
        "uint32_t" => "uint",
        "int16_t" => "short",
        "uint16_t" => "ushort",
        "int64_t" => "long",
        "uint64_t" => "ulong",
        _ => throw new ArgumentException(c),
    };
}

/// <summary>Maps clang types to their ABI representation, or explains why they can't cross.</summary>
sealed class TypeMapper(Func<string, ClassInfo?> lookupClass, Action<EnumDecl> useEnum)
{
    public Mapped? MapParam(Type type, out string why)
    {
        why = "";
        var canon = type.CanonicalType;
        if (canon is LValueReferenceType lref)
        {
            var pointee = lref.PointeeType.CanonicalType;
            bool isConst = pointee.IsLocalConstQualified;
            if (isConst)
                return MapByValue(pointee, out why, forParam: true);
            if (pointee is BuiltinType b)
            {
                if (b.Kind == CXTypeKind.CXType_Bool)
                    return new Mapped(Kind.RefBool, "bool");
                if (b.Kind is CXTypeKind.CXType_Double or CXTypeKind.CXType_Int)
                    return new Mapped(Kind.RefPrim, PrimName(b.Kind)!);
                why = $"non-const reference to {type.AsString}";
                return null;
            }
            var cls = ClassOf(pointee);
            if (cls is { IsTransient: false, IsStatic: false })
                return new Mapped(Kind.MutValue, cls.Name);
            why = $"non-const reference to {type.AsString}";
            return null;
        }
        if (canon is RValueReferenceType)
        {
            why = "rvalue reference";
            return null;
        }
        return MapByValue(canon, out why, forParam: true);
    }

    public Mapped? MapReturn(Type type, out string why)
    {
        why = "";
        var canon = type.CanonicalType;
        if (canon is LValueReferenceType lref)
        {
            var pointee = lref.PointeeType.CanonicalType;
            if (!pointee.IsLocalConstQualified)
            {
                // A mutable reference into the object's state can't be honoured by a copy
                why = $"returns non-const reference {type.AsString}";
                return null;
            }
            return MapByValue(pointee, out why, forParam: false);
        }
        return MapByValue(canon, out why, forParam: false);
    }

    Mapped? MapByValue(Type canon, out string why, bool forParam)
    {
        why = "";
        switch (canon)
        {
            case BuiltinType b when b.Kind == CXTypeKind.CXType_Void:
                if (forParam) break;
                return Mapped.Void;
            case BuiltinType b when b.Kind == CXTypeKind.CXType_Bool:
                return new Mapped(Kind.Bool, "bool");
            case BuiltinType b when PrimName(b.Kind) is { } prim:
                return new Mapped(Kind.Prim, prim);
            case PointerType p when p.PointeeType.CanonicalType is BuiltinType pb
                                    && pb.Kind is CXTypeKind.CXType_Char_S or CXTypeKind.CXType_Char_U
                                    && p.PointeeType.CanonicalType.IsLocalConstQualified:
                return new Mapped(Kind.CString, "const char*");
            case EnumType e:
                if (e.Decl.DeclContext is not TranslationUnitDecl)
                {
                    why = $"nested enum {canon.AsString}";
                    return null;
                }
                useEnum(e.Decl);
                return new Mapped(Kind.Enum, e.Decl.Name);
            case RecordType r:
                if (HandleTarget(r) is { } target)
                {
                    var tcls = lookupClass(target);
                    if (tcls is { IsTransient: true })
                        return new Mapped(Kind.Handle, tcls.Name);
                    why = $"handle to unbound {target}";
                    return null;
                }
                var cls = lookupClass(r.Decl.Name);
                if (cls is { IsTransient: false, IsStatic: false })
                    return new Mapped(Kind.Value, cls.Name);
                why = cls is null ? $"unbound type {canon.AsString}" : $"unsupported use of {cls.Name}";
                return null;
        }
        why = $"unsupported type {canon.AsString}";
        return null;
    }

    ClassInfo? ClassOf(Type canon) => canon is RecordType r ? lookupClass(r.Decl.Name) : null;

    /// <summary>T for opencascade::handle&lt;T&gt;, else null.</summary>
    static string? HandleTarget(RecordType r)
    {
        if (r.Decl is ClassTemplateSpecializationDecl spec && spec.Name == "handle"
            && spec.DeclContext is NamespaceDecl { Name: "opencascade" }
            && spec.TemplateArgs.Count == 1
            && spec.TemplateArgs[0].AsType.CanonicalType is RecordType inner)
            return inner.Decl.Name;
        return null;
    }

    static string? PrimName(CXTypeKind kind) => kind switch
    {
        CXTypeKind.CXType_Double => "double",
        CXTypeKind.CXType_Float => "float",
        CXTypeKind.CXType_Int => "int32_t",
        CXTypeKind.CXType_UInt => "uint32_t",
        CXTypeKind.CXType_Short => "int16_t",
        CXTypeKind.CXType_UShort => "uint16_t",
        CXTypeKind.CXType_LongLong => "int64_t",
        CXTypeKind.CXType_ULongLong => "uint64_t", // size_t on Windows; converted implicitly elsewhere
        _ => null,
    };
}
