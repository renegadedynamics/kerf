// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// What tools/stepgen knows of OCCT's classes: every class and enum of the packages it reads (the ones it generates,
// and the ones the generated code calls: StepData, Interface, TCollection_HAsciiString...), and how their C++ types
// map to C#.

namespace StepGen;

internal sealed class Db
{
    public readonly Dictionary<string, CppClass> Classes = new(StringComparer.Ordinal);
    public readonly Dictionary<string, CppEnum> Enums = new(StringComparer.Ordinal);
    public readonly Dictionary<string, string> EnumOfMember = new(StringComparer.Ordinal);
    /// <summary>The classes being generated (entity and RW packages of this run).</summary>
    public readonly HashSet<string> Generated = new(StringComparer.Ordinal);
    /// <summary>The C++ value classes the generator gives a copy constructor (SELECT types and the plain ones).</summary>
    public readonly HashSet<string> ValueClasses = new(StringComparer.Ordinal);
    public readonly List<string> Log = [];
    /// <summary>Handles passed by non-const reference that C# passes by value (the callee never reassigns them).</summary>
    public static readonly HashSet<string> ByValueHandles = ["Interface_Check"];
    /// <summary>
    /// "Class::Method" called as obj->Class::Method() somewhere in TKDESTEP's sources: a virtual one gets a
    /// non-virtual twin Class_Method (decision 12604).
    /// </summary>
    public readonly HashSet<string> QualifiedCalls = new(StringComparer.Ordinal);

    public void AddHeader(string thePath, string theRelPath, string thePackage)
    {
        var aTokens = Lexer.Lex(File.ReadAllText(thePath));
        List<CppClass> aClasses;
        List<CppEnum> anEnums;
        try
        {
            (aClasses, anEnums) = Parser.ParseHeader(aTokens, theRelPath);
        }
        catch (Exception ex) when (ex is FormatException or ArgumentOutOfRangeException or InvalidOperationException)
        {
            Log.Add($"cannot read {theRelPath}: {ex.Message}");
            return;
        }
        foreach (var c in aClasses)
        {
            c.Package = thePackage;
            if (!Classes.TryAdd(c.Name, c))
            {
                Log.Add($"duplicate class {c.Name} in {theRelPath}");
            }
        }

        foreach (var e in anEnums)
        {
            Enums[e.Name] = e;
            foreach (var m in e.Members)
            {
                EnumOfMember[m.Name] = e.Name;
            }
        }
    }

    public CppClass? Class(string? theName) => theName != null && Classes.TryGetValue(theName, out var c) ? c : null;

    public bool IsEnum(string theName) => Enums.ContainsKey(theName);

    public IEnumerable<CppClass> Ancestors(string theName, bool theSelf = true)
    {
        var c = Class(theName);
        if (c != null && !theSelf)
        {
            c = Class(c.Base);
        }

        int aGuard = 0;
        while (c != null && aGuard++ < 64)
        {
            yield return c;
            c = Class(c.Base);
        }
    }

    public bool DerivesFrom(string theName, string theAncestor) => Ancestors(theName).Any(c => c.Name == theAncestor);

    /// <summary>A class used through handles in C++ (Standard_Transient's descendants).</summary>
    public bool IsTransient(string theName)
    {
        foreach (var c in Ancestors(theName))
        {
            if (c.Base == "Standard_Transient" || c.Name == "Standard_Transient")
            {
                return true;
            }
        }

        return theName == "Standard_Transient";
    }

    public IEnumerable<CppMethod> FindMethods(string? theClass, string theName)
    {
        if (theClass == null)
        {
            yield break;
        }

        foreach (var c in Ancestors(theClass))
        {
            foreach (var m in c.Methods)
            {
                if (m.Name == theName && !m.IsCtor)
                {
                    yield return m;
                }
            }
        }
    }

    // ------------------------------------------------------------------ type mapping

    public static readonly Dictionary<string, string> BuiltinMap = new()
    {
        ["void"] = "void", ["bool"] = "bool", ["char"] = "char", ["int"] = "int", ["double"] = "double",
        ["float"] = "float", ["long"] = "long", ["short"] = "short", ["unsigned"] = "uint", ["unsigned int"] = "uint",
        ["size_t"] = "int", ["Standard_Integer"] = "int", ["Standard_Real"] = "double", ["Standard_Boolean"] = "bool",
        ["Standard_Character"] = "char", ["Standard_ExtCharacter"] = "char", ["Standard_Size"] = "int",
        ["Standard_CString"] = "string", ["uint8_t"] = "byte", ["int32_t"] = "int", ["uint32_t"] = "uint",
        ["unsigned char"] = "byte", ["long long"] = "long", ["Standard_Address"] = "object",
    };

    /// <summary>The C# type for a C++ type (without ref/out).</summary>
    public string Map(CppType t)
    {
        if (t.IsHandle)
        {
            return Map(t.Args[0]);
        }

        if (t.IsCString || (t.Name == "char" && t.Pointers >= 1))
        {
            return "string";
        }

        if (t.Name is "TCollection_AsciiString" or "std::string" or "std::string_view")
        {
            return "string";
        }

        if (t.Name == "Standard_Type")
        {
            return "Type";
        }

        if (t.Name == "std::array" && t.Args.Count == 1)
        {
            return Map(t.Args[0]) + "[]";
        }

        if (BuiltinMap.TryGetValue(t.Name, out string? b))
        {
            return b;
        }

        string aName = t.Name.Replace("::", ".");
        if (t.Args.Count > 0)
        {
            aName += "<" + string.Join(", ", t.Args.Select(Map)) + ">";
        }

        return aName;
    }

    /// <summary>A C++ reference to this type becomes a C# <c>ref</c> (values, enums, handles, strings).</summary>
    public bool IsRefKind(CppType t)
    {
        if (t.IsHandle || t.IsCString || t.Pointers > 0)
        {
            return true;
        }

        if (t.Name is "TCollection_AsciiString" or "std::string")
        {
            return true;
        }

        if (BuiltinMap.ContainsKey(t.Name))
        {
            return true;
        }

        if (IsEnum(t.Name) || t.Name is "StepData_Logical" or "Interface_ParamType")
        {
            return true;
        }

        // a struct in OCCT.NET (gp)
        if (t.Name.StartsWith("gp_", StringComparison.Ordinal))
        {
            return true;
        }

        return false;
    }

    /// <summary>The parameter's C# modifier: "ref " for a non-const reference to a ref kind.</summary>
    public string ParamModifier(CppType t)
    {
        // occ::handle<Interface_Check>&: the callee completes the check, never replaces it, so the reference
        // is passed by value, as StepData's port does (decision 12602)
        if (t.IsHandle && ByValueHandles.Contains(t.Args[0].Name))
        {
            return "";
        }

        if (t.IsRef && IsRefKind(t))
        {
            bool isConst = t.Pointers > 0 ? t.IsConstPointer : t.IsConst;
            if (!isConst)
            {
                return "ref ";
            }
        }

        return "";
    }

    /// <summary>A class type C++ holds by value, which C# holds by reference (a class): copied where C++ copies.</summary>
    public bool IsValueClass(CppType t) => !t.IsHandle && t.Pointers == 0 && ValueClasses.Contains(t.Name);

    /// <summary>A class type held by value in C++ (not a handle, builtin, enum or string).</summary>
    public bool IsClassByValue(CppType t)
    {
        if (t.IsHandle || t.Pointers > 0 || t.IsCString)
        {
            return false;
        }

        if (BuiltinMap.ContainsKey(t.Name) || IsEnum(t.Name) || t.Name is "TCollection_AsciiString" or "std::string")
        {
            return false;
        }

        if (t.Name.StartsWith("gp_", StringComparison.Ordinal) || t.Name == "std::array")
        {
            return false;
        }

        return true;
    }

    public static readonly HashSet<string> CSharpKeywords =
    [
        "abstract", "as", "base", "byte", "checked", "decimal", "delegate", "event", "explicit", "extern", "fixed",
        "foreach", "implicit", "in", "interface", "internal", "is", "lock", "namespace", "object", "out", "override",
        "params", "readonly", "ref", "sbyte", "sealed", "stackalloc", "string", "typeof", "uint", "ulong", "unchecked",
        "unsafe", "ushort", "using", "event", "operator", "checked", "fixed", "base", "params",
    ];

    public static string Id(string theName) => CSharpKeywords.Contains(theName) ? "@" + theName : theName;
}
