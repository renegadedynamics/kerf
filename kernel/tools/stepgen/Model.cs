// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// The C++ model tools/stepgen reads OCCT's headers and sources into.

namespace StepGen;

/// <summary>A C++ type as written: a (qualified) name, template arguments, const, references and pointers.</summary>
internal sealed class CppType
{
    public string Name = "";
    public List<CppType> Args = [];
    /// <summary>Non-type template arguments (std::array's size).</summary>
    public List<string> ValueArgs = [];
    public bool IsConst;
    public bool IsRef;
    public int Pointers;
    /// <summary>Pointer constness (<c>const char* const</c>).</summary>
    public bool IsConstPointer;

    public bool IsHandle => Name is "occ::handle" or "Handle" or "opencascade::handle" or "NCollection_Handle";
    public CppType Element => IsHandle ? Args[0] : this;

    public bool IsCString => (Name is "char" && Pointers == 1) || Name == "Standard_CString";

    public CppType Clone() => new()
    {
        Name = Name, Args = Args.Select(a => a.Clone()).ToList(), ValueArgs = [.. ValueArgs], IsConst = IsConst, IsRef = IsRef,
        Pointers = Pointers, IsConstPointer = IsConstPointer,
    };

    /// <summary>The type without const and reference (the value a variable of this type holds).</summary>
    public CppType Bare()
    {
        var a = Clone();
        a.IsConst = false;
        a.IsRef = false;
        return a;
    }

    public override string ToString()
    {
        string s = (IsConst ? "const " : "") + Name;
        if (Args.Count > 0 || ValueArgs.Count > 0)
        {
            s += "<" + string.Join(", ", Args.Select(a => a.ToString()).Concat(ValueArgs)) + ">";
        }

        s += new string('*', Pointers) + (IsRef ? "&" : "");
        return s;
    }
}

internal sealed class CppParam
{
    public CppType Type = new();
    public string? Name;
    public List<Token>? Default;
    /// <summary>A C array parameter (<c>double x[3]</c>).</summary>
    public bool IsArray;
}

internal sealed class CppMethod
{
    public string Name = "";
    public CppType? Return; // null for constructors
    public List<CppParam> Params = [];
    public bool IsConst, IsVirtual, IsOverride, IsStatic, IsPure, IsDefault, IsDeleted, IsCtor, IsDtor, IsInline;
    public string Access = "public";
    public List<string> Doc = [];
    public List<Token>? Body;
    public List<Token>? InitList;
    /// <summary>Template parameters (<c>template &lt;class T&gt;</c>).</summary>
    public List<string> TemplateParams = [];
    /// <summary>The C++ text this member came from, for the report.</summary>
    public string Origin = "";
    /// <summary>The comment lines just before the definition in the source.</summary>
    public List<string> LeadingComments = [];
    public bool Hidden; // Standard_HIDDEN

    public string SignatureKey(Func<CppType, string> theMap) =>
        Name + "(" + string.Join(",", Params.Select(p => theMap(p.Type))) + ")";
}

internal sealed class CppField
{
    public CppType Type = new();
    public string Name = "";
    public string Access = "private";
    public List<Token>? Init;
    public bool IsStatic;
    public bool IsConstexpr;
    public string? ArraySize;
    public List<string> Doc = [];
}

internal sealed class CppEnum
{
    public string Name = "";
    public List<(string Name, List<Token>? Value, List<string> Doc)> Members = [];
    public List<string> Doc = [];
    public bool IsClassEnum;
}

/// <summary>A class, struct, or (for the RW enum tools) a namespace of constants and inline functions.</summary>
internal sealed class CppClass
{
    public string Name = "";
    public string? Base;
    public List<string> OtherBases = [];
    public bool IsNamespace;
    public bool IsStruct;
    public List<CppMethod> Methods = [];
    public List<CppField> Fields = [];
    public List<CppEnum> Enums = [];
    public List<string> Doc = [];
    public string Header = ""; // path relative to the OCCT source root
    public List<string> Sources = [];
    /// <summary>Functions of the source's anonymous namespace or file-static ones.</summary>
    public List<CppMethod> FileHelpers = [];
    /// <summary>Constants at file scope in the source (static const ...).</summary>
    public List<CppField> FileConstants = [];
    public string Package = "";
    public List<string> Problems = [];
    public bool HasDefineAlloc;
}
