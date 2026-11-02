// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace Kerf.Tools.Rename;

/// <summary>The systematic rules: OCCT's prefixed names to .NET's.</summary>
internal static class Naming
{
    /// <summary>Words a renamed local or parameter mustn't become, though C# would accept some of them.</summary>
    private static readonly HashSet<string> Contextual = new(StringComparer.Ordinal)
    {
        "var", "dynamic", "nameof", "value", "field", "await", "yield", "async", "global", "record", "scoped", "_",
        "args", "when", "select", "from", "where", "let", "join", "into", "orderby", "group", "by", "on", "equals",
    };

    /// <summary>Whether a name can't be used for a variable as written.</summary>
    public static bool IsReserved(string name)
        => SyntaxFacts.GetKeywordKind(name) != SyntaxKind.None || Contextual.Contains(name);

    /// <summary>
    /// camelCase from the PascalCase remainder of a prefixed name: <c>P1</c> to <c>p1</c>, <c>XYZ</c> to <c>xyz</c>,
    /// <c>XYPlane</c> to <c>xyPlane</c>, <c>Curve</c> to <c>curve</c>.
    /// </summary>
    public static string Camel(string rest)
    {
        int upper = 0;
        while (upper < rest.Length && char.IsUpper(rest[upper]))
        {
            upper++;
        }

        if (upper == 0)
        {
            return rest;
        }

        // An acronym followed by a word keeps the word's capital: XYPlane -> xyPlane
        int lower = upper > 1 && upper < rest.Length && char.IsLower(rest[upper]) ? upper - 1 : upper;
        return rest[..lower].ToLowerInvariant() + rest[lower..];
    }

    public static string Pascal(string name)
        => name.Length == 0 ? name : char.ToUpperInvariant(name[0]) + name[1..];

    /// <summary><c>theX</c> to <c>x</c>, and OCCT's older PascalCase <c>Index</c> to <c>index</c>; null if already camelCase.</summary>
    public static string? Parameter(string name)
        => name.Length > 3 && name.StartsWith("the", StringComparison.Ordinal) && char.IsUpper(name[3]) ? Camel(name[3..])
            : char.IsUpper(name[0]) && !name.Contains('_') ? Camel(name)
            : null;

    /// <summary><c>aX</c>, <c>anX</c> to <c>x</c>; null when the name doesn't follow OCCT's convention.</summary>
    public static string? Local(string name)
    {
        if (name.Length > 2 && name.StartsWith("an", StringComparison.Ordinal) && char.IsUpper(name[2]))
        {
            return Camel(name[2..]);
        }

        return name.Length > 1 && name[0] == 'a' && char.IsUpper(name[1]) ? Camel(name[1..])
            : char.IsUpper(name[0]) && !name.Contains('_') ? Camel(name)
            : null;
    }

    /// <summary>A method's name in PascalCase (OCCT's file-static helpers are camelCase); null if it is already.</summary>
    public static string? Method(IMethodSymbol method)
        => method.MethodKind is MethodKind.Ordinary or MethodKind.LocalFunction && char.IsLower(method.Name[0]) && !method.Name.Contains('_')
            ? Pascal(method.Name)
            : null;

    /// <summary>The prefix-free stem of a field's name (<c>myX</c>, <c>theX</c>, <c>s_X</c>), camelCased, or the name itself.</summary>
    private static string FieldStem(string name)
    {
        foreach (string prefix in (string[])["my", "the", "s_"])
        {
            if (name.Length > prefix.Length && name.StartsWith(prefix, StringComparison.Ordinal) && char.IsUpper(name[prefix.Length]))
            {
                return Camel(name[prefix.Length..]);
            }
        }

        return Camel(name);
    }

    /// <summary>
    /// A field's .NET name, or null to leave it: non-public instance fields <c>_camel</c>; non-public static fields
    /// <c>s_camel</c>, or PascalCase when constant or read-only. Public fields and names with underscores or
    /// digits-only stems are left to the map.
    /// </summary>
    public static string? Field(IFieldSymbol field, bool instances, bool statics)
    {
        string name = field.Name;
        if (field.DeclaredAccessibility == Accessibility.Public || field.ContainingType.TypeKind == TypeKind.Enum
            || field.IsImplicitlyDeclared || name.StartsWith('_') || !char.IsLetter(name[0])
            || (name.Contains('_') && !(name.StartsWith("s_", StringComparison.Ordinal) && name.LastIndexOf('_') == 1)))
        {
            return null;
        }

        string stem = FieldStem(name);
        string? result;
        if (field.IsConst || (field.IsStatic && field.IsReadOnly))
        {
            result = statics ? Pascal(stem) : null;
        }
        else if (field.IsStatic)
        {
            result = statics ? "s_" + stem : null;
        }
        else
        {
            result = instances ? "_" + stem : null;
        }

        return result == name ? null : result;
    }

    /// <summary><c>NbPoles</c> to <c>PoleCount</c>; <c>IsDone</c> and <c>HasX</c> stay as they are.</summary>
    public static string? PropertyFor(string methodName)
    {
        if (methodName.Length > 2 && methodName.StartsWith("Nb", StringComparison.Ordinal) && char.IsUpper(methodName[2]))
        {
            return Singular(methodName[2..]) + "Count";
        }

        if ((methodName.StartsWith("Is", StringComparison.Ordinal) || methodName.StartsWith("Has", StringComparison.Ordinal))
            && methodName.Length > 3 && char.IsUpper(methodName[methodName.StartsWith("Is", StringComparison.Ordinal) ? 2 : 3]))
        {
            return methodName;
        }

        return null;
    }

    /// <summary>English singular of OCCT's plural nouns, for the last word: Poles, Vertices, Entities, Boxes.</summary>
    public static string Singular(string word)
    {
        (string Plural, string One)[] irregular =
        [
            ("Vertices", "Vertex"), ("Indices", "Index"), ("Matrices", "Matrix"), ("Apices", "Apex"),
            ("Children", "Child"), ("Axes", "Axis"), ("Analyses", "Analysis"),
        ];
        foreach ((string plural, string one) in irregular)
        {
            if (word.EndsWith(plural, StringComparison.Ordinal))
            {
                return word[..^plural.Length] + one;
            }
        }

        foreach (string ending in (string[])["xes", "ches", "shes", "sses", "zzes"])
        {
            if (word.EndsWith(ending, StringComparison.Ordinal))
            {
                return word[..^2];
            }
        }

        if (word.EndsWith("ies", StringComparison.Ordinal) && word.Length > 4)
        {
            return word[..^3] + "y";
        }

        if (word.EndsWith("us", StringComparison.Ordinal) || word.EndsWith("is", StringComparison.Ordinal)
            || word.EndsWith("ss", StringComparison.Ordinal) || !word.EndsWith('s') || word.Length < 3)
        {
            return word;
        }

        return word[..^1];
    }
}
