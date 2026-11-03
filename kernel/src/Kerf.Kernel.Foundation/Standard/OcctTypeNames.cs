// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Frozen;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The OCCT names of the kernel's renamed types and enum members (unit 10E), from docs/api/kerf.map. OCCT prints and
/// looks up its types' names (<c>DynamicType()->Name()</c>): message keys (<see cref="Algorithm"/>), STEP's type
/// signatures and statistics, an exception's type (<see cref="FailureException.OcctType"/>); and the kernel's tests
/// compare type and enum names with native OCCT's recordings. Those keep OCCT's names, so renaming a class changes no
/// output, no message and no recording.
/// </summary>
public static partial class OcctTypeNames
{
    private static readonly FrozenDictionary<string, string> Names = Parse(Table);

    private static readonly FrozenDictionary<string, string> MemberNames = Parse(MemberTable);

    /// <summary>
    /// The type's OCCT name, as <c>Standard_Type::Name()</c> gives it (<c>Standard_Transient</c>,
    /// <c>BVH_Tree`2</c>), or its .NET name if OCCT has no other.
    /// </summary>
    public static string Of(Type type)
    {
        ArgumentNullException.ThrowIfNull(type);
        Type definition = type.IsGenericType ? type.GetGenericTypeDefinition() : type;
        return definition.FullName is { } fullName && Names.TryGetValue(fullName, out string? name) ? name : type.Name;
    }

    /// <summary>
    /// An enum value's OCCT name (<c>TopAbs_EDGE</c> for <c>ShapeType.Edge</c>), as OCCT and the recordings of native
    /// OCCT print it, or its .NET name if OCCT has no other.
    /// </summary>
    public static string Of(Enum value)
    {
        ArgumentNullException.ThrowIfNull(value);
        string name = value.ToString();
        return value.GetType().FullName is { } type && MemberNames.TryGetValue(type + "." + name, out string? occt) ? occt : name;
    }

    private static FrozenDictionary<string, string> Parse(string table) => table
        .Split('\n', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
        .Select(line => line.Split(' '))
        .ToFrozenDictionary(parts => parts[0], parts => parts[1], StringComparer.Ordinal);
}
