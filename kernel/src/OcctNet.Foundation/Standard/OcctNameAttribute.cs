// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The OCCT name of a renamed class (unit 10E), where the name still reaches users or recordings: an exception's
/// <see cref="Standard_Failure.OcctType"/>, which Draw-lite prints and the oracle's recordings compare.
/// </summary>
[AttributeUsage(AttributeTargets.Class, Inherited = false)]
public sealed class OcctNameAttribute(string name) : Attribute
{
    /// <summary>The class's name in OCCT, such as <c>Standard_DomainError</c>.</summary>
    public string Name { get; } = name;
}
