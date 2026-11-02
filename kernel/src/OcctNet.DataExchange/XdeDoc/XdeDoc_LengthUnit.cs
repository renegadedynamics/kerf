// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The length unit of a tree's shapes, as XCAF's <c>XCAFDoc_LengthUnit</c>: a name and the unit's size in metres.
/// </summary>
/// <param name="Name">The unit's name, such as "MM".</param>
/// <param name="UnitValue">The unit's size in metres (0.001 for millimetres).</param>
public sealed record XdeDoc_LengthUnit(string Name, double UnitValue)
{
    /// <summary>Millimetres, OCCT's default.</summary>
    public static XdeDoc_LengthUnit Millimetre { get; } = new("MM", 0.001);

    /// <summary>Metres.</summary>
    public static XdeDoc_LengthUnit Metre { get; } = new("M", 1.0);
}
