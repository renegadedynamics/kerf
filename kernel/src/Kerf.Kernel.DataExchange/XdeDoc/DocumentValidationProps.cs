// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// A shape's validation properties, as XCAF keeps them in <c>XCAFDoc_Volume</c>, <c>XCAFDoc_Area</c> and
/// <c>XCAFDoc_Centroid</c>: the values a STEP file states for the shape, to check a reading against. Each is optional.
/// </summary>
public sealed record DocumentValidationProps
{
    /// <summary>No property.</summary>
    public static DocumentValidationProps None { get; } = new();

    /// <summary>The volume, in the document's length unit cubed.</summary>
    public double? Volume { get; init; }

    /// <summary>The area, in the document's length unit squared.</summary>
    public double? Area { get; init; }

    /// <summary>The centroid.</summary>
    public Point3D? Centroid { get; init; }

    /// <summary>True if no property is set.</summary>
    public bool IsEmpty => Volume is null && Area is null && Centroid is null;
}
