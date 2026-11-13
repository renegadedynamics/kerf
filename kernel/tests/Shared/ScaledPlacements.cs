// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

/// <summary>
/// A shape moved or located by any placement, a scale among them, through the internal forms: the public
/// <see cref="Shape.Moved"/> and <see cref="Shape.Located"/> refuse a scale (decision 2452), and these tests measure
/// what the kernel's own code does with one.
/// </summary>
internal static class ScaledPlacements
{
    public static Shape MovedAny(this Shape theShape, in Placement thePlacement)
    {
        theShape.Move(thePlacement);
        return theShape;
    }

    public static Shape LocatedAny(this Shape theShape, in Placement thePlacement)
    {
        theShape.Location(thePlacement);
        return theShape;
    }
}
