// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>
/// The attributes of a sub-shape of a part (a face, an edge, a solid...), as XCAF's sub-shape labels: the sub-shape as
/// found in the part's shape (with its location there) and its attributes (colours, name, layers).
/// </summary>
/// <remarks>Immutable; the sub-shape is frozen when made.</remarks>
public sealed class XdeDoc_SubShape
{
    /// <summary>The sub-shape (frozen here, if it isn't yet) with its attributes.</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public XdeDoc_SubShape(in Shape theShape, XdeDoc_Attributes theAttributes)
    {
        if (theShape.IsNull)
        {
            throw new ArgumentException("A sub-shape needs a shape", nameof(theShape));
        }
        theShape.TShape()?.FreezeGraph();
        Shape = theShape;
        Attributes = theAttributes ?? throw new ArgumentNullException(nameof(theAttributes));
    }

    /// <summary>The sub-shape, in the part's coordinates.</summary>
    public Shape Shape { get; }

    /// <summary>Its attributes.</summary>
    public XdeDoc_Attributes Attributes { get; }

    /// <summary>The sub-shape with other attributes.</summary>
    public XdeDoc_SubShape WithAttributes(XdeDoc_Attributes theAttributes) => new(Shape, theAttributes);
}
