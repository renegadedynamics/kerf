// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// The attributes of a sub-shape of a part (a face, an edge, a solid...), as XCAF's sub-shape labels: the sub-shape as
/// found in the part's shape (with its location there) and its attributes (colours, name, layers).
/// </summary>
/// <remarks>Immutable; the sub-shape is frozen when made.</remarks>
internal sealed class SubShape
{
    /// <summary>The sub-shape (frozen here, if it isn't yet) with its attributes.</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public SubShape(in Shape shape, Attributes attributes)
    {
        if (shape.IsNull)
        {
            throw new KernelArgumentException("A sub-shape needs a shape", nameof(shape));
        }
        shape.TShape()?.FreezeGraph();
        Shape = shape;
        Attributes = attributes ?? throw new KernelArgumentNullException(nameof(attributes));
    }

    /// <summary>The sub-shape, in the part's coordinates.</summary>
    public Shape Shape { get; }

    /// <summary>Its attributes.</summary>
    public Attributes Attributes { get; }

    /// <summary>The sub-shape with other attributes.</summary>
    public SubShape WithAttributes(Attributes attributes) => new(Shape, attributes);
}
