// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using System.Collections.Immutable;

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// A part where a tree places it (<see cref="Tree.Flatten"/>), for meshing and viewing: the part, the instances
/// leading to it from a root, the location composed along them and the style resolved along them.
/// </summary>
/// <param name="Product">The part.</param>
/// <param name="Path">The instances from the root (first) to the part's own (last).</param>
/// <param name="Location">The part's location in the world: the path's locations composed.</param>
/// <param name="Style">The style: the path's attributes merged, each instance's after its product's.</param>
internal sealed record FlatItem(DocumentProduct Product,
                                     ImmutableArray<Instance> Path,
                                     Placement Location,
                                     Style Style)
{
    /// <summary>The part's shape placed in the world.</summary>
    public Shape Shape
    {
        get
        {
            Shape shape = Product.Shape;
            return Location.IsIdentity ? shape : shape.Moved(Location);
        }
    }

    /// <summary>The names along the path (each instance's, or its product's), null where there is none.</summary>
    public IEnumerable<string?> NamePath => Path.Select(i => i.DisplayName);
}
