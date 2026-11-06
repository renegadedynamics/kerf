// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Documents;

/// <summary>A layer of a tree, as XCAF's layer labels: a name, and whether the layer's shapes are shown.</summary>
/// <param name="Name">The layer's name, which shapes' <see cref="Attributes.Layers"/> refer to.</param>
/// <param name="IsVisible">False if the layer is hidden.</param>
internal sealed record Layer(string Name, bool IsVisible = true);
