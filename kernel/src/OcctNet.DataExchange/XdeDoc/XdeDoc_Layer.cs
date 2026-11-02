// New in OCCT.NET (unit 8T): the immutable assembly tree in place of XCAF (decision 13101). Not a port.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A layer of a tree, as XCAF's layer labels: a name, and whether the layer's shapes are shown.</summary>
/// <param name="Name">The layer's name, which shapes' <see cref="XdeDoc_Attributes.Layers"/> refer to.</param>
/// <param name="IsVisible">False if the layer is hidden.</param>
public sealed record XdeDoc_Layer(string Name, bool IsVisible = true);
