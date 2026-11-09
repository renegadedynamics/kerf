// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#if KERF_0_4_OR_LATER
extern alias v016;

using Kerf.Modeling;
using Model = v016::Renegade.BladeBumper;

namespace Kerf.UserCases;

/// <summary>The v0.16 enclosure (enclosure-v0.16/README.md), in its own library under the alias <c>v016</c>.</summary>
internal static class EnclosureV016Model
{
    /// <summary>The whole model built again at 35 mm: every printed part, the print aid and the components, as the v0.11 case reads them.</summary>
    public static object Build()
    {
        Model.Rebuild.Forget();
        Model.Bumper.Use(35);
        var shapes = new List<(string, Part)>();
        shapes.AddRange(Model.Bumper.Parts.Select(kv => ("part " + kv.Key, kv.Value)));
        shapes.AddRange(Model.Bumper.PrintAids.Select(kv => ("aid " + kv.Key, kv.Value)));
        shapes.AddRange(Model.Bumper.Components.Select(kv => ("component " + kv.Key, kv.Value)));
        _ = Model.Bumper.Layout;
        return shapes;
    }
}
#endif
