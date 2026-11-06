// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Stl.Api;

/// <summary>Short-cuts to <see cref="ApiWriter"/> and <see cref="StlApiReader"/>.</summary>
internal static class StlApi
{
    /// <summary>Writes the shape's triangulations to an STL file, ASCII (the default) or binary.</summary>
    public static bool Write(in Shape shape, string file, bool asciiMode = true)
    {
        var writer = new ApiWriter();
        writer.SetASCIIMode(asciiMode);
        return writer.Write(shape, file);
    }
}
