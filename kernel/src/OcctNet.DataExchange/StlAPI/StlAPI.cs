// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Stl.Api;

/// <summary>Short-cuts to <see cref="ApiWriter"/> and <see cref="StlApiReader"/>.</summary>
public static class StlApi
{
    /// <summary>Writes the shape's triangulations to an STL file, ASCII (the default) or binary.</summary>
    public static bool Write(in Shape shape, string file, bool asciiMode = true)
    {
        var writer = new ApiWriter();
        writer.SetASCIIMode(asciiMode);
        return writer.Write(shape, file);
    }

    /// <summary>
    /// Legacy interface: reads an STL file into a shape of one face per facet (very slow for large files; see
    /// <see cref="RWStl"/>).
    /// </summary>
    [Obsolete("This method is very inefficient; see RWStl class for better alternative")]
    public static bool Read(ref Shape shape, string aFile)
    {
        var reader = new StlApiReader();
        return reader.Read(ref shape, aFile);
    }
}
