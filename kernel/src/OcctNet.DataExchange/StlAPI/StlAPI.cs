// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>Short-cuts to <see cref="StlAPI_Writer"/> and <see cref="StlAPI_Reader"/>.</summary>
public static class StlAPI
{
    /// <summary>Writes the shape's triangulations to an STL file, ASCII (the default) or binary.</summary>
    public static bool Write(in Shape theShape, string theFile, bool theAsciiMode = true)
    {
        var aWriter = new StlAPI_Writer();
        aWriter.SetASCIIMode(theAsciiMode);
        return aWriter.Write(theShape, theFile);
    }

    /// <summary>
    /// Legacy interface: reads an STL file into a shape of one face per facet (very slow for large files; see
    /// <see cref="RWStl"/>).
    /// </summary>
    [Obsolete("This method is very inefficient; see RWStl class for better alternative")]
    public static bool Read(ref Shape theShape, string aFile)
    {
        var aReader = new StlAPI_Reader();
        return aReader.Read(ref theShape, aFile);
    }
}
