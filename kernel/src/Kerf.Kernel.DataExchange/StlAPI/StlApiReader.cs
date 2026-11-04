// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Reader.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Stl.Api;

/// <summary>
/// Reads STL into a shape: a compound of planar faces, one per triangle, sharing vertices and edges
/// (<see cref="MakeShapeOnMesh"/>). Slow for large files: <see cref="RWStl"/> reads them into a
/// triangulation.
/// </summary>
/// <remarks>The shape is published frozen. The limits for hostile input are an addition (<see cref="StlReadLimits"/>).</remarks>
internal class StlApiReader
{
    /// <summary>Reads an STL file into the shape (left unchanged on failure). True on success.</summary>
    public bool Read(ref Shape shape, string fileName, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        Triangulation? mesh = global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(fileName, progress, limits);
        return MakeShape(ref shape, mesh);
    }

    /// <summary>
    /// Reads STL from a stream into the shape (left unchanged on failure): ASCII or binary as
    /// <see cref="RWStl.ReadStream"/> recognises them. True on success.
    /// </summary>
    public bool Read(ref Shape shape, Stream stream, ProgressRange progress = default, StlReadLimits? limits = null)
    {
        Triangulation? mesh = global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(stream, Math.PI / 2.0, progress, limits);
        return MakeShape(ref shape, mesh);
    }

    private static bool MakeShape(ref Shape shape, Triangulation? aMesh)
    {
        if (aMesh is null)
        {
            return false;
        }

        var converter = new MakeShapeOnMesh(aMesh);
        converter.Build();
        if (!converter.IsDone)
        {
            return false;
        }

        Shape result = converter.Shape();
        if (result.IsNull)
        {
            return false;
        }

        shape = result;
        return true;
    }
}
