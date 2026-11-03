// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Reader.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Meshing;

namespace Occt;

/// <summary>
/// Reads STL into a shape: a compound of planar faces, one per triangle, sharing vertices and edges
/// (<see cref="BRepBuilderAPI_MakeShapeOnMesh"/>). Slow for large files: <see cref="RWStl"/> reads them into a
/// triangulation.
/// </summary>
/// <remarks>The shape is published frozen. The limits for hostile input are an addition (<see cref="RWStl_ReadLimits"/>).</remarks>
public class StlAPI_Reader
{
    /// <summary>Reads an STL file into the shape (left unchanged on failure). True on success.</summary>
    public bool Read(ref TopoDS_Shape theShape, string theFileName, ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        Triangulation? aMesh = RWStl.ReadFile(theFileName, theProgress, theLimits);
        return MakeShape(ref theShape, aMesh);
    }

    /// <summary>
    /// Reads STL from a stream into the shape (left unchanged on failure): ASCII if the data start with "solid",
    /// binary otherwise, as OCCT's <see cref="RWStl.ReadStream"/>. True on success.
    /// </summary>
    public bool Read(ref TopoDS_Shape theShape, Stream theStream, ProgressRange theProgress = default, RWStl_ReadLimits? theLimits = null)
    {
        Triangulation? aMesh = RWStl.ReadStream(theStream, Math.PI / 2.0, theProgress, theLimits);
        return MakeShape(ref theShape, aMesh);
    }

    private static bool MakeShape(ref TopoDS_Shape theShape, Triangulation? aMesh)
    {
        if (aMesh is null)
        {
            return false;
        }

        var aConverter = new BRepBuilderAPI_MakeShapeOnMesh(aMesh);
        aConverter.Build();
        if (!aConverter.IsDone())
        {
            return false;
        }

        TopoDS_Shape aResult = aConverter.Shape();
        if (aResult.IsNull())
        {
            return false;
        }

        theShape = aResult;
        return true;
    }
}
