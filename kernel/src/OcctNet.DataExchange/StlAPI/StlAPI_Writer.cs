// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Writer.hxx
// Port of: src/DataExchange/TKDESTL/StlAPI/StlAPI_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Writes the triangulations of a shape's faces (as <see cref="BRepMesh_IncrementalMesh"/> makes them) to STL, ASCII
/// (the default) or binary: one triangulation of all the faces, placed by the faces' locations, triangles of reversed
/// faces turned over. Faces without a triangulation are skipped, with a warning.
/// </summary>
/// <remarks>
/// The shape is only read. The file form writes the stream form's bytes (LF line ends: decision 1168).
/// </remarks>
public class StlAPI_Writer
{
    private bool myASCIIMode;

    /// <summary>A writer in ASCII mode.</summary>
    public StlAPI_Writer()
    {
        myASCIIMode = true;
    }

    /// <summary>True (the default) to write ASCII, false to write binary.</summary>
    public bool ASCIIMode() => myASCIIMode;

    /// <summary>Sets the mode: true for ASCII, false for binary (OCCT's <c>ASCIIMode() = value</c>).</summary>
    public void SetASCIIMode(bool theASCIIMode) => myASCIIMode = theASCIIMode;

    /// <summary>Converts the shape to STL and writes it to the file. False on an error or with no triangulation.</summary>
    public bool Write(in Shape theShape, string theFileName, ProgressRange theProgress = default)
    {
        FileStream? aStream = RWStl.CreateFile(theFileName);
        if (aStream is null)
        {
            return false;
        }

        using (aStream)
        {
            return Write(theShape, aStream, theProgress);
        }
    }

    /// <summary>Converts the shape to STL and writes it to the stream. False on an error or with no triangulation.</summary>
    public bool Write(in Shape theShape, Stream theStream, ProgressRange theProgress = default)
    {
        int aNbNodes = 0;
        int aNbTriangles = 0;

        // calculate total number of the nodes and triangles
        for (var anExpSF = new Explorer(theShape, ShapeType.Face); anExpSF.More(); anExpSF.Next())
        {
            Triangulation? aTriangulation = Tool.Triangulation(ShapeCasts.Face(anExpSF.Current()), out _);
            if (aTriangulation is not null)
            {
                aNbNodes += aTriangulation.NodeCount;
                aNbTriangles += aTriangulation.TriangleCount;
            }
        }

        if (aNbTriangles == 0)
        {
            // No triangulation on the shape
            return false;
        }

        // create temporary triangulation
        var aMesh = new Triangulation(aNbNodes, aNbTriangles, false);
        // count faces missing triangulation
        int aNbFacesNoTri = 0;
        // fill temporary triangulation
        int aNodeOffset = 0;
        int aTriangleOffet = 0;
        for (var anExpSF = new Explorer(theShape, ShapeType.Face); anExpSF.More(); anExpSF.Next())
        {
            Shape aFace = anExpSF.Current();
            Triangulation? aTriangulation = Tool.Triangulation(ShapeCasts.Face(aFace), out Placement aLoc);
            if (aTriangulation is null)
            {
                ++aNbFacesNoTri;
                continue;
            }

            // copy nodes
            Transform3D aTrsf = aLoc.Transformation();
            for (int aNodeIter = 1; aNodeIter <= aTriangulation.NodeCount; ++aNodeIter)
            {
                Point3D aPnt = aTriangulation.Node(aNodeIter);
                aPnt.Transform(aTrsf);
                aMesh.SetNode(aNodeIter + aNodeOffset, aPnt);
            }

            // copy triangles
            Orientation anOrientation = anExpSF.Current().Orientation();
            for (int aTriIter = 1; aTriIter <= aTriangulation.TriangleCount; ++aTriIter)
            {
                Triangle1 aTri = aTriangulation.Triangle(aTriIter);

                aTri.Get(out int anId0, out int anId1, out int anId2);
                if (anOrientation == Reversed)
                {
                    // Swap 1, 2.
                    (anId1, anId2) = (anId2, anId1);
                }

                // Update nodes according to the offset.
                anId0 += aNodeOffset;
                anId1 += aNodeOffset;
                anId2 += aNodeOffset;

                aTri.Set(anId0, anId1, anId2);
                aMesh.SetTriangle(aTriIter + aTriangleOffet, aTri);
            }

            aNodeOffset += aTriangulation.NodeCount;
            aTriangleOffet += aTriangulation.TriangleCount;
        }

        bool isDone = myASCIIMode ? RWStl.WriteAscii(aMesh, theStream, theProgress)
                                  : RWStl.WriteBinary(aMesh, theStream, theProgress);

        if (isDone && (aNbFacesNoTri > 0))
        {
            // Print warning with number of faces missing triangulation
            string aWarningMsg = "Warning: " + aNbFacesNoTri
                                 + ((aNbFacesNoTri == 1) ? " face has" : " faces have")
                                 + " been skipped due to null triangulation";
            global::Kerf.Engine.Foundation.Message.SendWarning(aWarningMsg);
        }

        return isDone;
    }
}
