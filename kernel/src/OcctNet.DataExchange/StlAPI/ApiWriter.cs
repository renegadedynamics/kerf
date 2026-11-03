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
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Stl.Api;

/// <summary>
/// Writes the triangulations of a shape's faces (as <see cref="IncrementalMesh"/> makes them) to STL, ASCII
/// (the default) or binary: one triangulation of all the faces, placed by the faces' locations, triangles of reversed
/// faces turned over. Faces without a triangulation are skipped, with a warning.
/// </summary>
/// <remarks>
/// The shape is only read. The file form writes the stream form's bytes (LF line ends: decision 1168).
/// </remarks>
public class ApiWriter
{
    private bool _asciiMode;

    /// <summary>A writer in ASCII mode.</summary>
    public ApiWriter()
    {
        _asciiMode = true;
    }

    /// <summary>True (the default) to write ASCII, false to write binary.</summary>
    public bool ASCIIMode() => _asciiMode;

    /// <summary>Sets the mode: true for ASCII, false for binary (OCCT's <c>ASCIIMode() = value</c>).</summary>
    public void SetASCIIMode(bool asciiMode) => _asciiMode = asciiMode;

    /// <summary>Converts the shape to STL and writes it to the file. False on an error or with no triangulation.</summary>
    public bool Write(in Shape shape, string fileName, ProgressRange progress = default)
    {
        FileStream? stream = global::Kerf.Engine.Exchange.Stl.RWStl.CreateFile(fileName);
        if (stream is null)
        {
            return false;
        }

        using (stream)
        {
            return Write(shape, stream, progress);
        }
    }

    /// <summary>Converts the shape to STL and writes it to the stream. False on an error or with no triangulation.</summary>
    public bool Write(in Shape shape, Stream stream, ProgressRange progress = default)
    {
        int nbNodes = 0;
        int nbTriangles = 0;

        // calculate total number of the nodes and triangles
        for (var expSF = new Explorer(shape, ShapeType.Face); expSF.More(); expSF.Next())
        {
            Triangulation? triangulation = Tool.Triangulation(ShapeCasts.Face(expSF.Current()), out _);
            if (triangulation is not null)
            {
                nbNodes += triangulation.NodeCount;
                nbTriangles += triangulation.TriangleCount;
            }
        }

        if (nbTriangles == 0)
        {
            // No triangulation on the shape
            return false;
        }

        // create temporary triangulation
        var mesh = new Triangulation(nbNodes, nbTriangles, false);
        // count faces missing triangulation
        int nbFacesNoTri = 0;
        // fill temporary triangulation
        int nodeOffset = 0;
        int triangleOffet = 0;
        for (var expSF = new Explorer(shape, ShapeType.Face); expSF.More(); expSF.Next())
        {
            Shape face = expSF.Current();
            Triangulation? triangulation = Tool.Triangulation(ShapeCasts.Face(face), out Placement loc);
            if (triangulation is null)
            {
                ++nbFacesNoTri;
                continue;
            }

            // copy nodes
            Transform3D trsf = loc.Transformation();
            for (int nodeIter = 1; nodeIter <= triangulation.NodeCount; ++nodeIter)
            {
                Point3D pnt = triangulation.Node(nodeIter);
                pnt.Transform(trsf);
                mesh.SetNode(nodeIter + nodeOffset, pnt);
            }

            // copy triangles
            Orientation orientation = expSF.Current().Orientation();
            for (int triIter = 1; triIter <= triangulation.TriangleCount; ++triIter)
            {
                Triangle1 tri = triangulation.Triangle(triIter);

                tri.Get(out int id0, out int id1, out int id2);
                if (orientation == Reversed)
                {
                    // Swap 1, 2.
                    (id1, id2) = (id2, id1);
                }

                // Update nodes according to the offset.
                id0 += nodeOffset;
                id1 += nodeOffset;
                id2 += nodeOffset;

                tri.Set(id0, id1, id2);
                mesh.SetTriangle(triIter + triangleOffet, tri);
            }

            nodeOffset += triangulation.NodeCount;
            triangleOffet += triangulation.TriangleCount;
        }

        bool isDone = _asciiMode ? global::Kerf.Engine.Exchange.Stl.RWStl.WriteAscii(mesh, stream, progress)
                                  : global::Kerf.Engine.Exchange.Stl.RWStl.WriteBinary(mesh, stream, progress);

        if (isDone && (nbFacesNoTri > 0))
        {
            // Print warning with number of faces missing triangulation
            string warningMsg = "Warning: " + nbFacesNoTri
                                 + ((nbFacesNoTri == 1) ? " face has" : " faces have")
                                 + " been skipped due to null triangulation";
            global::Kerf.Engine.Foundation.Message.SendWarning(warningMsg);
        }

        return isDone;
    }
}
