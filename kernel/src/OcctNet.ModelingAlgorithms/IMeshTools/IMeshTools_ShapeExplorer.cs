// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Explores a shape for the parts to mesh: its faces and its free edges.</summary>
public class IMeshTools_ShapeExplorer : IMeshData_Shape
{
    /// <summary>An explorer of the shape.</summary>
    public IMeshTools_ShapeExplorer(in Shape theShape) : base(theShape)
    {
    }

    /// <summary>
    /// Visits the free edges (located at the identity), then, for each face with a surface in
    /// <see cref="BRepLib.ReverseSortFaces(in Shape, NCollection_List{Shape})"/>'s order (each TShape and
    /// orientation once, whatever its location), the face's edges and the face itself, forward.
    /// </summary>
    public virtual void Accept(IMeshTools_ShapeVisitor theVisitor)
    {
        // Explore all free edges in shape.
        visitEdges(theVisitor, GetShape(), true, ShapeType.Edge, ShapeType.Face);

        // Explore all related to some face edges in shape.
        // make array of faces suitable for processing (excluding faces without surface)
        NCollection_List<Shape> aFaceList = new();
        BRepLib.ReverseSortFaces(GetShape(), aFaceList);
        NCollection_Map<Shape> aFaceMap = new(ShapeHasher.Instance);

        Placement aEmptyLoc = new();
        var aFaceIter = new NCollection_List<Shape>.Iterator(aFaceList);
        for (; aFaceIter.More(); aFaceIter.Next())
        {
            Shape aFaceNoLoc = aFaceIter.Value();
            aFaceNoLoc.Location(aEmptyLoc);
            if (!aFaceMap.Add(aFaceNoLoc))
            {
                continue; // already processed
            }

            Face aFace = ShapeCasts.Face(aFaceIter.Value());
            if (!Tool.IsGeometric(aFace))
            {
                continue;
            }

            // Explore all edges in face.
            visitEdges(theVisitor, aFace, false, ShapeType.Edge);

            // Store only forward faces in order to prevent inverse issue.
            theVisitor.Visit(ShapeCasts.Face(aFace.Oriented(Forward)));
        }
    }

    // Explodes the given shape on edges according to the specified criteria and visits each one in order to add it
    // to data model.
    private static void visitEdges(IMeshTools_ShapeVisitor theVisitor,
                                   in Shape theShape,
                                   bool isResetLocation,
                                   ShapeType theToFind,
                                   ShapeType theToAvoid = ShapeType.Shape)
    {
        var aEdgesIt = new Explorer(theShape, theToFind, theToAvoid);
        for (; aEdgesIt.More(); aEdgesIt.Next())
        {
            Edge aEdge = ShapeCasts.Edge(aEdgesIt.Current());
            if (!Tool.IsGeometric(aEdge))
            {
                continue;
            }

            theVisitor.Visit(isResetLocation ? ShapeCasts.Edge(aEdge.Located(new Placement())) : aEdge);
        }
    }
}
