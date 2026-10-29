// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Explores a shape for the parts to mesh: its faces and its free edges.</summary>
public class IMeshTools_ShapeExplorer : IMeshData_Shape
{
    /// <summary>An explorer of the shape.</summary>
    public IMeshTools_ShapeExplorer(in TopoDS_Shape theShape) : base(theShape)
    {
    }

    /// <summary>
    /// Visits the free edges (located at the identity), then, for each face with a surface in
    /// <see cref="BRepLib.ReverseSortFaces(in TopoDS_Shape, NCollection_List{TopoDS_Shape})"/>'s order (each TShape and
    /// orientation once, whatever its location), the face's edges and the face itself, forward.
    /// </summary>
    public virtual void Accept(IMeshTools_ShapeVisitor theVisitor)
    {
        // Explore all free edges in shape.
        visitEdges(theVisitor, GetShape(), true, TopAbs_EDGE, TopAbs_FACE);

        // Explore all related to some face edges in shape.
        // make array of faces suitable for processing (excluding faces without surface)
        NCollection_List<TopoDS_Shape> aFaceList = new();
        BRepLib.ReverseSortFaces(GetShape(), aFaceList);
        NCollection_Map<TopoDS_Shape> aFaceMap = new(TopTools_ShapeMapHasher.Instance);

        TopLoc_Location aEmptyLoc = new();
        var aFaceIter = new NCollection_List<TopoDS_Shape>.Iterator(aFaceList);
        for (; aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Shape aFaceNoLoc = aFaceIter.Value();
            aFaceNoLoc.Location(aEmptyLoc);
            if (!aFaceMap.Add(aFaceNoLoc))
            {
                continue; // already processed
            }

            TopoDS_Face aFace = TopoDS.Face(aFaceIter.Value());
            if (!BRep_Tool.IsGeometric(aFace))
            {
                continue;
            }

            // Explore all edges in face.
            visitEdges(theVisitor, aFace, false, TopAbs_EDGE);

            // Store only forward faces in order to prevent inverse issue.
            theVisitor.Visit(TopoDS.Face(aFace.Oriented(TopAbs_FORWARD)));
        }
    }

    // Explodes the given shape on edges according to the specified criteria and visits each one in order to add it
    // to data model.
    private static void visitEdges(IMeshTools_ShapeVisitor theVisitor,
                                   in TopoDS_Shape theShape,
                                   bool isResetLocation,
                                   TopAbs_ShapeEnum theToFind,
                                   TopAbs_ShapeEnum theToAvoid = TopAbs_SHAPE)
    {
        var aEdgesIt = new TopExp_Explorer(theShape, theToFind, theToAvoid);
        for (; aEdgesIt.More(); aEdgesIt.Next())
        {
            TopoDS_Edge aEdge = TopoDS.Edge(aEdgesIt.Current());
            if (!BRep_Tool.IsGeometric(aEdge))
            {
                continue;
            }

            theVisitor.Visit(isResetLocation ? TopoDS.Edge(aEdge.Located(new TopLoc_Location())) : aEdge);
        }
    }
}
