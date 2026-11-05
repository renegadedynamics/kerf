// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_ShapeExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>Explores a shape for the parts to mesh: its faces and its free edges.</summary>
internal class ShapeExplorer : ShapeModel
{
    /// <summary>An explorer of the shape.</summary>
    public ShapeExplorer(in Shape shape) : base(shape)
    {
    }

    /// <summary>
    /// Visits the free edges (located at the identity), then, for each face with a surface in
    /// <see cref="BrepLibrary.ReverseSortFaces(in Shape, NCollection_List{Shape})"/>'s order (each TShape and
    /// orientation once, whatever its location), the face's edges and the face itself, forward.
    /// </summary>
    public virtual void Accept(ShapeVisitor visitor)
    {
        // Explore all free edges in shape.
        VisitEdges(visitor, GetShape(), true, ShapeType.Edge, ShapeType.Face);

        // Explore all related to some face edges in shape.
        // make array of faces suitable for processing (excluding faces without surface)
        NCollection_List<Shape> faceList = new();
        BrepLibrary.ReverseSortFaces(GetShape(), faceList);
        HashSet<Shape> faceMap = new(ShapeHasher.Instance);

        Placement emptyLoc = new();
        var faceIter = new NCollection_List<Shape>.Iterator(faceList);
        for (; faceIter.More(); faceIter.Next())
        {
            Shape faceNoLoc = faceIter.Value();
            faceNoLoc.Location(emptyLoc);
            if (!faceMap.Add(faceNoLoc))
            {
                continue; // already processed
            }

            Face face = ShapeCasts.Face(faceIter.Value());
            if (!Tool.IsGeometric(face))
            {
                continue;
            }

            // Explore all edges in face.
            VisitEdges(visitor, face, false, ShapeType.Edge);

            // Store only forward faces in order to prevent inverse issue.
            visitor.Visit(ShapeCasts.Face(face.Oriented(Forward)));
        }
    }

    // Explodes the given shape on edges according to the specified criteria and visits each one in order to add it
    // to data model.
    private static void VisitEdges(ShapeVisitor visitor,
                                   in Shape shape,
                                   bool isResetLocation,
                                   ShapeType toFind,
                                   ShapeType toAvoid = ShapeType.Shape)
    {
        var edgesIt = new Explorer(shape, toFind, toAvoid);
        for (; edgesIt.More(); edgesIt.Next())
        {
            Edge edge = ShapeCasts.Edge(edgesIt.Current());
            if (!Tool.IsGeometric(edge))
            {
                continue;
            }

            visitor.Visit(isResetLocation ? ShapeCasts.Edge(edge.Located(new Placement())) : edge);
        }
    }
}
