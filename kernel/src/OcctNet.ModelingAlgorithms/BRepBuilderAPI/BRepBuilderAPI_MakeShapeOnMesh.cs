// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShapeOnMesh.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShapeOnMesh.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Builds a shape from a triangulation: a compound of planar faces, one per triangle, sharing their vertices (one per
/// mesh node) and edges (one per mesh link). Degenerate triangles (repeated nodes, or two nodes closer than
/// <see cref="Constants.Resolution"/>) are skipped.
/// </summary>
/// <remarks>
/// The result is published frozen by <see cref="BRepBuilderAPI_MakeShape.Shape"/>; the triangulation is only read.
/// OCCT's map of mesh links is a hash map with insertion-ordered indices (NCollection_IndexedDataMap), as the port's.
/// </remarks>
public class BRepBuilderAPI_MakeShapeOnMesh : BRepBuilderAPI_MakeShape
{
    private readonly Triangulation? myMesh;

    /// <summary>Sets the mesh to process.</summary>
    public BRepBuilderAPI_MakeShapeOnMesh(Triangulation? theMesh)
    {
        myMesh = theMesh;
    }

    // A mesh link: its node indices, the lower first
    private readonly struct Edge : IEquatable<Edge>
    {
        public Edge(int TheIdx1, int TheIdx2)
        {
            Idx1 = Math.Min(TheIdx1, TheIdx2);
            Idx2 = Math.Max(TheIdx1, TheIdx2);
        }

        // First index. It is lower or equal than the second.
        public readonly int Idx1;

        // Second index.
        public readonly int Idx2;

        public bool Equals(Edge theOther) => theOther.Idx1 == Idx1 && theOther.Idx2 == Idx2;

        public override bool Equals(object? obj) => obj is Edge other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Idx1, Idx2);
    }

    /// <summary>Builds the shape on the mesh. Does nothing for a null or empty mesh, or on a user break.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        // Generally, this method guarantees topology sharing by mapping mesh primitives
        // into topological counterparts.
        // mesh points -> topological vertices
        // mesh edges  -> topological edges

        // Cannot reconstruct anything from null or empty mesh.
        if (myMesh is null || myMesh.NodeCount == 0 || myMesh.TriangleCount == 0)
        {
            return;
        }

        int aNbNodes = myMesh.NodeCount;
        int aNbTriangles = myMesh.TriangleCount;

        // We are going to have three loops: iterate once over nodes and iterate twice
        // over triangles of input mesh.
        using var aPS = new ProgressScope(theRange, "Per-facet shape construction", (double)aNbNodes + 2.0 * aNbTriangles);

        // Build shared vertices.
        var aPnt2VertexMap = new NCollection_IndexedDataMap<int, TopoDS_Vertex>();

        for (int i = 1; i <= aNbNodes; ++i)
        {
            aPS.Next();
            if (aPS.UserBreak())
            {
                return;
            }

            Point3D aP = myMesh.Node(i);
            TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(aP).InternalVertex();
            aPnt2VertexMap.Add(i, aV);
        }

        // Build shared edges.
        var anEdgeToTEgeMap = new NCollection_IndexedDataMap<Edge, TopoDS_Edge>();
        for (int i = 1; i <= aNbTriangles; ++i)
        {
            aPS.Next();
            if (aPS.UserBreak())
            {
                return;
            }

            Triangle1 aTriangle = myMesh.Triangle(i);
            aTriangle.Get(out int anIdx0, out int anIdx1, out int anIdx2);

            // Skip degenerated triangles.
            if (anIdx0 == anIdx1 || anIdx0 == anIdx2 || anIdx1 == anIdx2)
            {
                continue;
            }

            Point3D aP1 = myMesh.Node(anIdx0);
            Point3D aP2 = myMesh.Node(anIdx1);
            Point3D aP3 = myMesh.Node(anIdx2);
            double aD1 = aP1.SquareDistance(aP2);
            double aD2 = aP1.SquareDistance(aP3);
            double aD3 = aP2.SquareDistance(aP3);
            if (aD1 < Constants.Resolution() || aD2 < Constants.Resolution() || aD3 < Constants.Resolution())
            {
                continue;
            }

            // Edges are constructed in forward order for the existing normals orientation.
            // In Poly_Triangulation, positive direction is defined as cross product:
            // (aV1, aV2) x (aV1, aV3).
            TopoDS_Vertex aV1 = aPnt2VertexMap.FindFromKey(anIdx0);
            TopoDS_Vertex aV2 = aPnt2VertexMap.FindFromKey(anIdx1);
            TopoDS_Vertex aV3 = aPnt2VertexMap.FindFromKey(anIdx2);

            var aMeshEdge1 = new Edge(anIdx0, anIdx1);
            var aMeshEdge2 = new Edge(anIdx1, anIdx2);
            var aMeshEdge3 = new Edge(anIdx2, anIdx0);

            var aMaker1 = new BRepBuilderAPI_MakeEdge(aV1, aV2);
            var aMaker2 = new BRepBuilderAPI_MakeEdge(aV2, aV3);
            var aMaker3 = new BRepBuilderAPI_MakeEdge(aV3, aV1);

            TopoDS_Edge aTE1 = aMaker1.InternalEdge();
            if (anIdx1 < anIdx0)
            {
                aTE1.Reverse();
            }

            TopoDS_Edge aTE2 = aMaker2.InternalEdge();
            if (anIdx2 < anIdx1)
            {
                aTE2.Reverse();
            }

            TopoDS_Edge aTE3 = aMaker3.InternalEdge();
            if (anIdx0 < anIdx2)
            {
                aTE3.Reverse();
            }

            anEdgeToTEgeMap.Add(aMeshEdge1, aTE1);
            anEdgeToTEgeMap.Add(aMeshEdge2, aTE2);
            anEdgeToTEgeMap.Add(aMeshEdge3, aTE3);
        }

        // Construct planar faces using shared topology.
        TopoDS_Compound aResult = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aResult);
        for (int i = 1; i <= aNbTriangles; ++i)
        {
            aPS.Next();
            if (aPS.UserBreak())
            {
                return;
            }

            Triangle1 aTriangle = myMesh.Triangle(i);
            aTriangle.Get(out int anIdx0, out int anIdx1, out int anIdx2);

            var aMeshEdge1 = new Edge(anIdx0, anIdx1);
            var aMeshEdge2 = new Edge(anIdx1, anIdx2);
            var aMeshEdge3 = new Edge(anIdx2, anIdx0);
            bool isReversed1 = anIdx1 < anIdx0;
            bool isReversed2 = anIdx2 < anIdx1;
            bool isReversed3 = anIdx0 < anIdx2;

            // Edges can be skipped in case of mesh defects - topologically or geometrically
            // degenerated triangles.
            bool aHasAllEdges = anEdgeToTEgeMap.Contains(aMeshEdge1)
                                && anEdgeToTEgeMap.Contains(aMeshEdge2)
                                && anEdgeToTEgeMap.Contains(aMeshEdge3);
            if (!aHasAllEdges)
            {
                continue;
            }

            TopoDS_Edge aTEdge1 = anEdgeToTEgeMap.FindFromKey(aMeshEdge1);
            if (isReversed1)
            {
                aTEdge1.Reverse();
            }
            TopoDS_Edge aTEdge2 = anEdgeToTEgeMap.FindFromKey(aMeshEdge2);
            if (isReversed2)
            {
                aTEdge2.Reverse();
            }
            TopoDS_Edge aTEdge3 = anEdgeToTEgeMap.FindFromKey(aMeshEdge3);
            if (isReversed3)
            {
                aTEdge3.Reverse();
            }

            var aWireMaker = new BRepBuilderAPI_MakeWire();
            aWireMaker.Add(aTEdge1);
            aWireMaker.Add(aTEdge2);
            aWireMaker.Add(aTEdge3);
            TopoDS_Wire aWire = aWireMaker.InternalWire();

            // Construct plane explicitly since it is faster than automatic construction
            // within BRepBuilderAPI_MakeFace.
            var aC1 = new BRepAdaptor_Curve(aTEdge1);
            var aC2 = new BRepAdaptor_Curve(aTEdge2);
            Direction3D aD1 = aC1.Line().Direction();
            Direction3D aD2 = aC2.Line().Direction();
            Xyz aN = aD1.XYZ().Crossed(aD2.XYZ());
            if (aN.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                continue;
            }
            if (aTEdge1.Orientation() == TopAbs_REVERSED)
            {
                aN.Reverse();
            }
            if (aTEdge2.Orientation() == TopAbs_REVERSED)
            {
                aN.Reverse();
            }
            var aNorm = new Direction3D(aN);
            var aPln = new Plane3D(myMesh.Node(anIdx0), aNorm);

            var aFaceMaker = new BRepBuilderAPI_MakeFace(aPln, aWire);
            TopoDS_Face aFace = aFaceMaker.InternalFace();

            aBB.Add(aResult, aFace);
        }

        Done();
        myShape = aResult;
    }
}
