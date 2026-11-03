// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The viewer's mesh extraction (unit 4W): counts, order, normals, winding, locations and orientations, and where each
/// edge's polyline comes from. The oracle compares it with build123d.NET's native extraction float for float.
/// </summary>
public class MeshExtractionTests
{
    private const double Deflection = 0.05, Angle = 0.3;

    private static TopoDS_Shape Meshed(in TopoDS_Shape theShape)
        => new BRepMesh_IncrementalMesh(theShape, Deflection, false, Angle, false).Shape();

    private static Xyz Vec(float[] theBuffer, long theIndex)
        => new(theBuffer[3 * theIndex], theBuffer[3 * theIndex + 1], theBuffer[3 * theIndex + 2]);

    /// <summary>Every triangle's geometric normal (from its winding) agrees with its nodes' normals.</summary>
    private static void AssertWoundOutward(MeshBuffers theBuffers)
    {
        for (int f = 0; f < theBuffers.FaceTypes.Length; f++)
        {
            long aFirst = theBuffers.FaceNodes[f];
            for (long t = theBuffers.FaceTriangles[f]; t < theBuffers.FaceTriangles[f + 1]; t += 3)
            {
                Xyz a = Vec(theBuffers.Positions, aFirst + theBuffers.Triangles[t]);
                Xyz b = Vec(theBuffers.Positions, aFirst + theBuffers.Triangles[t + 1]);
                Xyz c = Vec(theBuffers.Positions, aFirst + theBuffers.Triangles[t + 2]);
                Xyz aWinding = (b - a).Crossed(c - a);
                if (aWinding.Modulus() < 1e-12)
                {
                    continue;
                }
                Xyz aNormal = Vec(theBuffers.Normals, aFirst + theBuffers.Triangles[t])
                               + Vec(theBuffers.Normals, aFirst + theBuffers.Triangles[t + 1])
                               + Vec(theBuffers.Normals, aFirst + theBuffers.Triangles[t + 2]);
                Assert.True(aWinding.Dot(aNormal) > 0, $"face {f}, triangle {(t - theBuffers.FaceTriangles[f]) / 3} is wound inward");
            }
        }
    }

    private static void AssertConsistentOffsets(MeshBuffers theBuffers)
    {
        Assert.Equal(theBuffers.FaceTypes.Length + 1, theBuffers.FaceNodes.Length);
        Assert.Equal(theBuffers.FaceTypes.Length + 1, theBuffers.FaceTriangles.Length);
        Assert.Equal(theBuffers.EdgeTypes.Length + 1, theBuffers.EdgePoints.Length);
        Assert.Equal(0, theBuffers.FaceNodes[0]);
        Assert.Equal(0, theBuffers.EdgePoints[0]);
        Assert.Equal(theBuffers.Positions.Length, 3 * theBuffers.FaceNodes[^1]);
        Assert.Equal(theBuffers.Normals.Length, theBuffers.Positions.Length);
        Assert.Equal(theBuffers.Triangles.Length, theBuffers.FaceTriangles[^1]);
        Assert.Equal(theBuffers.Points.Length, 3 * theBuffers.EdgePoints[^1]);
        for (int f = 0; f < theBuffers.FaceTypes.Length; f++)
        {
            long aNodes = theBuffers.FaceNodes[f + 1] - theBuffers.FaceNodes[f];
            Assert.True(aNodes >= 0);
            Assert.Equal(0, (theBuffers.FaceTriangles[f + 1] - theBuffers.FaceTriangles[f]) % 3);
            for (long t = theBuffers.FaceTriangles[f]; t < theBuffers.FaceTriangles[f + 1]; t++)
            {
                Assert.InRange(theBuffers.Triangles[t], 0, aNodes - 1);
            }
        }
        for (int i = 0; i < theBuffers.Normals.Length; i += 3)
        {
            Assert.Equal(1.0, Vec(theBuffers.Normals, i / 3).Modulus(), 5);
        }
    }

    [Fact]
    public void A_box_gives_its_faces_and_edges_in_order_with_flat_outward_normals()
    {
        TopoDS_Shape aBox = Meshed(new BRepPrimAPI_MakeBox(10, 20, 30).Shape());
        MeshBuffers aBuffers = MeshExtraction.Extract(aBox, Deflection, Angle);

        List<TopoDS_Shape> aFaces = MeshExtraction.Faces(aBox);
        Assert.Equal(6, aFaces.Count);
        Assert.Equal(12, MeshExtraction.Edges(aBox).Count);
        Assert.Equal(6, aBuffers.FaceTypes.Length);
        Assert.Equal(12, aBuffers.EdgeTypes.Length);
        Assert.All(aBuffers.FaceTypes, t => Assert.Equal((int)SurfaceKind.Plane, t));
        Assert.All(aBuffers.EdgeTypes, t => Assert.Equal((int)CurveKind.Line, t));
        AssertConsistentOffsets(aBuffers);
        AssertWoundOutward(aBuffers);

        // The faces come in exploration order, each with its own triangulation's nodes
        var aExplored = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aExplored.Add(anExp.Current());
        }
        Assert.Equal(aExplored.Count, aFaces.Count);
        var aCenter = new Xyz(5, 10, 15);
        for (int f = 0; f < 6; f++)
        {
            Assert.True(aExplored[f].IsEqual(aFaces[f]));
            Triangulation aTri = BRep_Tool.Triangulation(TopoDS.Face(aFaces[f]), out _)!;
            Assert.Equal(aTri.NodeCount, aBuffers.FaceNodes[f + 1] - aBuffers.FaceNodes[f]);
            Assert.Equal(3 * aTri.TriangleCount, aBuffers.FaceTriangles[f + 1] - aBuffers.FaceTriangles[f]);
            for (long n = aBuffers.FaceNodes[f]; n < aBuffers.FaceNodes[f + 1]; n++)
            {
                // A box's face normal is an axis pointing away from the centre
                Xyz aNormal = Vec(aBuffers.Normals, n);
                Xyz aFirstNormal = Vec(aBuffers.Normals, aBuffers.FaceNodes[f]);
                Assert.True(aNormal.IsEqual(aFirstNormal, 1e-7));
                Assert.True(aNormal.Dot(Vec(aBuffers.Positions, n) - aCenter) > 0);
                Assert.Equal(1.0, Math.Abs(aNormal.X()) + Math.Abs(aNormal.Y()) + Math.Abs(aNormal.Z()), 6);
            }
        }
    }

    [Fact]
    public void A_sphere_has_radial_normals_and_its_edges_lie_on_its_mesh()
    {
        TopoDS_Shape aSphere = Meshed(new BRepPrimAPI_MakeSphere(7).Shape());
        MeshBuffers aBuffers = MeshExtraction.Extract(aSphere, Deflection, Angle);
        AssertConsistentOffsets(aBuffers);
        AssertWoundOutward(aBuffers);
        Assert.Equal((int)SurfaceKind.Sphere, aBuffers.FaceTypes[0]);
        for (long n = 0; n < aBuffers.FaceNodes[^1]; n++)
        {
            Xyz aP = Vec(aBuffers.Positions, n);
            Assert.Equal(7.0, aP.Modulus(), 4);
            // The surface's normal at the node, not an average of the facets
            Assert.True(Vec(aBuffers.Normals, n).IsEqual(aP / aP.Modulus(), 1e-5), $"node {n}");
        }

        // Every edge point is one of the face's nodes (the polygon on triangulation)
        var aNodes = new HashSet<(float, float, float)>();
        for (long n = 0; n < aBuffers.FaceNodes[^1]; n++)
        {
            aNodes.Add((aBuffers.Positions[3 * n], aBuffers.Positions[3 * n + 1], aBuffers.Positions[3 * n + 2]));
        }
        Assert.True(aBuffers.EdgePoints[^1] > 0);
        for (long p = 0; p < aBuffers.EdgePoints[^1]; p++)
        {
            Assert.Contains((aBuffers.Points[3 * p], aBuffers.Points[3 * p + 1], aBuffers.Points[3 * p + 2]), aNodes);
        }
    }

    [Fact]
    public void A_reversed_shape_flips_its_normals_and_winding()
    {
        TopoDS_Shape aCylinder = Meshed(new BRepPrimAPI_MakeCylinder(4, 9).Shape());
        MeshBuffers aForward = MeshExtraction.Extract(aCylinder, Deflection, Angle);
        MeshBuffers aReversed = MeshExtraction.Extract(aCylinder.Reversed(), Deflection, Angle);
        AssertWoundOutward(aForward);
        AssertWoundOutward(aReversed);
        Assert.Equal(aForward.Positions, aReversed.Positions);
        Assert.Equal(aForward.Normals.Select(x => -x), aReversed.Normals);
        for (int t = 0; t < aForward.Triangles.Length; t += 3)
        {
            Assert.Equal(aForward.Triangles[t], aReversed.Triangles[t]);
            Assert.Equal(aForward.Triangles[t + 1], aReversed.Triangles[t + 2]);
            Assert.Equal(aForward.Triangles[t + 2], aReversed.Triangles[t + 1]);
        }
        // Each edge's polyline runs the other way (a seam's from the other polygon of its pair, the same points but for the last bits)
        for (int e = 0; e < aForward.EdgeTypes.Length; e++)
        {
            long a0 = aForward.EdgePoints[e], a1 = aForward.EdgePoints[e + 1];
            Assert.Equal(a1 - a0, aReversed.EdgePoints[e + 1] - aReversed.EdgePoints[e]);
            for (long i = 0; i < a1 - a0; i++)
            {
                Assert.True(Vec(aForward.Points, a0 + i).IsEqual(Vec(aReversed.Points, aReversed.EdgePoints[e] + (a1 - a0 - 1 - i)), 1e-5));
            }
        }
    }

    [Fact]
    public void Located_and_mirrored_shapes_give_located_buffers_wound_outward()
    {
        TopoDS_Shape aCone = Meshed(new BRepPrimAPI_MakeCone(5, 2, 8).Shape());
        MeshBuffers aPlain = MeshExtraction.Extract(aCone, Deflection, Angle);

        var aRotation = new Transform3D();
        aRotation.SetRotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0.5)), 0.8);
        var aMirror = new Transform3D();
        aMirror.SetMirror(new Frame3D(new Point3D(0, 0, 2), new Direction3D(0.2, 0.1, 1)));
        foreach (Transform3D aTrsf in new[] { aRotation, aMirror })
        {
            MeshBuffers aMoved = MeshExtraction.Extract(aCone.Moved(new Placement(aTrsf)), Deflection, Angle);
            AssertConsistentOffsets(aMoved);
            AssertWoundOutward(aMoved);
            Assert.Equal(aPlain.FaceNodes, aMoved.FaceNodes);
            Assert.Equal(aPlain.EdgePoints, aMoved.EdgePoints);
            for (long n = 0; n < aPlain.FaceNodes[^1]; n++)
            {
                Xyz aExpected = new Point3D(Vec(aPlain.Positions, n)).Transformed(aTrsf).XYZ();
                Assert.True(Vec(aMoved.Positions, n).IsEqual(aExpected, 1e-4));
            }
            for (long p = 0; p < aPlain.EdgePoints[^1]; p++)
            {
                Xyz aExpected = new Point3D(Vec(aPlain.Points, p)).Transformed(aTrsf).XYZ();
                Assert.True(Vec(aMoved.Points, p).IsEqual(aExpected, 1e-4));
            }
        }
    }

    [Fact]
    public void Edges_come_from_polygons_on_triangulation_3D_polygons_or_tangential_deflection()
    {
        // A free circle: the mesher gives it a 3D polygon
        TopoDS_Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(Constants.Origin(), Constants.DZ()), 3)).Edge();
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(10, 0, 0), 2, 2, 2).Shape();
        TopoDS_Shape aMeshed = Meshed(TopoDS_Builder.MakeCompound(aBox, aCircle));
        MeshBuffers aBuffers = MeshExtraction.Extract(aMeshed, Deflection, Angle);
        List<TopoDS_Shape> anEdges = MeshExtraction.Edges(aMeshed);
        Assert.Equal(13, anEdges.Count);
        int aCircleIndex = anEdges.FindIndex(e => BRepAdaptor_Curve_Type(e) == CurveKind.Circle);
        Assert.Equal((int)CurveKind.Circle, aBuffers.EdgeTypes[aCircleIndex]);
        Polygon3D aPolygon = BRep_Tool.Polygon3D(TopoDS.Edge(anEdges[aCircleIndex]), out _)!;
        Assert.Equal(aPolygon.NodeCount, aBuffers.EdgePoints[aCircleIndex + 1] - aBuffers.EdgePoints[aCircleIndex]);
        for (int i = 1; i <= aPolygon.NodeCount; i++)
        {
            Point3D aP = aPolygon.Nodes().Value(i);
            Assert.Equal((float)aP.X(), aBuffers.Points[3 * (aBuffers.EdgePoints[aCircleIndex] + i - 1)]);
        }

        // Unmeshed: every edge is discretised with the given tolerances, and faces give no nodes
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(4, 9).Shape();
        MeshBuffers anUnmeshed = MeshExtraction.Extract(aCylinder, Deflection, Angle);
        AssertConsistentOffsets(anUnmeshed);
        Assert.Empty(anUnmeshed.Positions);
        Assert.Empty(anUnmeshed.Triangles);
        List<TopoDS_Shape> aCylEdges = MeshExtraction.Edges(aCylinder);
        for (int e = 0; e < aCylEdges.Count; e++)
        {
            var aDiscretizer = new GCPnts_TangentialDeflection(new BRepAdaptor_Curve(TopoDS.Edge(aCylEdges[e])), Angle, Deflection);
            Assert.Equal(aDiscretizer.NbPoints(), anUnmeshed.EdgePoints[e + 1] - anUnmeshed.EdgePoints[e]);
        }

        // Each polyline starts at its edge's first vertex, as oriented
        TopoDS_Shape aMeshedCylinder = Meshed(aCylinder);
        MeshBuffers aMeshedBuffers = MeshExtraction.Extract(aMeshedCylinder, Deflection, Angle);
        List<TopoDS_Shape> aMeshedEdges = MeshExtraction.Edges(aMeshedCylinder);
        for (int e = 0; e < aMeshedEdges.Count; e++)
        {
            Point3D aFirst = BRep_Tool.Pnt(TopExp.FirstVertex(TopoDS.Edge(aMeshedEdges[e]), true));
            Assert.True(Vec(aMeshedBuffers.Points, aMeshedBuffers.EdgePoints[e]).IsEqual(aFirst.XYZ(), 1e-5), $"edge {e}");
        }
    }

    private static CurveKind BRepAdaptor_Curve_Type(in TopoDS_Shape theEdge) => new BRepAdaptor_Curve(TopoDS.Edge(theEdge)).GetType();

    [Fact]
    public void Each_face_and_edge_comes_once_with_its_last_orientation()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        List<TopoDS_Shape> aFaces = MeshExtraction.Faces(aBox);
        TopoDS_Compound aCompound = TopoDS_Builder.MakeCompound(aFaces[0], aFaces[1], aFaces[0].Reversed());
        List<TopoDS_Shape> aDistinct = MeshExtraction.Faces(aCompound);
        Assert.Equal(2, aDistinct.Count);
        Assert.True(aDistinct[0].IsEqual(aFaces[0].Reversed()));
        Assert.True(aDistinct[1].IsEqual(aFaces[1]));
        // Every edge of a solid is reached twice, once each way: the second (last) one is kept
        Assert.Equal(12, MeshExtraction.Edges(aBox).Count);
        Assert.Empty(MeshExtraction.Faces(new TopoDS_Shape()));
    }

    [Fact]
    public void Extraction_writes_nothing_and_is_the_same_from_any_thread()
    {
        TopoDS_Shape aShape = Meshed(new BRepPrimAPI_MakeTorus(6, 2).Shape());
        MeshBuffers aFirst = MeshExtraction.Extract(aShape, Deflection, Angle);
        foreach (TopoDS_Shape aFace in MeshExtraction.Faces(aShape))
        {
            Triangulation aTri = BRep_Tool.Triangulation(TopoDS.Face(aFace), out _)!;
            Assert.False(aTri.HasNormals);
        }
        var aResults = new MeshBuffers[8];
        Parallel.For(0, aResults.Length, i => aResults[i] = MeshExtraction.Extract(aShape, Deflection, Angle));
        foreach (MeshBuffers aResult in aResults)
        {
            Assert.Equal(aFirst.Positions, aResult.Positions);
            Assert.Equal(aFirst.Normals, aResult.Normals);
            Assert.Equal(aFirst.Triangles, aResult.Triangles);
            Assert.Equal(aFirst.Points, aResult.Points);
        }
    }

    [Fact]
    public void Given_faces_and_edges_are_extracted_in_the_order_given()
    {
        TopoDS_Shape aBox = Meshed(new BRepPrimAPI_MakeBox(3, 4, 5).Shape());
        List<TopoDS_Shape> aFaces = MeshExtraction.Faces(aBox);
        List<TopoDS_Shape> anEdges = MeshExtraction.Edges(aBox);
        MeshBuffers anAll = MeshExtraction.Extract(aBox, Deflection, Angle);
        MeshBuffers aSome = MeshExtraction.Extract(aBox, [aFaces[3], aFaces[1]], [anEdges[5]], Deflection, Angle);
        Assert.Equal(2, aSome.FaceTypes.Length);
        Assert.Single(aSome.EdgeTypes);
        Assert.Equal(anAll.Positions.AsSpan((int)(3 * anAll.FaceNodes[3]), (int)(3 * (anAll.FaceNodes[4] - anAll.FaceNodes[3]))).ToArray(),
                     aSome.Positions.AsSpan(0, (int)(3 * aSome.FaceNodes[1])).ToArray());
        Assert.Equal(anAll.Points.AsSpan((int)(3 * anAll.EdgePoints[5]), (int)(3 * (anAll.EdgePoints[6] - anAll.EdgePoints[5]))).ToArray(),
                     aSome.Points);
    }
}
