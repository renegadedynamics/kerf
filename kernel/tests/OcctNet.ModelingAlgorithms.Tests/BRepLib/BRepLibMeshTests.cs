// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib's mesh statics (UpdateDeflection, EnsureNormalConsistency), of
/// BRepLib_ToolTriangulatedShape and of BRepLib_PointCloudShape, on meshes attached (and so frozen) to planar faces.
/// </summary>
public class BRepLibMeshTests
{
    private const double Conf = 1e-7;

    /// <summary>
    /// Attaches to each planar polygon face of the shape a fan triangulation of its wire's vertices with UV nodes and the
    /// given deflection, and to each of its edges the polygon of its two end nodes on that triangulation.
    /// </summary>
    private static void MeshPlanarFaces(in TopoDS_Shape theShape, double theDeflection)
    {
        var aB = new BRep_Builder();
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(theShape))
        {
            TopoDS_Face aFwd = TopoDS.Face(aF.Oriented(TopAbs_FORWARD));
            gp_Pln aPln = ((Geom_Plane)BRep_Tool.Surface(aFwd)!).Pln();
            var aVertices = new List<TopoDS_Vertex>();
            for (var anExp = new TopExp_Explorer(aFwd, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                TopExp.Vertices(TopoDS.Edge(anExp.Current()), out TopoDS_Vertex aFirst, out _, true);
                aVertices.Add(aFirst);
            }

            int n = aVertices.Count;
            var aTri = new Poly_Triangulation(n, n - 2, true);
            for (int i = 0; i < n; i++)
            {
                gp_Pnt aP = BRep_Tool.Pnt(aVertices[i]);
                aTri.SetNode(i + 1, aP);
                ElSLib.Parameters(aPln, aP, out double u, out double v);
                aTri.SetUVNode(i + 1, new gp_Pnt2d(u, v));
            }
            for (int i = 1; i <= n - 2; i++)
            {
                aTri.SetTriangle(i, new Poly_Triangle(1, i + 1, i + 2));
            }
            aTri.Deflection(theDeflection);
            aB.UpdateFace(aF, aTri);

            for (var anExp = new TopExp_Explorer(aFwd, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
                TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                var aNodes = new NCollection_Array1<int>(1, 2);
                aNodes[1] = aVertices.FindIndex(v => v.IsSame(aV1)) + 1;
                aNodes[2] = aVertices.FindIndex(v => v.IsSame(aV2)) + 1;
                aB.UpdateEdge(anE, new Poly_PolygonOnTriangulation(aNodes), aTri, new TopLoc_Location());
            }
        }
    }

    /// <summary>A meshed unit cube, frozen.</summary>
    private static TopoDS_Solid MeshedBox(double theDeflection)
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        MeshPlanarFaces(aBox, theDeflection);
        return TestShapes.Frozen(aBox);
    }

    /// <summary>
    /// Two meshed quadrangles sharing the edge from (1, 1, 0) to (0, 1, 0): one on z = 0, the other rising with slope
    /// <paramref name="theSlope"/>; in a shell, frozen.
    /// </summary>
    private static TopoDS_Shell MeshedRoof(double theSlope)
    {
        var aB = new BRep_Builder();
        gp_Pnt[] aPoints = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(0, 2, theSlope), new(1, 2, theSlope)];
        TopoDS_Vertex[] aV = aPoints.Select(p => TestShapes.Vertex(p)).ToArray();
        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, TestShapes.PolygonFace(aV, new gp_Dir(0, 0, 1), anEdges, [0, 1, 2, 3]));
        aB.Add(aShell, TestShapes.PolygonFace(aV, new gp_Dir(0, -theSlope, 1), anEdges, [3, 2, 5, 4]));
        MeshPlanarFaces(aShell, 0.0);
        return TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)aShell));
    }

    private static Poly_Triangulation Triangulation(in TopoDS_Face theF) => BRep_Tool.Triangulation(theF, out _)!;

    [Fact]
    public void UpdateDeflection_replaces_each_triangulation_by_a_copy_with_the_measured_deflection()
    {
        TopoDS_Solid aBox = MeshedBox(0.3);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        Assert.Throws<TopoDS_LockedShape>(() => BRepLib.UpdateDeflection(aBox));

        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aBox, out BRepTools_History aH);
        BRepLib.UpdateDeflection(aCopy);
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(aBox))
        {
            Poly_Triangulation anOld = Triangulation(aF);
            Assert.Equal(0.3, anOld.Deflection());
            TopoDS_Face aNewF = TopoDS.Face(aH.Modified(aF)[0]);
            Poly_Triangulation aNew = Triangulation(aNewF);
            Assert.NotSame(anOld, aNew);
            Assert.Equal(0.0, aNew.Deflection()); // the planar mesh is exact
            Assert.True(aNew.IsFrozen());
            Assert.Equal(1, ((BRep_TFace)aNewF.TShape()!).NbTriangulations());
            // the edges' polygons now refer to the copy
            for (var anExp = new TopExp_Explorer(aNewF, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
                Assert.NotNull(BRep_Tool.PolygonOnTriangulation(anE, aNew, new TopLoc_Location()));
                Assert.Null(BRep_Tool.PolygonOnTriangulation(anE, anOld, new TopLoc_Location()));
            }
        }
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void DeflectionUpdated_and_NormalConsistencyEnsured_are_the_statics_on_a_copy()
    {
        TopoDS_Solid aBox = MeshedBox(0.3);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aResult = BRepLib.DeflectionUpdated(aBox, out BRepTools_History aH);
        Assert.True(aResult.TShape()!.IsFrozen());
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(aBox))
        {
            TopoDS_Face aNewF = TopoDS.Face(Assert.Single(aH.Modified(aF)));
            Assert.Equal(0.0, Triangulation(aNewF).Deflection());
        }
        // the vertices are the box's own; every edge changed (its polygons refer to the copies)
        var aVertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_VERTEX, aVertices);
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.True(aVertices.Contains(anExp.Current()));
        }

        TopoDS_Shape aNormals = BRepLib.NormalConsistencyEnsured(aBox, out bool isCorrected);
        Assert.True(isCorrected);
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(aNormals))
        {
            Assert.True(Triangulation(aF).HasNormals());
        }
        // normals present: nothing to do, and the result is the input itself
        Assert.True(BRepLib.NormalConsistencyEnsured(aNormals, out isCorrected).IsEqual(aNormals));
        Assert.False(isCorrected);
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void UpdateDeflection_measures_a_mesh_off_its_surface()
    {
        var aB = new BRep_Builder();
        // a cylinder face meshed with one quad: its midpoints are off the surface by 1 - cos(pi / 4) / ...
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(), 1.0);
        TopoDS_Face aF = new BRepLib_MakeFace(aCyl, 0, Math.PI / 2, 0, 1, Conf).InternalFace();
        var aTri = new Poly_Triangulation(4, 2, true);
        (double u, double v)[] aUV = [(0, 0), (Math.PI / 2, 0), (Math.PI / 2, 1), (0, 1)];
        for (int i = 0; i < 4; i++)
        {
            aTri.SetNode(i + 1, aCyl.Value(aUV[i].u, aUV[i].v));
            aTri.SetUVNode(i + 1, new gp_Pnt2d(aUV[i].u, aUV[i].v));
        }
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.SetTriangle(2, new Poly_Triangle(1, 3, 4));
        aB.UpdateFace(aF, aTri);
        BRepLib.UpdateDeflection(aF);
        // the inner link (1, 3): its midpoint (0.5, 0.5, 0.5) against the surface at (pi / 4, 0.5)
        double aLink = new gp_Pnt(0.5, 0.5, 0.5).Distance(aCyl.Value(Math.PI / 4, 0.5));
        // the triangle midpoints: ((1 + 0 + 0) / 3, (0 + 1 + 1) / 3) against (pi / 6, 1 / 3) and the other
        double aMid = new gp_Pnt(1.0 / 3, 2.0 / 3, 1.0 / 3).Distance(aCyl.Value(Math.PI / 3, 1.0 / 3));
        Assert.Equal(Math.Max(aLink, aMid), Triangulation(aF).Deflection(), 1e-12);
    }

    [Fact]
    public void EnsureNormalConsistency_computes_the_missing_normals_on_copies()
    {
        TopoDS_Solid aBox = MeshedBox(0.0);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aBox, out BRepTools_History aH);
        Assert.True(BRepLib.EnsureNormalConsistency(aCopy));
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(aBox))
        {
            Assert.False(Triangulation(aF).HasNormals());
            TopoDS_Face aNewF = TopoDS.Face(aH.Modified(aF)[0]);
            Poly_Triangulation aNew = Triangulation(aNewF);
            Assert.True(aNew.HasNormals());
            gp_Dir aN = ((Geom_Plane)BRep_Tool.Surface(aNewF)!).Pln().Axis().Direction();
            if (aNewF.Orientation() == TopAbs_REVERSED)
            {
                aN.Reverse();
            }
            for (int i = 1; i <= aNew.NbNodes(); i++)
            {
                Assert.True(aNew.Normal(i).IsEqual(aN, 1e-6)); // the box's faces are at right angles: nothing is smoothed
            }
        }
        // normals present: nothing to do
        Assert.False(BRepLib.EnsureNormalConsistency(aCopy));
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void EnsureNormalConsistency_smooths_the_normals_along_a_shallow_edge()
    {
        const double aSlope = 0.05;
        TopoDS_Shell aRoof = MeshedRoof(aSlope);
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aRoof, out _);
        Assert.True(BRepLib.EnsureNormalConsistency(aCopy, 0.1));
        List<TopoDS_Face> aFaces = BRepGPropTestShapes.Faces(aCopy);
        var aN1 = new gp_XYZ(0, 0, 1);
        gp_XYZ aN2 = new gp_Dir(0, -aSlope, 1).XYZ();
        var anAverage = new gp_Dir(aN1 + aN2);
        Poly_Triangulation aT1 = Triangulation(aFaces[0]), aT2 = Triangulation(aFaces[1]);
        // nodes 3, 4 of the first face are (1, 1, 0), (0, 1, 0); nodes 1, 2 of the second
        foreach ((Poly_Triangulation aT, int aNode) in new[] { (aT1, 3), (aT1, 4), (aT2, 1), (aT2, 2) })
        {
            Assert.True(aT.Normal(aNode).IsEqual(anAverage, 1e-6));
        }
        Assert.True(aT1.Normal(1).IsEqual(new gp_Dir(aN1), 1e-6));
        Assert.True(aT2.Normal(3).IsEqual(new gp_Dir(aN2), 1e-6));

        // a smaller angular tolerance: the normals are only computed
        TopoDS_Shape aCopy2 = BRepTools.CopyForWrite(aRoof, out _);
        Assert.True(BRepLib.EnsureNormalConsistency(aCopy2, 0.01));
        Assert.True(Triangulation(BRepGPropTestShapes.Faces(aCopy2)[0]).Normal(3).IsEqual(new gp_Dir(aN1), 1e-6));
    }

    [Fact]
    public void ComputeNormals_writes_into_an_unfrozen_triangulation_only()
    {
        TopoDS_Solid aBox = MeshedBox(0.0);
        TopoDS_Face aF = BRepGPropTestShapes.Faces(aBox)[0];
        Poly_Triangulation aT = Triangulation(aF);
        Assert.Throws<Standard_ImmutableObject>(() => BRepLib_ToolTriangulatedShape.ComputeNormals(aF, aT));

        Poly_Triangulation aWithNormals = BRepLib_ToolTriangulatedShape.NormalsComputed(aF, aT)!;
        Assert.NotSame(aT, aWithNormals);
        Assert.False(aT.HasNormals());
        gp_Dir aN = ((Geom_Plane)BRep_Tool.Surface(aF)!).Pln().Axis().Direction();
        for (int i = 1; i <= aWithNormals.NbNodes(); i++)
        {
            Assert.True(aWithNormals.Normal(i).IsEqual(aN, 1e-6));
        }
        Assert.Same(aWithNormals, BRepLib_ToolTriangulatedShape.NormalsComputed(aF, aWithNormals));
        Assert.Null(BRepLib_ToolTriangulatedShape.NormalsComputed(aF, null));

        // without UV nodes: the average of the triangles' normals
        Poly_Triangulation aCopy = aT.Copy();
        aCopy.RemoveUVNodes();
        BRepLib_ToolTriangulatedShape.ComputeNormals(aF, aCopy, new Poly_Connect());
        Assert.True(aCopy.HasNormals());
    }

    [Fact]
    public void ComputeNormals_falls_back_to_flat_normals_at_a_singular_point()
    {
        // a cone apex: the surface normal is undefined at v = 0
        var aCone = new Geom_ConicalSurface(new gp_Ax3(), Math.PI / 4, 1.0);
        TopoDS_Face aF = BRepGPropTestShapes.NaturalFace(aCone);
        double aV0 = -1.0 / Math.Sin(Math.PI / 4); // the apex
        var aT = new Poly_Triangulation(3, 1, true);
        aT.SetNode(1, aCone.Value(0, aV0));
        aT.SetNode(2, aCone.Value(0, 0));
        aT.SetNode(3, aCone.Value(Math.PI / 2, 0));
        aT.SetUVNode(1, new gp_Pnt2d(0, aV0));
        aT.SetUVNode(2, new gp_Pnt2d(0, 0));
        aT.SetUVNode(3, new gp_Pnt2d(Math.PI / 2, 0));
        aT.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        BRepLib_ToolTriangulatedShape.ComputeNormals(aF, aT);
        gp_XYZ v1 = aT.Node(2).Coord() - aT.Node(1).Coord();
        gp_XYZ v2 = aT.Node(3).Coord() - aT.Node(2).Coord();
        Assert.True(aT.Normal(1).IsEqual(new gp_Dir(v1 ^ v2), 1e-6));
    }

    private sealed class Collector : BRepLib_PointCloudShape
    {
        public readonly List<(gp_Pnt P, gp_Vec N, gp_Pnt2d UV, TopoDS_Shape F)> Points = [];

        public Collector(in TopoDS_Shape theShape) : base(theShape) { }

        protected override void addPoint(in gp_Pnt thePoint, in gp_Vec theNorm, in gp_Pnt2d theUV, in TopoDS_Shape theFace)
            => Points.Add((thePoint, theNorm, theUV, theFace));
    }

    [Fact]
    public void PointCloudShape_by_density_puts_points_on_the_faces()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        var aCloud = new Collector(aBox);
        Assert.Equal(24, aCloud.NbPointsByDensity(0.3));
        Assert.True(aCloud.GeneratePointsByDensity(0.3));
        Assert.Equal(24, aCloud.Points.Count);
        foreach ((gp_Pnt aP, gp_Vec aN, _, _) in aCloud.Points)
        {
            double[] c = [aP.X(), aP.Y(), aP.Z()];
            Assert.All(c, x => Assert.InRange(x, -1e-12, 1 + 1e-12));
            Assert.Contains(c, x => Math.Abs(x) < 1e-12 || Math.Abs(x - 1) < 1e-12);
            Assert.Equal(1.0, aN.Magnitude(), 1e-12);
        }

        // the same seed for every face: the same draws, so the same run gives the same points
        var aCloud2 = new Collector(aBox);
        aCloud2.GeneratePointsByDensity(0.3);
        Assert.Equal(aCloud.Points.Select(p => p.P.XYZ()), aCloud2.Points.Select(p => p.P.XYZ()));

        // moved off along the outward normal by at most the distance
        var aFar = new Collector(aBox);
        aFar.SetDistance(0.5);
        Assert.True(aFar.GeneratePointsByDensity(0.3));
        foreach ((gp_Pnt aP, gp_Vec aN, _, _) in aFar.Points)
        {
            double[] c = [aP.X(), aP.Y(), aP.Z()];
            Assert.All(c, x => Assert.InRange(x, -0.5 - 1e-12, 1.5 + 1e-12));
        }
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void PointCloudShape_with_the_computed_density_gives_one_point_per_face()
    {
        // OCCT divides each face's area by the given density, 0 here: one point per face (occt-issues.md)
        var aCloud = new Collector(TestShapes.Frozen(TestShapes.Box(1, 2, 3)));
        Assert.Equal(6, aCloud.NbPointsByDensity());
        Assert.True(aCloud.GeneratePointsByDensity());
        Assert.Equal(6, aCloud.Points.Count);
    }

    [Fact]
    public void PointCloudShape_by_triangulation_takes_the_nodes_without_writing_normals()
    {
        TopoDS_Solid aBox = MeshedBox(0.0);
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        var aCloud = new Collector(aBox);
        Assert.Equal(24, aCloud.NbPointsByTriangulation());
        Assert.True(aCloud.GeneratePointsByTriangulation());
        Assert.Equal(24, aCloud.Points.Count);
        foreach ((gp_Pnt aP, gp_Vec aN, _, TopoDS_Shape aF) in aCloud.Points)
        {
            gp_Dir aFaceN = ((Geom_Plane)BRep_Tool.Surface(TopoDS.Face(aF))!).Pln().Axis().Direction();
            Assert.True(new gp_Dir(aN).IsParallel(aFaceN, 1e-6));
        }
        aBefore.AssertUnchanged(aBox);

        Assert.False(new Collector(TestShapes.Frozen(TestShapes.Box(1, 1, 1))).GeneratePointsByTriangulation());
    }
}
