// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known-answer tests of BRepExtrema: the distance between vertices, edges, faces and solids (supports, parameters,
/// inner solutions, parallel cases, several solutions, located shapes, multi-threading, deflection), the Ext*
/// tools, and the mesh-based tools (triangle sets, overlap, self-intersection, proximity, BRepExtrema_Poly). Inputs
/// are frozen as callers hold them, and checked unchanged. The shapes are built with the BRepBuilderAPI makers.
/// </summary>
public class BRepExtremaTests
{
    private const double Tol = 1.0e-9;

    private static Vertex V(double x, double y, double z) => new MakeVertex(new Point3D(x, y, z)).Vertex();

    private static Edge E(double x1, double y1, double z1, double x2, double y2, double z2)
        => new MakeEdge(new Point3D(x1, y1, z1), new Point3D(x2, y2, z2)).Edge();

    // A square [x0, x0 + s] x [y0, y0 + s] in the plane z = z0, normal +Z.
    private static Face Square(double x0, double y0, double z0, double s)
        => new MakeFace(new Plane3D(new Point3D(x0, y0, z0), new Direction3D(0, 0, 1)), 0, s, 0, s).Face();

    private static void AssertPnt(in Point3D theExpected, in Point3D theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X, theActual.X, theTol);
        Assert.Equal(theExpected.Y, theActual.Y, theTol);
        Assert.Equal(theExpected.Z, theActual.Z, theTol);
    }

    // ------------------------------------------------------------------ vertices and edges

    [Fact]
    public void VertexVertex()
    {
        Vertex a = V(0, 0, 0), b = V(3, 4, 0);
        var aD = new DistanceShapeShape(a, b);
        Assert.True(aD.IsDone);
        Assert.Equal(1, aD.SolutionCount);
        Assert.Equal(5.0, aD.Value());
        Assert.False(aD.InnerSolution());
        Assert.Equal(SupportType.IsVertex, aD.SupportTypeShape1(1));
        Assert.Equal(SupportType.IsVertex, aD.SupportTypeShape2(1));
        Assert.True(aD.SupportOnShape1(1).IsSame(a));
        Assert.True(aD.SupportOnShape2(1).IsSame(b));
        AssertPnt(new Point3D(3, 4, 0), aD.PointOnShape2(1));
        EngineAssert.ThrowsAny<IncompatibleShapeException>(() => aD.ParOnEdgeS1(1, out _));
        EngineAssert.ThrowsAny<IncompatibleShapeException>(() => aD.ParOnFaceS2(1, out _, out _));
    }

    [Fact]
    public void VertexEdge_FootInside_AndBeyondTheEnd()
    {
        Edge anEdge = E(0, 0, 0, 10, 0, 0);
        var aD = new DistanceShapeShape(V(4, 3, 0), anEdge);
        Assert.True(aD.IsDone);
        Assert.Equal(1, aD.SolutionCount);
        Assert.Equal(3.0, aD.Value(), Tol);
        Assert.Equal(SupportType.IsVertex, aD.SupportTypeShape1(1));
        Assert.Equal(SupportType.IsOnEdge, aD.SupportTypeShape2(1));
        Assert.True(aD.SupportOnShape2(1).IsSame(anEdge));
        aD.ParOnEdgeS2(1, out double t);
        Assert.Equal(4.0, t, Tol);
        AssertPnt(new Point3D(4, 0, 0), aD.PointOnShape2(1));

        // beyond the end: the nearest point is the edge's vertex
        var aD2 = new DistanceShapeShape(anEdge, V(13, 4, 0));
        Assert.Equal(5.0, aD2.Value(), Tol);
        Assert.Equal(SupportType.IsVertex, aD2.SupportTypeShape1(1));
        AssertPnt(new Point3D(10, 0, 0), aD2.PointOnShape1(1));
    }

    [Fact]
    public void VertexFace_InFace()
    {
        Face aF = Square(0, 0, 0, 10);
        var aD = new DistanceShapeShape(aF, V(2, 3, 7));
        Assert.True(aD.IsDone);
        Assert.Equal(7.0, aD.Value(), Tol);
        Assert.Equal(1, aD.SolutionCount);
        Assert.Equal(SupportType.IsInFace, aD.SupportTypeShape1(1));
        Assert.True(aD.SupportOnShape1(1).IsSame(aF));
        aD.ParOnFaceS1(1, out double u, out double v);
        Assert.Equal(2.0, u, Tol);
        Assert.Equal(3.0, v, Tol);
        AssertPnt(new Point3D(2, 3, 0), aD.PointOnShape1(1));
    }

    [Fact]
    public void EdgeEdge_Skew()
    {
        Edge e1 = E(-5, 0, 0, 5, 0, 0), e2 = E(1, -5, 2, 1, 5, 2);
        var aD = new DistanceShapeShape(e1, e2);
        Assert.True(aD.IsDone);
        Assert.Equal(1, aD.SolutionCount);
        Assert.Equal(2.0, aD.Value(), Tol);
        Assert.Equal(SupportType.IsOnEdge, aD.SupportTypeShape1(1));
        Assert.Equal(SupportType.IsOnEdge, aD.SupportTypeShape2(1));
        aD.ParOnEdgeS1(1, out double t1);
        aD.ParOnEdgeS2(1, out double t2);
        Assert.Equal(6.0, t1, Tol);
        Assert.Equal(5.0, t2, Tol);
        AssertPnt(new Point3D(1, 0, 0), aD.PointOnShape1(1));
        AssertPnt(new Point3D(1, 0, 2), aD.PointOnShape2(1));
    }

    [Fact]
    public void EdgeEdge_Parallel_GivesSeveralSolutions()
    {
        Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(0, 1, 0, 1, 1, 0);
        var aD = new DistanceShapeShape(e1, e2);
        Assert.True(aD.IsDone);
        Assert.Equal(1.0, aD.Value(), Tol);
        Assert.True(aD.SolutionCount >= 2);
        for (int i = 1; i <= aD.SolutionCount; i++)
        {
            Assert.Equal(1.0, aD.PointOnShape1(i).DistanceTo(aD.PointOnShape2(i)), Tol);
        }
    }

    [Fact]
    public void EquidistantEdges_TwoSolutions()
    {
        Compound aC = Builder.MakeCompound(E(-1, -5, -2, -1, 5, -2), E(1, -5, 2, 1, 5, 2));
        var aD = new DistanceShapeShape(V(0, 0, 0), aC);
        Assert.True(aD.IsDone);
        Assert.Equal(Math.Sqrt(5.0), aD.Value(), Tol);
        Assert.Equal(2, aD.SolutionCount);
        Assert.Equal(SupportType.IsOnEdge, aD.SupportTypeShape2(1));
        Assert.Equal(SupportType.IsOnEdge, aD.SupportTypeShape2(2));
        Assert.False(aD.SupportOnShape2(1).IsSame(aD.SupportOnShape2(2)));
    }

    [Fact]
    public void EdgeFace_AboveAndThrough()
    {
        Face aF = Square(0, 0, 0, 10);
        // a segment slanting above the face: its lower end is the nearest point (a vertex)
        var aD = new DistanceShapeShape(E(2, 2, 1, 5, 5, 4), aF);
        Assert.True(aD.IsDone);
        Assert.Equal(1.0, aD.Value(), Tol);
        AssertPnt(new Point3D(2, 2, 0), aD.PointOnShape2(1));

        // a segment through the face: distance 0, on the edge and in the face
        var aD2 = new DistanceShapeShape(E(3, 4, -1, 3, 4, 1), aF);
        Assert.True(aD2.IsDone);
        Assert.Equal(0.0, aD2.Value(), Tol);
        Assert.Equal(SupportType.IsOnEdge, aD2.SupportTypeShape1(1));
        Assert.Equal(SupportType.IsInFace, aD2.SupportTypeShape2(1));
        aD2.ParOnFaceS2(1, out double u, out double v);
        Assert.Equal(3.0, u, 1e-7);
        Assert.Equal(4.0, v, 1e-7);
    }

    [Fact]
    public void FaceFace_Parallel()
    {
        var aD = new DistanceShapeShape(Square(0, 0, 0, 1), Square(0.25, 0.25, 2, 0.5));
        Assert.True(aD.IsDone);
        Assert.Equal(2.0, aD.Value(), Tol);
        Assert.True(aD.SolutionCount >= 1);
        for (int i = 1; i <= aD.SolutionCount; i++)
        {
            Assert.Equal(2.0, aD.PointOnShape1(i).DistanceTo(aD.PointOnShape2(i)), Tol);
        }
    }

    // ------------------------------------------------------------------ solids

    [Fact]
    public void Solid_InnerSolution()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        Vertex anInside = V(5, 5, 5);
        var aD = new DistanceShapeShape(aBox, anInside);
        Assert.True(aD.IsDone);
        Assert.True(aD.InnerSolution());
        Assert.Equal(0.0, aD.Value());
        Assert.Equal(1, aD.SolutionCount);
        Assert.True(aD.SupportOnShape1(1).IsSame(anInside));
        Assert.True(aD.SupportOnShape2(1).IsSame(anInside));

        var aOut = new DistanceShapeShape(V(5, 5, 12), aBox);
        Assert.True(aOut.IsDone);
        Assert.False(aOut.InnerSolution());
        Assert.Equal(2.0, aOut.Value(), Tol);
        Assert.Equal(SupportType.IsInFace, aOut.SupportTypeShape2(1));
    }

    [Fact]
    public void TwoBoxes_SingleAndMultiThread_Agree_AndInputsUnchanged()
    {
        Solid aBox1 = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Shape aBox2 = TestShapes.Frozen(TestShapes.Box(1, 1, 1)).Moved(new Placement(
            new Transform3D().WithTranslation(new Vector3D(3, 0.5, 1))));
        var aSnap1 = ShapeSnapshot.Of(aBox1);
        var aSnap2 = ShapeSnapshot.Of(aBox2);

        var aSingle = new DistanceShapeShape(aBox1, aBox2);
        var aMulti = new DistanceShapeShape();
        aMulti.SetMultiThread(true);
        Assert.True(aMulti.IsMultiThread);
        aMulti.LoadS1(aBox1);
        aMulti.LoadS2(aBox2);
        Assert.True(aMulti.Perform());

        Assert.Equal(2.0, aSingle.Value(), Tol);
        Assert.Equal(aSingle.Value(), aMulti.Value());
        Assert.Equal(aSingle.SolutionCount, aMulti.SolutionCount);
        for (int i = 1; i <= aSingle.SolutionCount; i++)
        {
            Assert.Equal(aSingle.PointOnShape1(i), aMulti.PointOnShape1(i));
            Assert.Equal(aSingle.PointOnShape2(i), aMulti.PointOnShape2(i));
            Assert.Equal(2.0, aSingle.PointOnShape1(i).DistanceTo(aSingle.PointOnShape2(i)), Tol);
        }

        aSnap1.AssertUnchanged(aBox1);
        aSnap2.AssertUnchanged(aBox2);
    }

    /// <summary>
    /// The loops moved onto OSD_Parallel (decision 487): a multi-threaded run done by the serial fallback (no threads,
    /// as in WebAssembly) gives the single-threaded run's solutions, in the same order.
    /// </summary>
    [Fact]
    public void SerialFallback_GivesTheSingleThreadSolutions()
    {
        Shape aBox1 = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Shape aBox2 = TestShapes.Frozen(TestShapes.Box(1, 1, 1)).Moved(new Placement(
            new Transform3D().WithTranslation(new Vector3D(3, 0.5, 1))));
        Shape aBox3 = TestShapes.Frozen(TestShapes.Box(2, 2, 2)).Moved(new Placement(
            new Transform3D().WithTranslation(new Vector3D(-1, -1, 4))));
        Shape aCompound = Builder.MakeCompound(aBox2, aBox3);
        foreach ((Shape aS1, Shape aS2) in new[] { (aBox1, aBox2), (aBox1, aCompound), (aCompound, aBox1) })
        {
            var aSingle = new DistanceShapeShape(aS1, aS2);
            var aFallback = new DistanceShapeShape();
            aFallback.SetMultiThread(true);
            aFallback.LoadS1(aS1);
            aFallback.LoadS2(aS2);
            using (global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope())
            {
                Assert.True(aFallback.Perform());
            }

            Assert.Equal(aSingle.Value(), aFallback.Value());
            Assert.Equal(aSingle.SolutionCount, aFallback.SolutionCount);
            for (int i = 1; i <= aSingle.SolutionCount; i++)
            {
                Assert.Equal(aSingle.PointOnShape1(i), aFallback.PointOnShape1(i));
                Assert.Equal(aSingle.PointOnShape2(i), aFallback.PointOnShape2(i));
                Assert.True(aSingle.SupportOnShape1(i).IsEqual(aFallback.SupportOnShape1(i)));
                Assert.True(aSingle.SupportOnShape2(i).IsEqual(aFallback.SupportOnShape2(i)));
            }
        }
    }

    [Fact]
    public void LocatedShapes()
    {
        var aT = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 2);
        Shape anEdge = E(0, 0, 0, 10, 0, 0).Moved(new Placement(aT)); // now along +Y
        var aD = new DistanceShapeShape(anEdge, V(3, 4, 0));
        Assert.Equal(3.0, aD.Value(), Tol);
        AssertPnt(new Point3D(0, 4, 0), aD.PointOnShape1(1));
        aD.ParOnEdgeS1(1, out double t);
        Assert.Equal(4.0, t, Tol);
        Assert.True(aD.SupportOnShape1(1).IsEqual(anEdge));
    }

    [Fact]
    public void Deflection_KeepsNearSolutions()
    {
        // two vertices at distances 1 and 1.05 from the origin
        Compound aC = Builder.MakeCompound(V(1, 0, 0), V(0, 1.05, 0));
        var aTight = new DistanceShapeShape(V(0, 0, 0), aC);
        Assert.Equal(1, aTight.SolutionCount);
        var aLoose = new DistanceShapeShape(V(0, 0, 0), aC, 0.1);
        Assert.Equal(2, aLoose.SolutionCount);
        Assert.Equal(1.0, aLoose.Value());
    }

    [Fact]
    public void InfiniteEdge_IsTrimmed_AndTheSupportIsFrozen()
    {
        var aLine = new LineCurve3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0));
        Edge anInfinite = new MakeEdge(aLine).Edge();
        Edge aFinite = E(3, -1, 0, 3, 1, 0);
        var aD = new DistanceShapeShape(anInfinite, aFinite);
        Assert.True(aD.IsDone);
        Assert.Equal(5.0, aD.Value(), Tol);
        AssertPnt(new Point3D(3, 0, 5), aD.PointOnShape1(1));
        Shape aSupport = aD.SupportOnShape1(1);
        Assert.False(aSupport.IsSame(anInfinite)); // a trimmed copy, built by DistanceSS
        Assert.True(aSupport.TShape()!.IsFrozen);
    }

    [Fact]
    public void Errors()
    {
        var anEmpty = new DistanceShapeShape();
        Assert.False(anEmpty.Perform());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anEmpty.Value());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anEmpty.SupportOnShape1(1));

        var aD = new DistanceShapeShape(V(0, 0, 0), V(1, 0, 0));
        var aText = new StringWriter();
        aD.Dump(aText);
        Assert.StartsWith("the distance  value is :  1\nthe number of solutions is :1\n", aText.ToString());
    }

    // ------------------------------------------------------------------ Ext tools and DistanceSS

    [Fact]
    public void ExtTools()
    {
        var aPC = new ExtPC(V(5, 1, 0), E(0, 0, 0, 10, 0, 0));
        Assert.True(aPC.IsDone);
        Assert.Equal(1, aPC.ExtCount);
        Assert.Equal(1.0, aPC.SquareDistance(1), Tol);
        Assert.Equal(5.0, aPC.Parameter(1), Tol);
        Assert.True(aPC.IsMin(1));

        var aPF = new ExtPF(V(1, 2, 3), Square(0, 0, 0, 5));
        Assert.True(aPF.IsDone);
        Assert.Equal(9.0, aPF.SquareDistance(1), Tol);
        aPF.Parameter(1, out double u, out double v);
        Assert.Equal(1.0, u, Tol);
        Assert.Equal(2.0, v, Tol);

        var aCC = new DistanceExtCC(E(0, 0, 0, 1, 0, 0), E(0, 1, 0, 1, 1, 0));
        Assert.True(aCC.IsDone);
        Assert.True(aCC.IsParallel);

        var aCF = new ExtCF(E(1, 1, -1, 1, 1, 1), Square(0, 0, 0, 5));
        Assert.True(aCF.IsDone);
        Assert.Equal(1, aCF.ExtCount);
        Assert.Equal(0.0, aCF.SquareDistance(1), Tol);

        var aFF = new ExtFF(Square(0, 0, 0, 1), Square(0, 0, 1, 1));
        Assert.True(aFF.IsDone);
        Assert.True(aFF.IsParallel);
        Assert.Equal(1.0, aFF.SquareDistance(1), Tol);
    }

    [Fact]
    public void DistanceSS_SkipsPairsBeyondTheReference()
    {
        Vertex a = V(0, 0, 0), b = V(3, 0, 0);
        var aBox1 = new Box();
        var aBox2 = new Box();
        Bounds.Add(a, ref aBox1);
        Bounds.Add(b, ref aBox2);
        var aFar = new DistanceSS(a, b, aBox1, aBox2, 1.0);
        Assert.False(aFar.IsDone);
        var aNear = new DistanceSS(a, b, aBox1, aBox2, 10.0);
        Assert.True(aNear.IsDone);
        Assert.Equal(3.0, aNear.DistValue());
        Assert.Single(aNear.Seq1Value());
        Assert.Equal(SupportType.IsVertex, aNear.Seq2Value()[0].SupportKind());
    }

    // ------------------------------------------------------------------ mesh tools

    // The two triangles of a unit square at height z, as a face without surface.
    private static Face MeshSquare(double z, double x0 = 0)
        => TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new Point3D(x0, 0, z), new Point3D(x0 + 1, 0, z), new Point3D(x0 + 1, 1, z), new Point3D(x0, 1, z)],
            (1, 2, 3), (1, 3, 4)));

    [Fact]
    public void TriangleSet_Indices()
    {
        var aShapes = new List<Shape>();
        aShapes.Add(MeshSquare(0));
        aShapes.Add(MeshSquare(1));
        aShapes.Add(TestShapes.Frozen(BRepGPropTestShapes.PolygonEdge(new Point3D(0, 0, 2), new Point3D(1, 0, 2), new Point3D(2, 0, 2))));
        var aSet = new TriangleSet(aShapes);
        Assert.Equal(6, aSet.Size()); // 2 + 2 triangles, 2 segments
        Assert.Equal(11, aSet.GetVertices().Size());
        Assert.Equal(1, aSet.GetShapeIDOfVtx(4));
        Assert.Equal(0, aSet.GetVtxIdxInShape(4));
        Assert.Equal(2, aSet.GetVtxIdxInShape(10));
        int aSum = 0;
        for (int i = 0; i < aSet.Size(); i++)
        {
            aSum += aSet.GetFaceID(i);
            int aTrg = aSet.GetTrgIdxInShape(i);
            Assert.InRange(aTrg, 0, 1);
        }
        Assert.Equal(0 + 0 + 1 + 1 + 2 + 2, aSum);
        Assert.NotNull(aSet.BVH());
    }

    [Fact]
    public void ShapeProximity_Overlap()
    {
        Face a = MeshSquare(0), b = MeshSquare(0.3);
        var aNear = new ShapeProximity(a, b, 0.5);
        aNear.Perform();
        Assert.True(aNear.IsDone);
        Assert.True(aNear.OverlapSubShapes1().ContainsKey(0));
        Assert.True(aNear.OverlapSubShapes1().GetValue(0).Contains(0));
        Assert.True(aNear.OverlapSubShapes2().GetValue(0).Contains(0));
        Assert.True(aNear.GetSubShape1(0).IsSame(a));

        var aFar = new ShapeProximity(a, b, 0.2);
        aFar.Perform();
        Assert.False(aFar.IsDone);
        Assert.Empty(aFar.OverlapSubShapes1());

        // tolerance zero: crossing triangles
        Face aVertical = TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new Point3D(0.5, -1, -1), new Point3D(0.5, 2, -1), new Point3D(0.5, 0.5, 1)], (1, 2, 3)));
        var aCross = new ShapeProximity(a, aVertical, 0.0);
        aCross.Perform();
        Assert.True(aCross.OverlapSubShapes1().ContainsKey(0));
    }

    [Fact]
    public void ShapeProximity_Value()
    {
        // two boxes' top faces, exactly meshed, one unit apart
        Solid aBox1 = TestShapes.Box(1, 1, 1);
        BRepGPropTestShapes.AttachQuadMeshes(aBox1);
        TestShapes.Frozen(aBox1);
        Face aTop = TopFace(aBox1);
        Face aTop2 = ShapeCasts.Face(aTop.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(0, 0, 1)))));

        var aProx = new ShapeProximity(aTop, aTop2);
        aProx.Perform();
        Assert.True(aProx.IsDone);
        Assert.Equal(1.0, aProx.Proximity(), Tol);
        Assert.Equal(1.0, aProx.ProximityPoint1().DistanceTo(aProx.ProximityPoint2()), Tol);
    }

    private static Face TopFace(in Shape theBox)
    {
        foreach (Face aF in BRepGPropTestShapes.Faces(theBox))
        {
            var aBox = new Box();
            Bounds.Add(aF, ref aBox);
            aBox.Get(out _, out _, out double aZmin, out _, out _, out _);
            if (aZmin > 0.5)
            {
                return aF;
            }
        }
        throw new InvalidOperationException("no top face");
    }

    [Fact]
    public void SelfIntersection()
    {
        Face a = MeshSquare(0);
        Face aVertical = TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new Point3D(0.5, -1, -1), new Point3D(0.5, 2, -1), new Point3D(0.5, 0.5, 1)], (1, 2, 3)));
        var aSelf = new SelfIntersection(Builder.MakeCompound(a, aVertical));
        aSelf.Perform();
        Assert.True(aSelf.IsDone);
        Assert.True(aSelf.OverlapElements().ContainsKey(0) || aSelf.OverlapElements().ContainsKey(1));
        Assert.True(aSelf.GetSubShape(1).IsSame(aVertical));

        // two squares side by side, sharing an edge: regular, no self-intersection
        var aClean = new SelfIntersection(Builder.MakeCompound(MeshSquare(0), MeshSquare(0, 1)));
        aClean.Perform();
        Assert.Empty(aClean.OverlapElements());
    }

    [Fact]
    public void Poly_Distance()
    {
        Point3D p1 = new(), p2 = new();
        Assert.True(DistancePoly.Distance(MeshSquare(0), MeshSquare(2, 3), ref p1, ref p2, out double aDist));
        Assert.Equal(Math.Sqrt(8.0), aDist, Tol);
        AssertPnt(new Point3D(1, 0, 0), p1);
        AssertPnt(new Point3D(3, 0, 2), p2);
        Assert.False(DistancePoly.Distance(Square(0, 0, 0, 1), MeshSquare(2), ref p1, ref p2, out _));
    }

    [Fact]
    public void Orientations_DoNotMatter()
    {
        Face aF = Square(0, 0, 0, 10);
        var aD = new DistanceShapeShape(aF.Reversed(), V(2, 3, -7));
        Assert.Equal(7.0, aD.Value(), Tol);
        Assert.Equal(Reversed, aD.SupportOnShape1(1).Orientation);
        Assert.Equal(ShapeType.Face, aD.SupportOnShape1(1).Type);
    }

    // A segment edge with a 3D polygon of the given number of evenly spaced nodes.
    private static Edge PolygonSegment(in Point3D theP1, in Point3D theP2, int theNbNodes)
    {
        Edge anEdge = new MakeEdge(theP1, theP2).InternalEdge();
        var aNodes = new NCollection_Array1<Point3D>(1, theNbNodes);
        for (int i = 1; i <= theNbNodes; i++)
        {
            double t = (double)(i - 1) / (theNbNodes - 1);
            aNodes[i] = new Point3D(theP1.XYZ() * (1 - t) + theP2.XYZ() * t);
        }
        new BrepBuilder().UpdateEdge(anEdge, new Polygon3D(aNodes));
        return TestShapes.Frozen(anEdge);
    }

    // A unit square face at height z (plane surface) with an n x n grid triangulation.
    private static Face GridSquare(double z, int n)
    {
        Face aF = new MakeFace(new Plane3D(new Point3D(0, 0, z), new Direction3D(0, 0, 1)), 0, 1, 0, 1).InternalFace();
        var aTri = new Triangulation((n + 1) * (n + 1), 2 * n * n, false);
        for (int j = 0; j <= n; j++)
        {
            for (int i = 0; i <= n; i++)
            {
                aTri.SetNode(j * (n + 1) + i + 1, new Point3D((double)i / n, (double)j / n, z));
            }
        }
        int k = 1;
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                int a = j * (n + 1) + i + 1, b = a + 1, c = a + n + 2, d = a + n + 1;
                aTri.SetTriangle(k++, new Triangle1(a, b, c));
                aTri.SetTriangle(k++, new Triangle1(a, c, d));
            }
        }
        new BrepBuilder().UpdateFace(aF, aTri);
        return TestShapes.Frozen(aF);
    }

    [Fact]
    public void ProximityValue_EdgesWithRefinement()
    {
        // the second edge's polygon is coarser: it is refined to the first's step
        Edge e1 = PolygonSegment(new Point3D(0, 0, 0), new Point3D(10, 0, 0), 11);
        Edge e2 = PolygonSegment(new Point3D(0, 1, 0), new Point3D(10, 1, 0), 3);
        var aProx = new ShapeProximity(e1, e2);
        aProx.Perform();
        Assert.True(aProx.IsDone);
        Assert.Equal(1.0, aProx.Proximity(), Tol);
        Assert.Equal(1.0, aProx.ProximityPoint1().DistanceTo(aProx.ProximityPoint2()), Tol);
        Assert.NotEqual(ProximityDistTool.ProxPnt_Status.ProxPnt_Status_UNKNOWN, aProx.ProxPntStatus1());

        // more samples than nodes on both
        var aSampled = new ShapeProximity(e1, e2);
        aSampled.SetNbSamples1(40);
        aSampled.SetNbSamples2(40);
        aSampled.Perform();
        Assert.True(aSampled.IsDone);
        Assert.Equal(1.0, aSampled.Proximity(), Tol);
    }

    [Fact]
    public void ProximityValue_FacesWithRefinement()
    {
        Face aCoarse = GridSquare(0, 1), aFine = GridSquare(0.5, 4);
        var aProx = new ShapeProximity(aCoarse, aFine);
        aProx.Perform();
        Assert.True(aProx.IsDone);
        Assert.Equal(0.5, aProx.Proximity(), Tol);

        var aTool = new ProximityValueTool();
        var aList1 = new List<Shape>();
        aList1.Add(aCoarse);
        var aList2 = new List<Shape>();
        aList2.Add(aFine);
        aTool.LoadTriangleSets(new TriangleSet(aList1), new TriangleSet(aList2));
        aTool.LoadShapeLists(aList1, aList2);
        double aTol = 0;
        aTool.Perform(ref aTol);
        Assert.True(aTool.IsDone);
        Assert.Equal(aTool.Distance(), aTol);
        Assert.Equal(0.5, aTol, Tol);
        aTool.ProximityPoints(out Point3D p1, out Point3D p2);
        Assert.Equal(0.5, p1.DistanceTo(p2), Tol);
    }

    [Fact]
    public void SelfIntersection_WithTolerance()
    {
        // two parallel squares 0.1 apart: no intersection, but closer than the tolerance
        Compound aC = Builder.MakeCompound(MeshSquare(0), MeshSquare(0.1));
        var anExact = new SelfIntersection(aC);
        anExact.Perform();
        Assert.Empty(anExact.OverlapElements());
        var aTolerant = new SelfIntersection(aC, 0.2);
        Assert.Equal(0.2, aTolerant.Tolerance());
        aTolerant.Perform();
        Assert.True(aTolerant.IsDone);
        Assert.True(aTolerant.OverlapElements().ContainsKey(0) || aTolerant.OverlapElements().ContainsKey(1));
        Assert.NotNull(aTolerant.ElementSet());
    }

    // ------------------------------------------------------------------ OCCT issues, pinned (occt-issues.md)

    private static Face MeshTriangle(double z)
        => TestShapes.Frozen(BRepGPropTestShapes.MeshFace([new Point3D(0, 0, z), new Point3D(1, 0, z), new Point3D(0, 1, z)], (1, 2, 3)));

    [Fact]
    public void OcctIssue_TriangleSet_InitAgainKeepsTheFirstShapesVertexIds()
    {
        var aFirst = new List<Shape>();
        aFirst.Add(MeshTriangle(0)); // 3 nodes
        aFirst.Add(MeshSquare(1)); // 4 nodes
        var aSet = new TriangleSet(aFirst);
        Assert.Equal(1, aSet.GetShapeIDOfVtx(3));

        var aSecond = new List<Shape>();
        aSecond.Add(MeshSquare(1));
        aSecond.Add(MeshTriangle(0));
        Assert.True(aSet.Init(aSecond));
        Assert.Equal(7, aSet.GetVertices().Size());
        // vertex 3 is the square's (shape 0), but OCCT reads the first Init's entry
        Assert.Equal(1, aSet.GetShapeIDOfVtx(3));
    }

    [Fact]
    public void OcctIssue_TriangleSet_InitIsTrueWithoutMeshes()
    {
        var aShapes = new List<Shape>();
        aShapes.Add(Square(0, 0, 0, 1)); // no triangulation
        aShapes.Add(MeshSquare(1));
        var aSet = new TriangleSet();
        Assert.True(aSet.Init(aShapes));
        Assert.Equal(0, aSet.Size()); // stopped at the first shape
    }

    [Fact]
    public void OcctIssue_OverlapTool_KeepsOldPairs()
    {
        Face a = MeshSquare(0), b = MeshSquare(0.3);
        var aProx = new ShapeProximity(a, b, 0.5);
        aProx.Perform();
        Assert.True(aProx.IsDone);
        Assert.True(aProx.LoadShape2(MeshSquare(5)));
        aProx.Perform();
        Assert.False(aProx.IsDone);
        Assert.True(aProx.OverlapSubShapes1().ContainsKey(0)); // the first Perform's pair is still there
    }

    [Fact]
    public void OcctIssue_ShapeProximity_ValueBecomesTheTolerance()
    {
        Face aCoarse = GridSquare(0, 1), aFine = GridSquare(0.5, 2);
        var aProx = new ShapeProximity(aCoarse, aFine);
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aProx.Tolerance());
        aProx.Perform();
        Assert.Equal(0.5, aProx.Tolerance(), Tol);
    }
}
