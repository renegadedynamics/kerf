// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using OcctNet.ModelingData.Tests;
using static Occt.BRepExtrema_SupportType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known-answer tests of BRepExtrema: the distance between vertices, edges, faces and solids (supports, parameters,
/// inner solutions, parallel cases, several solutions, located shapes, multi-threading, deflection), the Ext*
/// tools, and the mesh-based tools (triangle sets, overlap, self-intersection, proximity, BRepExtrema_Poly). Inputs
/// are frozen as callers hold them, and checked unchanged. The makers are BRepLib's (BRepBuilderAPI wraps them).
/// </summary>
public class BRepExtremaTests
{
    private const double Tol = 1.0e-9;

    private static TopoDS_Vertex V(double x, double y, double z) => new BRepLib_MakeVertex(new gp_Pnt(x, y, z)).Vertex();

    private static TopoDS_Edge E(double x1, double y1, double z1, double x2, double y2, double z2)
        => new BRepLib_MakeEdge(new gp_Pnt(x1, y1, z1), new gp_Pnt(x2, y2, z2)).Edge();

    // A square [x0, x0 + s] x [y0, y0 + s] in the plane z = z0, normal +Z.
    private static TopoDS_Face Square(double x0, double y0, double z0, double s)
        => new BRepLib_MakeFace(new gp_Pln(new gp_Pnt(x0, y0, z0), new gp_Dir(0, 0, 1)), 0, s, 0, s).Face();

    private static void AssertPnt(in gp_Pnt theExpected, in gp_Pnt theActual, double theTol = Tol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    // ------------------------------------------------------------------ vertices and edges

    [Fact]
    public void VertexVertex()
    {
        TopoDS_Vertex a = V(0, 0, 0), b = V(3, 4, 0);
        var aD = new BRepExtrema_DistShapeShape(a, b);
        Assert.True(aD.IsDone());
        Assert.Equal(1, aD.NbSolution());
        Assert.Equal(5.0, aD.Value());
        Assert.False(aD.InnerSolution());
        Assert.Equal(BRepExtrema_IsVertex, aD.SupportTypeShape1(1));
        Assert.Equal(BRepExtrema_IsVertex, aD.SupportTypeShape2(1));
        Assert.True(aD.SupportOnShape1(1).IsSame(a));
        Assert.True(aD.SupportOnShape2(1).IsSame(b));
        AssertPnt(new gp_Pnt(3, 4, 0), aD.PointOnShape2(1));
        Assert.ThrowsAny<BRepExtrema_UnCompatibleShape>(() => aD.ParOnEdgeS1(1, out _));
        Assert.ThrowsAny<BRepExtrema_UnCompatibleShape>(() => aD.ParOnFaceS2(1, out _, out _));
    }

    [Fact]
    public void VertexEdge_FootInside_AndBeyondTheEnd()
    {
        TopoDS_Edge anEdge = E(0, 0, 0, 10, 0, 0);
        var aD = new BRepExtrema_DistShapeShape(V(4, 3, 0), anEdge);
        Assert.True(aD.IsDone());
        Assert.Equal(1, aD.NbSolution());
        Assert.Equal(3.0, aD.Value(), Tol);
        Assert.Equal(BRepExtrema_IsVertex, aD.SupportTypeShape1(1));
        Assert.Equal(BRepExtrema_IsOnEdge, aD.SupportTypeShape2(1));
        Assert.True(aD.SupportOnShape2(1).IsSame(anEdge));
        aD.ParOnEdgeS2(1, out double t);
        Assert.Equal(4.0, t, Tol);
        AssertPnt(new gp_Pnt(4, 0, 0), aD.PointOnShape2(1));

        // beyond the end: the nearest point is the edge's vertex
        var aD2 = new BRepExtrema_DistShapeShape(anEdge, V(13, 4, 0));
        Assert.Equal(5.0, aD2.Value(), Tol);
        Assert.Equal(BRepExtrema_IsVertex, aD2.SupportTypeShape1(1));
        AssertPnt(new gp_Pnt(10, 0, 0), aD2.PointOnShape1(1));
    }

    [Fact]
    public void VertexFace_InFace()
    {
        TopoDS_Face aF = Square(0, 0, 0, 10);
        var aD = new BRepExtrema_DistShapeShape(aF, V(2, 3, 7));
        Assert.True(aD.IsDone());
        Assert.Equal(7.0, aD.Value(), Tol);
        Assert.Equal(1, aD.NbSolution());
        Assert.Equal(BRepExtrema_IsInFace, aD.SupportTypeShape1(1));
        Assert.True(aD.SupportOnShape1(1).IsSame(aF));
        aD.ParOnFaceS1(1, out double u, out double v);
        Assert.Equal(2.0, u, Tol);
        Assert.Equal(3.0, v, Tol);
        AssertPnt(new gp_Pnt(2, 3, 0), aD.PointOnShape1(1));
    }

    [Fact]
    public void EdgeEdge_Skew()
    {
        TopoDS_Edge e1 = E(-5, 0, 0, 5, 0, 0), e2 = E(1, -5, 2, 1, 5, 2);
        var aD = new BRepExtrema_DistShapeShape(e1, e2);
        Assert.True(aD.IsDone());
        Assert.Equal(1, aD.NbSolution());
        Assert.Equal(2.0, aD.Value(), Tol);
        Assert.Equal(BRepExtrema_IsOnEdge, aD.SupportTypeShape1(1));
        Assert.Equal(BRepExtrema_IsOnEdge, aD.SupportTypeShape2(1));
        aD.ParOnEdgeS1(1, out double t1);
        aD.ParOnEdgeS2(1, out double t2);
        Assert.Equal(6.0, t1, Tol);
        Assert.Equal(5.0, t2, Tol);
        AssertPnt(new gp_Pnt(1, 0, 0), aD.PointOnShape1(1));
        AssertPnt(new gp_Pnt(1, 0, 2), aD.PointOnShape2(1));
    }

    [Fact]
    public void EdgeEdge_Parallel_GivesSeveralSolutions()
    {
        TopoDS_Edge e1 = E(0, 0, 0, 1, 0, 0), e2 = E(0, 1, 0, 1, 1, 0);
        var aD = new BRepExtrema_DistShapeShape(e1, e2);
        Assert.True(aD.IsDone());
        Assert.Equal(1.0, aD.Value(), Tol);
        Assert.True(aD.NbSolution() >= 2);
        for (int i = 1; i <= aD.NbSolution(); i++)
        {
            Assert.Equal(1.0, aD.PointOnShape1(i).Distance(aD.PointOnShape2(i)), Tol);
        }
    }

    [Fact]
    public void EquidistantEdges_TwoSolutions()
    {
        TopoDS_Compound aC = TopoDS_Builder.MakeCompound(E(-1, -5, -2, -1, 5, -2), E(1, -5, 2, 1, 5, 2));
        var aD = new BRepExtrema_DistShapeShape(V(0, 0, 0), aC);
        Assert.True(aD.IsDone());
        Assert.Equal(Math.Sqrt(5.0), aD.Value(), Tol);
        Assert.Equal(2, aD.NbSolution());
        Assert.Equal(BRepExtrema_IsOnEdge, aD.SupportTypeShape2(1));
        Assert.Equal(BRepExtrema_IsOnEdge, aD.SupportTypeShape2(2));
        Assert.False(aD.SupportOnShape2(1).IsSame(aD.SupportOnShape2(2)));
    }

    [Fact]
    public void EdgeFace_AboveAndThrough()
    {
        TopoDS_Face aF = Square(0, 0, 0, 10);
        // a segment slanting above the face: its lower end is the nearest point (a vertex)
        var aD = new BRepExtrema_DistShapeShape(E(2, 2, 1, 5, 5, 4), aF);
        Assert.True(aD.IsDone());
        Assert.Equal(1.0, aD.Value(), Tol);
        AssertPnt(new gp_Pnt(2, 2, 0), aD.PointOnShape2(1));

        // a segment through the face: distance 0, on the edge and in the face
        var aD2 = new BRepExtrema_DistShapeShape(E(3, 4, -1, 3, 4, 1), aF);
        Assert.True(aD2.IsDone());
        Assert.Equal(0.0, aD2.Value(), Tol);
        Assert.Equal(BRepExtrema_IsOnEdge, aD2.SupportTypeShape1(1));
        Assert.Equal(BRepExtrema_IsInFace, aD2.SupportTypeShape2(1));
        aD2.ParOnFaceS2(1, out double u, out double v);
        Assert.Equal(3.0, u, 1e-7);
        Assert.Equal(4.0, v, 1e-7);
    }

    [Fact]
    public void FaceFace_Parallel()
    {
        var aD = new BRepExtrema_DistShapeShape(Square(0, 0, 0, 1), Square(0.25, 0.25, 2, 0.5));
        Assert.True(aD.IsDone());
        Assert.Equal(2.0, aD.Value(), Tol);
        Assert.True(aD.NbSolution() >= 1);
        for (int i = 1; i <= aD.NbSolution(); i++)
        {
            Assert.Equal(2.0, aD.PointOnShape1(i).Distance(aD.PointOnShape2(i)), Tol);
        }
    }

    // ------------------------------------------------------------------ solids

    [Fact]
    public void Solid_InnerSolution()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        TopoDS_Vertex anInside = V(5, 5, 5);
        var aD = new BRepExtrema_DistShapeShape(aBox, anInside);
        Assert.True(aD.IsDone());
        Assert.True(aD.InnerSolution());
        Assert.Equal(0.0, aD.Value());
        Assert.Equal(1, aD.NbSolution());
        Assert.True(aD.SupportOnShape1(1).IsSame(anInside));
        Assert.True(aD.SupportOnShape2(1).IsSame(anInside));

        var aOut = new BRepExtrema_DistShapeShape(V(5, 5, 12), aBox);
        Assert.True(aOut.IsDone());
        Assert.False(aOut.InnerSolution());
        Assert.Equal(2.0, aOut.Value(), Tol);
        Assert.Equal(BRepExtrema_IsInFace, aOut.SupportTypeShape2(1));
    }

    [Fact]
    public void TwoBoxes_SingleAndMultiThread_Agree_AndInputsUnchanged()
    {
        TopoDS_Solid aBox1 = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        TopoDS_Shape aBox2 = TestShapes.Frozen(TestShapes.Box(1, 1, 1)).Moved(new TopLoc_Location(
            new gp_Trsf().WithTranslation(new gp_Vec(3, 0.5, 1))));
        var aSnap1 = ShapeSnapshot.Of(aBox1);
        var aSnap2 = ShapeSnapshot.Of(aBox2);

        var aSingle = new BRepExtrema_DistShapeShape(aBox1, aBox2);
        var aMulti = new BRepExtrema_DistShapeShape();
        aMulti.SetMultiThread(true);
        Assert.True(aMulti.IsMultiThread());
        aMulti.LoadS1(aBox1);
        aMulti.LoadS2(aBox2);
        Assert.True(aMulti.Perform());

        Assert.Equal(2.0, aSingle.Value(), Tol);
        Assert.Equal(aSingle.Value(), aMulti.Value());
        Assert.Equal(aSingle.NbSolution(), aMulti.NbSolution());
        for (int i = 1; i <= aSingle.NbSolution(); i++)
        {
            Assert.Equal(aSingle.PointOnShape1(i), aMulti.PointOnShape1(i));
            Assert.Equal(aSingle.PointOnShape2(i), aMulti.PointOnShape2(i));
            Assert.Equal(2.0, aSingle.PointOnShape1(i).Distance(aSingle.PointOnShape2(i)), Tol);
        }

        aSnap1.AssertUnchanged(aBox1);
        aSnap2.AssertUnchanged(aBox2);
    }

    [Fact]
    public void LocatedShapes()
    {
        var aT = new gp_Trsf().WithRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), Math.PI / 2);
        TopoDS_Shape anEdge = E(0, 0, 0, 10, 0, 0).Moved(new TopLoc_Location(aT)); // now along +Y
        var aD = new BRepExtrema_DistShapeShape(anEdge, V(3, 4, 0));
        Assert.Equal(3.0, aD.Value(), Tol);
        AssertPnt(new gp_Pnt(0, 4, 0), aD.PointOnShape1(1));
        aD.ParOnEdgeS1(1, out double t);
        Assert.Equal(4.0, t, Tol);
        Assert.True(aD.SupportOnShape1(1).IsEqual(anEdge));
    }

    [Fact]
    public void Deflection_KeepsNearSolutions()
    {
        // two vertices at distances 1 and 1.05 from the origin
        TopoDS_Compound aC = TopoDS_Builder.MakeCompound(V(1, 0, 0), V(0, 1.05, 0));
        var aTight = new BRepExtrema_DistShapeShape(V(0, 0, 0), aC);
        Assert.Equal(1, aTight.NbSolution());
        var aLoose = new BRepExtrema_DistShapeShape(V(0, 0, 0), aC, 0.1);
        Assert.Equal(2, aLoose.NbSolution());
        Assert.Equal(1.0, aLoose.Value());
    }

    [Fact]
    public void InfiniteEdge_IsTrimmed_AndTheSupportIsFrozen()
    {
        var aLine = new Geom_Line(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0));
        TopoDS_Edge anInfinite = new BRepLib_MakeEdge(aLine).Edge();
        TopoDS_Edge aFinite = E(3, -1, 0, 3, 1, 0);
        var aD = new BRepExtrema_DistShapeShape(anInfinite, aFinite);
        Assert.True(aD.IsDone());
        Assert.Equal(5.0, aD.Value(), Tol);
        AssertPnt(new gp_Pnt(3, 0, 5), aD.PointOnShape1(1));
        TopoDS_Shape aSupport = aD.SupportOnShape1(1);
        Assert.False(aSupport.IsSame(anInfinite)); // a trimmed copy, built by DistanceSS
        Assert.True(aSupport.TShape()!.IsFrozen());
    }

    [Fact]
    public void Errors()
    {
        var anEmpty = new BRepExtrema_DistShapeShape();
        Assert.False(anEmpty.Perform());
        Assert.ThrowsAny<StdFail_NotDone>(() => anEmpty.Value());
        Assert.ThrowsAny<StdFail_NotDone>(() => anEmpty.SupportOnShape1(1));

        var aD = new BRepExtrema_DistShapeShape(V(0, 0, 0), V(1, 0, 0));
        var aText = new StringWriter();
        aD.Dump(aText);
        Assert.StartsWith("the distance  value is :  1\nthe number of solutions is :1\n", aText.ToString());
    }

    // ------------------------------------------------------------------ Ext tools and DistanceSS

    [Fact]
    public void ExtTools()
    {
        var aPC = new BRepExtrema_ExtPC(V(5, 1, 0), E(0, 0, 0, 10, 0, 0));
        Assert.True(aPC.IsDone());
        Assert.Equal(1, aPC.NbExt());
        Assert.Equal(1.0, aPC.SquareDistance(1), Tol);
        Assert.Equal(5.0, aPC.Parameter(1), Tol);
        Assert.True(aPC.IsMin(1));

        var aPF = new BRepExtrema_ExtPF(V(1, 2, 3), Square(0, 0, 0, 5));
        Assert.True(aPF.IsDone());
        Assert.Equal(9.0, aPF.SquareDistance(1), Tol);
        aPF.Parameter(1, out double u, out double v);
        Assert.Equal(1.0, u, Tol);
        Assert.Equal(2.0, v, Tol);

        var aCC = new BRepExtrema_ExtCC(E(0, 0, 0, 1, 0, 0), E(0, 1, 0, 1, 1, 0));
        Assert.True(aCC.IsDone());
        Assert.True(aCC.IsParallel());

        var aCF = new BRepExtrema_ExtCF(E(1, 1, -1, 1, 1, 1), Square(0, 0, 0, 5));
        Assert.True(aCF.IsDone());
        Assert.Equal(1, aCF.NbExt());
        Assert.Equal(0.0, aCF.SquareDistance(1), Tol);

        var aFF = new BRepExtrema_ExtFF(Square(0, 0, 0, 1), Square(0, 0, 1, 1));
        Assert.True(aFF.IsDone());
        Assert.True(aFF.IsParallel());
        Assert.Equal(1.0, aFF.SquareDistance(1), Tol);
    }

    [Fact]
    public void DistanceSS_SkipsPairsBeyondTheReference()
    {
        TopoDS_Vertex a = V(0, 0, 0), b = V(3, 0, 0);
        var aBox1 = new Bnd_Box();
        var aBox2 = new Bnd_Box();
        BRepBndLib.Add(a, ref aBox1);
        BRepBndLib.Add(b, ref aBox2);
        var aFar = new BRepExtrema_DistanceSS(a, b, aBox1, aBox2, 1.0);
        Assert.False(aFar.IsDone());
        var aNear = new BRepExtrema_DistanceSS(a, b, aBox1, aBox2, 10.0);
        Assert.True(aNear.IsDone());
        Assert.Equal(3.0, aNear.DistValue());
        Assert.Single(aNear.Seq1Value());
        Assert.Equal(BRepExtrema_IsVertex, aNear.Seq2Value()[0].SupportKind());
    }

    // ------------------------------------------------------------------ mesh tools

    // The two triangles of a unit square at height z, as a face without surface.
    private static TopoDS_Face MeshSquare(double z, double x0 = 0)
        => TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new gp_Pnt(x0, 0, z), new gp_Pnt(x0 + 1, 0, z), new gp_Pnt(x0 + 1, 1, z), new gp_Pnt(x0, 1, z)],
            (1, 2, 3), (1, 3, 4)));

    [Fact]
    public void TriangleSet_Indices()
    {
        var aShapes = new NCollection_DynamicArray<TopoDS_Shape>();
        aShapes.Append(MeshSquare(0));
        aShapes.Append(MeshSquare(1));
        aShapes.Append(TestShapes.Frozen(BRepGPropTestShapes.PolygonEdge(new gp_Pnt(0, 0, 2), new gp_Pnt(1, 0, 2), new gp_Pnt(2, 0, 2))));
        var aSet = new BRepExtrema_TriangleSet(aShapes);
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
        TopoDS_Face a = MeshSquare(0), b = MeshSquare(0.3);
        var aNear = new BRepExtrema_ShapeProximity(a, b, 0.5);
        aNear.Perform();
        Assert.True(aNear.IsDone());
        Assert.True(aNear.OverlapSubShapes1().IsBound(0));
        Assert.True(aNear.OverlapSubShapes1().Find(0).Contains(0));
        Assert.True(aNear.OverlapSubShapes2().Find(0).Contains(0));
        Assert.True(aNear.GetSubShape1(0).IsSame(a));

        var aFar = new BRepExtrema_ShapeProximity(a, b, 0.2);
        aFar.Perform();
        Assert.False(aFar.IsDone());
        Assert.Equal(0, aFar.OverlapSubShapes1().Extent());

        // tolerance zero: crossing triangles
        TopoDS_Face aVertical = TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new gp_Pnt(0.5, -1, -1), new gp_Pnt(0.5, 2, -1), new gp_Pnt(0.5, 0.5, 1)], (1, 2, 3)));
        var aCross = new BRepExtrema_ShapeProximity(a, aVertical, 0.0);
        aCross.Perform();
        Assert.True(aCross.OverlapSubShapes1().IsBound(0));
    }

    [Fact]
    public void ShapeProximity_Value()
    {
        // two boxes' top faces, exactly meshed, one unit apart
        TopoDS_Solid aBox1 = TestShapes.Box(1, 1, 1);
        BRepGPropTestShapes.AttachQuadMeshes(aBox1);
        TestShapes.Frozen(aBox1);
        TopoDS_Face aTop = TopFace(aBox1);
        TopoDS_Face aTop2 = TopoDS.Face(aTop.Moved(new TopLoc_Location(new gp_Trsf().WithTranslation(new gp_Vec(0, 0, 1)))));

        var aProx = new BRepExtrema_ShapeProximity(aTop, aTop2);
        aProx.Perform();
        Assert.True(aProx.IsDone());
        Assert.Equal(1.0, aProx.Proximity(), Tol);
        Assert.Equal(1.0, aProx.ProximityPoint1().Distance(aProx.ProximityPoint2()), Tol);
    }

    private static TopoDS_Face TopFace(in TopoDS_Shape theBox)
    {
        foreach (TopoDS_Face aF in BRepGPropTestShapes.Faces(theBox))
        {
            var aBox = new Bnd_Box();
            BRepBndLib.Add(aF, ref aBox);
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
        TopoDS_Face a = MeshSquare(0);
        TopoDS_Face aVertical = TestShapes.Frozen(BRepGPropTestShapes.MeshFace(
            [new gp_Pnt(0.5, -1, -1), new gp_Pnt(0.5, 2, -1), new gp_Pnt(0.5, 0.5, 1)], (1, 2, 3)));
        var aSelf = new BRepExtrema_SelfIntersection(TopoDS_Builder.MakeCompound(a, aVertical));
        aSelf.Perform();
        Assert.True(aSelf.IsDone());
        Assert.True(aSelf.OverlapElements().IsBound(0) || aSelf.OverlapElements().IsBound(1));
        Assert.True(aSelf.GetSubShape(1).IsSame(aVertical));

        // two squares side by side, sharing an edge: regular, no self-intersection
        var aClean = new BRepExtrema_SelfIntersection(TopoDS_Builder.MakeCompound(MeshSquare(0), MeshSquare(0, 1)));
        aClean.Perform();
        Assert.Equal(0, aClean.OverlapElements().Extent());
    }

    [Fact]
    public void Poly_Distance()
    {
        gp_Pnt p1 = new(), p2 = new();
        Assert.True(BRepExtrema_Poly.Distance(MeshSquare(0), MeshSquare(2, 3), ref p1, ref p2, out double aDist));
        Assert.Equal(Math.Sqrt(8.0), aDist, Tol);
        AssertPnt(new gp_Pnt(1, 0, 0), p1);
        AssertPnt(new gp_Pnt(3, 0, 2), p2);
        Assert.False(BRepExtrema_Poly.Distance(Square(0, 0, 0, 1), MeshSquare(2), ref p1, ref p2, out _));
    }

    [Fact]
    public void Orientations_DoNotMatter()
    {
        TopoDS_Face aF = Square(0, 0, 0, 10);
        var aD = new BRepExtrema_DistShapeShape(aF.Reversed(), V(2, 3, -7));
        Assert.Equal(7.0, aD.Value(), Tol);
        Assert.Equal(TopAbs_REVERSED, aD.SupportOnShape1(1).Orientation());
        Assert.Equal(TopAbs_FACE, aD.SupportOnShape1(1).ShapeType());
    }

    // A segment edge with a 3D polygon of the given number of evenly spaced nodes.
    private static TopoDS_Edge PolygonSegment(in gp_Pnt theP1, in gp_Pnt theP2, int theNbNodes)
    {
        TopoDS_Edge anEdge = new BRepLib_MakeEdge(theP1, theP2).InternalEdge();
        var aNodes = new NCollection_Array1<gp_Pnt>(1, theNbNodes);
        for (int i = 1; i <= theNbNodes; i++)
        {
            double t = (double)(i - 1) / (theNbNodes - 1);
            aNodes[i] = new gp_Pnt(theP1.XYZ() * (1 - t) + theP2.XYZ() * t);
        }
        new BRep_Builder().UpdateEdge(anEdge, new Poly_Polygon3D(aNodes));
        return TestShapes.Frozen(anEdge);
    }

    // A unit square face at height z (plane surface) with an n x n grid triangulation.
    private static TopoDS_Face GridSquare(double z, int n)
    {
        TopoDS_Face aF = new BRepLib_MakeFace(new gp_Pln(new gp_Pnt(0, 0, z), new gp_Dir(0, 0, 1)), 0, 1, 0, 1).InternalFace();
        var aTri = new Poly_Triangulation((n + 1) * (n + 1), 2 * n * n, false);
        for (int j = 0; j <= n; j++)
        {
            for (int i = 0; i <= n; i++)
            {
                aTri.SetNode(j * (n + 1) + i + 1, new gp_Pnt((double)i / n, (double)j / n, z));
            }
        }
        int k = 1;
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                int a = j * (n + 1) + i + 1, b = a + 1, c = a + n + 2, d = a + n + 1;
                aTri.SetTriangle(k++, new Poly_Triangle(a, b, c));
                aTri.SetTriangle(k++, new Poly_Triangle(a, c, d));
            }
        }
        new BRep_Builder().UpdateFace(aF, aTri);
        return TestShapes.Frozen(aF);
    }

    [Fact]
    public void ProximityValue_EdgesWithRefinement()
    {
        // the second edge's polygon is coarser: it is refined to the first's step
        TopoDS_Edge e1 = PolygonSegment(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0), 11);
        TopoDS_Edge e2 = PolygonSegment(new gp_Pnt(0, 1, 0), new gp_Pnt(10, 1, 0), 3);
        var aProx = new BRepExtrema_ShapeProximity(e1, e2);
        aProx.Perform();
        Assert.True(aProx.IsDone());
        Assert.Equal(1.0, aProx.Proximity(), Tol);
        Assert.Equal(1.0, aProx.ProximityPoint1().Distance(aProx.ProximityPoint2()), Tol);
        Assert.NotEqual(BRepExtrema_ProximityDistTool.ProxPnt_Status.ProxPnt_Status_UNKNOWN, aProx.ProxPntStatus1());

        // more samples than nodes on both
        var aSampled = new BRepExtrema_ShapeProximity(e1, e2);
        aSampled.SetNbSamples1(40);
        aSampled.SetNbSamples2(40);
        aSampled.Perform();
        Assert.True(aSampled.IsDone());
        Assert.Equal(1.0, aSampled.Proximity(), Tol);
    }

    [Fact]
    public void ProximityValue_FacesWithRefinement()
    {
        TopoDS_Face aCoarse = GridSquare(0, 1), aFine = GridSquare(0.5, 4);
        var aProx = new BRepExtrema_ShapeProximity(aCoarse, aFine);
        aProx.Perform();
        Assert.True(aProx.IsDone());
        Assert.Equal(0.5, aProx.Proximity(), Tol);

        var aTool = new BRepExtrema_ProximityValueTool();
        var aList1 = new NCollection_DynamicArray<TopoDS_Shape>();
        aList1.Append(aCoarse);
        var aList2 = new NCollection_DynamicArray<TopoDS_Shape>();
        aList2.Append(aFine);
        aTool.LoadTriangleSets(new BRepExtrema_TriangleSet(aList1), new BRepExtrema_TriangleSet(aList2));
        aTool.LoadShapeLists(aList1, aList2);
        double aTol = 0;
        aTool.Perform(ref aTol);
        Assert.True(aTool.IsDone());
        Assert.Equal(aTool.Distance(), aTol);
        Assert.Equal(0.5, aTol, Tol);
        aTool.ProximityPoints(out gp_Pnt p1, out gp_Pnt p2);
        Assert.Equal(0.5, p1.Distance(p2), Tol);
    }

    [Fact]
    public void SelfIntersection_WithTolerance()
    {
        // two parallel squares 0.1 apart: no intersection, but closer than the tolerance
        TopoDS_Compound aC = TopoDS_Builder.MakeCompound(MeshSquare(0), MeshSquare(0.1));
        var anExact = new BRepExtrema_SelfIntersection(aC);
        anExact.Perform();
        Assert.Equal(0, anExact.OverlapElements().Extent());
        var aTolerant = new BRepExtrema_SelfIntersection(aC, 0.2);
        Assert.Equal(0.2, aTolerant.Tolerance());
        aTolerant.Perform();
        Assert.True(aTolerant.IsDone());
        Assert.True(aTolerant.OverlapElements().IsBound(0) || aTolerant.OverlapElements().IsBound(1));
        Assert.NotNull(aTolerant.ElementSet());
    }

    // ------------------------------------------------------------------ OCCT issues, pinned (occt-issues.md)

    private static TopoDS_Face MeshTriangle(double z)
        => TestShapes.Frozen(BRepGPropTestShapes.MeshFace([new gp_Pnt(0, 0, z), new gp_Pnt(1, 0, z), new gp_Pnt(0, 1, z)], (1, 2, 3)));

    [Fact]
    public void OcctIssue_TriangleSet_InitAgainKeepsTheFirstShapesVertexIds()
    {
        var aFirst = new NCollection_DynamicArray<TopoDS_Shape>();
        aFirst.Append(MeshTriangle(0)); // 3 nodes
        aFirst.Append(MeshSquare(1)); // 4 nodes
        var aSet = new BRepExtrema_TriangleSet(aFirst);
        Assert.Equal(1, aSet.GetShapeIDOfVtx(3));

        var aSecond = new NCollection_DynamicArray<TopoDS_Shape>();
        aSecond.Append(MeshSquare(1));
        aSecond.Append(MeshTriangle(0));
        Assert.True(aSet.Init(aSecond));
        Assert.Equal(7, aSet.GetVertices().Size());
        // vertex 3 is the square's (shape 0), but OCCT reads the first Init's entry
        Assert.Equal(1, aSet.GetShapeIDOfVtx(3));
    }

    [Fact]
    public void OcctIssue_TriangleSet_InitIsTrueWithoutMeshes()
    {
        var aShapes = new NCollection_DynamicArray<TopoDS_Shape>();
        aShapes.Append(Square(0, 0, 0, 1)); // no triangulation
        aShapes.Append(MeshSquare(1));
        var aSet = new BRepExtrema_TriangleSet();
        Assert.True(aSet.Init(aShapes));
        Assert.Equal(0, aSet.Size()); // stopped at the first shape
    }

    [Fact]
    public void OcctIssue_OverlapTool_KeepsOldPairs()
    {
        TopoDS_Face a = MeshSquare(0), b = MeshSquare(0.3);
        var aProx = new BRepExtrema_ShapeProximity(a, b, 0.5);
        aProx.Perform();
        Assert.True(aProx.IsDone());
        Assert.True(aProx.LoadShape2(MeshSquare(5)));
        aProx.Perform();
        Assert.False(aProx.IsDone());
        Assert.True(aProx.OverlapSubShapes1().IsBound(0)); // the first Perform's pair is still there
    }

    [Fact]
    public void OcctIssue_ShapeProximity_ValueBecomesTheTolerance()
    {
        TopoDS_Face aCoarse = GridSquare(0, 1), aFine = GridSquare(0.5, 2);
        var aProx = new BRepExtrema_ShapeProximity(aCoarse, aFine);
        Assert.Equal(Precision.Infinite(), aProx.Tolerance());
        aProx.Perform();
        Assert.Equal(0.5, aProx.Tolerance(), Tol);
    }
}
