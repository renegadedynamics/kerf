// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ChFi3d_Builder_0's free functions that the native recording (<see cref="ChFiKPartNativeTests"/>)
/// does not reach: what they write into the data structure, the input's vertices and geometry shared elsewhere.
/// </summary>
public class ChFi3dBuilder0Tests
{
    private static Vertex FirstVertex(in Shape S)
    {
        global::Kerf.Engine.Topology.Explorer ex = new(S, ShapeType.Vertex);
        return global::Kerf.Engine.Topology.ShapeCasts.Vertex(ex.Current());
    }

    [Fact]
    public void IndexPointInDSWritesTheVertexOfTheWorkingCopyAndRefusesAFrozenOne()
    {
        // OCCT's line 2333: the common point's vertex gets the point's position and tolerance. The builder works on a
        // copy of its input (unit 7N2); a frozen input vertex is refused, never changed (decision 1087).
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Vertex aFrozen = FirstVertex(aBox);
        double aTol = Tool.Tolerance(aFrozen);
        CommonPoint P = new();
        P.SetVertex(aFrozen);
        P.SetPoint(Tool.Pnt(aFrozen));
        P.SetTolerance(1.0e-3);
        DsDataStructure DS = new();
        Assert.Throws<global::Kerf.Engine.Topology.LockedShapeException>(() => ChFi3d_IndexPointInDS(P, DS));
        Assert.Equal(aTol, Tool.Tolerance(aFrozen));

        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out global::Kerf.Engine.Topology.History _);
        Vertex aV = FirstVertex(aCopy);
        P.SetVertex(aV);
        int i = ChFi3d_IndexPointInDS(P, DS);
        Assert.Equal(1.0e-3, Tool.Tolerance(aV));
        Assert.True(DS.Shape(i).IsSame(aV));
        Assert.Equal(aTol, Tool.Tolerance(aFrozen));

        // a point that is not a vertex is a new point of the data structure
        CommonPoint Q = new();
        Q.SetPoint(new Point3D(1, 2, 3));
        Q.SetTolerance(1.0e-4);
        int ip = ChFi3d_IndexPointInDS(Q, DS);
        Assert.Equal(1, DS.NbPoints());
        Assert.Equal(1.0e-4, DS.Point(ip).Tolerance());
    }

    [Fact]
    public void ReparamPcurvSegmentsAFrozenCurveOnACopy()
    {
        // OCCT segments the B-spline in place; a frozen one (an intersection's result) is left as it is
        NCollection_Array1<Point2D> p = new(1, 3);
        p[1] = new Point2D(0, 0);
        p[2] = new Point2D(1, 2);
        p[3] = new Point2D(3, 1);
        NCollection_Array1<double> k = new(1, 2);
        k[1] = 0.0;
        k[2] = 1.0;
        NCollection_Array1<int> m = new(1, 2);
        m.Init(3);
        BSplineCurve2D aBS = new(p, k, m, 2);
        aBS.Freeze();
        Curve2D? aPc = new TrimmedCurve2D(aBS, 0.25, 0.75);
        // the trimmed curve copies its basis: freeze that copy too, as GeomInt's results are
        ((TrimmedCurve2D)aPc).BasisCurve().Freeze();
        ChFi3d_ReparamPcurv(10.0, 20.0, ref aPc);
        BSplineCurve2D aRes = Assert.IsType<BSplineCurve2D>(aPc);
        Assert.Equal(10.0, aRes.FirstParameter());
        Assert.Equal(20.0, aRes.LastParameter());
        Assert.True(aRes.Value(10.0).IsEqual(aBS.Value(0.25), 1.0e-12));
        Assert.True(aRes.Value(20.0).IsEqual(aBS.Value(0.75), 1.0e-12));
        Assert.Equal(0.0, aBS.FirstParameter());
        Assert.Equal(1.0, aBS.LastParameter());
    }

    [Fact]
    public void FilPointAndCurveInterferences()
    {
        DsCurvePointInterference I = ChFi3d_FilPointInDS(Reversed, 3, 7, 1.5, true);
        Assert.Equal(DsKind.Curve, I.SupportType());
        Assert.Equal(3, I.Support());
        Assert.Equal(DsKind.Vertex, I.GeometryType());
        Assert.Equal(7, I.Geometry());
        Assert.Equal(1.5, I.Parameter());
        Assert.Equal(Reversed, I.Transition().Orientation(Containment.In));
        Assert.Equal(DsKind.Point, ChFi3d_FilPointInDS(Forward, 3, 7, 1.5).GeometryType());

        LineCurve2D L = new(new Point2D(0, 0), new Direction2D(1, 0));
        DsSurfaceCurveInterference S = ChFi3d_FilCurveInDS(4, 2, L, Forward);
        Assert.Equal(DsKind.Surface, S.SupportType());
        Assert.Equal(2, S.Support());
        Assert.Equal(DsKind.Curve, S.GeometryType());
        Assert.Equal(4, S.Geometry());
        Assert.Same(L, S.PCurve());

        Assert.Equal(Forward, ChFi3d_TrsfTrans(WalkingTypeTrans.In));
        Assert.Equal(Reversed, ChFi3d_TrsfTrans(WalkingTypeTrans.Out));
        Assert.Equal(Internal, ChFi3d_TrsfTrans(WalkingTypeTrans.Undecided));
    }

    [Fact]
    public void FilCommonPointMeetsTheVertexOfTheArc()
    {
        // an extremity of the walking at the end of a box edge: the common point takes the edge's vertex
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        global::Kerf.Engine.Topology.Explorer exf = new(aBox, ShapeType.Face);
        Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(exf.Current());
        global::Kerf.Engine.Topology.Explorer exe = new(F, ShapeType.Edge);
        Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(exe.Current());
        global::Kerf.Engine.Topology.Exploration.Vertices(E, out Vertex V1, out Vertex _);
        Point3D P = Tool.Pnt(V1);
        EdgeCurve2DAdaptor anArc = new(E, F);
        Extremity SP = new(P, 0.0, 0.0, 2.5, 1.0e-6);
        SP.SetTangent(new Vector3D(0, 0, 1));
        WalkingTransition T = new();
        SP.AddArc(anArc, Tool.Parameter(V1, E), T, T);
        CommonPoint CP = new();
        ChFi3d_FilCommonPoint(SP, WalkingTypeTrans.In, true, ref CP, 1.0e-5);
        Assert.True(CP.IsVertex);
        Assert.True(CP.Vertex().IsSame(V1));
        Assert.True(CP.IsOnArc);
        Assert.True(CP.Arc().IsSame(E));
        Assert.Equal(2.5, CP.Parameter());
        Assert.True(CP.Vector().IsEqual(new Vector3D(0, 0, -1), 1.0e-15, 1.0e-15));
        Assert.Equal(global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(Forward, E.Orientation())), CP.TransitionOnArc());
        Assert.True(CP.Tolerance() >= Tool.Tolerance(V1));
    }

    [Fact]
    public void SmallHelpers()
    {
        Assert.Equal(0.5, ChFi3d_InPeriod(0.5 + 2 * Math.PI, 0.0, 2 * Math.PI, 1.0e-9), 14);
        Assert.Equal(0.0, ChFi3d_InPeriod(-1.0e-12, 0.0, 2 * Math.PI, 1.0e-9));
        ChFi3d_Boite(new Point2D(3, 1), new Point2D(-1, 4), out double mu, out double Mu, out double mv,
                     out double Mv);
        Assert.Equal((-1.0, 3.0, 1.0, 4.0), (mu, Mu, mv, Mv));
        ChFi3d_Boite(new Point2D(3, 1), new Point2D(-1, 4), new Point2D(5, 0), new Point2D(0, 2), out double Du,
                     out double Dv, out mu, out Mu, out mv, out Mv);
        Assert.Equal((6.0, 4.0, -1.0, 5.0, 0.0, 4.0), (Du, Dv, mu, Mu, mv, Mv));
        // the pivot: the sections of the other two fillets do not cross on its opposite face
        double[,] u = { { 0, 1, 2 }, { 2, 0, 1 }, { 1, 2, 0 } };
        Assert.Equal(-1, ChFi3d_SearchPivot([1, 1, 1], u, 1.0e-9));
        Assert.Equal(2, ChFi3d_SearchPivot([1, -1, 1], u, 1.0e-9));
    }

    [Fact]
    public void KParticularOnABoxAndACylinder()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Map EF = new();
        EF.Fill(aBox, ShapeType.Edge, ShapeType.Face);
        global::Kerf.Engine.Topology.Explorer ex = new(aBox, ShapeType.Edge);
        Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(ex.Current());
        ChFi3d_conexfaces(E, out Face F1, out Face F2, EF);
        FilSpine Sp = new();
        Sp.SetEdges(E);
        Sp.Load();
        Sp.SetRadius(1.0);
        Assert.True(ChFi3d_KParticular(Sp, 1, new FaceSurfaceAdaptor(F1), new FaceSurfaceAdaptor(F2)));
        // an evolving radius is not particular
        Sp.SetRadius(new Xy(0.0, 2.0), 1);
        Assert.False(ChFi3d_KParticular(Sp, 1, new FaceSurfaceAdaptor(F1), new FaceSurfaceAdaptor(F2)));
    }
}
