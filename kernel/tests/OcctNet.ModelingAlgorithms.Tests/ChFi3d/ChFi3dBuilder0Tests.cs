// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFi3d_Builder_0;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopOpeBRepDS_Kind;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ChFi3d_Builder_0's free functions that the native recording (<see cref="ChFiKPartNativeTests"/>)
/// does not reach: what they write into the data structure, the input's vertices and geometry shared elsewhere.
/// </summary>
public class ChFi3dBuilder0Tests
{
    private static TopoDS_Vertex FirstVertex(in TopoDS_Shape S)
    {
        TopExp_Explorer ex = new(S, TopAbs_VERTEX);
        return TopoDS.Vertex(ex.Current());
    }

    [Fact]
    public void IndexPointInDSWritesTheVertexOfTheWorkingCopyAndRefusesAFrozenOne()
    {
        // OCCT's line 2333: the common point's vertex gets the point's position and tolerance. The builder works on a
        // copy of its input (unit 7N2); a frozen input vertex is refused, never changed (decision 1087).
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopoDS_Vertex aFrozen = FirstVertex(aBox);
        double aTol = BRep_Tool.Tolerance(aFrozen);
        ChFiDS_CommonPoint P = new();
        P.SetVertex(aFrozen);
        P.SetPoint(BRep_Tool.Pnt(aFrozen));
        P.SetTolerance(1.0e-3);
        TopOpeBRepDS_DataStructure DS = new();
        Assert.Throws<TopoDS_LockedShape>(() => ChFi3d_IndexPointInDS(P, DS));
        Assert.Equal(aTol, BRep_Tool.Tolerance(aFrozen));

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History _);
        TopoDS_Vertex aV = FirstVertex(aCopy);
        P.SetVertex(aV);
        int i = ChFi3d_IndexPointInDS(P, DS);
        Assert.Equal(1.0e-3, BRep_Tool.Tolerance(aV));
        Assert.True(DS.Shape(i).IsSame(aV));
        Assert.Equal(aTol, BRep_Tool.Tolerance(aFrozen));

        // a point that is not a vertex is a new point of the data structure
        ChFiDS_CommonPoint Q = new();
        Q.SetPoint(new gp_Pnt(1, 2, 3));
        Q.SetTolerance(1.0e-4);
        int ip = ChFi3d_IndexPointInDS(Q, DS);
        Assert.Equal(1, DS.NbPoints());
        Assert.Equal(1.0e-4, DS.Point(ip).Tolerance());
    }

    [Fact]
    public void ReparamPcurvSegmentsAFrozenCurveOnACopy()
    {
        // OCCT segments the B-spline in place; a frozen one (an intersection's result) is left as it is
        NCollection_Array1<gp_Pnt2d> p = new(1, 3);
        p[1] = new gp_Pnt2d(0, 0);
        p[2] = new gp_Pnt2d(1, 2);
        p[3] = new gp_Pnt2d(3, 1);
        NCollection_Array1<double> k = new(1, 2);
        k[1] = 0.0;
        k[2] = 1.0;
        NCollection_Array1<int> m = new(1, 2);
        m.Init(3);
        Geom2d_BSplineCurve aBS = new(p, k, m, 2);
        aBS.Freeze();
        Geom2d_Curve? aPc = new Geom2d_TrimmedCurve(aBS, 0.25, 0.75);
        // the trimmed curve copies its basis: freeze that copy too, as GeomInt's results are
        ((Geom2d_TrimmedCurve)aPc).BasisCurve().Freeze();
        ChFi3d_ReparamPcurv(10.0, 20.0, ref aPc);
        Geom2d_BSplineCurve aRes = Assert.IsType<Geom2d_BSplineCurve>(aPc);
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
        TopOpeBRepDS_CurvePointInterference I = ChFi3d_FilPointInDS(TopAbs_REVERSED, 3, 7, 1.5, true);
        Assert.Equal(TopOpeBRepDS_CURVE, I.SupportType());
        Assert.Equal(3, I.Support());
        Assert.Equal(TopOpeBRepDS_VERTEX, I.GeometryType());
        Assert.Equal(7, I.Geometry());
        Assert.Equal(1.5, I.Parameter());
        Assert.Equal(TopAbs_REVERSED, I.Transition().Orientation(TopAbs_State.TopAbs_IN));
        Assert.Equal(TopOpeBRepDS_POINT, ChFi3d_FilPointInDS(TopAbs_FORWARD, 3, 7, 1.5).GeometryType());

        Geom2d_Line L = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        TopOpeBRepDS_SurfaceCurveInterference S = ChFi3d_FilCurveInDS(4, 2, L, TopAbs_FORWARD);
        Assert.Equal(TopOpeBRepDS_SURFACE, S.SupportType());
        Assert.Equal(2, S.Support());
        Assert.Equal(TopOpeBRepDS_CURVE, S.GeometryType());
        Assert.Equal(4, S.Geometry());
        Assert.Same(L, S.PCurve());

        Assert.Equal(TopAbs_FORWARD, ChFi3d_TrsfTrans(IntSurf_TypeTrans.IntSurf_In));
        Assert.Equal(TopAbs_REVERSED, ChFi3d_TrsfTrans(IntSurf_TypeTrans.IntSurf_Out));
        Assert.Equal(TopAbs_INTERNAL, ChFi3d_TrsfTrans(IntSurf_TypeTrans.IntSurf_Undecided));
    }

    [Fact]
    public void FilCommonPointMeetsTheVertexOfTheArc()
    {
        // an extremity of the walking at the end of a box edge: the common point takes the edge's vertex
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopExp_Explorer exf = new(aBox, TopAbs_FACE);
        TopoDS_Face F = TopoDS.Face(exf.Current());
        TopExp_Explorer exe = new(F, TopAbs_EDGE);
        TopoDS_Edge E = TopoDS.Edge(exe.Current());
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex _);
        gp_Pnt P = BRep_Tool.Pnt(V1);
        BRepAdaptor_Curve2d anArc = new(E, F);
        BRepBlend_Extremity SP = new(P, 0.0, 0.0, 2.5, 1.0e-6);
        SP.SetTangent(new gp_Vec(0, 0, 1));
        IntSurf_Transition T = new();
        SP.AddArc(anArc, BRep_Tool.Parameter(V1, E), T, T);
        ChFiDS_CommonPoint CP = new();
        ChFi3d_FilCommonPoint(SP, IntSurf_TypeTrans.IntSurf_In, true, ref CP, 1.0e-5);
        Assert.True(CP.IsVertex());
        Assert.True(CP.Vertex().IsSame(V1));
        Assert.True(CP.IsOnArc());
        Assert.True(CP.Arc().IsSame(E));
        Assert.Equal(2.5, CP.Parameter());
        Assert.True(CP.Vector().IsEqual(new gp_Vec(0, 0, -1), 1.0e-15, 1.0e-15));
        Assert.Equal(TopAbs.Reverse(TopAbs.Compose(TopAbs_FORWARD, E.Orientation())), CP.TransitionOnArc());
        Assert.True(CP.Tolerance() >= BRep_Tool.Tolerance(V1));
    }

    [Fact]
    public void SmallHelpers()
    {
        Assert.Equal(0.5, ChFi3d_InPeriod(0.5 + 2 * Math.PI, 0.0, 2 * Math.PI, 1.0e-9), 14);
        Assert.Equal(0.0, ChFi3d_InPeriod(-1.0e-12, 0.0, 2 * Math.PI, 1.0e-9));
        ChFi3d_Boite(new gp_Pnt2d(3, 1), new gp_Pnt2d(-1, 4), out double mu, out double Mu, out double mv,
                     out double Mv);
        Assert.Equal((-1.0, 3.0, 1.0, 4.0), (mu, Mu, mv, Mv));
        ChFi3d_Boite(new gp_Pnt2d(3, 1), new gp_Pnt2d(-1, 4), new gp_Pnt2d(5, 0), new gp_Pnt2d(0, 2), out double Du,
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
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        ChFiDS_Map EF = new();
        EF.Fill(aBox, TopAbs_EDGE, TopAbs_FACE);
        TopExp_Explorer ex = new(aBox, TopAbs_EDGE);
        TopoDS_Edge E = TopoDS.Edge(ex.Current());
        ChFi3d_conexfaces(E, out TopoDS_Face F1, out TopoDS_Face F2, EF);
        ChFiDS_FilSpine Sp = new();
        Sp.SetEdges(E);
        Sp.Load();
        Sp.SetRadius(1.0);
        Assert.True(ChFi3d_KParticular(Sp, 1, new BRepAdaptor_Surface(F1), new BRepAdaptor_Surface(F2)));
        // an evolving radius is not particular
        Sp.SetRadius(new gp_XY(0.0, 2.0), 1);
        Assert.False(ChFi3d_KParticular(Sp, 1, new BRepAdaptor_Surface(F1), new BRepAdaptor_Surface(F2)));
    }
}
