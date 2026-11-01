// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiDS_ChamfMethod;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ChFiDS's data classes against OCCT's semantics: the common points, face interferences, regularities, surface data,
/// stripes and their maps, and the chamfer and fillet spines' settings (the spines' geometry is compared with native
/// OCCT in <see cref="BlendFuncNativeTests"/>).
/// </summary>
public class ChFiDSTests
{
    [Fact]
    public void CommonPoint_StoresVertexArcAndVector()
    {
        ChFiDS_CommonPoint P = new();
        Assert.False(P.IsVertex());
        Assert.False(P.IsOnArc());
        Assert.False(P.HasVector());
        Assert.Equal(0.0, P.Tolerance());
        Assert.ThrowsAny<Standard_DomainError>(() => P.Vertex());
        Assert.ThrowsAny<Standard_DomainError>(() => P.Arc());
        Assert.ThrowsAny<Standard_DomainError>(() => P.ParameterOnArc());
        Assert.ThrowsAny<Standard_DomainError>(() => P.TransitionOnArc());
        Assert.ThrowsAny<Standard_DomainError>(() => P.Vector());

        TopoDS_Edge E = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        TopoDS_Vertex V = TopExp.FirstVertex(E);
        P.SetVertex(V);
        P.SetArc(1.0e-3, E, 0.25, TopAbs_REVERSED);
        P.SetArc(1.0e-5, E, 0.5, TopAbs_FORWARD); // the tolerance only grows
        P.SetParameter(2.5);
        P.SetPoint(new gp_Pnt(1, 2, 3));
        P.SetVector(new gp_Vec(0, 0, 1));
        P.SetTolerance(1.0e-4);
        Assert.True(P.IsVertex() && P.IsOnArc() && P.HasVector());
        Assert.True(P.Vertex().IsSame(V));
        Assert.True(P.Arc().IsSame(E));
        Assert.Equal(0.5, P.ParameterOnArc());
        Assert.Equal(TopAbs_FORWARD, P.TransitionOnArc());
        Assert.Equal(1.0e-3, P.Tolerance());
        Assert.Equal(2.5, P.Parameter());
        Assert.Equal(new gp_Pnt(1, 2, 3), P.Point());
        Assert.Equal(new gp_Vec(0, 0, 1), P.Vector());

        // Reset keeps the point, the arc and the parameter, as OCCT's.
        ChFiDS_CommonPoint R = P.Reseted();
        Assert.False(R.IsVertex() || R.IsOnArc() || R.HasVector());
        Assert.Equal(0.0, R.Tolerance());
        Assert.Equal(new gp_Pnt(1, 2, 3), R.Point());
        Assert.Equal(2.5, R.Parameter());
        Assert.True(P.IsVertex()); // the twin does not change the original
        Assert.Equal(P, P.WithTolerance(1.0e-6));
        Assert.NotEqual(P, P.WithTolerance(1.0));
    }

    [Fact]
    public void FaceInterference_StoresIndexTransitionPCurvesAndParameters()
    {
        ChFiDS_FaceInterference I = new();
        Assert.Equal(0, I.LineIndex());
        Assert.Equal(TopAbs_FORWARD, I.Transition());
        Assert.Null(I.PCurveOnFace());
        Geom2d_Line L1 = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        Geom2d_Line L2 = new(new gp_Pnt2d(0, 1), new gp_Dir2d(0, 1));
        I.SetInterference(3, TopAbs_REVERSED, L1, L2);
        I.SetParameter(0.5, true);
        I.SetParameter(2.5, false);
        Assert.Equal(3, I.LineIndex());
        Assert.Equal(TopAbs_REVERSED, I.Transition());
        Assert.Same(L1, I.PCurveOnFace());
        Assert.Same(L2, I.PCurveOnSurf());
        Assert.Equal(0.5, I.Parameter(true));
        Assert.Equal(2.5, I.LastParameter());
        I.ChangePCurveOnSurf() = L1;
        Assert.Same(L1, I.PCurveOnSurf());
        Assert.Equal(7, I.WithLineIndex(7).LineIndex());
        Assert.Equal(3, I.LineIndex());
    }

    [Fact]
    public void Regul_KeepsFacesPositiveAndSurfacesNegative()
    {
        ChFiDS_Regul R = new();
        R.SetCurve(-4);
        R.SetS1(2);
        R.SetS2(5, false);
        Assert.Equal(4, R.Curve());
        Assert.Equal(2, R.S1());
        Assert.Equal(5, R.S2());
        Assert.False(R.IsSurface1());
        Assert.True(R.IsSurface2());
        Assert.True(R.WithS1(2, false).IsSurface1());
        Assert.Equal(R, new ChFiDS_Regul().WithCurve(4).WithS1(2).WithS2(-5, false));
    }

    [Fact]
    public void SurfData_ChangeAccessorsWriteInPlace()
    {
        ChFiDS_SurfData D = new();
        Assert.Equal(TopAbs_FORWARD, D.Orientation());
        Assert.False(D.IsOnCurve1());
        Assert.ThrowsAny<Standard_Failure>(() => D.IndexOfC1());
        D.ChangeIndexOfS1(1);
        D.ChangeIndexOfS2(2);
        D.ChangeSurf(7);
        D.SetIndexOfC1(4);
        D.SetIndexOfC2(0);
        D.ChangeOrientation() = TopAbs_REVERSED;
        D.ChangeVertexFirstOnS1().SetParameter(1.5);
        D.ChangeVertex(false, 2).SetPoint(new gp_Pnt(1, 1, 1));
        D.ChangeInterference(2).SetLineIndex(9);
        D.FirstSpineParam(0.25);
        D.LastSpineParam(0.75);
        D.FirstExtensionValue(1.0);
        D.Set2dPoints(new gp_Pnt2d(1, 0), new gp_Pnt2d(2, 0), new gp_Pnt2d(3, 0), new gp_Pnt2d(4, 0));
        D.TwistOnS2(true);

        Assert.Equal(1, D.Index(1));
        Assert.Equal(2, D.IndexOfS2());
        Assert.Equal(7, D.Surf());
        Assert.True(D.IsOnCurve(1));
        Assert.Equal(4, D.IndexOfC(1));
        Assert.False(D.IsOnCurve2());
        Assert.Equal(TopAbs_REVERSED, D.Orientation());
        Assert.Equal(1.5, D.VertexFirstOnS1().Parameter());
        Assert.Equal(new gp_Pnt(1, 1, 1), D.VertexLastOnS2().Point());
        Assert.Equal(9, D.InterferenceOnS2().LineIndex());
        Assert.Equal(new gp_Pnt2d(2, 0), D.Get2dPoints(false, 1));
        Assert.Equal(new gp_Pnt2d(3, 0), D.Get2dPoints(true, 2));
        Assert.True(D.TwistOnS2());

        // Copy copies all but the curves' indices, as OCCT's.
        ChFiDS_SurfData E = new();
        E.Copy(D);
        Assert.Equal(7, E.Surf());
        Assert.Equal(TopAbs_REVERSED, E.Orientation());
        Assert.Equal(9, E.InterferenceOnS2().LineIndex());
        Assert.Equal(0.75, E.LastSpineParam());
        Assert.False(E.IsOnCurve1());
        E.ChangeVertexFirstOnS1().SetParameter(9.0);
        Assert.Equal(1.5, D.VertexFirstOnS1().Parameter()); // the points are copied, not shared
    }

    [Fact]
    public void Stripe_StoresBothEndsAndResets()
    {
        ChFiDS_Stripe S = new();
        ChFiDS_FilSpine Sp = new();
        TopoDS_Face aFace = BlendRecorder.BoxFace();
        for (TopExp_Explorer anExp = new(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(TopoDS.Edge(anExp.Current()));
        }

        Sp.Load();
        S.ChangeSpine() = Sp;
        S.ChangeSetOfSurfData() = new NCollection_HSequence<ChFiDS_SurfData>();
        S.SetOfSurfData()!.Append(new ChFiDS_SurfData());
        S.SetParameters(true, 1, 2);
        S.SetParameters(false, 3, 4);
        S.SetCurve(5, true);
        S.ChangeLastCurve(6);
        S.SetIndexPoint(7, true, 1);
        S.SetIndexPoint(8, false, 2);
        S.SetOrientation(TopAbs_REVERSED, 2);
        S.SetOrientation(TopAbs_REVERSED, true);
        S.ChangeFirstPCurve() = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0));
        S.ChangeLastPCurve() = new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1));
        S.Choix(3);
        S.InDS(false, 2);
        S.SetSolidIndex(11);

        S.Parameters(false, out double d, out double f);
        Assert.Equal((3.0, 4.0), (d, f));
        S.FirstParameters(out d, out f);
        Assert.Equal((1.0, 2.0), (d, f));
        Assert.Equal(5, S.Curve(true));
        Assert.Equal(6, S.LastCurve());
        Assert.Equal(7, S.IndexFirstPointOnS1());
        Assert.Equal(8, S.IndexPoint(false, 2));
        Assert.Equal(TopAbs_FORWARD, S.OrientationOnFace1());
        Assert.Equal(TopAbs_REVERSED, S.Orientation(2));
        Assert.Equal(TopAbs_REVERSED, S.FirstPCurveOrientation());
        Assert.Equal(3, S.Choix());
        Assert.Equal(0, S.IsInDS(true));
        Assert.Equal(2, S.IsInDS(false));
        Assert.Equal(11, S.SolidIndex());

        Sp.SetRadius(1.0);
        S.Reset();
        Assert.Null(S.SetOfSurfData());
        Assert.Equal(TopAbs_FORWARD, S.FirstPCurveOrientation());
        Assert.Null(S.FirstPCurve());
        Assert.NotNull(S.LastPCurve()); // OCCT: Reset forgets pcrv1 twice and keeps pcrv2 (decision 1040)
        Assert.Same(Sp, S.Spine());
    }

    [Fact]
    public void Maps_IndexShapesAndStripesByIsSame()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
        ChFiDS_Map M = new();
        M.Fill(aBox, TopAbs_EDGE, TopAbs_FACE);
        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        TopoDS_Edge E = TopoDS.Edge(anExp.Current());
        Assert.True(M.Contains(E));
        Assert.True(M.Contains(E.Reversed()));
        Assert.Equal(2, M[E].Extent());
        Assert.Equal(2, M[1].Extent());

        ChFiDS_StripeMap SM = new();
        TopoDS_Vertex V = TopExp.FirstVertex(E);
        ChFiDS_Stripe S1 = new(), S2 = new();
        SM.Add(V, S1);
        SM.Add(TopoDS.Vertex(V.Reversed()), S2);
        Assert.Equal(1, SM.Extent());
        Assert.Equal(2, SM[V].Extent());
        Assert.Same(S1, SM[1].First());
        Assert.True(SM.FindKey(1).IsSame(V));
        SM.Clear();
        Assert.Equal(0, SM.Extent());
    }

    [Fact]
    public void ChamfSpine_GivesTheDistancesOfItsMethodOnly()
    {
        ChFiDS_ChamfSpine Sp = new(1.0e-6);
        Assert.Equal(ChFiDS_Sym, Sp.IsChamfer());
        Assert.Equal(ChFiDS_ClassicChamfer, Sp.Mode());
        Assert.Equal(1.0e-6, Sp.GetTolesp());
        Sp.SetDist(2.0);
        Sp.GetDist(out double d);
        Assert.Equal(2.0, d);
        Assert.ThrowsAny<Standard_Failure>(() => Sp.Dists(out _, out _));
        Sp.SetDists(1.0, 3.0);
        Sp.Dists(out double d1, out double d2);
        Assert.Equal((1.0, 3.0), (d1, d2));
        Assert.Equal(ChFiDS_TwoDist, Sp.IsChamfer());
        Assert.ThrowsAny<Standard_Failure>(() => Sp.GetDist(out _));
        Sp.SetDistAngle(1.5, 0.5);
        Sp.GetDistAngle(out double dd, out double a);
        Assert.Equal((1.5, 0.5), (dd, a));
        Assert.Equal(ChFiDS_DistAngle, Sp.IsChamfer());
        Sp.SetMode(ChFiDS_ConstThroatChamfer);
        Assert.Equal(ChFiDS_ConstThroatChamfer, Sp.Mode());
    }

    [Fact]
    public void Spine_StatesAndErrorStatus()
    {
        ChFiDS_Spine Sp = new();
        Assert.Equal(Precision.Confusion(), Sp.GetTolesp());
        Assert.Equal(ChFiDS_OnSame, Sp.FirstStatus());
        Assert.Equal(ChFiDS_TypeOfConcavity.ChFiDS_Other, Sp.GetTypeOfConcavity());
        Assert.Equal(ChFiDS_Ok, Sp.ErrorStatus());
        Sp.SetStatus(ChFiDS_FreeBoundary, false);
        Sp.SetTangencyExtremity(true, true);
        Sp.SetTypeOfConcavity(ChFiDS_TypeOfConcavity.ChFiDS_Convex);
        Sp.SetErrorStatus(ChFiDS_WalkingFailure);
        Sp.SplitDone(true);
        Assert.Equal(ChFiDS_FreeBoundary, Sp.Status(false));
        Assert.True(Sp.IsTangencyExtremity(true));
        Assert.False(Sp.IsTangencyExtremity(false));
        Assert.Equal(ChFiDS_TypeOfConcavity.ChFiDS_Convex, Sp.GetTypeOfConcavity());
        Assert.Equal(ChFiDS_WalkingFailure, Sp.ErrorStatus());
        Assert.True(Sp.SplitDone());
        Assert.False(Sp.IsPeriodic());

        // The cylinder's circle: closed, one edge, a full turn of length 10 pi.
        for (TopExp_Explorer anExp = new(BlendRecorder.CylinderTopFace(), TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(TopoDS.Edge(anExp.Current()));
        }

        Sp.Load();
        Assert.True(Sp.IsClosed());
        Assert.Equal(10.0 * Math.PI, Sp.LastParameter(), 1.0e-9);
        Assert.ThrowsAny<Standard_Failure>(() => Sp.Period());
        Sp.SetFirstStatus(ChFiDS_Closed);
        Assert.Equal(Sp.LastParameter(), Sp.Period());
        Assert.ThrowsAny<Standard_Failure>(() => Sp.SetFirstTgt(0.0));
        Assert.Null(Sp.ElSpine(1.0));

        ChFiDS_ElSpine anEls = new();
        anEls.FirstParameter(0.0);
        anEls.LastParameter(Sp.LastParameter());
        Sp.AppendElSpine(anEls);
        Assert.Same(anEls, Sp.ElSpine(1));
        Sp.Reset();
        Assert.False(Sp.SplitDone());
        Assert.Null(Sp.ElSpine(1.0));
    }

    [Fact]
    public void FilSpine_RadiusOnEdgesAndVertices()
    {
        ChFiDS_FilSpine Sp = new();
        for (TopExp_Explorer anExp = new(BlendRecorder.BoxFace(), TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(TopoDS.Edge(anExp.Current()));
        }

        Sp.Load();
        Assert.False(Sp.IsConstant());
        Sp.SetRadius(2.0);
        Assert.True(Sp.IsConstant());
        Assert.Equal(2.0, Sp.Radius());
        Assert.Equal(2.0, Sp.Radius(Sp.Edges(3)));

        Sp.SetRadius(3.0, Sp.Edges(3));
        Assert.False(Sp.IsConstant());
        Assert.ThrowsAny<Standard_DomainError>(() => Sp.Radius());
        Assert.True(Sp.IsConstant(3));
        Assert.Equal(3.0, Sp.Radius(3));
        Assert.False(Sp.IsConstant(2));
        Assert.ThrowsAny<Standard_DomainError>(() => Sp.Radius(2));
        Assert.Equal(3.0, Sp.MaxRadFromSeqAndLaws());

        Sp.UnSetRadius(Sp.Edges(3));
        Assert.True(Sp.IsConstant());

        TopoDS_Vertex V = Sp.LastVertex();
        Sp.SetRadius(5.0, V);
        Assert.Equal(5.0, Sp.MaxRadFromSeqAndLaws());
        Sp.UnSetRadius(V);
        Assert.Equal(2.0, Sp.MaxRadFromSeqAndLaws());
        Assert.ThrowsAny<Standard_DomainError>(() => Sp.ChangeLaw(Sp.Edges(1)));
    }
}
