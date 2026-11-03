// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Data.ChamfMethod;
using static Kerf.Engine.Fillets.Data.ChamfMode;
using static Kerf.Engine.Fillets.Data.ErrorStatus;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
        CommonPoint P = new();
        Assert.False(P.IsVertex);
        Assert.False(P.IsOnArc);
        Assert.False(P.HasVector);
        Assert.Equal(0.0, P.Tolerance());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => P.Vertex());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => P.Arc());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => P.ParameterOnArc());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => P.TransitionOnArc());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => P.Vector());

        Edge E = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Vertex V = Exploration.FirstVertex(E);
        P.SetVertex(V);
        P.SetArc(1.0e-3, E, 0.25, Reversed);
        P.SetArc(1.0e-5, E, 0.5, Forward); // the tolerance only grows
        P.SetParameter(2.5);
        P.SetPoint(new Point3D(1, 2, 3));
        P.SetVector(new Vector3D(0, 0, 1));
        P.SetTolerance(1.0e-4);
        Assert.True(P.IsVertex && P.IsOnArc && P.HasVector);
        Assert.True(P.Vertex().IsSame(V));
        Assert.True(P.Arc().IsSame(E));
        Assert.Equal(0.5, P.ParameterOnArc());
        Assert.Equal(Forward, P.TransitionOnArc());
        Assert.Equal(1.0e-3, P.Tolerance());
        Assert.Equal(2.5, P.Parameter());
        Assert.Equal(new Point3D(1, 2, 3), P.Point());
        Assert.Equal(new Vector3D(0, 0, 1), P.Vector());

        // Reset keeps the point, the arc and the parameter, as OCCT's.
        CommonPoint R = P.Reseted();
        Assert.False(R.IsVertex || R.IsOnArc || R.HasVector);
        Assert.Equal(0.0, R.Tolerance());
        Assert.Equal(new Point3D(1, 2, 3), R.Point());
        Assert.Equal(2.5, R.Parameter());
        Assert.True(P.IsVertex); // the twin does not change the original
        Assert.Equal(P, P.WithTolerance(1.0e-6));
        Assert.NotEqual(P, P.WithTolerance(1.0));
    }

    [Fact]
    public void FaceInterference_StoresIndexTransitionPCurvesAndParameters()
    {
        FaceInterference I = new();
        Assert.Equal(0, I.LineIndex());
        Assert.Equal(Forward, I.Transition());
        Assert.Null(I.PCurveOnFace());
        LineCurve2D L1 = new(new Point2D(0, 0), new Direction2D(1, 0));
        LineCurve2D L2 = new(new Point2D(0, 1), new Direction2D(0, 1));
        I.SetInterference(3, Reversed, L1, L2);
        I.SetParameter(0.5, true);
        I.SetParameter(2.5, false);
        Assert.Equal(3, I.LineIndex());
        Assert.Equal(Reversed, I.Transition());
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
        Regul R = new();
        R.SetCurve(-4);
        R.SetS1(2);
        R.SetS2(5, false);
        Assert.Equal(4, R.Curve());
        Assert.Equal(2, R.S1());
        Assert.Equal(5, R.S2());
        Assert.False(R.IsSurface1);
        Assert.True(R.IsSurface2);
        Assert.True(R.WithS1(2, false).IsSurface1);
        Assert.Equal(R, new Regul().WithCurve(4).WithS1(2).WithS2(-5, false));
    }

    [Fact]
    public void SurfData_ChangeAccessorsWriteInPlace()
    {
        SurfData D = new();
        Assert.Equal(Forward, D.Orientation());
        Assert.False(D.IsOnCurve1);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => D.IndexOfC1());
        D.ChangeIndexOfS1(1);
        D.ChangeIndexOfS2(2);
        D.ChangeSurf(7);
        D.SetIndexOfC1(4);
        D.SetIndexOfC2(0);
        D.ChangeOrientation() = Reversed;
        D.ChangeVertexFirstOnS1().SetParameter(1.5);
        D.ChangeVertex(false, 2).SetPoint(new Point3D(1, 1, 1));
        D.ChangeInterference(2).SetLineIndex(9);
        D.FirstSpineParam(0.25);
        D.LastSpineParam(0.75);
        D.FirstExtensionValue(1.0);
        D.Set2dPoints(new Point2D(1, 0), new Point2D(2, 0), new Point2D(3, 0), new Point2D(4, 0));
        D.TwistOnS2(true);

        Assert.Equal(1, D.Index(1));
        Assert.Equal(2, D.IndexOfS2());
        Assert.Equal(7, D.Surf());
        Assert.True(D.IsOnCurve(1));
        Assert.Equal(4, D.IndexOfC(1));
        Assert.False(D.IsOnCurve2);
        Assert.Equal(Reversed, D.Orientation());
        Assert.Equal(1.5, D.VertexFirstOnS1().Parameter());
        Assert.Equal(new Point3D(1, 1, 1), D.VertexLastOnS2().Point());
        Assert.Equal(9, D.InterferenceOnS2().LineIndex());
        Assert.Equal(new Point2D(2, 0), D.Get2dPoints(false, 1));
        Assert.Equal(new Point2D(3, 0), D.Get2dPoints(true, 2));
        Assert.True(D.TwistOnS2());

        // Copy copies all but the curves' indices, as OCCT's.
        SurfData E = new();
        E.Copy(D);
        Assert.Equal(7, E.Surf());
        Assert.Equal(Reversed, E.Orientation());
        Assert.Equal(9, E.InterferenceOnS2().LineIndex());
        Assert.Equal(0.75, E.LastSpineParam());
        Assert.False(E.IsOnCurve1);
        E.ChangeVertexFirstOnS1().SetParameter(9.0);
        Assert.Equal(1.5, D.VertexFirstOnS1().Parameter()); // the points are copied, not shared
    }

    [Fact]
    public void Stripe_StoresBothEndsAndResets()
    {
        Stripe S = new();
        FilSpine Sp = new();
        Face aFace = BlendRecorder.BoxFace();
        for (Explorer anExp = new(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(ShapeCasts.Edge(anExp.Current()));
        }

        Sp.Load();
        S.ChangeSpine() = Sp;
        S.ChangeSetOfSurfData() = new NCollection_HSequence<SurfData>();
        S.SetOfSurfData()!.Append(new SurfData());
        S.SetParameters(true, 1, 2);
        S.SetParameters(false, 3, 4);
        S.SetCurve(5, true);
        S.ChangeLastCurve(6);
        S.SetIndexPoint(7, true, 1);
        S.SetIndexPoint(8, false, 2);
        S.SetOrientation(Reversed, 2);
        S.SetOrientation(Reversed, true);
        S.ChangeFirstPCurve() = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        S.ChangeLastPCurve() = new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1));
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
        Assert.Equal(Forward, S.OrientationOnFace1());
        Assert.Equal(Reversed, S.Orientation(2));
        Assert.Equal(Reversed, S.FirstPCurveOrientation());
        Assert.Equal(3, S.Choix());
        Assert.Equal(0, S.IsInDS(true));
        Assert.Equal(2, S.IsInDS(false));
        Assert.Equal(11, S.SolidIndex());

        Sp.SetRadius(1.0);
        S.Reset();
        Assert.Null(S.SetOfSurfData());
        Assert.Equal(Forward, S.FirstPCurveOrientation());
        Assert.Null(S.FirstPCurve());
        Assert.NotNull(S.LastPCurve()); // OCCT: Reset forgets pcrv1 twice and keeps pcrv2 (decision 1040)
        Assert.Same(Sp, S.Spine());
    }

    [Fact]
    public void Maps_IndexShapesAndStripesByIsSame()
    {
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        Map M = new();
        M.Fill(aBox, ShapeType.Edge, ShapeType.Face);
        Explorer anExp = new(aBox, ShapeType.Edge);
        Edge E = ShapeCasts.Edge(anExp.Current());
        Assert.True(M.Contains(E));
        Assert.True(M.Contains(E.Reversed()));
        Assert.Equal(2, M[E].Extent());
        Assert.Equal(2, M[1].Extent());

        StripeMap SM = new();
        Vertex V = Exploration.FirstVertex(E);
        Stripe S1 = new(), S2 = new();
        SM.Add(V, S1);
        SM.Add(ShapeCasts.Vertex(V.Reversed()), S2);
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
        ChamfSpine Sp = new(1.0e-6);
        Assert.Equal(ChamfMethod.Sym, Sp.IsChamfer);
        Assert.Equal(ChamfMode.ClassicChamfer, Sp.Mode());
        Assert.Equal(1.0e-6, Sp.GetTolesp());
        Sp.SetDist(2.0);
        Sp.GetDist(out double d);
        Assert.Equal(2.0, d);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => Sp.Dists(out _, out _));
        Sp.SetDists(1.0, 3.0);
        Sp.Dists(out double d1, out double d2);
        Assert.Equal((1.0, 3.0), (d1, d2));
        Assert.Equal(ChamfMethod.TwoDist, Sp.IsChamfer);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => Sp.GetDist(out _));
        Sp.SetDistAngle(1.5, 0.5);
        Sp.GetDistAngle(out double dd, out double a);
        Assert.Equal((1.5, 0.5), (dd, a));
        Assert.Equal(ChamfMethod.DistAngle, Sp.IsChamfer);
        Sp.SetMode(ChamfMode.ConstThroatChamfer);
        Assert.Equal(ChamfMode.ConstThroatChamfer, Sp.Mode());
    }

    [Fact]
    public void Spine_StatesAndErrorStatus()
    {
        Spine Sp = new();
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Sp.GetTolesp());
        Assert.Equal(State.OnSame, Sp.FirstStatus());
        Assert.Equal(TypeOfConcavity.Other, Sp.GetTypeOfConcavity());
        Assert.Equal(ErrorStatus.Ok, Sp.ErrorStatus());
        Sp.SetStatus(State.FreeBoundary, false);
        Sp.SetTangencyExtremity(true, true);
        Sp.SetTypeOfConcavity(TypeOfConcavity.Convex);
        Sp.SetErrorStatus(ErrorStatus.WalkingFailure);
        Sp.SplitDone(true);
        Assert.Equal(State.FreeBoundary, Sp.Status(false));
        Assert.True(Sp.IsTangencyExtremity(true));
        Assert.False(Sp.IsTangencyExtremity(false));
        Assert.Equal(TypeOfConcavity.Convex, Sp.GetTypeOfConcavity());
        Assert.Equal(ErrorStatus.WalkingFailure, Sp.ErrorStatus());
        Assert.True(Sp.SplitDone());
        Assert.False(Sp.IsPeriodic);

        // The cylinder's circle: closed, one edge, a full turn of length 10 pi.
        for (Explorer anExp = new(BlendRecorder.CylinderTopFace(), ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(ShapeCasts.Edge(anExp.Current()));
        }

        Sp.Load();
        Assert.True(Sp.IsClosed);
        Assert.Equal(10.0 * Math.PI, Sp.LastParameter(), 1.0e-9);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => Sp.Period());
        Sp.SetFirstStatus(State.Closed);
        Assert.Equal(Sp.LastParameter(), Sp.Period());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => Sp.SetFirstTgt(0.0));
        Assert.Null(Sp.ElSpine(1.0));

        ElSpine anEls = new();
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
        FilSpine Sp = new();
        for (Explorer anExp = new(BlendRecorder.BoxFace(), ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Sp.SetEdges(ShapeCasts.Edge(anExp.Current()));
        }

        Sp.Load();
        Assert.False(Sp.IsConstant());
        Sp.SetRadius(2.0);
        Assert.True(Sp.IsConstant());
        Assert.Equal(2.0, Sp.Radius());
        Assert.Equal(2.0, Sp.Radius(Sp.Edges(3)));

        Sp.SetRadius(3.0, Sp.Edges(3));
        Assert.False(Sp.IsConstant());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => Sp.Radius());
        Assert.True(Sp.IsConstant(3));
        Assert.Equal(3.0, Sp.Radius(3));
        Assert.False(Sp.IsConstant(2));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => Sp.Radius(2));
        Assert.Equal(3.0, Sp.MaxRadFromSeqAndLaws());

        Sp.UnSetRadius(Sp.Edges(3));
        Assert.True(Sp.IsConstant());

        Vertex V = Sp.LastVertex();
        Sp.SetRadius(5.0, V);
        Assert.Equal(5.0, Sp.MaxRadFromSeqAndLaws());
        Sp.UnSetRadius(V);
        Assert.Equal(2.0, Sp.MaxRadFromSeqAndLaws());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => Sp.ChangeLaw(Sp.Edges(1)));
    }
}
