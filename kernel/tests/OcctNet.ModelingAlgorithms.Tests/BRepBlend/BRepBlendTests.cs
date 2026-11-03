// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Occt.IntSurf_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepBlend's data and tools against OCCT's semantics: the line, its extremities and points on restrictions (value
/// semantics included), the curve tools, the blend tool's projection and intersection on arcs, the approximation's
/// function (the points it inserts into the line) and the walkers' states (the walking itself is compared with native
/// OCCT in <see cref="BRepBlendNativeTests"/>).
/// </summary>
public class BRepBlendTests
{
    private static Blend_Point Pt(double theParam) =>
        new(new Point3D(theParam, 1, 0), new Point3D(theParam, 0, 1), theParam, theParam, 1, theParam, -1);

    private static Adaptor2d_Curve2d Segment(double x0, double y0, double x1, double y1)
    {
        Vector2D V = new(new Point2D(x0, y0), new Point2D(x1, y1));
        return new Geom2dAdaptor_Curve(new Geom2d_Line(new Point2D(x0, y0), new Direction2D(V)), 0.0, V.Magnitude());
    }

    [Fact]
    public void Line_SequenceAndTransitions()
    {
        BRepBlend_Line L = new();
        Assert.Equal(0, L.NbPoints());
        Assert.ThrowsAny<DomainErrorException>(() => L.TransitionOnS1());
        Assert.ThrowsAny<DomainErrorException>(() => L.TransitionOnS2());
        Assert.ThrowsAny<DomainErrorException>(() => L.TransitionOnS());

        L.Append(Pt(2));
        L.Append(Pt(3));
        L.Prepend(Pt(1));
        L.InsertBefore(3, Pt(2.5));
        Assert.Equal([1.0, 2.0, 2.5, 3.0], Enumerable.Range(1, L.NbPoints()).Select(i => L.Point(i).Parameter()));
        L.Remove(2, 3);
        Assert.Equal([1.0, 3.0], Enumerable.Range(1, L.NbPoints()).Select(i => L.Point(i).Parameter()));

        L.Set(IntSurf_Out);
        Assert.Equal(IntSurf_Out, L.TransitionOnS1());
        Assert.Equal(IntSurf_Out, L.TransitionOnS());
        Assert.ThrowsAny<DomainErrorException>(() => L.TransitionOnS2());
        L.Set(IntSurf_In, IntSurf_Out);
        Assert.Equal(IntSurf_In, L.TransitionOnS1());
        Assert.Equal(IntSurf_Out, L.TransitionOnS2());

        // The line holds copies: changing the point given changes nothing.
        Blend_Point P = Pt(7);
        L.Append(P);
        P.SetParameter(8);
        Assert.Equal(7.0, L.Point(3).Parameter());
        // ... and the builders write a stored point in place, as OCCT's references do.
        L.ChangePoint(3).SetParameter(9);
        Assert.Equal(9.0, L.Point(3).Parameter());

        L.Clear();
        Assert.Equal(0, L.NbPoints());
        Assert.ThrowsAny<DomainErrorException>(() => L.TransitionOnS1());
    }

    [Fact]
    public void Extremity_ValuesVertexAndArcs()
    {
        BRepBlend_Extremity E = new();
        Assert.Equal(new Point3D(0, 0, 0), E.Value());
        Assert.False(E.HasTangent());
        Assert.False(E.IsVertex());
        Assert.Equal(0, E.NbPointOnRst());
        Assert.ThrowsAny<DomainErrorException>(() => E.Tangent());
        Assert.ThrowsAny<DomainErrorException>(() => E.Vertex());

        E.SetValue(new Point3D(1, 2, 3), 0.5, 0.25, 4.0, 1.0e-4);
        E.Parameters(out double u, out double v);
        Assert.Equal((0.5, 0.25, 4.0, 1.0e-4), (u, v, E.ParameterOnGuide(), E.Tolerance()));
        Assert.Equal(0.5, E.Parameter());

        // The curve overload sets the parameter W in U and keeps V, as OCCT's.
        E.SetValue(new Point3D(1, 2, 3), 7.0, 5.0, 1.0e-3);
        E.Parameters(out u, out v);
        Assert.Equal((7.0, 0.25, 5.0), (u, v, E.ParameterOnGuide()));

        E.SetTangent(new Vector3D(0, 0, 1));
        Assert.True(E.HasTangent());
        Assert.Equal(new Vector3D(0, 0, 1), E.Tangent());

        Adaptor3d_HVertex V = new(new Point2D(0, 0), TopAbs_Orientation.TopAbs_FORWARD, 1.0e-7);
        E.SetVertex(V);
        Assert.True(E.IsVertex());
        Assert.Same(V, E.Vertex());

        Adaptor2d_Curve2d A = Segment(0, 0, 1, 0);
        IntSurf_Transition TIn = new(false, IntSurf_In), TOut = new(false, IntSurf_Out);
        E.AddArc(A, 0.5, TIn, TOut);
        Assert.Equal(1, E.NbPointOnRst());
        Assert.Same(A, E.PointOnRst(1).Arc());
        Assert.Equal(0.5, E.PointOnRst(1).ParameterOnArc());
        Assert.Equal(IntSurf_In, E.PointOnRst(1).TransitionOnLine().TransitionType());
        Assert.Equal(IntSurf_Out, E.PointOnRst(1).TransitionOnArc().TransitionType());

        // SetValue forgets the arcs and the vertex flag (not the tangent).
        E.SetValue(new Point3D(0, 0, 0), 1.0, 2.0, 3.0, 1.0e-4);
        Assert.Equal(0, E.NbPointOnRst());
        Assert.False(E.IsVertex());
        Assert.True(E.HasTangent());
        Assert.ThrowsAny<OutOfRangeException>(() => E.PointOnRst(1));
    }

    [Fact]
    public void Extremity_CopiesAreIndependent()
    {
        // OCCT copies extremities by value (with their sequence of arcs); so does the port (decision 1059).
        Adaptor2d_Curve2d A = Segment(0, 0, 1, 0);
        IntSurf_Transition T = new(false, IntSurf_In);
        BRepBlend_Extremity E1 = new(new Point3D(1, 0, 0), 0.0, 0.0, 1.0, 1.0e-4);
        E1.AddArc(A, 0.25, T, T);
        BRepBlend_Extremity E2 = E1;
        E2.AddArc(A, 0.75, T, T);
        Assert.Equal(1, E1.NbPointOnRst());
        Assert.Equal(2, E2.NbPointOnRst());
        Assert.NotEqual(E1, E2);

        BRepBlend_Extremity E3 = E1.WithArc(A, 0.75, T, T);
        Assert.Equal(E2, E3);
        Assert.Equal(E2.GetHashCode(), E3.GetHashCode());
        Assert.Equal(1, E1.NbPointOnRst());

        BRepBlend_Line L = new();
        L.SetStartPoints(E1, E2);
        E1.AddArc(A, 0.5, T, T);
        Assert.Equal(1, L.StartPointOnFirst().NbPointOnRst());
        Assert.Equal(2, L.StartPointOnSecond().NbPointOnRst());
        L.SetEndPoints(E2, E1);
        Assert.Equal(2, L.EndPointOnSecond().NbPointOnRst());
    }

    [Fact]
    public void PointOnRst_SetArc()
    {
        BRepBlend_PointOnRst P = new();
        Assert.Equal(0.0, P.ParameterOnArc());
        Assert.Equal(IntSurf_Undecided, P.TransitionOnLine().TransitionType());
        Adaptor2d_Curve2d A = Segment(0, 0, 2, 0);
        P.SetArc(A, 1.5, new IntSurf_Transition(true, IntSurf_Out), new IntSurf_Transition(false, IntSurf_In));
        Assert.Same(A, P.Arc());
        Assert.Equal(1.5, P.ParameterOnArc());
        Assert.True(P.TransitionOnLine().IsTangent());
        Assert.Equal(IntSurf_In, P.TransitionOnArc().TransitionType());
        Assert.Equal(P, new BRepBlend_PointOnRst().WithArc(A, 1.5, P.TransitionOnLine(), P.TransitionOnArc()));
    }

    [Fact]
    public void CurveTools_NbSamples()
    {
        Assert.Equal(2, BRepBlend_HCurve2dTool.NbSamples(Segment(0, 0, 1, 0), 0, 1));
        Assert.Equal(2, BRepBlend_HCurveTool.NbSamples(BlendRecorder.Guide(0), 0, 10));
        Assert.Equal(10, BRepBlend_HCurveTool.NbSamples(BlendRecorder.Guide(1), 0, 1));

        NCollection_Array1<Point2D> P2 = new(1, 4);
        P2[1] = new Point2D(0, 0);
        P2[2] = new Point2D(1, 1);
        P2[3] = new Point2D(2, 0);
        P2[4] = new Point2D(3, 1);
        NCollection_Array1<double> K = new(1, 2);
        K[1] = 0.0;
        K[2] = 1.0;
        NCollection_Array1<int> M = new(1, 2);
        M[1] = 4;
        M[2] = 4;
        Geom2dAdaptor_Curve B = new(new Geom2d_BSplineCurve(P2, K, M, 3));
        // 2 knots x degree 3 x the curve's range over the range asked for
        Assert.Equal(12, BRepBlend_HCurve2dTool.NbSamples(B, 0.0, 0.5));
        Assert.Equal(2, BRepBlend_HCurve2dTool.NbSamples(B, 0.0, 10.0));
        Assert.Equal(50, BRepBlend_HCurve2dTool.NbSamples(B, 0.0, 0.01));
        Assert.Equal(7, BRepBlend_HCurve2dTool.NbSamples(new Geom2dAdaptor_Curve(new Geom2d_BezierCurve(P2)), 0, 1));
    }

    [Fact]
    public void BlendTool_ProjectAndInters()
    {
        Adaptor2d_Curve2d A = Segment(0, 1, 4, 1); // y = 1, 0 <= x <= 4
        Assert.True(BRepBlend_BlendTool.Project(new Point2D(1.5, 3), null, A, out double prm, out double dist));
        Assert.Equal(1.5, prm, 1.0e-9);
        Assert.Equal(2.0, dist, 1.0e-9);
        // Beyond the end: the end.
        Assert.True(BRepBlend_BlendTool.Project(new Point2D(6, 1), null, A, out prm, out dist));
        Assert.Equal(4.0, prm, 1.0e-12);
        Assert.Equal(2.0, dist, 1.0e-12);

        double p = -1, d = -1;
        Assert.True(BRepBlend_BlendTool.Inters(new Point2D(2, 0), new Point2D(2, 2), null, A, ref p, ref d));
        Assert.Equal(2.0, p, 1.0e-9);
        Assert.Equal(1.0, d, 1.0e-9);
        // No intersection, or a degenerate segment: unchanged.
        p = -1;
        d = -1;
        Assert.False(BRepBlend_BlendTool.Inters(new Point2D(5, 0), new Point2D(5, 0.5), null, A, ref p, ref d));
        Assert.False(BRepBlend_BlendTool.Inters(new Point2D(2, 0), new Point2D(2, 1.0e-9), null, A, ref p, ref d));
        Assert.Equal((-1.0, -1.0), (p, d));

        Adaptor3d_HVertex V = new(new Point2D(4, 1), TopAbs_Orientation.TopAbs_REVERSED, 1.0e-6);
        Assert.Equal(4.0, BRepBlend_BlendTool.Parameter(V, A), 1.0e-12);
        Assert.Equal(1.0e-6, BRepBlend_BlendTool.Tolerance(V, A));
        BRepBlend_BlendTool.Bounds(A, out double f, out double l);
        Assert.Equal((0.0, 4.0), (f, l));
        Assert.Same(A, BRepBlend_BlendTool.CurveOnSurf(A, BlendRecorder.Surf(0)));
        Assert.Equal(10, BRepBlend_BlendTool.NbSamplesU(BlendRecorder.Surf(0), 0, 1));
        Assert.False(BRepBlend_BlendTool.SingularOnUMin(BlendRecorder.Surf(0)));
    }

    // A walked constant-radius line between two planes (the floor z = 0 and the wall y = 0) and its function.
    private static (BRepBlend_Line, BlendFunc_ConstRad) PlanePlaneLine()
    {
        Adaptor3d_Surface S1 = new GeomAdaptor_Surface(((GeomAdaptor_Surface)BlendRecorder.Surf(0)).Surface()!, 0, 8, 0,
                                                       5);
        Adaptor3d_Surface S2 = new GeomAdaptor_Surface(((GeomAdaptor_Surface)BlendRecorder.Surf(1)).Surface()!, 1, 10,
                                                       -5, 0);
        ChFiDS_ElSpine G = new();
        G.SetCurve(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        G.FirstParameter(0.0);
        G.LastParameter(10.0);
        BlendFunc_ConstRad F = new(S1, S2, G);
        F.Set(1.0, 1);
        BlendFunc_ConstRadInv FInv = new(S1, S2, G);
        FInv.Set(1.0, 1);
        BRepBlend_Walking W = new(S1, S2, new Adaptor3d_TopolTool(S1), new Adaptor3d_TopolTool(S2), G);
        Assert.ThrowsAny<NotDoneException>(() => W.Line());
        Assert.ThrowsAny<NotDoneException>(() => W.Complete(F, FInv, 0.0));
        math_Vector X0 = new(1, 4);
        X0[1] = 4;
        X0[2] = 1;
        X0[3] = 4;
        X0[4] = -1;
        W.Check2d(false);
        W.Perform(F, FInv, 4.0, 10.0, 1.0, 1.0e-4, 1.0e-5, X0, 0.01, true);
        Assert.True(W.IsDone());
        Assert.True(W.Complete(F, FInv, 0.0));
        return (W.Line(), F);
    }

    [Fact]
    public void Walking_PlanePlane_ReachesBothRestrictions()
    {
        (BRepBlend_Line L, _) = PlanePlaneLine();
        Assert.True(L.NbPoints() >= 6);
        // The fillet of radius 1 in the corner: points (x, 1, 0) on the floor and (x, 0, 1) on the wall.
        for (int i = 1; i <= L.NbPoints(); i++)
        {
            Blend_Point P = L.Point(i);
            Assert.True(P.PointOnS1().IsEqual(new Point3D(P.Parameter(), 1, 0), 1.0e-6));
            Assert.True(P.PointOnS2().IsEqual(new Point3D(P.Parameter(), 0, 1), 1.0e-6));
            if (i > 1)
            {
                Assert.True(P.Parameter() > L.Point(i - 1).Parameter());
            }
        }
        // It stops on the wall's restriction u = 1 going down, and on the floor's u = 8 going up.
        Assert.Equal(1.0, L.Point(1).Parameter(), 1.0e-6);
        Assert.Equal(8.0, L.Point(L.NbPoints()).Parameter(), 1.0e-6);
        Assert.Equal(1, L.StartPointOnSecond().NbPointOnRst());
        Assert.Equal(1, L.EndPointOnFirst().NbPointOnRst());
    }

    [Fact]
    public void AppFunc_InsertsSlowPointsIntoTheLine()
    {
        (BRepBlend_Line L, BlendFunc_ConstRad F) = PlanePlaneLine();
        BRepBlend_AppFunc AF = new(L, F, 1.0e-4, 1.0e-5);
        AF.SectionShape(out int aNbPoles, out _, out _);
        NCollection_Array1<Point3D> P = new(1, aNbPoles);
        NCollection_Array1<Point2D> P2 = new(1, AF.Nb2dCurves());
        NCollection_Array1<double> W = new(1, aNbPoles);
        // Outside the line: false.
        Assert.False(AF.D0(L.Point(1).Parameter() - 0.5, 0, 10, P, P2, W));
        Assert.False(AF.D0(L.Point(L.NbPoints()).Parameter() + 0.5, 0, 10, P, P2, W));
        // On a point and between two: the section of radius 1.
        int n = L.NbPoints();
        Assert.True(AF.D0(L.Point(2).Parameter(), 0, 10, P, P2, W));
        Assert.Equal(n, L.NbPoints());
        double t = 0.5 * (L.Point(2).Parameter() + L.Point(3).Parameter());
        Assert.True(AF.D0(t, 0, 10, P, P2, W));
        Assert.True(P[1].IsEqual(new Point3D(t, 1, 0), 1.0e-6));
        Assert.True(P[aNbPoles].IsEqual(new Point3D(t, 0, 1), 1.0e-6));
        // A planar case converges at once (no insertion); the line is ordered either way.
        Assert.True(L.NbPoints() == n || L.NbPoints() == n + 1);
        for (int i = 2; i <= L.NbPoints(); i++)
        {
            Assert.True(L.Point(i).Parameter() > L.Point(i - 1).Parameter());
        }

        // The surface: approximated within the tolerance, its 2D curves on the floor and the wall.
        BRepBlend_AppSurface A = new(AF, L.Point(1).Parameter(), L.Point(L.NbPoints()).Parameter(), 1.0e-4, 1.0e-5,
                                     1.0e-2, C1);
        Assert.True(A.IsDone());
        Assert.Equal(2, A.NbCurves2d());
        A.TolReached(out double t3, out double t2);
        Assert.True(t3 < 1.0e-4 && t2 < 1.0e-5);
    }

    [Fact]
    public void CSWalking_IsDoneAndLine()
    {
        // OCCT 8 declares CSWalking's IsDone and Line without defining them; the port defines them as the other
        // walkers' (decision 1059).
        Adaptor3d_Surface S = new GeomAdaptor_Surface(((GeomAdaptor_Surface)BlendRecorder.Surf(0)).Surface()!, 0, 8, -3,
                                                      3);
        Adaptor3d_Curve C = BlendRecorder.Guide(4);
        ChFiDS_ElSpine G = new();
        G.SetCurve(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        G.FirstParameter(0.0);
        G.LastParameter(10.0);
        BlendFunc_CSConstRad F = new(S, C, G);
        F.Set(1.0, 1);
        BRepBlend_CSWalking W = new(C, S, new Adaptor3d_TopolTool(S));
        Assert.False(W.IsDone());
        Assert.ThrowsAny<NotDoneException>(() => W.Line());
        math_Vector X3 = new(1, 3);
        X3[1] = 3.0;
        X3[2] = -0.5;
        X3[3] = 3.0;
        W.Perform(F, 3.0, 9.0, 1.0, 1.0e-4, 1.0e-5, X3, 0.01, true);
        Assert.True(W.IsDone());
        Assert.True(W.Complete(F, 0.0));
        Assert.True(W.Line().NbPoints() >= 5);
    }

    [Fact]
    public void Typedefs_AreBlendFuncs()
    {
        // BRepBlend's typedef headers name BlendFunc's functions, written out (decision 1059): BRepBlend_AppSurf is
        // AppBlend_AppSurf over Blend_AppFunction and BRepBlend_Line.
        Assert.True(typeof(AppBlend_AppSurf<Blend_AppFunction, BRepBlend_Line, Blend_Point>)
                        .IsAssignableFrom(typeof(BRepBlend_AppSurf)));
        Assert.True(typeof(IAppBlend_Line<Blend_Point>).IsAssignableFrom(typeof(BRepBlend_Line)));
    }
}
