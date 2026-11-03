// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// A minimal adaptor over a Geom curve, for tests until GeomAdaptor_Curve (unit 2A) is ported: evaluation through the
/// curve, the elementary types and the B-spline and Bezier queries. Continuity and intervals are the curve's
/// whole range (one interval).
/// </summary>
internal sealed class TestCurveAdaptor(Curve3D theCurve, double theFirst, double theLast) : CurveAdaptorBase
{
    public TestCurveAdaptor(Curve3D theCurve) : this(theCurve, theCurve.FirstParameter, theCurve.LastParameter) { }

    public Curve3D Curve() => theCurve;

    public override CurveAdaptorBase ShallowCopy() => new TestCurveAdaptor(theCurve, theFirst, theLast);
    public override double FirstParameter() => theFirst;
    public override double LastParameter() => theLast;
    public override Continuity Continuity() => theCurve.Continuity;
    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = theFirst;
        T[T.Lower() + 1] = theLast;
    }

    public override CurveAdaptorBase Trim(double First, double Last, double Tol) => new TestCurveAdaptor(theCurve, First, Last);
    public override bool IsClosed => theCurve.IsClosed;
    public override bool IsPeriodic => theCurve.IsPeriodic;
    public override double Period() => theCurve.Period;
    public override double Resolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);

    public override CurveKind GetType() => theCurve switch
    {
        LineCurve3D => global::Kerf.Geometry.CurveKind.Line,
        CircleCurve3D => global::Kerf.Geometry.CurveKind.Circle,
        EllipseCurve3D => global::Kerf.Geometry.CurveKind.Ellipse,
        HyperbolaCurve3D => global::Kerf.Geometry.CurveKind.Hyperbola,
        ParabolaCurve3D => global::Kerf.Geometry.CurveKind.Parabola,
        BezierCurve3D => BezierCurve,
        BSplineCurve3D => BSplineCurve,
        OffsetCurve3D => global::Kerf.Geometry.CurveKind.OffsetCurve,
        _ => OtherCurve,
    };

    public override Line3D Line() => ((LineCurve3D)theCurve).Line;
    public override Circle3D Circle() => ((CircleCurve3D)theCurve).Circle;
    public override Ellipse3D Ellipse() => ((EllipseCurve3D)theCurve).Ellipse;
    public override Hyperbola3D Hyperbola() => ((HyperbolaCurve3D)theCurve).Hyperbola;
    public override Parabola3D Parabola() => ((ParabolaCurve3D)theCurve).Parabola;
    public override int Degree() => theCurve is BezierCurve3D B ? B.Degree : ((BSplineCurve3D)theCurve).Degree;
    public override bool IsRational => theCurve is BezierCurve3D B ? B.IsRational : theCurve is BSplineCurve3D S && S.IsRational;
    public override int PoleCount => theCurve is BezierCurve3D B ? B.PoleCount : ((BSplineCurve3D)theCurve).PoleCount;
    public override int KnotCount => ((BSplineCurve3D)theCurve).KnotCount;
    public override BezierCurve3D Bezier() => (BezierCurve3D)theCurve;
    public override BSplineCurve3D BSpline() => (BSplineCurve3D)theCurve;
    public override Point3D EvalD0(double theU) => theCurve.EvalD0(theU);
    internal override Curve3D.ResD1 EvalD1(double theU) => theCurve.EvalD1(theU);
    internal override Curve3D.ResD2 EvalD2(double theU) => theCurve.EvalD2(theU);
    internal override Curve3D.ResD3 EvalD3(double theU) => theCurve.EvalD3(theU);
    public override Vector3D EvalDN(double theU, int theN) => theCurve.EvalDN(theU, theN);
}

/// <summary>
/// A minimal adaptor over a Geom surface between given bounds, for tests until GeomAdaptor_Surface (unit 2A) is
/// ported: evaluation through the surface, the type queries and the elementary, swept, Bezier and B-spline data.
/// Continuity and intervals are the whole range (one interval in each direction).
/// </summary>
internal sealed class TestSurfaceAdaptor : SurfaceAdaptorBase
{
    private readonly Surface mySurf;
    private readonly double myU1, myU2, myV1, myV2;

    public TestSurfaceAdaptor(Surface theSurf, double theU1, double theU2, double theV1, double theV2)
    {
        mySurf = theSurf;
        myU1 = theU1;
        myU2 = theU2;
        myV1 = theV1;
        myV2 = theV2;
    }

    public TestSurfaceAdaptor(Surface theSurf)
    {
        mySurf = theSurf;
        theSurf.ParameterBounds(out myU1, out myU2, out myV1, out myV2);
    }

    public Surface Surface() => mySurf;

    public override SurfaceAdaptorBase ShallowCopy() => new TestSurfaceAdaptor(mySurf, myU1, myU2, myV1, myV2);
    public override double FirstUParameter() => myU1;
    public override double LastUParameter() => myU2;
    public override double FirstVParameter() => myV1;
    public override double LastVParameter() => myV2;
    public override Continuity UContinuity() => mySurf.Continuity;
    public override Continuity VContinuity() => mySurf.Continuity;

    /// <summary>Interior interval bounds in U (strictly between the first and last parameters), if any.</summary>
    public double[] UBreaks { get; init; } = [];

    /// <summary>Interior interval bounds in V (strictly between the first and last parameters), if any.</summary>
    public double[] VBreaks { get; init; } = [];

    public override int NbUIntervals(Continuity S) => UBreaks.Length + 1;
    public override int NbVIntervals(Continuity S) => VBreaks.Length + 1;

    private static void Fill(NCollection_Array1<double> T, double theFirst, double[] theBreaks, double theLast)
    {
        T[T.Lower()] = theFirst;
        for (int i = 0; i < theBreaks.Length; i++)
        {
            T[T.Lower() + 1 + i] = theBreaks[i];
        }
        T[T.Lower() + 1 + theBreaks.Length] = theLast;
    }

    public override void UIntervals(NCollection_Array1<double> T, Continuity S) => Fill(T, myU1, UBreaks, myU2);

    public override void VIntervals(NCollection_Array1<double> T, Continuity S) => Fill(T, myV1, VBreaks, myV2);

    public override SurfaceAdaptorBase UTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, First, Last, myV1, myV2);
    public override SurfaceAdaptorBase VTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, myU1, myU2, First, Last);
    public override bool IsUClosed => mySurf.IsUClosed;
    public override bool IsVClosed => mySurf.IsVClosed;
    public override bool IsUPeriodic => mySurf.IsUPeriodic;
    public override double UPeriod() => mySurf.UPeriod;
    public override bool IsVPeriodic => mySurf.IsVPeriodic;
    public override double VPeriod() => mySurf.VPeriod;
    public override double UResolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);
    public override double VResolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);

    public override SurfaceKind GetType() => mySurf switch
    {
        PlaneSurface => global::Kerf.Geometry.SurfaceKind.Plane,
        CylindricalSurface => global::Kerf.Geometry.SurfaceKind.Cylinder,
        ConicalSurface => global::Kerf.Geometry.SurfaceKind.Cone,
        SphericalSurface => global::Kerf.Geometry.SurfaceKind.Sphere,
        ToroidalSurface => global::Kerf.Geometry.SurfaceKind.Torus,
        global::Kerf.Geometry.BezierSurface => SurfaceKind.BezierSurface,
        global::Kerf.Geometry.BSplineSurface => SurfaceKind.BSplineSurface,
        RevolutionSurface => SurfaceOfRevolution,
        ExtrusionSurface => SurfaceOfExtrusion,
        _ => OtherSurface,
    };

    public override Plane3D Plane() => ((PlaneSurface)mySurf).Plane;
    public override Cylinder Cylinder() => ((CylindricalSurface)mySurf).Cylinder;
    public override Cone Cone() => ((ConicalSurface)mySurf).Cone;
    public override Sphere Sphere() => ((SphericalSurface)mySurf).Sphere;
    public override Torus Torus() => ((ToroidalSurface)mySurf).Torus;
    public override int UDegree() => mySurf is global::Kerf.Geometry.BezierSurface B ? B.UDegree : ((global::Kerf.Geometry.BSplineSurface)mySurf).UDegree;
    public override int VDegree() => mySurf is global::Kerf.Geometry.BezierSurface B ? B.VDegree : ((global::Kerf.Geometry.BSplineSurface)mySurf).VDegree;
    public override int UPoleCount => mySurf is global::Kerf.Geometry.BezierSurface B ? B.UPoleCount : ((global::Kerf.Geometry.BSplineSurface)mySurf).UPoleCount;
    public override int VPoleCount => mySurf is global::Kerf.Geometry.BezierSurface B ? B.VPoleCount : ((global::Kerf.Geometry.BSplineSurface)mySurf).VPoleCount;
    public override int UKnotCount => ((global::Kerf.Geometry.BSplineSurface)mySurf).UKnotCount;
    public override int VKnotCount => ((global::Kerf.Geometry.BSplineSurface)mySurf).VKnotCount;
    public override bool IsURational => mySurf is global::Kerf.Geometry.BezierSurface B ? B.IsURational : ((global::Kerf.Geometry.BSplineSurface)mySurf).IsURational;
    public override bool IsVRational => mySurf is global::Kerf.Geometry.BezierSurface B ? B.IsVRational : ((global::Kerf.Geometry.BSplineSurface)mySurf).IsVRational;
    public override global::Kerf.Geometry.BezierSurface Bezier() => (global::Kerf.Geometry.BezierSurface)mySurf;
    public override global::Kerf.Geometry.BSplineSurface BSpline() => (global::Kerf.Geometry.BSplineSurface)mySurf;
    public override Axis3D AxeOfRevolution() => ((RevolutionSurface)mySurf).Axis;
    public override Direction3D Direction() => ((SweptSurface)mySurf).Direction;
    public override CurveAdaptorBase BasisCurve() => new TestCurveAdaptor(((SweptSurface)mySurf).BasisCurve);
    public override Point3D EvalD0(double theU, double theV) => mySurf.EvalD0(theU, theV);
    internal override global::Kerf.Geometry.Surface.ResD1 EvalD1(double theU, double theV) => mySurf.EvalD1(theU, theV);
    internal override global::Kerf.Geometry.Surface.ResD2 EvalD2(double theU, double theV) => mySurf.EvalD2(theU, theV);
    internal override global::Kerf.Geometry.Surface.ResD3 EvalD3(double theU, double theV) => mySurf.EvalD3(theU, theV);
    public override Vector3D EvalDN(double theU, double theV, int theNu, int theNv) => mySurf.EvalDN(theU, theV, theNu, theNv);
}
