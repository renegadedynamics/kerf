// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// A minimal adaptor over a Geom curve, for tests until GeomAdaptor_Curve (unit 2A) is ported: evaluation through the
/// curve, the elementary types and the B-spline and Bezier queries. Continuity and intervals are the curve's
/// whole range (one interval).
/// </summary>
internal sealed class TestCurveAdaptor(Geom_Curve theCurve, double theFirst, double theLast) : Adaptor3d_Curve
{
    public TestCurveAdaptor(Geom_Curve theCurve) : this(theCurve, theCurve.FirstParameter(), theCurve.LastParameter()) { }

    public Geom_Curve Curve() => theCurve;

    public override Adaptor3d_Curve ShallowCopy() => new TestCurveAdaptor(theCurve, theFirst, theLast);
    public override double FirstParameter() => theFirst;
    public override double LastParameter() => theLast;
    public override Continuity Continuity() => theCurve.Continuity();
    public override int NbIntervals(Continuity S) => 1;

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[T.Lower()] = theFirst;
        T[T.Lower() + 1] = theLast;
    }

    public override Adaptor3d_Curve Trim(double First, double Last, double Tol) => new TestCurveAdaptor(theCurve, First, Last);
    public override bool IsClosed() => theCurve.IsClosed();
    public override bool IsPeriodic() => theCurve.IsPeriodic();
    public override double Period() => theCurve.Period();
    public override double Resolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);

    public override CurveKind GetType() => theCurve switch
    {
        Geom_Line => global::Kerf.Geometry.CurveKind.Line,
        Geom_Circle => global::Kerf.Geometry.CurveKind.Circle,
        Geom_Ellipse => global::Kerf.Geometry.CurveKind.Ellipse,
        Geom_Hyperbola => global::Kerf.Geometry.CurveKind.Hyperbola,
        Geom_Parabola => global::Kerf.Geometry.CurveKind.Parabola,
        Geom_BezierCurve => BezierCurve,
        Geom_BSplineCurve => BSplineCurve,
        Geom_OffsetCurve => global::Kerf.Geometry.CurveKind.OffsetCurve,
        _ => OtherCurve,
    };

    public override Line3D Line() => ((Geom_Line)theCurve).Lin();
    public override Circle3D Circle() => ((Geom_Circle)theCurve).Circ();
    public override Ellipse3D Ellipse() => ((Geom_Ellipse)theCurve).Elips();
    public override Hyperbola3D Hyperbola() => ((Geom_Hyperbola)theCurve).Hypr();
    public override Parabola3D Parabola() => ((Geom_Parabola)theCurve).Parab();
    public override int Degree() => theCurve is Geom_BezierCurve B ? B.Degree() : ((Geom_BSplineCurve)theCurve).Degree();
    public override bool IsRational() => theCurve is Geom_BezierCurve B ? B.IsRational() : theCurve is Geom_BSplineCurve S && S.IsRational();
    public override int NbPoles() => theCurve is Geom_BezierCurve B ? B.NbPoles() : ((Geom_BSplineCurve)theCurve).NbPoles();
    public override int NbKnots() => ((Geom_BSplineCurve)theCurve).NbKnots();
    public override Geom_BezierCurve Bezier() => (Geom_BezierCurve)theCurve;
    public override Geom_BSplineCurve BSpline() => (Geom_BSplineCurve)theCurve;
    public override Point3D EvalD0(double theU) => theCurve.EvalD0(theU);
    public override Geom_Curve.ResD1 EvalD1(double theU) => theCurve.EvalD1(theU);
    public override Geom_Curve.ResD2 EvalD2(double theU) => theCurve.EvalD2(theU);
    public override Geom_Curve.ResD3 EvalD3(double theU) => theCurve.EvalD3(theU);
    public override Vector3D EvalDN(double theU, int theN) => theCurve.EvalDN(theU, theN);
}

/// <summary>
/// A minimal adaptor over a Geom surface between given bounds, for tests until GeomAdaptor_Surface (unit 2A) is
/// ported: evaluation through the surface, the type queries and the elementary, swept, Bezier and B-spline data.
/// Continuity and intervals are the whole range (one interval in each direction).
/// </summary>
internal sealed class TestSurfaceAdaptor : Adaptor3d_Surface
{
    private readonly Geom_Surface mySurf;
    private readonly double myU1, myU2, myV1, myV2;

    public TestSurfaceAdaptor(Geom_Surface theSurf, double theU1, double theU2, double theV1, double theV2)
    {
        mySurf = theSurf;
        myU1 = theU1;
        myU2 = theU2;
        myV1 = theV1;
        myV2 = theV2;
    }

    public TestSurfaceAdaptor(Geom_Surface theSurf)
    {
        mySurf = theSurf;
        theSurf.Bounds(out myU1, out myU2, out myV1, out myV2);
    }

    public Geom_Surface Surface() => mySurf;

    public override Adaptor3d_Surface ShallowCopy() => new TestSurfaceAdaptor(mySurf, myU1, myU2, myV1, myV2);
    public override double FirstUParameter() => myU1;
    public override double LastUParameter() => myU2;
    public override double FirstVParameter() => myV1;
    public override double LastVParameter() => myV2;
    public override Continuity UContinuity() => mySurf.Continuity();
    public override Continuity VContinuity() => mySurf.Continuity();

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

    public override Adaptor3d_Surface UTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, First, Last, myV1, myV2);
    public override Adaptor3d_Surface VTrim(double First, double Last, double Tol) => new TestSurfaceAdaptor(mySurf, myU1, myU2, First, Last);
    public override bool IsUClosed() => mySurf.IsUClosed();
    public override bool IsVClosed() => mySurf.IsVClosed();
    public override bool IsUPeriodic() => mySurf.IsUPeriodic();
    public override double UPeriod() => mySurf.UPeriod();
    public override bool IsVPeriodic() => mySurf.IsVPeriodic();
    public override double VPeriod() => mySurf.VPeriod();
    public override double UResolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);
    public override double VResolution(double R3d) => global::Kerf.Engine.Foundation.Precision.Parametric(R3d);

    public override SurfaceKind GetType() => mySurf switch
    {
        Geom_Plane => global::Kerf.Geometry.SurfaceKind.Plane,
        Geom_CylindricalSurface => global::Kerf.Geometry.SurfaceKind.Cylinder,
        Geom_ConicalSurface => global::Kerf.Geometry.SurfaceKind.Cone,
        Geom_SphericalSurface => global::Kerf.Geometry.SurfaceKind.Sphere,
        Geom_ToroidalSurface => global::Kerf.Geometry.SurfaceKind.Torus,
        Geom_BezierSurface => BezierSurface,
        Geom_BSplineSurface => BSplineSurface,
        Geom_SurfaceOfRevolution => SurfaceOfRevolution,
        Geom_SurfaceOfLinearExtrusion => SurfaceOfExtrusion,
        _ => OtherSurface,
    };

    public override Plane3D Plane() => ((Geom_Plane)mySurf).Pln();
    public override Cylinder Cylinder() => ((Geom_CylindricalSurface)mySurf).Cylinder();
    public override Cone Cone() => ((Geom_ConicalSurface)mySurf).Cone();
    public override Sphere Sphere() => ((Geom_SphericalSurface)mySurf).Sphere();
    public override Torus Torus() => ((Geom_ToroidalSurface)mySurf).Torus();
    public override int UDegree() => mySurf is Geom_BezierSurface B ? B.UDegree() : ((Geom_BSplineSurface)mySurf).UDegree();
    public override int VDegree() => mySurf is Geom_BezierSurface B ? B.VDegree() : ((Geom_BSplineSurface)mySurf).VDegree();
    public override int NbUPoles() => mySurf is Geom_BezierSurface B ? B.NbUPoles() : ((Geom_BSplineSurface)mySurf).NbUPoles();
    public override int NbVPoles() => mySurf is Geom_BezierSurface B ? B.NbVPoles() : ((Geom_BSplineSurface)mySurf).NbVPoles();
    public override int NbUKnots() => ((Geom_BSplineSurface)mySurf).NbUKnots();
    public override int NbVKnots() => ((Geom_BSplineSurface)mySurf).NbVKnots();
    public override bool IsURational() => mySurf is Geom_BezierSurface B ? B.IsURational() : ((Geom_BSplineSurface)mySurf).IsURational();
    public override bool IsVRational() => mySurf is Geom_BezierSurface B ? B.IsVRational() : ((Geom_BSplineSurface)mySurf).IsVRational();
    public override Geom_BezierSurface Bezier() => (Geom_BezierSurface)mySurf;
    public override Geom_BSplineSurface BSpline() => (Geom_BSplineSurface)mySurf;
    public override Axis3D AxeOfRevolution() => ((Geom_SurfaceOfRevolution)mySurf).Axis();
    public override Direction3D Direction() => ((Geom_SweptSurface)mySurf).Direction();
    public override Adaptor3d_Curve BasisCurve() => new TestCurveAdaptor(((Geom_SweptSurface)mySurf).BasisCurve());
    public override Point3D EvalD0(double theU, double theV) => mySurf.EvalD0(theU, theV);
    public override Geom_Surface.ResD1 EvalD1(double theU, double theV) => mySurf.EvalD1(theU, theV);
    public override Geom_Surface.ResD2 EvalD2(double theU, double theV) => mySurf.EvalD2(theU, theV);
    public override Geom_Surface.ResD3 EvalD3(double theU, double theV) => mySurf.EvalD3(theU, theV);
    public override Vector3D EvalDN(double theU, double theV, int theNu, int theNv) => mySurf.EvalDN(theU, theV, theNu, theNv);
}
