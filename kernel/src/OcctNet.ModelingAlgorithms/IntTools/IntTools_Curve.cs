// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// One 3D curve, its two 2D curves and two tolerances: a result of the face/face intersection. The 3D curve is the
/// intersection curve, the 2D curves its p-curves on the faces; the tolerance is the largest deviation between the 3D
/// curve and the 2D curves (or the surfaces), the tangential tolerance the largest distance from the 3D curve to the
/// end of the tangential zone between the faces.
/// </summary>
/// <remarks>
/// A struct, as OCCT's by-value class: a copy shares the curves (handles), as in C++. The setters are internal
/// (translation rules, section 3).
/// </remarks>
public partial struct IntTools_Curve
{
    private Geom_Curve? my3dCurve;
    private Geom2d_Curve? my2dCurve1;
    private Geom2d_Curve? my2dCurve2;
    private double myTolerance;
    private double myTangentialTolerance;

    /// <summary>An empty curve.</summary>
    public IntTools_Curve()
    {
        myTolerance = 0.0;
        myTangentialTolerance = 0.0;
    }

    /// <summary>A 3D curve, two 2D curves and two tolerances.</summary>
    public IntTools_Curve(Geom_Curve? the3dCurve, Geom2d_Curve? the2dCurve1, Geom2d_Curve? the2dCurve2,
                          double theTolerance = 0.0, double theTangentialTolerance = 0.0)
    {
        myTolerance = theTolerance;
        myTangentialTolerance = theTangentialTolerance;
        SetCurves(the3dCurve, the2dCurve1, the2dCurve2);
    }

    internal void SetCurves(Geom_Curve? the3dCurve, Geom2d_Curve? the2dCurve1, Geom2d_Curve? the2dCurve2)
    {
        my3dCurve = the3dCurve;
        my2dCurve1 = the2dCurve1;
        my2dCurve2 = the2dCurve2;
    }

    internal void SetCurve(Geom_Curve? the3dCurve) => my3dCurve = the3dCurve;

    internal void SetFirstCurve2d(Geom2d_Curve? the2dCurve1) => my2dCurve1 = the2dCurve1;

    internal void SetSecondCurve2d(Geom2d_Curve? the2dCurve2) => my2dCurve2 = the2dCurve2;

    internal void SetTolerance(double theTolerance) => myTolerance = theTolerance;

    internal void SetTangentialTolerance(double theTangentialTolerance) => myTangentialTolerance = theTangentialTolerance;

    /// <summary>The 3D curve.</summary>
    public readonly Geom_Curve? Curve() => my3dCurve;

    /// <summary>The first 2D curve.</summary>
    public readonly Geom2d_Curve? FirstCurve2d() => my2dCurve1;

    /// <summary>The second 2D curve.</summary>
    public readonly Geom2d_Curve? SecondCurve2d() => my2dCurve2;

    public readonly double Tolerance() => myTolerance;

    public readonly double TangentialTolerance() => myTangentialTolerance;

    /// <summary>True if the 3D curve is a bounded curve.</summary>
    public readonly bool HasBounds()
    {
        bool bIsBounded = my3dCurve is Geom_BoundedCurve;
        return bIsBounded;
    }

    /// <summary>
    /// If the 3D curve is bounded, its first and last parameters and points, and true; otherwise false, the outputs
    /// left as they were.
    /// </summary>
    public readonly bool Bounds(ref double theFirst, ref double theLast, ref Point3D theFirstPnt, ref Point3D theLastPnt)
    {
        bool bIsBounded = HasBounds();
        if (bIsBounded)
        {
            theFirst = my3dCurve!.FirstParameter();
            theLast = my3dCurve.LastParameter();
            my3dCurve.D0(theFirst, out theFirstPnt);
            my3dCurve.D0(theLast, out theLastPnt);
        }
        return bIsBounded;
    }

    /// <summary>
    /// The 3D point at the parameter, and true, if the parameter is inside the curve's bounds; otherwise false, the
    /// point left as it was.
    /// </summary>
    public readonly bool D0(double thePar, ref Point3D thePnt)
    {
        // OCCT: "||" where "&&" is meant, so every parameter is inside (occt-issues.md, unit 6A)
        bool bInside = thePar >= my3dCurve!.FirstParameter() || thePar <= my3dCurve.LastParameter();
        if (bInside)
        {
            my3dCurve.D0(thePar, out thePnt);
        }
        return bInside;
    }

    /// <summary>The type of the 3D curve.</summary>
    public readonly CurveKind Type()
    {
        GeomAdaptor_Curve aGAC = new(my3dCurve!);
        CurveKind aType = aGAC.GetType();
        return aType;
    }

    // ---- OCCT.NET additions: the setters' non-mutating twins

    public readonly IntTools_Curve WithCurves(Geom_Curve? the3dCurve, Geom2d_Curve? the2dCurve1, Geom2d_Curve? the2dCurve2)
    {
        IntTools_Curve aCopy = this;
        aCopy.SetCurves(the3dCurve, the2dCurve1, the2dCurve2);
        return aCopy;
    }

    public readonly IntTools_Curve WithTolerance(double theTolerance)
    {
        IntTools_Curve aCopy = this;
        aCopy.SetTolerance(theTolerance);
        return aCopy;
    }

    public readonly IntTools_Curve WithTangentialTolerance(double theTangentialTolerance)
    {
        IntTools_Curve aCopy = this;
        aCopy.SetTangentialTolerance(theTangentialTolerance);
        return aCopy;
    }
}
