// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntCurveSurface_TransitionOnCurve;

namespace Occt;

/// <summary>
/// An intersection point between a curve and a surface: the point, its parameters (U, V) on the surface and W on the
/// curve, and the transition of the curve through the surface.
/// </summary>
/// <remarks>
/// <c>new IntCurveSurface_IntersectionPoint()</c> and <c>default</c> are the same: the origin, zero parameters and a
/// tangent transition (OCCT's default). <c>SetValues</c> is an internal mutator; the constructor is its twin.
/// OCCT's <c>Dump</c> (to std::cout) is <see cref="ToString"/>.
/// </remarks>
public partial struct IntCurveSurface_IntersectionPoint : IEquatable<IntCurveSurface_IntersectionPoint>
{
    private gp_Pnt myP;
    private double myUSurf;
    private double myVSurf;
    private double myUCurv;
    private IntCurveSurface_TransitionOnCurve myTrOnCurv;

    public IntCurveSurface_IntersectionPoint()
    {
        myP = new gp_Pnt();
        myUSurf = 0.0;
        myVSurf = 0.0;
        myUCurv = 0.0;
        myTrOnCurv = IntCurveSurface_Tangent;
    }

    public IntCurveSurface_IntersectionPoint(in gp_Pnt P, double USurf, double VSurf, double UCurv,
                                             IntCurveSurface_TransitionOnCurve TrOnCurv)
    {
        myP = P;
        myUSurf = USurf;
        myVSurf = VSurf;
        myUCurv = UCurv;
        myTrOnCurv = TrOnCurv;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValues(in gp_Pnt P, double USurf, double VSurf, double UCurv,
                            IntCurveSurface_TransitionOnCurve TrOnCurv)
    {
        myP = P;
        myUSurf = USurf;
        myVSurf = VSurf;
        myUCurv = UCurv;
        myTrOnCurv = TrOnCurv;
    }

    // Queries

    /// <summary>All the values of the point.</summary>
    public readonly void Values(out gp_Pnt P, out double USurf, out double VSurf, out double UCurv,
                                out IntCurveSurface_TransitionOnCurve TrOnCurv)
    {
        P = myP;
        USurf = myUSurf;
        VSurf = myVSurf;
        UCurv = myUCurv;
        TrOnCurv = myTrOnCurv;
    }

    /// <summary>The point.</summary>
    public readonly gp_Pnt Pnt() => myP;

    /// <summary>The U parameter on the surface.</summary>
    public readonly double U() => myUSurf;

    /// <summary>The V parameter on the surface.</summary>
    public readonly double V() => myVSurf;

    /// <summary>The parameter on the curve.</summary>
    public readonly double W() => myUCurv;

    /// <summary>The transition of the curve through the surface.</summary>
    public readonly IntCurveSurface_TransitionOnCurve Transition() => myTrOnCurv;

    public readonly bool Equals(IntCurveSurface_IntersectionPoint other)
        => myP.Equals(other.myP) && myUSurf.Equals(other.myUSurf) && myVSurf.Equals(other.myVSurf)
           && myUCurv.Equals(other.myUCurv) && myTrOnCurv == other.myTrOnCurv;

    public override readonly bool Equals(object? obj) => obj is IntCurveSurface_IntersectionPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myP, myUSurf, myVSurf, myUCurv, myTrOnCurv);

    public static bool operator ==(in IntCurveSurface_IntersectionPoint theLeft,
                                   in IntCurveSurface_IntersectionPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntCurveSurface_IntersectionPoint theLeft,
                                   in IntCurveSurface_IntersectionPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{gp.Format("IntCurveSurface_IntersectionPoint", myP.X(), myP.Y(), myP.Z(), myUSurf, myVSurf, myUCurv)} {myTrOnCurv}";
}
