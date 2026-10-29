// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtCC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtCC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The extremum distance between two 2D curves nearest the starting parameters (U0, V0), by
/// <see cref="Extrema_LocECC2d"/> with the curves' resolutions of Precision::Confusion() as tolerances.
/// </summary>
/// <remarks>Computed by its constructor.</remarks>
public class Extrema_LocateExtCC2d
{
    private readonly bool myDone;
    private readonly double mySqDist;
    private readonly Extrema_POnCurv2d myPoint1 = new();
    private readonly Extrema_POnCurv2d myPoint2 = new();

    /// <summary>The extremum between C1 and C2 nearest (U0, V0).</summary>
    public Extrema_LocateExtCC2d(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, double U0, double V0)
    {
        mySqDist = RealLast();
        double TolU = Extrema_Curve2dTool.Resolution(C1, Precision.Confusion());
        double TolV = Extrema_Curve2dTool.Resolution(C2, Precision.Confusion());
        Extrema_POnCurv2d P1, P2;

        // Not implemented for now: the call to Geom2dExtrema_ExtCC.

        Extrema_LocECC2d Xtrem = new(C1, C2, U0, V0, TolU, TolV);
        // Exploitation

        myDone = Xtrem.IsDone();
        if (Xtrem.IsDone())
        {
            mySqDist = Xtrem.SquareDistance();
            Xtrem.Point(out P1, out P2);
            myPoint1 = P1;
            myPoint2 = P2;
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return mySqDist;
    }

    /// <summary>The points of the extremum, on the first and on the second curve.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public void Point(out Extrema_POnCurv2d P1, out Extrema_POnCurv2d P2)
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        P1 = myPoint1;
        P2 = myPoint2;
    }
}
