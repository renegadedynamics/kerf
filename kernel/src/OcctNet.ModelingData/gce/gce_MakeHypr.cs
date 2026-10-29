// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Hypr: from a center and two points (one on the major axis), or from an axis system and
/// radii. The X axis of the hyperbola is its major axis, the Y axis its minor axis; the major radius may be lower
/// than the minor radius.
/// </summary>
public sealed class gce_MakeHypr : gce_Root
{
    private gp_Hypr TheHypr = new();

    /// <summary>
    /// A hyperbola from its center and two points: Center-S1 is the major axis and its length the major radius, and
    /// the distance from S2 to that axis is the minor radius. Fails with gce_ConfusedPoints if two of the points
    /// coincide and gce_ColinearPoints if they are collinear.
    /// </summary>
    public gce_MakeHypr(in gp_Pnt S1, in gp_Pnt S2, in gp_Pnt Center)
    {
        gp_XYZ aVecS1 = S1.XYZ() - Center.XYZ();
        gp_XYZ aVecS2 = S2.XYZ() - Center.XYZ();
        double aDistS1 = aVecS1.Modulus();
        double aDistS2 = aVecS2.Modulus();
        if (aDistS1 <= gp.Resolution() || aDistS2 <= gp.Resolution()
            || S1.Distance(S2) <= gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
            return;
        }
        gp_Dir aXAxis = new(aVecS1);
        gp_Lin aLine = new(Center, aXAxis);
        double aMinorDist = aLine.Distance(S2);
        if (aMinorDist <= gp.Resolution())
        {
            TheError = gce_ColinearPoints;
            return;
        }
        gp_XYZ aCross = aXAxis.XYZ().Crossed(aVecS2);
        if (aCross.Modulus() <= gp.Resolution())
        {
            TheError = gce_ColinearPoints;
            return;
        }
        gp_Dir aNorm = new(aCross);
        TheHypr = new gp_Hypr(new gp_Ax2(Center, aNorm, aXAxis), aDistS1, aMinorDist);
        TheError = gce_Done;
    }

    /// <summary>
    /// A hyperbola X*X / (MajorRadius*MajorRadius) - Y*Y / (MinorRadius*MinorRadius) = 1 in the system A2; fails with
    /// gce_NegativeRadius if a radius is negative.
    /// </summary>
    public gce_MakeHypr(in gp_Ax2 A2, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheHypr = new gp_Hypr(A2, MajorRadius, MinorRadius);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed hyperbola.</summary>
    /// <exception cref="StdFail_NotDone">No hyperbola was constructed.</exception>
    public gp_Hypr Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeHypr::Value() - no result");
        return TheHypr;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Hypr Operator() => Value();

    /// <summary>The constructed hyperbola (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Hypr(gce_MakeHypr theMaker) => theMaker.Operator();
}
