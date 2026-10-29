// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Hypr2d: from a center and two points (one on the major axis), from a major axis and radii,
/// or from an axis system and radii. The major radius may be lower than the minor radius.
/// </summary>
public sealed class gce_MakeHypr2d : gce_Root
{
    private gp_Hypr2d TheHypr2d = new();

    /// <summary>
    /// A hyperbola from its center and two points: Center-S1 is the major axis and its length the major radius, and
    /// the distance from S2 to that axis is the minor radius. Fails with gce_ConfusedPoints if two of the points
    /// coincide and gce_ColinearPoints if they are collinear.
    /// </summary>
    public gce_MakeHypr2d(in gp_Pnt2d S1, in gp_Pnt2d S2, in gp_Pnt2d Center)
    {
        gp_XY aVecS1 = S1.XY() - Center.XY();
        gp_XY aVecS2 = S2.XY() - Center.XY();
        double aDistS1 = aVecS1.Modulus();
        double aDistS2 = aVecS2.Modulus();
        if (aDistS1 <= gp.Resolution() || aDistS2 <= gp.Resolution()
            || S1.Distance(S2) <= gp.Resolution())
        {
            TheError = gce_ConfusedPoints;
            return;
        }
        gp_Dir2d aXAxis = new(aVecS1);
        gp_Dir2d aYAxis = new(aVecS2);
        gp_Ax22d anAxis = new(Center, aXAxis, aYAxis);
        gp_Lin2d aLine = new(Center, aXAxis);
        double aMinorDist = aLine.Distance(S2);
        if (aMinorDist <= gp.Resolution())
        {
            TheError = gce_ColinearPoints;
            return;
        }
        TheHypr2d = new gp_Hypr2d(anAxis, aDistS1, aMinorDist);
        TheError = gce_Done;
    }

    /// <summary>
    /// A hyperbola from its major axis (whose location is the center), its radii and a sense (false uses the opposite
    /// direction of MajorAxis); fails with gce_NegativeRadius if a radius is negative.
    /// </summary>
    public gce_MakeHypr2d(in gp_Ax2d MajorAxis, double MajorRadius, double MinorRadius, bool Sense)
    {
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheHypr2d = new gp_Hypr2d(MajorAxis, MajorRadius, MinorRadius, Sense);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A hyperbola from its axis system (whose location is the center, whose X axis is the major axis) and its radii;
    /// fails with gce_NegativeRadius if a radius is negative.
    /// </summary>
    public gce_MakeHypr2d(in gp_Ax22d A, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheHypr2d = new gp_Hypr2d(A, MajorRadius, MinorRadius);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed hyperbola.</summary>
    /// <exception cref="StdFail_NotDone">No hyperbola was constructed.</exception>
    public gp_Hypr2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeHypr2d::Value() - no result");
        return TheHypr2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Hypr2d Operator() => Value();

    /// <summary>The constructed hyperbola (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Hypr2d(gce_MakeHypr2d theMaker) => theMaker.Operator();
}
