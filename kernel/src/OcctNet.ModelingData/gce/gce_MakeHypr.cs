// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Hypr: from a center and two points (one on the major axis), or from an axis system and
/// radii. The X axis of the hyperbola is its major axis, the Y axis its minor axis; the major radius may be lower
/// than the minor radius.
/// </summary>
public sealed class gce_MakeHypr : gce_Root
{
    private Hyperbola3D TheHypr = new();

    /// <summary>
    /// A hyperbola from its center and two points: Center-S1 is the major axis and its length the major radius, and
    /// the distance from S2 to that axis is the minor radius. Fails with gce_ConfusedPoints if two of the points
    /// coincide and gce_ColinearPoints if they are collinear.
    /// </summary>
    public gce_MakeHypr(in Point3D S1, in Point3D S2, in Point3D Center)
    {
        Xyz aVecS1 = S1.XYZ() - Center.XYZ();
        Xyz aVecS2 = S2.XYZ() - Center.XYZ();
        double aDistS1 = aVecS1.Modulus();
        double aDistS2 = aVecS2.Modulus();
        if (aDistS1 <= Constants.Resolution() || aDistS2 <= Constants.Resolution()
            || S1.Distance(S2) <= Constants.Resolution())
        {
            TheError = gce_ConfusedPoints;
            return;
        }
        Direction3D aXAxis = new(aVecS1);
        Line3D aLine = new(Center, aXAxis);
        double aMinorDist = aLine.Distance(S2);
        if (aMinorDist <= Constants.Resolution())
        {
            TheError = gce_ColinearPoints;
            return;
        }
        Xyz aCross = aXAxis.XYZ().Crossed(aVecS2);
        if (aCross.Modulus() <= Constants.Resolution())
        {
            TheError = gce_ColinearPoints;
            return;
        }
        Direction3D aNorm = new(aCross);
        TheHypr = new Hyperbola3D(new Frame3D(Center, aNorm, aXAxis), aDistS1, aMinorDist);
        TheError = gce_Done;
    }

    /// <summary>
    /// A hyperbola X*X / (MajorRadius*MajorRadius) - Y*Y / (MinorRadius*MinorRadius) = 1 in the system A2; fails with
    /// gce_NegativeRadius if a radius is negative.
    /// </summary>
    public gce_MakeHypr(in Frame3D A2, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheHypr = new Hyperbola3D(A2, MajorRadius, MinorRadius);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed hyperbola.</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public Hyperbola3D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeHypr::Value() - no result");
        return TheHypr;
    }

    /// <summary>Alias for Value().</summary>
    public Hyperbola3D Operator() => Value();

    /// <summary>The constructed hyperbola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Hyperbola3D(gce_MakeHypr theMaker) => theMaker.Operator();
}
