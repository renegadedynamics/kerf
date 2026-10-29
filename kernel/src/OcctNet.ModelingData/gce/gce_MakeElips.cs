// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of a gp_Elips: from an axis system and radii, or from a center and two points.</summary>
public sealed class gce_MakeElips : gce_Root
{
    private gp_Elips TheElips = new();

    /// <summary>
    /// An ellipse with its major radius on the X axis of A2 and its minor radius on its Y axis. The radii may be
    /// equal. Fails with gce_InvertRadius if MajorRadius &lt; MinorRadius and gce_NegativeRadius if MinorRadius &lt; 0.
    /// </summary>
    public gce_MakeElips(in gp_Ax2 A2, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < MinorRadius)
        {
            TheError = gce_InvertRadius;
        }
        else if (MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheElips = new gp_Elips(A2, MajorRadius, MinorRadius);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// An ellipse from its center and two points: S1 gives the major axis and radius, the distance from S2 to the
    /// major axis the minor radius. Fails with gce_NullAxis if S1 and Center coincide and gce_InvertAxis if the minor
    /// radius is null or greater than the major radius, or the points are collinear.
    /// </summary>
    public gce_MakeElips(in gp_Pnt S1, in gp_Pnt S2, in gp_Pnt Center)
    {
        double D1 = S1.Distance(Center);
        if (D1 < gp.Resolution())
        {
            TheError = gce_NullAxis;
        }
        else
        {
            gp_Dir XAxis = new(S1.XYZ() - Center.XYZ());
            double D2 = new gp_Lin(Center, XAxis).Distance(S2);
            if (D1 < D2 || D2 < gp.Resolution())
            {
                TheError = gce_InvertAxis;
            }
            else
            {
                gp_XYZ aCross = XAxis.XYZ().Crossed(S2.XYZ() - Center.XYZ());
                if (aCross.Modulus() <= gp.Resolution())
                {
                    TheError = gce_InvertAxis;
                }
                else
                {
                    gp_Dir Norm = new(aCross);
                    TheElips = new gp_Elips(new gp_Ax2(Center, Norm, XAxis), D1, D2);
                    TheError = gce_Done;
                }
            }
        }
    }

    /// <summary>The constructed ellipse.</summary>
    /// <exception cref="StdFail_NotDone">No ellipse was constructed.</exception>
    public gp_Elips Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeElips::Value() - no result");
        return TheElips;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Elips Operator() => Value();

    /// <summary>The constructed ellipse (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Elips(gce_MakeElips theMaker) => theMaker.Operator();
}
