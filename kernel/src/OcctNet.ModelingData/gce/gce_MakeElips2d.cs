// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Elips2d: from a major axis (or an axis system) and radii, or from a center and two points.
/// </summary>
public sealed class gce_MakeElips2d : gce_Root
{
    private gp_Elips2d TheElips2d = new();

    /// <summary>
    /// An ellipse from its center and two points: S1 gives the major axis and radius, the distance from S2 to the
    /// major axis the minor radius. Fails with gce_InvertAxis if the minor radius is greater than the major one and
    /// gce_NullAxis if it is null.
    /// </summary>
    public gce_MakeElips2d(in gp_Pnt2d S1, in gp_Pnt2d S2, in gp_Pnt2d Center)
    {
        double D1 = S1.Distance(Center);
        gp_Dir2d XAxis = new(S1.XY() - Center.XY());
        gp_Dir2d YAxis = new(S2.XY() - Center.XY());
        double D2 = new gp_Lin2d(Center, XAxis).Distance(S2);
        if (D1 < D2)
        {
            TheError = gce_InvertAxis;
        }
        else if (D2 < gp.Resolution())
        {
            TheError = gce_NullAxis;
        }
        else
        {
            TheElips2d = new gp_Elips2d(new gp_Ax22d(Center, XAxis, YAxis), D1, D2);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// An ellipse from its major axis (whose location is the center), its radii and a sense of parametrization.
    /// Fails with gce_NegativeRadius if MajorRadius &lt; 0 and gce_InvertRadius if MajorRadius &lt; MinorRadius.
    /// </summary>
    public gce_MakeElips2d(in gp_Ax2d MajorAxis, double MajorRadius, double MinorRadius, bool Sense = true)
    {
        if (MajorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else if (MajorRadius < MinorRadius)
        {
            TheError = gce_InvertRadius;
        }
        else
        {
            TheElips2d = new gp_Elips2d(MajorAxis, MajorRadius, MinorRadius, Sense);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// An ellipse from its axis system (whose location is the center) and its radii. Fails with gce_NegativeRadius if
    /// MajorRadius &lt; 0 and gce_InvertRadius if MajorRadius &lt; MinorRadius.
    /// </summary>
    public gce_MakeElips2d(in gp_Ax22d A, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else if (MajorRadius < MinorRadius)
        {
            TheError = gce_InvertRadius;
        }
        else
        {
            TheElips2d = new gp_Elips2d(A, MajorRadius, MinorRadius);
            TheError = gce_Done;
        }
    }

    /// <summary>The constructed ellipse.</summary>
    /// <exception cref="StdFail_NotDone">No ellipse was constructed.</exception>
    public gp_Elips2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeElips2d::Value() - no result");
        return TheElips2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Elips2d Operator() => Value();

    /// <summary>The constructed ellipse (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Elips2d(gce_MakeElips2d theMaker) => theMaker.Operator();
}
