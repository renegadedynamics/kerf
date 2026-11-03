// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_Ellipse: from a gp_Elips, an axis system and two radii, or two apexes and the center.
/// </summary>
public sealed class GC_MakeEllipse : GC_Root
{
    private Geom_Ellipse? TheEllipse;

    /// <summary>An ellipse from the gp_Elips E.</summary>
    public GC_MakeEllipse(in Ellipse3D E)
    {
        TheError = gce_Done;
        TheEllipse = new Geom_Ellipse(E);
    }

    /// <summary>
    /// An ellipse in the axis system A2; fails with gce_NegativeRadius if MinorRadius is negative and with
    /// gce_InvertAxis if MajorRadius is less than MinorRadius.
    /// </summary>
    public GC_MakeEllipse(in Frame3D A2, double MajorRadius, double MinorRadius)
    {
        if (MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else if (MajorRadius < MinorRadius)
        {
            TheError = gce_InvertAxis;
        }
        else
        {
            TheError = gce_Done;
            TheEllipse = new Geom_Ellipse(new Ellipse3D(A2, MajorRadius, MinorRadius));
        }
    }

    /// <summary>
    /// An ellipse of center Center, with S1 at the end of the major axis and S2 defining the minor radius; fails with
    /// the status of gce_MakeElips.
    /// </summary>
    public GC_MakeEllipse(in Point3D S1, in Point3D S2, in Point3D Center)
    {
        gce_MakeElips E = new gce_MakeElips(S1, S2, Center);
        TheError = E.Status();
        if (TheError == gce_Done)
        {
            TheEllipse = new Geom_Ellipse(E.Value());
        }
    }

    /// <summary>The constructed ellipse (frozen).</summary>
    /// <exception cref="NotDoneException">No ellipse was constructed.</exception>
    public Geom_Ellipse Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeEllipse::Value() - no result");
        TheEllipse!.Freeze();
        return TheEllipse;
    }

    /// <summary>The constructed ellipse (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Ellipse(GC_MakeEllipse theMaker) => theMaker.Value();
}
