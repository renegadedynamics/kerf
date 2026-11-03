// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeHyperbola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom2d_Hyperbola: from a gp_Hypr2d, an axis (system) and two radii, or two apexes and the
/// center.
/// </summary>
public sealed class GC_MakeHyperbola2d : GC_Root
{
    private Geom2d_Hyperbola? TheHyperbola;

    /// <summary>A hyperbola from the gp_Hypr2d H.</summary>
    public GC_MakeHyperbola2d(in Hyperbola2D H)
    {
        TheError = gce_Done;
        TheHyperbola = new Geom2d_Hyperbola(H);
    }

    /// <summary>A hyperbola on the major axis MajorAxis; fails with the status of gce_MakeHypr2d.</summary>
    public GC_MakeHyperbola2d(in Axis2D MajorAxis, double MajorRadius, double MinorRadius, bool Sense)
    {
        gce_MakeHypr2d H = new gce_MakeHypr2d(MajorAxis, MajorRadius, MinorRadius, Sense);
        TheError = H.Status();
        if (TheError == gce_Done)
        {
            TheHyperbola = new Geom2d_Hyperbola(H.Value());
        }
    }

    /// <summary>A hyperbola in the axis system Axis; fails with the status of gce_MakeHypr2d.</summary>
    public GC_MakeHyperbola2d(in Frame2D Axis, double MajorRadius, double MinorRadius)
    {
        gce_MakeHypr2d H = new gce_MakeHypr2d(Axis, MajorRadius, MinorRadius);
        TheError = H.Status();
        if (TheError == gce_Done)
        {
            TheHyperbola = new Geom2d_Hyperbola(H.Value());
        }
    }

    /// <summary>A hyperbola from two apexes and its center; fails with the status of gce_MakeHypr2d.</summary>
    public GC_MakeHyperbola2d(in Point2D S1, in Point2D S2, in Point2D Center)
    {
        gce_MakeHypr2d H = new gce_MakeHypr2d(S1, S2, Center);
        TheError = H.Status();
        if (TheError == gce_Done)
        {
            TheHyperbola = new Geom2d_Hyperbola(H.Value());
        }
    }

    /// <summary>The constructed hyperbola (frozen).</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public Geom2d_Hyperbola Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeHyperbola2d::Value() - no result");
        TheHyperbola!.Freeze();
        return TheHyperbola;
    }

    /// <summary>The constructed hyperbola (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Hyperbola(GC_MakeHyperbola2d theMaker) => theMaker.Value();
}
