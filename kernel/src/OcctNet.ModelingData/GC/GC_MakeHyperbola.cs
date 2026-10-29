// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_Hyperbola: from a gp_Hypr, an axis system and two radii, or two apexes and the center.
/// </summary>
public sealed class GC_MakeHyperbola : GC_Root
{
    private Geom_Hyperbola? TheHyperbola;

    /// <summary>A hyperbola from the gp_Hypr H.</summary>
    public GC_MakeHyperbola(in gp_Hypr H)
    {
        TheError = gce_Done;
        TheHyperbola = new Geom_Hyperbola(H);
    }

    /// <summary>A hyperbola in the axis system A2; fails with gce_NegativeRadius if a radius is negative.</summary>
    public GC_MakeHyperbola(in gp_Ax2 A2, double MajorRadius, double MinorRadius)
    {
        if (MajorRadius < 0.0 || MinorRadius < 0.0)
        {
            TheError = gce_NegativeRadius;
        }
        else
        {
            TheError = gce_Done;
            TheHyperbola = new Geom_Hyperbola(new gp_Hypr(A2, MajorRadius, MinorRadius));
        }
    }

    /// <summary>A hyperbola from two apexes and its center; fails with the status of gce_MakeHypr.</summary>
    public GC_MakeHyperbola(in gp_Pnt S1, in gp_Pnt S2, in gp_Pnt Center)
    {
        gce_MakeHypr H = new gce_MakeHypr(S1, S2, Center);
        TheError = H.Status();
        if (TheError == gce_Done)
        {
            TheHyperbola = new Geom_Hyperbola(H.Value());
        }
    }

    /// <summary>The constructed hyperbola (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No hyperbola was constructed.</exception>
    public Geom_Hyperbola Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeHyperbola::Value() - no result");
        TheHyperbola!.Freeze();
        return TheHyperbola;
    }

    /// <summary>The constructed hyperbola (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Hyperbola(GC_MakeHyperbola theMaker) => theMaker.Value();
}
