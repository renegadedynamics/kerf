// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfHyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of an arc of hyperbola as a Geom_TrimmedCurve.</summary>
public sealed class GC_MakeArcOfHyperbola : GC_Root
{
    private Geom_TrimmedCurve? TheArc;

    /// <summary>The arc of hyperbola between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public GC_MakeArcOfHyperbola(in Hyperbola3D Hypr, in Point3D P1, in Point3D P2, bool Sense)
    {
        double Alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Hypr, P1);
        double Alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Hypr, P2);
        Geom_Hyperbola C = new Geom_Hyperbola(Hypr);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of hyperbola from the parameter of P to the parameter Alpha.</summary>
    public GC_MakeArcOfHyperbola(in Hyperbola3D Hypr, in Point3D P, double Alpha, bool Sense)
    {
        double Alphafirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(Hypr, P);
        Geom_Hyperbola C = new Geom_Hyperbola(Hypr);
        TheArc = new Geom_TrimmedCurve(C, Alphafirst, Alpha, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of hyperbola between the parameters Alpha1 and Alpha2.</summary>
    public GC_MakeArcOfHyperbola(in Hyperbola3D Hypr, double Alpha1, double Alpha2, bool Sense)
    {
        Geom_Hyperbola C = new Geom_Hyperbola(Hypr);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="NotDoneException">No arc was constructed.</exception>
    public Geom_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeArcOfHyperbola::Value() - no result");
        TheArc!.Freeze();
        return TheArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_TrimmedCurve(GC_MakeArcOfHyperbola theMaker) => theMaker.Value();
}
