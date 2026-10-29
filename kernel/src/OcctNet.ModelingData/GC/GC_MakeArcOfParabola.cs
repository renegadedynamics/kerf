// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of an arc of parabola as a Geom_TrimmedCurve.</summary>
public sealed class GC_MakeArcOfParabola : GC_Root
{
    private Geom_TrimmedCurve? TheArc;

    /// <summary>The arc of parabola between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public GC_MakeArcOfParabola(in gp_Parab Parab, in gp_Pnt P1, in gp_Pnt P2, bool Sense)
    {
        double Alpha1 = ElCLib.Parameter(Parab, P1);
        double Alpha2 = ElCLib.Parameter(Parab, P2);
        Geom_Parabola C = new Geom_Parabola(Parab);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of parabola from the parameter of P to the parameter Alpha.</summary>
    public GC_MakeArcOfParabola(in gp_Parab Parab, in gp_Pnt P, double Alpha, bool Sense)
    {
        double Alphafirst = ElCLib.Parameter(Parab, P);
        Geom_Parabola C = new Geom_Parabola(Parab);
        TheArc = new Geom_TrimmedCurve(C, Alphafirst, Alpha, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of parabola between the parameters Alpha1 and Alpha2.</summary>
    public GC_MakeArcOfParabola(in gp_Parab Parab, double Alpha1, double Alpha2, bool Sense)
    {
        Geom_Parabola C = new Geom_Parabola(Parab);
        TheArc = new Geom_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No arc was constructed.</exception>
    public Geom_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeArcOfParabola::Value() - no result");
        TheArc!.Freeze();
        return TheArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_TrimmedCurve(GC_MakeArcOfParabola theMaker) => theMaker.Value();
}
