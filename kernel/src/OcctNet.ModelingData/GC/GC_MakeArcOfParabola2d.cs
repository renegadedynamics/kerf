// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeArcOfParabola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>Constructions of an arc of parabola in 2D as a Geom2d_TrimmedCurve.</summary>
public sealed class GC_MakeArcOfParabola2d : GC_Root
{
    private Geom2d_TrimmedCurve? TheArc;

    /// <summary>The arc of parabola in 2D between the parameters of P1 and P2, in the conic's sense if Sense is true.</summary>
    public GC_MakeArcOfParabola2d(in gp_Parab2d Parab, in gp_Pnt2d P1, in gp_Pnt2d P2, bool Sense = true)
    {
        double Alpha1 = ElCLib.Parameter(Parab, P1);
        double Alpha2 = ElCLib.Parameter(Parab, P2);
        Geom2d_Parabola C = new Geom2d_Parabola(Parab);
        TheArc = new Geom2d_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of parabola in 2D from the parameter of P to the parameter Alpha.</summary>
    public GC_MakeArcOfParabola2d(in gp_Parab2d Parab, in gp_Pnt2d P, double Alpha, bool Sense = true)
    {
        double Alphafirst = ElCLib.Parameter(Parab, P);
        Geom2d_Parabola C = new Geom2d_Parabola(Parab);
        TheArc = new Geom2d_TrimmedCurve(C, Alphafirst, Alpha, Sense);
        TheError = gce_Done;
    }

    /// <summary>The arc of parabola in 2D between the parameters Alpha1 and Alpha2.</summary>
    public GC_MakeArcOfParabola2d(in gp_Parab2d Parab, double Alpha1, double Alpha2, bool Sense = true)
    {
        Geom2d_Parabola C = new Geom2d_Parabola(Parab);
        TheArc = new Geom2d_TrimmedCurve(C, Alpha1, Alpha2, Sense);
        TheError = gce_Done;
    }

    /// <summary>The constructed arc (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No arc was constructed.</exception>
    public Geom2d_TrimmedCurve Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeArcOfParabola2d::Value() - no result");
        TheArc!.Freeze();
        return TheArc;
    }

    /// <summary>The constructed arc (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_TrimmedCurve(GC_MakeArcOfParabola2d theMaker) => theMaker.Value();
}
