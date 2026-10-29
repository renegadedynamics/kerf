// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom2d_Parabola: from a gp_Parab2d, a mirror axis (or axis system) and the focal length, a
/// directrix and a focus, or a focus and the apex.
/// </summary>
public sealed class GC_MakeParabola2d : GC_Root
{
    private Geom2d_Parabola? TheParabola;

    /// <summary>A parabola from the gp_Parab2d Prb.</summary>
    public GC_MakeParabola2d(in gp_Parab2d Prb)
    {
        TheError = gce_Done;
        TheParabola = new Geom2d_Parabola(Prb);
    }

    /// <summary>A parabola of mirror axis MirrorAxis; fails with gce_NullFocusLength if Focal is negative.</summary>
    public GC_MakeParabola2d(in gp_Ax2d MirrorAxis, double Focal, bool Sense)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheError = gce_Done;
            TheParabola = new Geom2d_Parabola(MirrorAxis, Focal, Sense);
        }
    }

    /// <summary>A parabola in the axis system Axis; fails with gce_NullFocusLength if Focal is negative.</summary>
    public GC_MakeParabola2d(in gp_Ax22d Axis, double Focal)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheError = gce_Done;
            TheParabola = new Geom2d_Parabola(Axis, Focal);
        }
    }

    /// <summary>A parabola of directrix D and focus F.</summary>
    public GC_MakeParabola2d(in gp_Ax2d D, in gp_Pnt2d F, bool Sense = true)
    {
        TheError = gce_Done;
        gp_Parab2d para = new gp_Parab2d(D, F, Sense);
        TheParabola = new Geom2d_Parabola(para);
    }

    /// <summary>A parabola of focus S1 and apex O; fails with the status of gce_MakeParab2d.</summary>
    public GC_MakeParabola2d(in gp_Pnt2d S1, in gp_Pnt2d O)
    {
        gce_MakeParab2d P = new gce_MakeParab2d(S1, O);
        TheError = P.Status();
        if (TheError == gce_Done)
        {
            TheParabola = new Geom2d_Parabola(P.Value());
        }
    }

    /// <summary>The constructed parabola (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No parabola was constructed.</exception>
    public Geom2d_Parabola Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeParabola2d::Value() - no result");
        TheParabola!.Freeze();
        return TheParabola;
    }

    /// <summary>The constructed parabola (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Parabola(GC_MakeParabola2d theMaker) => theMaker.Value();
}
