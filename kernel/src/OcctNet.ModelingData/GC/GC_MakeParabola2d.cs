// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeParabola2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeParabola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
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
    public GC_MakeParabola2d(in Parabola2D Prb)
    {
        TheError = gce_Done;
        TheParabola = new Geom2d_Parabola(Prb);
    }

    /// <summary>A parabola of mirror axis MirrorAxis; fails with gce_NullFocusLength if Focal is negative.</summary>
    public GC_MakeParabola2d(in Axis2D MirrorAxis, double Focal, bool Sense)
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
    public GC_MakeParabola2d(in Frame2D Axis, double Focal)
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
    public GC_MakeParabola2d(in Axis2D D, in Point2D F, bool Sense = true)
    {
        TheError = gce_Done;
        Parabola2D para = new Parabola2D(D, F, Sense);
        TheParabola = new Geom2d_Parabola(para);
    }

    /// <summary>A parabola of focus S1 and apex O; fails with the status of gce_MakeParab2d.</summary>
    public GC_MakeParabola2d(in Point2D S1, in Point2D O)
    {
        gce_MakeParab2d P = new gce_MakeParab2d(S1, O);
        TheError = P.Status();
        if (TheError == gce_Done)
        {
            TheParabola = new Geom2d_Parabola(P.Value());
        }
    }

    /// <summary>The constructed parabola (frozen).</summary>
    /// <exception cref="NotDoneException">No parabola was constructed.</exception>
    public Geom2d_Parabola Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeParabola2d::Value() - no result");
        TheParabola!.Freeze();
        return TheParabola;
    }

    /// <summary>The constructed parabola (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Parabola(GC_MakeParabola2d theMaker) => theMaker.Value();
}
