// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Parab2d: from a symmetry axis or an axis system and a focal length, from a directrix and a
/// focus, or from a focus and a vertex.
/// </summary>
public sealed class gce_MakeParab2d : gce_Root
{
    private gp_Parab2d TheParab2d = new();

    /// <summary>
    /// A parabola from its axis system and focal length (0 is accepted); fails with gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public gce_MakeParab2d(in gp_Ax22d A, double Focal)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheParab2d = new gp_Parab2d(A, Focal);
            TheError = gce_Done;
        }
    }

    /// <summary>
    /// A parabola from its symmetry axis, focal length (0 is accepted) and sense of parametrization; fails with
    /// gce_NullFocusLength if Focal &lt; 0.
    /// </summary>
    public gce_MakeParab2d(in gp_Ax2d MirrorAxis, double Focal, bool Sense = true)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheParab2d = new gp_Parab2d(MirrorAxis, Focal, Sense);
            TheError = gce_Done;
        }
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public gce_MakeParab2d(in gp_Ax2d D, in gp_Pnt2d F, bool Sense = true)
    {
        TheParab2d = new gp_Parab2d(D, F, Sense);
        TheError = gce_Done;
    }

    /// <summary>
    /// A parabola from its focus S and vertex Center; fails with gce_NullAxis if they coincide.
    /// </summary>
    public gce_MakeParab2d(in gp_Pnt2d S, in gp_Pnt2d Center, bool Sense = true)
    {
        if (S.Distance(Center) >= gp.Resolution())
        {
            gp_Dir2d XAxis = new(S.XY() - Center.XY());
            TheParab2d = new gp_Parab2d(new gp_Ax2d(Center, XAxis), S.Distance(Center), Sense);
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_NullAxis;
        }
    }

    /// <summary>The constructed parabola.</summary>
    /// <exception cref="StdFail_NotDone">No parabola was constructed.</exception>
    public gp_Parab2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeParab2d::Value() - no result");
        return TheParab2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Parab2d Operator() => Value();

    /// <summary>The constructed parabola (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Parab2d(gce_MakeParab2d theMaker) => theMaker.Operator();
}
