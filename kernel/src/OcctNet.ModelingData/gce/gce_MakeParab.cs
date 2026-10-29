// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeParab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Parab: from an axis system and a focal length, or from a directrix and a focus. The vertex
/// is the location of the axis system, its X axis the symmetry axis (towards the focus); the equation in it is
/// Y**2 = (2*P) * X, P being the distance between focus and directrix and the focal length P / 2.
/// </summary>
public sealed class gce_MakeParab : gce_Root
{
    private gp_Parab TheParab = new();

    /// <summary>A parabola from its axis system and focal length; fails with gce_NullFocusLength if Focal &lt; 0.</summary>
    public gce_MakeParab(in gp_Ax2 A2, double Focal)
    {
        if (Focal < 0.0)
        {
            TheError = gce_NullFocusLength;
        }
        else
        {
            TheParab = new gp_Parab(A2, Focal);
            TheError = gce_Done;
        }
    }

    /// <summary>A parabola from its directrix D and focus F.</summary>
    public gce_MakeParab(in gp_Ax1 D, in gp_Pnt F)
    {
        TheParab = new gp_Parab(D, F);
        TheError = gce_Done;
    }

    /// <summary>The constructed parabola.</summary>
    /// <exception cref="StdFail_NotDone">No parabola was constructed.</exception>
    public gp_Parab Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeParab::Value() - no result");
        return TheParab;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Parab Operator() => Value();

    /// <summary>The constructed parabola (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Parab(gce_MakeParab theMaker) => theMaker.Operator();
}
