// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Lin: from an axis, a point and a direction, parallel to a line through a point, or through
/// two points.
/// </summary>
public sealed class gce_MakeLin : gce_Root
{
    private gp_Lin TheLin = new();

    /// <summary>A line along the axis A1 (its location is the line's origin).</summary>
    public gce_MakeLin(in gp_Ax1 A1)
    {
        TheLin = new gp_Lin(A1);
        TheError = gce_Done;
    }

    /// <summary>A line through P (its origin) with direction V.</summary>
    public gce_MakeLin(in gp_Pnt P, in gp_Dir V)
    {
        TheLin = new gp_Lin(P, V);
        TheError = gce_Done;
    }

    /// <summary>A line through two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeLin(in gp_Pnt P1, in gp_Pnt P2)
    {
        if (P1.Distance(P2) >= gp.Resolution())
        {
            TheLin = new gp_Lin(P1, new gp_Dir(P2.XYZ() - P1.XYZ()));
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_ConfusedPoints;
        }
    }

    /// <summary>A line parallel to Lin through the point P.</summary>
    public gce_MakeLin(in gp_Lin Lin, in gp_Pnt P)
    {
        TheLin = new gp_Lin(P, Lin.Direction());
        TheError = gce_Done;
    }

    /// <summary>The constructed line.</summary>
    /// <exception cref="StdFail_NotDone">No line was constructed.</exception>
    public gp_Lin Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeLin::Value() - no result");
        return TheLin;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Lin Operator() => Value();

    /// <summary>The constructed line (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Lin(gce_MakeLin theMaker) => theMaker.Operator();
}
