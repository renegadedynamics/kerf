// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Lin2d: from an axis, a point and a direction, a cartesian equation, parallel to a line
/// through a point or at a signed distance, or through two points.
/// </summary>
public sealed class gce_MakeLin2d : gce_Root
{
    private gp_Lin2d TheLin2d = new();

    /// <summary>A line along the axis A (its location is the line's origin).</summary>
    public gce_MakeLin2d(in gp_Ax2d A)
    {
        TheLin2d = new gp_Lin2d(A);
        TheError = gce_Done;
    }

    /// <summary>A line through P (its origin) with direction V.</summary>
    public gce_MakeLin2d(in gp_Pnt2d P, in gp_Dir2d V)
    {
        TheLin2d = new gp_Lin2d(P, V);
        TheError = gce_Done;
    }

    /// <summary>The line A*X + B*Y + C = 0; fails with gce_NullAxis if A*A + B*B &lt;= gp::Resolution().</summary>
    public gce_MakeLin2d(double A, double B, double C)
    {
        if (A * A + B * B <= gp.Resolution())
        {
            TheError = gce_NullAxis;
        }
        else
        {
            TheLin2d = new gp_Lin2d(A, B, C);
            TheError = gce_Done;
        }
    }

    /// <summary>A line through two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeLin2d(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        if (P1.Distance(P2) >= gp.Resolution())
        {
            TheLin2d = new gp_Lin2d(P1, new gp_Dir2d(P2.XY() - P1.XY()));
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_ConfusedPoints;
        }
    }

    /// <summary>A line parallel to Line through Point.</summary>
    public gce_MakeLin2d(in gp_Lin2d Line, in gp_Pnt2d Point)
    {
        TheLin2d = new gp_Lin2d(Point, Line.Direction());
        TheError = gce_Done;
    }

    /// <summary>A line parallel to Line at the signed distance Dist (to its left, in its orientation, when positive).</summary>
    // OCCT: documented as on the right for a positive distance; the code offsets along the left normal
    public gce_MakeLin2d(in gp_Lin2d Line, double Dist)
    {
        gp_Pnt2d Point = new(Line.Location().XY() + Dist * new gp_XY(-Line.Direction().Y(), Line.Direction().X()));
        TheLin2d = new gp_Lin2d(Point, Line.Direction());
        TheError = gce_Done;
    }

    /// <summary>The constructed line.</summary>
    /// <exception cref="StdFail_NotDone">No line was constructed.</exception>
    public gp_Lin2d Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeLin2d::Value() - no result");
        return TheLin2d;
    }

    /// <summary>Alias for Value().</summary>
    public gp_Lin2d Operator() => Value();

    /// <summary>The constructed line (OCCT's implicit conversion operator).</summary>
    public static explicit operator gp_Lin2d(gce_MakeLin2d theMaker) => theMaker.Operator();
}
