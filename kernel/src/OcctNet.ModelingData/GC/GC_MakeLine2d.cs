// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom2d_Line: from an axis, a gp_Lin2d, a point and a direction, two points, or parallel to a line
/// through a point or at a distance.
/// </summary>
public sealed class GC_MakeLine2d : GC_Root
{
    private Geom2d_Line? TheLine;

    /// <summary>A line along the axis A.</summary>
    public GC_MakeLine2d(in gp_Ax2d A)
    {
        TheError = gce_Done;
        TheLine = new Geom2d_Line(A);
    }

    /// <summary>A line from the gp_Lin2d L.</summary>
    public GC_MakeLine2d(in gp_Lin2d L)
    {
        TheError = gce_Done;
        TheLine = new Geom2d_Line(L);
    }

    /// <summary>A line through P with direction V.</summary>
    public GC_MakeLine2d(in gp_Pnt2d P, in gp_Dir2d V)
    {
        TheError = gce_Done;
        TheLine = new Geom2d_Line(P, V);
    }

    /// <summary>A line through P1 and P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public GC_MakeLine2d(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        gce_MakeLin2d L = new gce_MakeLin2d(P1, P2);
        TheError = L.Status();
        if (TheError == gce_Done)
        {
            TheLine = new Geom2d_Line(L.Value());
        }
    }

    /// <summary>A line parallel to Lin through Point.</summary>
    public GC_MakeLine2d(in gp_Lin2d Lin, in gp_Pnt2d Point)
    {
        gce_MakeLin2d L = new gce_MakeLin2d(Lin, Point);
        TheError = L.Status();
        if (TheError == gce_Done)
        {
            TheLine = new Geom2d_Line(L.Value());
        }
    }

    /// <summary>A line parallel to Lin at the signed distance Dist.</summary>
    public GC_MakeLine2d(in gp_Lin2d Lin, double Dist)
    {
        gce_MakeLin2d L = new gce_MakeLin2d(Lin, Dist);
        TheError = L.Status();
        if (TheError == gce_Done)
        {
            TheLine = new Geom2d_Line(L.Value());
        }
    }

    /// <summary>The constructed line (frozen).</summary>
    /// <exception cref="StdFail_NotDone">No line was constructed.</exception>
    public Geom2d_Line Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeLine2d::Value() - no result");
        TheLine!.Freeze();
        return TheLine;
    }

    /// <summary>The constructed line (OCCT's implicit conversion).</summary>
    public static explicit operator Geom2d_Line(GC_MakeLine2d theMaker) => theMaker.Value();
}
