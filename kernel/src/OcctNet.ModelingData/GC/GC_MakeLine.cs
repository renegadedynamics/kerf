// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a Geom_Line: from a point and a direction, an axis, a gp_Lin, two points, or parallel to a line
/// through a point.
/// </summary>
public sealed class GC_MakeLine : GC_Root
{
    private Geom_Line? TheLine;

    /// <summary>A line through P with direction V.</summary>
    public GC_MakeLine(in Point3D P, in Direction3D V)
    {
        TheError = gce_Done;
        TheLine = new Geom_Line(P, V);
    }

    /// <summary>A line along the axis A1.</summary>
    public GC_MakeLine(in Axis3D A1)
    {
        TheError = gce_Done;
        TheLine = new Geom_Line(A1);
    }

    /// <summary>A line from the gp_Lin L.</summary>
    public GC_MakeLine(in Line3D L)
    {
        TheError = gce_Done;
        TheLine = new Geom_Line(L);
    }

    /// <summary>A line through P1 and P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public GC_MakeLine(in Point3D P1, in Point3D P2)
    {
        gce_MakeLin L = new gce_MakeLin(P1, P2);
        TheError = L.Status();
        if (TheError == gce_Done)
        {
            TheLine = new Geom_Line(L.Value());
        }
    }

    /// <summary>A line parallel to Lin through Point.</summary>
    public GC_MakeLine(in Line3D Lin, in Point3D Point)
    {
        gce_MakeLin L = new gce_MakeLin(Lin, Point);
        TheError = L.Status();
        if (TheError == gce_Done)
        {
            TheLine = new Geom_Line(L.Value());
        }
    }

    /// <summary>The constructed line (frozen).</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public Geom_Line Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "GC_MakeLine::Value() - no result");
        TheLine!.Freeze();
        return TheLine;
    }

    /// <summary>The constructed line (OCCT's implicit conversion).</summary>
    public static explicit operator Geom_Line(GC_MakeLine theMaker) => theMaker.Value();
}
