// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.gce_ErrorType;

namespace Occt;

/// <summary>
/// Constructions of a gp_Lin: from an axis, a point and a direction, parallel to a line through a point, or through
/// two points.
/// </summary>
public sealed class gce_MakeLin : gce_Root
{
    private Line3D TheLin = new();

    /// <summary>A line along the axis A1 (its location is the line's origin).</summary>
    public gce_MakeLin(in Axis3D A1)
    {
        TheLin = new Line3D(A1);
        TheError = gce_Done;
    }

    /// <summary>A line through P (its origin) with direction V.</summary>
    public gce_MakeLin(in Point3D P, in Direction3D V)
    {
        TheLin = new Line3D(P, V);
        TheError = gce_Done;
    }

    /// <summary>A line through two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public gce_MakeLin(in Point3D P1, in Point3D P2)
    {
        if (P1.Distance(P2) >= Constants.Resolution())
        {
            TheLin = new Line3D(P1, new Direction3D(P2.XYZ() - P1.XYZ()));
            TheError = gce_Done;
        }
        else
        {
            TheError = gce_ConfusedPoints;
        }
    }

    /// <summary>A line parallel to Lin through the point P.</summary>
    public gce_MakeLin(in Line3D Lin, in Point3D P)
    {
        TheLin = new Line3D(P, Lin.Direction());
        TheError = gce_Done;
    }

    /// <summary>The constructed line.</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public Line3D Value()
    {
        StdFail_NotDone_Raise_if(TheError != gce_Done, "gce_MakeLin::Value() - no result");
        return TheLin;
    }

    /// <summary>Alias for Value().</summary>
    public Line3D Operator() => Value();

    /// <summary>The constructed line (OCCT's implicit conversion operator).</summary>
    public static explicit operator Line3D(gce_MakeLin theMaker) => theMaker.Operator();
}
