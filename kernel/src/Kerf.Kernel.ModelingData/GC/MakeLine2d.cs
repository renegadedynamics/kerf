// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeLine.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom2d_Line: from an axis, a gp_Lin2d, a point and a direction, two points, or parallel to a line
/// through a point or at a distance.
/// </summary>
internal sealed class MakeLine2d : Root
{
    private LineCurve2D? _theLine;

    /// <summary>A line along the axis A.</summary>
    public MakeLine2d(in Axis2D a)
    {
        _theError = Done;
        _theLine = new LineCurve2D(a);
    }

    /// <summary>A line from the gp_Lin2d L.</summary>
    public MakeLine2d(in Line2D l)
    {
        _theError = Done;
        _theLine = new LineCurve2D(l);
    }

    /// <summary>A line through P with direction V.</summary>
    public MakeLine2d(in Point2D p, in Direction2D v)
    {
        _theError = Done;
        _theLine = new LineCurve2D(p, v);
    }

    /// <summary>A line through P1 and P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public MakeLine2d(in Point2D p1, in Point2D p2)
    {
        MakeLin2d l = new MakeLin2d(p1, p2);
        _theError = l.Status();
        if (_theError == Done)
        {
            _theLine = new LineCurve2D(l.Value());
        }
    }

    /// <summary>A line parallel to Lin through Point.</summary>
    public MakeLine2d(in Line2D lin, in Point2D point)
    {
        MakeLin2d l = new MakeLin2d(lin, point);
        _theError = l.Status();
        if (_theError == Done)
        {
            _theLine = new LineCurve2D(l.Value());
        }
    }

    /// <summary>A line parallel to Lin at the signed distance Dist.</summary>
    public MakeLine2d(in Line2D lin, double dist)
    {
        MakeLin2d l = new MakeLin2d(lin, dist);
        _theError = l.Status();
        if (_theError == Done)
        {
            _theLine = new LineCurve2D(l.Value());
        }
    }

    /// <summary>The constructed line (frozen).</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public LineCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeLine2d::Value() - no result");
        _theLine!.Freeze();
        return _theLine;
    }

    /// <summary>The constructed line (OCCT's implicit conversion).</summary>
    public static explicit operator LineCurve2D(MakeLine2d maker) => maker.Value();
}
