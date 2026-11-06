// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Lin2d: from an axis, a point and a direction, a cartesian equation, parallel to a line
/// through a point or at a signed distance, or through two points.
/// </summary>
internal sealed class MakeLin2d : Root
{
    private Line2D _theLin2d = new();

    /// <summary>A line along the axis A (its location is the line's origin).</summary>
    public MakeLin2d(in Axis2D a)
    {
        _theLin2d = new Line2D(a);
        _theError = Done;
    }

    /// <summary>A line through P (its origin) with direction V.</summary>
    public MakeLin2d(in Point2D p, in Direction2D v)
    {
        _theLin2d = new Line2D(p, v);
        _theError = Done;
    }

    /// <summary>The line A*X + B*Y + C = 0; fails with gce_NullAxis if A*A + B*B &lt;= gp::Resolution().</summary>
    public MakeLin2d(double a, double b, double c)
    {
        if (a * a + b * b <= Constants.Resolution())
        {
            _theError = NullAxis;
        }
        else
        {
            _theLin2d = new Line2D(a, b, c);
            _theError = Done;
        }
    }

    /// <summary>A line through two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public MakeLin2d(in Point2D p1, in Point2D p2)
    {
        if (p1.DistanceTo(p2) >= Constants.Resolution())
        {
            _theLin2d = new Line2D(p1, new Direction2D(p2.XY() - p1.XY()));
            _theError = Done;
        }
        else
        {
            _theError = ConfusedPoints;
        }
    }

    /// <summary>A line parallel to Line through Point.</summary>
    public MakeLin2d(in Line2D line, in Point2D point)
    {
        _theLin2d = new Line2D(point, line.Direction);
        _theError = Done;
    }

    /// <summary>A line parallel to Line at the signed distance Dist (to its left, in its orientation, when positive).</summary>
    // OCCT: documented as on the right for a positive distance; the code offsets along the left normal
    public MakeLin2d(in Line2D line, double dist)
    {
        Point2D point = new(line.Location.XY() + dist * new Xy(-line.Direction.Y, line.Direction.X));
        _theLin2d = new Line2D(point, line.Direction);
        _theError = Done;
    }

    /// <summary>The constructed line.</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public Line2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeLin2d::Value() - no result");
        return _theLin2d;
    }

    /// <summary>Alias for Value().</summary>
    public Line2D Operator() => Value();

    /// <summary>The constructed line (OCCT's implicit conversion operator).</summary>
    public static explicit operator Line2D(MakeLin2d maker) => maker.Operator();
}
