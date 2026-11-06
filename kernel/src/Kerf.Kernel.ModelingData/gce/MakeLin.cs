// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeLin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Lin: from an axis, a point and a direction, parallel to a line through a point, or through
/// two points.
/// </summary>
internal sealed class MakeLin : Root
{
    private Line3D _theLin = new();

    /// <summary>A line along the axis A1 (its location is the line's origin).</summary>
    public MakeLin(in Axis3D a1)
    {
        _theLin = new Line3D(a1);
        _theError = Done;
    }

    /// <summary>A line through P (its origin) with direction V.</summary>
    public MakeLin(in Point3D p, in Direction3D v)
    {
        _theLin = new Line3D(p, v);
        _theError = Done;
    }

    /// <summary>A line through two points; fails with gce_ConfusedPoints if they are coincident.</summary>
    public MakeLin(in Point3D p1, in Point3D p2)
    {
        if (p1.DistanceTo(p2) >= Constants.Resolution())
        {
            _theLin = new Line3D(p1, new Direction3D(p2.XYZ() - p1.XYZ()));
            _theError = Done;
        }
        else
        {
            _theError = ConfusedPoints;
        }
    }

    /// <summary>A line parallel to Lin through the point P.</summary>
    public MakeLin(in Line3D lin, in Point3D p)
    {
        _theLin = new Line3D(p, lin.Direction);
        _theError = Done;
    }

    /// <summary>The constructed line.</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public Line3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeLin::Value() - no result");
        return _theLin;
    }

    /// <summary>Alias for Value().</summary>
    public Line3D Operator() => Value();

    /// <summary>The constructed line (OCCT's implicit conversion operator).</summary>
    public static explicit operator Line3D(MakeLin maker) => maker.Operator();
}
