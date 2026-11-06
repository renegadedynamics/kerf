// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_Line: from a point and a direction, an axis, a gp_Lin, two points, or parallel to a line
/// through a point.
/// </summary>
internal sealed class MakeLine : Root
{
    private LineCurve3D? _theLine;

    /// <summary>A line through P with direction V.</summary>
    public MakeLine(in Point3D p, in Direction3D v)
    {
        _theError = Done;
        _theLine = new LineCurve3D(p, v);
    }

    /// <summary>A line along the axis A1.</summary>
    public MakeLine(in Axis3D a1)
    {
        _theError = Done;
        _theLine = new LineCurve3D(a1);
    }

    /// <summary>A line from the gp_Lin L.</summary>
    public MakeLine(in Line3D l)
    {
        _theError = Done;
        _theLine = new LineCurve3D(l);
    }

    /// <summary>A line through P1 and P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public MakeLine(in Point3D p1, in Point3D p2)
    {
        MakeLin l = new MakeLin(p1, p2);
        _theError = l.Status();
        if (_theError == Done)
        {
            _theLine = new LineCurve3D(l.Value());
        }
    }

    /// <summary>A line parallel to Lin through Point.</summary>
    public MakeLine(in Line3D lin, in Point3D point)
    {
        MakeLin l = new MakeLin(lin, point);
        _theError = l.Status();
        if (_theError == Done)
        {
            _theLine = new LineCurve3D(l.Value());
        }
    }

    /// <summary>The constructed line (frozen).</summary>
    /// <exception cref="NotDoneException">No line was constructed.</exception>
    public LineCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeLine::Value() - no result");
        _theLine!.Freeze();
        return _theLine;
    }

    /// <summary>The constructed line (OCCT's implicit conversion).</summary>
    public static explicit operator LineCurve3D(MakeLine maker) => maker.Value();
}
