// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianPoint.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of points (CartesianPoint from Geom and Geom2d, Pnt and Pnt2d from gp) to a STEP cartesian_point.
/// </summary>
internal sealed class MakeCartesianPoint : GeometryRoot
{
    private StepCartesianPoint _cartesianPoint;

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point3d de gp
    //=============================================================================

    /// <summary>A 3D point, its coordinates divided by <paramref name="aFactor"/> (the length factor).</summary>
    public MakeCartesianPoint(in Point3D p, double aFactor)
    {
        StepCartesianPoint pstep = new StepCartesianPoint();
        p.Coord(out double x, out double y, out double z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pstep.Init3D(name, x / aFactor, y / aFactor, z / aFactor);
        _cartesianPoint = pstep;
        _done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 2d de gp
    //=============================================================================

    /// <summary>A 2D point (the factor isn't used: 2D points are parameters).</summary>
    public MakeCartesianPoint(in Point2D p, double aFactor)
    {
        StepCartesianPoint pstep = new StepCartesianPoint();
        p.Coord(out double x, out double y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pstep.Init2D(name, x, y);
        _cartesianPoint = pstep;
        _done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 3d de Geom
    //=============================================================================

    /// <summary>A 3D point, its coordinates divided by <paramref name="aFactor"/>.</summary>
    public MakeCartesianPoint(CartesianPoint p, double aFactor)
    {
        StepCartesianPoint pstep = new StepCartesianPoint();
        p.Coord(out double x, out double y, out double z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pstep.Init3D(name, x / aFactor, y / aFactor, z / aFactor);
        _cartesianPoint = pstep;
        _done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 2d de Geom2d
    //=============================================================================

    /// <summary>A 2D point.</summary>
    public MakeCartesianPoint(CartesianPoint2 p)
    {
        StepCartesianPoint pstep = new StepCartesianPoint();
        p.Coord(out double x, out double y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pstep.Init2D(name, x, y);
        _cartesianPoint = pstep;
        _done = true;
    }

    /// <summary>The cartesian_point made.</summary>
    public StepCartesianPoint Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeCartesianPoint::Value() - no result");
        return _cartesianPoint;
    }
}
