// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntCS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// The intersections of a curve and a surface: points (with their parameters on the surface and the curve) and
/// segments (parts of the curve lying on the surface), through IntCurveSurface_HInter on the whole curve and
/// surface.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Perform</c> can be called again. Results are returned by value;
/// <see cref="Segment"/> builds a new trimmed curve on the stored curve. (IntCurveSurface_HInter makes no segments,
/// so <see cref="SegmentCount"/> is always 0.)
/// </remarks>
internal class CurveSurfaceIntersection
{
    private Curve3D? _curve;
    private readonly HInter _intCS = new();

    /// <summary>An empty intersector; <see cref="Perform"/> computes the intersections.</summary>
    public CurveSurfaceIntersection()
    {
    }

    /// <summary>Computes the intersections of the curve <paramref name="c"/> and the surface <paramref name="s"/>.</summary>
    public CurveSurfaceIntersection(Curve3D c, Surface s)
    {
        Perform(c, s);
    }

    /// <summary>Computes the intersections of the curve <paramref name="c"/> and the surface <paramref name="s"/>.</summary>
    public void Perform(Curve3D c, Surface s)
    {
        _curve = c;
        CurveAdaptor hc = new(c);
        SurfaceAdaptor hs = new(s);
        _intCS.Perform(hc, hs);
    }

    /// <summary>True if the intersections were computed.</summary>
    public bool IsDone => _intCS.IsDone;

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int PointCount => _intCS.PointCount;

    /// <summary>The intersection point of range <paramref name="index"/>.</summary>
    public Point3D Point(int index) => _intCS.Point(index).Pnt();

    /// <summary>
    /// The parameters (<paramref name="u"/>, <paramref name="v"/>) on the surface and <paramref name="w"/> on the
    /// curve of the intersection point of range <paramref name="index"/>.
    /// </summary>
    public void Parameters(int index, out double u, out double v, out double w)
    {
        CurveSurfaceIntersectionPoint thePoint = _intCS.Point(index);
        u = thePoint.U();
        v = thePoint.V();
        w = thePoint.W();
    }

    /// <summary>The number of intersection segments.</summary>
    public int SegmentCount => _intCS.SegmentCount;

    /// <summary>The part of the curve that is the intersection segment of range <paramref name="index"/>.</summary>
    public Curve3D Segment(int index)
    {
        CurveSurfaceIntersectionPoint firstPoint = _intCS.Segment(index).FirstPoint();
        CurveSurfaceIntersectionPoint lastPoint = _intCS.Segment(index).SecondPoint();
        TrimmedCurve3D theCurve = new(_curve!, firstPoint.W(), lastPoint.W());
        return theCurve;
    }

    /// <summary>The parameters on the surface of the ends of the intersection segment of range <paramref name="index"/>.</summary>
    public void Parameters(int index, out double u1, out double v1, out double u2, out double v2)
    {
        CurveSurfaceIntersectionPoint firstPoint = _intCS.Segment(index).FirstPoint();
        CurveSurfaceIntersectionPoint lastPoint = _intCS.Segment(index).SecondPoint();
        u1 = firstPoint.U();
        v1 = firstPoint.V();
        u2 = lastPoint.U();
        v2 = lastPoint.V();
    }
}
