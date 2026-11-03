// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// The intersections between two 2D curves, or the self-intersections of one: points where the curves cross, and
/// segments where they are tangent along a portion.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117). It keeps copies of the curves, as OCCT does, for <see cref="Segment"/>.
/// </remarks>
public class CurveCurveIntersection2D
{
    private bool _isDone;
    private Curve2D? _curve1;
    private Curve2D? _curve2;
    private GInter _intersector = new();

    /// <summary>An empty intersector; use <see cref="Init(Curve2D, Curve2D, double)"/>.</summary>
    public CurveCurveIntersection2D()
    {
        _isDone = false;
    }

    /// <summary>The intersections between <paramref name="c1"/> and <paramref name="c2"/>.</summary>
    public CurveCurveIntersection2D(Curve2D c1, Curve2D c2, double tol = 1.0e-6)
    {
        Init(c1, c2, tol);
    }

    /// <summary>
    /// The self-intersections of <paramref name="c1"/>. <paramref name="tol"/> is the precision of the points and,
    /// for tangential intersections, the size of the segments.
    /// </summary>
    public CurveCurveIntersection2D(Curve2D c1, double tol = 1.0e-6)
    {
        Init(c1, tol);
    }

    /// <summary>Computes the intersections between <paramref name="c1"/> and <paramref name="c2"/>.</summary>
    /// <exception cref="NullObjectException">A curve is null.</exception>
    public void Init(Curve2D c1, Curve2D c2, double tol = 1.0e-6)
    {
        Standard_NullObject_Raise_if(c1 is null, "Geom2dAPI_InterCurveCurve::Init - C1 is null");
        Standard_NullObject_Raise_if(c2 is null, "Geom2dAPI_InterCurveCurve::Init - C2 is null");
        _curve1 = c1!.Copy() as Curve2D;
        _curve2 = c2!.Copy() as Curve2D;

        CurveAdaptor2D ac1 = new(c1);
        CurveAdaptor2D ac2 = new(c2);
        _intersector = new GInter(ac1, ac2, tol, tol);
        _isDone = _intersector.IsDone;
    }

    /// <summary>Computes the self-intersections of <paramref name="c1"/>.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public void Init(Curve2D c1, double tol = 1.0e-6)
    {
        Standard_NullObject_Raise_if(c1 is null, "Geom2dAPI_InterCurveCurve::Init - C1 is null");
        _curve1 = c1!.Copy() as Curve2D;
        _curve2 = null;

        CurveAdaptor2D ac1 = new(c1);
        _intersector = new GInter(ac1, tol, tol);
        _isDone = _intersector.IsDone;
    }

    /// <summary>The number of intersection points (cross intersections); 0 if none were found.</summary>
    public int NbPoints()
    {
        if (_isDone)
        {
            return _intersector.PointCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The intersection point of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbPoints].</exception>
    public Point2D Point(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "Geom2dAPI_InterCurveCurve::Point");

        return (_intersector.Point(index)).Value();
    }

    /// <summary>The number of tangential intersections (segments); 0 if none were found.</summary>
    public int NbSegments()
    {
        if (_isDone)
        {
            return _intersector.SegmentCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>
    /// The intersection segment of index <paramref name="index"/>, as the portions of the first and the second
    /// curve it covers (both of the one curve for a self-intersection).
    /// </summary>
    /// <remarks>
    /// If the segment is opposite, the second portion runs from the parameter of the segment's last point to that
    /// of its first point.
    /// </remarks>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbSegments].</exception>
    /// <exception cref="NullObjectException">The algorithm was not initialised.</exception>
    public void Segment(int index, out Curve2D curve1, out Curve2D curve2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbSegments(), "Geom2dAPI_InterCurveCurve::Segment");

        Standard_NullObject_Raise_if(_curve1 is null, "Geom2dAPI_InterCurveCurve::Segment");

        double u1, u2, v1, v2;
        u1 = _curve1!.FirstParameter;
        u2 = _curve1.LastParameter;
        if (_curve2 is null)
        {
            v1 = u1;
            v2 = u2;
        }
        else
        {
            v1 = _curve2.FirstParameter;
            v2 = _curve2.LastParameter;
        }

        IntersectionSegment2D seg = _intersector.Segment(index);
        bool isOpposite = seg.IsOpposite;

        if (seg.HasFirstPoint)
        {
            IntersectionPoint2D ipf = seg.FirstPoint();
            u1 = ipf.ParamOnFirst();

            if (isOpposite)
            {
                v2 = ipf.ParamOnSecond();
            }
            else
            {
                v1 = ipf.ParamOnSecond();
            }
        }

        if (seg.HasLastPoint)
        {
            IntersectionPoint2D ipl = seg.LastPoint();
            u2 = ipl.ParamOnFirst();

            if (isOpposite)
            {
                v1 = ipl.ParamOnSecond();
            }
            else
            {
                v2 = ipl.ParamOnSecond();
            }
        }

        curve1 = new TrimmedCurve2D(_curve1, u1, u2);
        if (_curve2 is null)
        {
            curve2 = new TrimmedCurve2D(_curve1, v1, v2);
        }
        else
        {
            curve2 = new TrimmedCurve2D(_curve2, v1, v2);
        }
    }

    /// <summary>The intersection algorithm used.</summary>
    public GInter Intersector() => _intersector;
}
