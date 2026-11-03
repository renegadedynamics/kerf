// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Circle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-circle extrema: the point is projected onto the circle plane and the angle of the projection gives the
/// closest point (minimum) and the opposite one (maximum).
/// </summary>
/// <remarks>When the point projects onto the centre, every point of the circle is at the same distance:
/// Status.InfiniteSolutions with InfiniteSquareDistance = R² + h². The domain is fixed at construction; a full
/// period means the whole circle.</remarks>
public sealed class CircleExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly Circle3D _circle;                      // Circle geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? _domain;          // Parameter domain (null for full circle)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();     // Reusable result storage

    /// <summary>Constructor with circle geometry (full circle).</summary>
    public CircleExtrema(in Circle3D circle)
    {
        _circle = circle;
        _domain = null;
    }

    /// <summary>Constructor with circle geometry and parameter domain in radians (fixed for all queries).</summary>
    public CircleExtrema(in Circle3D circle, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _circle = circle;
        _domain = domain.IsFullPeriod(2.0 * M_PI) ? null : domain;
    }

    /// <summary>Evaluates point on circle at parameter (radians).</summary>
    public Point3D Value(double u) => global::Kerf.Engine.Geometry.ElCLib.Value(u, _circle);

    /// <summary>Returns true if domain is bounded (partial arc).</summary>
    public bool IsBounded => _domain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain!.Value;

    /// <summary>Computes the extrema between point P and the circle.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        _result.Clear();

        // Step 1: Project point P onto the circle plane
        Point3D center = _circle.Location();
        Direction3D axis = _circle.Axis().Direction();
        Vector3D toP = new(center, p);
        double height = toP.Dot(new Vector3D(axis));
        Vector3D trsl = new Vector3D(axis) * (-height);
        Point3D pp = p.Translated(trsl);

        // Step 2: Check for degenerate case - point projects to center
        Vector3D oPp = new(center, pp);
        double oPpMag = oPp.Magnitude();

        if (oPpMag < tol)
        {
            // Point is on the circle axis - all points on circle are equidistant
            _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.InfiniteSolutions;
            double radius = _circle.Radius();
            _result.InfiniteSquareDistance = radius * radius + height * height;
            return _result;
        }

        // Step 3: Compute the angle of the closest point
        // Us1 corresponds to minimum distance (closest point)
        double us1 = _circle.XAxis().Direction().AngleWithRef(new Direction3D(oPp), axis);

        // Handle angle boundaries
        double angTol = global::Kerf.Engine.Foundation.Precision.Angular();
        if (us1 + M_PI < angTol)
        {
            us1 = -M_PI;
        }
        else if (us1 - M_PI > -angTol)
        {
            us1 = M_PI;
        }

        // Us2 = Us1 + PI corresponds to maximum distance (farthest point)
        double us2 = us1 + M_PI;

        // Step 4: For bounded case, adjust for periodicity
        double tolU = global::Kerf.Engine.Foundation.Precision.Angular();
        if (_domain.HasValue)
        {
            double theUMin = _domain.Value.Min;

            double radius = _circle.Radius();
            if (radius > Constants.Resolution())
            {
                tolU = tol / radius;
            }

            // Adjust angles to be within [theUMin, theUMin + 2*PI]
            double uinf = theUMin;
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(theUMin, theUMin + 2.0 * M_PI, tolU, ref uinf, ref us1);
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(theUMin, theUMin + 2.0 * M_PI, tolU, ref uinf, ref us2);

            // Handle boundary tolerance
            if (Math.Abs(us1 - 2.0 * M_PI - theUMin) < tolU)
            {
                us1 = theUMin;
            }
            if (Math.Abs(us2 - 2.0 * M_PI - theUMin) < tolU)
            {
                us2 = theUMin;
            }
        }

        // Step 5: Add extrema (with bounds check if domain specified)
        // Skip based on search mode: i=0 is minimum, i=1 is maximum
        int start = (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max) ? 1 : 0;
        int end = (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min) ? 1 : 2;

        Span<double> solutions = [us1, us2];

        for (int i = start; i < end; ++i)
        {
            double u = solutions[i];

            // Check bounds only if domain is specified
            if (_domain.HasValue)
            {
                if (u < _domain.Value.Min - tolU || u > _domain.Value.Max + tolU)
                    continue;
            }

            Point3D curvePt = global::Kerf.Engine.Geometry.ElCLib.Value(u, _circle);

            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
            ext.Parameter = u;
            ext.Point = curvePt;
            ext.SquareDistance = p.SquareDistance(curvePt);
            ext.IsMinimum = (i == 0); // First solution is minimum, second is maximum

            _result._extrema.Add(ext);
        }

        _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
        return _result;
    }

    /// <summary>Computes the extrema between point P and the circle arc including endpoints.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PerformWithEndpoints(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        _ = Perform(p, tol, mode);

        // Add endpoints if interior computation succeeded and domain is bounded
        if (_result.Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK && _domain.HasValue)
        {
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.AddEndpointExtrema(_result, p, _domain.Value, this, tol, mode);
        }

        return _result;
    }

    /// <summary>Returns the circle geometry.</summary>
    public Circle3D Circle() => _circle;
}
