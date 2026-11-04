// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Ellipse.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-ellipse extrema: the point is projected onto the ellipse plane and the trigonometric equation
/// (B² - A²) cos u sin u - B Y cos u + A X sin u = 0 is solved by MathRoot.Trigonometric (up to 4 extrema).
/// </summary>
/// <remarks>When the point projects onto the centre of a circular ellipse (A = B): Status.InfiniteSolutions. The
/// domain is fixed at construction; a full period means the whole ellipse.</remarks>
internal sealed class EllipseExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly Ellipse3D _ellipse;                    // Ellipse geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? _domain;          // Parameter domain (null for full ellipse)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();     // Reusable result storage

    /// <summary>Constructor with ellipse geometry (full ellipse).</summary>
    public EllipseExtrema(in Ellipse3D ellipse)
    {
        _ellipse = ellipse;
        _domain = null;
    }

    /// <summary>Constructor with ellipse geometry and parameter domain in radians (fixed for all queries).</summary>
    public EllipseExtrema(in Ellipse3D ellipse, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _ellipse = ellipse;
        _domain = domain.IsFullPeriod(2.0 * M_PI) ? null : domain;
    }

    /// <summary>Evaluates point on ellipse at parameter (radians).</summary>
    public Point3D Value(double u) => global::Kerf.Engine.Geometry.ElCLib.Value(u, _ellipse);

    /// <summary>Returns true if domain is bounded (partial arc).</summary>
    public bool IsBounded => _domain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain!.Value;

    /// <summary>Computes the extrema between point P and the ellipse.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        // Use stored domain or full parameter range [0, 2*PI]
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = _domain ?? new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 2.0 * M_PI);
        PerformCore(p, domain, tol, mode);
        return _result;
    }

    /// <summary>Computes the extrema between point P and the ellipse arc including endpoints.</summary>
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

    /// <summary>Returns the ellipse geometry.</summary>
    public Ellipse3D Ellipse() => _ellipse;

    // Core algorithm - finds extrema with bounds checking. Stores results in myResult.
    private void PerformCore(in Point3D p, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _result.Clear();

        double theUMin = domain.Min;
        double theUMax = domain.Max;

        // Step 1: Project point P onto the ellipse plane
        Point3D center = _ellipse.Location;
        Direction3D axis = _ellipse.Axis.Direction;
        Vector3D toP = new(center, p);
        double height = toP.Dot(new Vector3D(axis));
        Vector3D trsl = new Vector3D(axis) * (-height);
        Point3D pp = p.Translated(trsl);

        // Step 2: Get ellipse radii and compute local coordinates
        double a = _ellipse.MajorRadius;
        double b = _ellipse.MinorRadius;

        Vector3D oPp = new(center, pp);
        double oPpMag = oPp.Magnitude;

        // Check for degenerate case: point at center with circular ellipse
        if (oPpMag < tol)
        {
            if (Math.Abs(a - b) < tol)
            {
                // Point at center of a circle - infinite solutions
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.InfiniteSolutions;
                _result.InfiniteSquareDistance = a * a + height * height;
                return;
            }
            // For non-circular ellipse at center, we still get valid extrema at semi-axes
        }

        // Local coordinates
        double x = oPp.Dot(new Vector3D(_ellipse.XAxis.Direction));
        double y = oPp.Dot(new Vector3D(_ellipse.YAxis.Direction));

        // Step 3: Solve trigonometric equation
        // (B^2 - A^2)*cos*sin - B*Y*cos + A*X*sin = 0
        // In MathRoot::Trigonometric form: a*cos^2 + 2*b*cos*sin + c*cos + d*sin + e = 0
        // a = 0, 2*b = (B^2 - A^2), c = -B*Y, d = A*X, e = 0
        double ko2 = (b * b - a * a) / 2.0;
        double ko3 = -b * y;
        double ko4 = a * x;

        // MathRoot::Trigonometric handles all special cases including Y ~= 0
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult trigRes =
          global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, ko2, ko3, ko4, 0.0, theUMin, theUMax);

        if (!trigRes.IsDone)
        {
            if (trigRes.InfiniteRoots)
            {
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.InfiniteSolutions;
                // For infinite case, compute distance to any point (use U=0)
                Point3D ptOnCurve = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, _ellipse);
                _result.InfiniteSquareDistance = p.SquareDistanceTo(ptOnCurve);
            }
            else
            {
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NumericalError;
            }
            return;
        }

        // Step 4: Collect extrema
        double tol2 = tol * tol;

        // Add solutions from solver
        for (int i = 0; i < trigRes.NbRoots; ++i)
        {
            AddExtremum(trigRes.Roots[i], p, theUMin, theUMax, tol, tol2, mode);
        }

        _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
    }

    // performCore's lambda addExtremum
    private void AddExtremum(double aU, in Point3D p, double uMin, double uMax, double tol, double aTol2,
                             global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        Point3D curvePt = global::Kerf.Engine.Geometry.ElCLib.Value(aU, _ellipse);

        // Check for duplicates using parameter proximity (more robust)
        for (int j = 0; j < _result._extrema.Count; ++j)
        {
            if (Math.Abs(_result._extrema[j].Parameter - aU) < tol)
            {
                return;
            }
            if (curvePt.SquareDistanceTo(_result._extrema[j].Point) < aTol2)
            {
                return;
            }
        }

        double sqDist = p.SquareDistanceTo(curvePt);

        // Determine if this is a minimum or maximum by checking neighboring points
        // Use step relative to parameter range
        double step = Max(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_NEIGHBOR_STEP_RATIO,
                           (uMax - uMin) * global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.THE_NEIGHBOR_STEP_RATIO);
        Point3D ptPlus = global::Kerf.Engine.Geometry.ElCLib.Value(aU + step, _ellipse);
        Point3D ptMinus = global::Kerf.Engine.Geometry.ElCLib.Value(aU - step, _ellipse);
        double distPlus = p.SquareDistanceTo(ptPlus);
        double distMinus = p.SquareDistanceTo(ptMinus);
        bool isMin = (sqDist <= distPlus) && (sqDist <= distMinus);

        // Filter by search mode
        if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min && !isMin)
        {
            return;
        }
        if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max && isMin)
        {
            return;
        }

        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
        ext.Parameter = aU;
        ext.Point = curvePt;
        ext.SquareDistance = sqDist;
        ext.IsMinimum = isMin;

        _result._extrema.Add(ext);
    }
}
