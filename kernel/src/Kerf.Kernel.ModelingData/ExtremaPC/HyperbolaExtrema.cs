// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Hyperbola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-hyperbola extrema: the point is projected onto the hyperbola plane and a quartic in v = e^u is solved
/// by MathPoly.Quartic; positive roots give u = ln(v).
/// </summary>
/// <remarks>The domain is fixed at construction; without one (or with an infinite one) the hyperbola branch is
/// unbounded.</remarks>
internal sealed class HyperbolaExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly Hyperbola3D _hyperbola;                   // Hyperbola geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? _domain;          // Parameter domain (null for infinite)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();     // Reusable result storage

    // Cached geometry components for fast computation
    private double _centerX, _centerY, _centerZ; // Center location
    private double _xDirX, _xDirY, _xDirZ;       // X-axis direction
    private double _yDirX, _yDirY, _yDirZ;       // Y-axis direction
    private double _axisX, _axisY, _axisZ;       // Main axis direction
    private double _majorR;                        // Major radius
    private double _minorR;                        // Minor radius
    private double _r2PlusR2Over4;                 // Precomputed (R^2 + r^2)/4

    /// <summary>Constructor with hyperbola geometry (unbounded).</summary>
    public HyperbolaExtrema(in Hyperbola3D hyperbola)
    {
        _hyperbola = hyperbola;
        _domain = null;
        CacheGeometry();
    }

    /// <summary>Constructor with hyperbola geometry and parameter domain (fixed for all queries).</summary>
    public HyperbolaExtrema(in Hyperbola3D hyperbola, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _hyperbola = hyperbola;
        _domain = domain.IsFinite() ? domain : null;
        CacheGeometry();
    }

    /// <summary>Evaluates point on hyperbola at parameter.</summary>
    public Point3D Value(double u)
    {
        // Hyperbola: P(u) = Center + R*cosh(u)*XDir + r*sinh(u)*YDir
        double cosh = Math.Cosh(u);
        double sinh = Math.Sinh(u);
        double rCosh = _majorR * cosh;
        double arSinh = _minorR * sinh;
        return new Point3D(_centerX + rCosh * _xDirX + arSinh * _yDirX,
                          _centerY + rCosh * _xDirY + arSinh * _yDirY,
                          _centerZ + rCosh * _xDirZ + arSinh * _yDirZ);
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded => _domain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain!.Value;

    /// <summary>Computes the extrema between point P and the hyperbola.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        PerformCore(p, _domain, tol, mode);
        return _result;
    }

    /// <summary>Computes the extrema between point P and the hyperbola including endpoints.</summary>
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

    /// <summary>Returns the hyperbola geometry.</summary>
    public Hyperbola3D Hyperbola() => _hyperbola;

    private void CacheGeometry()
    {
        Point3D center = _hyperbola.Location;
        _centerX = center.X;
        _centerY = center.Y;
        _centerZ = center.Z;

        Direction3D xDir = _hyperbola.XAxis.Direction;
        _xDirX = xDir.X;
        _xDirY = xDir.Y;
        _xDirZ = xDir.Z;

        Direction3D yDir = _hyperbola.YAxis.Direction;
        _yDirX = yDir.X;
        _yDirY = yDir.Y;
        _yDirZ = yDir.Z;

        Direction3D axis = _hyperbola.Axis.Direction;
        _axisX = axis.X;
        _axisY = axis.Y;
        _axisZ = axis.Z;

        _majorR = _hyperbola.MajorRadius;
        _minorR = _hyperbola.MinorRadius;
        _r2PlusR2Over4 = (_majorR * _majorR + _minorR * _minorR) / 4.0;
    }

    private global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult SolveQuartic(in Point3D p)
    {
        // Vector from center to point
        double dx = p.X - _centerX;
        double dy = p.Y - _centerY;
        double dz = p.Z - _centerZ;

        // Project point P onto the hyperbola plane
        // Height = (P - Center) . Axis
        double height = dx * _axisX + dy * _axisY + dz * _axisZ;

        // Projected point Pp = P - Height * Axis
        double ppX = p.X - height * _axisX;
        double ppY = p.Y - height * _axisY;
        double ppZ = p.Z - height * _axisZ;

        // Vector from center to projected point
        double oPpX = ppX - _centerX;
        double oPpY = ppY - _centerY;
        double oPpZ = ppZ - _centerZ;

        // Local coordinates in hyperbola frame
        double x = oPpX * _xDirX + oPpY * _xDirY + oPpZ * _xDirZ;
        double y = oPpX * _yDirX + oPpY * _yDirY + oPpZ * _yDirZ;

        // Solve quartic equation in v = e^u
        double c1 = _r2PlusR2Over4;
        double c2 = -(x * _majorR + y * _minorR) / 2.0;
        double c3 = 0.0;
        double c4 = (x * _majorR - y * _minorR) / 2.0;
        double c5 = -_r2PlusR2Over4;

        return global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(c1, c2, c3, c4, c5);
    }

    private void PerformCore(in Point3D p, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? domain, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _result.Clear();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult polyRes = SolveQuartic(p);

        if (!polyRes.IsDone)
        {
            if (polyRes.Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions)
            {
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.InfiniteSolutions;
                Point3D ptOnCurve = Value(0.0);
                _result.InfiniteSquareDistance = p.SquareDistanceTo(ptOnCurve);
            }
            else
            {
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.NumericalError;
            }
            return;
        }

        double tol2 = tol * tol;

        // Process all positive roots (v > 0 required for u = ln(v))
        for (int i = 0; i < polyRes.NbRoots; ++i)
        {
            double v = polyRes.Roots[i];
            if (v <= 0.0)
                continue;

            double u = Math.Log(v);

            // Check bounds if domain is specified
            if (domain.HasValue)
            {
                if (u < domain.Value.Min || u > domain.Value.Max)
                    continue;
            }

            Point3D curvePt = Value(u);

            // Check for duplicates
            bool duplicate = false;
            for (int j = 0; j < _result._extrema.Count; ++j)
            {
                if (curvePt.SquareDistanceTo(_result._extrema[j].Point) < tol2)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate)
                continue;

            // Determine if minimum or maximum using neighbor comparison
            double sqDist = p.SquareDistanceTo(curvePt);
            double neighborU;
            if (domain.HasValue)
            {
                // For bounded case, choose neighbor direction based on position in domain
                neighborU = u + (u < (domain.Value.Min + domain.Value.Max) * 0.5 ? 1.0 : -1.0);
                neighborU = Max(domain.Value.Min, Min(domain.Value.Max, neighborU));
            }
            else
            {
                // For unbounded case, just use +1.0
                neighborU = u + 1.0;
            }
            Point3D neighborPt = Value(neighborU);
            double neighborDist = p.SquareDistanceTo(neighborPt);
            bool isMin = sqDist < neighborDist;

            // Filter by search mode
            if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min && !isMin)
                continue;
            if (mode == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max && isMin)
                continue;

            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
            ext.Parameter = u;
            ext.Point = curvePt;
            ext.SquareDistance = sqDist;
            ext.IsMinimum = isMin;

            _result._extrema.Add(ext);
        }

        _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
    }
}
