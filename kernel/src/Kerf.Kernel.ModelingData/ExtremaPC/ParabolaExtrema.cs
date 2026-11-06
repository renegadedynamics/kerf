// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Parabola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-parabola extrema: the point is projected onto the parabola plane and the cubic
/// (1/4F) u³ + (2F - X) u - 2F Y = 0 is solved by MathPoly.Cubic.
/// </summary>
/// <remarks>The domain is fixed at construction; without one (or with an infinite one) the parabola is
/// unbounded.</remarks>
internal sealed class ParabolaExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly Parabola3D _parabola;                   // Parabola geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? _domain;          // Parameter domain (null for infinite)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();     // Reusable result storage

    // Cached geometry components for fast computation
    private double _vertexX, _vertexY, _vertexZ; // Vertex location
    private double _xDirX, _xDirY, _xDirZ;       // X-axis direction
    private double _yDirX, _yDirY, _yDirZ;       // Y-axis direction
    private double _axisX, _axisY, _axisZ;       // Main axis direction
    private double _focal;                         // Focal length
    private double _my1Over4F;                       // Precomputed 1/(4*F)
    private double _my2F;                            // Precomputed 2*F

    /// <summary>Constructor with parabola geometry (unbounded).</summary>
    public ParabolaExtrema(in Parabola3D parabola)
    {
        _parabola = parabola;
        _domain = null;
        CacheGeometry();
    }

    /// <summary>Constructor with parabola geometry and parameter domain (fixed for all queries).</summary>
    public ParabolaExtrema(in Parabola3D parabola, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _parabola = parabola;
        _domain = domain.IsFinite() ? domain : null;
        CacheGeometry();
    }

    /// <summary>Evaluates point on parabola at parameter.</summary>
    public Point3D Value(double u)
    {
        // Parabola: P(u) = Vertex + (u^2/4F)*XDir + u*YDir
        double x = u * u * _my1Over4F;
        return new Point3D(_vertexX + x * _xDirX + u * _yDirX,
                          _vertexY + x * _xDirY + u * _yDirY,
                          _vertexZ + x * _xDirZ + u * _yDirZ);
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded => _domain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain!.Value;

    /// <summary>Computes the extrema between point P and the parabola.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        PerformCore(p, _domain, tol, mode);
        return _result;
    }

    /// <summary>Computes the extrema between point P and the parabola including endpoints.</summary>
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

    /// <summary>Returns the parabola geometry.</summary>
    public Parabola3D Parabola() => _parabola;

    private void CacheGeometry()
    {
        Point3D vertex = _parabola.Location;
        _vertexX = vertex.X;
        _vertexY = vertex.Y;
        _vertexZ = vertex.Z;

        Direction3D xDir = _parabola.XAxis.Direction;
        _xDirX = xDir.X;
        _xDirY = xDir.Y;
        _xDirZ = xDir.Z;

        Direction3D yDir = _parabola.YAxis.Direction;
        _yDirX = yDir.X;
        _yDirY = yDir.Y;
        _yDirZ = yDir.Z;

        Direction3D axis = _parabola.Axis.Direction;
        _axisX = axis.X;
        _axisY = axis.Y;
        _axisZ = axis.Z;

        _focal = _parabola.Focal;
        _my1Over4F = 1.0 / (4.0 * _focal);
        _my2F = 2.0 * _focal;
    }

    private global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult SolveCubic(in Point3D p)
    {
        // Vector from vertex to point
        double dx = p.X - _vertexX;
        double dy = p.Y - _vertexY;
        double dz = p.Z - _vertexZ;

        // Project point P onto the parabola plane
        // Height = (P - Vertex) . Axis
        double height = dx * _axisX + dy * _axisY + dz * _axisZ;

        // Projected point Pp = P - Height * Axis
        double ppX = p.X - height * _axisX;
        double ppY = p.Y - height * _axisY;
        double ppZ = p.Z - height * _axisZ;

        // Vector from vertex to projected point
        double oPpX = ppX - _vertexX;
        double oPpY = ppY - _vertexY;
        double oPpZ = ppZ - _vertexZ;

        // Local coordinates in parabola frame
        double x = oPpX * _xDirX + oPpY * _xDirY + oPpZ * _xDirZ;
        double y = oPpX * _yDirX + oPpY * _yDirY + oPpZ * _yDirZ;

        // Solve cubic equation: (1/(4F)) * u^3 + (2F - X) * u - 2F*Y = 0
        return global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(_my1Over4F, 0.0, _my2F - x, -_my2F * y);
    }

    private void PerformCore(in Point3D p, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? domain, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode)
    {
        _ = tol; // Tolerance used for endpoint detection

        _result.Clear();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult polyRes = SolveCubic(p);

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

        double tol2 = global::Kerf.Engine.Foundation.Precision.SquareConfusion();

        // Process all roots
        for (int i = 0; i < polyRes.NbRoots; ++i)
        {
            double u = polyRes.Roots[i];

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
            bool isMin = sqDist <= neighborDist;

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
