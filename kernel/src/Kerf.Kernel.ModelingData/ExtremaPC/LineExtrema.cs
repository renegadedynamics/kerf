// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Point-line extrema: the closest point by direct projection, u = (P - O) . D. The domain is fixed at
/// construction; without one (or with an infinite one) the line is unbounded.
/// </summary>
/// <remarks>Lines always have exactly one extremum (minimum) if within bounds. <c>Perform</c> returns the
/// evaluator's own reusable result, which the next call overwrites.</remarks>
internal sealed class LineExtrema : global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ICurveEvaluator
{
    private readonly Line3D _line;                         // Line geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? _domain;          // Parameter domain (null for unbounded)
    private readonly global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result _result = new();     // Reusable result storage

    /// <summary>Constructor with line geometry (unbounded).</summary>
    public LineExtrema(in Line3D line)
    {
        _line = line;
        _domain = null;
    }

    /// <summary>Constructor with line geometry and parameter domain (fixed for all queries).</summary>
    public LineExtrema(in Line3D line, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain)
    {
        _line = line;
        _domain = domain.IsFinite() ? domain : null;
    }

    /// <summary>Evaluates point on line at parameter.</summary>
    public Point3D Value(double u)
    {
        return _line.Location.Translated(u * new Vector3D(_line.Direction));
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded => _domain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => _domain!.Value;

    /// <summary>Computes the extremum between point P and the line (theMode is unused: always the minimum).</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result Perform(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        _ = mode; // Lines always have exactly one extremum (minimum)

        _result.Clear();

        // Compute projection parameter: u = (P - O) . Direction
        Direction3D dir = _line.Direction;
        Point3D origin = _line.Location;
        Vector3D vec = new(origin, p);
        double u = vec.Dot(new Vector3D(dir));

        // Check bounds if domain is specified
        if (_domain.HasValue)
        {
            global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D domain = _domain.Value;
            if (u < domain.Min - tol || u > domain.Max + tol)
            {
                // Projection is outside bounds - no interior extremum
                _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
                return _result;
            }
            // Clamp to bounds
            u = u < domain.Min ? domain.Min : (domain.Max < u ? domain.Max : u); // std::clamp
        }

        // Compute point on line at projected parameter
        Point3D ptOnLine = origin.Translated(u * new Vector3D(dir));

        // Store result
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.ExtremumResult ext = new();
        ext.Parameter = u;
        ext.Point = ptOnLine;
        ext.SquareDistance = p.SquareDistanceTo(ptOnLine);
        ext.IsMinimum = true;

        _result._extrema.Add(ext);
        _result.Status = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK;
        return _result;
    }

    /// <summary>Computes the extrema between point P and the line segment including endpoints.</summary>
    public global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result PerformWithEndpoints(in Point3D p, double tol, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode mode = global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax)
    {
        _ = Perform(p, tol, mode);

        if (_result.Status == global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Status.OK && _domain.HasValue)
        {
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.AddEndpointExtrema(_result, p, _domain.Value, this, tol, mode);
        }

        return _result;
    }

    /// <summary>Returns the line geometry.</summary>
    public Line3D Line() => _line;
}
