// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for 2D Bezier curve grid points: one BSplCLib_Cache built for the single span,
/// then polynomial evaluation at each parameter.
/// </summary>
internal sealed class BezierCurveGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly BezierCurve2D? _geom;

    /// <summary>Constructor with the Bezier curve to evaluate (null gives empty results).</summary>
    public BezierCurveGrid2D(BezierCurve2D? curve)
    {
        _geom = curve;
    }

    /// <summary>Returns the geometry.</summary>
    public BezierCurve2D? Geometry() => _geom;

    // Creates a BSplCLib_Cache for a 2D Bezier curve, ready for evaluation.
    private static Cache CreateBezierCache2d(BezierCurve2D curve)
    {
        NCollection_Array1<double> knotSequence = curve.InternalKnotSequence();
        NCollection_Array1<Point2D> poles = curve.InternalPoles();
        NCollection_Array1<double>? weights = curve.InternalWeights();
        Cache cache = new(curve.Degree, false, knotSequence, poles, weights);
        cache.BuildCache(0.5, knotSequence, poles, weights);
        return cache;
    }

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        Cache cache = CreateBezierCache2d(_geom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            cache.D0(theParams.Value(i), out Point2D p);
            result.SetValue(i - theParams.Lower() + 1, p);
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve2D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD1>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD1> result = new(1, nb);

        Cache cache = CreateBezierCache2d(_geom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            cache.D1(theParams.Value(i), out Point2D p, out Vector2D d1);
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1 { Point = p, D1 = d1 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve2D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD2>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD2> result = new(1, nb);

        Cache cache = CreateBezierCache2d(_geom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            cache.D2(theParams.Value(i), out Point2D p, out Vector2D d1, out Vector2D d2);
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2 { Point = p, D1 = d1, D2 = d2 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_geom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD3> result = new(1, nb);

        Cache cache = CreateBezierCache2d(_geom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            cache.D3(theParams.Value(i), out Point2D p, out Vector2D d1, out Vector2D d2, out Vector2D d3);
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3 { Point = p, D1 = d1, D2 = d2, D3 = d3 };
        }
        return result;
    }

    /// <summary>Evaluates the Nth derivative at all grid points (zero above the degree, also when rational, as in OCCT).</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_geom is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector2D> result = new(1, nb);

        // For Bezier curves, derivatives become zero when order exceeds degree
        // OCCT: not for a rational curve, whose derivatives above the degree are generally not zero
        int degree = _geom.Degree;
        if (n > degree)
        {
            Vector2D zeroVec = new(0.0, 0.0);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, zeroVec);
            }
            return result;
        }

        // Get poles, weights, and flat knots from geometry
        NCollection_Array1<Point2D> poles = _geom.InternalPoles();
        NCollection_Array1<double>? weights = _geom.InternalWeights();
        NCollection_Array1<double> knotSequence = _geom.InternalKnotSequence();

        // Bezier has a single span (index 0 with flat knots), non-periodic
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(theParams.Value(i),
                        n,
                        0, // span index (single span for Bezier with flat knots)
                        degree,
                        false, // not periodic
                        poles,
                        weights,
                        knotSequence,
                        null, // no multiplicities with flat knots
                        out Vector2D dn);
            result.SetValue(i - theParams.Lower() + 1, dn);
        }
        return result;
    }
}
