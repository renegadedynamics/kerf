// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OtherCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Fallback batch evaluator for any 2D curve adaptor: evaluates point by point through the adaptor's EvalD0 to
/// EvalDN.
/// </summary>
/// <remarks>
/// OCCT keeps a reference to the adaptor, which must outlive the evaluator; the port keeps the adaptor object.
/// </remarks>
internal sealed class OtherCurveGrid2D : Geom2dGridEvalGrid2D.ICurveEvaluator
{
    private readonly Curve2DAdaptorBase _curve;

    /// <summary>Constructor with the curve adaptor to evaluate.</summary>
    public OtherCurveGrid2D(Curve2DAdaptorBase curve)
    {
        _curve = curve;
    }

    /// <summary>Returns the curve adaptor.</summary>
    public Curve2DAdaptorBase Curve() => _curve;

    /// <summary>Evaluates all grid points; 1-based, or empty if there are no parameters.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Point2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point2D> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Point2D p = _curve.EvalD0(theParams.Value(i));
            result.SetValue(i - theParams.Lower() + 1, p);
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve2D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD1>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD1> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Curve2D.ResD1 d1 = _curve.EvalD1(theParams.Value(i));
            result[i - theParams.Lower() + 1] = new Curve2D.ResD1 { Point = d1.Point, D1 = d1.D1 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve2D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD2>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD2> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Curve2D.ResD2 d2 = _curve.EvalD2(theParams.Value(i));
            result[i - theParams.Lower() + 1] = new Curve2D.ResD2 { Point = d2.Point, D1 = d2.D1, D2 = d2.D2 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve2D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve2D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve2D.ResD3> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Curve2D.ResD3 d3 = _curve.EvalD3(theParams.Value(i));
            result[i - theParams.Lower() + 1] = new Curve2D.ResD3
            {
                Point = d3.Point,
                D1 = d3.D1,
                D2 = d3.D2,
                D3 = d3.D3,
            };
        }
        return result;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, with the adaptor's
    /// EvalDN above.
    /// </summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector2D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector2D> result = new(1, nb);

        // Reuse existing grid evaluators for orders 1-3
        if (n == 1)
        {
            NCollection_Array1<Curve2D.ResD1> d1Grid = EvaluateGridD1(theParams);
            if (d1Grid.IsEmpty())
            {
                return new NCollection_Array1<Vector2D>();
            }
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d1Grid[i].D1);
            }
        }
        else if (n == 2)
        {
            NCollection_Array1<Curve2D.ResD2> d2Grid = EvaluateGridD2(theParams);
            if (d2Grid.IsEmpty())
            {
                return new NCollection_Array1<Vector2D>();
            }
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d2Grid[i].D2);
            }
        }
        else if (n == 3)
        {
            NCollection_Array1<Curve2D.ResD3> d3Grid = EvaluateGridD3(theParams);
            if (d3Grid.IsEmpty())
            {
                return new NCollection_Array1<Vector2D>();
            }
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d3Grid[i].D3);
            }
        }
        else
        {
            // For orders > 3, use adaptor EvalDN method
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                Vector2D dn = _curve.EvalDN(theParams.Value(i), n);
                result.SetValue(i - theParams.Lower() + 1, dn);
            }
        }
        return result;
    }
}
