// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Fallback batch evaluator for any curve adaptor: evaluates point by point through the adaptor's Value, D1 to
/// D3 and DN.
/// </summary>
/// <remarks>
/// OCCT keeps a reference to the adaptor, which must outlive the evaluator; the port keeps the adaptor object.
/// </remarks>
internal sealed class OtherCurveGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly CurveAdaptorBase _curve;

    /// <summary>Constructor with the curve adaptor to evaluate.</summary>
    public OtherCurveGrid(CurveAdaptorBase curve)
    {
        _curve = curve;
    }

    /// <summary>Returns the curve adaptor.</summary>
    public CurveAdaptorBase Curve() => _curve;

    /// <summary>Evaluates all grid points; 1-based, or empty if there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Point3D> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            result.SetValue(i - theParams.Lower() + 1, _curve.Value(theParams.Value(i)));
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD1> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            _curve.D1(theParams.Value(i), out Point3D point, out Vector3D d1);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD1 { Point = point, D1 = d1 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD2> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            _curve.D2(theParams.Value(i), out Point3D point, out Vector3D d1, out Vector3D d2);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD2 { Point = point, D1 = d1, D2 = d2 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Curve3D.ResD3> result = new(1, nb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            _curve.D3(theParams.Value(i), out Point3D point, out Vector3D d1, out Vector3D d2, out Vector3D d3);
            result[i - theParams.Lower() + 1] = new Curve3D.ResD3 { Point = point, D1 = d1, D2 = d2, D3 = d3 };
        }
        return result;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, with the adaptor's DN
    /// above.
    /// </summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nb = theParams.Length();
        NCollection_Array1<Vector3D> result = new(1, nb);

        // Reuse existing grid evaluators for orders 1-3
        if (n == 1)
        {
            NCollection_Array1<Curve3D.ResD1> d1Grid = EvaluateGridD1(theParams);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d1Grid[i].D1);
            }
        }
        else if (n == 2)
        {
            NCollection_Array1<Curve3D.ResD2> d2Grid = EvaluateGridD2(theParams);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d2Grid[i].D2);
            }
        }
        else if (n == 3)
        {
            NCollection_Array1<Curve3D.ResD3> d3Grid = EvaluateGridD3(theParams);
            for (int i = 1; i <= nb; ++i)
            {
                result.SetValue(i, d3Grid[i].D3);
            }
        }
        else
        {
            // For orders > 3, use adaptor DN method
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                result.SetValue(i - theParams.Lower() + 1, _curve.DN(theParams.Value(i), n));
            }
        }
        return result;
    }
}
