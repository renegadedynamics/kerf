// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for offset curve grid points: evaluates the basis curve with
/// <see cref="CurveGrid"/>, then applies the offset (Geom_OffsetCurveUtils).
/// </summary>
/// <remarks>
/// If the offset can't be computed at one parameter (a zero basis tangent that can't be adjusted), the whole
/// result is empty, as in OCCT.
/// </remarks>
public sealed class OffsetCurveGrid : GeomGridEvalGrid.ICurveEvaluator
{
    private readonly OffsetCurve3D? _geom;
    private readonly Curve3D? _basis;
    private readonly double _offset;
    private readonly Direction3D _direction = new();

    /// <summary>Constructor with the offset curve to evaluate (null gives empty results).</summary>
    public OffsetCurveGrid(OffsetCurve3D? offset)
    {
        _geom = offset;
        _offset = 0.0;
        if (_geom is not null)
        {
            _offset = _geom.Offset();
            _basis = _geom.BasisCurve();
            _direction = _geom.Direction();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public OffsetCurve3D? Geometry() => _geom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (_basis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        // Offset D0 requires basis D1 to compute offset direction
        // Batch evaluate basis curve D1
        CurveGrid basisEval = new(_basis);

        NCollection_Array1<Curve3D.ResD1> basisD1 = basisEval.EvaluateGridD1(theParams);
        if (basisD1.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int nbParams = theParams.Length();
        NCollection_Array1<Point3D> result = new(1, nbParams);

        Xyz dirXYZ = _direction.XYZ();

        for (int i = 1; i <= nbParams; ++i)
        {
            ref readonly Curve3D.ResD1 basis = ref basisD1[i];
            Point3D p = basis.Point;
            if (!OffsetCurveUtils3D.CalculateD0(ref p, basis.D1, dirXYZ, _offset))
            {
                return new NCollection_Array1<Point3D>();
            }
            result.SetValue(i, p);
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Curve3D.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (_basis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        // Offset D1 requires basis D2
        // Batch evaluate basis curve D2
        CurveGrid basisEval = new(_basis);

        NCollection_Array1<Curve3D.ResD2> basisD2 = basisEval.EvaluateGridD2(theParams);
        if (basisD2.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD1>();
        }

        int nbParams = theParams.Length();
        NCollection_Array1<Curve3D.ResD1> result = new(1, nbParams);

        Xyz dirXYZ = _direction.XYZ();

        for (int i = 1; i <= nbParams; ++i)
        {
            ref readonly Curve3D.ResD2 basis = ref basisD2[i];
            Point3D p = basis.Point;
            Vector3D d1 = basis.D1;
            if (!OffsetCurveUtils3D.CalculateD1(ref p, ref d1, basis.D2, dirXYZ, _offset))
            {
                return new NCollection_Array1<Curve3D.ResD1>();
            }
            result[i] = new Curve3D.ResD1 { Point = p, D1 = d1 };
        }
        return result;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Curve3D.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (_basis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        // Offset D2 requires basis D3
        // Batch evaluate basis curve D3
        CurveGrid basisEval = new(_basis);

        NCollection_Array1<Curve3D.ResD3> basisD3 = basisEval.EvaluateGridD3(theParams);
        if (basisD3.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD2>();
        }

        int nbParams = theParams.Length();
        NCollection_Array1<Curve3D.ResD2> result = new(1, nbParams);

        Xyz dirXYZ = _direction.XYZ();

        for (int i = 1; i <= nbParams; ++i)
        {
            ref readonly Curve3D.ResD3 basis = ref basisD3[i];
            Point3D p = basis.Point;
            Vector3D d1 = basis.D1;
            Vector3D d2 = basis.D2;
            Vector3D d3 = basis.D3;

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (d1.SquareMagnitude() <= Constants.Resolution())
            {
                Vector3D dummyD4 = new();
                if (!OffsetCurveUtils3D.AdjustDerivative(_basis,
                                                              3,
                                                              theParams.Value(theParams.Lower() + i - 1),
                                                              ref d1,
                                                              ref d2,
                                                              ref d3,
                                                              ref dummyD4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Curve3D.ResD2>();
                }
            }

            if (!OffsetCurveUtils3D.CalculateD2(ref p, ref d1, ref d2, d3, dirXYZ, _offset, isDirectionChange))
            {
                return new NCollection_Array1<Curve3D.ResD2>();
            }
            result[i] = new Curve3D.ResD2 { Point = p, D1 = d1, D2 = d2 };
        }
        return result;
    }

    /// <summary>
    /// Evaluates all grid points with derivatives up to the third; the basis's fourth derivative is evaluated point
    /// by point.
    /// </summary>
    public NCollection_Array1<Curve3D.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (_basis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        // Offset D3 requires basis D3 + D4
        // Batch evaluate basis curve D3, get D4 individually
        CurveGrid basisEval = new(_basis);

        NCollection_Array1<Curve3D.ResD3> basisD3 = basisEval.EvaluateGridD3(theParams);
        if (basisD3.IsEmpty())
        {
            return new NCollection_Array1<Curve3D.ResD3>();
        }

        int nbParams = theParams.Length();
        NCollection_Array1<Curve3D.ResD3> result = new(1, nbParams);

        Xyz dirXYZ = _direction.XYZ();

        for (int i = 1; i <= nbParams; ++i)
        {
            double param = theParams.Value(theParams.Lower() + i - 1);

            ref readonly Curve3D.ResD3 basis = ref basisD3[i];
            Point3D p = basis.Point;
            Vector3D d1 = basis.D1;
            Vector3D d2 = basis.D2;
            Vector3D d3 = basis.D3;
            Vector3D d4 = _basis.DN(param, 4);

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (d1.SquareMagnitude() <= Constants.Resolution())
            {
                if (!OffsetCurveUtils3D.AdjustDerivative(_basis,
                                                              4,
                                                              param,
                                                              ref d1,
                                                              ref d2,
                                                              ref d3,
                                                              ref d4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Curve3D.ResD3>();
                }
            }

            if (!OffsetCurveUtils3D.CalculateD3(ref p, ref d1, ref d2, ref d3, d4, dirXYZ, _offset,
                                                   isDirectionChange))
            {
                return new NCollection_Array1<Curve3D.ResD3>();
            }
            result[i] = new Curve3D.ResD3 { Point = p, D1 = d1, D2 = d2, D3 = d3 };
        }
        return result;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, and as the basis
    /// curve's DN above.
    /// </summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int n)
    {
        if (_basis is null || theParams.IsEmpty() || n < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int nbParams = theParams.Length();

        // Reuse optimized grid evaluators for orders 1-3
        // OCCT: when the offset can't be computed at a parameter, EvaluateGridD1/D2/D3 return an empty array that
        // is then read out of range (undefined in OCCT, Standard_OutOfRange here); Geom2dGridEval checks for it.
        if (n == 1)
        {
            NCollection_Array1<Curve3D.ResD1> d1Grid = EvaluateGridD1(theParams);
            NCollection_Array1<Vector3D> result = new(1, nbParams);
            for (int i = 1; i <= nbParams; ++i)
            {
                result.SetValue(i, d1Grid[i].D1);
            }
            return result;
        }
        else if (n == 2)
        {
            NCollection_Array1<Curve3D.ResD2> d2Grid = EvaluateGridD2(theParams);
            NCollection_Array1<Vector3D> result = new(1, nbParams);
            for (int i = 1; i <= nbParams; ++i)
            {
                result.SetValue(i, d2Grid[i].D2);
            }
            return result;
        }
        else if (n == 3)
        {
            NCollection_Array1<Curve3D.ResD3> d3Grid = EvaluateGridD3(theParams);
            NCollection_Array1<Vector3D> result = new(1, nbParams);
            for (int i = 1; i <= nbParams; ++i)
            {
                result.SetValue(i, d3Grid[i].D3);
            }
            return result;
        }
        else
        {
            // For orders > 3, offset curve DN = basis curve DN (no offset contribution)
            // Batch evaluate basis curve DN
            CurveGrid basisEval = new(_basis);
            return basisEval.EvaluateGridDN(theParams, n);
        }
    }
}
