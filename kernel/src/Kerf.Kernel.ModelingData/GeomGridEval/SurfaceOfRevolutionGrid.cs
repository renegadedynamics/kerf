// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for surface of revolution grid points: the basis curve is evaluated once per V
/// parameter with <see cref="CurveGrid"/>, then rotated about the axis for each U parameter
/// (Geom_RevolutionUtils).
/// </summary>
internal sealed class SurfaceOfRevolutionGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    private readonly RevolutionSurface? _geom;
    private readonly Curve3D? _basisCurve;
    private readonly Axis3D _axis;
    private readonly Point3D _axisLocation;

    /// <summary>Constructor with the surface of revolution to evaluate (null gives empty results).</summary>
    public SurfaceOfRevolutionGrid(RevolutionSurface? revolution)
    {
        _geom = revolution;
        _axis = new Axis3D();
        _axisLocation = new Point3D();
        if (_geom is not null)
        {
            _basisCurve = _geom.BasisCurve;
            _axis = _geom.Axis;
            _axisLocation = _axis.Location;
        }
    }

    /// <summary>Returns the geometry.</summary>
    public RevolutionSurface? Geometry() => _geom;

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        if (_basisCurve is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Batch evaluate curve points using optimized curve evaluator
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Point3D> curvePoints = curveEval.EvaluateGrid(vParams);
        if (curvePoints.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);
        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                RevolutionUtils.CalculateD0(curvePoints.Value(j), u, _axis, out Point3D p);
                result.SetValue(i, j, p);
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basisCurve is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Batch evaluate curve D1
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD1> curveD1 = curveEval.EvaluateGridD1(vParams);
        if (curveD1.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);
        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                ref Curve3D.ResD1 curveData = ref curveD1.ChangeValue(j);
                RevolutionUtils.CalculateD1(curveData.Point,
                                                 curveData.D1,
                                                 u,
                                                 _axis,
                                                 out Point3D p,
                                                 out Vector3D d1U,
                                                 out Vector3D d1V);
                result[i, j] = new Surface.ResD1 { Point = p, D1U = d1U, D1V = d1V };
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basisCurve is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Batch evaluate curve D2
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD2> curveD2 = curveEval.EvaluateGridD2(vParams);
        if (curveD2.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);
        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                ref Curve3D.ResD2 curveData = ref curveD2.ChangeValue(j);
                RevolutionUtils.CalculateD2(curveData.Point,
                                                 curveData.D1,
                                                 curveData.D2,
                                                 u,
                                                 _axis,
                                                 out Point3D p,
                                                 out Vector3D d1U,
                                                 out Vector3D d1V,
                                                 out Vector3D d2U,
                                                 out Vector3D d2V,
                                                 out Vector3D d2UV);
                result[i, j] = new Surface.ResD2
                {
                    Point = p,
                    D1U = d1U,
                    D1V = d1V,
                    D2U = d2U,
                    D2V = d2V,
                    D2UV = d2UV,
                };
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basisCurve is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Batch evaluate curve D3
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD3> curveD3 = curveEval.EvaluateGridD3(vParams);
        if (curveD3.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);
        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                ref Curve3D.ResD3 curveData = ref curveD3.ChangeValue(j);
                RevolutionUtils.CalculateD3(curveData.Point,
                                                 curveData.D1,
                                                 curveData.D2,
                                                 curveData.D3,
                                                 u,
                                                 _axis,
                                                 out Point3D p,
                                                 out Vector3D d1U,
                                                 out Vector3D d1V,
                                                 out Vector3D d2U,
                                                 out Vector3D d2V,
                                                 out Vector3D d2UV,
                                                 out Vector3D d3U,
                                                 out Vector3D d3V,
                                                 out Vector3D d3UUV,
                                                 out Vector3D d3UVV);
                result[i, j] = new Surface.ResD3
                {
                    Point = p,
                    D1U = d1U,
                    D1V = d1V,
                    D2U = d2U,
                    D2V = d2V,
                    D2UV = d2UV,
                    D3U = d3U,
                    D3V = d3V,
                    D3UUV = d3UUV,
                    D3UVV = d3UVV,
                };
            }
        }
        return result;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        if (_basisCurve is null || uParams.IsEmpty() || vParams.IsEmpty() || nu < 0
            || nv < 0 || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();
        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);

        // Get curve data
        CurveGrid curveEval = new(_basisCurve);

        if (nu == 0)
        {
            // Pure V derivative = curve derivative, rotated
            NCollection_Array1<Vector3D> curveDN = curveEval.EvaluateGridDN(vParams, nv);
            for (int i = 1; i <= nbU; ++i)
            {
                double u = uParams.Value(uParams.Lower() + i - 1);
                for (int j = 1; j <= nbV; ++j)
                {
                    Vector3D dn = RevolutionUtils.CalculateDN(curveDN.Value(j), u, _axis, nu, nv);
                    result.SetValue(i, j, dn);
                }
            }
        }
        else
        {
            // Mixed or pure U derivative
            // Get curve point or derivative depending on theNV
            NCollection_Array1<Vector3D> curveDV = new();
            NCollection_Array1<Point3D> curvePts = new();
            if (nv == 0)
            {
                curvePts = curveEval.EvaluateGrid(vParams);
            }
            else
            {
                curveDV = curveEval.EvaluateGridDN(vParams, nv);
            }

            for (int i = 1; i <= nbU; ++i)
            {
                double u = uParams.Value(uParams.Lower() + i - 1);
                for (int j = 1; j <= nbV; ++j)
                {
                    Vector3D curvePtOrDN;
                    if (nv == 0)
                    {
                        // For pure U derivative, pass (P - AxisLocation) as the base vector
                        curvePtOrDN = new Vector3D(curvePts.Value(j).XYZ() - _axisLocation.XYZ());
                    }
                    else
                    {
                        curvePtOrDN = curveDV.Value(j);
                    }
                    Vector3D dn = RevolutionUtils.CalculateDN(curvePtOrDN, u, _axis, nu, nv);
                    result.SetValue(i, j, dn);
                }
            }
        }

        return result;
    }
}
