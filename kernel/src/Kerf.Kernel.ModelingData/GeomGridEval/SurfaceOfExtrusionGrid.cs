// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfExtrusion.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for surface of linear extrusion grid points, S(u, v) = C(u) + v * Direction: the
/// basis curve is evaluated once per U parameter with <see cref="CurveGrid"/> (Geom_ExtrusionUtils).
/// </summary>
public sealed class SurfaceOfExtrusionGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    private readonly ExtrusionSurface? _geom;
    private readonly Curve3D? _basisCurve;
    private readonly Direction3D _direction;

    /// <summary>Constructor with the extrusion surface to evaluate (null gives empty results).</summary>
    public SurfaceOfExtrusionGrid(ExtrusionSurface? extrusion)
    {
        _geom = extrusion;
        _direction = new Direction3D();
        if (_geom is not null)
        {
            _basisCurve = _geom.BasisCurve();
            _direction = _geom.Direction();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public ExtrusionSurface? Geometry() => _geom;

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
        NCollection_Array1<Point3D> curvePoints = curveEval.EvaluateGrid(uParams);
        if (curvePoints.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // For each curve point, compute surface points for all V values
        // S(u, v) = C(u) + v * Direction
        Xyz dirXYZ = _direction.XYZ();
        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);

        for (int i = 1; i <= nbU; ++i)
        {
            Point3D curvePt = curvePoints.Value(i);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ExtrusionUtils.CalculateD0(curvePt, v, dirXYZ, out Point3D p);
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

        // Batch evaluate curve D1 using optimized curve evaluator
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD1> curveD1 = curveEval.EvaluateGridD1(uParams);
        if (curveD1.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction (constant)
        Xyz dirXYZ = _direction.XYZ();
        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);

        for (int i = 1; i <= nbU; ++i)
        {
            ref Curve3D.ResD1 curveData = ref curveD1.ChangeValue(i);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ExtrusionUtils.CalculateD1(curveData.Point,
                                                curveData.D1,
                                                v,
                                                dirXYZ,
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

        // Batch evaluate curve D2 using optimized curve evaluator
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD2> curveD2 = curveEval.EvaluateGridD2(uParams);
        if (curveD2.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction
        // D2U = C''(u), D2V = 0, D2UV = 0
        Xyz dirXYZ = _direction.XYZ();
        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);

        for (int i = 1; i <= nbU; ++i)
        {
            ref Curve3D.ResD2 curveData = ref curveD2.ChangeValue(i);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ExtrusionUtils.CalculateD2(curveData.Point,
                                                curveData.D1,
                                                curveData.D2,
                                                v,
                                                dirXYZ,
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

        // Batch evaluate curve D3 using optimized curve evaluator
        CurveGrid curveEval = new(_basisCurve);
        NCollection_Array1<Curve3D.ResD3> curveD3 = curveEval.EvaluateGridD3(uParams);
        if (curveD3.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction
        // D2U = C''(u), D2V = 0, D2UV = 0
        // D3U = C'''(u), D3V = 0, D3UUV = 0, D3UVV = 0
        Xyz dirXYZ = _direction.XYZ();
        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);

        for (int i = 1; i <= nbU; ++i)
        {
            ref Curve3D.ResD3 curveData = ref curveD3.ChangeValue(i);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ExtrusionUtils.CalculateD3(curveData.Point,
                                                curveData.D1,
                                                curveData.D2,
                                                curveData.D3,
                                                v,
                                                dirXYZ,
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

        Xyz dirXYZ = _direction.XYZ();

        // For theNV == 0: need curve derivative, otherwise use CalculateDN directly
        if (nv == 0)
        {
            // Pure U derivative = curve derivative
            CurveGrid curveEval = new(_basisCurve);
            NCollection_Array1<Vector3D> curveDN = curveEval.EvaluateGridDN(uParams, nu);
            for (int i = 1; i <= nbU; ++i)
            {
                Vector3D dn = ExtrusionUtils.CalculateDN(curveDN.Value(i), dirXYZ, nu, nv);
                for (int j = 1; j <= nbV; ++j)
                {
                    result.SetValue(i, j, dn);
                }
            }
        }
        else
        {
            // For theNV >= 1: result doesn't depend on curve, use CalculateDN with zero vector
            Vector3D dn = ExtrusionUtils.CalculateDN(new Vector3D(), dirXYZ, nu, nv);
            for (int i = 1; i <= nbU; ++i)
            {
                for (int j = 1; j <= nbV; ++j)
                {
                    result.SetValue(i, j, dn);
                }
            }
        }

        return result;
    }
}
