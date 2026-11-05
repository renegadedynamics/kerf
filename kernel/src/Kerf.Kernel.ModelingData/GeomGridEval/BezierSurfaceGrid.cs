// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BezierSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for Bezier surface grid points. A Bezier surface is a single span, so the
/// polynomial cache (BSplSLib_Cache) is built once per call; D3 and DN use BSplSLib directly. A tall column of
/// points (many U values, one V value) is evaluated along the V-isoline curve with <see cref="CurveGrid"/>.
/// </summary>
internal sealed class BezierSurfaceGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    /// <summary>
    /// Minimum number of points along varying dimension to use isoline optimization.
    /// For small grids (e.g., 1x4), cache-based surface evaluation is faster than
    /// extracting an isoline curve and setting up a curve evaluator.
    /// </summary>
    private const int THE_ISOLINE_THRESHOLD = 8;

    private readonly BezierSurface? _geom;

    /// <summary>Constructor with the Bezier surface to evaluate (null gives empty results).</summary>
    public BezierSurfaceGrid(BezierSurface? bezier)
    {
        _geom = bezier;
    }

    /// <summary>Returns the geometry.</summary>
    public BezierSurface? Geometry() => _geom;

    /// <summary>
    /// Creates and builds the cache for Bezier surface evaluation. Bezier surfaces are single-span, so the
    /// cache is built once at parameter (0.5, 0.5).
    /// </summary>
    private static SurfaceCache BuildBezierCache(BezierSurface geom)
    {
        NCollection_Array1<double> uKnotSequence = geom.InternalUKnotSequence();
        NCollection_Array1<double> vKnotSequence = geom.InternalVKnotSequence();
        NCollection_Array2<Point3D> poles = geom.InternalPoles();
        NCollection_Array2<double>? weights = geom.InternalWeights();

        SurfaceCache cache = new(geom.UDegree,
                                    false,
                                    uKnotSequence,
                                    geom.VDegree,
                                    false,
                                    vKnotSequence,
                                    weights);
        cache.BuildCache(0.5, 0.5, uKnotSequence, vKnotSequence, poles, weights);
        return cache;
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        // Check for V-isoline case (Nx1) - use 1D curve evaluation
        // For U-isoline (1xN), cache-based surface evaluation is efficient since U span is fixed.
        // For V-isoline (Nx1), extracting the isoline curve avoids repeated cache rebuilds.
        // Only use isoline optimization when varying dimension is large enough.
        bool isVIso = (nbV == 1 && nbU >= THE_ISOLINE_THRESHOLD);

        if (isVIso)
        {
            try
            {
                // Extract V-isoline curve (parameterized by U)
                Curve3D? curve = _geom.VIso(vParams.Value(vParams.Lower()));
                if (curve is not null)
                {
                    // Use unified curve evaluator
                    CurveGrid curveEval = new(curve);
                    NCollection_Array1<Point3D> curveResult = curveEval.EvaluateGrid(uParams);

                    // Reshape 1D curve result to 2D surface result (Nx1 grid)
                    NCollection_Array2<Point3D> resultIso = new(1, nbU, 1, 1);
                    for (int k = 1; k <= nbU; ++k)
                    {
                        resultIso[k, 1] = curveResult[k];
                    }
                    return resultIso;
                }
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                // Isoline extraction failed, fall through to surface evaluation
            }
        }

        // Build cache (Bezier is single span, cache is built once)
        SurfaceCache cache = BuildBezierCache(_geom);

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);

        // Single span - use cache for all points
        for (int i = 0; i < nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i);
            for (int j = 0; j < nbV; ++j)
            {
                cache.D0(u, vParams.Value(vParams.Lower() + j), out Point3D point);
                result.SetValue(i + 1, j + 1, point);
            }
        }

        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        // Build cache (Bezier is single span, cache is built once)
        SurfaceCache cache = BuildBezierCache(_geom);

        int nbU = uParams.Length();
        int nbV = vParams.Length();
        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);

        // Single span - use cache for all points
        for (int i = 0; i < nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i);
            for (int j = 0; j < nbV; ++j)
            {
                cache.D1(u, vParams.Value(vParams.Lower() + j), out Point3D point, out Vector3D d1U,
                          out Vector3D d1V);
                result[i + 1, j + 1] = new Surface.ResD1 { Point = point, D1U = d1U, D1V = d1V };
            }
        }

        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        // Build cache (Bezier is single span, cache is built once)
        SurfaceCache cache = BuildBezierCache(_geom);

        int nbU = uParams.Length();
        int nbV = vParams.Length();
        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);

        // Single span - use cache for all points
        for (int i = 0; i < nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i);
            for (int j = 0; j < nbV; ++j)
            {
                cache.D2(u, vParams.Value(vParams.Lower() + j), out Point3D point, out Vector3D d1U,
                          out Vector3D d1V, out Vector3D d2U, out Vector3D d2V, out Vector3D d2UV);
                result[i + 1, j + 1] = new Surface.ResD2
                {
                    Point = point,
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

    /// <summary>Evaluates all grid points with partial derivatives up to the third order (BSplSLib::D3).</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();
        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);

        // Get degrees, flat knots, poles, and weights from geometry
        int uDegree = _geom.UDegree;
        int vDegree = _geom.VDegree;
        NCollection_Array1<double> uKnotSequence = _geom.InternalUKnotSequence();
        NCollection_Array1<double> vKnotSequence = _geom.InternalVKnotSequence();
        NCollection_Array2<Point3D> poles = _geom.InternalPoles();
        NCollection_Array2<double>? weights = _geom.InternalWeights();
        bool isRational = (weights != null);

        // D3 evaluation using BSplSLib::D3 directly
        // Bezier surface is single span (span index = 0), non-periodic
        for (int i = 0; i < nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i);
            for (int j = 0; j < nbV; ++j)
            {
                SurfaceBSplSLib.D3(u,
                            vParams.Value(vParams.Lower() + j),
                            0, // U span index (single span for Bezier)
                            0, // V span index (single span for Bezier)
                            poles,
                            weights,
                            uKnotSequence,
                            vKnotSequence,
                            null, // U multiplicities (null for flat knots)
                            null, // V multiplicities (null for flat knots)
                            uDegree,
                            vDegree,
                            isRational,
                            isRational,
                            false, // not U periodic
                            false, // not V periodic
                            out Point3D point,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV,
                            out Vector3D d3U,
                            out Vector3D d3V,
                            out Vector3D d3UUV,
                            out Vector3D d3UVV);
                result[i + 1, j + 1] = new Surface.ResD3
                {
                    Point = point,
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

    /// <summary>
    /// Evaluates the partial derivative d^(NU+NV)S/(dU^NU dV^NV) at all grid points (BSplSLib::DN); zero above the
    /// degrees.
    /// </summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty() || nu < 0 || nv < 0
            || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();
        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);

        // For Bezier surfaces, derivatives become zero when order exceeds degree in that direction
        // OCCT: not for a rational surface, whose derivatives of every order are non-zero in general; the grid
        // returns zero where Geom_BezierSurface::DN (BSplSLib::DN) does not (occt-issues.md).
        int uDegree = _geom.UDegree;
        int vDegree = _geom.VDegree;
        if (nu > uDegree || nv > vDegree)
        {
            // All derivatives are zero
            Vector3D zeroVec = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= nbU; ++i)
            {
                for (int j = 1; j <= nbV; ++j)
                {
                    result.SetValue(i, j, zeroVec);
                }
            }
            return result;
        }

        // Get poles, weights, and flat knots from geometry
        NCollection_Array2<Point3D> poles = _geom.InternalPoles();
        NCollection_Array2<double>? weights = _geom.InternalWeights();
        bool isRational = (weights != null);
        NCollection_Array1<double> uKnotSequence = _geom.InternalUKnotSequence();
        NCollection_Array1<double> vKnotSequence = _geom.InternalVKnotSequence();

        // Bezier has a single span (index 0 with flat knots), non-periodic
        for (int i = 0; i < nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i);
            for (int j = 0; j < nbV; ++j)
            {
                SurfaceBSplSLib.DN(u,
                            vParams.Value(vParams.Lower() + j),
                            nu,
                            nv,
                            0, // U span index (single span for Bezier with flat knots)
                            0, // V span index (single span for Bezier with flat knots)
                            poles,
                            weights,
                            uKnotSequence,
                            vKnotSequence,
                            null, // no U multiplicities with flat knots
                            null, // no V multiplicities with flat knots
                            uDegree,
                            vDegree,
                            isRational,
                            isRational,
                            false, // not U-periodic
                            false, // not V-periodic
                            out Vector3D dn);
                result.SetValue(i + 1, j + 1, dn);
            }
        }

        return result;
    }
}
