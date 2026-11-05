// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Batch evaluator for offset surface grid points: P(u, v) = Basis(u, v) + Offset * N(u, v). The basis surface is
/// evaluated on the whole grid first (through <see cref="SurfaceGrid"/>), then the offset is applied
/// point by point with Geom_OffsetSurfaceUtils, which handles singular points through the osculating surfaces.
/// When the offset surface has an equivalent elementary surface (Geom_OffsetSurface.Surface()), that surface is
/// evaluated instead.
/// </summary>
internal sealed class OffsetSurfaceGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    private readonly OffsetSurface? _geom;
    private readonly Surface? _basis;
    private readonly double _offset;

    /// <summary>Constructor with the offset surface to evaluate (null gives empty results).</summary>
    public OffsetSurfaceGrid(OffsetSurface? offset)
    {
        _geom = offset;
        _offset = 0.0;
        if (_geom is not null)
        {
            _offset = _geom.Offset;
            _basis = _geom.BasisSurface;
        }
    }

    /// <summary>Returns the geometry.</summary>
    public OffsetSurface? Geometry() => _geom;

    /// <summary>
    /// The equivalent non-offset surface of the fast path, or null: OCCT's
    /// <c>!anEquivSurf.IsNull() &amp;&amp; anEquivSurf.get() != myGeom.get()</c>.
    /// </summary>
    private Surface? EquivalentSurface()
    {
        if (_geom is not null)
        {
            Surface? equivSurf = _geom.Surface();
            if (equivSurf is not null && !ReferenceEquals(equivSurf, _geom))
            {
                return equivSurf;
            }
        }
        return null;
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        if (_basis is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        // (e.g., offset of plane/sphere/cylinder/cone/torus with same parameters)
        Surface? equivSurf = EquivalentSurface();
        if (equivSurf is not null)
        {
            // Use the equivalent surface directly
            SurfaceGrid equivEval = new(equivSurf);
            return equivEval.EvaluateGrid(uParams, vParams);
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);

        // Batch evaluate basis surface D1 (offset D0 requires basis D1)
        SurfaceGrid basisEval = new(_basis);
        NCollection_Array2<Surface.ResD1> basisD1 = basisEval.EvaluateGridD1(uParams, vParams);
        if (basisD1.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // Use GeomAdaptor for evaluation - works with template functions
        OffsetSurfaceUtils.AdaptorBasis basisAdaptor = new(new SurfaceAdaptor(_basis));
        // Pass Geom_OffsetSurface which has UOsculatingSurface/VOsculatingSurface methods
        // for proper handling of singular points via osculating surface queries
        OsculatingSurface? oscQuery = _geom!.OsculatingSurface();

        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ref Surface.ResD1 basis = ref basisD1.ChangeValue(i, j);
                Point3D p = basis.Point;
                OffsetSurfaceUtils.EvaluateD0(u, v, basisAdaptor, _offset, oscQuery, ref p, basis.D1U,
                                                   basis.D1V);
                result.SetValue(i, j, p);
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basis is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Surface? equivSurf = EquivalentSurface();
        if (equivSurf is not null)
        {
            SurfaceGrid equivEval = new(equivSurf);
            return equivEval.EvaluateGridD1(uParams, vParams);
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);

        // Batch evaluate basis surface D2 (offset D1 requires basis D2)
        SurfaceGrid basisEval = new(_basis);
        NCollection_Array2<Surface.ResD2> basisD2 = basisEval.EvaluateGridD2(uParams, vParams);
        if (basisD2.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        OffsetSurfaceUtils.AdaptorBasis basisAdaptor = new(new SurfaceAdaptor(_basis));
        OsculatingSurface? oscQuery = _geom!.OsculatingSurface();

        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ref Surface.ResD2 basis = ref basisD2.ChangeValue(i, j);
                Point3D p = basis.Point;
                Vector3D d1U = basis.D1U;
                Vector3D d1V = basis.D1V;
                OffsetSurfaceUtils.EvaluateD1(u, v, basisAdaptor, _offset, oscQuery, ref p, ref d1U,
                                                   ref d1V, basis.D2U, basis.D2V, basis.D2UV);
                result[i, j] = new Surface.ResD1 { Point = p, D1U = d1U, D1V = d1V };
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basis is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Surface? equivSurf = EquivalentSurface();
        if (equivSurf is not null)
        {
            SurfaceGrid equivEval = new(equivSurf);
            return equivEval.EvaluateGridD2(uParams, vParams);
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);

        // Batch evaluate basis surface D3 (offset D2 requires basis D3)
        SurfaceGrid basisEval = new(_basis);
        NCollection_Array2<Surface.ResD3> basisD3 = basisEval.EvaluateGridD3(uParams, vParams);
        if (basisD3.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        OffsetSurfaceUtils.AdaptorBasis basisAdaptor = new(new SurfaceAdaptor(_basis));
        OsculatingSurface? oscQuery = _geom!.OsculatingSurface();

        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ref Surface.ResD3 basis = ref basisD3.ChangeValue(i, j);
                Point3D p = basis.Point;
                Vector3D d1U = basis.D1U;
                Vector3D d1V = basis.D1V;
                Vector3D d2U = basis.D2U;
                Vector3D d2V = basis.D2V;
                Vector3D d2UV = basis.D2UV;
                OffsetSurfaceUtils.EvaluateD2(u, v, basisAdaptor, _offset, oscQuery, ref p, ref d1U,
                                                   ref d1V, ref d2U, ref d2V, ref d2UV, basis.D3U, basis.D3V,
                                                   basis.D3UUV, basis.D3UVV);
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

    /// <summary>Evaluates all grid points with partial derivatives up to the third order (point by point).</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_basis is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Surface? equivSurf = EquivalentSurface();
        if (equivSurf is not null)
        {
            SurfaceGrid equivEval = new(equivSurf);
            return equivEval.EvaluateGridD3(uParams, vParams);
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);

        OffsetSurfaceUtils.AdaptorBasis basisAdaptor = new(new SurfaceAdaptor(_basis));
        OsculatingSurface? oscQuery = _geom!.OsculatingSurface();

        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ref Surface.ResD3 res = ref result.ChangeValue(i, j);
                OffsetSurfaceUtils.EvaluateD3(u, v, basisAdaptor, _offset, oscQuery, out res.Point,
                                                   out res.D1U, out res.D1V, out res.D2U, out res.D2V,
                                                   out res.D2UV, out res.D3U, out res.D3V, out res.D3UUV,
                                                   out res.D3UVV);
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
        if (_basis is null || uParams.IsEmpty() || vParams.IsEmpty() || nu < 0 || nv < 0
            || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Surface? equivSurf = EquivalentSurface();
        if (equivSurf is not null)
        {
            SurfaceGrid equivEval = new(equivSurf);
            return equivEval.EvaluateGridDN(uParams, vParams, nu, nv);
        }

        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);

        // Batch evaluate basis surface D1 (offset DN requires basis D1)
        SurfaceGrid basisEval = new(_basis);
        NCollection_Array2<Surface.ResD1> basisD1 = basisEval.EvaluateGridD1(uParams, vParams);
        if (basisD1.IsEmpty())
        {
            return new NCollection_Array2<Vector3D>();
        }

        OffsetSurfaceUtils.AdaptorBasis basisAdaptor = new(new SurfaceAdaptor(_basis));
        OsculatingSurface? oscQuery = _geom!.OsculatingSurface();

        for (int i = 1; i <= nbU; ++i)
        {
            double u = uParams.Value(uParams.Lower() + i - 1);
            for (int j = 1; j <= nbV; ++j)
            {
                double v = vParams.Value(vParams.Lower() + j - 1);
                ref Surface.ResD1 basis = ref basisD1.ChangeValue(i, j);
                OffsetSurfaceUtils.EvaluateDN(u, v, nu, nv, basisAdaptor, _offset, oscQuery,
                                                   out Vector3D dn, basis.D1U, basis.D1V);
                result.SetValue(i, j, dn);
            }
        }
        return result;
    }
}
