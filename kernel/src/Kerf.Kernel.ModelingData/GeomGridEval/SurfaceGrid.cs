// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Surface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Unified grid evaluator for any surface: detects the surface type from an Adaptor3d_Surface or a Geom_Surface
/// and dispatches to the specialised evaluator (plane, cylinder, sphere, cone, torus, Bezier, B-spline, offset,
/// revolution, extrusion; any other surface point by point through <see cref="OtherSurfaceGrid"/>).
/// </summary>
/// <remarks>
/// Trimmed surfaces are evaluated through their basis surface. For a GeomAdaptor_TransformedSurface (the base of
/// BRepAdaptor_Surface) the underlying Geom_Surface is evaluated and the transformation applied to the results.
/// A fallback evaluator keeps the adaptor it was given (OCCT keeps a pointer, which must outlive the evaluator).
/// OCCT's <c>std::variant</c> of evaluators is a <see cref="GeomGridEvalGrid.ISurfaceEvaluator"/> field, null for
/// <c>std::monostate</c> (decision 236).
/// </remarks>
internal sealed class SurfaceGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    private GeomGridEvalGrid.ISurfaceEvaluator? _evaluator;
    private SurfaceKind _surfaceType;
    private Transform3D? _trsf; // Optional transformation for BRepAdaptor surfaces

    /// <summary>Constructor from an adaptor (auto-detects the surface type).</summary>
    public SurfaceGrid(SurfaceAdaptorBase surface)
    {
        _evaluator = null;
        _surfaceType = OtherSurface;
        Initialization(surface);
    }

    /// <summary>Constructor from a geometry (auto-detects the surface type); null gives empty results.</summary>
    public SurfaceGrid(Surface? surface)
    {
        _evaluator = null;
        _surfaceType = OtherSurface;
        Initialization(surface);
    }

    /// <summary>Returns the detected surface type.</summary>
    /// <remarks>Hides <c>object.GetType()</c> to keep OCCT's name (decision 98).</remarks>
    public new SurfaceKind GetType() => _surfaceType;

    /// <summary>True if a (non-identity) transformation is applied to the results.</summary>
    public bool HasTransformation => _trsf.HasValue;

    /// <summary>The transformation applied to the results, if any.</summary>
    public Transform3D? GetTransformation() => _trsf;

    // Extracts basis surface from potentially nested RectangularTrimmedSurface wrappers.
    private static Surface ExtractBasisSurface(Surface surface)
    {
        Surface result = surface;
        while (result is TrimmedSurface trimmed)
        {
            result = trimmed.BasisSurface;
        }
        return result;
    }

    // Creates Geom_Surface from adaptor's elementary surface type; null if the type is not elementary.
    private static Surface? CreateGeomSurfaceFromAdaptor(SurfaceAdaptorBase surface)
    {
        switch (surface.GetType())
        {
            case Plane:
                return new PlaneSurface(surface.Plane());
            case SurfaceKind.Cylinder:
                return new CylindricalSurface(surface.Cylinder());
            case SurfaceKind.Cone:
                return new ConicalSurface(surface.Cone());
            case SurfaceKind.Sphere:
                return new SphericalSurface(surface.Sphere());
            case SurfaceKind.Torus:
                return new ToroidalSurface(surface.Torus());
            default:
                return null;
        }
    }

    // Extracts Geom_Curve from Adaptor3d_Curve if possible; null if not available.
    private static Curve3D? ExtractGeomCurve(CurveAdaptorBase? curve)
    {
        if (curve is null)
        {
            return null;
        }
        if (curve is CurveAdaptor geomAdaptor)
        {
            return geomAdaptor.Curve();
        }
        return null;
    }

    // Creates Geom_SurfaceOfRevolution from adaptor data; null if the curve is not available.
    private static Surface? CreateRevolutionSurface(RevolutionSurfaceAdaptor adaptor)
    {
        Curve3D? curve = ExtractGeomCurve(adaptor.BasisCurve());
        if (curve is null)
        {
            return null;
        }
        return new RevolutionSurface(curve, adaptor.AxeOfRevolution());
    }

    // Creates Geom_SurfaceOfLinearExtrusion from adaptor data; null if the curve is not available.
    private static Surface? CreateExtrusionSurface(SurfaceOfLinearExtrusion adaptor)
    {
        Curve3D? curve = ExtractGeomCurve(adaptor.BasisCurve());
        if (curve is null)
        {
            return null;
        }
        return new ExtrusionSurface(curve, adaptor.Direction());
    }

    // Initialize from adaptor reference (auto-detects surface type). Protected in OCCT; the class is sealed.
    private void Initialization(SurfaceAdaptorBase surface)
    {
        // Reset transformation
        _trsf = null;

        // Check for GeomAdaptor_TransformedSurface (includes BRepAdaptor_Surface)
        // to extract transformation and underlying geometry
        if (surface is TransformedSurface transformed)
        {
            Transform3D trsf = transformed.Trsf();

            // Only store transformation if it's not identity
            if (trsf.Form != Identity)
            {
                _trsf = trsf;
            }

            // Initialize with the underlying Geom_Surface
            Initialization(transformed.GeomSurfaceOriginal());
            return;
        }

        // Check for SurfaceOfRevolution adaptor - may not have stored Geom_Surface
        if (surface is RevolutionSurfaceAdaptor revAdaptor)
        {
            // First try to get elementary surface type (Plane, Cylinder, Cone, Sphere, Torus)
            Surface? geomSurf = CreateGeomSurfaceFromAdaptor(revAdaptor);
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // Try the stored Surface() handle
            geomSurf = revAdaptor.Surface();
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // Try to recreate Geom_SurfaceOfRevolution from axis and basis curve
            geomSurf = CreateRevolutionSurface(revAdaptor);
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // No geometry available - use adaptor directly as fallback
            _surfaceType = surface.GetType();
            _evaluator = new OtherSurfaceGrid(surface);
            return;
        }

        // Check for SurfaceOfExtrusion adaptor - may not have stored Geom_Surface
        if (surface is SurfaceOfLinearExtrusion extAdaptor)
        {
            // First try to get elementary surface type (Plane, Cylinder, Cone, Sphere, Torus)
            Surface? geomSurf = CreateGeomSurfaceFromAdaptor(extAdaptor);
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // Try the stored Surface() handle
            geomSurf = extAdaptor.Surface();
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // Try to recreate Geom_SurfaceOfLinearExtrusion from direction and basis curve
            geomSurf = CreateExtrusionSurface(extAdaptor);
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // No geometry available - use adaptor directly as fallback
            _surfaceType = surface.GetType();
            _evaluator = new OtherSurfaceGrid(surface);
            return;
        }

        // Check for plain GeomAdaptor_Surface (without transformation)
        if (surface is SurfaceAdaptor geomAdaptor)
        {
            Surface? geomSurf = geomAdaptor.Surface();

            // If Surface() is null, try to create from elementary type
            if (geomSurf is null)
            {
                geomSurf = CreateGeomSurfaceFromAdaptor(geomAdaptor);
            }

            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }

            // Surface() is null and not elementary - use adaptor directly
            _surfaceType = surface.GetType();
            _evaluator = new OtherSurfaceGrid(surface);
            return;
        }

        // For non-GeomAdaptor adaptors, try to create Geom_Surface from elementary type first
        {
            Surface? geomSurf = CreateGeomSurfaceFromAdaptor(surface);
            if (geomSurf is not null)
            {
                Initialization(geomSurf);
                return;
            }
        }

        // No way to get Geom_Surface - use adaptor directly as fallback
        _surfaceType = surface.GetType();
        _evaluator = new OtherSurfaceGrid(surface);
    }

    // Initialize from geometry handle (auto-detects surface type). Protected in OCCT; the class is sealed.
    private void Initialization(Surface? surface)
    {
        if (surface is null)
        {
            _evaluator = null;
            _surfaceType = OtherSurface;
            return;
        }

        // Extract basis surface from potentially nested RectangularTrimmedSurface wrappers
        Surface basisSurf = ExtractBasisSurface(surface);

        if (basisSurf is PlaneSurface plane)
        {
            _surfaceType = Plane;
            _evaluator = new PlaneGrid(plane);
        }
        else if (basisSurf is CylindricalSurface cyl)
        {
            _surfaceType = SurfaceKind.Cylinder;
            _evaluator = new CylinderGrid(cyl);
        }
        else if (basisSurf is SphericalSurface sphere)
        {
            _surfaceType = SurfaceKind.Sphere;
            _evaluator = new SphereGrid(sphere);
        }
        else if (basisSurf is ConicalSurface cone)
        {
            _surfaceType = SurfaceKind.Cone;
            _evaluator = new ConeGrid(cone);
        }
        else if (basisSurf is ToroidalSurface torus)
        {
            _surfaceType = SurfaceKind.Torus;
            _evaluator = new TorusGrid(torus);
        }
        else if (basisSurf is global::Kerf.Geometry.BezierSurface bezier)
        {
            _surfaceType = SurfaceKind.BezierSurface;
            _evaluator = new BezierSurfaceGrid(bezier);
        }
        else if (basisSurf is global::Kerf.Geometry.BSplineSurface bSpline)
        {
            _surfaceType = SurfaceKind.BSplineSurface;
            _evaluator = new BSplineSurfaceGrid(bSpline);
        }
        else if (basisSurf is global::Kerf.Geometry.OffsetSurface offset)
        {
            _surfaceType = SurfaceKind.OffsetSurface;
            _evaluator = new OffsetSurfaceGrid(offset);
        }
        else if (basisSurf is RevolutionSurface revolution)
        {
            _surfaceType = SurfaceOfRevolution;
            _evaluator = new SurfaceOfRevolutionGrid(revolution);
        }
        else if (basisSurf is ExtrusionSurface extrusion)
        {
            _surfaceType = SurfaceOfExtrusion;
            _evaluator = new SurfaceOfExtrusionGrid(extrusion);
        }
        else
        {
            // Unknown surface type - use OtherSurface fallback
            _surfaceType = OtherSurface;
            _evaluator = new OtherSurfaceGrid(basisSurf);
        }
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        NCollection_Array2<Point3D> result = _evaluator is null
                                                 ? new NCollection_Array2<Point3D>()
                                                 : _evaluator.EvaluateGrid(uParams, vParams);

        if (_trsf.HasValue)
        {
            ApplyTransformation(result);
        }

        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        NCollection_Array2<Surface.ResD1> result = _evaluator is null
                                                             ? new NCollection_Array2<Surface.ResD1>()
                                                             : _evaluator.EvaluateGridD1(uParams, vParams);

        if (_trsf.HasValue)
        {
            ApplyTransformation(result);
        }

        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        NCollection_Array2<Surface.ResD2> result = _evaluator is null
                                                             ? new NCollection_Array2<Surface.ResD2>()
                                                             : _evaluator.EvaluateGridD2(uParams, vParams);

        if (_trsf.HasValue)
        {
            ApplyTransformation(result);
        }

        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        NCollection_Array2<Surface.ResD3> result = _evaluator is null
                                                             ? new NCollection_Array2<Surface.ResD3>()
                                                             : _evaluator.EvaluateGridD3(uParams, vParams);

        if (_trsf.HasValue)
        {
            ApplyTransformation(result);
        }

        return result;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        NCollection_Array2<Vector3D> result = _evaluator is null
                                                 ? new NCollection_Array2<Vector3D>()
                                                 : _evaluator.EvaluateGridDN(uParams, vParams, nu, nv);

        if (_trsf.HasValue)
        {
            ApplyTransformation(result);
        }

        return result;
    }

    private void ApplyTransformation(NCollection_Array2<Point3D> grid)
    {
        if (!_trsf.HasValue || grid.IsEmpty())
        {
            return;
        }

        Transform3D trsf = _trsf.Value;
        for (int uIdx = grid.LowerRow(); uIdx <= grid.UpperRow(); ++uIdx)
        {
            for (int vIdx = grid.LowerCol(); vIdx <= grid.UpperCol(); ++vIdx)
            {
                grid.ChangeValue(uIdx, vIdx) = grid.ChangeValue(uIdx, vIdx).Transformed(trsf);
            }
        }
    }

    private void ApplyTransformation(NCollection_Array2<Surface.ResD1> grid)
    {
        if (!_trsf.HasValue || grid.IsEmpty())
        {
            return;
        }

        Transform3D trsf = _trsf.Value;
        for (int uIdx = grid.LowerRow(); uIdx <= grid.UpperRow(); ++uIdx)
        {
            for (int vIdx = grid.LowerCol(); vIdx <= grid.UpperCol(); ++vIdx)
            {
                ref Surface.ResD1 val = ref grid.ChangeValue(uIdx, vIdx);
                val.Point = val.Point.Transformed(trsf);
                val.D1U = val.D1U.Transformed(trsf);
                val.D1V = val.D1V.Transformed(trsf);
            }
        }
    }

    private void ApplyTransformation(NCollection_Array2<Surface.ResD2> grid)
    {
        if (!_trsf.HasValue || grid.IsEmpty())
        {
            return;
        }

        Transform3D trsf = _trsf.Value;
        for (int uIdx = grid.LowerRow(); uIdx <= grid.UpperRow(); ++uIdx)
        {
            for (int vIdx = grid.LowerCol(); vIdx <= grid.UpperCol(); ++vIdx)
            {
                ref Surface.ResD2 val = ref grid.ChangeValue(uIdx, vIdx);
                val.Point = val.Point.Transformed(trsf);
                val.D1U = val.D1U.Transformed(trsf);
                val.D1V = val.D1V.Transformed(trsf);
                val.D2U = val.D2U.Transformed(trsf);
                val.D2V = val.D2V.Transformed(trsf);
                val.D2UV = val.D2UV.Transformed(trsf);
            }
        }
    }

    private void ApplyTransformation(NCollection_Array2<Surface.ResD3> grid)
    {
        if (!_trsf.HasValue || grid.IsEmpty())
        {
            return;
        }

        Transform3D trsf = _trsf.Value;
        for (int uIdx = grid.LowerRow(); uIdx <= grid.UpperRow(); ++uIdx)
        {
            for (int vIdx = grid.LowerCol(); vIdx <= grid.UpperCol(); ++vIdx)
            {
                ref Surface.ResD3 val = ref grid.ChangeValue(uIdx, vIdx);
                val.Point = val.Point.Transformed(trsf);
                val.D1U = val.D1U.Transformed(trsf);
                val.D1V = val.D1V.Transformed(trsf);
                val.D2U = val.D2U.Transformed(trsf);
                val.D2V = val.D2V.Transformed(trsf);
                val.D2UV = val.D2UV.Transformed(trsf);
                val.D3U = val.D3U.Transformed(trsf);
                val.D3V = val.D3V.Transformed(trsf);
                val.D3UUV = val.D3UUV.Transformed(trsf);
                val.D3UVV = val.D3UVV.Transformed(trsf);
            }
        }
    }

    private void ApplyTransformation(NCollection_Array2<Vector3D> grid)
    {
        if (!_trsf.HasValue || grid.IsEmpty())
        {
            return;
        }

        Transform3D trsf = _trsf.Value;
        for (int uIdx = grid.LowerRow(); uIdx <= grid.UpperRow(); ++uIdx)
        {
            for (int vIdx = grid.LowerCol(); vIdx <= grid.UpperCol(); ++vIdx)
            {
                grid.ChangeValue(uIdx, vIdx) = grid.ChangeValue(uIdx, vIdx).Transformed(trsf);
            }
        }
    }
}
