// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Surface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// Unified grid evaluator for any surface: detects the surface type from an Adaptor3d_Surface or a Geom_Surface
/// and dispatches to the specialised evaluator (plane, cylinder, sphere, cone, torus, Bezier, B-spline, offset,
/// revolution, extrusion; any other surface point by point through <see cref="GeomGridEval_OtherSurface"/>).
/// </summary>
/// <remarks>
/// Trimmed surfaces are evaluated through their basis surface. For a GeomAdaptor_TransformedSurface (the base of
/// BRepAdaptor_Surface) the underlying Geom_Surface is evaluated and the transformation applied to the results.
/// A fallback evaluator keeps the adaptor it was given (OCCT keeps a pointer, which must outlive the evaluator).
/// OCCT's <c>std::variant</c> of evaluators is a <see cref="GeomGridEval.ISurfaceEvaluator"/> field, null for
/// <c>std::monostate</c> (decision 236).
/// </remarks>
public sealed class GeomGridEval_Surface : GeomGridEval.ISurfaceEvaluator
{
    private GeomGridEval.ISurfaceEvaluator? myEvaluator;
    private GeomAbs_SurfaceType mySurfaceType;
    private gp_Trsf? myTrsf; // Optional transformation for BRepAdaptor surfaces

    /// <summary>Constructor from an adaptor (auto-detects the surface type).</summary>
    public GeomGridEval_Surface(Adaptor3d_Surface theSurface)
    {
        myEvaluator = null;
        mySurfaceType = GeomAbs_OtherSurface;
        initialization(theSurface);
    }

    /// <summary>Constructor from a geometry (auto-detects the surface type); null gives empty results.</summary>
    public GeomGridEval_Surface(Geom_Surface? theSurface)
    {
        myEvaluator = null;
        mySurfaceType = GeomAbs_OtherSurface;
        initialization(theSurface);
    }

    /// <summary>Returns the detected surface type.</summary>
    /// <remarks>Hides <c>object.GetType()</c> to keep OCCT's name (decision 98).</remarks>
    public new GeomAbs_SurfaceType GetType() => mySurfaceType;

    /// <summary>True if a (non-identity) transformation is applied to the results.</summary>
    public bool HasTransformation() => myTrsf.HasValue;

    /// <summary>The transformation applied to the results, if any.</summary>
    public gp_Trsf? GetTransformation() => myTrsf;

    // Extracts basis surface from potentially nested RectangularTrimmedSurface wrappers.
    private static Geom_Surface ExtractBasisSurface(Geom_Surface theSurface)
    {
        Geom_Surface aResult = theSurface;
        while (aResult is Geom_RectangularTrimmedSurface aTrimmed)
        {
            aResult = aTrimmed.BasisSurface();
        }
        return aResult;
    }

    // Creates Geom_Surface from adaptor's elementary surface type; null if the type is not elementary.
    private static Geom_Surface? CreateGeomSurfaceFromAdaptor(Adaptor3d_Surface theSurface)
    {
        switch (theSurface.GetType())
        {
            case GeomAbs_Plane:
                return new Geom_Plane(theSurface.Plane());
            case GeomAbs_Cylinder:
                return new Geom_CylindricalSurface(theSurface.Cylinder());
            case GeomAbs_Cone:
                return new Geom_ConicalSurface(theSurface.Cone());
            case GeomAbs_Sphere:
                return new Geom_SphericalSurface(theSurface.Sphere());
            case GeomAbs_Torus:
                return new Geom_ToroidalSurface(theSurface.Torus());
            default:
                return null;
        }
    }

    // Extracts Geom_Curve from Adaptor3d_Curve if possible; null if not available.
    private static Geom_Curve? ExtractGeomCurve(Adaptor3d_Curve? theCurve)
    {
        if (theCurve is null)
        {
            return null;
        }
        if (theCurve is GeomAdaptor_Curve aGeomAdaptor)
        {
            return aGeomAdaptor.Curve();
        }
        return null;
    }

    // Creates Geom_SurfaceOfRevolution from adaptor data; null if the curve is not available.
    private static Geom_Surface? CreateRevolutionSurface(GeomAdaptor_SurfaceOfRevolution theAdaptor)
    {
        Geom_Curve? aCurve = ExtractGeomCurve(theAdaptor.BasisCurve());
        if (aCurve is null)
        {
            return null;
        }
        return new Geom_SurfaceOfRevolution(aCurve, theAdaptor.AxeOfRevolution());
    }

    // Creates Geom_SurfaceOfLinearExtrusion from adaptor data; null if the curve is not available.
    private static Geom_Surface? CreateExtrusionSurface(GeomAdaptor_SurfaceOfLinearExtrusion theAdaptor)
    {
        Geom_Curve? aCurve = ExtractGeomCurve(theAdaptor.BasisCurve());
        if (aCurve is null)
        {
            return null;
        }
        return new Geom_SurfaceOfLinearExtrusion(aCurve, theAdaptor.Direction());
    }

    // Initialize from adaptor reference (auto-detects surface type). Protected in OCCT; the class is sealed.
    private void initialization(Adaptor3d_Surface theSurface)
    {
        // Reset transformation
        myTrsf = null;

        // Check for GeomAdaptor_TransformedSurface (includes BRepAdaptor_Surface)
        // to extract transformation and underlying geometry
        if (theSurface is GeomAdaptor_TransformedSurface aTransformed)
        {
            gp_Trsf aTrsf = aTransformed.Trsf();

            // Only store transformation if it's not identity
            if (aTrsf.Form() != gp_Identity)
            {
                myTrsf = aTrsf;
            }

            // Initialize with the underlying Geom_Surface
            initialization(aTransformed.GeomSurfaceOriginal());
            return;
        }

        // Check for SurfaceOfRevolution adaptor - may not have stored Geom_Surface
        if (theSurface is GeomAdaptor_SurfaceOfRevolution aRevAdaptor)
        {
            // First try to get elementary surface type (Plane, Cylinder, Cone, Sphere, Torus)
            Geom_Surface? aGeomSurf = CreateGeomSurfaceFromAdaptor(aRevAdaptor);
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // Try the stored Surface() handle
            aGeomSurf = aRevAdaptor.Surface();
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // Try to recreate Geom_SurfaceOfRevolution from axis and basis curve
            aGeomSurf = CreateRevolutionSurface(aRevAdaptor);
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // No geometry available - use adaptor directly as fallback
            mySurfaceType = theSurface.GetType();
            myEvaluator = new GeomGridEval_OtherSurface(theSurface);
            return;
        }

        // Check for SurfaceOfExtrusion adaptor - may not have stored Geom_Surface
        if (theSurface is GeomAdaptor_SurfaceOfLinearExtrusion aExtAdaptor)
        {
            // First try to get elementary surface type (Plane, Cylinder, Cone, Sphere, Torus)
            Geom_Surface? aGeomSurf = CreateGeomSurfaceFromAdaptor(aExtAdaptor);
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // Try the stored Surface() handle
            aGeomSurf = aExtAdaptor.Surface();
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // Try to recreate Geom_SurfaceOfLinearExtrusion from direction and basis curve
            aGeomSurf = CreateExtrusionSurface(aExtAdaptor);
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // No geometry available - use adaptor directly as fallback
            mySurfaceType = theSurface.GetType();
            myEvaluator = new GeomGridEval_OtherSurface(theSurface);
            return;
        }

        // Check for plain GeomAdaptor_Surface (without transformation)
        if (theSurface is GeomAdaptor_Surface aGeomAdaptor)
        {
            Geom_Surface? aGeomSurf = aGeomAdaptor.Surface();

            // If Surface() is null, try to create from elementary type
            if (aGeomSurf is null)
            {
                aGeomSurf = CreateGeomSurfaceFromAdaptor(aGeomAdaptor);
            }

            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }

            // Surface() is null and not elementary - use adaptor directly
            mySurfaceType = theSurface.GetType();
            myEvaluator = new GeomGridEval_OtherSurface(theSurface);
            return;
        }

        // For non-GeomAdaptor adaptors, try to create Geom_Surface from elementary type first
        {
            Geom_Surface? aGeomSurf = CreateGeomSurfaceFromAdaptor(theSurface);
            if (aGeomSurf is not null)
            {
                initialization(aGeomSurf);
                return;
            }
        }

        // No way to get Geom_Surface - use adaptor directly as fallback
        mySurfaceType = theSurface.GetType();
        myEvaluator = new GeomGridEval_OtherSurface(theSurface);
    }

    // Initialize from geometry handle (auto-detects surface type). Protected in OCCT; the class is sealed.
    private void initialization(Geom_Surface? theSurface)
    {
        if (theSurface is null)
        {
            myEvaluator = null;
            mySurfaceType = GeomAbs_OtherSurface;
            return;
        }

        // Extract basis surface from potentially nested RectangularTrimmedSurface wrappers
        Geom_Surface aBasisSurf = ExtractBasisSurface(theSurface);

        if (aBasisSurf is Geom_Plane aPlane)
        {
            mySurfaceType = GeomAbs_Plane;
            myEvaluator = new GeomGridEval_Plane(aPlane);
        }
        else if (aBasisSurf is Geom_CylindricalSurface aCyl)
        {
            mySurfaceType = GeomAbs_Cylinder;
            myEvaluator = new GeomGridEval_Cylinder(aCyl);
        }
        else if (aBasisSurf is Geom_SphericalSurface aSphere)
        {
            mySurfaceType = GeomAbs_Sphere;
            myEvaluator = new GeomGridEval_Sphere(aSphere);
        }
        else if (aBasisSurf is Geom_ConicalSurface aCone)
        {
            mySurfaceType = GeomAbs_Cone;
            myEvaluator = new GeomGridEval_Cone(aCone);
        }
        else if (aBasisSurf is Geom_ToroidalSurface aTorus)
        {
            mySurfaceType = GeomAbs_Torus;
            myEvaluator = new GeomGridEval_Torus(aTorus);
        }
        else if (aBasisSurf is Geom_BezierSurface aBezier)
        {
            mySurfaceType = GeomAbs_BezierSurface;
            myEvaluator = new GeomGridEval_BezierSurface(aBezier);
        }
        else if (aBasisSurf is Geom_BSplineSurface aBSpline)
        {
            mySurfaceType = GeomAbs_BSplineSurface;
            myEvaluator = new GeomGridEval_BSplineSurface(aBSpline);
        }
        else if (aBasisSurf is Geom_OffsetSurface anOffset)
        {
            mySurfaceType = GeomAbs_OffsetSurface;
            myEvaluator = new GeomGridEval_OffsetSurface(anOffset);
        }
        else if (aBasisSurf is Geom_SurfaceOfRevolution aRevolution)
        {
            mySurfaceType = GeomAbs_SurfaceOfRevolution;
            myEvaluator = new GeomGridEval_SurfaceOfRevolution(aRevolution);
        }
        else if (aBasisSurf is Geom_SurfaceOfLinearExtrusion anExtrusion)
        {
            mySurfaceType = GeomAbs_SurfaceOfExtrusion;
            myEvaluator = new GeomGridEval_SurfaceOfExtrusion(anExtrusion);
        }
        else
        {
            // Unknown surface type - use OtherSurface fallback
            mySurfaceType = GeomAbs_OtherSurface;
            myEvaluator = new GeomGridEval_OtherSurface(aBasisSurf);
        }
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty.</summary>
    public NCollection_Array2<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        NCollection_Array2<gp_Pnt> aResult = myEvaluator is null
                                                 ? new NCollection_Array2<gp_Pnt>()
                                                 : myEvaluator.EvaluateGrid(theUParams, theVParams);

        if (myTrsf.HasValue)
        {
            applyTransformation(aResult);
        }

        return aResult;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        NCollection_Array2<Geom_Surface.ResD1> aResult = myEvaluator is null
                                                             ? new NCollection_Array2<Geom_Surface.ResD1>()
                                                             : myEvaluator.EvaluateGridD1(theUParams, theVParams);

        if (myTrsf.HasValue)
        {
            applyTransformation(aResult);
        }

        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        NCollection_Array2<Geom_Surface.ResD2> aResult = myEvaluator is null
                                                             ? new NCollection_Array2<Geom_Surface.ResD2>()
                                                             : myEvaluator.EvaluateGridD2(theUParams, theVParams);

        if (myTrsf.HasValue)
        {
            applyTransformation(aResult);
        }

        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        NCollection_Array2<Geom_Surface.ResD3> aResult = myEvaluator is null
                                                             ? new NCollection_Array2<Geom_Surface.ResD3>()
                                                             : myEvaluator.EvaluateGridD3(theUParams, theVParams);

        if (myTrsf.HasValue)
        {
            applyTransformation(aResult);
        }

        return aResult;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid.</summary>
    public NCollection_Array2<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        NCollection_Array2<gp_Vec> aResult = myEvaluator is null
                                                 ? new NCollection_Array2<gp_Vec>()
                                                 : myEvaluator.EvaluateGridDN(theUParams, theVParams, theNU, theNV);

        if (myTrsf.HasValue)
        {
            applyTransformation(aResult);
        }

        return aResult;
    }

    private void applyTransformation(NCollection_Array2<gp_Pnt> theGrid)
    {
        if (!myTrsf.HasValue || theGrid.IsEmpty())
        {
            return;
        }

        gp_Trsf aTrsf = myTrsf.Value;
        for (int aUIdx = theGrid.LowerRow(); aUIdx <= theGrid.UpperRow(); ++aUIdx)
        {
            for (int aVIdx = theGrid.LowerCol(); aVIdx <= theGrid.UpperCol(); ++aVIdx)
            {
                theGrid.ChangeValue(aUIdx, aVIdx).Transform(aTrsf);
            }
        }
    }

    private void applyTransformation(NCollection_Array2<Geom_Surface.ResD1> theGrid)
    {
        if (!myTrsf.HasValue || theGrid.IsEmpty())
        {
            return;
        }

        gp_Trsf aTrsf = myTrsf.Value;
        for (int aUIdx = theGrid.LowerRow(); aUIdx <= theGrid.UpperRow(); ++aUIdx)
        {
            for (int aVIdx = theGrid.LowerCol(); aVIdx <= theGrid.UpperCol(); ++aVIdx)
            {
                ref Geom_Surface.ResD1 aVal = ref theGrid.ChangeValue(aUIdx, aVIdx);
                aVal.Point.Transform(aTrsf);
                aVal.D1U.Transform(aTrsf);
                aVal.D1V.Transform(aTrsf);
            }
        }
    }

    private void applyTransformation(NCollection_Array2<Geom_Surface.ResD2> theGrid)
    {
        if (!myTrsf.HasValue || theGrid.IsEmpty())
        {
            return;
        }

        gp_Trsf aTrsf = myTrsf.Value;
        for (int aUIdx = theGrid.LowerRow(); aUIdx <= theGrid.UpperRow(); ++aUIdx)
        {
            for (int aVIdx = theGrid.LowerCol(); aVIdx <= theGrid.UpperCol(); ++aVIdx)
            {
                ref Geom_Surface.ResD2 aVal = ref theGrid.ChangeValue(aUIdx, aVIdx);
                aVal.Point.Transform(aTrsf);
                aVal.D1U.Transform(aTrsf);
                aVal.D1V.Transform(aTrsf);
                aVal.D2U.Transform(aTrsf);
                aVal.D2V.Transform(aTrsf);
                aVal.D2UV.Transform(aTrsf);
            }
        }
    }

    private void applyTransformation(NCollection_Array2<Geom_Surface.ResD3> theGrid)
    {
        if (!myTrsf.HasValue || theGrid.IsEmpty())
        {
            return;
        }

        gp_Trsf aTrsf = myTrsf.Value;
        for (int aUIdx = theGrid.LowerRow(); aUIdx <= theGrid.UpperRow(); ++aUIdx)
        {
            for (int aVIdx = theGrid.LowerCol(); aVIdx <= theGrid.UpperCol(); ++aVIdx)
            {
                ref Geom_Surface.ResD3 aVal = ref theGrid.ChangeValue(aUIdx, aVIdx);
                aVal.Point.Transform(aTrsf);
                aVal.D1U.Transform(aTrsf);
                aVal.D1V.Transform(aTrsf);
                aVal.D2U.Transform(aTrsf);
                aVal.D2V.Transform(aTrsf);
                aVal.D2UV.Transform(aTrsf);
                aVal.D3U.Transform(aTrsf);
                aVal.D3V.Transform(aTrsf);
                aVal.D3UUV.Transform(aTrsf);
                aVal.D3UVV.Transform(aTrsf);
            }
        }
    }

    private void applyTransformation(NCollection_Array2<gp_Vec> theGrid)
    {
        if (!myTrsf.HasValue || theGrid.IsEmpty())
        {
            return;
        }

        gp_Trsf aTrsf = myTrsf.Value;
        for (int aUIdx = theGrid.LowerRow(); aUIdx <= theGrid.UpperRow(); ++aUIdx)
        {
            for (int aVIdx = theGrid.LowerCol(); aVIdx <= theGrid.UpperCol(); ++aVIdx)
            {
                theGrid.ChangeValue(aUIdx, aVIdx).Transform(aTrsf);
            }
        }
    }
}
