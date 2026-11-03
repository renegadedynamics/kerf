// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Batch evaluator for offset surface grid points: P(u, v) = Basis(u, v) + Offset * N(u, v). The basis surface is
/// evaluated on the whole grid first (through <see cref="GeomGridEval_Surface"/>), then the offset is applied
/// point by point with Geom_OffsetSurfaceUtils, which handles singular points through the osculating surfaces.
/// When the offset surface has an equivalent elementary surface (Geom_OffsetSurface.Surface()), that surface is
/// evaluated instead.
/// </summary>
public sealed class GeomGridEval_OffsetSurface : GeomGridEval.ISurfaceEvaluator
{
    private readonly Geom_OffsetSurface? myGeom;
    private readonly Geom_Surface? myBasis;
    private readonly double myOffset;

    /// <summary>Constructor with the offset surface to evaluate (null gives empty results).</summary>
    public GeomGridEval_OffsetSurface(Geom_OffsetSurface? theOffset)
    {
        myGeom = theOffset;
        myOffset = 0.0;
        if (myGeom is not null)
        {
            myOffset = myGeom.Offset();
            myBasis = myGeom.BasisSurface();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_OffsetSurface? Geometry() => myGeom;

    /// <summary>
    /// The equivalent non-offset surface of the fast path, or null: OCCT's
    /// <c>!anEquivSurf.IsNull() &amp;&amp; anEquivSurf.get() != myGeom.get()</c>.
    /// </summary>
    private Geom_Surface? equivalentSurface()
    {
        if (myGeom is not null)
        {
            Geom_Surface? anEquivSurf = myGeom.Surface();
            if (anEquivSurf is not null && !ReferenceEquals(anEquivSurf, myGeom))
            {
                return anEquivSurf;
            }
        }
        return null;
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (myBasis is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        // (e.g., offset of plane/sphere/cylinder/cone/torus with same parameters)
        Geom_Surface? anEquivSurf = equivalentSurface();
        if (anEquivSurf is not null)
        {
            // Use the equivalent surface directly
            GeomGridEval_Surface anEquivEval = new(anEquivSurf);
            return anEquivEval.EvaluateGrid(theUParams, theVParams);
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Point3D> aResult = new(1, aNbU, 1, aNbV);

        // Batch evaluate basis surface D1 (offset D0 requires basis D1)
        GeomGridEval_Surface aBasisEval = new(myBasis);
        NCollection_Array2<Geom_Surface.ResD1> aBasisD1 = aBasisEval.EvaluateGridD1(theUParams, theVParams);
        if (aBasisD1.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // Use GeomAdaptor for evaluation - works with template functions
        Geom_OffsetSurfaceUtils.AdaptorBasis aBasisAdaptor = new(new GeomAdaptor_Surface(myBasis));
        // Pass Geom_OffsetSurface which has UOsculatingSurface/VOsculatingSurface methods
        // for proper handling of singular points via osculating surface queries
        Geom_OsculatingSurface? anOscQuery = myGeom!.OsculatingSurface();

        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                ref Geom_Surface.ResD1 aBasis = ref aBasisD1.ChangeValue(i, j);
                Point3D aP = aBasis.Point;
                Geom_OffsetSurfaceUtils.EvaluateD0(aU, aV, aBasisAdaptor, myOffset, anOscQuery, ref aP, aBasis.D1U,
                                                   aBasis.D1V);
                aResult.SetValue(i, j, aP);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasis is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Geom_Surface? anEquivSurf = equivalentSurface();
        if (anEquivSurf is not null)
        {
            GeomGridEval_Surface anEquivEval = new(anEquivSurf);
            return anEquivEval.EvaluateGridD1(theUParams, theVParams);
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);

        // Batch evaluate basis surface D2 (offset D1 requires basis D2)
        GeomGridEval_Surface aBasisEval = new(myBasis);
        NCollection_Array2<Geom_Surface.ResD2> aBasisD2 = aBasisEval.EvaluateGridD2(theUParams, theVParams);
        if (aBasisD2.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        Geom_OffsetSurfaceUtils.AdaptorBasis aBasisAdaptor = new(new GeomAdaptor_Surface(myBasis));
        Geom_OsculatingSurface? anOscQuery = myGeom!.OsculatingSurface();

        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                ref Geom_Surface.ResD2 aBasis = ref aBasisD2.ChangeValue(i, j);
                Point3D aP = aBasis.Point;
                Vector3D aD1U = aBasis.D1U;
                Vector3D aD1V = aBasis.D1V;
                Geom_OffsetSurfaceUtils.EvaluateD1(aU, aV, aBasisAdaptor, myOffset, anOscQuery, ref aP, ref aD1U,
                                                   ref aD1V, aBasis.D2U, aBasis.D2V, aBasis.D2UV);
                aResult[i, j] = new Geom_Surface.ResD1 { Point = aP, D1U = aD1U, D1V = aD1V };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasis is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Geom_Surface? anEquivSurf = equivalentSurface();
        if (anEquivSurf is not null)
        {
            GeomGridEval_Surface anEquivEval = new(anEquivSurf);
            return anEquivEval.EvaluateGridD2(theUParams, theVParams);
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);

        // Batch evaluate basis surface D3 (offset D2 requires basis D3)
        GeomGridEval_Surface aBasisEval = new(myBasis);
        NCollection_Array2<Geom_Surface.ResD3> aBasisD3 = aBasisEval.EvaluateGridD3(theUParams, theVParams);
        if (aBasisD3.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        Geom_OffsetSurfaceUtils.AdaptorBasis aBasisAdaptor = new(new GeomAdaptor_Surface(myBasis));
        Geom_OsculatingSurface? anOscQuery = myGeom!.OsculatingSurface();

        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                ref Geom_Surface.ResD3 aBasis = ref aBasisD3.ChangeValue(i, j);
                Point3D aP = aBasis.Point;
                Vector3D aD1U = aBasis.D1U;
                Vector3D aD1V = aBasis.D1V;
                Vector3D aD2U = aBasis.D2U;
                Vector3D aD2V = aBasis.D2V;
                Vector3D aD2UV = aBasis.D2UV;
                Geom_OffsetSurfaceUtils.EvaluateD2(aU, aV, aBasisAdaptor, myOffset, anOscQuery, ref aP, ref aD1U,
                                                   ref aD1V, ref aD2U, ref aD2V, ref aD2UV, aBasis.D3U, aBasis.D3V,
                                                   aBasis.D3UUV, aBasis.D3UVV);
                aResult[i, j] = new Geom_Surface.ResD2
                {
                    Point = aP,
                    D1U = aD1U,
                    D1V = aD1V,
                    D2U = aD2U,
                    D2V = aD2V,
                    D2UV = aD2UV,
                };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order (point by point).</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasis is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Geom_Surface? anEquivSurf = equivalentSurface();
        if (anEquivSurf is not null)
        {
            GeomGridEval_Surface anEquivEval = new(anEquivSurf);
            return anEquivEval.EvaluateGridD3(theUParams, theVParams);
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);

        Geom_OffsetSurfaceUtils.AdaptorBasis aBasisAdaptor = new(new GeomAdaptor_Surface(myBasis));
        Geom_OsculatingSurface? anOscQuery = myGeom!.OsculatingSurface();

        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                ref Geom_Surface.ResD3 aRes = ref aResult.ChangeValue(i, j);
                Geom_OffsetSurfaceUtils.EvaluateD3(aU, aV, aBasisAdaptor, myOffset, anOscQuery, out aRes.Point,
                                                   out aRes.D1U, out aRes.D1V, out aRes.D2U, out aRes.D2V,
                                                   out aRes.D2UV, out aRes.D3U, out aRes.D3V, out aRes.D3UUV,
                                                   out aRes.D3UVV);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        if (myBasis is null || theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0 || theNV < 0
            || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        // Fast path: check if offset surface has an equivalent simple surface
        Geom_Surface? anEquivSurf = equivalentSurface();
        if (anEquivSurf is not null)
        {
            GeomGridEval_Surface anEquivEval = new(anEquivSurf);
            return anEquivEval.EvaluateGridDN(theUParams, theVParams, theNU, theNV);
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Vector3D> aResult = new(1, aNbU, 1, aNbV);

        // Batch evaluate basis surface D1 (offset DN requires basis D1)
        GeomGridEval_Surface aBasisEval = new(myBasis);
        NCollection_Array2<Geom_Surface.ResD1> aBasisD1 = aBasisEval.EvaluateGridD1(theUParams, theVParams);
        if (aBasisD1.IsEmpty())
        {
            return new NCollection_Array2<Vector3D>();
        }

        Geom_OffsetSurfaceUtils.AdaptorBasis aBasisAdaptor = new(new GeomAdaptor_Surface(myBasis));
        Geom_OsculatingSurface? anOscQuery = myGeom!.OsculatingSurface();

        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                ref Geom_Surface.ResD1 aBasis = ref aBasisD1.ChangeValue(i, j);
                Geom_OffsetSurfaceUtils.EvaluateDN(aU, aV, theNU, theNV, aBasisAdaptor, myOffset, anOscQuery,
                                                   out Vector3D aDN, aBasis.D1U, aBasis.D1V);
                aResult.SetValue(i, j, aDN);
            }
        }
        return aResult;
    }
}
