// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Fallback evaluator for any surface type: point-by-point evaluation through the D0/D1/D2/D3/DN methods of an
/// Adaptor3d_Surface or a Geom_Surface. The slowest evaluator, but it handles any surface.
/// </summary>
/// <remarks>The evaluator keeps the adaptor or surface it is given (OCCT keeps a pointer or a handle).</remarks>
public sealed class GeomGridEval_OtherSurface : GeomGridEval.ISurfaceEvaluator
{
    // OCCT's std::variant<const Adaptor3d_Surface*, occ::handle<Geom_Surface>>: one of the two.
    private readonly object mySurface;

    /// <summary>Constructor with an adaptor.</summary>
    public GeomGridEval_OtherSurface(Adaptor3d_Surface theSurface)
    {
        mySurface = theSurface;
    }

    /// <summary>Constructor with a geometry.</summary>
    public GeomGridEval_OtherSurface(Geom_Surface theSurface)
    {
        mySurface = theSurface;
    }

    /// <summary>Evaluate D0 at given UV parameters.</summary>
    private void evaluateD0(double theU, double theV, out Point3D thePoint)
    {
        if (mySurface is Adaptor3d_Surface aSurf)
        {
            aSurf.D0(theU, theV, out thePoint);
        }
        else
        {
            ((Geom_Surface)mySurface).D0(theU, theV, out thePoint);
        }
    }

    /// <summary>Evaluate D1 at given UV parameters.</summary>
    private void evaluateD1(double theU, double theV, out Point3D thePoint, out Vector3D theD1U, out Vector3D theD1V)
    {
        if (mySurface is Adaptor3d_Surface aSurf)
        {
            aSurf.D1(theU, theV, out thePoint, out theD1U, out theD1V);
        }
        else
        {
            ((Geom_Surface)mySurface).D1(theU, theV, out thePoint, out theD1U, out theD1V);
        }
    }

    /// <summary>Evaluate D2 at given UV parameters.</summary>
    private void evaluateD2(double theU,
                            double theV,
                            out Point3D thePoint,
                            out Vector3D theD1U,
                            out Vector3D theD1V,
                            out Vector3D theD2U,
                            out Vector3D theD2V,
                            out Vector3D theD2UV)
    {
        if (mySurface is Adaptor3d_Surface aSurf)
        {
            aSurf.D2(theU, theV, out thePoint, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV);
        }
        else
        {
            ((Geom_Surface)mySurface).D2(theU, theV, out thePoint, out theD1U, out theD1V, out theD2U, out theD2V,
                                         out theD2UV);
        }
    }

    /// <summary>Evaluate D3 at given UV parameters.</summary>
    private void evaluateD3(double theU,
                            double theV,
                            out Point3D thePoint,
                            out Vector3D theD1U,
                            out Vector3D theD1V,
                            out Vector3D theD2U,
                            out Vector3D theD2V,
                            out Vector3D theD2UV,
                            out Vector3D theD3U,
                            out Vector3D theD3V,
                            out Vector3D theD3UUV,
                            out Vector3D theD3UVV)
    {
        if (mySurface is Adaptor3d_Surface aSurf)
        {
            aSurf.D3(theU, theV, out thePoint, out theD1U, out theD1V, out theD2U, out theD2V, out theD2UV,
                     out theD3U, out theD3V, out theD3UUV, out theD3UVV);
        }
        else
        {
            ((Geom_Surface)mySurface).D3(theU, theV, out thePoint, out theD1U, out theD1V, out theD2U, out theD2V,
                                         out theD2UV, out theD3U, out theD3V, out theD3UUV, out theD3UVV);
        }
    }

    /// <summary>Evaluate DN at given UV parameters.</summary>
    private Vector3D evaluateDN(double theU, double theV, int theNU, int theNV)
    {
        if (mySurface is Adaptor3d_Surface aSurf)
        {
            return aSurf.DN(theU, theV, theNU, theNV);
        }
        return ((Geom_Surface)mySurface).DN(theU, theV, theNU, theNV);
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Point3D>();
        }

        NCollection_Array2<Point3D> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                evaluateD0(u, v, out Point3D aPoint);
                aResult.SetValue(iU, iV, aPoint);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                ref Geom_Surface.ResD1 aRes = ref aResult.ChangeValue(iU, iV);
                evaluateD1(u, v, out aRes.Point, out aRes.D1U, out aRes.D1V);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                ref Geom_Surface.ResD2 aRes = ref aResult.ChangeValue(iU, iV);
                evaluateD2(u, v, out aRes.Point, out aRes.D1U, out aRes.D1V, out aRes.D2U, out aRes.D2V,
                           out aRes.D2UV);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                ref Geom_Surface.ResD3 aRes = ref aResult.ChangeValue(iU, iV);
                evaluateD3(u, v, out aRes.Point, out aRes.D1U, out aRes.D1V, out aRes.D2U, out aRes.D2V,
                           out aRes.D2UV, out aRes.D3U, out aRes.D3V, out aRes.D3UUV, out aRes.D3UVV);
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
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (aNbU == 0 || aNbV == 0 || theNU < 0 || theNV < 0 || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        NCollection_Array2<Vector3D> aResult = new(1, aNbU, 1, aNbV);
        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                Vector3D aDN = evaluateDN(u, v, theNU, theNV);
                aResult.SetValue(iU, iV, aDN);
            }
        }
        return aResult;
    }
}
