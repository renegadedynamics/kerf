// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfExtrusion.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for surface of linear extrusion grid points, S(u, v) = C(u) + v * Direction: the
/// basis curve is evaluated once per U parameter with <see cref="GeomGridEval_Curve"/> (Geom_ExtrusionUtils).
/// </summary>
public sealed class GeomGridEval_SurfaceOfExtrusion : GeomGridEval.ISurfaceEvaluator
{
    private readonly Geom_SurfaceOfLinearExtrusion? myGeom;
    private readonly Geom_Curve? myBasisCurve;
    private readonly Direction3D myDirection;

    /// <summary>Constructor with the extrusion surface to evaluate (null gives empty results).</summary>
    public GeomGridEval_SurfaceOfExtrusion(Geom_SurfaceOfLinearExtrusion? theExtrusion)
    {
        myGeom = theExtrusion;
        myDirection = new Direction3D();
        if (myGeom is not null)
        {
            myBasisCurve = myGeom.BasisCurve();
            myDirection = myGeom.Direction();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_SurfaceOfLinearExtrusion? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Batch evaluate curve points using optimized curve evaluator
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Point3D> aCurvePoints = aCurveEval.EvaluateGrid(theUParams);
        if (aCurvePoints.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        // For each curve point, compute surface points for all V values
        // S(u, v) = C(u) + v * Direction
        Xyz aDirXYZ = myDirection.XYZ();
        NCollection_Array2<Point3D> aResult = new(1, aNbU, 1, aNbV);

        for (int i = 1; i <= aNbU; ++i)
        {
            Point3D aCurvePt = aCurvePoints.Value(i);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                Geom_ExtrusionUtils.CalculateD0(aCurvePt, aV, aDirXYZ, out Point3D aP);
                aResult.SetValue(i, j, aP);
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Batch evaluate curve D1 using optimized curve evaluator
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD1> aCurveD1 = aCurveEval.EvaluateGridD1(theUParams);
        if (aCurveD1.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction (constant)
        Xyz aDirXYZ = myDirection.XYZ();
        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);

        for (int i = 1; i <= aNbU; ++i)
        {
            ref Geom_Curve.ResD1 aCurveData = ref aCurveD1.ChangeValue(i);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                Geom_ExtrusionUtils.CalculateD1(aCurveData.Point,
                                                aCurveData.D1,
                                                aV,
                                                aDirXYZ,
                                                out Point3D aP,
                                                out Vector3D aD1U,
                                                out Vector3D aD1V);
                aResult[i, j] = new Geom_Surface.ResD1 { Point = aP, D1U = aD1U, D1V = aD1V };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Batch evaluate curve D2 using optimized curve evaluator
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD2> aCurveD2 = aCurveEval.EvaluateGridD2(theUParams);
        if (aCurveD2.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction
        // D2U = C''(u), D2V = 0, D2UV = 0
        Xyz aDirXYZ = myDirection.XYZ();
        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);

        for (int i = 1; i <= aNbU; ++i)
        {
            ref Geom_Curve.ResD2 aCurveData = ref aCurveD2.ChangeValue(i);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                Geom_ExtrusionUtils.CalculateD2(aCurveData.Point,
                                                aCurveData.D1,
                                                aCurveData.D2,
                                                aV,
                                                aDirXYZ,
                                                out Point3D aP,
                                                out Vector3D aD1U,
                                                out Vector3D aD1V,
                                                out Vector3D aD2U,
                                                out Vector3D aD2V,
                                                out Vector3D aD2UV);
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

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Batch evaluate curve D3 using optimized curve evaluator
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD3> aCurveD3 = aCurveEval.EvaluateGridD3(theUParams);
        if (aCurveD3.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        // For extrusion: D1U = C'(u), D1V = Direction
        // D2U = C''(u), D2V = 0, D2UV = 0
        // D3U = C'''(u), D3V = 0, D3UUV = 0, D3UVV = 0
        Xyz aDirXYZ = myDirection.XYZ();
        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);

        for (int i = 1; i <= aNbU; ++i)
        {
            ref Geom_Curve.ResD3 aCurveData = ref aCurveD3.ChangeValue(i);
            for (int j = 1; j <= aNbV; ++j)
            {
                double aV = theVParams.Value(theVParams.Lower() + j - 1);
                Geom_ExtrusionUtils.CalculateD3(aCurveData.Point,
                                                aCurveData.D1,
                                                aCurveData.D2,
                                                aCurveData.D3,
                                                aV,
                                                aDirXYZ,
                                                out Point3D aP,
                                                out Vector3D aD1U,
                                                out Vector3D aD1V,
                                                out Vector3D aD2U,
                                                out Vector3D aD2V,
                                                out Vector3D aD2UV,
                                                out Vector3D aD3U,
                                                out Vector3D aD3V,
                                                out Vector3D aD3UUV,
                                                out Vector3D aD3UVV);
                aResult[i, j] = new Geom_Surface.ResD3
                {
                    Point = aP,
                    D1U = aD1U,
                    D1V = aD1V,
                    D2U = aD2U,
                    D2V = aD2V,
                    D2UV = aD2UV,
                    D3U = aD3U,
                    D3V = aD3V,
                    D3UUV = aD3UUV,
                    D3UVV = aD3UVV,
                };
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
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0
            || theNV < 0 || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        NCollection_Array2<Vector3D> aResult = new(1, aNbU, 1, aNbV);

        Xyz aDirXYZ = myDirection.XYZ();

        // For theNV == 0: need curve derivative, otherwise use CalculateDN directly
        if (theNV == 0)
        {
            // Pure U derivative = curve derivative
            GeomGridEval_Curve aCurveEval = new(myBasisCurve);
            NCollection_Array1<Vector3D> aCurveDN = aCurveEval.EvaluateGridDN(theUParams, theNU);
            for (int i = 1; i <= aNbU; ++i)
            {
                Vector3D aDN = Geom_ExtrusionUtils.CalculateDN(aCurveDN.Value(i), aDirXYZ, theNU, theNV);
                for (int j = 1; j <= aNbV; ++j)
                {
                    aResult.SetValue(i, j, aDN);
                }
            }
        }
        else
        {
            // For theNV >= 1: result doesn't depend on curve, use CalculateDN with zero vector
            Vector3D aDN = Geom_ExtrusionUtils.CalculateDN(new Vector3D(), aDirXYZ, theNU, theNV);
            for (int i = 1; i <= aNbU; ++i)
            {
                for (int j = 1; j <= aNbV; ++j)
                {
                    aResult.SetValue(i, j, aDN);
                }
            }
        }

        return aResult;
    }
}
