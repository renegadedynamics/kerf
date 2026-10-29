// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for surface of revolution grid points: the basis curve is evaluated once per V
/// parameter with <see cref="GeomGridEval_Curve"/>, then rotated about the axis for each U parameter
/// (Geom_RevolutionUtils).
/// </summary>
public sealed class GeomGridEval_SurfaceOfRevolution : GeomGridEval.ISurfaceEvaluator
{
    private readonly Geom_SurfaceOfRevolution? myGeom;
    private readonly Geom_Curve? myBasisCurve;
    private readonly gp_Ax1 myAxis;
    private readonly gp_Pnt myAxisLocation;
    private readonly gp_Dir myAxisDirection;

    /// <summary>Constructor with the surface of revolution to evaluate (null gives empty results).</summary>
    public GeomGridEval_SurfaceOfRevolution(Geom_SurfaceOfRevolution? theRevolution)
    {
        myGeom = theRevolution;
        myAxis = new gp_Ax1();
        myAxisLocation = new gp_Pnt();
        myAxisDirection = new gp_Dir();
        if (myGeom is not null)
        {
            myBasisCurve = myGeom.BasisCurve();
            myAxis = myGeom.Axis();
            myAxisLocation = myAxis.Location();
            myAxisDirection = myAxis.Direction();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_SurfaceOfRevolution? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<gp_Pnt>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        // Batch evaluate curve points using optimized curve evaluator
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<gp_Pnt> aCurvePoints = aCurveEval.EvaluateGrid(theVParams);
        if (aCurvePoints.IsEmpty())
        {
            return new NCollection_Array2<gp_Pnt>();
        }

        NCollection_Array2<gp_Pnt> aResult = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                Geom_RevolutionUtils.CalculateD0(aCurvePoints.Value(j), aU, myAxis, out gp_Pnt aP);
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

        // Batch evaluate curve D1
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD1> aCurveD1 = aCurveEval.EvaluateGridD1(theVParams);
        if (aCurveD1.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                ref Geom_Curve.ResD1 aCurveData = ref aCurveD1.ChangeValue(j);
                Geom_RevolutionUtils.CalculateD1(aCurveData.Point,
                                                 aCurveData.D1,
                                                 aU,
                                                 myAxis,
                                                 out gp_Pnt aP,
                                                 out gp_Vec aD1U,
                                                 out gp_Vec aD1V);
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

        // Batch evaluate curve D2
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD2> aCurveD2 = aCurveEval.EvaluateGridD2(theVParams);
        if (aCurveD2.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                ref Geom_Curve.ResD2 aCurveData = ref aCurveD2.ChangeValue(j);
                Geom_RevolutionUtils.CalculateD2(aCurveData.Point,
                                                 aCurveData.D1,
                                                 aCurveData.D2,
                                                 aU,
                                                 myAxis,
                                                 out gp_Pnt aP,
                                                 out gp_Vec aD1U,
                                                 out gp_Vec aD1V,
                                                 out gp_Vec aD2U,
                                                 out gp_Vec aD2V,
                                                 out gp_Vec aD2UV);
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

        // Batch evaluate curve D3
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);
        NCollection_Array1<Geom_Curve.ResD3> aCurveD3 = aCurveEval.EvaluateGridD3(theVParams);
        if (aCurveD3.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; ++i)
        {
            double aU = theUParams.Value(theUParams.Lower() + i - 1);
            for (int j = 1; j <= aNbV; ++j)
            {
                ref Geom_Curve.ResD3 aCurveData = ref aCurveD3.ChangeValue(j);
                Geom_RevolutionUtils.CalculateD3(aCurveData.Point,
                                                 aCurveData.D1,
                                                 aCurveData.D2,
                                                 aCurveData.D3,
                                                 aU,
                                                 myAxis,
                                                 out gp_Pnt aP,
                                                 out gp_Vec aD1U,
                                                 out gp_Vec aD1V,
                                                 out gp_Vec aD2U,
                                                 out gp_Vec aD2V,
                                                 out gp_Vec aD2UV,
                                                 out gp_Vec aD3U,
                                                 out gp_Vec aD3V,
                                                 out gp_Vec aD3UUV,
                                                 out gp_Vec aD3UVV);
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
    public NCollection_Array2<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        if (myBasisCurve is null || theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0
            || theNV < 0 || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<gp_Vec>();
        }

        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        NCollection_Array2<gp_Vec> aResult = new(1, aNbU, 1, aNbV);

        // Get curve data
        GeomGridEval_Curve aCurveEval = new(myBasisCurve);

        if (theNU == 0)
        {
            // Pure V derivative = curve derivative, rotated
            NCollection_Array1<gp_Vec> aCurveDN = aCurveEval.EvaluateGridDN(theVParams, theNV);
            for (int i = 1; i <= aNbU; ++i)
            {
                double aU = theUParams.Value(theUParams.Lower() + i - 1);
                for (int j = 1; j <= aNbV; ++j)
                {
                    gp_Vec aDN = Geom_RevolutionUtils.CalculateDN(aCurveDN.Value(j), aU, myAxis, theNU, theNV);
                    aResult.SetValue(i, j, aDN);
                }
            }
        }
        else
        {
            // Mixed or pure U derivative
            // Get curve point or derivative depending on theNV
            NCollection_Array1<gp_Vec> aCurveDV = new();
            NCollection_Array1<gp_Pnt> aCurvePts = new();
            if (theNV == 0)
            {
                aCurvePts = aCurveEval.EvaluateGrid(theVParams);
            }
            else
            {
                aCurveDV = aCurveEval.EvaluateGridDN(theVParams, theNV);
            }

            for (int i = 1; i <= aNbU; ++i)
            {
                double aU = theUParams.Value(theUParams.Lower() + i - 1);
                for (int j = 1; j <= aNbV; ++j)
                {
                    gp_Vec aCurvePtOrDN;
                    if (theNV == 0)
                    {
                        // For pure U derivative, pass (P - AxisLocation) as the base vector
                        aCurvePtOrDN = new gp_Vec(aCurvePts.Value(j).XYZ() - myAxisLocation.XYZ());
                    }
                    else
                    {
                        aCurvePtOrDN = aCurveDV.Value(j);
                    }
                    gp_Vec aDN = Geom_RevolutionUtils.CalculateDN(aCurvePtOrDN, aU, myAxis, theNU, theNV);
                    aResult.SetValue(i, j, aDN);
                }
            }
        }

        return aResult;
    }
}
