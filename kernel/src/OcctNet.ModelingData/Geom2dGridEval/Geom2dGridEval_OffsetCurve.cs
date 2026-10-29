// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OffsetCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D offset curve grid points: evaluates the basis curve with
/// <see cref="Geom2dGridEval_Curve"/>, then applies the offset (Geom2d_OffsetCurveUtils).
/// </summary>
/// <remarks>
/// If the offset can't be computed at one parameter (a zero basis tangent that can't be adjusted), the whole
/// result is empty, as in OCCT.
/// </remarks>
public sealed class Geom2dGridEval_OffsetCurve : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_OffsetCurve? myGeom;
    private readonly Geom2d_Curve? myBasis;
    private readonly double myOffset;

    /// <summary>Constructor with the offset curve to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_OffsetCurve(Geom2d_OffsetCurve? theOffset)
    {
        myGeom = theOffset;
        myOffset = 0.0;
        if (myGeom is not null)
        {
            myOffset = myGeom.Offset();
            myBasis = myGeom.BasisCurve();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_OffsetCurve? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        // Offset D0 requires basis D1 to compute offset normal direction
        Geom2dGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom2d_Curve.ResD1> aBasisD1 = aBasisEval.EvaluateGridD1(theParams);
        if (aBasisD1.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aResult = new(1, aNbParams);

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom2d_Curve.ResD1 aBasis = ref aBasisD1[i];
            gp_Pnt2d aP = aBasis.Point;
            if (!Geom2d_OffsetCurveUtils.CalculateD0(ref aP, aBasis.D1, myOffset))
            {
                return new NCollection_Array1<gp_Pnt2d>();
            }
            aResult.SetValue(i, aP);
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        // Offset D1 requires basis D2
        Geom2dGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom2d_Curve.ResD2> aBasisD2 = aBasisEval.EvaluateGridD2(theParams);
        if (aBasisD2.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD1> aResult = new(1, aNbParams);

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom2d_Curve.ResD2 aBasis = ref aBasisD2[i];
            gp_Pnt2d aP = aBasis.Point;
            gp_Vec2d aD1 = aBasis.D1;
            if (!Geom2d_OffsetCurveUtils.CalculateD1(ref aP, ref aD1, aBasis.D2, myOffset))
            {
                return new NCollection_Array1<Geom2d_Curve.ResD1>();
            }
            aResult[i] = new Geom2d_Curve.ResD1 { Point = aP, D1 = aD1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        // Offset D2 requires basis D3
        Geom2dGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom2d_Curve.ResD3> aBasisD3 = aBasisEval.EvaluateGridD3(theParams);
        if (aBasisD3.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD2> aResult = new(1, aNbParams);

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom2d_Curve.ResD3 aBasis = ref aBasisD3[i];
            gp_Pnt2d aP = aBasis.Point;
            gp_Vec2d aD1 = aBasis.D1;
            gp_Vec2d aD2 = aBasis.D2;
            gp_Vec2d aD3 = aBasis.D3;

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (aD1.SquareMagnitude() <= gp.Resolution())
            {
                gp_Vec2d aDummyD4 = new();
                if (!Geom2d_OffsetCurveUtils.AdjustDerivative(myBasis,
                                                              3,
                                                              theParams.Value(theParams.Lower() + i - 1),
                                                              ref aD1,
                                                              ref aD2,
                                                              ref aD3,
                                                              ref aDummyD4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Geom2d_Curve.ResD2>();
                }
            }

            if (!Geom2d_OffsetCurveUtils.CalculateD2(ref aP, ref aD1, ref aD2, aD3, isDirectionChange, myOffset))
            {
                return new NCollection_Array1<Geom2d_Curve.ResD2>();
            }
            aResult[i] = new Geom2d_Curve.ResD2 { Point = aP, D1 = aD1, D2 = aD2 };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates all grid points with derivatives up to the third; the basis's fourth derivative is evaluated point
    /// by point.
    /// </summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        // Offset D3 requires basis D3 + D4
        Geom2dGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom2d_Curve.ResD3> aBasisD3 = aBasisEval.EvaluateGridD3(theParams);
        if (aBasisD3.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNbParams);

        for (int i = 1; i <= aNbParams; ++i)
        {
            double aParam = theParams.Value(theParams.Lower() + i - 1);

            ref readonly Geom2d_Curve.ResD3 aBasis = ref aBasisD3[i];
            gp_Pnt2d aP = aBasis.Point;
            gp_Vec2d aD1 = aBasis.D1;
            gp_Vec2d aD2 = aBasis.D2;
            gp_Vec2d aD3 = aBasis.D3;
            gp_Vec2d aD4 = myBasis.EvalDN(aParam, 4);

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (aD1.SquareMagnitude() <= gp.Resolution())
            {
                if (!Geom2d_OffsetCurveUtils.AdjustDerivative(myBasis,
                                                              4,
                                                              aParam,
                                                              ref aD1,
                                                              ref aD2,
                                                              ref aD3,
                                                              ref aD4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Geom2d_Curve.ResD3>();
                }
            }

            if (!Geom2d_OffsetCurveUtils.CalculateD3(ref aP, ref aD1, ref aD2, ref aD3, aD4, isDirectionChange,
                                                     myOffset))
            {
                return new NCollection_Array1<Geom2d_Curve.ResD3>();
            }
            aResult[i] = new Geom2d_Curve.ResD3 { Point = aP, D1 = aD1, D2 = aD2, D3 = aD3 };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, and as the basis
    /// curve's DN above.
    /// </summary>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myBasis is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNbParams = theParams.Length();

        // Reuse optimized grid evaluators for orders 1-3
        if (theN == 1)
        {
            NCollection_Array1<Geom2d_Curve.ResD1> aD1Grid = EvaluateGridD1(theParams);
            if (aD1Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            NCollection_Array1<gp_Vec2d> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD1Grid[i].D1);
            }
            return aResult;
        }
        else if (theN == 2)
        {
            NCollection_Array1<Geom2d_Curve.ResD2> aD2Grid = EvaluateGridD2(theParams);
            if (aD2Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            NCollection_Array1<gp_Vec2d> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD2Grid[i].D2);
            }
            return aResult;
        }
        else if (theN == 3)
        {
            NCollection_Array1<Geom2d_Curve.ResD3> aD3Grid = EvaluateGridD3(theParams);
            if (aD3Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            NCollection_Array1<gp_Vec2d> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD3Grid[i].D3);
            }
            return aResult;
        }
        else
        {
            // For orders > 3, batch evaluate basis curve DN
            Geom2dGridEval_Curve aBasisEval = new(myBasis);
            return aBasisEval.EvaluateGridDN(theParams, theN);
        }
    }
}
