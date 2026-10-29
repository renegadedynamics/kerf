// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OffsetCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for offset curve grid points: evaluates the basis curve with
/// <see cref="GeomGridEval_Curve"/>, then applies the offset (Geom_OffsetCurveUtils).
/// </summary>
/// <remarks>
/// If the offset can't be computed at one parameter (a zero basis tangent that can't be adjusted), the whole
/// result is empty, as in OCCT.
/// </remarks>
public sealed class GeomGridEval_OffsetCurve : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_OffsetCurve? myGeom;
    private readonly Geom_Curve? myBasis;
    private readonly double myOffset;
    private readonly gp_Dir myDirection = new();

    /// <summary>Constructor with the offset curve to evaluate (null gives empty results).</summary>
    public GeomGridEval_OffsetCurve(Geom_OffsetCurve? theOffset)
    {
        myGeom = theOffset;
        myOffset = 0.0;
        if (myGeom is not null)
        {
            myOffset = myGeom.Offset();
            myBasis = myGeom.BasisCurve();
            myDirection = myGeom.Direction();
        }
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_OffsetCurve? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt>();
        }

        // Offset D0 requires basis D1 to compute offset direction
        // Batch evaluate basis curve D1
        GeomGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom_Curve.ResD1> aBasisD1 = aBasisEval.EvaluateGridD1(theParams);
        if (aBasisD1.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<gp_Pnt> aResult = new(1, aNbParams);

        gp_XYZ aDirXYZ = myDirection.XYZ();

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom_Curve.ResD1 aBasis = ref aBasisD1[i];
            gp_Pnt aP = aBasis.Point;
            if (!Geom_OffsetCurveUtils.CalculateD0(ref aP, aBasis.D1, aDirXYZ, myOffset))
            {
                return new NCollection_Array1<gp_Pnt>();
            }
            aResult.SetValue(i, aP);
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        // Offset D1 requires basis D2
        // Batch evaluate basis curve D2
        GeomGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom_Curve.ResD2> aBasisD2 = aBasisEval.EvaluateGridD2(theParams);
        if (aBasisD2.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, aNbParams);

        gp_XYZ aDirXYZ = myDirection.XYZ();

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom_Curve.ResD2 aBasis = ref aBasisD2[i];
            gp_Pnt aP = aBasis.Point;
            gp_Vec aD1 = aBasis.D1;
            if (!Geom_OffsetCurveUtils.CalculateD1(ref aP, ref aD1, aBasis.D2, aDirXYZ, myOffset))
            {
                return new NCollection_Array1<Geom_Curve.ResD1>();
            }
            aResult[i] = new Geom_Curve.ResD1 { Point = aP, D1 = aD1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        // Offset D2 requires basis D3
        // Batch evaluate basis curve D3
        GeomGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom_Curve.ResD3> aBasisD3 = aBasisEval.EvaluateGridD3(theParams);
        if (aBasisD3.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, aNbParams);

        gp_XYZ aDirXYZ = myDirection.XYZ();

        for (int i = 1; i <= aNbParams; ++i)
        {
            ref readonly Geom_Curve.ResD3 aBasis = ref aBasisD3[i];
            gp_Pnt aP = aBasis.Point;
            gp_Vec aD1 = aBasis.D1;
            gp_Vec aD2 = aBasis.D2;
            gp_Vec aD3 = aBasis.D3;

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (aD1.SquareMagnitude() <= gp.Resolution())
            {
                gp_Vec aDummyD4 = new();
                if (!Geom_OffsetCurveUtils.AdjustDerivative(myBasis,
                                                              3,
                                                              theParams.Value(theParams.Lower() + i - 1),
                                                              ref aD1,
                                                              ref aD2,
                                                              ref aD3,
                                                              ref aDummyD4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Geom_Curve.ResD2>();
                }
            }

            if (!Geom_OffsetCurveUtils.CalculateD2(ref aP, ref aD1, ref aD2, aD3, aDirXYZ, myOffset, isDirectionChange))
            {
                return new NCollection_Array1<Geom_Curve.ResD2>();
            }
            aResult[i] = new Geom_Curve.ResD2 { Point = aP, D1 = aD1, D2 = aD2 };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates all grid points with derivatives up to the third; the basis's fourth derivative is evaluated point
    /// by point.
    /// </summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myBasis is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        // Offset D3 requires basis D3 + D4
        // Batch evaluate basis curve D3, get D4 individually
        GeomGridEval_Curve aBasisEval = new(myBasis);

        NCollection_Array1<Geom_Curve.ResD3> aBasisD3 = aBasisEval.EvaluateGridD3(theParams);
        if (aBasisD3.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        int aNbParams = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, aNbParams);

        gp_XYZ aDirXYZ = myDirection.XYZ();

        for (int i = 1; i <= aNbParams; ++i)
        {
            double aParam = theParams.Value(theParams.Lower() + i - 1);

            ref readonly Geom_Curve.ResD3 aBasis = ref aBasisD3[i];
            gp_Pnt aP = aBasis.Point;
            gp_Vec aD1 = aBasis.D1;
            gp_Vec aD2 = aBasis.D2;
            gp_Vec aD3 = aBasis.D3;
            gp_Vec aD4 = myBasis.DN(aParam, 4);

            // Check for direction change at singular points
            bool isDirectionChange = false;
            if (aD1.SquareMagnitude() <= gp.Resolution())
            {
                if (!Geom_OffsetCurveUtils.AdjustDerivative(myBasis,
                                                              4,
                                                              aParam,
                                                              ref aD1,
                                                              ref aD2,
                                                              ref aD3,
                                                              ref aD4,
                                                              out isDirectionChange))
                {
                    return new NCollection_Array1<Geom_Curve.ResD3>();
                }
            }

            if (!Geom_OffsetCurveUtils.CalculateD3(ref aP, ref aD1, ref aD2, ref aD3, aD4, aDirXYZ, myOffset,
                                                   isDirectionChange))
            {
                return new NCollection_Array1<Geom_Curve.ResD3>();
            }
            aResult[i] = new Geom_Curve.ResD3 { Point = aP, D1 = aD1, D2 = aD2, D3 = aD3 };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, and as the basis
    /// curve's DN above.
    /// </summary>
    public NCollection_Array1<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myBasis is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec>();
        }

        int aNbParams = theParams.Length();

        // Reuse optimized grid evaluators for orders 1-3
        // OCCT: when the offset can't be computed at a parameter, EvaluateGridD1/D2/D3 return an empty array that
        // is then read out of range (undefined in OCCT, Standard_OutOfRange here); Geom2dGridEval checks for it.
        if (theN == 1)
        {
            NCollection_Array1<Geom_Curve.ResD1> aD1Grid = EvaluateGridD1(theParams);
            NCollection_Array1<gp_Vec> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD1Grid[i].D1);
            }
            return aResult;
        }
        else if (theN == 2)
        {
            NCollection_Array1<Geom_Curve.ResD2> aD2Grid = EvaluateGridD2(theParams);
            NCollection_Array1<gp_Vec> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD2Grid[i].D2);
            }
            return aResult;
        }
        else if (theN == 3)
        {
            NCollection_Array1<Geom_Curve.ResD3> aD3Grid = EvaluateGridD3(theParams);
            NCollection_Array1<gp_Vec> aResult = new(1, aNbParams);
            for (int i = 1; i <= aNbParams; ++i)
            {
                aResult.SetValue(i, aD3Grid[i].D3);
            }
            return aResult;
        }
        else
        {
            // For orders > 3, offset curve DN = basis curve DN (no offset contribution)
            // Batch evaluate basis curve DN
            GeomGridEval_Curve aBasisEval = new(myBasis);
            return aBasisEval.EvaluateGridDN(theParams, theN);
        }
    }
}
