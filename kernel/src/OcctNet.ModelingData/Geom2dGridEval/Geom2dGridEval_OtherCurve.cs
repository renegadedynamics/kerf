// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OtherCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_OtherCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Fallback batch evaluator for any 2D curve adaptor: evaluates point by point through the adaptor's EvalD0 to
/// EvalDN.
/// </summary>
/// <remarks>
/// OCCT keeps a reference to the adaptor, which must outlive the evaluator; the port keeps the adaptor object.
/// </remarks>
public sealed class Geom2dGridEval_OtherCurve : Geom2dGridEval.ICurveEvaluator
{
    private readonly Adaptor2d_Curve2d myCurve;

    /// <summary>Constructor with the curve adaptor to evaluate.</summary>
    public Geom2dGridEval_OtherCurve(Adaptor2d_Curve2d theCurve)
    {
        myCurve = theCurve;
    }

    /// <summary>Returns the curve adaptor.</summary>
    public Adaptor2d_Curve2d Curve() => myCurve;

    /// <summary>Evaluates all grid points; 1-based, or empty if there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            gp_Pnt2d aP = myCurve.EvalD0(theParams.Value(i));
            aResult.SetValue(i - theParams.Lower() + 1, aP);
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD1> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Geom2d_Curve.ResD1 aD1 = myCurve.EvalD1(theParams.Value(i));
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1 { Point = aD1.Point, D1 = aD1.D1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD2> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Geom2d_Curve.ResD2 aD2 = myCurve.EvalD2(theParams.Value(i));
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2 { Point = aD2.Point, D1 = aD2.D1, D2 = aD2.D2 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNb);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            Geom2d_Curve.ResD3 aD3 = myCurve.EvalD3(theParams.Value(i));
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3
            {
                Point = aD3.Point,
                D1 = aD3.D1,
                D2 = aD3.D2,
                D3 = aD3.D3,
            };
        }
        return aResult;
    }

    /// <summary>
    /// Evaluates the Nth derivative at all grid points: from the D1-D3 grids for N up to 3, with the adaptor's
    /// EvalDN above.
    /// </summary>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec2d> aResult = new(1, aNb);

        // Reuse existing grid evaluators for orders 1-3
        if (theN == 1)
        {
            NCollection_Array1<Geom2d_Curve.ResD1> aD1Grid = EvaluateGridD1(theParams);
            if (aD1Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD1Grid[i].D1);
            }
        }
        else if (theN == 2)
        {
            NCollection_Array1<Geom2d_Curve.ResD2> aD2Grid = EvaluateGridD2(theParams);
            if (aD2Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD2Grid[i].D2);
            }
        }
        else if (theN == 3)
        {
            NCollection_Array1<Geom2d_Curve.ResD3> aD3Grid = EvaluateGridD3(theParams);
            if (aD3Grid.IsEmpty())
            {
                return new NCollection_Array1<gp_Vec2d>();
            }
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD3Grid[i].D3);
            }
        }
        else
        {
            // For orders > 3, use adaptor EvalDN method
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                gp_Vec2d aDN = myCurve.EvalDN(theParams.Value(i), theN);
                aResult.SetValue(i - theParams.Lower() + 1, aDN);
            }
        }
        return aResult;
    }
}
