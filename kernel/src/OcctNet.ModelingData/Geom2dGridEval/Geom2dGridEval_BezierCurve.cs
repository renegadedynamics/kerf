// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D Bezier curve grid points: one BSplCLib_Cache built for the single span,
/// then polynomial evaluation at each parameter.
/// </summary>
public sealed class Geom2dGridEval_BezierCurve : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_BezierCurve? myGeom;

    /// <summary>Constructor with the Bezier curve to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_BezierCurve(Geom2d_BezierCurve? theCurve)
    {
        myGeom = theCurve;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_BezierCurve? Geometry() => myGeom;

    // Creates a BSplCLib_Cache for a 2D Bezier curve, ready for evaluation.
    private static BSplCLib_Cache CreateBezierCache2d(Geom2d_BezierCurve theCurve)
    {
        NCollection_Array1<double> aKnotSequence = theCurve.InternalKnotSequence();
        NCollection_Array1<gp_Pnt2d> aPoles = theCurve.InternalPoles();
        NCollection_Array1<double>? aWeights = theCurve.InternalWeights();
        BSplCLib_Cache aCache = new(theCurve.Degree(), false, aKnotSequence, aPoles, aWeights);
        aCache.BuildCache(0.5, aKnotSequence, aPoles, aWeights);
        return aCache;
    }

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aResult = new(1, aNb);

        BSplCLib_Cache aCache = CreateBezierCache2d(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D0(theParams.Value(i), out gp_Pnt2d aP);
            aResult.SetValue(i - theParams.Lower() + 1, aP);
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD1> aResult = new(1, aNb);

        BSplCLib_Cache aCache = CreateBezierCache2d(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D1(theParams.Value(i), out gp_Pnt2d aP, out gp_Vec2d aD1);
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1 { Point = aP, D1 = aD1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD2> aResult = new(1, aNb);

        BSplCLib_Cache aCache = CreateBezierCache2d(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D2(theParams.Value(i), out gp_Pnt2d aP, out gp_Vec2d aD1, out gp_Vec2d aD2);
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2 { Point = aP, D1 = aD1, D2 = aD2 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNb);

        BSplCLib_Cache aCache = CreateBezierCache2d(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D3(theParams.Value(i), out gp_Pnt2d aP, out gp_Vec2d aD1, out gp_Vec2d aD2, out gp_Vec2d aD3);
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3 { Point = aP, D1 = aD1, D2 = aD2, D3 = aD3 };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points (zero above the degree).</summary>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec2d> aResult = new(1, aNb);

        // For Bezier curves, derivatives become zero when order exceeds degree
        int aDegree = myGeom.Degree();
        if (theN > aDegree)
        {
            gp_Vec2d aZeroVec = new(0.0, 0.0);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aZeroVec);
            }
            return aResult;
        }

        // Get poles, weights, and flat knots from geometry
        NCollection_Array1<gp_Pnt2d> aPoles = myGeom.InternalPoles();
        NCollection_Array1<double>? aWeights = myGeom.InternalWeights();
        NCollection_Array1<double> aKnotSequence = myGeom.InternalKnotSequence();

        // Bezier has a single span (index 0 with flat knots), non-periodic
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            BSplCLib.DN(theParams.Value(i),
                        theN,
                        0, // span index (single span for Bezier with flat knots)
                        aDegree,
                        false, // not periodic
                        aPoles,
                        aWeights,
                        aKnotSequence,
                        null, // no multiplicities with flat knots
                        out gp_Vec2d aDN);
            aResult.SetValue(i - theParams.Lower() + 1, aDN);
        }
        return aResult;
    }
}
