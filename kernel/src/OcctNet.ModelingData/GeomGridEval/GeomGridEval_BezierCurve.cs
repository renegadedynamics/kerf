// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BezierCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Splines;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for Bezier curve grid points: one BSplCLib_Cache built for the single span,
/// then polynomial evaluation at each parameter.
/// </summary>
public sealed class GeomGridEval_BezierCurve : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_BezierCurve? myGeom;

    /// <summary>Constructor with the Bezier curve to evaluate (null gives empty results).</summary>
    public GeomGridEval_BezierCurve(Geom_BezierCurve? theCurve)
    {
        myGeom = theCurve;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_BezierCurve? Geometry() => myGeom;

    // Creates a BSplCLib_Cache for a Bezier curve, ready for evaluation.
    private static Cache CreateBezierCache(Geom_BezierCurve theCurve)
    {
        NCollection_Array1<double> aKnotSequence = theCurve.InternalKnotSequence();
        NCollection_Array1<Point3D> aPoles = theCurve.InternalPoles();
        NCollection_Array1<double>? aWeights = theCurve.InternalWeights();
        Cache aCache = new(theCurve.Degree(), false, aKnotSequence, aPoles, aWeights);
        aCache.BuildCache(0.5, aKnotSequence, aPoles, aWeights);
        return aCache;
    }

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point3D> aResult = new(1, aNb);

        Cache aCache = CreateBezierCache(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D0(theParams.Value(i), out Point3D aP);
            aResult.SetValue(i - theParams.Lower() + 1, aP);
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, aNb);

        Cache aCache = CreateBezierCache(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D1(theParams.Value(i), out Point3D aP, out Vector3D aD1);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1 { Point = aP, D1 = aD1 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, aNb);

        Cache aCache = CreateBezierCache(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D2(theParams.Value(i), out Point3D aP, out Vector3D aD1, out Vector3D aD2);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2 { Point = aP, D1 = aD1, D2 = aD2 };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, aNb);

        Cache aCache = CreateBezierCache(myGeom);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            aCache.D3(theParams.Value(i), out Point3D aP, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3 { Point = aP, D1 = aD1, D2 = aD2, D3 = aD3 };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points (zero above the degree, also when rational, as in OCCT).</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector3D> aResult = new(1, aNb);

        // For Bezier curves, derivatives become zero when order exceeds degree
        // OCCT: not for a rational curve, whose derivatives above the degree are generally not zero
        int aDegree = myGeom.Degree();
        if (theN > aDegree)
        {
            Vector3D aZeroVec = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aZeroVec);
            }
            return aResult;
        }

        // Get poles, weights, and flat knots from geometry
        NCollection_Array1<Point3D> aPoles = myGeom.InternalPoles();
        NCollection_Array1<double>? aWeights = myGeom.InternalWeights();
        NCollection_Array1<double> aKnotSequence = myGeom.InternalKnotSequence();

        // Bezier has a single span (index 0 with flat knots), non-periodic
        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(theParams.Value(i),
                        theN,
                        0, // span index (single span for Bezier with flat knots)
                        aDegree,
                        false, // not periodic
                        aPoles,
                        aWeights,
                        aKnotSequence,
                        null, // no multiplicities with flat knots
                        out Vector3D aDN);
            aResult.SetValue(i - theParams.Lower() + 1, aDN);
        }
        return aResult;
    }
}
