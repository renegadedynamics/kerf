// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Curve.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Unified grid evaluator for any curve: detects the curve type (through trimmed curves) and dispatches to the
/// specialised evaluator (line, conics, Bezier, B-spline, offset), or evaluates point by point through the adaptor
/// for an adaptor that isn't a GeomAdaptor_Curve.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="GeomGridEval.ICurveEvaluator"/> field, null for the
/// uninitialised state (a null curve or an unknown Geom_Curve subclass), which gives empty results. OCCT's
/// protected <c>initialization</c> overloads are private: the class is sealed.
/// </remarks>
public sealed class GeomGridEval_Curve : GeomGridEval.ICurveEvaluator
{
    private GeomGridEval.ICurveEvaluator? myEvaluator;
    private CurveKind myCurveType;

    /// <summary>
    /// Constructor from an adaptor: a GeomAdaptor_Curve is evaluated through its curve, any other adaptor point by
    /// point (the adaptor is kept).
    /// </summary>
    public GeomGridEval_Curve(Adaptor3d_Curve theCurve)
    {
        myEvaluator = null;
        myCurveType = OtherCurve;
        initialization(theCurve);
    }

    /// <summary>Constructor from a curve (null gives empty results).</summary>
    public GeomGridEval_Curve(Geom_Curve? theCurve)
    {
        myEvaluator = null;
        myCurveType = OtherCurve;
        initialization(theCurve);
    }

    /// <summary>Returns the detected curve type.</summary>
    public new CurveKind GetType() => myCurveType;

    // Extracts basis curve from potentially nested TrimmedCurve wrappers.
    private static Geom_Curve ExtractBasisCurve(Geom_Curve theCurve)
    {
        Geom_Curve aResult = theCurve;
        while (aResult is Geom_TrimmedCurve aTrimmed)
        {
            aResult = aTrimmed.BasisCurve();
        }
        return aResult;
    }

    private void initialization(Adaptor3d_Curve theCurve)
    {
        if (theCurve is GeomAdaptor_Curve aGeomAdaptor)
        {
            initialization(aGeomAdaptor.Curve());
            return;
        }

        // For non-GeomAdaptor or when Geom_Curve is not available,
        // use reference for the evaluator
        myCurveType = theCurve.GetType();
        myEvaluator = new GeomGridEval_OtherCurve(theCurve);
    }

    private void initialization(Geom_Curve? theCurve)
    {
        if (theCurve is null)
        {
            myEvaluator = null;
            myCurveType = OtherCurve;
            return;
        }

        // Extract basis curve from potentially nested TrimmedCurve wrappers
        Geom_Curve aBasisCurve = ExtractBasisCurve(theCurve);

        if (aBasisCurve is Geom_Line aLine)
        {
            myCurveType = Line;
            myEvaluator = new GeomGridEval_Line(aLine);
        }
        else if (aBasisCurve is Geom_Circle aCircle)
        {
            myCurveType = Circle;
            myEvaluator = new GeomGridEval_Circle(aCircle);
        }
        else if (aBasisCurve is Geom_Ellipse anEllipse)
        {
            myCurveType = Ellipse;
            myEvaluator = new GeomGridEval_Ellipse(anEllipse);
        }
        else if (aBasisCurve is Geom_Hyperbola aHyperbola)
        {
            myCurveType = Hyperbola;
            myEvaluator = new GeomGridEval_Hyperbola(aHyperbola);
        }
        else if (aBasisCurve is Geom_Parabola aParabola)
        {
            myCurveType = Parabola;
            myEvaluator = new GeomGridEval_Parabola(aParabola);
        }
        else if (aBasisCurve is Geom_BezierCurve aBezier)
        {
            myCurveType = BezierCurve;
            myEvaluator = new GeomGridEval_BezierCurve(aBezier);
        }
        else if (aBasisCurve is Geom_BSplineCurve aBSpline)
        {
            myCurveType = BSplineCurve;
            myEvaluator = new GeomGridEval_BSplineCurve(aBSpline);
        }
        else if (aBasisCurve is Geom_OffsetCurve anOffset)
        {
            myCurveType = OffsetCurve;
            myEvaluator = new GeomGridEval_OffsetCurve(anOffset);
        }
        else
        {
            // Unknown curve type - set uninitialized
            // All known Geom_Curve types are handled above
            myEvaluator = null;
            myCurveType = OtherCurve;
        }
    }

    /// <summary>Evaluates grid points at all parameters; 1-based, or empty.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Point3D>() : myEvaluator.EvaluateGrid(theParams);
    }

    /// <summary>Evaluates grid points with the first derivative; 1-based, or empty.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom_Curve.ResD1>() : myEvaluator.EvaluateGridD1(theParams);
    }

    /// <summary>Evaluates grid points with the first and second derivatives; 1-based, or empty.</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom_Curve.ResD2>() : myEvaluator.EvaluateGridD2(theParams);
    }

    /// <summary>Evaluates grid points with derivatives up to the third; 1-based, or empty.</summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom_Curve.ResD3>() : myEvaluator.EvaluateGridD3(theParams);
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; 1-based, or empty.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        return myEvaluator is null ? new NCollection_Array1<Vector3D>() : myEvaluator.EvaluateGridDN(theParams, theN);
    }
}
