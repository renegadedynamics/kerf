// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Curve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Unified grid evaluator for any 2D curve: detects the curve type (through trimmed curves) and dispatches to the
/// specialised evaluator (line, conics, Bezier, B-spline, offset), or evaluates point by point through the adaptor
/// for an adaptor that isn't a Geom2dAdaptor_Curve.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="Geom2dGridEval.ICurveEvaluator"/> field, null for the
/// uninitialised state (a null curve or an unknown Geom2d_Curve subclass), which gives empty results. OCCT's
/// protected <c>initialization</c> overloads are private: the class is sealed.
/// </remarks>
public sealed class Geom2dGridEval_Curve : Geom2dGridEval.ICurveEvaluator
{
    private Geom2dGridEval.ICurveEvaluator? myEvaluator;
    private CurveKind myCurveType;

    /// <summary>
    /// Constructor from an adaptor: a Geom2dAdaptor_Curve is evaluated through its curve, any other adaptor point by
    /// point (the adaptor is kept).
    /// </summary>
    public Geom2dGridEval_Curve(Adaptor2d_Curve2d theCurve)
    {
        myEvaluator = null;
        myCurveType = OtherCurve;
        initialization(theCurve);
    }

    /// <summary>Constructor from a curve (null gives empty results).</summary>
    public Geom2dGridEval_Curve(Geom2d_Curve? theCurve)
    {
        myEvaluator = null;
        myCurveType = OtherCurve;
        initialization(theCurve);
    }

    /// <summary>Returns the detected curve type.</summary>
    public new CurveKind GetType() => myCurveType;

    // Extracts basis curve from potentially nested TrimmedCurve wrappers.
    private static Geom2d_Curve ExtractBasisCurve(Geom2d_Curve theCurve)
    {
        Geom2d_Curve aResult = theCurve;
        while (aResult is Geom2d_TrimmedCurve aTrimmed)
        {
            aResult = aTrimmed.BasisCurve();
        }
        return aResult;
    }

    private void initialization(Adaptor2d_Curve2d theCurve)
    {
        if (theCurve is Geom2dAdaptor_Curve aGeomAdaptor)
        {
            initialization(aGeomAdaptor.Curve());
            return;
        }

        // For non-Geom2dAdaptor or when Geom2d_Curve is not available,
        // use reference for the evaluator
        myCurveType = theCurve.GetType();
        myEvaluator = new Geom2dGridEval_OtherCurve(theCurve);
    }

    private void initialization(Geom2d_Curve? theCurve)
    {
        if (theCurve is null)
        {
            myEvaluator = null;
            myCurveType = OtherCurve;
            return;
        }

        // Extract basis curve from potentially nested TrimmedCurve wrappers
        Geom2d_Curve aBasisCurve = ExtractBasisCurve(theCurve);

        if (aBasisCurve is Geom2d_Line aLine)
        {
            myCurveType = Line;
            myEvaluator = new Geom2dGridEval_Line(aLine);
        }
        else if (aBasisCurve is Geom2d_Circle aCircle)
        {
            myCurveType = Circle;
            myEvaluator = new Geom2dGridEval_Circle(aCircle);
        }
        else if (aBasisCurve is Geom2d_Ellipse anEllipse)
        {
            myCurveType = Ellipse;
            myEvaluator = new Geom2dGridEval_Ellipse(anEllipse);
        }
        else if (aBasisCurve is Geom2d_Hyperbola aHyperbola)
        {
            myCurveType = Hyperbola;
            myEvaluator = new Geom2dGridEval_Hyperbola(aHyperbola);
        }
        else if (aBasisCurve is Geom2d_Parabola aParabola)
        {
            myCurveType = Parabola;
            myEvaluator = new Geom2dGridEval_Parabola(aParabola);
        }
        else if (aBasisCurve is Geom2d_BezierCurve aBezier)
        {
            myCurveType = BezierCurve;
            myEvaluator = new Geom2dGridEval_BezierCurve(aBezier);
        }
        else if (aBasisCurve is Geom2d_BSplineCurve aBSpline)
        {
            myCurveType = BSplineCurve;
            myEvaluator = new Geom2dGridEval_BSplineCurve(aBSpline);
        }
        else if (aBasisCurve is Geom2d_OffsetCurve anOffset)
        {
            myCurveType = OffsetCurve;
            myEvaluator = new Geom2dGridEval_OffsetCurve(anOffset);
        }
        else
        {
            // Unknown curve type - set uninitialized
            myEvaluator = null;
            myCurveType = OtherCurve;
        }
    }

    /// <summary>Evaluates grid points at all parameters; 1-based, or empty.</summary>
    public NCollection_Array1<Point2D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Point2D>() : myEvaluator.EvaluateGrid(theParams);
    }

    /// <summary>Evaluates grid points with the first derivative; 1-based, or empty.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom2d_Curve.ResD1>() : myEvaluator.EvaluateGridD1(theParams);
    }

    /// <summary>Evaluates grid points with the first and second derivatives; 1-based, or empty.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom2d_Curve.ResD2>() : myEvaluator.EvaluateGridD2(theParams);
    }

    /// <summary>Evaluates grid points with derivatives up to the third; 1-based, or empty.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        return myEvaluator is null ? new NCollection_Array1<Geom2d_Curve.ResD3>() : myEvaluator.EvaluateGridD3(theParams);
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; 1-based, or empty.</summary>
    public NCollection_Array1<Vector2D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        return myEvaluator is null ? new NCollection_Array1<Vector2D>() : myEvaluator.EvaluateGridDN(theParams, theN);
    }
}
