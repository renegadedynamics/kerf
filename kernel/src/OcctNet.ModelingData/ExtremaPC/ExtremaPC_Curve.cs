// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// Point-curve extrema dispatcher: picks the specialised evaluator for the curve type (line, circle, ellipse,
/// hyperbola, parabola, Bezier, B-spline, offset, other) at construction and delegates every query to it.
/// </summary>
/// <remarks>
/// OCCT's <c>std::variant</c> of evaluators is an <see cref="ExtremaPC.ICurveEvaluator"/> field, null for
/// <c>std::monostate</c> (decision 236's pattern). The adaptor constructors keep the caller's adaptor where the
/// evaluator needs it (offset and other curves), as OCCT keeps a pointer: it must outlive this object. A
/// transformed B-spline or Bezier curve is evaluated untransformed, with the query point transformed back and the
/// results forward. <c>Perform</c> returns the evaluator's reusable result (or a shared "not done" result when
/// uninitialised), which the next call overwrites.
/// </remarks>
public sealed class ExtremaPC_Curve
{
    // Static result for uninitialized evaluator case
    private static readonly ExtremaPC.Result THE_NOT_DONE_RESULT = new();

    private ExtremaPC.ICurveEvaluator? myEvaluator;         // Specialized evaluator
    private Adaptor3d_Curve? myAdaptorRef;                  // The caller's adaptor (non-owning in OCCT)
    private Adaptor3d_Curve? myAdaptorOwned;                // Owned adaptor for lifetime management
    private gp_Trsf myTrsf = new();                         // Forward transform (identity = no post-processing)
    private readonly ExtremaPC.Result myTransformResult = new(); // Result buffer for transform post-processing

    /// <summary>Constructor from a GeomAdaptor_Curve, on its parameter range.</summary>
    public ExtremaPC_Curve(GeomAdaptor_Curve theCurve)
    {
        MathUtils.Domain1D aDomain = new(theCurve.FirstParameter(), theCurve.LastParameter());
        initFromAdaptor(theCurve, aDomain);
    }

    /// <summary>Constructor from a GeomAdaptor_Curve with parameter bounds.</summary>
    public ExtremaPC_Curve(GeomAdaptor_Curve theCurve, double theUMin, double theUMax)
    {
        MathUtils.Domain1D aDomain = new(theUMin, theUMax);
        initFromAdaptor(theCurve, aDomain);
    }

    /// <summary>Constructor from a GeomAdaptor_TransformedCurve, on its parameter range.</summary>
    public ExtremaPC_Curve(GeomAdaptor_TransformedCurve theCurve)
    {
        initFromTransformedCurve(theCurve, null);
    }

    /// <summary>Constructor from a GeomAdaptor_TransformedCurve with parameter bounds.</summary>
    public ExtremaPC_Curve(GeomAdaptor_TransformedCurve theCurve, double theUMin, double theUMax)
    {
        MathUtils.Domain1D aDomain = new(theUMin, theUMax);
        initFromTransformedCurve(theCurve, aDomain);
    }

    /// <summary>Constructor from a Geom_Curve (a trimmed curve uses its bounds; others are unbounded).</summary>
    public ExtremaPC_Curve(Geom_Curve? theCurve)
    {
        if (theCurve is null)
        {
            return;
        }

        // Check for trimmed curve - if so, use bounds
        if (theCurve is Geom_TrimmedCurve aTrimmed)
        {
            MathUtils.Domain1D aDomain = new(aTrimmed.FirstParameter(), aTrimmed.LastParameter());
            initFromGeomCurve(aTrimmed.BasisCurve(), aDomain);
            return;
        }

        // Initialize based on curve type - without setting domain (unbounded)
        initFromGeomCurve(theCurve, null);
    }

    /// <summary>Constructor from a Geom_Curve with parameter bounds (intersected with a trimmed curve's).</summary>
    public ExtremaPC_Curve(Geom_Curve? theCurve, double theUMin, double theUMax)
    {
        if (theCurve is null)
        {
            return;
        }

        // Get base curve and effective bounds
        Geom_Curve aBaseCurve = theCurve;
        double aEffectiveUMin = theUMin;
        double aEffectiveUMax = theUMax;

        // For trimmed curve, intersect input bounds with trimmed bounds
        if (theCurve is Geom_TrimmedCurve aTrimmed)
        {
            aBaseCurve = aTrimmed.BasisCurve();
            // Intersect bounds: take max of mins, min of maxs
            aEffectiveUMin = Max(theUMin, aTrimmed.FirstParameter());
            aEffectiveUMax = Min(theUMax, aTrimmed.LastParameter());
        }

        // Use common helper for curve type detection and evaluator creation
        MathUtils.Domain1D aDomain2 = new(aEffectiveUMin, aEffectiveUMax);
        initFromGeomCurve(aBaseCurve, aDomain2);
    }

    /// <summary>Computes the interior extrema between point P and the curve.</summary>
    public ExtremaPC.Result Perform(in gp_Pnt theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        ExtremaPC.Result aResultPtr = THE_NOT_DONE_RESULT;

        if (myTrsf.Form() != gp_Identity)
        {
            // Inverse-transform query point and delegate to untransformed evaluator
            gp_Pnt aPInv = theP.Transformed(myTrsf.Inverted());
            if (myEvaluator is not null)
            {
                aResultPtr = myEvaluator.Perform(aPInv, theTol, theMode);
            }
            return postProcessTransform(aResultPtr);
        }

        if (myEvaluator is not null)
        {
            aResultPtr = myEvaluator.Perform(theP, theTol, theMode);
        }
        return aResultPtr;
    }

    /// <summary>Computes the extrema between point P and the curve including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in gp_Pnt theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        ExtremaPC.Result aResultPtr = THE_NOT_DONE_RESULT;

        if (myTrsf.Form() != gp_Identity)
        {
            // Inverse-transform query point and delegate to untransformed evaluator
            gp_Pnt aPInv = theP.Transformed(myTrsf.Inverted());
            if (myEvaluator is not null)
            {
                aResultPtr = myEvaluator.PerformWithEndpoints(aPInv, theTol, theMode);
            }
            return postProcessTransform(aResultPtr);
        }

        if (myEvaluator is not null)
        {
            aResultPtr = myEvaluator.PerformWithEndpoints(theP, theTol, theMode);
        }
        return aResultPtr;
    }

    /// <summary>Returns true if an evaluator was created.</summary>
    public bool IsInitialized() => myEvaluator is not null;

    private void initFromAdaptor(GeomAdaptor_Curve theCurve, in MathUtils.Domain1D theDomain)
    {
        myAdaptorRef = theCurve;

        GeomAbs_CurveType aCurveType = theCurve.GetType();

        switch (aCurveType)
        {
            case GeomAbs_Line:
                myEvaluator = new ExtremaPC_Line(theCurve.Line(), theDomain);
                break;

            case GeomAbs_Circle:
                myEvaluator = new ExtremaPC_Circle(theCurve.Circle(), theDomain);
                break;

            case GeomAbs_Ellipse:
                myEvaluator = new ExtremaPC_Ellipse(theCurve.Ellipse(), theDomain);
                break;

            case GeomAbs_Hyperbola:
                myEvaluator = new ExtremaPC_Hyperbola(theCurve.Hyperbola(), theDomain);
                break;

            case GeomAbs_Parabola:
                myEvaluator = new ExtremaPC_Parabola(theCurve.Parabola(), theDomain);
                break;

            case GeomAbs_BezierCurve:
                myEvaluator = new ExtremaPC_BezierCurve(theCurve.Bezier(), theDomain);
                break;

            case GeomAbs_BSplineCurve:
                myEvaluator = new ExtremaPC_BSplineCurve(theCurve.BSpline(), theDomain);
                break;

            case GeomAbs_OffsetCurve:
                myEvaluator = new ExtremaPC_OffsetCurve(theCurve, theDomain);
                break;

            default:
                myEvaluator = new ExtremaPC_OtherCurve(theCurve, theDomain);
                break;
        }
    }

    private void initFromTransformedCurve(GeomAdaptor_TransformedCurve theCurve, MathUtils.Domain1D? theDomain)
    {
        // Identity transform on a 3D curve: delegate to the faster GeomAdaptor_Curve path
        // which avoids transform overhead in evaluators.
        // Skipped for CurveOnSurface since its myCurve member is empty.
        if (theCurve.Is3DCurve() && theCurve.Trsf().Form() == gp_Identity)
        {
            GeomAdaptor_Curve aCurve = theCurve.Curve();
            MathUtils.Domain1D aDomain0 =
              theDomain.HasValue ? theDomain.Value
                                 : new MathUtils.Domain1D(aCurve.FirstParameter(), aCurve.LastParameter());
            initFromAdaptor(aCurve, aDomain0);
            return;
        }

        MathUtils.Domain1D aDomain =
          theDomain.HasValue
            ? theDomain.Value
            : new MathUtils.Domain1D(theCurve.FirstParameter(), theCurve.LastParameter());

        GeomAbs_CurveType aCurveType = theCurve.GetType();

        switch (aCurveType)
        {
            // Elementary curves: Line(), Circle() etc. return already-transformed primitives
            case GeomAbs_Line:
                myEvaluator = new ExtremaPC_Line(theCurve.Line(), aDomain);
                break;

            case GeomAbs_Circle:
                myEvaluator = new ExtremaPC_Circle(theCurve.Circle(), aDomain);
                break;

            case GeomAbs_Ellipse:
                myEvaluator = new ExtremaPC_Ellipse(theCurve.Ellipse(), aDomain);
                break;

            case GeomAbs_Hyperbola:
                myEvaluator = new ExtremaPC_Hyperbola(theCurve.Hyperbola(), aDomain);
                break;

            case GeomAbs_Parabola:
                myEvaluator = new ExtremaPC_Parabola(theCurve.Parabola(), aDomain);
                break;

            // BSpline/Bezier: use untransformed handle, set up inverse-transform path.
            // For CurveOnSurface the underlying myCurve is empty, so fall through to OtherCurve.
            case GeomAbs_BSplineCurve:
                if (theCurve.Is3DCurve())
                {
                    myEvaluator = new ExtremaPC_BSplineCurve(theCurve.Curve().BSpline(), aDomain);
                    myTrsf = theCurve.Trsf();
                    break;
                }
                myAdaptorRef = theCurve;
                myEvaluator = new ExtremaPC_OtherCurve(myAdaptorRef, aDomain);
                break;

            case GeomAbs_BezierCurve:
                if (theCurve.Is3DCurve())
                {
                    myEvaluator = new ExtremaPC_BezierCurve(theCurve.Curve().Bezier(), aDomain);
                    myTrsf = theCurve.Trsf();
                    break;
                }
                myAdaptorRef = theCurve;
                myEvaluator = new ExtremaPC_OtherCurve(myAdaptorRef, aDomain);
                break;

            // Offset/Other: use non-owning pointer to caller's TransformedCurve
            case GeomAbs_OffsetCurve:
                myAdaptorRef = theCurve;
                myEvaluator = new ExtremaPC_OffsetCurve(myAdaptorRef, aDomain);
                break;

            default:
                myAdaptorRef = theCurve;
                myEvaluator = new ExtremaPC_OtherCurve(myAdaptorRef, aDomain);
                break;
        }
    }

    private void initFromGeomCurve(Geom_Curve theCurve, MathUtils.Domain1D? theDomain)
    {
        // Try specific curve types for direct initialization
        if (theCurve is Geom_Line aLine)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_Line(aLine.Lin(), theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_Line(aLine.Lin());
            }
            return;
        }

        if (theCurve is Geom_Circle aCircle)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_Circle(aCircle.Circ(), theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_Circle(aCircle.Circ());
            }
            return;
        }

        if (theCurve is Geom_Ellipse anEllipse)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_Ellipse(anEllipse.Elips(), theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_Ellipse(anEllipse.Elips());
            }
            return;
        }

        if (theCurve is Geom_Hyperbola aHyperbola)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_Hyperbola(aHyperbola.Hypr(), theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_Hyperbola(aHyperbola.Hypr());
            }
            return;
        }

        if (theCurve is Geom_Parabola aParabola)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_Parabola(aParabola.Parab(), theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_Parabola(aParabola.Parab());
            }
            return;
        }

        if (theCurve is Geom_BezierCurve aBezier)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_BezierCurve(aBezier, theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_BezierCurve(aBezier);
            }
            return;
        }

        if (theCurve is Geom_BSplineCurve aBSpline)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_BSplineCurve(aBSpline, theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_BSplineCurve(aBSpline);
            }
            return;
        }

        // For offset and other curves, store adaptor and use it
        // Use domain bounds if set, otherwise use natural curve bounds
        if (theDomain.HasValue)
        {
            myAdaptorOwned = new GeomAdaptor_Curve(theCurve, theDomain.Value.Min, theDomain.Value.Max);
        }
        else
        {
            myAdaptorOwned = new GeomAdaptor_Curve(theCurve);
        }
        GeomAbs_CurveType aCurveType = myAdaptorOwned.GetType();

        if (aCurveType == GeomAbs_OffsetCurve)
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_OffsetCurve(myAdaptorOwned, theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_OffsetCurve(myAdaptorOwned);
            }
        }
        else
        {
            if (theDomain.HasValue)
            {
                myEvaluator = new ExtremaPC_OtherCurve(myAdaptorOwned, theDomain.Value);
            }
            else
            {
                myEvaluator = new ExtremaPC_OtherCurve(myAdaptorOwned);
            }
        }
    }

    private ExtremaPC.Result postProcessTransform(ExtremaPC.Result theSrc)
    {
        double aSF = myTrsf.ScaleFactor();
        double aSFSq = aSF * aSF;

        myTransformResult.Clear();
        myTransformResult.Status = theSrc.Status;
        myTransformResult.InfiniteSquareDistance = theSrc.InfiniteSquareDistance * aSFSq;
        for (int i = 0; i < theSrc.Extrema.Length(); ++i)
        {
            ExtremaPC.ExtremumResult anExt = new();
            anExt.Parameter = theSrc[i].Parameter;
            anExt.Point = theSrc[i].Point.Transformed(myTrsf);
            anExt.SquareDistance = theSrc[i].SquareDistance * aSFSq;
            anExt.IsMinimum = theSrc[i].IsMinimum;
            myTransformResult.Extrema.Append(anExt);
        }
        return myTransformResult;
    }
}
