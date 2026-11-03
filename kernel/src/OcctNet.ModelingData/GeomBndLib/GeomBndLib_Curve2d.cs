// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve2d.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Bounding box of a 2D curve: detects the curve type once and dispatches to the evaluator of that type
/// (GeomBndLib_Line2d, _Circle2d, ..., _OtherCurve2d).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom2d_Curve, it owns a
/// Geom2dAdaptor_Curve for a trimmed curve or an unknown type. OCCT's <c>std::variant</c> of evaluators is an
/// interface field, null for <c>std::monostate</c>, as in <see cref="GeomBndLib_Curve"/>.
/// </remarks>
public sealed class GeomBndLib_Curve2d
{
    /// <summary>The members of the per-type evaluators that the dispatchers call through OCCT's <c>std::visit</c>.</summary>
    internal interface ICurve2dEvaluator
    {
        Box2d Box(double theTol);

        Box2d Box(double theU1, double theU2, double theTol);

        Box2d BoxOptimal(double theU1, double theU2, double theTol);

        /// <summary>The evaluator's <c>Geometry()</c> as a Geom2d_Curve; null for GeomBndLib_OtherCurve2d, which has none.</summary>
        Geom2d_Curve? CurveGeometry();
    }

    private readonly Adaptor2d_Curve2d? myAdaptorRef;
    private readonly Adaptor2d_Curve2d? myAdaptorOwned;
    private readonly ICurve2dEvaluator? myEvaluator;
    private readonly CurveKind myCurveType;

    /// <summary>Construct from an adaptor curve.</summary>
    public GeomBndLib_Curve2d(Adaptor2d_Curve2d theCurve)
    {
        myCurveType = theCurve.GetType();
        myAdaptorRef = theCurve;

        switch (myCurveType)
        {
            case Line:
            {
                Geom2d_Line aLine = new(theCurve.Line());
                myEvaluator = new GeomBndLib_Line2d(aLine);
                break;
            }
            case Circle:
            {
                Geom2d_Circle aCircle = new(theCurve.Circle());
                myEvaluator = new GeomBndLib_Circle2d(aCircle);
                break;
            }
            case Ellipse:
            {
                Geom2d_Ellipse anEllipse = new(theCurve.Ellipse());
                myEvaluator = new GeomBndLib_Ellipse2d(anEllipse);
                break;
            }
            case Hyperbola:
            {
                Geom2d_Hyperbola aHyp = new(theCurve.Hyperbola());
                myEvaluator = new GeomBndLib_Hyperbola2d(aHyp);
                break;
            }
            case Parabola:
            {
                Geom2d_Parabola aPar = new(theCurve.Parabola());
                myEvaluator = new GeomBndLib_Parabola2d(aPar);
                break;
            }
            case BezierCurve:
            {
                myEvaluator = new GeomBndLib_BezierCurve2d(theCurve.Bezier());
                break;
            }
            case BSplineCurve:
            {
                myEvaluator = new GeomBndLib_BSplineCurve2d(theCurve.BSpline());
                break;
            }
            case OffsetCurve:
            {
                if (myAdaptorRef is Geom2dAdaptor_Curve aGA)
                {
                    if (aGA.Curve() is Geom2d_OffsetCurve anOff)
                    {
                        myEvaluator = new GeomBndLib_OffsetCurve2d(anOff);
                        break;
                    }
                }
                myCurveType = OtherCurve;
                myEvaluator = new GeomBndLib_OtherCurve2d(myAdaptorRef);
                break;
            }
            default:
            {
                myCurveType = OtherCurve;
                myEvaluator = new GeomBndLib_OtherCurve2d(myAdaptorRef);
                break;
            }
        }
    }

    /// <summary>Construct from a Geom2d_Curve.</summary>
    public GeomBndLib_Curve2d(Geom2d_Curve theCurve)
    {
        myCurveType = OtherCurve;

        Geom2d_Curve aCurveForDetection = theCurve;
        if (theCurve is Geom2d_TrimmedCurve aTrim)
        {
            myAdaptorOwned = new Geom2dAdaptor_Curve(theCurve);
            aCurveForDetection = aTrim.BasisCurve();
        }

        // OCCT's lambda tryEmplaceEvaluator
        switch (aCurveForDetection)
        {
            case Geom2d_Line aLine:
                myCurveType = Line;
                myEvaluator = new GeomBndLib_Line2d(aLine);
                return;
            case Geom2d_Circle aCircle:
                myCurveType = Circle;
                myEvaluator = new GeomBndLib_Circle2d(aCircle);
                return;
            case Geom2d_Ellipse anEllipse:
                myCurveType = Ellipse;
                myEvaluator = new GeomBndLib_Ellipse2d(anEllipse);
                return;
            case Geom2d_Hyperbola aHyp:
                myCurveType = Hyperbola;
                myEvaluator = new GeomBndLib_Hyperbola2d(aHyp);
                return;
            case Geom2d_Parabola aPar:
                myCurveType = Parabola;
                myEvaluator = new GeomBndLib_Parabola2d(aPar);
                return;
            case Geom2d_BezierCurve aBez:
                myCurveType = BezierCurve;
                myEvaluator = new GeomBndLib_BezierCurve2d(aBez);
                return;
            case Geom2d_BSplineCurve aBSpl:
                myCurveType = BSplineCurve;
                myEvaluator = new GeomBndLib_BSplineCurve2d(aBSpl);
                return;
            case Geom2d_OffsetCurve anOff:
                myCurveType = OffsetCurve;
                myEvaluator = new GeomBndLib_OffsetCurve2d(anOff);
                return;
        }

        myAdaptorOwned ??= new Geom2dAdaptor_Curve(theCurve);
        myCurveType = OtherCurve;
        myEvaluator = new GeomBndLib_OtherCurve2d(myAdaptorOwned);
    }

    /// <summary>Return detected curve type.</summary>
    public new CurveKind GetType() => myCurveType;

    private Adaptor2d_Curve2d? adaptor() => myAdaptorRef ?? myAdaptorOwned;

    private double effectiveU1()
    {
        Adaptor2d_Curve2d? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.FirstParameter() : 0.0;
    }

    private double effectiveU2()
    {
        Adaptor2d_Curve2d? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.LastParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Box2d Box(double theTol)
    {
        if (adaptor() is not null)
        {
            return Box(effectiveU1(), effectiveU2(), theTol);
        }

        return myEvaluator is null ? new Box2d() : myEvaluator.Box(theTol);
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Box2d Box(double theU1, double theU2, double theTol)
        => myEvaluator is null ? new Box2d() : myEvaluator.Box(theU1, theU2, theTol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Box2d BoxOptimal(double theTol)
    {
        if (adaptor() is not null)
        {
            return BoxOptimal(effectiveU1(), effectiveU2(), theTol);
        }

        switch (myEvaluator)
        {
            case null:
                return new Box2d();
            case GeomBndLib_OtherCurve2d anOther:
                return anOther.BoxOptimal(theTol);
            default:
            {
                Geom2d_Curve aGeom = myEvaluator.CurveGeometry()!;
                return myEvaluator.BoxOptimal(aGeom.FirstParameter(), aGeom.LastParameter(), theTol);
            }
        }
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2].</summary>
    public Box2d BoxOptimal(double theU1, double theU2, double theTol)
        => myEvaluator is null ? new Box2d() : myEvaluator.BoxOptimal(theU1, theU2, theTol);

    /// <summary>Add bounding box for full curve.</summary>
    public void Add(double theTol, ref Box2d theBox) => theBox.Add(Box(theTol));

    /// <summary>Add bounding box for arc [theU1, theU2].</summary>
    public void Add(double theU1, double theU2, double theTol, ref Box2d theBox) => theBox.Add(Box(theU1, theU2, theTol));

    /// <summary>Add precise bounding box for full curve.</summary>
    public void AddOptimal(double theTol, ref Box2d theBox) => theBox.Add(BoxOptimal(theTol));

    /// <summary>Add precise bounding box for arc [theU1, theU2].</summary>
    public void AddOptimal(double theU1, double theU2, double theTol, ref Box2d theBox)
        => theBox.Add(BoxOptimal(theU1, theU2, theTol));
}
