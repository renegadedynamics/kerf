// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// Bounding box of a 3D curve: detects the curve type once and dispatches to the evaluator of that type
/// (GeomBndLib_Line, _Circle, ..., _OtherCurve).
/// </summary>
/// <remarks>
/// Built from an adaptor, it keeps a reference to it (as OCCT does); built from a Geom_Curve, it owns a
/// GeomAdaptor_Curve for a trimmed curve or an unknown type. OCCT's <c>std::variant</c> of evaluators is an
/// interface field, null for <c>std::monostate</c> (decision 236's pattern).
/// </remarks>
public sealed class GeomBndLib_Curve
{
    /// <summary>The members of the per-type evaluators that the dispatchers call through OCCT's <c>std::visit</c>.</summary>
    internal interface ICurveEvaluator
    {
        Bnd_Box Box(double theTol);

        Bnd_Box Box(double theU1, double theU2, double theTol);

        Bnd_Box BoxOptimal(double theU1, double theU2, double theTol);

        /// <summary>The evaluator's <c>Geometry()</c> as a Geom_Curve; null for GeomBndLib_OtherCurve, which has none.</summary>
        Geom_Curve? CurveGeometry();
    }

    private readonly Adaptor3d_Curve? myAdaptorRef;
    private readonly Adaptor3d_Curve? myAdaptorOwned;
    private readonly ICurveEvaluator? myEvaluator;
    private readonly GeomAbs_CurveType myCurveType;

    /// <summary>Construct from an adaptor curve.</summary>
    public GeomBndLib_Curve(Adaptor3d_Curve theCurve)
    {
        myCurveType = theCurve.GetType();
        myAdaptorRef = theCurve;

        switch (myCurveType)
        {
            case GeomAbs_Line:
            {
                Geom_Line aLine = new(theCurve.Line());
                myEvaluator = new GeomBndLib_Line(aLine);
                break;
            }
            case GeomAbs_Circle:
            {
                Geom_Circle aCircle = new(theCurve.Circle());
                myEvaluator = new GeomBndLib_Circle(aCircle);
                break;
            }
            case GeomAbs_Ellipse:
            {
                Geom_Ellipse anEllipse = new(theCurve.Ellipse());
                myEvaluator = new GeomBndLib_Ellipse(anEllipse);
                break;
            }
            case GeomAbs_Hyperbola:
            {
                Geom_Hyperbola aHyp = new(theCurve.Hyperbola());
                myEvaluator = new GeomBndLib_Hyperbola(aHyp);
                break;
            }
            case GeomAbs_Parabola:
            {
                Geom_Parabola aPar = new(theCurve.Parabola());
                myEvaluator = new GeomBndLib_Parabola(aPar);
                break;
            }
            case GeomAbs_BezierCurve:
            {
                myEvaluator = new GeomBndLib_BezierCurve(theCurve.Bezier());
                break;
            }
            case GeomAbs_BSplineCurve:
            {
                myEvaluator = new GeomBndLib_BSplineCurve(theCurve.BSpline());
                break;
            }
            case GeomAbs_OffsetCurve:
            {
                if (myAdaptorRef is GeomAdaptor_Curve aGA)
                {
                    if (aGA.Curve() is Geom_OffsetCurve anOff)
                    {
                        myEvaluator = new GeomBndLib_OffsetCurve(anOff);
                        break;
                    }
                }
                myCurveType = GeomAbs_OtherCurve;
                myEvaluator = new GeomBndLib_OtherCurve(myAdaptorRef);
                break;
            }
            default:
            {
                myCurveType = GeomAbs_OtherCurve;
                myEvaluator = new GeomBndLib_OtherCurve(myAdaptorRef);
                break;
            }
        }
    }

    /// <summary>Construct from a Geom_Curve.</summary>
    public GeomBndLib_Curve(Geom_Curve theCurve)
    {
        myCurveType = GeomAbs_OtherCurve;

        Geom_Curve aCurveForDetection = theCurve;
        if (theCurve is Geom_TrimmedCurve aTrim)
        {
            myAdaptorOwned = new GeomAdaptor_Curve(theCurve);
            aCurveForDetection = aTrim.BasisCurve();
        }

        // OCCT's lambda tryEmplaceEvaluator
        switch (aCurveForDetection)
        {
            case Geom_Line aLine:
                myCurveType = GeomAbs_Line;
                myEvaluator = new GeomBndLib_Line(aLine);
                return;
            case Geom_Circle aCircle:
                myCurveType = GeomAbs_Circle;
                myEvaluator = new GeomBndLib_Circle(aCircle);
                return;
            case Geom_Ellipse anEllipse:
                myCurveType = GeomAbs_Ellipse;
                myEvaluator = new GeomBndLib_Ellipse(anEllipse);
                return;
            case Geom_Hyperbola aHyp:
                myCurveType = GeomAbs_Hyperbola;
                myEvaluator = new GeomBndLib_Hyperbola(aHyp);
                return;
            case Geom_Parabola aPar:
                myCurveType = GeomAbs_Parabola;
                myEvaluator = new GeomBndLib_Parabola(aPar);
                return;
            case Geom_BezierCurve aBez:
                myCurveType = GeomAbs_BezierCurve;
                myEvaluator = new GeomBndLib_BezierCurve(aBez);
                return;
            case Geom_BSplineCurve aBSpl:
                myCurveType = GeomAbs_BSplineCurve;
                myEvaluator = new GeomBndLib_BSplineCurve(aBSpl);
                return;
            case Geom_OffsetCurve anOff:
                myCurveType = GeomAbs_OffsetCurve;
                myEvaluator = new GeomBndLib_OffsetCurve(anOff);
                return;
        }

        myAdaptorOwned ??= new GeomAdaptor_Curve(theCurve);
        myCurveType = GeomAbs_OtherCurve;
        myEvaluator = new GeomBndLib_OtherCurve(myAdaptorOwned);
    }

    /// <summary>Return detected curve type.</summary>
    public new GeomAbs_CurveType GetType() => myCurveType;

    private Adaptor3d_Curve? adaptor() => myAdaptorRef ?? myAdaptorOwned;

    private double effectiveU1()
    {
        Adaptor3d_Curve? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.FirstParameter() : 0.0;
    }

    private double effectiveU2()
    {
        Adaptor3d_Curve? anAdaptor = adaptor();
        return anAdaptor is not null ? anAdaptor.LastParameter() : 0.0;
    }

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box Box(double theTol)
    {
        if (adaptor() is not null)
        {
            return Box(effectiveU1(), effectiveU2(), theTol);
        }

        return myEvaluator is null ? new Bnd_Box() : myEvaluator.Box(theTol);
    }

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol)
        => myEvaluator is null ? new Bnd_Box() : myEvaluator.Box(theU1, theU2, theTol);

    /// <summary>Compute precise bounding box for full curve.</summary>
    public Bnd_Box BoxOptimal(double theTol)
    {
        if (adaptor() is not null)
        {
            return BoxOptimal(effectiveU1(), effectiveU2(), theTol);
        }

        switch (myEvaluator)
        {
            case null:
                return new Bnd_Box();
            case GeomBndLib_OtherCurve anOther:
                return anOther.BoxOptimal(theTol);
            default:
            {
                Geom_Curve aGeom = myEvaluator.CurveGeometry()!;
                return myEvaluator.BoxOptimal(aGeom.FirstParameter(), aGeom.LastParameter(), theTol);
            }
        }
    }

    /// <summary>Compute precise bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol)
        => myEvaluator is null ? new Bnd_Box() : myEvaluator.BoxOptimal(theU1, theU2, theTol);

    /// <summary>Add bounding box for full curve.</summary>
    public void Add(double theTol, ref Bnd_Box theBox) => theBox.Add(Box(theTol));

    /// <summary>Add bounding box for arc [theU1, theU2].</summary>
    public void Add(double theU1, double theU2, double theTol, ref Bnd_Box theBox) => theBox.Add(Box(theU1, theU2, theTol));

    /// <summary>Add precise bounding box for full curve.</summary>
    public void AddOptimal(double theTol, ref Bnd_Box theBox) => theBox.Add(BoxOptimal(theTol));

    /// <summary>Add precise bounding box for arc [theU1, theU2].</summary>
    public void AddOptimal(double theU1, double theU2, double theTol, ref Bnd_Box theBox)
        => theBox.Add(BoxOptimal(theU1, theU2, theTol));
}
