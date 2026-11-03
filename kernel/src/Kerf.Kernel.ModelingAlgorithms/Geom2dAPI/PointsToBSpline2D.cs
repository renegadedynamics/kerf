// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_PointsToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_PointsToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Approximates a set of 2D points by a B-spline curve: by least squares (AppDef_BSplineCompute) with a
/// parametrisation type or given parameters, as the graph of a function sampled at regular X values, or by the
/// variational method (AppDef_Variational) with criterion weights.
/// </summary>
/// <remarks>
/// A working object (decision 21), as GeomAPI_PointsToBSpline. OCCT's implicit conversion to the curve is an
/// explicit conversion (decision 28). <see cref="Curve"/> returns the computed curve itself, as OCCT returns its
/// handle, frozen (decision 243).
/// </remarks>
public class PointsToBSpline2D
{
    private bool _isDone;
    private BSplineCurve2D? _curve;

    private static bool HasMeaningfulSpan(double span)
    {
        double resolution = Constants.Resolution();
        return span * span > resolution * resolution;
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c>.</summary>
    public PointsToBSpline2D()
    {
        _isDone = false;
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public PointsToBSpline2D(NCollection_Array1<Point2D> points, int degMin = 3, int degMax = 8,
                                     Continuity continuity = C2, double tol2D = 1.0e-6)
    {
        _isDone = false;
        Init(points, degMin, degMax, continuity, tol2D);
    }

    /// <summary>
    /// Approximates the graph of a function: the points (X0 + DX * (i - YValues.Lower()), YValues(i)). The curve is
    /// parametrised from X0 to X0 + DX * (YValues.Upper() - YValues.Lower()), with X(t) = t.
    /// </summary>
    public PointsToBSpline2D(NCollection_Array1<double> yValues, double x0, double dx, int degMin = 3,
                                     int degMax = 8, Continuity continuity = C2, double tol2D = 1.0e-6)
    {
        _isDone = false;
        Init(yValues, x0, dx, degMin, degMax, continuity, tol2D);
    }

    /// <summary>Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>.</summary>
    public PointsToBSpline2D(NCollection_Array1<Point2D> points, ParametrizationType parType,
                                     int degMin = 3, int degMax = 8, Continuity continuity = C2,
                                     double tol2D = 1.0e-3)
    {
        _isDone = false;
        Init(points, parType, degMin, degMax, continuity, tol2D);
    }

    /// <summary>Approximates <paramref name="points"/> at the parameters <paramref name="Params"/>.</summary>
    public PointsToBSpline2D(NCollection_Array1<Point2D> points, NCollection_Array1<double> Params,
                                     int degMin = 3, int degMax = 8, Continuity continuity = C2,
                                     double tol2D = 1.0e-3)
    {
        _isDone = false;
        Init(points, Params, degMin, degMax, continuity, tol2D);
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public PointsToBSpline2D(NCollection_Array1<Point2D> points, double w1, double w2, double w3,
                                     int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, w1, w2, w3, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>.</summary>
    public void Init(NCollection_Array1<Point2D> points, ParametrizationType parType, int degMin = 3,
                     int degMax = 8, Continuity continuity = C2, double tol2D = 1.0e-3)
    {
        _isDone = false;
        double tol3D = 0.0; // dummy argument for BSplineCompute.

        int nbit = 2;
        bool useSquares = false;
        if (tol2D <= 1.0e-3)
        {
            useSquares = true;
        }

        BSplineCompute theComputer = new(degMin, degMax, tol3D, tol2D, nbit, true, parType, useSquares);

        SetContinuity(theComputer, continuity);

        theComputer.Perform(new MultiLine(points));

        MultiBSpCurve theCurve = theComputer.Value();

        NCollection_Array1<Point2D> poles = new(1, theCurve.NbPoles());

        theCurve.Curve(1, poles);

        _curve = new BSplineCurve2D(poles, theCurve.Knots(), theCurve.Multiplicities(), theCurve.Degree());
        _isDone = true;
    }

    /// <summary>
    /// Approximates the graph of a function: the points (X0 + DX * (i - YValues.Lower()), YValues(i)), with X(t) = t.
    /// </summary>
    public void Init(NCollection_Array1<double> yValues, double x0, double dx, int degMin = 3, int degMax = 8,
                     Continuity continuity = C2, double tol2D = 1.0e-6)
    {
        _isDone = false;
        // first approximate the Y values (with dummy 0 as X values)

        double tol3D = 0.0; // dummy argument for BSplineCompute.
        NCollection_Array1<Point2D> points = new(yValues.Lower(), yValues.Upper());
        math_Vector param = new(yValues.Lower(), yValues.Upper());
        double length = dx * (yValues.Upper() - yValues.Lower());
        double denominator = x0 + length;
        int paramSpan = yValues.Upper() - yValues.Lower();
        int i;

        for (i = yValues.Lower(); i <= yValues.Upper(); i++)
        {
            if (HasMeaningfulSpan(denominator))
            {
                param[i] = (x0 + (i - yValues.Lower()) * dx) / denominator;
            }
            else
            {
                param[i] = (paramSpan > 0) ? (double)(i - yValues.Lower()) / (double)paramSpan : 0.0;
            }
            points[i].SetCoord(0.0, yValues[i]);
        }

        BSplineCompute theComputer = new(param, degMin, degMax, tol3D, tol2D, 0, true, true);

        SetContinuity(theComputer, continuity);

        theComputer.Perform(new MultiLine(points));

        MultiBSpCurve theCurve = theComputer.Value();

        int degree = theCurve.Degree();
        NCollection_Array1<Point2D> poles = new(1, theCurve.NbPoles());
        int nk = theCurve.Knots().Length();
        NCollection_Array1<double> knots = new(1, nk);
        NCollection_Array1<int> mults = new(1, nk);

        theCurve.Curve(1, poles);

        // compute X values for the poles
        NCollection_Array1<double> xPoles = new(1, poles.Upper());

        // start with a line
        NCollection_Array1<double> tempPoles = new(1, 2);
        NCollection_Array1<double> tempKnots = new(1, 2);
        NCollection_Array1<int> tempMults = new(1, 2);
        tempMults.Init(2);
        tempPoles[1] = x0;
        tempPoles[2] = x0 + length;
        tempKnots[1] = 0.0;
        tempKnots[2] = 1.0;

        // increase the Degree
        NCollection_Array1<double> newTempPoles = new(1, degree + 1);
        NCollection_Array1<double> newTempKnots = new(1, 2);
        NCollection_Array1<int> newTempMults = new(1, 2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(1, degree, false, 1, tempPoles, tempKnots, tempMults, newTempPoles, newTempKnots,
                                newTempMults);

        // insert the Knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(degree, false, 1, newTempPoles, newTempKnots, newTempMults, theCurve.Knots(),
                             theCurve.Multiplicities(), xPoles, knots, mults, Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk; i++)
        {
            knots[i] = x0 + length * knots[i];
        }

        // set the Poles
        for (i = 1; i <= poles.Upper(); i++)
        {
            poles[i].SetX(xPoles[i]);
        }

        _curve = new BSplineCurve2D(poles, knots, mults, degree);
        _isDone = true;
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array1<Point2D> points, int degMin = 3, int degMax = 8,
                     Continuity continuity = C2, double tol2D = 1.0e-6)
    {
        _isDone = false;
        Init(points, ChordLength, degMin, degMax, continuity, tol2D);
    }

    /// <summary>Approximates <paramref name="points"/> at the parameters <paramref name="Params"/>.</summary>
    public void Init(NCollection_Array1<Point2D> points, NCollection_Array1<double> Params, int degMin = 3,
                     int degMax = 8, Continuity continuity = C2, double tol2D = 1.0e-3)
    {
        _isDone = false;
        if (Params.Length() != points.Length())
        {
            throw new OutOfRangeException("Geom2dAPI_PointsToBSpline::Init() - invalid input");
        }

        double tol3D = 0.0; // dummy argument for BSplineCompute.
        int nbp = Params.Length();
        math_Vector theParams = new(1, nbp);
        theParams[1] = 0.0;
        theParams[nbp] = 1.0;

        double uf = Params[Params.Lower()];
        double ul = Params[Params.Upper()] - uf;
        if (!HasMeaningfulSpan(ul))
        {
            return;
        }

        for (int i = 2; i < nbp; i++)
        {
            // OCCT reads Params(i), right only for parameters indexed from 1
            theParams[i] = (Params[i] - uf) / ul;
        }

        BSplineCompute theComputer = new(degMin, degMax, tol3D, tol2D, 0, true, IsoParametric, true);

        theComputer.SetParameters(theParams);

        SetContinuity(theComputer, continuity);

        theComputer.Perform(new MultiLine(points));

        MultiBSpCurve theCurve = theComputer.Value();

        NCollection_Array1<Point2D> poles = new(1, theCurve.NbPoles());

        theCurve.Curve(1, poles);

        // unlike GeomAPI_PointsToBSpline, OCCT keeps the knots on [0, 1] here (no Reparametrize to Params' range)
        _curve = new BSplineCurve2D(poles, theCurve.Knots(), theCurve.Multiplicities(), theCurve.Degree());
        _isDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public void Init(NCollection_Array1<Point2D> points, double w1, double w2, double w3, int degMax = 8,
                     Continuity continuity = C2, double tol2D = 1.0e-3)
    {
        _isDone = false;
        int nbPoint = points.Length(), i;

        int nbit = 2;
        if (tol2D <= 1.0e-3)
        {
            nbit = 0;
        }

        // Variational algo
        MultiLine multL = new(nbPoint);
        for (i = 1; i <= nbPoint; ++i)
        {
            MultiPointConstraint mpc = new(0, 1);
            mpc.SetPoint2d(1, points.Value(points.Lower() + i - 1));
            multL.SetValue(i, mpc);
        }

        NCollection_HArray1<ConstraintCouple> taBofCC = new(1, nbPoint);
        Constraint constraint = global::Kerf.Engine.Approximation.Constraint.NoConstraint;

        for (i = 1; i <= nbPoint; ++i)
        {
            ConstraintCouple acc = new(i, constraint);
            taBofCC.SetValue(i, acc);
        }

        Variational variation = new(multL, 1, nbPoint, taBofCC);

        //===================================
        int theMaxSegments = 1000;
        bool theWithMinMax = false;
        //===================================

        variation.SetMaxDegree(degMax);
        variation.SetContinuity(continuity);
        variation.SetMaxSegment(theMaxSegments);

        variation.SetTolerance(tol2D);
        variation.SetWithMinMax(theWithMinMax);
        variation.SetNbIterations(nbit);

        variation.SetCriteriumWeight(w1, w2, w3);

        if (!variation.IsCreated)
        {
            return;
        }

        if (variation.IsOverConstrained)
        {
            return;
        }

        try
        {
            variation.Approximate();
        }
        catch (FailureException)
        {
            return;
        }

        if (!variation.IsDone)
        {
            return;
        }

        MultiBSpCurve theCurve = variation.Value();

        NCollection_Array1<Point2D> poles = new(1, theCurve.NbPoles());

        theCurve.Curve(1, poles);

        _curve = new BSplineCurve2D(poles, theCurve.Knots(), theCurve.Multiplicities(), theCurve.Degree());

        _isDone = true;
    }

    // The switch on the continuity that OCCT writes out in each least-squares Init.
    private static void SetContinuity(BSplineCompute theComputer, Continuity continuity)
    {
        switch (continuity)
        {
            case C0:
                theComputer.SetContinuity(0);
                break;

            case G1:
            case C1:
                theComputer.SetContinuity(1);
                break;

            case G2:
            case C2:
                theComputer.SetContinuity(2);
                break;

            default:
                theComputer.SetContinuity(3);
                break;
        }
    }

    /// <summary>The approximating curve (frozen). Throws StdFail_NotDone if the approximation failed.</summary>
    public BSplineCurve2D Curve()
    {
        if (!_isDone)
        {
            throw new NotDoneException(" ");
        }
        _curve!.Freeze();
        return _curve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom2d_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator BSplineCurve2D?(PointsToBSpline2D approx)
    {
        approx._curve?.Freeze();
        return approx._curve;
    }

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone => _isDone;
}
