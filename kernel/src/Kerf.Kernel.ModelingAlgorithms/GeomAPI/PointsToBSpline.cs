// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Approximates a set of points by a B-spline curve: by least squares (AppDef_BSplineCompute) with a
/// parametrisation type or given parameters, or by the variational method (AppDef_Variational) with criterion
/// weights.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's implicit conversion to the curve is an explicit conversion (decision 28).
/// <see cref="Curve"/> returns the computed curve itself, as OCCT returns its handle, frozen (decision 243).
/// </remarks>
internal class PointsToBSpline
{
    private bool _isDone;
    private BSplineCurve3D? _curve;

    private static bool HasMeaningfulSpan(double span)
    {
        double resolution = Constants.Resolution();
        return span * span > resolution * resolution;
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c>.</summary>
    public PointsToBSpline()
    {
        _isDone = false;
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public PointsToBSpline(NCollection_Array1<Point3D> points, int degMin = 3, int degMax = 8,
                                   Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, degMin, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>.</summary>
    public PointsToBSpline(NCollection_Array1<Point3D> points, global::Kerf.Geometry.ParametrizationType parType,
                                   int degMin = 3, int degMax = 8, Continuity continuity = C2,
                                   double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, parType, degMin, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> at the parameters <paramref name="parameters"/>.</summary>
    public PointsToBSpline(NCollection_Array1<Point3D> points, NCollection_Array1<double> parameters,
                                   int degMin = 3, int degMax = 8, Continuity continuity = C2,
                                   double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, parameters, degMin, degMax, continuity, tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public PointsToBSpline(NCollection_Array1<Point3D> points, double w1, double w2, double w3,
                                   int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, w1, w2, w3, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array1<Point3D> points, int degMin = 3, int degMax = 8,
                     Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        Init(points, ChordLength, degMin, degMax, continuity, tol3D);
    }

    /// <summary>Approximates <paramref name="points"/> with the parametrisation <paramref name="parType"/>.</summary>
    public void Init(NCollection_Array1<Point3D> points, global::Kerf.Geometry.ParametrizationType parType, int degMin = 3,
                     int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        double tol2D = 0.0; // dummy argument for BSplineCompute.
        int nbit = 2;
        bool useSquares = false;
        if (tol3D <= 1.0e-3)
        {
            useSquares = true;
        }

        BSplineCompute theComputer = new(degMin, degMax, tol3D, tol2D, nbit, true, parType, useSquares);

        SetContinuity(theComputer, continuity);

        theComputer.Perform(new MultiLine(points));

        MultiBSpCurve theCurve = theComputer.Value();

        NCollection_Array1<Point3D> poles = new(1, theCurve.NbPoles());

        theCurve.Curve(1, poles);

        _curve = new BSplineCurve3D(poles, theCurve.Knots(), theCurve.Multiplicities(), theCurve.Degree());
        _isDone = true;
    }

    /// <summary>Approximates <paramref name="points"/> at the parameters <paramref name="parameters"/>.</summary>
    public void Init(NCollection_Array1<Point3D> points, NCollection_Array1<double> parameters, int degMin = 3,
                     int degMax = 8, Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        if (parameters.Length() != points.Length())
        {
            throw new OutOfRangeException("GeomAPI_PointsToBSpline::Init() - invalid input");
        }

        double tol2D = 0.0; // dummy argument for BSplineCompute.
        int nbp = parameters.Length();
        math_Vector theParams = new(1, nbp);
        theParams[1] = 0.0;
        theParams[nbp] = 1.0;

        double uf = parameters[parameters.Lower()];
        double ul = parameters[parameters.Upper()] - uf;
        if (!HasMeaningfulSpan(ul))
        {
            return;
        }
        for (int i = 2; i < nbp; i++)
        {
            // OCCT reads Params(i), right only for parameters indexed from 1
            theParams[i] = (parameters[i] - uf) / ul;
        }

        BSplineCompute theComputer = new(degMin, degMax, tol3D, tol2D, 0, true, IsoParametric, true);

        theComputer.SetParameters(theParams);

        SetContinuity(theComputer, continuity);

        theComputer.Perform(new MultiLine(points));

        MultiBSpCurve theCurve = theComputer.Value();

        NCollection_Array1<Point3D> poles = new(1, theCurve.NbPoles());
        NCollection_Array1<double> knots;

        theCurve.Curve(1, poles);
        knots = theCurve.Knots();

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(parameters[parameters.Lower()], parameters[parameters.Upper()], knots);

        _curve = new BSplineCurve3D(poles, knots, theCurve.Multiplicities(), theCurve.Degree());
        _isDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public void Init(NCollection_Array1<Point3D> points, double w1, double w2, double w3, int degMax = 8,
                     Continuity continuity = C2, double tol3D = 1.0e-3)
    {
        _isDone = false;
        int nbPoint = points.Length(), i;

        int nbit = 2;
        if (tol3D <= 1.0e-3)
        {
            nbit = 0;
        }

        // Variational algo
        MultiLine multL = new(nbPoint);
        for (i = 1; i <= nbPoint; ++i)
        {
            MultiPointConstraint mpc = new(1, 0);
            mpc.SetPoint(1, points.Value(points.Lower() + i - 1));
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

        variation.SetTolerance(tol3D);
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
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return;
        }

        if (!variation.IsDone)
        {
            return;
        }

        MultiBSpCurve theCurve = variation.Value();

        NCollection_Array1<Point3D> poles = new(1, theCurve.NbPoles());

        theCurve.Curve(1, poles);

        _curve = new BSplineCurve3D(poles, theCurve.Knots(), theCurve.Multiplicities(), theCurve.Degree());

        _isDone = true;
    }

    // The switch on the continuity that OCCT writes out in both least-squares Init.
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
    public BSplineCurve3D Curve()
    {
        if (!_isDone)
        {
            throw new NotDoneException("GeomAPI_PointsToBSpline::Curve ");
        }
        _curve!.Freeze();
        return _curve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator BSplineCurve3D?(PointsToBSpline approx)
    {
        approx._curve?.Freeze();
        return approx._curve;
    }

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone => _isDone;
}
