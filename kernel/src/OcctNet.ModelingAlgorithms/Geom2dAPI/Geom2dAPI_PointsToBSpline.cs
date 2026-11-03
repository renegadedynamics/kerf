// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_PointsToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_PointsToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.Approx_ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Occt;

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
public class Geom2dAPI_PointsToBSpline
{
    private bool myIsDone;
    private Geom2d_BSplineCurve? myCurve;

    private static bool hasMeaningfulSpan(double theSpan)
    {
        double aResolution = Constants.Resolution();
        return theSpan * theSpan > aResolution * aResolution;
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c>.</summary>
    public Geom2dAPI_PointsToBSpline()
    {
        myIsDone = false;
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public Geom2dAPI_PointsToBSpline(NCollection_Array1<Point2D> Points, int DegMin = 3, int DegMax = 8,
                                     Continuity Continuity = C2, double Tol2D = 1.0e-6)
    {
        myIsDone = false;
        Init(Points, DegMin, DegMax, Continuity, Tol2D);
    }

    /// <summary>
    /// Approximates the graph of a function: the points (X0 + DX * (i - YValues.Lower()), YValues(i)). The curve is
    /// parametrised from X0 to X0 + DX * (YValues.Upper() - YValues.Lower()), with X(t) = t.
    /// </summary>
    public Geom2dAPI_PointsToBSpline(NCollection_Array1<double> YValues, double X0, double DX, int DegMin = 3,
                                     int DegMax = 8, Continuity Continuity = C2, double Tol2D = 1.0e-6)
    {
        myIsDone = false;
        Init(YValues, X0, DX, DegMin, DegMax, Continuity, Tol2D);
    }

    /// <summary>Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>.</summary>
    public Geom2dAPI_PointsToBSpline(NCollection_Array1<Point2D> Points, Approx_ParametrizationType ParType,
                                     int DegMin = 3, int DegMax = 8, Continuity Continuity = C2,
                                     double Tol2D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, ParType, DegMin, DegMax, Continuity, Tol2D);
    }

    /// <summary>Approximates <paramref name="Points"/> at the parameters <paramref name="Params"/>.</summary>
    public Geom2dAPI_PointsToBSpline(NCollection_Array1<Point2D> Points, NCollection_Array1<double> Params,
                                     int DegMin = 3, int DegMax = 8, Continuity Continuity = C2,
                                     double Tol2D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, Params, DegMin, DegMax, Continuity, Tol2D);
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public Geom2dAPI_PointsToBSpline(NCollection_Array1<Point2D> Points, double W1, double W2, double W3,
                                     int DegMax = 8, Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, W1, W2, W3, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>.</summary>
    public void Init(NCollection_Array1<Point2D> Points, Approx_ParametrizationType ParType, int DegMin = 3,
                     int DegMax = 8, Continuity Continuity = C2, double Tol2D = 1.0e-3)
    {
        myIsDone = false;
        double Tol3D = 0.0; // dummy argument for BSplineCompute.

        int nbit = 2;
        bool UseSquares = false;
        if (Tol2D <= 1.0e-3)
        {
            UseSquares = true;
        }

        AppDef_BSplineCompute TheComputer = new(DegMin, DegMax, Tol3D, Tol2D, nbit, true, ParType, UseSquares);

        SetContinuity(TheComputer, Continuity);

        TheComputer.Perform(new AppDef_MultiLine(Points));

        AppParCurves_MultiBSpCurve TheCurve = TheComputer.Value();

        NCollection_Array1<Point2D> Poles = new(1, TheCurve.NbPoles());

        TheCurve.Curve(1, Poles);

        myCurve = new Geom2d_BSplineCurve(Poles, TheCurve.Knots(), TheCurve.Multiplicities(), TheCurve.Degree());
        myIsDone = true;
    }

    /// <summary>
    /// Approximates the graph of a function: the points (X0 + DX * (i - YValues.Lower()), YValues(i)), with X(t) = t.
    /// </summary>
    public void Init(NCollection_Array1<double> YValues, double X0, double DX, int DegMin = 3, int DegMax = 8,
                     Continuity Continuity = C2, double Tol2D = 1.0e-6)
    {
        myIsDone = false;
        // first approximate the Y values (with dummy 0 as X values)

        double Tol3D = 0.0; // dummy argument for BSplineCompute.
        NCollection_Array1<Point2D> Points = new(YValues.Lower(), YValues.Upper());
        math_Vector Param = new(YValues.Lower(), YValues.Upper());
        double length = DX * (YValues.Upper() - YValues.Lower());
        double aDenominator = X0 + length;
        int aParamSpan = YValues.Upper() - YValues.Lower();
        int i;

        for (i = YValues.Lower(); i <= YValues.Upper(); i++)
        {
            if (hasMeaningfulSpan(aDenominator))
            {
                Param[i] = (X0 + (i - YValues.Lower()) * DX) / aDenominator;
            }
            else
            {
                Param[i] = (aParamSpan > 0) ? (double)(i - YValues.Lower()) / (double)aParamSpan : 0.0;
            }
            Points[i].SetCoord(0.0, YValues[i]);
        }

        AppDef_BSplineCompute TheComputer = new(Param, DegMin, DegMax, Tol3D, Tol2D, 0, true, true);

        SetContinuity(TheComputer, Continuity);

        TheComputer.Perform(new AppDef_MultiLine(Points));

        AppParCurves_MultiBSpCurve TheCurve = TheComputer.Value();

        int Degree = TheCurve.Degree();
        NCollection_Array1<Point2D> Poles = new(1, TheCurve.NbPoles());
        int nk = TheCurve.Knots().Length();
        NCollection_Array1<double> Knots = new(1, nk);
        NCollection_Array1<int> Mults = new(1, nk);

        TheCurve.Curve(1, Poles);

        // compute X values for the poles
        NCollection_Array1<double> XPoles = new(1, Poles.Upper());

        // start with a line
        NCollection_Array1<double> TempPoles = new(1, 2);
        NCollection_Array1<double> TempKnots = new(1, 2);
        NCollection_Array1<int> TempMults = new(1, 2);
        TempMults.Init(2);
        TempPoles[1] = X0;
        TempPoles[2] = X0 + length;
        TempKnots[1] = 0.0;
        TempKnots[2] = 1.0;

        // increase the Degree
        NCollection_Array1<double> NewTempPoles = new(1, Degree + 1);
        NCollection_Array1<double> NewTempKnots = new(1, 2);
        NCollection_Array1<int> NewTempMults = new(1, 2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(1, Degree, false, 1, TempPoles, TempKnots, TempMults, NewTempPoles, NewTempKnots,
                                NewTempMults);

        // insert the Knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(Degree, false, 1, NewTempPoles, NewTempKnots, NewTempMults, TheCurve.Knots(),
                             TheCurve.Multiplicities(), XPoles, Knots, Mults, Epsilon(1));

        // scale the knots
        for (i = 1; i <= nk; i++)
        {
            Knots[i] = X0 + length * Knots[i];
        }

        // set the Poles
        for (i = 1; i <= Poles.Upper(); i++)
        {
            Poles[i].SetX(XPoles[i]);
        }

        myCurve = new Geom2d_BSplineCurve(Poles, Knots, Mults, Degree);
        myIsDone = true;
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array1<Point2D> Points, int DegMin = 3, int DegMax = 8,
                     Continuity Continuity = C2, double Tol2D = 1.0e-6)
    {
        myIsDone = false;
        Init(Points, Approx_ChordLength, DegMin, DegMax, Continuity, Tol2D);
    }

    /// <summary>Approximates <paramref name="Points"/> at the parameters <paramref name="Params"/>.</summary>
    public void Init(NCollection_Array1<Point2D> Points, NCollection_Array1<double> Params, int DegMin = 3,
                     int DegMax = 8, Continuity Continuity = C2, double Tol2D = 1.0e-3)
    {
        myIsDone = false;
        if (Params.Length() != Points.Length())
        {
            throw new OutOfRangeException("Geom2dAPI_PointsToBSpline::Init() - invalid input");
        }

        double Tol3D = 0.0; // dummy argument for BSplineCompute.
        int Nbp = Params.Length();
        math_Vector theParams = new(1, Nbp);
        theParams[1] = 0.0;
        theParams[Nbp] = 1.0;

        double Uf = Params[Params.Lower()];
        double Ul = Params[Params.Upper()] - Uf;
        if (!hasMeaningfulSpan(Ul))
        {
            return;
        }

        for (int i = 2; i < Nbp; i++)
        {
            // OCCT reads Params(i), right only for parameters indexed from 1
            theParams[i] = (Params[i] - Uf) / Ul;
        }

        AppDef_BSplineCompute TheComputer = new(DegMin, DegMax, Tol3D, Tol2D, 0, true, Approx_IsoParametric, true);

        TheComputer.SetParameters(theParams);

        SetContinuity(TheComputer, Continuity);

        TheComputer.Perform(new AppDef_MultiLine(Points));

        AppParCurves_MultiBSpCurve TheCurve = TheComputer.Value();

        NCollection_Array1<Point2D> Poles = new(1, TheCurve.NbPoles());

        TheCurve.Curve(1, Poles);

        // unlike GeomAPI_PointsToBSpline, OCCT keeps the knots on [0, 1] here (no Reparametrize to Params' range)
        myCurve = new Geom2d_BSplineCurve(Poles, TheCurve.Knots(), TheCurve.Multiplicities(), TheCurve.Degree());
        myIsDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public void Init(NCollection_Array1<Point2D> Points, double W1, double W2, double W3, int DegMax = 8,
                     Continuity Continuity = C2, double Tol2D = 1.0e-3)
    {
        myIsDone = false;
        int NbPoint = Points.Length(), i;

        int nbit = 2;
        if (Tol2D <= 1.0e-3)
        {
            nbit = 0;
        }

        // Variational algo
        AppDef_MultiLine multL = new(NbPoint);
        for (i = 1; i <= NbPoint; ++i)
        {
            AppDef_MultiPointConstraint mpc = new(0, 1);
            mpc.SetPoint2d(1, Points.Value(Points.Lower() + i - 1));
            multL.SetValue(i, mpc);
        }

        NCollection_HArray1<AppParCurves_ConstraintCouple> TABofCC = new(1, NbPoint);
        AppParCurves_Constraint Constraint = AppParCurves_Constraint.AppParCurves_NoConstraint;

        for (i = 1; i <= NbPoint; ++i)
        {
            AppParCurves_ConstraintCouple ACC = new(i, Constraint);
            TABofCC.SetValue(i, ACC);
        }

        AppDef_Variational Variation = new(multL, 1, NbPoint, TABofCC);

        //===================================
        int theMaxSegments = 1000;
        bool theWithMinMax = false;
        //===================================

        Variation.SetMaxDegree(DegMax);
        Variation.SetContinuity(Continuity);
        Variation.SetMaxSegment(theMaxSegments);

        Variation.SetTolerance(Tol2D);
        Variation.SetWithMinMax(theWithMinMax);
        Variation.SetNbIterations(nbit);

        Variation.SetCriteriumWeight(W1, W2, W3);

        if (!Variation.IsCreated())
        {
            return;
        }

        if (Variation.IsOverConstrained())
        {
            return;
        }

        try
        {
            Variation.Approximate();
        }
        catch (FailureException)
        {
            return;
        }

        if (!Variation.IsDone())
        {
            return;
        }

        AppParCurves_MultiBSpCurve TheCurve = Variation.Value();

        NCollection_Array1<Point2D> Poles = new(1, TheCurve.NbPoles());

        TheCurve.Curve(1, Poles);

        myCurve = new Geom2d_BSplineCurve(Poles, TheCurve.Knots(), TheCurve.Multiplicities(), TheCurve.Degree());

        myIsDone = true;
    }

    // The switch on the continuity that OCCT writes out in each least-squares Init.
    private static void SetContinuity(AppDef_BSplineCompute TheComputer, Continuity Continuity)
    {
        switch (Continuity)
        {
            case C0:
                TheComputer.SetContinuity(0);
                break;

            case G1:
            case C1:
                TheComputer.SetContinuity(1);
                break;

            case G2:
            case C2:
                TheComputer.SetContinuity(2);
                break;

            default:
                TheComputer.SetContinuity(3);
                break;
        }
    }

    /// <summary>The approximating curve (frozen). Throws StdFail_NotDone if the approximation failed.</summary>
    public Geom2d_BSplineCurve Curve()
    {
        if (!myIsDone)
        {
            throw new NotDoneException(" ");
        }
        myCurve!.Freeze();
        return myCurve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom2d_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator Geom2d_BSplineCurve?(Geom2dAPI_PointsToBSpline theApprox)
    {
        theApprox.myCurve?.Freeze();
        return theApprox.myCurve;
    }

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone() => myIsDone;
}
