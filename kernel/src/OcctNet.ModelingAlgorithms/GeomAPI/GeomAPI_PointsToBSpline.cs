// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_PointsToBSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Approximates a set of points by a B-spline curve: by least squares (AppDef_BSplineCompute) with a
/// parametrisation type or given parameters, or by the variational method (AppDef_Variational) with criterion
/// weights.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's implicit conversion to the curve is an explicit conversion (decision 28).
/// <see cref="Curve"/> returns the computed curve itself, as OCCT returns its handle, frozen (decision 243).
/// </remarks>
public class GeomAPI_PointsToBSpline
{
    private bool myIsDone;
    private BSplineCurve3D? myCurve;

    private static bool hasMeaningfulSpan(double theSpan)
    {
        double aResolution = Constants.Resolution();
        return theSpan * theSpan > aResolution * aResolution;
    }

    /// <summary>An empty approximation, to be computed with <c>Init</c>.</summary>
    public GeomAPI_PointsToBSpline()
    {
        myIsDone = false;
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public GeomAPI_PointsToBSpline(NCollection_Array1<Point3D> Points, int DegMin = 3, int DegMax = 8,
                                   Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>.</summary>
    public GeomAPI_PointsToBSpline(NCollection_Array1<Point3D> Points, ParametrizationType ParType,
                                   int DegMin = 3, int DegMax = 8, Continuity Continuity = C2,
                                   double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, ParType, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> at the parameters <paramref name="Params"/>.</summary>
    public GeomAPI_PointsToBSpline(NCollection_Array1<Point3D> Points, NCollection_Array1<double> Params,
                                   int DegMin = 3, int DegMax = 8, Continuity Continuity = C2,
                                   double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, Params, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public GeomAPI_PointsToBSpline(NCollection_Array1<Point3D> Points, double W1, double W2, double W3,
                                   int DegMax = 8, Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, W1, W2, W3, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> with chord-length parameters.</summary>
    public void Init(NCollection_Array1<Point3D> Points, int DegMin = 3, int DegMax = 8,
                     Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        Init(Points, ChordLength, DegMin, DegMax, Continuity, Tol3D);
    }

    /// <summary>Approximates <paramref name="Points"/> with the parametrisation <paramref name="ParType"/>.</summary>
    public void Init(NCollection_Array1<Point3D> Points, ParametrizationType ParType, int DegMin = 3,
                     int DegMax = 8, Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        double Tol2D = 0.0; // dummy argument for BSplineCompute.
        int nbit = 2;
        bool UseSquares = false;
        if (Tol3D <= 1.0e-3)
        {
            UseSquares = true;
        }

        BSplineCompute TheComputer = new(DegMin, DegMax, Tol3D, Tol2D, nbit, true, ParType, UseSquares);

        SetContinuity(TheComputer, Continuity);

        TheComputer.Perform(new MultiLine(Points));

        MultiBSpCurve TheCurve = TheComputer.Value();

        NCollection_Array1<Point3D> Poles = new(1, TheCurve.NbPoles());

        TheCurve.Curve(1, Poles);

        myCurve = new BSplineCurve3D(Poles, TheCurve.Knots(), TheCurve.Multiplicities(), TheCurve.Degree());
        myIsDone = true;
    }

    /// <summary>Approximates <paramref name="Points"/> at the parameters <paramref name="Params"/>.</summary>
    public void Init(NCollection_Array1<Point3D> Points, NCollection_Array1<double> Params, int DegMin = 3,
                     int DegMax = 8, Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        if (Params.Length() != Points.Length())
        {
            throw new OutOfRangeException("GeomAPI_PointsToBSpline::Init() - invalid input");
        }

        double Tol2D = 0.0; // dummy argument for BSplineCompute.
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

        BSplineCompute TheComputer = new(DegMin, DegMax, Tol3D, Tol2D, 0, true, IsoParametric, true);

        TheComputer.SetParameters(theParams);

        SetContinuity(TheComputer, Continuity);

        TheComputer.Perform(new MultiLine(Points));

        MultiBSpCurve TheCurve = TheComputer.Value();

        NCollection_Array1<Point3D> Poles = new(1, TheCurve.NbPoles());
        NCollection_Array1<double> Knots;

        TheCurve.Curve(1, Poles);
        Knots = TheCurve.Knots();

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(Params[Params.Lower()], Params[Params.Upper()], Knots);

        myCurve = new BSplineCurve3D(Poles, Knots, TheCurve.Multiplicities(), TheCurve.Degree());
        myIsDone = true;
    }

    /// <summary>
    /// Approximates <paramref name="Points"/> by the variational method, minimising W1 x length + W2 x curvature +
    /// W3 x torsion.
    /// </summary>
    public void Init(NCollection_Array1<Point3D> Points, double W1, double W2, double W3, int DegMax = 8,
                     Continuity Continuity = C2, double Tol3D = 1.0e-3)
    {
        myIsDone = false;
        int NbPoint = Points.Length(), i;

        int nbit = 2;
        if (Tol3D <= 1.0e-3)
        {
            nbit = 0;
        }

        // Variational algo
        MultiLine multL = new(NbPoint);
        for (i = 1; i <= NbPoint; ++i)
        {
            MultiPointConstraint mpc = new(1, 0);
            mpc.SetPoint(1, Points.Value(Points.Lower() + i - 1));
            multL.SetValue(i, mpc);
        }

        NCollection_HArray1<ConstraintCouple> TABofCC = new(1, NbPoint);
        Constraint Constraint = global::Kerf.Engine.Approximation.Constraint.NoConstraint;

        for (i = 1; i <= NbPoint; ++i)
        {
            ConstraintCouple ACC = new(i, Constraint);
            TABofCC.SetValue(i, ACC);
        }

        Variational Variation = new(multL, 1, NbPoint, TABofCC);

        //===================================
        int theMaxSegments = 1000;
        bool theWithMinMax = false;
        //===================================

        Variation.SetMaxDegree(DegMax);
        Variation.SetContinuity(Continuity);
        Variation.SetMaxSegment(theMaxSegments);

        Variation.SetTolerance(Tol3D);
        Variation.SetWithMinMax(theWithMinMax);
        Variation.SetNbIterations(nbit);

        Variation.SetCriteriumWeight(W1, W2, W3);

        if (!Variation.IsCreated)
        {
            return;
        }

        if (Variation.IsOverConstrained)
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

        if (!Variation.IsDone)
        {
            return;
        }

        MultiBSpCurve TheCurve = Variation.Value();

        NCollection_Array1<Point3D> Poles = new(1, TheCurve.NbPoles());

        TheCurve.Curve(1, Poles);

        myCurve = new BSplineCurve3D(Poles, TheCurve.Knots(), TheCurve.Multiplicities(), TheCurve.Degree());

        myIsDone = true;
    }

    // The switch on the continuity that OCCT writes out in both least-squares Init.
    private static void SetContinuity(BSplineCompute TheComputer, Continuity Continuity)
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
    public BSplineCurve3D Curve()
    {
        if (!myIsDone)
        {
            throw new NotDoneException("GeomAPI_PointsToBSpline::Curve ");
        }
        myCurve!.Freeze();
        return myCurve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator BSplineCurve3D?(GeomAPI_PointsToBSpline theApprox)
    {
        theApprox.myCurve?.Freeze();
        return theApprox.myCurve;
    }

    /// <summary>True if the approximation succeeded.</summary>
    public bool IsDone() => myIsDone;
}
