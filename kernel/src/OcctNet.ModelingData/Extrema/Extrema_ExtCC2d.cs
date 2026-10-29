// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The extremum distances between two 2D curves on parameter ranges. A line or circle against a conic (and a few
/// other pairs) are solved analytically (<see cref="Extrema_ExtElC2d"/>); other pairs by global optimisation
/// (<see cref="Extrema_ECC2d"/>).
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): the second curve is set by <c>Initialize</c>, the first by <c>Perform</c>.
/// Keeps a reference to the second curve, as OCCT does. OCCT's <c>std::shared_ptr</c> locals are plain references.
/// </remarks>
public class Extrema_ExtCC2d
{
    private bool myIsFindSingleSolution; // Default value is false.
    private bool myDone;
    private bool myIsPar;
    private readonly NCollection_Sequence<Extrema_POnCurv2d> mypoints = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private int mynbext;
    private bool inverse;
    private Adaptor2d_Curve2d? myC;
    private double myv1;
    private double myv2;
    private double mytolc1;
    private double mytolc2;
    private gp_Pnt2d P1f = new();
    private gp_Pnt2d P1l = new();
    private gp_Pnt2d P2f = new();
    private gp_Pnt2d P2l = new();
    private double mydist11;
    private double mydist12;
    private double mydist21;
    private double mydist22;

    public Extrema_ExtCC2d()
    {
        myIsFindSingleSolution = false;
        myDone = false;
        myIsPar = false;
        mynbext = 0;
        inverse = false;
        myC = null;
        myv1 = 0.0;
        myv2 = 0.0;
        mytolc1 = 0.0;
        mytolc2 = 0.0;
        mydist11 = 0.0;
        mydist12 = 0.0;
        mydist21 = 0.0;
        mydist22 = 0.0;
    }

    /// <summary>The extremum distances between C1 and C2 on their whole ranges.</summary>
    public Extrema_ExtCC2d(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myIsFindSingleSolution = false;
        Initialize(C2, Extrema_Curve2dTool.FirstParameter(C2), Extrema_Curve2dTool.LastParameter(C2), TolC1, TolC2);
        Perform(C1, Extrema_Curve2dTool.FirstParameter(C1), Extrema_Curve2dTool.LastParameter(C1));
    }

    /// <summary>The extremum distances between C1 on [U1, U2] and C2 on [V1, V2].</summary>
    public Extrema_ExtCC2d(Adaptor2d_Curve2d C1, Adaptor2d_Curve2d C2, double U1, double U2, double V1, double V2,
        double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myIsFindSingleSolution = false;
        Initialize(C2, V1, V2, TolC1, TolC2);
        Perform(C1, U1, U2);
    }

    /// <summary>Sets the second curve C2 on [V1, V2] and the tolerances.</summary>
    public void Initialize(Adaptor2d_Curve2d C2, double V1, double V2, double TolC1 = 1.0e-10, double TolC2 = 1.0e-10)
    {
        myC = C2;
        myv1 = V1;
        myv2 = V2;
        mytolc1 = TolC1;
        mytolc2 = TolC2;
    }

    /// <summary>Computes the extremum distances between C1 on [U1, U2] and the second curve.</summary>
    public void Perform(Adaptor2d_Curve2d C1, double U1, double U2)
    {
        mypoints.Clear();
        mySqDist.Clear();
        GeomAbs_CurveType type1 = Extrema_Curve2dTool.GetType(C1), type2 = Extrema_Curve2dTool.GetType(myC!);
        double U11, U12, U21, U22, Tol = Min(mytolc1, mytolc2);
        //  Extrema_POnCurv2d P1, P2;
        mynbext = 0;
        inverse = false;
        myIsPar = false;

        U11 = U1;
        U12 = U2;
        U21 = myv1;
        U22 = myv2;
        P1f = Extrema_Curve2dTool.Value(C1, U11);
        P1l = Extrema_Curve2dTool.Value(C1, U12);
        P2f = Extrema_Curve2dTool.Value(myC!, U21);
        P2l = Extrema_Curve2dTool.Value(myC!, U22);

        Extrema_ExtElC2d aXtream;
        Extrema_ECC2d aParamSolver;

        switch (type1)
        {
            //
            //  The first curve is a circle:
            //
            case GeomAbs_Circle:
            {
                switch (type2)
                {
                    case GeomAbs_Line:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(myC!), Extrema_Curve2dTool.Circle(C1), Tol);
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    case GeomAbs_Circle:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(C1), Extrema_Curve2dTool.Circle(myC!));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Ellipse:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(C1), Extrema_Curve2dTool.Ellipse(myC!));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Parabola:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(C1), Extrema_Curve2dTool.Parabola(myC!));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    case GeomAbs_Hyperbola:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(C1), Extrema_Curve2dTool.Hyperbola(myC!));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    default:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        double Period2 = 0.0;
                        if (Extrema_Curve2dTool.IsPeriodic(myC!))
                        {
                            Period2 = Extrema_Curve2dTool.Period(myC!);
                        }
                        Results(aParamSolver, U11, U12, U21, U22, 2 * M_PI, Period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is an ellipse:
            //
            case GeomAbs_Ellipse:
            {
                switch (type2)
                {
                    case GeomAbs_Line:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(myC!), Extrema_Curve2dTool.Ellipse(C1));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    case GeomAbs_Circle:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(myC!), Extrema_Curve2dTool.Ellipse(C1));
                        Results(aXtream, U11, U12, U21, U22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Ellipse:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    case GeomAbs_Hyperbola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(C1),
                        // Extrema_Curve2dTool::Hyperbola(*myC));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 2 * M_PI, 0.0);
                    }
                    break;
                    default:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        double Period2 = 0.0;
                        if (Extrema_Curve2dTool.IsPeriodic(myC!))
                        {
                            Period2 = Extrema_Curve2dTool.Period(myC!);
                        }
                        Results(aParamSolver, U11, U12, U21, U22, 2 * M_PI, Period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a parabola:
            //
            case GeomAbs_Parabola:
            {
                switch (type2)
                {
                    case GeomAbs_Line:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(myC!), Extrema_Curve2dTool.Parabola(C1));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Circle:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(myC!), Extrema_Curve2dTool.Parabola(C1));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Ellipse:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(*myC),
                        // Extrema_Curve2dTool::Parabola(C1));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Parabola(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Hyperbola:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(*myC),
                        // Extrema_Curve2dTool::Parabola(C1));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        double Period2 = 0.0;
                        if (Extrema_Curve2dTool.IsPeriodic(myC!))
                        {
                            Period2 = Extrema_Curve2dTool.Period(myC!);
                        }
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, Period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a hyperbola:
            //
            case GeomAbs_Hyperbola:
            {
                switch (type2)
                {
                    case GeomAbs_Line:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(myC!), Extrema_Curve2dTool.Hyperbola(C1));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Circle:
                    {
                        inverse = true;
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Circle(myC!), Extrema_Curve2dTool.Hyperbola(C1));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Ellipse:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(*myC),
                        // Extrema_Curve2dTool::Hyperbola(C1));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Hyperbola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(C1),
                        // Extrema_Curve2dTool::Hyperbola(*myC));
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        double Period2 = 0.0;
                        if (Extrema_Curve2dTool.IsPeriodic(myC!))
                        {
                            Period2 = Extrema_Curve2dTool.Period(myC!);
                        }
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, Period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a line:
            //
            case GeomAbs_Line:
            {
                switch (type2)
                {
                    case GeomAbs_Line:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(C1), Extrema_Curve2dTool.Line(myC!), Tol);
                        Results(aXtream, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Circle:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(C1), Extrema_Curve2dTool.Circle(myC!), Tol);
                        Results(aXtream, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Ellipse:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(C1), Extrema_Curve2dTool.Ellipse(myC!));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 2 * M_PI);
                    }
                    break;
                    case GeomAbs_Parabola:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(C1), Extrema_Curve2dTool.Parabola(myC!));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    case GeomAbs_Hyperbola:
                    {
                        aXtream = new Extrema_ExtElC2d(Extrema_Curve2dTool.Line(C1), Extrema_Curve2dTool.Hyperbola(myC!));
                        Results(aXtream, U11, U12, U21, U22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        aParamSolver = new Extrema_ECC2d(C1, myC!);
                        aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        aParamSolver.Perform();
                        double Period2 = 0.0;
                        if (Extrema_Curve2dTool.IsPeriodic(myC!))
                        {
                            Period2 = Extrema_Curve2dTool.Period(myC!);
                        }
                        Results(aParamSolver, U11, U12, U21, U22, 0.0, Period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a BezierCurve or a BSplineCurve:
            //
            default:
            {
                aParamSolver = new Extrema_ECC2d(C1, myC!);
                aParamSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                aParamSolver.Perform();
                double Period1 = 0.0;
                if (Extrema_Curve2dTool.IsPeriodic(C1))
                {
                    Period1 = Extrema_Curve2dTool.Period(C1);
                }
                double Period2 = 0.0;
                if (Extrema_Curve2dTool.IsPeriodic(myC!))
                {
                    Period2 = Extrema_Curve2dTool.Period(myC!);
                }
                Results(aParamSolver, U11, U12, U21, U22, Period1, Period2);
            }
            break;
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_OutOfRange">N is not in 1..NbExt().</exception>
    public double SquareDistance(int N = 1)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if ((N <= 0) || (N > mynbext))
        {
            throw new Standard_OutOfRange();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbExt()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return mynbext;
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    /// <exception cref="Standard_OutOfRange">N is not in 1..NbExt().</exception>
    public void Points(int N, out Extrema_POnCurv2d P1, out Extrema_POnCurv2d P2)
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        if ((N <= 0) || (N > mynbext))
        {
            throw new Standard_OutOfRange();
        }
        P1 = mypoints.Value(2 * N - 1);
        P2 = mypoints.Value(2 * N);
    }

    /// <summary>The square distances between the ends of the ranges and the end points (set only by a successful
    /// computation).</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
        out gp_Pnt2d P11, out gp_Pnt2d P12, out gp_Pnt2d P21, out gp_Pnt2d P22)
    {
        dist11 = mydist11;
        dist12 = mydist12;
        dist21 = mydist21;
        dist22 = mydist22;
        P11 = P1f;
        P12 = P1l;
        P21 = P2f;
        P22 = P2l;
    }

    protected void Results(Extrema_ExtElC2d AlgExt, double Ut11, double Ut12, double Ut21, double Ut22,
        double Period1 = 0.0, double Period2 = 0.0)
    {
        int i, NbExt;
        double Val, U, U2;
        Extrema_POnCurv2d P1, P2;

        myDone = AlgExt.IsDone();
        // OCCT asks IsParallel before checking IsDone: an Extrema_ExtElC2d that is not done throws StdFail_NotDone
        // here (a circle and a concentric circular ellipse) instead of leaving this algorithm not done
        myIsPar = AlgExt.IsParallel();
        if (myDone)
        {
            if (!myIsPar)
            {
                NbExt = AlgExt.NbExt();
                for (i = 1; i <= NbExt; i++)
                {
                    // Verification of the validity of parameters for the trimmed case:
                    AlgExt.Points(i, out P1, out P2);
                    if (!inverse)
                    {
                        U = P1.Parameter();
                        if (Period1 != 0.0)
                        {
                            U = ElCLib.InPeriod(U, Ut11, Ut11 + Period1);
                        }
                        U2 = P2.Parameter();
                        if (Period2 != 0.0)
                        {
                            U2 = ElCLib.InPeriod(U2, Ut21, Ut21 + Period2);
                        }
                    }
                    else
                    {
                        U2 = P1.Parameter();
                        if (Period2 != 0.0)
                        {
                            U2 = ElCLib.InPeriod(U2, Ut21, Ut21 + Period2);
                        }
                        U = P2.Parameter();
                        if (Period1 != 0.0)
                        {
                            U = ElCLib.InPeriod(U, Ut11, Ut11 + Period1);
                        }
                    }
                    if ((U >= Ut11 - Precision.PConfusion()) && (U <= Ut12 + Precision.PConfusion())
                        && (U2 >= Ut21 - Precision.PConfusion()) && (U2 <= Ut22 + Precision.PConfusion()))
                    {
                        mynbext++;
                        Val = AlgExt.SquareDistance(i);
                        mySqDist.Append(Val);
                        if (!inverse)
                        {
                            P1.SetValues(U, P1.Value());
                            P2.SetValues(U2, P2.Value());
                            mypoints.Append(P1);
                            mypoints.Append(P2);
                        }
                        else
                        {
                            P1.SetValues(U2, P1.Value());
                            P2.SetValues(U, P2.Value());
                            mypoints.Append(P2);
                            mypoints.Append(P1);
                        }
                    }
                }
            }

            mydist11 = P1f.SquareDistance(P2f);
            mydist12 = P1f.SquareDistance(P2l);
            mydist21 = P1l.SquareDistance(P2f);
            mydist22 = P1l.SquareDistance(P2l);
        }
    }

    protected void Results(Extrema_ECC2d AlgExt, double Ut11, double Ut12, double Ut21, double Ut22,
        double Period1 = 0.0, double Period2 = 0.0)
    {
        int i, NbExt;
        double Val, U, U2;
        Extrema_POnCurv2d P1, P2;

        myDone = AlgExt.IsDone();
        if (myDone)
        {
            myIsPar = AlgExt.IsParallel();
            NbExt = AlgExt.NbExt();
            for (i = 1; i <= NbExt; i++)
            {
                // Verification of parameter validity for the trimmed case:
                AlgExt.Points(i, out P1, out P2);
                U = P1.Parameter();
                if (Period1 != 0.0)
                {
                    U = ElCLib.InPeriod(U, Ut11, Ut11 + Period1);
                }
                U2 = P2.Parameter();
                if (Period2 != 0.0)
                {
                    U2 = ElCLib.InPeriod(U2, Ut21, Ut21 + Period2);
                }

                if ((U >= Ut11 - Precision.PConfusion()) && (U <= Ut12 + Precision.PConfusion())
                    && (U2 >= Ut21 - Precision.PConfusion()) && (U2 <= Ut22 + Precision.PConfusion()))
                {
                    mynbext++;
                    Val = AlgExt.SquareDistance(i);
                    P1.SetValues(U, P1.Value());
                    P2.SetValues(U2, P2.Value());
                    mySqDist.Append(Val);
                    mypoints.Append(P1);
                    mypoints.Append(P2);
                }
            }

            mydist11 = P1f.SquareDistance(P2f);
            mydist12 = P1f.SquareDistance(P2l);
            mydist21 = P1l.SquareDistance(P2f);
            mydist22 = P1l.SquareDistance(P2l);
        }
    }

    /// <summary>True if the curves are parallel.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public bool IsParallel()
    {
        if (!myDone)
        {
            throw new StdFail_NotDone();
        }
        return myIsPar;
    }

    /// <summary>Asks for one solution only (faster, for the global optimisation).</summary>
    public void SetSingleSolutionFlag(bool theFlag) => myIsFindSingleSolution = theFlag;

    public bool GetSingleSolutionFlag() => myIsFindSingleSolution;
}
