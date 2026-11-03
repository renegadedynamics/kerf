// Port of: src/ModelingData/TKGeomBase/Approx/Approx_ComputeCLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Approximates a continuous MultiLine (an <see cref="AppCont_Function"/>) by Bezier MultiCurves, from degree
/// degreemin to degreemax, cutting the parameter range (when cutting is on) until the tolerances are reached.
/// </summary>
/// <remarks>
/// The body of OCCT's <c>Approx_ComputeCLine.gxx</c>. OCCT instantiates it twice, as Approx_FitAndDivide and
/// Approx_FitAndDivide2d, both over AppCont_Function (which needs no tool), so the body is written once here and the
/// two instantiations are sealed subclasses. A working object (decision 21): the setters are public.
/// </remarks>
public abstract class Approx_ComputeCLine
{
    private const int MAXSEGM = 1000;

    private readonly NCollection_Sequence<AppParCurves_MultiCurve> myMultiCurves = new();
    private readonly NCollection_Sequence<double> myfirstparam = new();
    private readonly NCollection_Sequence<double> mylastparam = new();
    private AppParCurves_MultiCurve TheMultiCurve = new();
    private bool alldone;
    private bool tolreached;
    private readonly NCollection_Sequence<double> Tolers3d = new();
    private readonly NCollection_Sequence<double> Tolers2d = new();
    private int mydegremin;
    private int mydegremax;
    private double mytol3d;
    private double mytol2d;
    private double currenttol3d;
    private double currenttol2d;
    private readonly bool mycut;
    private AppParCurves_Constraint myfirstC;
    private AppParCurves_Constraint mylastC;
    private int myMaxSegments;
    private bool myInvOrder;
    private bool myHangChecking;

    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    protected Approx_ComputeCLine(AppCont_Function Line,
                                  int degreemin,
                                  int degreemax,
                                  double Tolerance3d,
                                  double Tolerance2d,
                                  bool cutting,
                                  AppParCurves_Constraint FirstC,
                                  AppParCurves_Constraint LastC)
    {
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mycut = cutting;
        myfirstC = FirstC;
        mylastC = LastC;
        myMaxSegments = MAXSEGM;
        myInvOrder = true;
        myHangChecking = true;
        alldone = false;
        tolreached = false;
        currenttol3d = 0.0;
        currenttol2d = 0.0;
        Perform(Line);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    protected Approx_ComputeCLine(int degreemin,
                                  int degreemax,
                                  double Tolerance3d,
                                  double Tolerance2d,
                                  bool cutting,
                                  AppParCurves_Constraint FirstC,
                                  AppParCurves_Constraint LastC)
    {
        alldone = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mycut = cutting;
        myfirstC = FirstC;
        mylastC = LastC;
        myMaxSegments = MAXSEGM;
        myInvOrder = true;
        myHangChecking = true;
        tolreached = false;
        currenttol3d = 0.0;
        currenttol2d = 0.0;
    }

    /// <summary>Runs the algorithm after having initialized the fields.</summary>
    public void Perform(AppCont_Function Line)
    {
        double UFirst, ULast;
        bool Finish = false, begin = true, Ok = false;
        double thetol3d = global::Kerf.Engine.Foundation.Precision.Confusion(), thetol2d = global::Kerf.Engine.Foundation.Precision.Confusion();
        UFirst = Line.FirstParameter();
        ULast = Line.LastParameter();
        double TolU;
        if (myHangChecking)
        {
            TolU = Max((ULast - UFirst) * 1.0e-03, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            TolU = Max((ULast - UFirst) * 1.0e-05, global::Kerf.Engine.Foundation.Precision.PApproximation());
        }
        double myfirstU = UFirst;
        double mylastU = ULast;
        int aMaxSegments = 0;
        int aMaxSegments1 = myMaxSegments - 1;
        int aNbCut = 0, aNbImp = 0, aNbComp = 10;

        if (!mycut)
        {
            alldone = Compute(Line, UFirst, ULast, ref thetol3d, ref thetol2d);
            if (!alldone)
            {
                tolreached = false;
                myfirstparam.Append(UFirst);
                mylastparam.Append(ULast);
                myMultiCurves.Append(new AppParCurves_MultiCurve(TheMultiCurve));
                Tolers3d.Append(currenttol3d);
                Tolers2d.Append(currenttol2d);
            }
        }
        else
        {
            // previous decision to be taken if we get worse with next cut (eap)
            AppParCurves_MultiCurve KeptMultiCurve = new();
            double KeptUfirst = 0.0, KeptUlast = 0.0, KeptT3d = RealLast(), KeptT2d = 0.0;

            while (!Finish)
            {
                // Management of the multiline splitting for approximation:
                if (!begin)
                {
                    if (Ok)
                    {
                        // Calcul de la partie a approximer.
                        myfirstU = mylastU;
                        mylastU = ULast;
                        aNbCut = 0;
                        aNbImp = 0;
                        if (Math.Abs(ULast - myfirstU) <= RealEpsilon() || aMaxSegments >= myMaxSegments)
                        {
                            Finish = true;
                            alldone = true;
                            return;
                        }
                        KeptT3d = RealLast();
                        KeptT2d = 0;
                        KeptUfirst = myfirstU;
                        KeptUlast = mylastU;
                    }
                    else
                    {
                        // keep best decision
                        if ((thetol3d + thetol2d) < (KeptT3d + KeptT2d))
                        {
                            KeptMultiCurve = new AppParCurves_MultiCurve(TheMultiCurve);
                            KeptUfirst = myfirstU;
                            KeptUlast = mylastU;
                            KeptT3d = thetol3d;
                            KeptT2d = thetol2d;
                            aNbImp++;
                        }

                        // cut an interval
                        mylastU = (myfirstU + mylastU) / 2;
                        aNbCut++;
                    }
                }

                // Calculation of parameters on this new interval.
                Ok = Compute(Line, myfirstU, mylastU, ref thetol3d, ref thetol2d);
                if (Ok)
                {
                    aMaxSegments++;
                }

                bool aStopCutting = false;
                if (myHangChecking && aNbCut >= aNbComp)
                {
                    if (aNbCut > aNbImp + 1)
                    {
                        aStopCutting = true;
                    }
                    aNbCut = 0;
                    aNbImp = 0;
                }
                // is new decision better?
                if (!Ok
                    && (Math.Abs(myfirstU - mylastU) <= TolU || aMaxSegments >= aMaxSegments1
                        || aStopCutting))
                {
                    Ok = true; // stop interval cutting, approx the rest part

                    if ((thetol3d + thetol2d) < (KeptT3d + KeptT2d))
                    {
                        KeptMultiCurve = new AppParCurves_MultiCurve(TheMultiCurve);
                        KeptUfirst = myfirstU;
                        KeptUlast = mylastU;
                        KeptT3d = thetol3d;
                        KeptT2d = thetol2d;
                    }

                    mylastU = KeptUlast;

                    tolreached = false; // helas
                    myMultiCurves.Append(new AppParCurves_MultiCurve(KeptMultiCurve));
                    aMaxSegments++;
                    Tolers3d.Append(KeptT3d);
                    Tolers2d.Append(KeptT2d);
                    myfirstparam.Append(KeptUfirst);
                    mylastparam.Append(KeptUlast);
                }

                begin = false;
            } // while (!Finish)
        }
    }

    /// <summary>The number of MultiCurves approximating the MultiLine.</summary>
    public int NbMultiCurves() => myMultiCurves.Length();

    /// <summary>
    /// The approximating MultiCurve of range Index: a copy sharing the poles, as OCCT's by-value return (decision 141).
    /// </summary>
    public AppParCurves_MultiCurve Value(int Index = 1) => new(myMultiCurves.Value(Index));

    private bool Compute(AppCont_Function Line,
                         double Ufirst,
                         double Ulast,
                         ref double TheTol3d,
                         ref double TheTol2d)
    {
        const int NbPointsMax = 24;
        const double aMinRatio = 0.05;
        const int aMaxDeg = 8;
        //
        int deg, NbPoints;
        bool mydone;

        AppParCurves_MultiCurve aPrevCurve;
        double aPrevTol3d, aPrevTol2d;
        bool aPrevIsOk = false;
        bool anInvOrder = myInvOrder;
        if (anInvOrder && mydegremax > aMaxDeg)
        {
            if ((Ulast - Ufirst) / (Line.LastParameter() - Line.FirstParameter()) < aMinRatio)
            {
                anInvOrder = false;
            }
        }
        if (anInvOrder)
        {
            aPrevCurve = new();
            aPrevTol3d = RealLast();
            aPrevTol2d = RealLast();
            for (deg = mydegremax; deg >= mydegremin; deg--)
            {
                NbPoints = Min(2 * deg + 1, NbPointsMax);
                AppCont_LeastSquare LSquare = new(Line, Ufirst, Ulast, myfirstC, mylastC, deg, NbPoints);
                mydone = LSquare.IsDone();
                if (mydone)
                {
                    LSquare.Error(out _, out TheTol3d, out TheTol2d);
                    if (TheTol3d <= mytol3d && TheTol2d <= mytol2d)
                    {
                        if (deg == mydegremin)
                        {
                            // Stockage de la multicurve approximee.
                            tolreached = true;
                            myMultiCurves.Append(LSquare.Value());
                            myfirstparam.Append(Ufirst);
                            mylastparam.Append(Ulast);
                            Tolers3d.Append(TheTol3d);
                            Tolers2d.Append(TheTol2d);
                            return true;
                        }
                        aPrevTol3d = TheTol3d;
                        aPrevTol2d = TheTol2d;
                        aPrevCurve = LSquare.Value();
                        aPrevIsOk = true;
                        continue;
                    }
                    else if (aPrevIsOk)
                    {
                        // Stockage de la multicurve approximee.
                        tolreached = true;
                        TheTol3d = aPrevTol3d;
                        TheTol2d = aPrevTol2d;
                        myMultiCurves.Append(aPrevCurve);
                        myfirstparam.Append(Ufirst);
                        mylastparam.Append(Ulast);
                        Tolers3d.Append(aPrevTol3d);
                        Tolers2d.Append(aPrevTol2d);
                        return true;
                    }
                }
                else if (aPrevIsOk)
                {
                    // Stockage de la multicurve approximee.
                    tolreached = true;
                    TheTol3d = aPrevTol3d;
                    TheTol2d = aPrevTol2d;
                    myMultiCurves.Append(aPrevCurve);
                    myfirstparam.Append(Ufirst);
                    mylastparam.Append(Ulast);
                    Tolers3d.Append(aPrevTol3d);
                    Tolers2d.Append(aPrevTol2d);
                    return true;
                }
                if (!aPrevIsOk && deg == mydegremax)
                {
                    // OCCT also stores TheMultiCurve and the tolerances in aPrevCurve/aPrevTol*, which are not read again.
                    TheMultiCurve = LSquare.Value();
                    currenttol3d = TheTol3d;
                    currenttol2d = TheTol2d;
                    break;
                }
            }
        }
        else
        {
            for (deg = mydegremin; deg <= mydegremax; deg++)
            {
                NbPoints = Min(2 * deg + 1, NbPointsMax);
                AppCont_LeastSquare LSquare = new(Line, Ufirst, Ulast, myfirstC, mylastC, deg, NbPoints);
                mydone = LSquare.IsDone();
                if (mydone)
                {
                    LSquare.Error(out _, out TheTol3d, out TheTol2d);
                    if (TheTol3d <= mytol3d && TheTol2d <= mytol2d)
                    {
                        // Stockage de la multicurve approximee.
                        tolreached = true;
                        myMultiCurves.Append(LSquare.Value());
                        myfirstparam.Append(Ufirst);
                        mylastparam.Append(Ulast);
                        Tolers3d.Append(TheTol3d);
                        Tolers2d.Append(TheTol2d);
                        return true;
                    }
                }
                if (deg == mydegremax)
                {
                    TheMultiCurve = LSquare.Value();
                    currenttol3d = TheTol3d;
                    currenttol2d = TheTol2d;
                }
            }
        }
        return false;
    }

    /// <summary>The first and last parameters of the MultiCurve Index.</summary>
    public void Parameters(int Index, out double firstpar, out double lastpar)
    {
        firstpar = myfirstparam.Value(Index);
        lastpar = mylastparam.Value(Index);
    }

    /// <summary>Changes the degrees of the approximation.</summary>
    public void SetDegrees(int degreemin, int degreemax)
    {
        mydegremin = degreemin;
        mydegremax = degreemax;
    }

    /// <summary>Changes the tolerances of the approximation.</summary>
    public void SetTolerances(double Tolerance3d, double Tolerance2d)
    {
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
    }

    /// <summary>Changes the constraints of the approximation.</summary>
    public void SetConstraints(AppParCurves_Constraint FirstC, AppParCurves_Constraint LastC)
    {
        myfirstC = FirstC;
        mylastC = LastC;
    }

    /// <summary>Changes the maximum number of segments allowed for cutting.</summary>
    public void SetMaxSegments(int theMaxSegments)
    {
        myMaxSegments = theMaxSegments;
    }

    /// <summary>Sets the order of degree selection: from degreemax down to degreemin when true (the default).</summary>
    public void SetInvOrder(bool theInvOrder)
    {
        myInvOrder = theInvOrder;
    }

    /// <summary>Sets whether a possible hang of the cutting is checked and stopped (the default).</summary>
    public void SetHangChecking(bool theHangChecking)
    {
        myHangChecking = theHangChecking;
    }

    /// <summary>False if some part of the line could not be approximated within the tolerances.</summary>
    public bool IsAllApproximated() => alldone;

    /// <summary>Whether the tolerances were reached.</summary>
    public bool IsToleranceReached() => tolreached;

    /// <summary>The 3D and 2D errors of the MultiCurve Index.</summary>
    public void Error(int Index, out double tol3d, out double tol2d)
    {
        tol3d = Tolers3d.Value(Index);
        tol2d = Tolers2d.Value(Index);
    }
}
