// Port of: src/ModelingData/TKGeomBase/Approx/Approx_BSplComputeLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;

namespace Occt;

/// <summary>
/// Approximates a MultiLine by one MultiBSpCurve: from degree degreemin to degreemax, adding knots (when cutting
/// is on) until the tolerances are reached. With Squares, the computation is a plain least-squares fit, without
/// iterations on the parameters.
/// </summary>
/// <remarks>
/// <para>
/// The generic form of OCCT's <c>Approx_BSplComputeLine.gxx</c> (decisions 142 and 202), instantiated by OCCT as
/// AppDef_BSplineCompute (over AppDef_MultiLine), and as GeomInt's and BRepApprox's BSpline approximators. The
/// gxx's helper classes (Approx_MyBSplGradient, Approx_BSpParLeastSquareOfMyBSplGradient, Approx_MyGradientbis,
/// ...) are, in every instantiation, the AppParCurves generics over the same MultiLine and tool, so this class
/// names them directly.
/// </para>
/// <para>A working object (decision 21): the setters are public, and <see cref="Value"/> and
/// <see cref="Parameters()"/> return its own results (shared: don't modify).</para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="LineTool">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class Approx_BSplComputeLine<MultiLine, LineTool>
    where LineTool : struct, IApprox_LineTool<MultiLine>
{
    private AppParCurves_MultiBSpCurve TheMultiBSpCurve = new();
    private bool alldone;
    private bool tolreached;
    private Approx_ParametrizationType Par;
    private NCollection_HArray1<double>? myParameters;
    private NCollection_HArray1<double>? myfirstParam;
    private NCollection_HArray1<double>? myknots;
    private NCollection_HArray1<int>? mymults;
    private bool myhasknots;
    private bool myhasmults;
    private readonly NCollection_HArray1<AppParCurves_ConstraintCouple> myConstraints;
    private int mydegremin;
    private int mydegremax;
    private double mytol3d;
    private double mytol2d;
    private double currenttol3d;
    private double currenttol2d;
    private bool mycut;
    private bool mysquares;
    private int myitermax;
    private AppParCurves_Constraint myfirstC;
    private AppParCurves_Constraint mylastC;
    private AppParCurves_Constraint realfirstC;
    private AppParCurves_Constraint reallastC;
    private int mycont;
    private double mylambda1;
    private double mylambda2;
    private bool myPeriodic;

    private void FirstTangencyVector(MultiLine Line, int index, math_Vector V)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool Ok = false;
        NCollection_Array1<Vector3D> TabV = new(1, mynbP3d);
        NCollection_Array1<Vector2D> TabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV, TabV2d);
        }
        else if (nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV2d);
        }
        else if (nbP3d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV);
        }

        if (Ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = TabV.Lower(); i <= TabV.Upper(); i++)
                {
                    V[j] = TabV[i].X();
                    V[j + 1] = TabV[i].Y();
                    V[j + 2] = TabV[i].Z();
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = TabV2d.Lower(); i <= TabV2d.Upper(); i++)
                {
                    V[j] = TabV2d[i].X();
                    V[j + 1] = TabV2d[i].Y();
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            AppParCurves_Constraint firstC, lastC;
            firstC = lastC = AppParCurves_PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index, index + 2);
            Parameters(Line, index, index + 2, mypar);
            AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, index, index + 2, firstC, lastC, mypar, nbpoles);
            AppParCurves_MultiCurve C = LSQ.BezierValue();

            Point3D myP;
            Vector3D myV;
            Point2D myP2d;
            Vector2D myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C.D1(i, 0.0, out myP, out myV);
                V[j] = myV.X();
                V[j + 1] = myV.Y();
                V[j + 2] = myV.Z();
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                C.D1(i, 0.0, out myP2d, out myV2d);
                V[j] = myV2d.X();
                V[j + 1] = myV2d.Y();
                j += 2;
            }
        }
    }

    private void LastTangencyVector(MultiLine Line, int index, math_Vector V)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        bool Ok = false;
        NCollection_Array1<Vector3D> TabV = new(1, mynbP3d);
        NCollection_Array1<Vector2D> TabV2d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV, TabV2d);
        }
        else if (nbP2d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV2d);
        }
        else if (nbP3d != 0)
        {
            Ok = LineTool.Tangency(Line, index, TabV);
        }

        if (Ok)
        {
            if (nbP3d != 0)
            {
                j = 1;
                for (i = TabV.Lower(); i <= TabV.Upper(); i++)
                {
                    V[j] = TabV[i].X();
                    V[j + 1] = TabV[i].Y();
                    V[j + 2] = TabV[i].Z();
                    j += 3;
                }
            }
            if (nbP2d != 0)
            {
                j = nbP3d * 3 + 1;
                for (i = TabV2d.Lower(); i <= TabV2d.Upper(); i++)
                {
                    V[j] = TabV2d[i].X();
                    V[j + 1] = TabV2d[i].Y();
                    j += 2;
                }
            }
        }
        else
        {
            // Search for a tangent vector by construction of a parabola:
            AppParCurves_Constraint firstC, lastC;
            firstC = lastC = AppParCurves_PassPoint;
            int nbpoles = 3;
            math_Vector mypar = new(index - 2, index);
            Parameters(Line, index - 2, index, mypar);
            AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, index - 2, index, firstC, lastC, mypar, nbpoles);
            AppParCurves_MultiCurve C = LSQ.BezierValue();

            Point3D myP;
            Vector3D myV;
            Point2D myP2d;
            Vector2D myV2d;
            j = 1;
            for (i = 1; i <= nbP3d; i++)
            {
                C.D1(i, 1.0, out myP, out myV);
                V[j] = myV.X();
                V[j + 1] = myV.Y();
                V[j + 2] = myV.Z();
                j += 3;
            }
            j = nbP3d * 3 + 1;
            for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
            {
                C.D1(i, 1.0, out myP2d, out myV2d);
                V[j] = myV2d.X();
                V[j + 1] = myV2d.Y();
                j += 2;
            }
        }
    }

    private double SearchFirstLambda(MultiLine Line, math_Vector aPar, NCollection_Array1<double> Theknots, math_Vector V,
                                     int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        Point3D P1, P2;
        Point2D P12d, P22d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> tabP1 = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<Point2D> tabP12d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP1, tabP12d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP12d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index, tabP1);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index + 1, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index + 1, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index + 1, tabP2);
        }

        double U1 = aPar[index], U2 = aPar[index + 1];
        double lambda, S;
        int low = V.Lower();
        int nbknots = Theknots.Length();

        if (nbP3d != 0)
        {
            P1 = tabP1[1];
            P2 = tabP2[1];
            Vector3D P1P2 = new(P1, P2), myV = new();
            myV.SetCoord(V[low], V[low + 1], V[low + 2]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            P12d = tabP12d[1];
            P22d = tabP22d[1];
            Vector2D P1P2 = new(P12d, P22d), myV = new();
            myV.SetCoord(V[low], V[low + 1]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        return ((S * lambda) * (Theknots[2] - Theknots[1]) / (Theknots[nbknots] - Theknots[1]));
    }

    private double SearchLastLambda(MultiLine Line, math_Vector aPar, NCollection_Array1<double> Theknots, math_Vector V,
                                    int index)
    {
        // dq/dw = lambda* V = (p2-p1)/(u2-u1)

        int nbP2d, nbP3d;
        Point3D P1, P2;
        Point2D P12d, P22d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> tabP = new(1, mynbP3d), tabP2 = new(1, mynbP3d);
        NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d), tabP22d = new(1, mynbP2d);

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index - 1, tabP, tabP2d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index - 1, tabP2d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index - 1, tabP);
        }

        if (nbP3d != 0 && nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP2, tabP22d);
        }
        else if (nbP2d != 0)
        {
            LineTool.Value(Line, index, tabP22d);
        }
        else if (nbP3d != 0)
        {
            LineTool.Value(Line, index, tabP2);
        }

        double U1 = aPar[index - 1], U2 = aPar[index];
        double lambda, S;
        int low = V.Lower();
        int nbknots = Theknots.Length();
        if (nbP3d != 0)
        {
            P1 = tabP[1];
            P2 = tabP2[1];
            Vector3D P1P2 = new(P1, P2), myV = new();
            myV.SetCoord(V[low], V[low + 1], V[low + 2]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }
        else
        {
            P12d = tabP2d[1];
            P22d = tabP22d[1];
            Vector2D P1P2 = new(P12d, P22d), myV = new();
            myV.SetCoord(V[low], V[low + 1]);
            lambda = (P1P2.Magnitude()) / (myV.Magnitude() * (U2 - U1));
            S = (P1P2.Dot(myV) > 0.0) ? 1.0 : -1.0;
        }

        return ((S * lambda) * (Theknots[nbknots] - Theknots[nbknots - 1])
                / (Theknots[nbknots] - Theknots[1]));
    }

    /// <summary>
    /// Approximates <paramref name="Line"/> from the parameters <paramref name="Parameters"/> until the tolerances
    /// are reached, from degree <paramref name="degreemin"/> to <paramref name="degreemax"/>, adding knots if
    /// <paramref name="cutting"/>. With <paramref name="Squares"/>, without iterations.
    /// </summary>
    public Approx_BSplComputeLine(MultiLine Line, math_Vector Parameters, int degreemin = 4, int degreemax = 8,
                                  double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06, int NbIterations = 5,
                                  bool cutting = true, bool Squares = false)
    {
        myfirstParam = new NCollection_HArray1<double>(Parameters.Lower(), Parameters.Upper());
        for (int i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myfirstParam.SetValue(i, Parameters[i]);
        }
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = Approx_IsoParametric;
        myPeriodic = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        alldone = false;
        mycont = -1;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        myhasknots = false;
        myhasmults = false;
        currenttol3d = currenttol2d = RealLast();
        tolreached = false;
        Perform(Line);
    }

    /// <summary>Initialises the fields of the algorithm, starting from the parameters <paramref name="Parameters"/>.</summary>
    public Approx_BSplComputeLine(math_Vector Parameters, int degreemin = 4, int degreemax = 8,
                                  double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06, int NbIterations = 5,
                                  bool cutting = true, bool Squares = false)
    {
        myfirstParam = new NCollection_HArray1<double>(Parameters.Lower(), Parameters.Upper());
        for (int i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myfirstParam.SetValue(i, Parameters[i]);
        }
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = Approx_IsoParametric;
        myPeriodic = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        alldone = false;
        myhasknots = false;
        myhasmults = false;
        mycont = -1;
        currenttol3d = currenttol2d = RealLast();
        tolreached = false;
    }

    /// <summary>Initialises the fields of the algorithm; <see cref="Perform"/> runs it.</summary>
    public Approx_BSplComputeLine(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03,
                                  double Tolerance2d = 1.0e-06, int NbIterations = 5, bool cutting = true,
                                  Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        Par = parametrization;
        myPeriodic = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        alldone = false;
        myhasknots = false;
        myhasmults = false;
        mycont = -1;
        currenttol3d = currenttol2d = RealLast();
        tolreached = false;
    }

    /// <summary>
    /// Approximates <paramref name="Line"/> until the tolerances are reached, from degree
    /// <paramref name="degreemin"/> to <paramref name="degreemax"/>, adding knots if <paramref name="cutting"/>.
    /// With <paramref name="Squares"/>, without iterations. The multiplicities of the interior knots are set by
    /// default.
    /// </summary>
    public Approx_BSplComputeLine(MultiLine Line, int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-3,
                                  double Tolerance2d = 1.0e-6, int NbIterations = 5, bool cutting = true,
                                  Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        myConstraints = new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 2);
        alldone = false;
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
        Par = parametrization;
        myPeriodic = false;
        myfirstC = AppParCurves_TangencyPoint;
        mylastC = AppParCurves_TangencyPoint;
        myhasknots = false;
        myhasmults = false;
        mycont = -1;
        currenttol3d = currenttol2d = RealLast();
        tolreached = false;
        Perform(Line);
    }

    /// <summary>Runs the algorithm on <paramref name="Line"/> after the fields have been initialised.</summary>
    public void Perform(MultiLine Line)
    {
        int i, Thefirstpt, Thelastpt;
        bool Finish = false, begin = true;

        // Search for the actual constraints given by the Line:
        FindRealConstraints(Line);

        Thefirstpt = LineTool.FirstPoint(Line);
        Thelastpt = LineTool.LastPoint(Line);
        int myfirstpt = Thefirstpt;
        int mylastpt = Thelastpt;

        AppParCurves_ConstraintCouple myCouple1 = new(myfirstpt, realfirstC);
        AppParCurves_ConstraintCouple myCouple2 = new(mylastpt, reallastC);
        myConstraints.SetValue(1, myCouple1);
        myConstraints.SetValue(2, myCouple2);

        math_Vector TheParam = new(Thefirstpt, Thelastpt, 0.0);
        if (myfirstParam is null)
        {
            Parameters(Line, Thefirstpt, Thelastpt, TheParam);
        }
        else
        {
            for (i = myfirstParam.Lower(); i <= myfirstParam.Upper(); i++)
            {
                TheParam[i + Thefirstpt - 1] = myfirstParam.Value(i);
            }
        }

        myParameters = new NCollection_HArray1<double>(TheParam.Lower(), TheParam.Upper());
        for (i = TheParam.Lower(); i <= TheParam.Upper(); i++)
        {
            myParameters.SetValue(i, TheParam[i]);
        }
        int nbknots = 2;
        double l;
        alldone = false;

        if (!mycut)
        {
            // Case where no additional knots are desired.
            // ==================================================

            if (!myhasknots)
            {
                NCollection_Array1<double> theknots = new(1, 2);
                NCollection_Array1<int> themults = new(1, 2);
                theknots[1] = 0.0;
                theknots[2] = 1.0;
                alldone = Compute(Line, myfirstpt, mylastpt, TheParam, theknots, themults);
            }
            else
            {
                if (!myhasmults)
                {
                    NCollection_Array1<int> themults = new(1, myknots!.Length());
                    alldone = Compute(Line, myfirstpt, mylastpt, TheParam, myknots.Array1(), themults);
                }
                else
                {
                    alldone = Compute(Line, myfirstpt, mylastpt, TheParam, myknots!.Array1(), mymults!.ChangeArray1());
                }
            }
        }
        else
        {
            // cas ou on va iterer a partir de noeuds donnes par l'utilisateur
            // ou a partir d'une bezier.
            // ================================================================

            while (!Finish)
            {
                currenttol3d = currenttol2d = RealLast();

                if (myhasknots && begin)
                {
                    if (!myhasmults)
                    {
                        // 1st case: the user provides starting knots but
                        // it is up to us to set the multiplicities according to the
                        // desired continuity.
                        // ========================================================

                        NCollection_Array1<int> TheMults = new(1, myknots!.Length());
                        alldone = Compute(Line, myfirstpt, mylastpt, TheParam, myknots.Array1(), TheMults);
                    }
                    else
                    {
                        // 2nd case: the user provides starting knots
                        // with their multiplicities.
                        // ===================================================

                        alldone = Compute(Line, myfirstpt, mylastpt, TheParam, myknots!.Array1(), mymults!.ChangeArray1());
                    }
                    begin = false;
                }
                else
                {
                    // 3eme cas: l'utilisateur ne donne aucun noeuds de depart
                    // ========================================================

                    NCollection_Array1<double> Theknots = new(1, nbknots);
                    NCollection_Array1<int> TheMults = new(1, nbknots);
                    Theknots[1] = 0.0;
                    Theknots[nbknots] = 1.0;
                    for (i = 2; i <= nbknots - 1; i++)
                    {
                        l = (mylastpt - myfirstpt) * (double)(i - 1) / (double)(nbknots - 1);
                        int ll = (int)(l);
                        double a = l - ll;
                        double p1 = TheParam[ll + myfirstpt];
                        double p2 = TheParam[ll + 1 + myfirstpt];
                        Theknots[i] = (1.0 - a) * p1 + a * p2;
                    }

                    alldone = Compute(Line, myfirstpt, mylastpt, TheParam, Theknots, TheMults);
                }

                if (!alldone)
                {
                    nbknots++;
                }
                else
                {
                    Finish = true;
                }
            }
        }
    }

    /// <summary>
    /// The new parameters of the points, corresponding to the MultiBSpCurve: the object's own array (shared: don't
    /// modify).
    /// </summary>
    public NCollection_Array1<double> Parameters() => myParameters!.Array1();

    /// <summary>The result of the approximation: the object's own MultiBSpCurve (shared).</summary>
    public AppParCurves_MultiBSpCurve Value() => TheMultiBSpCurve;

    /// <summary>The result of the approximation, as <see cref="Value"/> (OCCT's modifiable reference).</summary>
    public AppParCurves_MultiBSpCurve ChangeValue() => TheMultiBSpCurve;

    // Computes new parameters between firstP and lastP.
    private void Parameters(MultiLine Line, int firstP, int lastP, math_Vector TheParameters)
    {
        int i, j, nbP2d, nbP3d;
        double dist;
        int aNbp = lastP - firstP + 1;

        // The first parameter should always be zero according to all the logic below,
        // so division by any value will give zero anyway, so it should never be scaled
        // to avoid case when there is only one parameter in the array thus division by zero happens.
        TheParameters[firstP] = 0.0;
        if (aNbp == 2)
        {
            TheParameters[lastP] = 1.0;
        }
        else if (Par == Approx_ChordLength || Par == Approx_Centripetal)
        {
            nbP3d = LineTool.NbP3d(Line);
            nbP2d = LineTool.NbP2d(Line);
            int mynbP3d = nbP3d, mynbP2d = nbP2d;
            if (nbP3d == 0)
            {
                mynbP3d = 1;
            }
            if (nbP2d == 0)
            {
                mynbP2d = 1;
            }

            NCollection_Array1<Point3D> tabP = new(1, mynbP3d);
            NCollection_Array1<Point3D> tabPP = new(1, mynbP3d);
            NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);
            NCollection_Array1<Point2D> tabPP2d = new(1, mynbP2d);

            for (i = firstP + 1; i <= lastP; i++)
            {
                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(Line, i - 1, tabP);
                }

                if (nbP3d != 0 && nbP2d != 0)
                {
                    LineTool.Value(Line, i, tabPP, tabPP2d);
                }
                else if (nbP2d != 0)
                {
                    LineTool.Value(Line, i, tabPP2d);
                }
                else if (nbP3d != 0)
                {
                    LineTool.Value(Line, i, tabPP);
                }
                dist = 0.0;
                for (j = 1; j <= nbP3d; j++)
                {
                    Point3D aP1 = tabP[j], aP2 = tabPP[j];
                    dist += aP2.SquareDistance(aP1);
                }
                for (j = 1; j <= nbP2d; j++)
                {
                    Point2D aP12d = tabP2d[j], aP22d = tabPP2d[j];

                    dist += aP22d.SquareDistance(aP12d);
                }

                dist = Math.Sqrt(dist);
                if (Par == Approx_ChordLength)
                {
                    TheParameters[i] = TheParameters[i - 1] + dist;
                }
                else
                { // Par == Approx_Centripetal
                    TheParameters[i] = TheParameters[i - 1] + Math.Sqrt(dist);
                }
            }
            for (i = firstP + 1; i <= lastP; i++)
            {
                TheParameters[i] /= TheParameters[lastP];
            }
        }
        else
        {
            for (i = firstP + 1; i <= lastP; i++)
            {
                TheParameters[i] = ((double)i - firstP) / ((double)(lastP - (double)firstP));
            }
        }
    }

    private bool Compute(MultiLine Line, int fpt, int lpt, math_Vector Para, NCollection_Array1<double> Knots,
                         NCollection_Array1<int> Mults)
    {
        int i, deg, nbpoles, multinter;
        bool mydone;
        double Fv, TheTol3d, TheTol2d, l1, l2;
        int nbp = lpt - fpt + 1;
        mylambda1 = 0.0;
        mylambda2 = 0.0;

        math_Vector aParams = new(Para.Lower(), Para.Upper());

        for (deg = mydegremin; deg <= mydegremax; deg++)
        {
            aParams.Initialized(Para);

            if (!myhasmults)
            {
                Mults[Mults.Lower()] = deg + 1;
                Mults[Mults.Upper()] = deg + 1;
                nbpoles = deg + 1;
                if (mycont == -1)
                {
                    multinter = 1;
                }
                else
                {
                    multinter = Max(1, deg - mycont);
                }
                for (i = Mults.Lower() + 1; i <= Mults.Upper() - 1; i++)
                {
                    Mults[i] = multinter;
                    nbpoles += multinter;
                }
            }
            else
            {
                nbpoles = -deg - 1;
                for (i = Mults.Lower(); i <= Mults.Upper(); i++)
                {
                    nbpoles += Mults.Value(i);
                }
            }

            int nbpolestocompare = nbpoles;
            if (realfirstC == AppParCurves_TangencyPoint)
            {
                nbpolestocompare++;
            }
            if (reallastC == AppParCurves_TangencyPoint)
            {
                nbpolestocompare++;
            }
            if (realfirstC == AppParCurves_CurvaturePoint)
            {
                nbpolestocompare++;
            }
            if (reallastC == AppParCurves_CurvaturePoint)
            {
                nbpolestocompare++;
            }
            if (nbpolestocompare > nbp)
            {
                Interpol(Line);
                tolreached = true;
                return true;
            }

            AppParCurves_MultiBSpCurve mySCU;

            if (mysquares)
            {
                AppParCurves_LeastSquare<MultiLine, LineTool> SQ =
                    new(Line, Knots, Mults, fpt, lpt, realfirstC, reallastC, aParams, nbpoles);
                mydone = SQ.IsDone();
                if (mydone)
                {
                    mySCU = new AppParCurves_MultiBSpCurve(SQ.BSplineValue());
                    SQ.Error(out Fv, out TheTol3d, out TheTol2d);
                }
                else
                {
                    continue;
                }
            }
            else
            {
                if (nbpoles != deg + 1)
                {
                    if (deg == mydegremin
                        && (realfirstC >= AppParCurves_TangencyPoint
                            || reallastC >= AppParCurves_TangencyPoint))
                    {
                        AppParCurves_LeastSquare<MultiLine, LineTool> thefitt =
                            new(Line, Knots, Mults, fpt, lpt, realfirstC, reallastC, aParams, nbpoles);
                        mylambda1 = thefitt.FirstLambda() * deg;
                        mylambda2 = thefitt.LastLambda() * deg;
                    }
                    l1 = mylambda1 / deg;
                    l2 = mylambda2 / deg;

                    AppParCurves_BSpGradient<MultiLine, LineTool> GRAD = new(Line, fpt, lpt, myConstraints, aParams, Knots,
                                                                            Mults, deg, mytol3d, mytol2d, myitermax,
                                                                            l1, l2);

                    mydone = GRAD.IsDone();
                    if (mydone)
                    {
                        mySCU = GRAD.Value();
                        TheTol3d = GRAD.MaxError3d();
                        TheTol2d = GRAD.MaxError2d();
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    AppParCurves_Gradient<MultiLine, LineTool> GRAD2 =
                        new(Line, fpt, lpt, myConstraints, aParams, deg, mytol3d, mytol2d, myitermax);
                    mydone = GRAD2.IsDone();
                    if (mydone)
                    {
                        if (GRAD2.Value().NbCurves() == 0)
                        {
                            continue;
                        }
                        mySCU = new AppParCurves_MultiBSpCurve(GRAD2.Value(), Knots, Mults);
                        TheTol3d = GRAD2.MaxError3d();
                        TheTol2d = GRAD2.MaxError2d();
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            bool save = true;

            for (i = aParams.Lower(); i <= aParams.Upper(); i++)
            {
                if (aParams[i] <= -0.000001 || aParams[i] >= 1.000001)
                {
                    save = false;
                    break;
                }
            }

            if (mydone)
            {
                if (TheTol3d <= mytol3d && TheTol2d <= mytol2d)
                {
                    // Stockage de la multicurve approximee.
                    tolreached = true;
                    TheMultiBSpCurve = mySCU;
                    currenttol3d = TheTol3d;
                    currenttol2d = TheTol2d;
                    if (save)
                    {
                        for (i = aParams.Lower(); i <= aParams.Upper(); i++)
                        {
                            myParameters!.SetValue(i, aParams[i]);
                        }
                    }
                    return true;
                }
            }

            if (TheTol3d <= currenttol3d && TheTol2d <= currenttol2d)
            {
                TheMultiBSpCurve = mySCU;
                currenttol3d = TheTol3d;
                currenttol2d = TheTol2d;
                if (save)
                {
                    for (i = aParams.Lower(); i <= aParams.Upper(); i++)
                    {
                        myParameters!.SetValue(i, aParams[i]);
                    }
                }
            }
        }

        return false;
    }

    /// <summary>The approximation will start from the parameters <paramref name="ThePar"/> (copied).</summary>
    public void SetParameters(math_Vector ThePar)
    {
        myfirstParam = new NCollection_HArray1<double>(ThePar.Lower(), ThePar.Upper());
        for (int i = ThePar.Lower(); i <= ThePar.Upper(); i++)
        {
            myfirstParam.SetValue(i, ThePar[i]);
        }
    }

    /// <summary>
    /// The approximation will use the knots <paramref name="Knots"/> (copied); the multiplicities follow from the
    /// degree and the continuity.
    /// </summary>
    public void SetKnots(NCollection_Array1<double> Knots)
    {
        myhasknots = true;
        myknots = new NCollection_HArray1<double>(Knots.Lower(), Knots.Upper());
        for (int i = Knots.Lower(); i <= Knots.Upper(); i++)
        {
            myknots.SetValue(i, Knots[i]);
        }
    }

    /// <summary>The approximation will use the knots <paramref name="Knots"/> and multiplicities <paramref name="Mults"/> (copied).</summary>
    public void SetKnotsAndMultiplicities(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults)
    {
        myhasknots = true;
        myhasmults = true;
        int i;
        myknots = new NCollection_HArray1<double>(Knots.Lower(), Knots.Upper());
        for (i = Knots.Lower(); i <= Knots.Upper(); i++)
        {
            myknots.SetValue(i, Knots[i]);
        }
        mymults = new NCollection_HArray1<int>(Mults.Lower(), Mults.Upper());
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            mymults.SetValue(i, Mults[i]);
        }
    }

    /// <summary>Initialises the fields of the algorithm.</summary>
    public void Init(int degreemin = 4, int degreemax = 8, double Tolerance3d = 1.0e-03, double Tolerance2d = 1.0e-06,
                     int NbIterations = 5, bool cutting = true,
                     Approx_ParametrizationType parametrization = Approx_ChordLength, bool Squares = false)
    {
        mydegremin = degreemin;
        mydegremax = degreemax;
        mytol3d = Tolerance3d;
        mytol2d = Tolerance2d;
        Par = parametrization;
        mysquares = Squares;
        mycut = cutting;
        myitermax = NbIterations;
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

    /// <summary>Changes the constraints at the first and the last points.</summary>
    public void SetConstraints(AppParCurves_Constraint firstC, AppParCurves_Constraint lastC)
    {
        myfirstC = firstC;
        mylastC = lastC;
    }

    /// <summary>
    /// If true, tries to build a periodic MultiCurve, with C1 conditions at the first and the last points. The
    /// MultiLine must be closed.
    /// </summary>
    public void SetPeriodic(bool thePeriodic)
    {
        myPeriodic = thePeriodic;
    }

    /// <summary>False if, during the approximation, the line tool returned NoApproximation when more points were needed.</summary>
    public bool IsAllApproximated() => alldone;

    /// <summary>False if the line tool returned NoPointsAdded.</summary>
    public bool IsToleranceReached() => tolreached;

    /// <summary>The 3d and 2d tolerances of the MultiBSpCurve.</summary>
    public void Error(out double tol3d, out double tol2d)
    {
        tol3d = currenttol3d;
        tol2d = currenttol2d;
    }

    /// <summary>Sets the continuity of the spline: with C = 2, it is C2.</summary>
    public void SetContinuity(int C)
    {
        mycont = C;
    }

    private void FindRealConstraints(MultiLine Line)
    {
        realfirstC = myfirstC;
        reallastC = mylastC;
        int nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);
        bool Ok = false;
        NCollection_Array1<Vector3D> TabV = new(1, Max(1, nbP3d));
        NCollection_Array1<Vector2D> TabV2d = new(1, Max(1, nbP2d));
        int Thefirstpt = LineTool.FirstPoint(Line);
        int Thelastpt = LineTool.LastPoint(Line);

        if (myfirstC >= AppParCurves_TangencyPoint)
        {
            if (nbP3d != 0 && nbP2d != 0)
            {
                Ok = LineTool.Tangency(Line, Thefirstpt, TabV, TabV2d);
            }
            else if (nbP2d != 0)
            {
                Ok = LineTool.Tangency(Line, Thefirstpt, TabV2d);
            }
            else if (nbP3d != 0)
            {
                Ok = LineTool.Tangency(Line, Thefirstpt, TabV);
            }

            realfirstC = AppParCurves_PassPoint;
            if (Ok)
            {
                realfirstC = AppParCurves_TangencyPoint;
                if (myfirstC == AppParCurves_CurvaturePoint)
                {
                    // OCCT tests the tangents again, not the curvatures (occt-issues.md).
                    if (nbP3d != 0 && nbP2d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thefirstpt, TabV, TabV2d);
                    }
                    else if (nbP2d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thefirstpt, TabV2d);
                    }
                    else if (nbP3d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thefirstpt, TabV);
                    }
                    if (Ok)
                    {
                        realfirstC = AppParCurves_CurvaturePoint;
                    }
                }
            }
        }

        if (mylastC >= AppParCurves_TangencyPoint)
        {
            if (nbP3d != 0 && nbP2d != 0)
            {
                Ok = LineTool.Tangency(Line, Thelastpt, TabV, TabV2d);
            }
            else if (nbP2d != 0)
            {
                Ok = LineTool.Tangency(Line, Thelastpt, TabV2d);
            }
            else if (nbP3d != 0)
            {
                Ok = LineTool.Tangency(Line, Thelastpt, TabV);
            }

            reallastC = AppParCurves_PassPoint;
            if (Ok)
            {
                reallastC = AppParCurves_TangencyPoint;
                if (mylastC == AppParCurves_CurvaturePoint)
                {
                    // OCCT tests the tangents again, not the curvatures (occt-issues.md).
                    if (nbP3d != 0 && nbP2d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thelastpt, TabV, TabV2d);
                    }
                    else if (nbP2d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thelastpt, TabV2d);
                    }
                    else if (nbP3d != 0)
                    {
                        Ok = LineTool.Tangency(Line, Thelastpt, TabV);
                    }
                    if (Ok)
                    {
                        reallastC = AppParCurves_CurvaturePoint;
                    }
                }
            }
        }
    }

    /// <summary>Interpolates the points of <paramref name="Line"/> by a C2 cubic B-spline.</summary>
    public void Interpol(MultiLine Line)
    {
        int i, Thefirstpt, Thelastpt, deg = 3;
        mycont = 2;
        Thefirstpt = LineTool.FirstPoint(Line);
        Thelastpt = LineTool.LastPoint(Line);
        math_Vector TheParam = new(Thefirstpt, Thelastpt, 0.0);
        // Par = Approx_ChordLength;
        if (myfirstParam is null)
        {
            Parameters(Line, Thefirstpt, Thelastpt, TheParam);
        }
        else
        {
            for (i = myfirstParam.Lower(); i <= myfirstParam.Upper(); i++)
            {
                TheParam[i + Thefirstpt - 1] = myfirstParam.Value(i);
            }
        }
        AppParCurves_Constraint Cons = AppParCurves_TangencyPoint;
        double lambda1, lambda2;
        double Fv;

        // Recherche du nombre de noeuds.
        int nbknots, nbpoles, nbpoints;
        nbpoints = Thelastpt - Thefirstpt + 1;

        if (nbpoints == 2)
        {
            Cons = AppParCurves_NoConstraint;
            int mydeg = 1;
            AppParCurves_LeastSquare<MultiLine, LineTool> LSQ = new(Line, Thefirstpt, Thelastpt, Cons, Cons, TheParam, mydeg + 1);
            alldone = LSQ.IsDone();
            NCollection_Array1<double> TheKnots = new(1, 2);
            NCollection_Array1<int> TheMults = new(1, 2);
            TheKnots[1] = TheParam[Thefirstpt];
            TheKnots[2] = TheParam[Thelastpt];
            TheMults[1] = TheMults[2] = mydeg + 1;
            TheMultiBSpCurve = new AppParCurves_MultiBSpCurve(LSQ.BezierValue(), TheKnots, TheMults);
            LSQ.Error(out Fv, out currenttol3d, out currenttol2d);
        }
        else
        {
            nbpoles = nbpoints + 2;
            nbknots = nbpoints;

            // Resolution:
            NCollection_Array1<double> Theknots = new(1, nbknots);
            Theknots[1] = TheParam[Thefirstpt];
            Theknots[nbknots] = TheParam[Thelastpt];
            NCollection_Array1<int> TheMults = new(1, nbknots);
            TheMults[1] = deg + 1;
            TheMults[nbknots] = deg + 1;

            int low = TheParam.Lower();
            for (i = 2; i <= nbknots - 1; i++)
            {
                Theknots[i] = TheParam[i + low - 1];
                TheMults[i] = 1;
            }

            int nbP = 3 * LineTool.NbP3d(Line) + 2 * LineTool.NbP2d(Line);
            math_Vector V1 = new(1, nbP), V2 = new(1, nbP);

            if (nbpoints == 3 || nbpoints == 4)
            {
                FirstTangencyVector(Line, Thefirstpt, V1);
                lambda1 = SearchFirstLambda(Line, TheParam, Theknots, V1, Thefirstpt);

                LastTangencyVector(Line, Thelastpt, V2);
                lambda2 = SearchLastLambda(Line, TheParam, Theknots, V2, Thelastpt);

                lambda1 = lambda1 / deg;
                lambda2 = lambda2 / deg;
            }
            else
            {
                int nnpol, nnp = Min(nbpoints, 9);
                nnpol = nnp;
                int lastp = Min(Thelastpt, Thefirstpt + nnp - 1);
                double U;
                AppParCurves_LeastSquare<MultiLine, LineTool> SQ1 = new(Line, Thefirstpt, lastp, Cons, Cons, nnpol);

                math_Vector P1 = new(Thefirstpt, lastp);
                for (i = Thefirstpt; i <= lastp; i++)
                {
                    P1[i] = TheParam[i];
                }
                SQ1.Perform(P1);
                AppParCurves_MultiCurve C1 = SQ1.BezierValue();
                U = 0.0;
                TangencyVector(Line, C1, U, V1);

                int firstp = Max(Thefirstpt, Thelastpt - nnp + 1);

                if (firstp == Thefirstpt && lastp == Thelastpt)
                {
                    U = 1.0;
                    TangencyVector(Line, C1, U, V2);
                }
                else
                {
                    AppParCurves_LeastSquare<MultiLine, LineTool> SQ2 = new(Line, firstp, Thelastpt, Cons, Cons, nnpol);

                    math_Vector P2 = new(firstp, Thelastpt);
                    for (i = firstp; i <= Thelastpt; i++)
                    {
                        P2[i] = TheParam[i];
                    }
                    SQ2.Perform(P2);
                    AppParCurves_MultiCurve C2 = SQ2.BezierValue();
                    U = 1.0;
                    TangencyVector(Line, C2, U, V2);
                }

                lambda1 = 1.0 / deg;
                lambda1 = lambda1 * (Theknots[2] - Theknots[1]) / (Theknots[nbknots] - Theknots[1]);
                lambda2 = 1.0 / deg;
                lambda2 = lambda2 * (Theknots[nbknots] - Theknots[nbknots - 1]) / (Theknots[nbknots] - Theknots[1]);
            }

            if (myPeriodic)
            {
                V1.Initialized(0.5 * (V1 + V2));
                V2.Initialized(V1);
            }

            AppParCurves_LeastSquare<MultiLine, LineTool> SQ = new(Line, Theknots, TheMults, Thefirstpt, Thelastpt, Cons, Cons,
                                                                  nbpoles);

            SQ.Perform(TheParam, V1, V2, lambda1, lambda2);
            alldone = SQ.IsDone();
            TheMultiBSpCurve = new AppParCurves_MultiBSpCurve(SQ.BSplineValue());
            SQ.Error(out Fv, out currenttol3d, out currenttol2d);
            tolreached = true;
        }
        myParameters = new NCollection_HArray1<double>(TheParam.Lower(), TheParam.Upper());
        for (i = TheParam.Lower(); i <= TheParam.Upper(); i++)
        {
            myParameters.SetValue(i, TheParam[i]);
        }
    }

    private static void TangencyVector(MultiLine Line, AppParCurves_MultiCurve C, double U, math_Vector V)
    {
        int i, j, nbP2d, nbP3d;
        nbP3d = LineTool.NbP3d(Line);
        nbP2d = LineTool.NbP2d(Line);

        Point3D myP;
        Vector3D myV;
        Point2D myP2d;
        Vector2D myV2d;
        j = 1;
        for (i = 1; i <= nbP3d; i++)
        {
            C.D1(i, U, out myP, out myV);
            V[j] = myV.X();
            V[j + 1] = myV.Y();
            V[j + 2] = myV.Z();
            j += 3;
        }
        j = nbP3d * 3 + 1;
        for (i = nbP3d + 1; i <= nbP3d + nbP2d; i++)
        {
            C.D1(i, U, out myP2d, out myV2d);
            V[j] = myV2d.X();
            V[j + 1] = myV2d.Y();
            j += 2;
        }
    }
}
