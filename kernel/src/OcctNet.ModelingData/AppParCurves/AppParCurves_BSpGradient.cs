// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_BSpGradient.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyBSplGradientOfBSplineCompute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 11/09/91

// Application of the corrected gradient method to minimize
// F  = sum(||C(ui, Poles(ui)) - ptli||2.
// The conjugate gradient method is implemented in the mathematical
// library: math_BFGS.

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// Approximates a MultiLine by B-spline curves with given knots, optimising the points' parameters: one
/// projection iteration (Rogers and Fog), then a BFGS minimisation of
/// <see cref="AppParCurves_BSpFunction{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_BSpGradient.gxx</c> (decision 1202), instantiated by OCCT as
/// AppDef_MyBSplGradientOfBSplineCompute and the GeomInt and BRepApprox equivalents. Its <c>BSpParLeastSquare</c>,
/// <c>BSpParFunction</c> and <c>BSpGradient_BFGS</c> arguments are always the LeastSquare, BSpFunction and
/// BSpGradient_BFGS instantiations over the same MultiLine and tool. The computation runs in the constructor, which
/// also writes the optimised parameters into <c>Parameters</c>.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_BSpGradient<MultiLine, ToolLine>
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    private AppParCurves_MultiBSpCurve SCU = new();
    private readonly math_Vector ParError;
    private double AvError;
    private double MError3d;
    private double MError2d;
    private double mylambda1;
    private double mylambda2;
    private readonly bool myIsLambdaDefined;
    private bool Done;

    private static AppParCurves_Constraint FirstConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                           int FirstPoint)
    {
        int i, myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons = AppParCurves_NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == FirstPoint)
            {
                break;
            }
        }
        return Cons;
    }

    private static AppParCurves_Constraint LastConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                          int LastPoint)
    {
        int i, myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons = AppParCurves_NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == LastPoint)
            {
                break;
            }
        }
        return Cons;
    }

    /// <summary>
    /// Approximates the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/> by B-spline curves of degree <paramref name="Deg"/> with the knots
    /// <paramref name="Knots"/> and multiplicities <paramref name="Mults"/> under the constraints
    /// <paramref name="TheConstraints"/>, from the parameters <paramref name="Parameters"/>, which receive the
    /// optimised parameters. End tangency values are computed by a first least-squares fit.
    /// </summary>
    public AppParCurves_BSpGradient(MultiLine SSP, int FirstPoint, int LastPoint,
                                    NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Vector Parameters,
                                    NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int Deg, double Tol3d,
                                    double Tol2d, int NbIterations = 1)
    {
        ParError = new math_Vector(FirstPoint, LastPoint, 0.0);
        mylambda1 = 0.0;
        mylambda2 = 0.0;
        myIsLambdaDefined = false;
        Perform(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Knots, Mults, Deg, Tol3d, Tol2d, NbIterations);
    }

    /// <summary>As the other constructor, with the end tangency values <paramref name="lambda1"/> and <paramref name="lambda2"/> given.</summary>
    public AppParCurves_BSpGradient(MultiLine SSP, int FirstPoint, int LastPoint,
                                    NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Vector Parameters,
                                    NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int Deg, double Tol3d,
                                    double Tol2d, int NbIterations, double lambda1, double lambda2)
    {
        ParError = new math_Vector(FirstPoint, LastPoint, 0.0);
        mylambda1 = lambda1;
        mylambda2 = lambda2;
        myIsLambdaDefined = true;
        Perform(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Knots, Mults, Deg, Tol3d, Tol2d, NbIterations);
    }

    /// <summary>Runs the approximation (called by the constructors).</summary>
    protected void Perform(MultiLine SSP, int FirstPoint, int LastPoint,
                           NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Vector Parameters,
                           NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int Deg, double Tol3d,
                           double Tol2d, int NbIterations = 200)
    {
        int i, j, k, i2, l;
        double UF, DU, Fval = 0.0, FU, DFU;
        int nbP3d = ToolLine.NbP3d(SSP);
        int nbP2d = ToolLine.NbP2d(SSP);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        int nbP = nbP3d + nbP2d;
        gp_Pnt Pt = new();
        gp_Pnt2d Pt2d = new();
        gp_Vec V1 = new(), MyV;
        gp_Vec2d V12d = new(), MyV2d;
        Done = false;

        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<gp_Pnt> TabP = new(1, mynbP3d);
        NCollection_Array1<gp_Pnt2d> TabP2d = new(1, mynbP2d);

        // Calculation of the function F= sum(||C(ui)-Ptli||2):
        // Call to a function inheriting from MultipleVarFunctionWithGradient
        // to compute F and grad_F.
        // ================================================================

        int nbpoles = -Deg - 1;
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            nbpoles += Mults[i];
        }

        NCollection_Array1<gp_Pnt> TabPole = new(1, nbpoles);
        NCollection_Array1<gp_Pnt2d> TabPole2d = new(1, nbpoles);
        NCollection_Array1<gp_Pnt> ThePoles = new(1, (nbpoles) * mynbP3d);
        NCollection_Array1<gp_Pnt2d> ThePoles2d = new(1, (nbpoles) * mynbP2d);

        AppParCurves_Constraint FirstCons = FirstConstraint(TheConstraints, FirstPoint),
                                LastCons = LastConstraint(TheConstraints, LastPoint);

        AppParCurves_BSpFunction<MultiLine, ToolLine> MyF =
            new(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Knots, Mults, nbpoles);

        if (FirstCons >= AppParCurves_TangencyPoint || LastCons >= AppParCurves_TangencyPoint)
        {
            if (!myIsLambdaDefined)
            {
                AppParCurves_LeastSquare<MultiLine, ToolLine> thefitt =
                    new(SSP, Knots, Mults, FirstPoint, LastPoint, FirstCons, LastCons, Parameters, nbpoles);
                if (FirstCons >= AppParCurves_TangencyPoint)
                {
                    mylambda1 = thefitt.FirstLambda();
                    MyF.SetFirstLambda(mylambda1);
                }
                if (LastCons >= AppParCurves_TangencyPoint)
                {
                    mylambda2 = thefitt.LastLambda();
                    MyF.SetLastLambda(mylambda2);
                }
            }
            else
            {
                MyF.SetFirstLambda(mylambda1);
                MyF.SetLastLambda(mylambda2);
            }
        }

        MyF.Value(Parameters, out Fval);
        MError3d = MyF.MaxError3d();
        MError2d = MyF.MaxError2d();
        SCU = MyF.CurveValue();

        if (MError3d > Tol3d || MError2d > Tol2d)
        {
            // Storage of curve poles for projection:
            // ============================================
            i2 = 0;
            for (k = 1; k <= nbP3d; k++)
            {
                SCU.Curve(k, TabPole);
                for (j = 1; j <= nbpoles; j++)
                {
                    ThePoles[j + i2] = TabPole[j];
                }
                i2 += nbpoles;
            }
            i2 = 0;
            for (k = 1; k <= nbP2d; k++)
            {
                SCU.Curve(nbP3d + k, TabPole2d);
                for (j = 1; j <= nbpoles; j++)
                {
                    ThePoles2d[j + i2] = TabPole2d[j];
                }
                i2 += nbpoles;
            }

            //  Une iteration rapide de projection est faite par la methode de
            //  Rogers & Fog 89, methode equivalente a Hoschek 88 qui ne necessite pas
            //  le calcul de D2.

            math_Matrix A = MyF.FunctionMatrix();
            math_Matrix DA = MyF.DerivativeFunctionMatrix();
            math_IntegerVector Index = MyF.Index();
            double aa, da, a, b, c, d, e, f, px, py, pz;
            int indexdeb, indexfin;

            for (j = FirstPoint + 1; j <= LastPoint - 1; j++)
            {
                UF = Parameters[j];
                if (nbP3d != 0 && nbP2d != 0)
                {
                    ToolLine.Value(SSP, j, TabP, TabP2d);
                }
                else if (nbP2d != 0)
                {
                    ToolLine.Value(SSP, j, TabP2d);
                }
                else
                {
                    ToolLine.Value(SSP, j, TabP);
                }

                FU = 0.0;
                DFU = 0.0;
                i2 = 0;

                indexdeb = Index[j] + 1;
                indexfin = indexdeb + Deg;

                for (k = 1; k <= nbP3d; k++)
                {
                    a = b = c = d = e = f = 0.0;
                    for (l = indexdeb; l <= indexfin; l++)
                    {
                        Pt = ThePoles[l + i2];
                        px = Pt.X();
                        py = Pt.Y();
                        pz = Pt.Z();
                        aa = A[j, l];
                        da = DA[j, l];
                        a += aa * px;
                        d += da * px;
                        b += aa * py;
                        e += da * py;
                        c += aa * pz;
                        f += da * pz;
                    }
                    Pt.SetCoord(a, b, c);
                    V1.SetCoord(d, e, f);
                    i2 += nbpoles;

                    MyV = new gp_Vec(Pt, TabP[k]);
                    FU += MyV * V1;
                    DFU += V1.SquareMagnitude();
                }
                i2 = 0;
                for (k = 1; k <= nbP2d; k++)
                {
                    a = b = d = e = 0.0;
                    for (l = indexdeb; l <= indexfin; l++)
                    {
                        Pt2d = ThePoles2d[l + i2];
                        px = Pt2d.X();
                        py = Pt2d.Y();
                        aa = A[j, l];
                        da = DA[j, l];
                        a += aa * px;
                        d += da * px;
                        b += aa * py;
                        e += da * py;
                    }
                    Pt2d.SetCoord(a, b);
                    V12d.SetCoord(d, e);
                    i2 += nbpoles;

                    MyV2d = new gp_Vec2d(Pt2d, TabP2d[k]);
                    FU += MyV2d * V12d;
                    DFU += V12d.SquareMagnitude();
                }

                if (DFU >= RealEpsilon())
                {
                    DU = FU / DFU;
                    DU = Math.CopySign(Min(5.0e-02, Math.Abs(DU)), DU);
                    UF += DU;
                    Parameters[j] = UF;
                }
            }

            MyF.Value(Parameters, out Fval);
            MError3d = MyF.MaxError3d();
            MError2d = MyF.MaxError2d();
        }

        if (MError3d <= Tol3d && MError2d <= Tol2d)
        {
            Done = true;
        }
        else if (NbIterations != 0)
        {
            // NbIterations de gradient conjugue:
            // =================================
            double Eps = 1.0e-07;
            AppParCurves_BSpGradient_BFGS<MultiLine, ToolLine> FResol = new(MyF, Parameters, Tol3d, Tol2d, Eps, NbIterations);
        }

        SCU = MyF.CurveValue();

        AvError = 0.0;
        for (j = FirstPoint; j <= LastPoint; j++)
        {
            Parameters[j] = MyF.NewParameters()[j];
            // Recherche des erreurs maxi et moyenne a un index donne:
            for (k = 1; k <= nbP; k++)
            {
                ParError[j] = Max(ParError[j], MyF.Error(j, k));
            }
            AvError += ParError[j];
        }
        AvError = AvError / (LastPoint - FirstPoint + 1);

        MError3d = MyF.MaxError3d();
        MError2d = MyF.MaxError2d();
        if (MError3d <= Tol3d && MError2d <= Tol2d)
        {
            Done = true;
        }
    }

    /// <summary>The approximating curves: a copy sharing the result's poles, knots and multiplicities (OCCT's by-value copy).</summary>
    public AppParCurves_MultiBSpCurve Value() => new(SCU);

    /// <summary>True if the errors are within the tolerances.</summary>
    public bool IsDone() => Done;

    /// <summary>The largest distance at the point <paramref name="Index"/>.</summary>
    public double Error(int Index) => ParError[Index];

    /// <summary>The average of the points' largest distances.</summary>
    public double AverageError() => AvError;

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => MError3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => MError2d;
}
