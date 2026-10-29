// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_Function.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Lpa, le 20/09/91

// Calcul de la valeur de F et grad_F, connaissant le parametrage.
// Cette fonction, appelee par le gradient conjugue, calcul F et
// DF(ui, Poles(ui)) ce qui implique un calcul des nouveaux poles
//  a chaque appel.

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// The function F = sum ||C(ui) - Ptli||^2 of the parameters ui of a MultiLine's points, and its gradient,
/// minimised by <see cref="AppParCurves_Gradient{MultiLine, ToolLine}"/>. Each evaluation computes new poles by
/// least squares (<see cref="AppParCurves_LeastSquare{MultiLine, ToolLine}"/>) and, for interior constraints,
/// corrects them with <see cref="AppParCurves_ResolConstraint{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_Function.gxx</c> (decision 142), whose <c>Squares</c> and
/// <c>ResolCons</c> arguments are always the LeastSquare and ResolConstraint instantiations over the same MultiLine
/// and tool. OCCT instantiates it as AppDef_TheFunction, AppDef_ParFunctionOf... and the GeomInt and BRepApprox
/// equivalents. The MultiLine is kept by reference (OCCT copies it; neither changes it).
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_Function<MultiLine, ToolLine> : math_MultipleVarFunctionWithGradient
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    // OCCT's Done field is written but never read; it is left out
    private readonly MultiLine MyMultiLine;
    private AppParCurves_MultiCurve MyMultiCurve;
    private readonly int Degre;
    private readonly math_Vector myParameters;
    private double FVal;
    private readonly math_Vector ValGrad_F;
    private readonly math_Matrix MyF;
    private readonly math_Matrix PTLX;
    private readonly math_Matrix PTLY;
    private readonly math_Matrix PTLZ;
    private readonly math_Matrix A;
    private readonly math_Matrix DA;
    private readonly AppParCurves_LeastSquare<MultiLine, ToolLine> MyLeastSquare;
    private readonly bool Contraintes;
    private readonly int NbP;
    private readonly int NbCu;
    private readonly int Adeb;
    private readonly int Afin;
    private readonly NCollection_HArray1<int> tabdim;
    private double ERR3d;
    private double ERR2d;
    private readonly int FirstP;
    private readonly int LastP;
    private readonly NCollection_HArray1<AppParCurves_ConstraintCouple> myConstraints;

    /// <summary>
    /// The function for the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/>, approximated by Bezier curves of degree <paramref name="Deg"/> under the constraints
    /// <paramref name="TheConstraints"/>, starting from the parameters <paramref name="Parameters"/> (copied).
    /// </summary>
    public AppParCurves_Function(MultiLine SSP, int FirstPoint, int LastPoint,
                                 NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Vector Parameters,
                                 int Deg)
    {
        MyMultiLine = SSP;
        MyMultiCurve = new AppParCurves_MultiCurve(Deg + 1);
        myParameters = new math_Vector(Parameters.Lower(), Parameters.Upper());
        ValGrad_F = new math_Vector(FirstPoint, LastPoint);
        MyF = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLX = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLY = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLZ = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        A = new math_Matrix(FirstPoint, LastPoint, 1, Deg + 1);
        DA = new math_Matrix(FirstPoint, LastPoint, 1, Deg + 1);
        MyLeastSquare = new AppParCurves_LeastSquare<MultiLine, ToolLine>(SSP,
                                                                          FirstPoint,
                                                                          LastPoint,
                                                                          FirstConstraint(TheConstraints, FirstPoint),
                                                                          LastConstraint(TheConstraints, LastPoint),
                                                                          Deg + 1);

        int i;
        for (i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myParameters[i] = Parameters[i];
        }
        FirstP = FirstPoint;
        LastP = LastPoint;
        myConstraints = TheConstraints;
        NbP = LastP - FirstP + 1;
        Adeb = FirstP;
        Afin = LastP;
        Degre = Deg;
        Contraintes = false;
        int myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == FirstP)
            {
                if ((int)Cons >= 1)
                {
                    Adeb = Adeb + 1;
                }
            }
            else if (myindex == LastP)
            {
                if ((int)Cons >= 1)
                {
                    Afin = Afin - 1;
                }
            }
            else
            {
                if ((int)Cons >= 1)
                {
                    Contraintes = true;
                }
            }
        }

        int nb3d = ToolLine.NbP3d(SSP);
        int nb2d = ToolLine.NbP2d(SSP);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }

        NbCu = nb3d + nb2d;
        tabdim = new NCollection_HArray1<int>(0, NbCu - 1);

        if (Contraintes)
        {
            for (i = 1; i <= NbCu; i++)
            {
                if (i <= nb3d)
                {
                    tabdim.SetValue(i - 1, 3);
                }
                else
                {
                    tabdim.SetValue(i - 1, 2);
                }
            }

            NCollection_Array1<gp_Pnt> TabP = new(1, mynb3d);
            NCollection_Array1<gp_Pnt2d> TabP2d = new(1, mynb2d);

            for (i = FirstP; i <= LastP; i++)
            {
                if (nb3d != 0 && nb2d != 0)
                {
                    ToolLine.Value(SSP, i, TabP, TabP2d);
                }
                else if (nb3d != 0)
                {
                    ToolLine.Value(SSP, i, TabP);
                }
                else
                {
                    ToolLine.Value(SSP, i, TabP2d);
                }
                for (int j = 1; j <= NbCu; j++)
                {
                    if (tabdim.Value(j - 1) == 3)
                    {
                        TabP[j].Coord(out PTLX[i, j], out PTLY[i, j], out PTLZ[i, j]);
                    }
                    else
                    {
                        // OCCT: TabP2d(j) where j - nb3d is meant; out of range for a line with 3d and 2d points
                        // (unchecked in OCCT; the port throws)
                        TabP2d[j].Coord(out PTLX[i, j], out PTLY[i, j]);
                    }
                }
            }
        }
    }

    /// <summary>The constraint of the point <paramref name="FirstPoint"/> (the last couple's when it has none).</summary>
    public AppParCurves_Constraint FirstConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
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

    /// <summary>The constraint of the point <paramref name="LastPoint"/> (the last couple's when it has none).</summary>
    public AppParCurves_Constraint LastConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
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

    /// <summary>The value of F at the parameters <paramref name="X"/>; false (and F 0) if the resolution fails.</summary>
    public override bool Value(math_Vector X, out double F)
    {
        myParameters.Initialized(X);

        // Resolution moindres carres:
        // ===========================
        MyLeastSquare.Perform(myParameters);
        if (!(MyLeastSquare.IsDone()))
        {
            F = 0.0;
            return false;
        }
        if (!Contraintes)
        {
            MyLeastSquare.Error(out FVal, out ERR3d, out ERR2d);
            F = FVal;
        }

        // Resolution avec contraintes:
        // ============================
        else
        {
            int Npol = Degre + 1;
            int Ci, i, j, dimen;
            double AA, BB, CC, AIJ, FX, FY, FZ, Fi;
            math_Vector PTCXCI = new(1, Npol), PTCYCI = new(1, Npol), PTCZCI = new(1, Npol);
            ERR3d = ERR2d = 0.0;

            MyMultiCurve = MyLeastSquare.BezierValue();

            A.Initialized(MyLeastSquare.FunctionMatrix());
            AppParCurves_ResolConstraint<MultiLine, ToolLine> Resol = new(MyMultiLine,
                                                                          MyMultiCurve,
                                                                          FirstP,
                                                                          LastP,
                                                                          myConstraints,
                                                                          A,
                                                                          MyLeastSquare.DerivativeFunctionMatrix());
            if (!Resol.IsDone())
            {
                F = 0.0;
                return false;
            }

            // Calcul de F = Sum||C(ui)-Ptli||2  sur toutes les courbes :
            // ========================================================================
            FVal = 0.0;

            for (Ci = 1; Ci <= NbCu; Ci++)
            {
                dimen = tabdim.Value(Ci - 1);
                for (j = 1; j <= Npol; j++)
                {
                    if (dimen == 3)
                    {
                        MyMultiCurve.Value(j).Point(Ci).Coord(out PTCXCI[j], out PTCYCI[j], out PTCZCI[j]);
                    }
                    else
                    {
                        MyMultiCurve.Value(j).Point2d(Ci).Coord(out PTCXCI[j], out PTCYCI[j]);
                    }
                }

                // Calcul de F:
                // ============
                for (i = Adeb; i <= Afin; i++)
                {
                    AA = 0.0;
                    BB = 0.0;
                    CC = 0.0;
                    for (j = 1; j <= Npol; j++)
                    {
                        AIJ = A[i, j];
                        AA += AIJ * PTCXCI[j];
                        BB += AIJ * PTCYCI[j];
                        if (dimen == 3)
                        {
                            CC += AIJ * PTCZCI[j];
                        }
                    }
                    FX = AA - PTLX[i, Ci];
                    FY = BB - PTLY[i, Ci];
                    MyF[i, Ci] = FX * FX + FY * FY;
                    if (dimen == 3)
                    {
                        FZ = CC - PTLZ[i, Ci];
                        MyF[i, Ci] += FZ * FZ;
                        Fi = MyF[i, Ci];
                        if (Math.Sqrt(Fi) > ERR3d)
                        {
                            ERR3d = Math.Sqrt(Fi);
                        }
                    }
                    else
                    {
                        Fi = MyF[i, Ci];
                        if (Math.Sqrt(Fi) > ERR2d)
                        {
                            ERR2d = Math.Sqrt(Fi);
                        }
                    }
                    FVal += Fi;
                }
            }
            F = FVal;
        }
        return true;
    }

    /// <summary>Computes F and its gradient at <paramref name="X"/>.</summary>
    protected void Perform(math_Vector X)
    {
        int j;

        myParameters.Initialized(X);
        // Resolution moindres carres:
        // ===========================
        MyLeastSquare.Perform(myParameters);

        if (!(MyLeastSquare.IsDone()))
        {
            return;
        }

        for (j = myParameters.Lower(); j <= myParameters.Upper(); j++)
        {
            ValGrad_F[j] = 0.0;
        }

        if (!Contraintes)
        {
            MyLeastSquare.ErrorGradient(ValGrad_F, out FVal, out ERR3d, out ERR2d);
        }
        else
        {
            int Pi, Ci, i, k, dimen;
            int Npol = Degre + 1;
            double Scal, AA, BB, CC, DAA, DBB, DCC;
            double FX, FY, FZ, AIJ, DAIJ, px, py, pz, Fi;
            AppParCurves_Constraint Cons = AppParCurves_NoConstraint;
            math_Matrix Grad_F = new(FirstP, LastP, 1, NbCu, 0.0);
            math_Vector PTCXCI = new(1, Npol), PTCYCI = new(1, Npol), PTCZCI = new(1, Npol);
            math_Vector PTCOXCI = new(1, Npol), PTCOYCI = new(1, Npol), PTCOZCI = new(1, Npol);
            ERR3d = ERR2d = 0.0;

            math_Matrix PTCOX = new(1, Npol, 1, NbCu), PTCOY = new(1, Npol, 1, NbCu), PTCOZ = new(1, Npol, 1, NbCu);
            math_Matrix PTCX = new(1, Npol, 1, NbCu), PTCY = new(1, Npol, 1, NbCu), PTCZ = new(1, Npol, 1, NbCu);
            int Inc;

            MyMultiCurve = MyLeastSquare.BezierValue();

            for (Ci = 1; Ci <= NbCu; Ci++)
            {
                dimen = tabdim.Value(Ci - 1);
                for (j = 1; j <= Npol; j++)
                {
                    if (dimen == 3)
                    {
                        MyMultiCurve.Value(j).Point(Ci).Coord(out PTCOX[j, Ci], out PTCOY[j, Ci], out PTCOZ[j, Ci]);
                    }
                    else
                    {
                        MyMultiCurve.Value(j).Point2d(Ci).Coord(out PTCOX[j, Ci], out PTCOY[j, Ci]);
                        PTCOZ[j, Ci] = 0.0;
                    }
                }
            }

            A.Initialized(MyLeastSquare.FunctionMatrix());
            DA.Initialized(MyLeastSquare.DerivativeFunctionMatrix());

            // Resolution avec contraintes:
            // ============================

            AppParCurves_ResolConstraint<MultiLine, ToolLine> Resol = new(MyMultiLine, MyMultiCurve, FirstP, LastP,
                                                                          myConstraints, A, DA);
            if (!Resol.IsDone())
            {
                return;
            }

            // Calcul de F = Sum||C(ui)-Ptli||2 et du gradient non contraint de F pour
            // chaque point PointIndex.
            // ========================================================================
            FVal = 0.0;
            for (j = FirstP; j <= LastP; j++)
            {
                ValGrad_F[j] = 0.0;
            }

            math_Matrix TrA = new(A.LowerCol(), A.UpperCol(), A.LowerRow(), A.UpperRow());
            math_Matrix TrDA = new(DA.LowerCol(), DA.UpperCol(), DA.LowerRow(), DA.UpperRow());
            math_Matrix RESTM = new(A.LowerCol(), A.UpperCol(), A.LowerCol(), A.UpperCol());

            math_Matrix K = Resol.ConstraintMatrix();
            math_Matrix DK = Resol.ConstraintDerivative(MyMultiLine, X, Degre, DA);
            math_Matrix TK = new(K.LowerCol(), K.UpperCol(), K.LowerRow(), K.UpperRow());
            TK.Initialized(K.Transposed());
            math_Vector Vardua = Resol.Duale();
            math_Matrix KK = new(K.LowerCol(), K.UpperCol(), Vardua.Lower(), Vardua.Upper());
            KK.Initialized((K.Transposed()) * (Resol.InverseMatrix()));
            math_Matrix DTK = new(DK.LowerCol(), DK.UpperCol(), DK.LowerRow(), DK.UpperRow());
            DTK.Initialized(DK.Transposed());
            TrA.Initialized(A.Transposed());
            TrDA.Initialized(DA.Transposed());
            RESTM.Initialized((A.Transposed() * A).Inverse());

            math_Vector DPTCO = new(1, K.ColNumber());
            math_Matrix DPTCO1 = new(FirstP, LastP, 1, K.ColNumber());
            math_Vector DKPTC = new(1, K.RowNumber());

            FVal = 0.0;
            for (Ci = 1; Ci <= NbCu; Ci++)
            {
                dimen = tabdim.Value(Ci - 1);
                for (j = 1; j <= Npol; j++)
                {
                    if (dimen == 3)
                    {
                        MyMultiCurve.Value(j).Point(Ci).Coord(out PTCX[j, Ci], out PTCY[j, Ci], out PTCZ[j, Ci]);
                    }
                    else
                    {
                        MyMultiCurve.Value(j).Point2d(Ci).Coord(out PTCX[j, Ci], out PTCY[j, Ci]);
                        PTCZ[j, Ci] = 0.0;
                    }
                }
            }

            // Calcul du gradient sans contraintes:
            // ====================================

            for (Ci = 1; Ci <= NbCu; Ci++)
            {
                dimen = tabdim.Value(Ci - 1);
                for (i = Adeb; i <= Afin; i++)
                {
                    AA = 0.0;
                    BB = 0.0;
                    CC = 0.0;
                    DAA = 0.0;
                    DBB = 0.0;
                    DCC = 0.0;
                    for (j = 1; j <= Npol; j++)
                    {
                        AIJ = A[i, j];
                        DAIJ = DA[i, j];
                        px = PTCX[j, Ci];
                        py = PTCY[j, Ci];
                        AA += AIJ * px;
                        BB += AIJ * py;
                        DAA += DAIJ * px;
                        DBB += DAIJ * py;
                        if (dimen == 3)
                        {
                            pz = PTCZ[j, Ci];
                            CC += AIJ * pz;
                            DCC += DAIJ * pz;
                        }
                    }
                    FX = AA - PTLX[i, Ci];
                    FY = BB - PTLY[i, Ci];
                    MyF[i, Ci] = FX * FX + FY * FY;
                    Grad_F[i, Ci] = 2.0 * (DAA * FX + DBB * FY);
                    if (dimen == 3)
                    {
                        FZ = CC - PTLZ[i, Ci];
                        MyF[i, Ci] += FZ * FZ;
                        Grad_F[i, Ci] += 2.0 * DCC * FZ;
                        Fi = MyF[i, Ci];
                        if (Math.Sqrt(Fi) > ERR3d)
                        {
                            ERR3d = Math.Sqrt(Fi);
                        }
                    }
                    else
                    {
                        Fi = MyF[i, Ci];
                        if (Math.Sqrt(Fi) > ERR2d)
                        {
                            ERR2d = Math.Sqrt(Fi);
                        }
                    }
                    FVal += Fi;
                    ValGrad_F[i] += Grad_F[i, Ci];
                }
            }

            // Calcul de DK*PTC:
            // =================
            for (i = 1; i <= K.RowNumber(); i++)
            {
                Inc = 0;
                for (Ci = 1; Ci <= NbCu; Ci++)
                {
                    dimen = tabdim.Value(Ci - 1);
                    DKPTC[i] = 0.0;
                    for (j = 1; j <= Npol; j++)
                    {
                        DKPTC[i] += DK[i, j + Inc] * PTCX[j, Ci] + DK[i, j + Inc + Npol] * PTCY[j, Ci];
                        if (dimen == 3)
                        {
                            DKPTC[i] += DK[i, j + Inc + 2 * Npol] * PTCZ[j, Ci];
                        }
                    }
                    if (dimen == 3)
                    {
                        Inc = Inc + 3 * Npol;
                    }
                    else
                    {
                        Inc = Inc + 2 * Npol;
                    }
                }
            }

            math_Vector DERR = new(DTK.LowerRow(), DTK.UpperRow());
            DERR.Initialized((DTK) * Vardua - KK * ((DKPTC) + K * (DTK) * Vardua));

            // rajout du gradient avec contraintes:
            // ====================================
            // dPTCO1/duk = [d(TA)/duk*[A*PTCO-PTL] + TA*dA/duk*PTCO]

            Inc = 0;

            math_Vector Errx = new(A.LowerRow(), A.UpperRow());
            math_Vector Erry = new(A.LowerRow(), A.UpperRow());
            math_Vector Errz = new(A.LowerRow(), A.UpperRow());
            math_Vector Scalx = new(DA.LowerRow(), DA.UpperRow());
            math_Vector Scaly = new(DA.LowerRow(), DA.UpperRow());
            math_Vector Scalz = new(DA.LowerRow(), DA.UpperRow());
            math_Vector Erruzax = new(PTCXCI.Lower(), PTCXCI.Upper());
            math_Vector Erruzay = new(PTCYCI.Lower(), PTCYCI.Upper());
            math_Vector Erruzaz = new(PTCZCI.Lower(), PTCZCI.Upper());
            math_Vector TrDAPI = new(TrDA.LowerRow(), TrDA.UpperRow());
            math_Vector TrAPI = new(TrA.LowerRow(), TrA.UpperRow());

            for (Ci = 1; Ci <= NbCu; Ci++)
            {
                dimen = tabdim.Value(Ci - 1);
                PTCOXCI.Initialized(PTCOX.Col(Ci));
                PTCOYCI.Initialized(PTCOY.Col(Ci));
                PTCOZCI.Initialized(PTCOZ.Col(Ci));
                PTCXCI.Initialized(PTCX.Col(Ci));
                PTCYCI.Initialized(PTCY.Col(Ci));
                PTCZCI.Initialized(PTCZ.Col(Ci));

                Errx.Initialized(A * PTCOXCI - PTLX.Col(Ci));
                Erry.Initialized(A * PTCOYCI - PTLY.Col(Ci));
                Errz.Initialized(A * PTCOZCI - PTLZ.Col(Ci));
                Scalx.Initialized(DA * PTCOXCI); // Scal = DA * PTCO
                Scaly.Initialized(DA * PTCOYCI);
                Scalz.Initialized(DA * PTCOZCI);
                Erruzax.Initialized(PTCXCI - PTCOXCI);
                Erruzay.Initialized(PTCYCI - PTCOYCI);
                Erruzaz.Initialized(PTCZCI - PTCOZCI);

                for (Pi = FirstP; Pi <= LastP; Pi++)
                {
                    TrDAPI.Initialized(TrDA.Col(Pi));
                    TrAPI.Initialized(TrA.Col(Pi));
                    double Taa = TrAPI * A.Row(Pi);
                    Scal = 0.0;
                    for (j = 1; j <= Npol; j++)
                    {
                        DPTCO1[Pi, j + Inc] = (TrDAPI * Errx[Pi] + TrAPI * Scalx[Pi])[j];
                        DPTCO1[Pi, j + Inc + Npol] = (TrDAPI * Erry[Pi] + TrAPI * Scaly[Pi])[j];
                        Scal += DPTCO1[Pi, j + Inc] * Taa * Erruzax[j] + DPTCO1[Pi, j + Inc + Npol] * Taa * Erruzay[j];
                        if (dimen == 3)
                        {
                            DPTCO1[Pi, j + Inc + 2 * Npol] = (TrDAPI * Errz[Pi] + TrAPI * Scalz[Pi])[j];
                            Scal += DPTCO1[Pi, j + Inc + 2 * Npol] * Taa * Erruzaz[j];
                        }
                    }
                    ValGrad_F[Pi] = ValGrad_F[Pi] - 2 * Scal;
                }
                if (dimen == 3)
                {
                    Inc = Inc + 3 * Npol;
                }
                else
                {
                    Inc = Inc + 2 * Npol;
                }
            }

            // on calcule DPTCO = - RESTM * DPTCO1:

            // Calcul de DPTCO/duk:
            // dPTCO/duk = -Inv(T(A)*A)*[d(TA)/duk*[A*PTCO-PTL] + TA*dA/duk*PTCO]

            int low = myConstraints.Lower(), upp = myConstraints.Upper();
            Inc = 0;
            for (Pi = FirstP; Pi <= LastP; Pi++)
            {
                for (i = low; i <= upp; i++)
                {
                    if (myConstraints.Value(i).Index() == Pi)
                    {
                        Cons = myConstraints.Value(i).Constraint();
                        break;
                    }
                }
                if ((int)Cons >= 1)
                {
                    Inc = 0;
                    for (Ci = 1; Ci <= NbCu; Ci++)
                    {
                        dimen = tabdim.Value(Ci - 1);
                        for (j = 1; j <= Npol; j++)
                        {
                            DPTCO[j + Inc] = 0.0;
                            DPTCO[j + Inc + Npol] = 0.0;
                            if (dimen == 3)
                            {
                                DPTCO[j + Inc + 2 * Npol] = 0.0;
                            }
                            for (k = 1; k <= Npol; k++)
                            {
                                // OCCT: the product reads DPTCO1(Pi, j + ...) where k + ... is meant
                                DPTCO[j + Inc] = DPTCO[j + Inc] - RESTM[j, k] * DPTCO1[Pi, j + Inc];
                                DPTCO[j + Inc + Npol] = DPTCO[j + Inc + Npol] - RESTM[j, k] * DPTCO1[Pi, j + Inc + Npol];
                                if (dimen == 3)
                                {
                                    DPTCO[j + Inc + 2 * Npol] =
                                        DPTCO[j + Inc + 2 * Npol] - RESTM[j, k] * DPTCO1[Pi, j + Inc + 2 * Npol];
                                }
                            }
                        }
                        if (dimen == 3)
                        {
                            Inc += 3 * Npol;
                        }
                        else
                        {
                            Inc += 2 * Npol;
                        }
                    }

                    DERR.Initialized(DERR - KK * K * DPTCO);

                    Inc = 0;
                    for (Ci = 1; Ci <= NbCu; Ci++)
                    {
                        dimen = tabdim.Value(Ci - 1);
                        PTCOXCI.Initialized(PTCOX.Col(Ci));
                        PTCOYCI.Initialized(PTCOY.Col(Ci));
                        PTCOZCI.Initialized(PTCOZ.Col(Ci));
                        PTCXCI.Initialized(PTCX.Col(Ci));
                        PTCYCI.Initialized(PTCY.Col(Ci));
                        PTCZCI.Initialized(PTCZ.Col(Ci));
                        Erruzax.Initialized(PTCXCI - PTCOXCI);
                        Erruzay.Initialized(PTCYCI - PTCOYCI);
                        Erruzaz.Initialized(PTCZCI - PTCOZCI);
                        Scal = 0.0;

                        for (j = 1; j <= Npol; j++)
                        {
                            // OCCT: = where += is meant, so only the last pole counts; and the z term uses Erruzax
                            Scal = (A[Pi, j] * Erruzax[j]) * (A[Pi, j] * DERR[j + Inc])
                                   + (A[Pi, j] * Erruzay[j]) * (A[Pi, j] * DERR[j + Inc + Npol]);
                            if (dimen == 3)
                            {
                                Scal += (A[Pi, j] * Erruzax[j]) * (A[Pi, j] * DERR[j + Inc + 2 * Npol]);
                            }
                        }

                        ValGrad_F[Pi] = ValGrad_F[Pi] + 2 * Scal;
                        if (dimen == 3)
                        {
                            Inc = Inc + 3 * Npol;
                        }
                        else
                        {
                            Inc = Inc + 2 * Npol;
                        }
                    }
                }
            }
        }
    }

    /// <summary>The number of variables: the number of points.</summary>
    public override int NbVariables() => NbP;

    /// <summary>The gradient of F at <paramref name="X"/>.</summary>
    public override bool Gradient(math_Vector X, math_Vector G)
    {
        Perform(X);
        G.Initialized(ValGrad_F);

        return true;
    }

    /// <summary>The value and gradient of F at <paramref name="X"/>.</summary>
    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        Perform(X);
        F = FVal;
        G.Initialized(ValGrad_F);
        return true;
    }

    /// <summary>The current curves (the object's own MultiCurve; shared: don't modify).</summary>
    public AppParCurves_MultiCurve CurveValue()
    {
        if (!Contraintes)
        {
            MyMultiCurve = MyLeastSquare.BezierValue();
        }
        return MyMultiCurve;
    }

    /// <summary>The distance between the point <paramref name="IPoint"/> and the curve <paramref name="CurveIndex"/>.</summary>
    public double Error(int IPoint, int CurveIndex) => Math.Sqrt(MyF[IPoint, CurveIndex]);

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => ERR3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => ERR2d;

    /// <summary>The parameters of the last evaluation (shared: don't modify).</summary>
    public math_Vector NewParameters() => myParameters;
}
