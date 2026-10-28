// Port of: src/FoundationClasses/TKMath/math/math_FunctionSetRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionSetRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The root of a set of N functions of M variables (N &lt; M, N = M or N &gt; M) from an initial guess, by a
/// minimization algorithm: a search in the Newton direction, then in the gradient direction if Newton fails. Can
/// also be used to minimize functions. The Jacobian is required.
/// </summary>
public class math_FunctionSetRoot
{
    // pmn 15/05/97 pas de Gauss avec des pivot trop petit. SVD fait mieux
    // l'affaire + limitation de la longeur du pas + qq comentaire issus d'EUCLID3
    // pmn 10/06/97 refonte totale du traitement des bords + ajustement des init
    // et des tolerances pour brent...

    //===========================================================================
    // - A partir d une solution de depart, recherche d une direction.( Newton la
    // plupart du temps, gradient si Newton echoue.
    // - Recadrage au niveau des bornes avec recalcul de la direction si une
    // inconnue a une valeur imposee.
    // -Si On ne sort pas des bornes
    //   Tant que (On ne progresse pas assez ou on ne change pas de direction)
    //    . Si (Progression encore possible)
    //        Si (On ne sort pas des bornes)
    //          On essaye de progresser dans cette meme direction.
    //        Sinon
    //          On diminue le pas d'avancement ou on change de direction.
    //      Sinon
    //        Si on depasse le minimum
    //          On fait une interpolation parabolique.
    // - Si on a progresse sur F
    //     On fait les tests d'arret
    //   Sinon
    //     On change de direction
    //============================================================================

    protected math_Vector Delta;
    protected math_Vector Sol;
    protected math_Matrix DF;
    protected math_Vector Tol;

    private bool Done;
    private int Kount;
    private int State;
    private int Itermax;
    private math_Vector InfBound;
    private math_Vector SupBound;
    private math_Vector SolSave;
    private math_Vector GH;
    private math_Vector DH;
    private math_Vector DHSave;
    private math_Vector FF;
    private math_Vector PreviousSolution;
    private math_Vector Save;
    private math_IntegerVector Constraints;
    private math_Vector Temp1;
    private math_Vector Temp2;
    private math_Vector Temp3;
    private math_Vector Temp4;
    private bool myIsDivergent;

    /// <summary>The function along a direction: x -> 0.5 * |F(P0 + x * Dir)|^2.</summary>
    private sealed class MyDirFunction : math_Function
    {
        private math_Vector P0;
        private math_Vector Dir;
        private math_Vector P;
        private math_Vector FV;
        private math_FunctionSetWithDerivatives F;

        public MyDirFunction(math_Vector V1, math_Vector V2, math_Vector V3, math_Vector V4,
                             math_FunctionSetWithDerivatives f)
        {
            P0 = V1;
            Dir = V2;
            P = V3;
            FV = V4;
            F = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            P0.Initialized(p0);
            Dir.Initialized(dir);
        }

        // For hp :
        // F2 and Gnr1 are ref: OCCT leaves them unchanged when the function fails.
        public bool Value(math_Vector Sol, math_Vector FF, math_Matrix DF, math_Vector GH, ref double F2, ref double Gnr1)
        {
            if (F.Values(Sol, FF, DF))
            {
                double aVal = 0.0;

                for (int i = FF.Lower(); i <= FF.Upper(); i++)
                {
                    // modified by NIZHNY-MKK  Mon Oct  3 17:56:50 2005.BEGIN
                    aVal = FF.Value(i);
                    if (aVal < 0.0)
                    {
                        if (aVal <= -1.0e+100)
                        { // Precision::HalfInfinite() later
                            return false;
                        }
                    }
                    else if (aVal >= 1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                    // modified by NIZHNY-MKK  Mon Oct  3 17:57:05 2005.END
                }

                F2 = 0.5 * (FF.Norm2());
                GH.TMultiply(DF, FF);
                for (int i = GH.Lower(); i <= GH.Upper(); i++)
                {
                    if (Precision.IsInfinite((GH.Value(i))))
                    {
                        return false;
                    }
                }

                Gnr1 = GH.Norm2();
                return true;
            }

            return false;
        }

        /// <remarks>On failure <paramref name="fval"/> is 0; OCCT leaves it unset.</remarks>
        public override bool Value(double x, out double fval)
        {
            fval = 0.0;
            double p;
            for (int i = P.Lower(); i <= P.Upper(); i++)
            {
                p = Dir.Value(i);
                P.Value(i) = p * x + P0.Value(i);
            }

            if (F.Value(P, FV))
            {
                double aVal = 0.0;

                for (int i = FV.Lower(); i <= FV.Upper(); i++)
                {
                    aVal = FV.Value(i);
                    if (aVal <= -1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                    else if (aVal >= 1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                }

                fval = 0.5 * (FV.Norm2());
                return true;
            }

            return false;
        }
    }

    //--------------------------------------------------------------
    // Purpose : minimisation a partir de 3 points
    //-------------------------------------------------------
    private static bool MinimizeDirection(math_Vector P0, math_Vector P1, math_Vector P2, double F1, math_Vector Delta,
                                          math_Vector Tol, MyDirFunction F)
    {
        // (1) Evaluation d'un tolerance parametrique 1D
        double tol1d = 2.1, invnorme, tsol;
        double Eps = 1.0e-16;
        double ax, bx, cx;

        for (int ii = 1; ii <= Tol.Length(); ii++)
        {
            invnorme = Math.Abs(Delta[ii]);
            if (invnorme > Eps)
            {
                tol1d = Min(tol1d, Tol[ii] / invnorme);
            }
        }

        if (tol1d > 1.9)
        {
            return false; // Pas la peine de se fatiguer
        }

        tol1d /= 3;

        // JR/Hp :
        math_Vector PP0 = P0;
        math_Vector PP1 = P1;
        Delta.Initialized(PP1 - PP0);
        //  Delta = P1 - P0;
        invnorme = Delta.Norm();
        if (invnorme <= Eps)
        {
            return false;
        }

        invnorme = ((double)1) / invnorme;

        F.Initialize(P1, Delta);

        // (2) On minimise
        ax = -1;
        bx = 0;
        cx = (P2 - P1).Norm() * invnorme;
        if (cx < 1.0e-2)
        {
            return false;
        }

        math_BrentMinimum Sol = new(tol1d, 100, tol1d);
        Sol.Perform(F, ax, bx, cx);

        if (Sol.IsDone())
        {
            tsol = Sol.Location();
            if (Sol.Minimum() < F1)
            {
                Delta.Multiply(tsol);
                return true;
            }
        }

        return false;
    }

    //----------------------------------------------------------------------
    // Purpose: minimisation a partir de 2 points et une derives
    //----------------------------------------------------------------------
    private static bool MinimizeDirection(math_Vector P, math_Vector Dir, double PValue, double PDirValue,
                                          math_Vector Gradient, math_Vector DGradient, math_Vector Tol, MyDirFunction F)
    {
        if (Precision.IsInfinite(PValue) || Precision.IsInfinite(PDirValue))
        {
            return false;
        }

        // (0) Evaluation d'un tolerance parametrique 1D
        bool good = false;
        double Eps = 1.0e-20;
        double tol1d = 1.1, Result = PValue, absdir;

        for (int ii = 1; ii <= Tol.Length(); ii++)
        {
            absdir = Math.Abs(Dir[ii]);
            if (absdir > Eps)
            {
                tol1d = Min(tol1d, Tol[ii] / absdir);
            }
        }

        if (tol1d > 0.9)
        {
            return false;
        }

        // (1) On realise une premiere interpolation quadratique
        double ax, bx, cx, df1, df2, Delta, tsol, fsol, tsolbis;

        df1 = Gradient * Dir;
        df2 = DGradient * Dir;

        if (df1 < -Eps && df2 > Eps)
        { // cuvette
            tsol = -df1 / (df2 - df1);
        }
        else
        {
            cx = PValue;
            bx = df1;
            ax = PDirValue - (bx + cx);

            if (Math.Abs(ax) <= Eps)
            { // cas lineaire
                if (Math.Abs(bx) >= Eps)
                {
                    tsol = -cx / bx;
                }
                else
                {
                    tsol = 0;
                }
            }
            else
            { // cas quadratique
                Delta = bx * bx - 4 * ax * cx;
                if (Delta > 1.0e-9)
                {
                    // il y a des racines, on prend la plus proche de 0
                    Delta = Math.Sqrt(Delta);
                    tsol = -(bx + Delta);
                    tsolbis = (Delta - bx);
                    if (Math.Abs(tsolbis) < Math.Abs(tsol))
                    {
                        tsol = tsolbis;
                    }

                    tsol /= 2 * ax;
                }
                else
                {
                    // pas ou peu de racine : on "extremise"
                    tsol = -(0.5 * bx) / ax;
                }
            }
        }

        if (Math.Abs(tsol) >= 1)
        {
            return false; // resultat sans interet
        }

        F.Initialize(P, Dir);
        // OCCT ignores the result: on failure fsol is uninitialised there (0 here)
        F.Value(tsol, out fsol);

        if (fsol < PValue)
        {
            good = true;
            Result = fsol;
        }

        // (2) Si l'on a pas assez progresser on realise une recherche
        //     en bonne et due forme, a partir des inits precedents
        if ((fsol > 0.2 * PValue) && (tol1d < 0.5))
        {
            if (tsol < 0)
            {
                ax = tsol;
                bx = 0.0;
                cx = 1.0;
            }
            else
            {
                ax = 0.0;
                bx = tsol;
                cx = 1.0;
            }

            math_BrentMinimum Sol = new(tol1d, 100, tol1d);

            // Base invocation.
            Sol.Perform(F, ax, bx, cx);
            if (Sol.IsDone())
            {
                if (Sol.Minimum() <= Result)
                {
                    tsol = Sol.Location();
                    good = true;
                    Result = Sol.Minimum();

                    // Objective function changes too fast ->
                    // perform additional computations.
                    if (Gradient.Norm2() > 1.0 / Precision.SquareConfusion() && tsol > ax
                        && tsol < cx) // Solution inside of (ax, cx) interval.
                    {
                        // First and second part invocation.
                        Sol.Perform(F, ax, (ax + tsol) / 2.0, tsol);
                        if (Sol.IsDone())
                        {
                            if (Sol.Minimum() <= Result)
                            {
                                tsol = Sol.Location();
                                good = true;
                                Result = Sol.Minimum();
                            }
                        }

                        Sol.Perform(F, tsol, (cx + tsol) / 2.0, cx);
                        if (Sol.IsDone())
                        {
                            if (Sol.Minimum() <= Result)
                            {
                                tsol = Sol.Location();
                                good = true;
                                Result = Sol.Minimum();
                            }
                        }
                    }
                } // if (Sol.Minimum() <= Result)
            } // if(Sol.IsDone())
        }

        if (good)
        {
            // mise a jour du Delta
            Dir.Multiply(tsol);
        }

        return good;
    }

    //------------------------------------------------------
    private static void SearchDirection(math_Matrix DF, math_Vector GH, math_Vector FF, bool ChangeDirection,
                                        math_Vector InvLengthMax, math_Vector Direction, out double Dy)
    {
        int Ninc = DF.ColNumber(), Neq = DF.RowNumber();
        double Eps = 1.0e-32;
        if (!ChangeDirection)
        {
            if (Ninc == Neq)
            {
                for (int i0 = FF.Lower(); i0 <= FF.Upper(); i0++)
                {
                    Direction[i0] = -FF[i0];
                }

                math_Gauss Solut = new(DF, 1.0e-9);
                if (Solut.IsDone())
                {
                    Solut.Solve(Direction);
                }
                else
                { // we have to "forget" singular directions.
                    math_SVD SolvebySVD = new(DF);
                    if (SolvebySVD.IsDone())
                    {
                        SolvebySVD.Solve(-1 * FF, Direction);
                    }
                    else
                    {
                        ChangeDirection = true;
                    }
                }
            }
            else if (Ninc > Neq)
            {
                math_SVD Solut = new(DF);
                if (Solut.IsDone())
                {
                    Solut.Solve(-1 * FF, Direction);
                }
                else
                {
                    ChangeDirection = true;
                }
            }
            else if (Ninc < Neq)
            { // Calcul par GaussLeastSquare
                math_GaussLeastSquare Solut = new(DF);
                if (Solut.IsDone())
                {
                    Solut.Solve(-1 * FF, Direction);
                }
                else
                {
                    ChangeDirection = true;
                }
            }
        }

        // Il vaut mieux interdire des directions trops longue
        // Afin de blinder les cas trop mal conditionne
        // PMN 12/05/97 Traitement des singularite dans les conges
        // Sur des surfaces periodiques

        double ratio = Math.Abs(Direction[Direction.Lower()] * InvLengthMax[Direction.Lower()]);
        int i;
        for (i = Direction.Lower() + 1; i <= Direction.Upper(); i++)
        {
            ratio = Max(ratio, Math.Abs(Direction[i] * InvLengthMax[i]));
        }

        if (ratio > 1)
        {
            Direction /= ratio;
        }

        Dy = Direction * GH;
        if (Dy >= -Eps)
        { // newton "ne descend pas" on prend le gradient
            ChangeDirection = true;
        }

        if (ChangeDirection)
        { // On va faire un gradient !
            for (i = Direction.Lower(); i <= Direction.Upper(); i++)
            {
                Direction[i] = -GH[i];
            }

            Dy = -(GH.Norm2());
        }
    }

    //=====================================================================
    // Purpose : Recherche une direction (et un pas si Newton Fonctionne) le long
    //           d'une frontiere
    //=====================================================================
    private static void SearchDirection(math_Matrix DF, math_Vector GH, math_Vector FF, math_IntegerVector Constraints,
                                        math_Vector X, // Le point d'init
                                        bool ChangeDirection, math_Vector InvLengthMax, math_Vector Direction,
                                        out double Dy)
    {
        int Ninc = DF.ColNumber(), Neq = DF.RowNumber();
        int i, j, k, Cons = 0;

        // verification sur les bornes imposees:

        for (i = 1; i <= Ninc; i++)
        {
            if (Constraints[i] != 0)
            {
                Cons++;
            }
            // sinon le systeme a resoudre ne change pas.
        }

        if (Cons == 0)
        {
            SearchDirection(DF, GH, FF, ChangeDirection, InvLengthMax, Direction, out Dy);
        }
        else if (Cons == Ninc)
        { // il n'y a plus rien a faire...
            for (i = Direction.Lower(); i <= Direction.Upper(); i++)
            {
                Direction[i] = 0;
            }

            Dy = 0;
        }
        else
        { //(1) Cas general : On definit un sous probleme
            math_Matrix DF2 = new(1, Neq, 1, Ninc - Cons);
            math_Vector MyGH = new(1, Ninc - Cons);
            math_Vector MyDirection = new(1, Ninc - Cons);
            math_Vector MyInvLengthMax = new(1, Ninc);

            for (k = 1, i = 1; i <= Ninc; i++)
            {
                if (Constraints[i] == 0)
                {
                    MyGH[k] = GH[i];
                    MyInvLengthMax[k] = InvLengthMax[i];
                    MyDirection[k] = Direction[i];
                    for (j = 1; j <= Neq; j++)
                    {
                        DF2[j, k] = DF[j, i];
                    }

                    k++; // on passe a l'inconnue suivante
                }
            }

            //(2) On le resoud
            SearchDirection(DF2, MyGH, FF, ChangeDirection, MyInvLengthMax, MyDirection, out Dy);

            // (3) On l'interprete...
            // Reconstruction de Direction:
            // (ChangeDirection is this function's copy, not updated by the call above, as in OCCT)
            for (i = 1, k = 1; i <= Ninc; i++)
            {
                if (Constraints[i] == 0)
                {
                    if (!ChangeDirection)
                    {
                        Direction[i] = MyDirection[k];
                    }
                    else
                    {
                        Direction[i] = -GH[i];
                    }

                    k++;
                }
                else
                {
                    Direction[i] = 0.0;
                }
            }
        }
    }

    //====================================================
    // Purpose: Trims an initial solution Sol to be within a domain defined by
    //   InfBound and SupBound. Delta will contain a distance between final Sol and
    //   SolSave.
    //   IsNewSol returns False, if final Sol fully coincides with SolSave, i.e.
    //   if SolSave already lied on a boundary and initial Sol was fully beyond it
    //======================================================
    private static bool Bounds(math_Vector InfBound, math_Vector SupBound, math_Vector Tol, math_Vector Sol,
                               math_Vector SolSave, math_IntegerVector Constraints, math_Vector Delta,
                               out bool theIsNewSol)
    {
        bool Out = false;
        int i, Ninc = Sol.Length();
        double monratio = 1;

        theIsNewSol = true;

        // Calcul du ratio de recadrage
        for (i = 1; i <= Ninc; i++)
        {
            Constraints[i] = 0;
            Delta[i] = Sol[i] - SolSave[i];
            if (InfBound[i] == SupBound[i])
            {
                Constraints[i] = 1;
                Out = true; // Ok mais, cela devrait etre eviter
            }
            else if (Sol[i] < InfBound[i])
            {
                Constraints[i] = 1;
                Out = true;
                // Delta(i) is negative
                if (-Delta[i] > Tol[i])
                { // Afin d'eviter des ratio nulles pour rien
                    monratio = Min(monratio, (InfBound[i] - SolSave[i]) / Delta[i]);
                }
            }
            else if (Sol[i] > SupBound[i])
            {
                Constraints[i] = 1;
                Out = true;
                // Delta(i) is positive
                if (Delta[i] > Tol[i])
                {
                    monratio = Min(monratio, (SupBound[i] - SolSave[i]) / Delta[i]);
                }
            }
        }

        if (Out)
        { // Troncature et derniers recadrage pour blinder (pb numeriques)
            if (monratio == 0.0)
            {
                theIsNewSol = false;
                Sol.Initialized(SolSave);
                Delta.Init(0.0);
            }
            else
            {
                Delta *= monratio;
                Sol.Initialized(SolSave + Delta);
                for (i = 1; i <= Ninc; i++)
                {
                    if (Sol[i] < InfBound[i])
                    {
                        Sol[i] = InfBound[i];
                        Delta[i] = Sol[i] - SolSave[i];
                    }
                    else if (Sol[i] > SupBound[i])
                    {
                        Sol[i] = SupBound[i];
                        Delta[i] = Sol[i] - SolSave[i];
                    }
                }
            }
        }

        return Out;
    }

    /// <summary>
    /// Prepares the algorithm. The vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public math_FunctionSetRoot(math_FunctionSetWithDerivatives theFunction, math_Vector theTolerance,
                                int theNbIterations = 100)
        : this(theFunction, theNbIterations)
    {
        SetTolerance(theTolerance);
    }

    /// <summary>
    /// Prepares the algorithm, for a sub-class; <see cref="SetTolerance"/> must be called afterwards. The vectors and
    /// matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public math_FunctionSetRoot(math_FunctionSetWithDerivatives theFunction, int theNbIterations = 100)
    {
        Delta = new math_Vector(1, theFunction.NbVariables());
        Sol = new math_Vector(1, theFunction.NbVariables());
        DF = new math_Matrix(1, theFunction.NbEquations(), 1, theFunction.NbVariables());
        Tol = new math_Vector(1, theFunction.NbVariables());
        Done = false;
        Kount = 0;
        State = 0;
        Itermax = theNbIterations;
        InfBound = new math_Vector(1, theFunction.NbVariables(), RealFirst());
        SupBound = new math_Vector(1, theFunction.NbVariables(), RealLast());
        SolSave = new math_Vector(1, theFunction.NbVariables());
        GH = new math_Vector(1, theFunction.NbVariables());
        DH = new math_Vector(1, theFunction.NbVariables());
        DHSave = new math_Vector(1, theFunction.NbVariables());
        FF = new math_Vector(1, theFunction.NbEquations());
        PreviousSolution = new math_Vector(1, theFunction.NbVariables());
        Save = new math_Vector(0, theNbIterations);
        Constraints = new math_IntegerVector(1, theFunction.NbVariables());
        Temp1 = new math_Vector(1, theFunction.NbVariables());
        Temp2 = new math_Vector(1, theFunction.NbVariables());
        Temp3 = new math_Vector(1, theFunction.NbVariables());
        Temp4 = new math_Vector(1, theFunction.NbEquations());
        myIsDivergent = false;
    }

    /// <summary>Sets the tolerances.</summary>
    public void SetTolerance(math_Vector theTolerance)
    {
        for (int i = 1; i <= Tol.Length(); ++i)
        {
            Tol[i] = theTolerance[i];
        }
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found; by default when
    /// abs(Xi - Xi-1) &lt;= Tolerance for all unknowns. A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(math_FunctionSetWithDerivatives F)
    {
        for (int i = 1; i <= Sol.Length(); ++i)
        {
            if (Math.Abs(Delta[i]) > Tol[i])
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Improves the root from the initial guess. The solution is found when abs(Xi - Xi-1)(j) &lt;= Tolerance(j) for
    /// all unknowns.
    /// </summary>
    public void Perform(math_FunctionSetWithDerivatives theFunction, math_Vector theStartingPoint,
                        bool theStopOnDivergent = false)
    {
        Perform(theFunction, theStartingPoint, InfBound, SupBound, theStopOnDivergent);
    }

    /// <summary>
    /// Improves the root from the initial guess, keeping it within the bounds. The solution is found when
    /// abs(Xi - Xi-1) &lt;= Tolerance for all unknowns. Throws <see cref="Standard_DimensionError"/> if the lengths of
    /// the starting point or the bounds differ from the number of variables.
    /// </summary>
    public void Perform(math_FunctionSetWithDerivatives F, math_Vector StartingPoint, math_Vector theInfBound,
                        math_Vector theSupBound, bool theStopOnDivergent = false)
    {
        int Ninc = F.NbVariables(), Neq = F.NbEquations();

        if ((Neq <= 0) || (StartingPoint.Length() != Ninc) || (theInfBound.Length() != Ninc)
            || (theSupBound.Length() != Ninc))
        {
            throw new Standard_DimensionError();
        }

        int i;
        bool ChangeDirection = false, Sort = false, isNewSol = false;
        bool Good, Verif;
        bool Stop;
        const double EpsSqrt = 1.0e-16, Eps = 1.0e-32, Eps2 = 1.0e-64, Progres = 0.005;
        double F2 = 0.0, PreviousMinimum, Dy, OldF;
        double Ambda, Ambda2, Gnr1 = 0.0, Oldgr;
        math_Vector InvLengthMax = new(1, Ninc); // Pour bloquer les pas a 1/4 du domaine
        math_IntegerVector aConstraints = new(1, Ninc); // Pour savoir sur quels bord on se trouve
        for (i = 1; i <= Ninc; i++)
        {
            double aSupBound = Min(theSupBound[i], Precision.Infinite());
            double anInfBound = Max(theInfBound[i], -Precision.Infinite());
            InvLengthMax[i] = 1.0 / Max((aSupBound - anInfBound) / 4, 1.0e-9);
        }

        MyDirFunction F_Dir = new(Temp1, Temp2, Temp3, Temp4, F);
        int DescenteIter;

        Done = false;
        Sol.Initialized(StartingPoint);
        Kount = 0;

        //
        myIsDivergent = false;
        for (i = 1; i <= Ninc; i++)
        {
            myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
        }

        if (theStopOnDivergent && myIsDivergent)
        {
            return;
        }

        // Verification de la validite des inconnues par rapport aux bornes.
        // Recentrage sur les bornes si pas valide.
        for (i = 1; i <= Ninc; i++)
        {
            if (Sol[i] <= theInfBound[i])
            {
                Sol[i] = theInfBound[i];
            }
            else if (Sol[i] > theSupBound[i])
            {
                Sol[i] = theSupBound[i];
            }
        }

        // Calcul de la premiere valeur de F et de son gradient
        if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
        {
            Done = false;
            if (!theStopOnDivergent || !myIsDivergent)
            {
                State = F.GetStateNumber();
            }

            return;
        }

        Ambda2 = Gnr1;
        // Rank 0 of Save should only be used for the accelerator test at the end of the loop.
        // If we are already on the solution, we need to bypass this test to avoid
        // making a second iteration...
        Save[0] = Max(F2, EpsSqrt);
        double aTol_Func = Epsilon(F2);

        if ((F2 <= Eps) || (Gnr1 <= Eps2))
        {
            Done = false;
            if (!theStopOnDivergent || !myIsDivergent)
            {
                Done = true;
                State = F.GetStateNumber();
            }

            return;
        }

        for (Kount = 1; Kount <= Itermax; Kount++)
        {
            PreviousMinimum = F2;
            Oldgr = Gnr1;
            PreviousSolution.Initialized(Sol);
            SolSave.Initialized(Sol);

            SearchDirection(DF, GH, FF, ChangeDirection, InvLengthMax, DH, out Dy);
            if (Math.Abs(Dy) <= Eps)
            {
                Done = false;
                if (!theStopOnDivergent || !myIsDivergent)
                {
                    Done = true;
                    ////modified by jgv, 31.08.2011////
                    F.Value(Sol, FF); // update F before GetStateNumber
                    ///////////////////////////////////
                    State = F.GetStateNumber();
                }

                return;
            }

            if (ChangeDirection)
            {
                Ambda = Ambda2 / Math.Sqrt(Math.Abs(Dy));
                if (Ambda > 1.0)
                {
                    Ambda = 1.0;
                }
            }
            else
            {
                Ambda = 1.0;
                Ambda2 = 0.5 * Ambda / DH.Norm();
            }

            for (i = Sol.Lower(); i <= Sol.Upper(); i++)
            {
                Sol[i] = Sol[i] + Ambda * DH[i];
            }

            //
            for (i = 1; i <= Ninc; i++)
            {
                myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
            }

            if (theStopOnDivergent && myIsDivergent)
            {
                return;
            }

            //
            Sort = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);

            DHSave.Initialized(GH);
            if (isNewSol)
            {
                //      F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                {
                    Done = false;
                    if (!theStopOnDivergent || !myIsDivergent)
                    {
                        State = F.GetStateNumber();
                    }

                    return;
                }
            }

            if ((F2 <= Eps) || (Gnr1 <= Eps2))
            {
                Done = false;
                if (!theStopOnDivergent || !myIsDivergent)
                {
                    Done = true;
                    ////modified by jgv, 31.08.2011////
                    F.Value(Sol, FF); // update F before GetStateNumber
                    ///////////////////////////////////
                    State = F.GetStateNumber();
                }

                return;
            }

            if (Sort || (F2 / PreviousMinimum > Progres))
            {
                Dy = GH * DH;
                OldF = PreviousMinimum;
                Stop = false;
                Good = false;
                DescenteIter = 0;
                bool Sortbis;

                // -------------------------------------------------
                // Standard processing without boundary handling
                // -------------------------------------------------
                if (!Sort)
                { // if we haven't exited, we try to progress forward
                    while ((F2 / PreviousMinimum > Progres) && !Stop)
                    {
                        if (F2 < OldF && (Dy < 0.0))
                        {
                            // We try to progress in this direction.
                            DescenteIter++;
                            SolSave.Initialized(Sol);
                            OldF = F2;
                            for (i = Sol.Lower(); i <= Sol.Upper(); i++)
                            {
                                Sol[i] = Sol[i] + Ambda * DH[i];
                            }

                            //
                            for (i = 1; i <= Ninc; i++)
                            {
                                myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
                            }

                            if (theStopOnDivergent && myIsDivergent)
                            {
                                return;
                            }

                            //
                            Stop = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);
                            Ambda *= 1.7;
                        }
                        else
                        {
                            if ((F2 >= OldF) || (F2 >= PreviousMinimum))
                            {
                                Good = false;
                                if (DescenteIter == 0)
                                {
                                    // C'est le premier pas qui flanche, on fait une interpolation.
                                    // et on minimise si necessaire.
                                    DescenteIter++;
                                    Good = MinimizeDirection(SolSave, Delta, OldF, F2, DHSave, GH, Tol, F_Dir);
                                }
                                else if (ChangeDirection || (DescenteIter > 1) || (OldF > PreviousMinimum))
                                {
                                    // La progression a ete utile, on minimise...
                                    DescenteIter++;
                                    Good = MinimizeDirection(PreviousSolution, SolSave, Sol, OldF, Delta, Tol, F_Dir);
                                }

                                if (!Good)
                                {
                                    Sol.Initialized(SolSave);
                                    F2 = OldF;
                                }
                                else
                                {
                                    Sol.Initialized(SolSave + Delta);
                                    //
                                    for (i = 1; i <= Ninc; i++)
                                    {
                                        myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
                                    }

                                    if (theStopOnDivergent && myIsDivergent)
                                    {
                                        return;
                                    }

                                    //
                                    Sort = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);
                                }

                                Sort = false; // On a rejete le point sur la frontiere
                            }

                            Stop = true; // et on sort dans tous les cas...
                        }

                        DHSave.Initialized(GH);
                        if (isNewSol)
                        {
                            //            F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                            if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                            {
                                Done = false;
                                if (!theStopOnDivergent || !myIsDivergent)
                                {
                                    State = F.GetStateNumber();
                                }

                                return;
                            }
                        }

                        Dy = GH * DH;
                        if (Math.Abs(Dy) <= Eps)
                        {
                            if (F2 > OldF)
                            {
                                Sol.Initialized(SolSave);
                            }

                            Done = false;
                            if (!theStopOnDivergent || !myIsDivergent)
                            {
                                Done = true;
                                ////modified by jgv, 31.08.2011////
                                F.Value(Sol, FF); // update F before GetStateNumber
                                ///////////////////////////////////
                                State = F.GetStateNumber();
                            }

                            return;
                        }

                        if (DescenteIter >= 100)
                        {
                            Stop = true;
                        }
                    }
                }

                // ------------------------------------
                //  on passe au traitement des bords
                // ------------------------------------
                if (Sort)
                {
                    Stop = (F2 > 1.001 * OldF); // Pour ne pas progresser sur le bord
                    Sortbis = Sort;
                    DescenteIter = 0;
                    while (Sortbis && ((F2 < OldF) || (DescenteIter == 0)) && (!Stop))
                    {
                        DescenteIter++;
                        // On essaye de progresser sur le bord
                        SolSave.Initialized(Sol);
                        OldF = F2;
                        SearchDirection(DF, GH, FF, aConstraints, Sol, ChangeDirection, InvLengthMax, DH, out Dy);
                        if (Dy < -Eps)
                        { // Pour eviter des calculs inutiles et des /0...
                            if (ChangeDirection)
                            {
                                Ambda = Ambda2 / Math.Sqrt(-Dy);
                                if (Ambda > 1.0)
                                {
                                    Ambda = 1.0;
                                }
                            }
                            else
                            {
                                Ambda = 1.0;
                                Ambda2 = 0.5 * Ambda / DH.Norm();
                            }

                            for (i = Sol.Lower(); i <= Sol.Upper(); i++)
                            {
                                Sol[i] = Sol[i] + Ambda * DH[i];
                            }

                            //
                            for (i = 1; i <= Ninc; i++)
                            {
                                myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
                            }

                            if (theStopOnDivergent && myIsDivergent)
                            {
                                return;
                            }

                            //
                            Sortbis = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);

                            DHSave.Initialized(GH);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                                {
                                    Done = false;
                                    if (!theStopOnDivergent || !myIsDivergent)
                                    {
                                        State = F.GetStateNumber();
                                    }

                                    return;
                                }
                            }

                            Ambda2 = Gnr1;
                        }
                        else
                        {
                            Stop = true;
                        }

                        while ((F2 / PreviousMinimum > Progres) && (F2 < OldF) && (!Stop))
                        {
                            DescenteIter++;
                            if (F2 < OldF && Dy < 0.0)
                            {
                                // On essaye de progresser dans cette direction.
                                SolSave.Initialized(Sol);
                                OldF = F2;
                                for (i = Sol.Lower(); i <= Sol.Upper(); i++)
                                {
                                    Sol[i] = Sol[i] + Ambda * DH[i];
                                }

                                //
                                for (i = 1; i <= Ninc; i++)
                                {
                                    myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
                                }

                                if (theStopOnDivergent && myIsDivergent)
                                {
                                    return;
                                }

                                //
                                Sortbis = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);
                            }

                            DHSave.Initialized(GH);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                                {
                                    Done = false;
                                    if (!theStopOnDivergent || !myIsDivergent)
                                    {
                                        State = F.GetStateNumber();
                                    }

                                    return;
                                }
                            }

                            Ambda2 = Gnr1;
                            Dy = GH * DH;
                            Stop = ((Dy >= 0) || (DescenteIter >= 10) || Sortbis);
                        }

                        Stop = ((Dy >= 0) || (DescenteIter >= 10));
                    }

                    if (((F2 / PreviousMinimum > Progres) && (F2 >= OldF)) || (F2 >= PreviousMinimum))
                    {
                        // On minimise par Brent
                        DescenteIter++;
                        Good = MinimizeDirection(SolSave, Delta, OldF, F2, DHSave, GH, Tol, F_Dir);
                        if (!Good)
                        {
                            Sol.Initialized(SolSave);
                            Sort = false;
                        }
                        else
                        {
                            Sol.Initialized(SolSave + Delta);
                            //
                            for (i = 1; i <= Ninc; i++)
                            {
                                myIsDivergent = myIsDivergent || Sol[i] < theInfBound[i] || Sol[i] > theSupBound[i];
                            }

                            if (theStopOnDivergent && myIsDivergent)
                            {
                                return;
                            }

                            //
                            Sort = Bounds(theInfBound, theSupBound, Tol, Sol, SolSave, aConstraints, Delta, out isNewSol);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                                {
                                    Done = false;
                                    if (!theStopOnDivergent || !myIsDivergent)
                                    {
                                        State = F.GetStateNumber();
                                    }

                                    return;
                                }
                            }
                        }

                        Dy = GH * DH;
                    }
                }
            }

            // ---------------------------------------------
            //  on passe aux tests d'ARRET
            // ---------------------------------------------
            Save[Kount] = F2;
            // Est ce la solution ?
            if (ChangeDirection)
            {
                Verif = true;
                // Gradient : Il faut eviter de boucler
            }
            else
            {
                Verif = false;
                if (Kount > 1)
                { // Pour accelerer les cas quasi-quadratique
                    if (Save[Kount - 1] < 1.0e-4 * Save[Kount - 2])
                    {
                        Verif = true;
                    }
                }
                else
                {
                    Verif = (F2 < 1.0e-6 * Save[0]); // Pour les cas dejas solutions
                }
            }

            if (Verif)
            {
                for (i = Delta.Lower(); i <= Delta.Upper(); i++)
                {
                    Delta[i] = PreviousSolution[i] - Sol[i];
                }

                if (IsSolutionReached(F))
                {
                    if (PreviousMinimum < F2)
                    {
                        Sol.Initialized(SolSave);
                    }

                    Done = false;
                    if (!theStopOnDivergent || !myIsDivergent)
                    {
                        Done = true;
                        ////modified by jgv, 31.08.2011////
                        F.Value(Sol, FF); // update F before GetStateNumber
                        ///////////////////////////////////
                        State = F.GetStateNumber();
                    }

                    return;
                }
            }
            // fin du test solution

            // Analyse de la progression...
            // comparison of current minimum and previous minimum
            if ((F2 - PreviousMinimum) <= aTol_Func)
            {
                if (Kount > 5)
                {
                    // L'historique est il bon ?
                    if (F2 >= 0.95 * Save[Kount - 5])
                    {
                        if (!ChangeDirection)
                        {
                            ChangeDirection = true;
                        }
                        else
                        {
                            Done = false;
                            if (!theStopOnDivergent || !myIsDivergent)
                            {
                                Done = true;
                                State = F.GetStateNumber();
                            }

                            return; //  si un gain inf a 5% on sort
                        }
                    }
                    else
                    {
                        ChangeDirection = false; // If yes we restart
                    }
                }
                else
                {
                    ChangeDirection = false; // No history, we continue
                }

                // If the gradient does not decrease sufficiently with Newton, we try
                // the gradient method unless f decreases (as strange as it may seem,
                // with NEWTON the gradient of f can increase while f
                // decreases: in this case we must keep NEWTON)
                if ((Gnr1 > 0.9 * Oldgr) && (F2 > 0.5 * PreviousMinimum))
                {
                    ChangeDirection = true;
                }

                // If we don't decide to change strategy, we verify
                // if not already done
                if ((!ChangeDirection) && (!Verif))
                {
                    for (i = Delta.Lower(); i <= Delta.Upper(); i++)
                    {
                        Delta[i] = PreviousSolution[i] - Sol[i];
                    }

                    if (IsSolutionReached(F))
                    {
                        Done = false;
                        if (!theStopOnDivergent || !myIsDivergent)
                        {
                            Done = true;
                            ////modified by jgv, 31.08.2011////
                            F.Value(Sol, FF); // update F before GetStateNumber
                            ///////////////////////////////////
                            State = F.GetStateNumber();
                        }

                        return;
                    }
                }
            }
            else
            { // Cas de regression
                if (!ChangeDirection)
                { // On passe au gradient
                    ChangeDirection = true;
                    Sol.Initialized(PreviousSolution);
                    //	F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                    if (!F_Dir.Value(Sol, FF, DF, GH, ref F2, ref Gnr1))
                    {
                        Done = false;
                        if (!theStopOnDivergent || !myIsDivergent)
                        {
                            State = F.GetStateNumber();
                        }

                        return;
                    }
                }
                else
                {
                    if (!theStopOnDivergent || !myIsDivergent)
                    {
                        State = F.GetStateNumber();
                    }

                    return; // y a plus d'issues
                }
            }
        }

        if (!theStopOnDivergent || !myIsDivergent)
        {
            State = F.GetStateNumber();
        }
    }

    public bool IsDone() => Done;

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the root was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Kount;
    }

    /// <summary>The state number (from F.GetStateNumber()) of the root. Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public int StateNumber()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return State;
    }

    /// <summary>The root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public math_Vector Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Sol;
    }

    /// <summary>
    /// Sets <paramref name="Root"/> to the root. Throws <see cref="StdFail_NotDone"/> if not found and
    /// <see cref="Standard_DimensionError"/> if its length differs from the starting point's.
    /// </summary>
    public void Root(math_Vector Root)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if(Root.Length() != Sol.Length(), " ");
        Root.Initialized(Sol);
    }

    /// <summary>The derivative matrix at the root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public math_Matrix Derivative()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return DF;
    }

    /// <summary>
    /// Sets <paramref name="Der"/> to the derivative matrix at the root. Throws <see cref="StdFail_NotDone"/> if not
    /// found and <see cref="Standard_DimensionError"/> if its column count differs from the starting point's length.
    /// </summary>
    public void Derivative(math_Matrix Der)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if(Der.ColNumber() != Sol.Length(), " ");
        Der.Initialized(DF);
    }

    /// <summary>The errors on the functions at the root (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not found.</summary>
    public math_Vector FunctionSetErrors()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Delta;
    }

    /// <summary>
    /// Sets <paramref name="Err"/> to the errors on the functions at the root. Throws <see cref="StdFail_NotDone"/> if
    /// not found and <see cref="Standard_DimensionError"/> if its length differs from the starting point's.
    /// </summary>
    public void FunctionSetErrors(math_Vector Err)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if(Err.Length() != Sol.Length(), " ");
        Err.Initialized(Delta);
    }

    public bool IsDivergent() => myIsDivergent;

    public void Dump(TextWriter o)
    {
        o.Write(" math_FunctionSetRoot");
        if (Done)
        {
            o.Write(" Status = Done\n");
            o.Write(" Location value = " + Sol + "\n");
            o.Write(" Number of iterations = " + Kount + "\n");
        }
        else
        {
            o.Write("Status = Not Done\n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
