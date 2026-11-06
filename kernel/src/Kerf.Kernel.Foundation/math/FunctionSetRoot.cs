// Port of: src/FoundationClasses/TKMath/math/math_FunctionSetRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionSetRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The root of a set of N functions of M variables (N &lt; M, N = M or N &gt; M) from an initial guess, by a
/// minimization algorithm: a search in the Newton direction, then in the gradient direction if Newton fails. Can
/// also be used to minimize functions. The Jacobian is required.
/// </summary>
internal class FunctionSetRoot
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

    protected math_Vector _delta;
    protected math_Vector _sol;
    protected Matrix _df;
    protected math_Vector _tol;

    private bool _done;
    private int _kount;
    private int _state;
    private int _itermax;
    private math_Vector _infBound;
    private math_Vector _supBound;
    // The search's own vectors, each written before it's read: a thread's set, for the length of a search
    private math_Vector _solSave = null!;
    private math_Vector _gh = null!;
    private math_Vector _dh = null!;
    private math_Vector _dhSave = null!;
    private math_Vector _ff;
    private math_Vector _previousSolution = null!;
    private math_Vector _save = null!;
    private math_Vector _temp1 = null!;
    private math_Vector _temp2 = null!;
    private math_Vector _temp3 = null!;
    private math_Vector _temp4 = null!;
    private bool _isDivergent;

    /// <summary>The function along a direction: x -> 0.5 * |F(P0 + x * Dir)|^2.</summary>
    private sealed class MyDirFunction : Function
    {
        private math_Vector _p0;
        private math_Vector _dir;
        private math_Vector _p;
        private math_Vector _fv;
        private FunctionSetWithDerivatives _f;

        public MyDirFunction(math_Vector v1, math_Vector v2, math_Vector v3, math_Vector v4,
                             FunctionSetWithDerivatives f)
        {
            _p0 = v1;
            _dir = v2;
            _p = v3;
            _fv = v4;
            _f = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            _p0.Initialized(p0);
            _dir.Initialized(dir);
        }

        // For hp :
        // F2 and Gnr1 are ref: OCCT leaves them unchanged when the function fails.
        public bool Value(math_Vector sol, math_Vector ff, Matrix df, math_Vector gh, ref double f2, ref double gnr1)
        {
            if (_f.Values(sol, ff, df))
            {
                double val = 0.0;

                for (int i = ff.Lower(); i <= ff.Upper(); i++)
                {
                    // modified by NIZHNY-MKK  Mon Oct  3 17:56:50 2005.BEGIN
                    val = ff.Value(i);
                    if (val < 0.0)
                    {
                        if (val <= -1.0e+100)
                        { // Precision::HalfInfinite() later
                            return false;
                        }
                    }
                    else if (val >= 1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                    // modified by NIZHNY-MKK  Mon Oct  3 17:57:05 2005.END
                }

                f2 = 0.5 * (ff.Norm2());
                gh.TMultiply(df, ff);
                for (int i = gh.Lower(); i <= gh.Upper(); i++)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite((gh.Value(i))))
                    {
                        return false;
                    }
                }

                gnr1 = gh.Norm2();
                return true;
            }

            return false;
        }

        /// <remarks>On failure <paramref name="fval"/> is 0; OCCT leaves it unset.</remarks>
        public override bool Value(double x, out double fval)
        {
            fval = 0.0;
            double p;
            for (int i = _p.Lower(); i <= _p.Upper(); i++)
            {
                p = _dir.Value(i);
                _p.Value(i) = p * x + _p0.Value(i);
            }

            if (_f.Value(_p, _fv))
            {
                double val = 0.0;

                for (int i = _fv.Lower(); i <= _fv.Upper(); i++)
                {
                    val = _fv.Value(i);
                    if (val <= -1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                    else if (val >= 1.0e+100)
                    { // Precision::HalfInfinite() later
                        return false;
                    }
                }

                fval = 0.5 * (_fv.Norm2());
                return true;
            }

            return false;
        }
    }

    //--------------------------------------------------------------
    // Purpose : minimisation a partir de 3 points
    //-------------------------------------------------------
    private static bool MinimizeDirection(math_Vector p0, math_Vector p1, math_Vector p2, double f1, math_Vector delta,
                                          math_Vector tol, MyDirFunction f)
    {
        // (1) Evaluation d'un tolerance parametrique 1D
        double tol1d = 2.1, invnorme, tsol;
        double eps = 1.0e-16;
        double ax, bx, cx;

        for (int ii = 1; ii <= tol.Length(); ii++)
        {
            invnorme = Math.Abs(delta[ii]);
            if (invnorme > eps)
            {
                tol1d = Min(tol1d, tol[ii] / invnorme);
            }
        }

        if (tol1d > 1.9)
        {
            return false; // Pas la peine de se fatiguer
        }

        tol1d /= 3;

        // JR/Hp :
        math_Vector pp0 = p0;
        math_Vector pp1 = p1;
        delta.Initialized(pp1 - pp0);
        //  Delta = P1 - P0;
        invnorme = delta.Norm();
        if (invnorme <= eps)
        {
            return false;
        }

        invnorme = ((double)1) / invnorme;

        f.Initialize(p1, delta);

        // (2) On minimise
        ax = -1;
        bx = 0;
        cx = (p2 - p1).Norm() * invnorme;
        if (cx < 1.0e-2)
        {
            return false;
        }

        BrentMinimum sol = new(tol1d, 100, tol1d);
        sol.Perform(f, ax, bx, cx);

        if (sol.IsDone)
        {
            tsol = sol.Location();
            if (sol.Minimum() < f1)
            {
                delta.Multiply(tsol);
                return true;
            }
        }

        return false;
    }

    //----------------------------------------------------------------------
    // Purpose: minimisation a partir de 2 points et une derives
    //----------------------------------------------------------------------
    private static bool MinimizeDirection(math_Vector p, math_Vector dir, double pValue, double pDirValue,
                                          math_Vector gradient, math_Vector dGradient, math_Vector tol, MyDirFunction f)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(pValue) || global::Kerf.Engine.Foundation.Precision.IsInfinite(pDirValue))
        {
            return false;
        }

        // (0) Evaluation d'un tolerance parametrique 1D
        bool good = false;
        double eps = 1.0e-20;
        double tol1d = 1.1, result = pValue, absdir;

        for (int ii = 1; ii <= tol.Length(); ii++)
        {
            absdir = Math.Abs(dir[ii]);
            if (absdir > eps)
            {
                tol1d = Min(tol1d, tol[ii] / absdir);
            }
        }

        if (tol1d > 0.9)
        {
            return false;
        }

        // (1) On realise une premiere interpolation quadratique
        double ax, bx, cx, df1, df2, delta, tsol, fsol, tsolbis;

        df1 = gradient * dir;
        df2 = dGradient * dir;

        if (df1 < -eps && df2 > eps)
        { // cuvette
            tsol = -df1 / (df2 - df1);
        }
        else
        {
            cx = pValue;
            bx = df1;
            ax = pDirValue - (bx + cx);

            if (Math.Abs(ax) <= eps)
            { // cas lineaire
                if (Math.Abs(bx) >= eps)
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
                delta = bx * bx - 4 * ax * cx;
                if (delta > 1.0e-9)
                {
                    // il y a des racines, on prend la plus proche de 0
                    delta = Math.Sqrt(delta);
                    tsol = -(bx + delta);
                    tsolbis = (delta - bx);
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

        f.Initialize(p, dir);
        // OCCT ignores the result: on failure fsol is uninitialised there (0 here)
        f.Value(tsol, out fsol);

        if (fsol < pValue)
        {
            good = true;
            result = fsol;
        }

        // (2) Si l'on a pas assez progresser on realise une recherche
        //     en bonne et due forme, a partir des inits precedents
        if ((fsol > 0.2 * pValue) && (tol1d < 0.5))
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

            BrentMinimum sol = new(tol1d, 100, tol1d);

            // Base invocation.
            sol.Perform(f, ax, bx, cx);
            if (sol.IsDone)
            {
                if (sol.Minimum() <= result)
                {
                    tsol = sol.Location();
                    good = true;
                    result = sol.Minimum();

                    // Objective function changes too fast ->
                    // perform additional computations.
                    if (gradient.Norm2() > 1.0 / global::Kerf.Engine.Foundation.Precision.SquareConfusion() && tsol > ax
                        && tsol < cx) // Solution inside of (ax, cx) interval.
                    {
                        // First and second part invocation.
                        sol.Perform(f, ax, (ax + tsol) / 2.0, tsol);
                        if (sol.IsDone)
                        {
                            if (sol.Minimum() <= result)
                            {
                                tsol = sol.Location();
                                good = true;
                                result = sol.Minimum();
                            }
                        }

                        sol.Perform(f, tsol, (cx + tsol) / 2.0, cx);
                        if (sol.IsDone)
                        {
                            if (sol.Minimum() <= result)
                            {
                                tsol = sol.Location();
                                good = true;
                                result = sol.Minimum();
                            }
                        }
                    }
                } // if (Sol.Minimum() <= Result)
            } // if(Sol.IsDone())
        }

        if (good)
        {
            // mise a jour du Delta
            dir.Multiply(tsol);
        }

        return good;
    }

    //------------------------------------------------------
    // The solvers and the right-hand side, kept from one iteration to the next (unit 11B). Each is written before it's read,
    // and a sub-problem at a bound has a different size, so each is checked for size
    private sealed class SearchWork
    {
        private Gauss? _gauss;
        private Svd? _svd;
        private GaussLeastSquare? _leastSquare;
        private math_Vector? _minusF;

        public Gauss GaussFor(Matrix a, double minPivot)
        {
            if (_gauss is null || _gauss.Size != a.RowNumber())
            {
                _gauss = new Gauss(a.RowNumber());
            }

            _gauss.Perform(a, minPivot);
            return _gauss;
        }

        public Svd SvdFor(Matrix a)
        {
            if (_svd is null || !_svd.Fits(a))
            {
                _svd = new Svd(a.RowNumber(), a.ColNumber());
            }

            _svd.Perform(a);
            return _svd;
        }

        public GaussLeastSquare LeastSquareFor(Matrix a)
        {
            if (_leastSquare is null || !_leastSquare.Fits(a))
            {
                _leastSquare = new GaussLeastSquare(a.RowNumber(), a.ColNumber());
            }

            _leastSquare.Perform(a);
            return _leastSquare;
        }

        // OCCT: -1 * ff
        public math_Vector MinusF(math_Vector ff)
        {
            if (_minusF is null || _minusF.Length() != ff.Length())
            {
                _minusF = new math_Vector(1, ff.Length());
            }

            for (int i = 0; i < ff.Length(); i++)
            {
                _minusF[i + 1] = ff[ff.Lower() + i] * -1.0;
            }

            return _minusF;
        }
    }

    private readonly SearchWork _searchWork = new();

    private static void SearchDirection(SearchWork work, Matrix df, math_Vector gh, math_Vector ff, bool changeDirection,
                                        math_Vector invLengthMax, math_Vector direction, out double dy)
    {
        int ninc = df.ColNumber(), neq = df.RowNumber();
        double eps = 1.0e-32;
        if (!changeDirection)
        {
            if (ninc == neq)
            {
                for (int i0 = ff.Lower(); i0 <= ff.Upper(); i0++)
                {
                    direction[i0] = -ff[i0];
                }

                Gauss solut = work.GaussFor(df, 1.0e-9);
                if (solut.IsDone)
                {
                    solut.Solve(direction);
                }
                else
                { // we have to "forget" singular directions.
                    Svd solvebySVD = work.SvdFor(df);
                    if (solvebySVD.IsDone)
                    {
                        solvebySVD.Solve(work.MinusF(ff), direction);
                    }
                    else
                    {
                        changeDirection = true;
                    }
                }
            }
            else if (ninc > neq)
            {
                Svd solut = work.SvdFor(df);
                if (solut.IsDone)
                {
                    solut.Solve(work.MinusF(ff), direction);
                }
                else
                {
                    changeDirection = true;
                }
            }
            else if (ninc < neq)
            { // Calcul par GaussLeastSquare
                GaussLeastSquare solut = work.LeastSquareFor(df);
                if (solut.IsDone)
                {
                    solut.Solve(work.MinusF(ff), direction);
                }
                else
                {
                    changeDirection = true;
                }
            }
        }

        // Il vaut mieux interdire des directions trops longue
        // Afin de blinder les cas trop mal conditionne
        // PMN 12/05/97 Traitement des singularite dans les conges
        // Sur des surfaces periodiques

        double ratio = Math.Abs(direction[direction.Lower()] * invLengthMax[direction.Lower()]);
        int i;
        for (i = direction.Lower() + 1; i <= direction.Upper(); i++)
        {
            ratio = Max(ratio, Math.Abs(direction[i] * invLengthMax[i]));
        }

        if (ratio > 1)
        {
            direction /= ratio;
        }

        dy = direction * gh;
        if (dy >= -eps)
        { // newton "ne descend pas" on prend le gradient
            changeDirection = true;
        }

        if (changeDirection)
        { // On va faire un gradient !
            for (i = direction.Lower(); i <= direction.Upper(); i++)
            {
                direction[i] = -gh[i];
            }

            dy = -(gh.Norm2());
        }
    }

    //=====================================================================
    // Purpose : Recherche une direction (et un pas si Newton Fonctionne) le long
    //           d'une frontiere
    //=====================================================================
    private static void SearchDirection(SearchWork work, Matrix df, math_Vector gh, math_Vector ff, math_IntegerVector constraints,
                                        bool changeDirection, math_Vector invLengthMax, math_Vector direction,
                                        out double dy)
    {
        int ninc = df.ColNumber(), neq = df.RowNumber();
        int i, j, k, cons = 0;

        // verification sur les bornes imposees:

        for (i = 1; i <= ninc; i++)
        {
            if (constraints[i] != 0)
            {
                cons++;
            }
            // sinon le systeme a resoudre ne change pas.
        }

        if (cons == 0)
        {
            SearchDirection(work, df, gh, ff, changeDirection, invLengthMax, direction, out dy);
        }
        else if (cons == ninc)
        { // il n'y a plus rien a faire...
            for (i = direction.Lower(); i <= direction.Upper(); i++)
            {
                direction[i] = 0;
            }

            dy = 0;
        }
        else
        { //(1) Cas general : On definit un sous probleme
            Matrix df2 = new(1, neq, 1, ninc - cons);
            math_Vector myGH = new(1, ninc - cons);
            math_Vector myDirection = new(1, ninc - cons);
            math_Vector myInvLengthMax = new(1, ninc);

            for (k = 1, i = 1; i <= ninc; i++)
            {
                if (constraints[i] == 0)
                {
                    myGH[k] = gh[i];
                    myInvLengthMax[k] = invLengthMax[i];
                    myDirection[k] = direction[i];
                    for (j = 1; j <= neq; j++)
                    {
                        df2[j, k] = df[j, i];
                    }

                    k++; // on passe a l'inconnue suivante
                }
            }

            //(2) On le resoud
            SearchDirection(work, df2, myGH, ff, changeDirection, myInvLengthMax, myDirection, out dy);

            // (3) On l'interprete...
            // Reconstruction de Direction:
            // (ChangeDirection is this function's copy, not updated by the call above, as in OCCT)
            for (i = 1, k = 1; i <= ninc; i++)
            {
                if (constraints[i] == 0)
                {
                    if (!changeDirection)
                    {
                        direction[i] = myDirection[k];
                    }
                    else
                    {
                        direction[i] = -gh[i];
                    }

                    k++;
                }
                else
                {
                    direction[i] = 0.0;
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
    private static bool Bounds(math_Vector infBound, math_Vector supBound, math_Vector tol, math_Vector sol,
                               math_Vector solSave, math_IntegerVector constraints, math_Vector delta,
                               out bool isNewSol)
    {
        bool Out = false;
        int i, ninc = sol.Length();
        double monratio = 1;

        isNewSol = true;

        // Calcul du ratio de recadrage
        for (i = 1; i <= ninc; i++)
        {
            constraints[i] = 0;
            delta[i] = sol[i] - solSave[i];
            if (infBound[i] == supBound[i])
            {
                constraints[i] = 1;
                Out = true; // Ok mais, cela devrait etre eviter
            }
            else if (sol[i] < infBound[i])
            {
                constraints[i] = 1;
                Out = true;
                // Delta(i) is negative
                if (-delta[i] > tol[i])
                { // Afin d'eviter des ratio nulles pour rien
                    monratio = Min(monratio, (infBound[i] - solSave[i]) / delta[i]);
                }
            }
            else if (sol[i] > supBound[i])
            {
                constraints[i] = 1;
                Out = true;
                // Delta(i) is positive
                if (delta[i] > tol[i])
                {
                    monratio = Min(monratio, (supBound[i] - solSave[i]) / delta[i]);
                }
            }
        }

        if (Out)
        { // Troncature et derniers recadrage pour blinder (pb numeriques)
            if (monratio == 0.0)
            {
                isNewSol = false;
                sol.Initialized(solSave);
                delta.Init(0.0);
            }
            else
            {
                delta *= monratio;
                sol.Initialized(solSave + delta);
                for (i = 1; i <= ninc; i++)
                {
                    if (sol[i] < infBound[i])
                    {
                        sol[i] = infBound[i];
                        delta[i] = sol[i] - solSave[i];
                    }
                    else if (sol[i] > supBound[i])
                    {
                        sol[i] = supBound[i];
                        delta[i] = sol[i] - solSave[i];
                    }
                }
            }
        }

        return Out;
    }

    /// <summary>
    /// Prepares the algorithm. The vectors and matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public FunctionSetRoot(FunctionSetWithDerivatives function, math_Vector tolerance,
                                int nbIterations = 100)
        : this(function, nbIterations)
    {
        SetTolerance(tolerance);
    }

    /// <summary>
    /// Prepares the algorithm, for a sub-class; <see cref="SetTolerance"/> must be called afterwards. The vectors and
    /// matrices of the function must use the range (1, F.NbVariables()).
    /// </summary>
    public FunctionSetRoot(FunctionSetWithDerivatives function, int nbIterations = 100)
    {
        _delta = new math_Vector(1, function.NbVariables());
        _sol = new math_Vector(1, function.NbVariables());
        _df = new Matrix(1, function.NbEquations(), 1, function.NbVariables());
        _tol = new math_Vector(1, function.NbVariables());
        _done = false;
        _kount = 0;
        _state = 0;
        _itermax = nbIterations;
        _infBound = new math_Vector(1, function.NbVariables(), RealFirst());
        _supBound = new math_Vector(1, function.NbVariables(), RealLast());
        _ff = new math_Vector(1, function.NbEquations());
        _isDivergent = false;
    }

    /// <summary>Sets the tolerances.</summary>
    public void SetTolerance(math_Vector tolerance)
    {
        for (int i = 1; i <= _tol.Length(); ++i)
        {
            _tol[i] = tolerance[i];
        }
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found; by default when
    /// abs(Xi - Xi-1) &lt;= Tolerance for all unknowns. A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(FunctionSetWithDerivatives f)
    {
        for (int i = 1; i <= _sol.Length(); ++i)
        {
            if (Math.Abs(_delta[i]) > _tol[i])
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
    public void Perform(FunctionSetWithDerivatives function, math_Vector startingPoint,
                        bool stopOnDivergent = false)
    {
        Perform(function, startingPoint, _infBound, _supBound, stopOnDivergent);
    }

    /// <summary>
    /// Improves the root from the initial guess, keeping it within the bounds. The solution is found when
    /// abs(Xi - Xi-1) &lt;= Tolerance for all unknowns. Throws <see cref="DimensionErrorException"/> if the lengths of
    /// the starting point or the bounds differ from the number of variables.
    /// </summary>
    public void Perform(FunctionSetWithDerivatives f, math_Vector startingPoint, math_Vector infBound,
                        math_Vector supBound, bool stopOnDivergent = false)
    {
        int ninc = f.NbVariables(), neq = f.NbEquations();

        if ((neq <= 0) || (startingPoint.Length() != ninc) || (infBound.Length() != ninc)
            || (supBound.Length() != ninc))
        {
            throw new DimensionErrorException();
        }

        Work work = Work.Rent(ninc, neq, _itermax);
        _solSave = work.SolSave;
        _gh = work.Gh;
        _dh = work.Dh;
        _dhSave = work.DhSave;
        _previousSolution = work.PreviousSolution;
        _save = work.Save;
        _temp1 = work.Temp1;
        _temp2 = work.Temp2;
        _temp3 = work.Temp3;
        _temp4 = work.Temp4;
        Search(f, startingPoint, infBound, supBound, stopOnDivergent, ninc);
        Work.Return(work);
    }

    // One set kept by each thread for the sizes it last used, as a root finder has no end of life to give them back at, and
    // nearly all the searches of a run have the same sizes. Taken out while in use, so a search inside a search makes its own.
    private sealed class Work
    {
        [ThreadStatic]
        private static Work? t_spare;

        private readonly int _ninc;
        private readonly int _neq;
        private readonly int _nbIterations;
        public readonly math_Vector SolSave, Gh, Dh, DhSave, PreviousSolution, Save, Temp1, Temp2, Temp3, Temp4;

        private Work(int ninc, int neq, int nbIterations)
        {
            _ninc = ninc;
            _neq = neq;
            _nbIterations = nbIterations;
            SolSave = new(1, ninc);
            Gh = new(1, ninc);
            Dh = new(1, ninc);
            DhSave = new(1, ninc);
            PreviousSolution = new(1, ninc);
            Save = new(0, nbIterations);
            Temp1 = new(1, ninc);
            Temp2 = new(1, ninc);
            Temp3 = new(1, ninc);
            Temp4 = new(1, neq);
        }

        public static Work Rent(int ninc, int neq, int nbIterations)
        {
            Work? work = t_spare;
            t_spare = null;
            return work is not null && work._ninc == ninc && work._neq == neq && work._nbIterations == nbIterations
                ? work
                : new Work(ninc, neq, nbIterations);
        }

        public static void Return(Work work)
        {
            if (work._ninc <= 16 && work._neq <= 16 && work._nbIterations <= 1024)
            {
                t_spare = work;
            }
        }
    }

    private void Search(FunctionSetWithDerivatives f, math_Vector startingPoint, math_Vector infBound, math_Vector supBound,
                        bool stopOnDivergent, int ninc)
    {

        int i;
        bool changeDirection = false, sort = false, isNewSol = false;
        bool good, verif;
        bool stop;
        const double epsSqrt = 1.0e-16, eps = 1.0e-32, eps2 = 1.0e-64, progres = 0.005;
        double f2 = 0.0, previousMinimum, dy, oldF;
        double ambda, ambda2, gnr1 = 0.0, oldgr;
        math_Vector invLengthMax = new(1, ninc); // Pour bloquer les pas a 1/4 du domaine
        math_IntegerVector constraints = new(1, ninc); // Pour savoir sur quels bord on se trouve
        for (i = 1; i <= ninc; i++)
        {
            double aSupBound = Min(supBound[i], global::Kerf.Engine.Foundation.Precision.Infinite());
            double anInfBound = Max(infBound[i], -global::Kerf.Engine.Foundation.Precision.Infinite());
            invLengthMax[i] = 1.0 / Max((aSupBound - anInfBound) / 4, 1.0e-9);
        }

        MyDirFunction F_Dir = new(_temp1, _temp2, _temp3, _temp4, f);
        int descenteIter;

        _done = false;
        _sol.Initialized(startingPoint);
        _kount = 0;

        //
        _isDivergent = false;
        for (i = 1; i <= ninc; i++)
        {
            _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
        }

        if (stopOnDivergent && _isDivergent)
        {
            return;
        }

        // Verification de la validite des inconnues par rapport aux bornes.
        // Recentrage sur les bornes si pas valide.
        for (i = 1; i <= ninc; i++)
        {
            if (_sol[i] <= infBound[i])
            {
                _sol[i] = infBound[i];
            }
            else if (_sol[i] > supBound[i])
            {
                _sol[i] = supBound[i];
            }
        }

        // Calcul de la premiere valeur de F et de son gradient
        if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
        {
            _done = false;
            if (!stopOnDivergent || !_isDivergent)
            {
                _state = f.GetStateNumber();
            }

            return;
        }

        ambda2 = gnr1;
        // Rank 0 of Save should only be used for the accelerator test at the end of the loop.
        // If we are already on the solution, we need to bypass this test to avoid
        // making a second iteration...
        _save[0] = Max(f2, epsSqrt);
        double tol_Func = Epsilon(f2);

        if ((f2 <= eps) || (gnr1 <= eps2))
        {
            _done = false;
            if (!stopOnDivergent || !_isDivergent)
            {
                _done = true;
                _state = f.GetStateNumber();
            }

            return;
        }

        for (_kount = 1; _kount <= _itermax; _kount++)
        {
            previousMinimum = f2;
            oldgr = gnr1;
            _previousSolution.Initialized(_sol);
            _solSave.Initialized(_sol);

            SearchDirection(_searchWork, _df, _gh, _ff, changeDirection, invLengthMax, _dh, out dy);
            if (Math.Abs(dy) <= eps)
            {
                _done = false;
                if (!stopOnDivergent || !_isDivergent)
                {
                    _done = true;
                    ////modified by jgv, 31.08.2011////
                    f.Value(_sol, _ff); // update F before GetStateNumber
                    ///////////////////////////////////
                    _state = f.GetStateNumber();
                }

                return;
            }

            if (changeDirection)
            {
                ambda = ambda2 / Math.Sqrt(Math.Abs(dy));
                if (ambda > 1.0)
                {
                    ambda = 1.0;
                }
            }
            else
            {
                ambda = 1.0;
                ambda2 = 0.5 * ambda / _dh.Norm();
            }

            for (i = _sol.Lower(); i <= _sol.Upper(); i++)
            {
                _sol[i] = _sol[i] + ambda * _dh[i];
            }

            //
            for (i = 1; i <= ninc; i++)
            {
                _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
            }

            if (stopOnDivergent && _isDivergent)
            {
                return;
            }

            //
            sort = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);

            _dhSave.Initialized(_gh);
            if (isNewSol)
            {
                //      F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                {
                    _done = false;
                    if (!stopOnDivergent || !_isDivergent)
                    {
                        _state = f.GetStateNumber();
                    }

                    return;
                }
            }

            if ((f2 <= eps) || (gnr1 <= eps2))
            {
                _done = false;
                if (!stopOnDivergent || !_isDivergent)
                {
                    _done = true;
                    ////modified by jgv, 31.08.2011////
                    f.Value(_sol, _ff); // update F before GetStateNumber
                    ///////////////////////////////////
                    _state = f.GetStateNumber();
                }

                return;
            }

            if (sort || (f2 / previousMinimum > progres))
            {
                dy = _gh * _dh;
                oldF = previousMinimum;
                stop = false;
                good = false;
                descenteIter = 0;
                bool sortbis;

                // -------------------------------------------------
                // Standard processing without boundary handling
                // -------------------------------------------------
                if (!sort)
                { // if we haven't exited, we try to progress forward
                    while ((f2 / previousMinimum > progres) && !stop)
                    {
                        if (f2 < oldF && (dy < 0.0))
                        {
                            // We try to progress in this direction.
                            descenteIter++;
                            _solSave.Initialized(_sol);
                            oldF = f2;
                            for (i = _sol.Lower(); i <= _sol.Upper(); i++)
                            {
                                _sol[i] = _sol[i] + ambda * _dh[i];
                            }

                            //
                            for (i = 1; i <= ninc; i++)
                            {
                                _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
                            }

                            if (stopOnDivergent && _isDivergent)
                            {
                                return;
                            }

                            //
                            stop = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);
                            ambda *= 1.7;
                        }
                        else
                        {
                            if ((f2 >= oldF) || (f2 >= previousMinimum))
                            {
                                good = false;
                                if (descenteIter == 0)
                                {
                                    // C'est le premier pas qui flanche, on fait une interpolation.
                                    // et on minimise si necessaire.
                                    descenteIter++;
                                    good = MinimizeDirection(_solSave, _delta, oldF, f2, _dhSave, _gh, _tol, F_Dir);
                                }
                                else if (changeDirection || (descenteIter > 1) || (oldF > previousMinimum))
                                {
                                    // La progression a ete utile, on minimise...
                                    descenteIter++;
                                    good = MinimizeDirection(_previousSolution, _solSave, _sol, oldF, _delta, _tol, F_Dir);
                                }

                                if (!good)
                                {
                                    _sol.Initialized(_solSave);
                                    f2 = oldF;
                                }
                                else
                                {
                                    _sol.Initialized(_solSave + _delta);
                                    //
                                    for (i = 1; i <= ninc; i++)
                                    {
                                        _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
                                    }

                                    if (stopOnDivergent && _isDivergent)
                                    {
                                        return;
                                    }

                                    //
                                    sort = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);
                                }

                                sort = false; // On a rejete le point sur la frontiere
                            }

                            stop = true; // et on sort dans tous les cas...
                        }

                        _dhSave.Initialized(_gh);
                        if (isNewSol)
                        {
                            //            F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                            if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                            {
                                _done = false;
                                if (!stopOnDivergent || !_isDivergent)
                                {
                                    _state = f.GetStateNumber();
                                }

                                return;
                            }
                        }

                        dy = _gh * _dh;
                        if (Math.Abs(dy) <= eps)
                        {
                            if (f2 > oldF)
                            {
                                _sol.Initialized(_solSave);
                            }

                            _done = false;
                            if (!stopOnDivergent || !_isDivergent)
                            {
                                _done = true;
                                ////modified by jgv, 31.08.2011////
                                f.Value(_sol, _ff); // update F before GetStateNumber
                                ///////////////////////////////////
                                _state = f.GetStateNumber();
                            }

                            return;
                        }

                        if (descenteIter >= 100)
                        {
                            stop = true;
                        }
                    }
                }

                // ------------------------------------
                //  on passe au traitement des bords
                // ------------------------------------
                if (sort)
                {
                    stop = (f2 > 1.001 * oldF); // Pour ne pas progresser sur le bord
                    sortbis = sort;
                    descenteIter = 0;
                    while (sortbis && ((f2 < oldF) || (descenteIter == 0)) && (!stop))
                    {
                        descenteIter++;
                        // On essaye de progresser sur le bord
                        _solSave.Initialized(_sol);
                        oldF = f2;
                        SearchDirection(_searchWork, _df, _gh, _ff, constraints, changeDirection, invLengthMax, _dh, out dy);
                        if (dy < -eps)
                        { // Pour eviter des calculs inutiles et des /0...
                            if (changeDirection)
                            {
                                ambda = ambda2 / Math.Sqrt(-dy);
                                if (ambda > 1.0)
                                {
                                    ambda = 1.0;
                                }
                            }
                            else
                            {
                                ambda = 1.0;
                                ambda2 = 0.5 * ambda / _dh.Norm();
                            }

                            for (i = _sol.Lower(); i <= _sol.Upper(); i++)
                            {
                                _sol[i] = _sol[i] + ambda * _dh[i];
                            }

                            //
                            for (i = 1; i <= ninc; i++)
                            {
                                _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
                            }

                            if (stopOnDivergent && _isDivergent)
                            {
                                return;
                            }

                            //
                            sortbis = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);

                            _dhSave.Initialized(_gh);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                                {
                                    _done = false;
                                    if (!stopOnDivergent || !_isDivergent)
                                    {
                                        _state = f.GetStateNumber();
                                    }

                                    return;
                                }
                            }

                            ambda2 = gnr1;
                        }
                        else
                        {
                            stop = true;
                        }

                        while ((f2 / previousMinimum > progres) && (f2 < oldF) && (!stop))
                        {
                            descenteIter++;
                            if (f2 < oldF && dy < 0.0)
                            {
                                // On essaye de progresser dans cette direction.
                                _solSave.Initialized(_sol);
                                oldF = f2;
                                for (i = _sol.Lower(); i <= _sol.Upper(); i++)
                                {
                                    _sol[i] = _sol[i] + ambda * _dh[i];
                                }

                                //
                                for (i = 1; i <= ninc; i++)
                                {
                                    _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
                                }

                                if (stopOnDivergent && _isDivergent)
                                {
                                    return;
                                }

                                //
                                sortbis = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);
                            }

                            _dhSave.Initialized(_gh);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                                {
                                    _done = false;
                                    if (!stopOnDivergent || !_isDivergent)
                                    {
                                        _state = f.GetStateNumber();
                                    }

                                    return;
                                }
                            }

                            ambda2 = gnr1;
                            dy = _gh * _dh;
                            stop = ((dy >= 0) || (descenteIter >= 10) || sortbis);
                        }

                        stop = ((dy >= 0) || (descenteIter >= 10));
                    }

                    if (((f2 / previousMinimum > progres) && (f2 >= oldF)) || (f2 >= previousMinimum))
                    {
                        // On minimise par Brent
                        descenteIter++;
                        good = MinimizeDirection(_solSave, _delta, oldF, f2, _dhSave, _gh, _tol, F_Dir);
                        if (!good)
                        {
                            _sol.Initialized(_solSave);
                            sort = false;
                        }
                        else
                        {
                            _sol.Initialized(_solSave + _delta);
                            //
                            for (i = 1; i <= ninc; i++)
                            {
                                _isDivergent = _isDivergent || _sol[i] < infBound[i] || _sol[i] > supBound[i];
                            }

                            if (stopOnDivergent && _isDivergent)
                            {
                                return;
                            }

                            //
                            sort = Bounds(infBound, supBound, _tol, _sol, _solSave, constraints, _delta, out isNewSol);
                            if (isNewSol)
                            {
                                //              F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                                if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                                {
                                    _done = false;
                                    if (!stopOnDivergent || !_isDivergent)
                                    {
                                        _state = f.GetStateNumber();
                                    }

                                    return;
                                }
                            }
                        }

                        dy = _gh * _dh;
                    }
                }
            }

            // ---------------------------------------------
            //  on passe aux tests d'ARRET
            // ---------------------------------------------
            _save[_kount] = f2;
            // Est ce la solution ?
            if (changeDirection)
            {
                verif = true;
                // Gradient : Il faut eviter de boucler
            }
            else
            {
                verif = false;
                if (_kount > 1)
                { // Pour accelerer les cas quasi-quadratique
                    if (_save[_kount - 1] < 1.0e-4 * _save[_kount - 2])
                    {
                        verif = true;
                    }
                }
                else
                {
                    verif = (f2 < 1.0e-6 * _save[0]); // Pour les cas dejas solutions
                }
            }

            if (verif)
            {
                for (i = _delta.Lower(); i <= _delta.Upper(); i++)
                {
                    _delta[i] = _previousSolution[i] - _sol[i];
                }

                if (IsSolutionReached(f))
                {
                    if (previousMinimum < f2)
                    {
                        _sol.Initialized(_solSave);
                    }

                    _done = false;
                    if (!stopOnDivergent || !_isDivergent)
                    {
                        _done = true;
                        ////modified by jgv, 31.08.2011////
                        f.Value(_sol, _ff); // update F before GetStateNumber
                        ///////////////////////////////////
                        _state = f.GetStateNumber();
                    }

                    return;
                }
            }
            // fin du test solution

            // Analyse de la progression...
            // comparison of current minimum and previous minimum
            if ((f2 - previousMinimum) <= tol_Func)
            {
                if (_kount > 5)
                {
                    // L'historique est il bon ?
                    if (f2 >= 0.95 * _save[_kount - 5])
                    {
                        if (!changeDirection)
                        {
                            changeDirection = true;
                        }
                        else
                        {
                            _done = false;
                            if (!stopOnDivergent || !_isDivergent)
                            {
                                _done = true;
                                _state = f.GetStateNumber();
                            }

                            return; //  si un gain inf a 5% on sort
                        }
                    }
                    else
                    {
                        changeDirection = false; // If yes we restart
                    }
                }
                else
                {
                    changeDirection = false; // No history, we continue
                }

                // If the gradient does not decrease sufficiently with Newton, we try
                // the gradient method unless f decreases (as strange as it may seem,
                // with NEWTON the gradient of f can increase while f
                // decreases: in this case we must keep NEWTON)
                if ((gnr1 > 0.9 * oldgr) && (f2 > 0.5 * previousMinimum))
                {
                    changeDirection = true;
                }

                // If we don't decide to change strategy, we verify
                // if not already done
                if ((!changeDirection) && (!verif))
                {
                    for (i = _delta.Lower(); i <= _delta.Upper(); i++)
                    {
                        _delta[i] = _previousSolution[i] - _sol[i];
                    }

                    if (IsSolutionReached(f))
                    {
                        _done = false;
                        if (!stopOnDivergent || !_isDivergent)
                        {
                            _done = true;
                            ////modified by jgv, 31.08.2011////
                            f.Value(_sol, _ff); // update F before GetStateNumber
                            ///////////////////////////////////
                            _state = f.GetStateNumber();
                        }

                        return;
                    }
                }
            }
            else
            { // Cas de regression
                if (!changeDirection)
                { // On passe au gradient
                    changeDirection = true;
                    _sol.Initialized(_previousSolution);
                    //	F_Dir.Value(Sol, FF, DF, GH, F2, Gnr1);
                    if (!F_Dir.Value(_sol, _ff, _df, _gh, ref f2, ref gnr1))
                    {
                        _done = false;
                        if (!stopOnDivergent || !_isDivergent)
                        {
                            _state = f.GetStateNumber();
                        }

                        return;
                    }
                }
                else
                {
                    if (!stopOnDivergent || !_isDivergent)
                    {
                        _state = f.GetStateNumber();
                    }

                    return; // y a plus d'issues
                }
            }
        }

        if (!stopOnDivergent || !_isDivergent)
        {
            _state = f.GetStateNumber();
        }
    }

    public bool IsDone => _done;

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if the root was not found.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _kount;
        }
    }

    /// <summary>The state number (from F.GetStateNumber()) of the root. Throws <see cref="NotDoneException"/> if not found.</summary>
    public int StateNumber()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _state;
    }

    /// <summary>The root (shared: don't modify). Throws <see cref="NotDoneException"/> if not found.</summary>
    public math_Vector Root()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _sol;
    }

    /// <summary>
    /// Sets <paramref name="root"/> to the root. Throws <see cref="NotDoneException"/> if not found and
    /// <see cref="DimensionErrorException"/> if its length differs from the starting point's.
    /// </summary>
    public void Root(math_Vector root)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if(root.Length() != _sol.Length(), " ");
        root.Initialized(_sol);
    }

    /// <summary>The derivative matrix at the root (shared: don't modify). Throws <see cref="NotDoneException"/> if not found.</summary>
    public Matrix Derivative()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _df;
    }

    /// <summary>
    /// Sets <paramref name="der"/> to the derivative matrix at the root. Throws <see cref="NotDoneException"/> if not
    /// found and <see cref="DimensionErrorException"/> if its column count differs from the starting point's length.
    /// </summary>
    public void Derivative(Matrix der)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if(der.ColNumber() != _sol.Length(), " ");
        der.Initialized(_df);
    }

    /// <summary>The errors on the functions at the root (shared: don't modify). Throws <see cref="NotDoneException"/> if not found.</summary>
    public math_Vector FunctionSetErrors()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _delta;
    }

    /// <summary>
    /// Sets <paramref name="err"/> to the errors on the functions at the root. Throws <see cref="NotDoneException"/> if
    /// not found and <see cref="DimensionErrorException"/> if its length differs from the starting point's.
    /// </summary>
    public void FunctionSetErrors(math_Vector err)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if(err.Length() != _sol.Length(), " ");
        err.Initialized(_delta);
    }

    public bool IsDivergent => _isDivergent;

    public void Dump(TextWriter o)
    {
        o.Write(" math_FunctionSetRoot");
        if (_done)
        {
            o.Write(" Status = Done\n");
            o.Write(" Location value = " + _sol + "\n");
            o.Write(" Number of iterations = " + _kount + "\n");
        }
        else
        {
            o.Write("Status = Not Done\n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
