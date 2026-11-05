// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// Solves C*X = B near an approximate solution X0, minimizing Norm(X-X0), by the Uzawa algorithm. There are no
/// conditions on the number of equations, which can be equalities or inequalities (&lt;). With only equalities the
/// resolution is direct (by the Crout algorithm).
/// </summary>
internal class Uzawa
{
    private math_Vector _resul;
    private math_Vector _erruza;
    private math_Vector _errinit;
    private math_Vector _vardua;
    private Matrix _ctCinv;
    private int _nbIter;
    private bool _done;

    // lpa le 8/08/91
    // Ce programme utilise l algorithme d Uzawa pour resoudre un systeme
    // dans le cas de contraintes. Si ce sont des contraintes d egalite, la
    // resolution est directe.
    // Le programme ci-dessous utilise la methode de Crout pour trouver
    // l inverse d une matrice symetrique (Le gain est d environ 30% par
    // rapport a Gauss.). Les calculs sur les matrices sont faits avec chaque
    // coordonnee car il est plus long d utiliser les methodes deja ecrites
    // de la classe Matrix avec un passage par valeur.

    /// <summary>
    /// Solves Cont*X = Secont (equalities only) minimizing Norm(X-StartingPoint). EpsLix is the tolerance on X,
    /// EpsLic on the dual variables. Throws <see cref="DimensionErrorException"/> if Cont's rows differ from Secont's length.
    /// </summary>
    public Uzawa(Matrix cont, math_Vector secont, math_Vector startingPoint,
                      double epsLix = 1.0e-06, double epsLic = 1.0e-06, int nbIterations = 500)
    {
        _resul = new math_Vector(1, cont.ColNumber());
        _erruza = new math_Vector(1, cont.ColNumber());
        _errinit = new math_Vector(1, cont.ColNumber());
        _vardua = new math_Vector(1, cont.RowNumber());
        _ctCinv = new Matrix(1, cont.RowNumber(), 1, cont.RowNumber());
        _nbIter = 0;
        _done = false;

        Perform(cont, secont, startingPoint, cont.RowNumber(), 0, epsLix, epsLic, nbIterations);
    }

    /// <summary>
    /// Solves Cont*X = Secont, the Nce first equations equalities and the Nci last inequalities (&lt;), minimizing
    /// Norm(X-StartingPoint). Throws <see cref="DimensionErrorException"/> if Cont's rows differ from Secont's length
    /// or from Nce + Nci.
    /// </summary>
    /// <remarks>OCCT's header names the 4th and 5th parameters Nci, Nce; its definition, which counts, Nce, Nci.</remarks>
    public Uzawa(Matrix cont, math_Vector secont, math_Vector startingPoint, int nce, int nci,
                      double epsLix = 1.0e-06, double epsLic = 1.0e-06, int nbIterations = 500)
    {
        _resul = new math_Vector(1, cont.ColNumber());
        _erruza = new math_Vector(1, cont.ColNumber());
        _errinit = new math_Vector(1, cont.ColNumber());
        _vardua = new math_Vector(1, cont.RowNumber());
        _ctCinv = new Matrix(1, cont.RowNumber(), 1, cont.RowNumber());
        _nbIter = 0;
        _done = false;

        Perform(cont, secont, startingPoint, nce, nci, epsLix, epsLic, nbIterations);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Perform(Matrix cont, math_Vector secont, math_Vector startingPoint, int nce, int nci,
                           double epsLix = 1.0e-06, double epsLic = 1.0e-06, int nbIterations = 500)
    {
        int i, j, k;
        double scale;
        double normat, normli, xian, xmax = 0, xmuian;
        double rho, err, err1, errMax = 0, coef = 1.0 / Math.Sqrt(2.0);
        int nlig = cont.RowNumber();
        int ncol = cont.ColNumber();

        Standard_DimensionError_Raise_if((secont.Length() != nlig) || ((nce + nci) != nlig), " ");

        // Calcul du vecteur Cont*X0 - D:  (erreur initiale)
        //==================================================
        // OCCT: Errinit has Ncol values but is indexed up to Nlig here and below; with more equations than unknowns
        // OCCT writes past it (here: Standard_OutOfRange).
        for (i = 1; i <= nlig; i++)
        {
            _errinit[i] = cont[i, 1] * startingPoint[1] - secont[i];
            for (j = 2; j <= ncol; j++)
            {
                _errinit[i] += cont[i, j] * startingPoint[j];
            }
        }

        if (nci == 0)
        {             // cas de resolution directe
            _nbIter = 1; //==========================
                        // Calcul de Cont*T(Cont)
            for (i = 1; i <= nlig; i++)
            {
                for (j = 1; j <= i; j++)
                { // a utiliser avec Crout.
                  //      for (j = 1; j <= Nlig; j++) {        // a utiliser pour Gauss.
                    _ctCinv[i, j] = cont[i, 1] * cont[j, 1];
                    for (k = 2; k <= ncol; k++)
                    {
                        _ctCinv[i, j] += cont[i, k] * cont[j, k];
                    }
                }
            }

            // Calcul de l inverse de CTCinv :
            //================================
            //      CTCinv = CTCinv.Inverse();           // utilisation de Gauss.
            Crout inv = new(_ctCinv); // utilisation de Crout.
            _ctCinv.Initialized(inv.Inverse());
            for (i = 1; i <= nlig; i++)
            {
                scale = _ctCinv[i, 1] * _errinit[1];
                for (j = 2; j <= i; j++)
                {
                    scale += _ctCinv[i, j] * _errinit[j];
                }

                for (j = i + 1; j <= nlig; j++)
                {
                    scale += _ctCinv[j, i] * _errinit[j];
                }

                _vardua[i] = scale;
            }

            for (i = 1; i <= ncol; i++)
            {
                _erruza[i] = -cont[1, i] * _vardua[1];
                for (j = 2; j <= nlig; j++)
                {
                    _erruza[i] -= cont[j, i] * _vardua[j];
                }
            }

            // restitution des valeurs calculees:
            //===================================
            _resul.Initialized(startingPoint + _erruza);
            _done = true;
            return;
        } // Fin de la resolution directe.
          //==============================

        else
        {
            // Initialisation des variables duales.
            //=====================================
            for (i = 1; i <= nlig; i++)
            {
                if (i <= nce)
                {
                    _vardua[i] = 0.0;
                }
                else
                {
                    _vardua[i] = 1.0;
                }
            }

            // Calcul du coefficient Rho:
            //===========================
            normat = 0.0;
            for (i = 1; i <= nlig; i++)
            {
                normli = cont[i, 1] * cont[i, 1];
                for (j = 2; j <= ncol; j++)
                {
                    normli += cont[i, j] * cont[i, j];
                }

                normat += normli;
            }

            rho = coef / normat;

            // Boucle des iterations de la methode d Uzawa.
            //=============================================
            for (_nbIter = 1; _nbIter <= nbIterations; _nbIter++)
            {
                for (i = 1; i <= ncol; i++)
                {
                    xian = _erruza[i];
                    _erruza[i] = -cont[1, i] * _vardua[1];
                    for (j = 2; j <= nlig; j++)
                    {
                        _erruza[i] -= cont[j, i] * _vardua[j];
                    }

                    if (_nbIter > 1)
                    {
                        if (i == 1)
                        {
                            xmax = Math.Abs(_erruza[i] - xian);
                        }

                        xmax = Max(xmax, Math.Abs(_erruza[i] - xian));
                    }
                }

                // Calcul de Xmu a l iteration NbIter et evaluation de l erreur sur
                // la verification des contraintes.
                //=================================================================
                for (i = 1; i <= nlig; i++)
                {
                    err = cont[i, 1] * _erruza[1] + _errinit[i];
                    for (j = 2; j <= ncol; j++)
                    {
                        err += cont[i, j] * _erruza[j];
                    }

                    if (i <= nce)
                    {
                        _vardua[i] += rho * err;
                        err1 = Math.Abs(rho * err);
                    }
                    else
                    {
                        xmuian = _vardua[i];
                        _vardua[i] = Max(0.0, _vardua[i] + rho * err);
                        err1 = Math.Abs(_vardua[i] - xmuian);
                    }

                    if (i == 1)
                    {
                        errMax = err1;
                    }

                    errMax = Max(errMax, err1);
                }

                if (_nbIter > 1)
                {
                    if (xmax <= epsLix)
                    {
                        if (errMax <= epsLic)
                        {
                            _done = true;
                        }
                        else
                        {
                            _done = false;
                            return;
                        }

                        // Restitution des valeurs calculees
                        //==================================
                        _resul.Initialized(startingPoint + _erruza);
                        _done = true;
                        return;
                    }
                }
            } // fin de la boucle d iterations.

            _done = false;
        }
    }

    public bool IsDone => _done;

    /// <summary>The solution (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public math_Vector Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _resul;
    }

    /// <summary>The initial error Cont*StartingPoint-Secont (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public math_Vector InitialError()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _errinit;
    }

    /// <summary>Sets <paramref name="v"/> to the dual variables of the system.</summary>
    public void Duale(math_Vector v)
    {
        v.Initialized(_vardua);
    }

    /// <summary>The difference between the solution and the starting point (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public math_Vector Error()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _erruza;
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if not done.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _nbIter;
        }
    }

    /// <summary>
    /// The inverse of C * Transposed(C), lower triangle (shared: don't modify), needed for the gradient when
    /// approximating a curve. Throws <see cref="NotDoneException"/> if not done.
    /// </summary>
    public Matrix InverseCont()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _ctCinv;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Uzawa");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + _nbIter + '\n');
            o.Write(" The solution vector is: " + _resul + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
