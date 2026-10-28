// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Uzawa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// Solves C*X = B near an approximate solution X0, minimizing Norm(X-X0), by the Uzawa algorithm. There are no
/// conditions on the number of equations, which can be equalities or inequalities (&lt;). With only equalities the
/// resolution is direct (by the Crout algorithm).
/// </summary>
public class math_Uzawa
{
    private math_Vector Resul;
    private math_Vector Erruza;
    private math_Vector Errinit;
    private math_Vector Vardua;
    private math_Matrix CTCinv;
    private int NbIter;
    private bool Done;

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
    /// EpsLic on the dual variables. Throws <see cref="Standard_DimensionError"/> if Cont's rows differ from Secont's length.
    /// </summary>
    public math_Uzawa(math_Matrix Cont, math_Vector Secont, math_Vector StartingPoint,
                      double EpsLix = 1.0e-06, double EpsLic = 1.0e-06, int NbIterations = 500)
    {
        Resul = new math_Vector(1, Cont.ColNumber());
        Erruza = new math_Vector(1, Cont.ColNumber());
        Errinit = new math_Vector(1, Cont.ColNumber());
        Vardua = new math_Vector(1, Cont.RowNumber());
        CTCinv = new math_Matrix(1, Cont.RowNumber(), 1, Cont.RowNumber());
        NbIter = 0;
        Done = false;

        Perform(Cont, Secont, StartingPoint, Cont.RowNumber(), 0, EpsLix, EpsLic, NbIterations);
    }

    /// <summary>
    /// Solves Cont*X = Secont, the Nce first equations equalities and the Nci last inequalities (&lt;), minimizing
    /// Norm(X-StartingPoint). Throws <see cref="Standard_DimensionError"/> if Cont's rows differ from Secont's length
    /// or from Nce + Nci.
    /// </summary>
    /// <remarks>OCCT's header names the 4th and 5th parameters Nci, Nce; its definition, which counts, Nce, Nci.</remarks>
    public math_Uzawa(math_Matrix Cont, math_Vector Secont, math_Vector StartingPoint, int Nce, int Nci,
                      double EpsLix = 1.0e-06, double EpsLic = 1.0e-06, int NbIterations = 500)
    {
        Resul = new math_Vector(1, Cont.ColNumber());
        Erruza = new math_Vector(1, Cont.ColNumber());
        Errinit = new math_Vector(1, Cont.ColNumber());
        Vardua = new math_Vector(1, Cont.RowNumber());
        CTCinv = new math_Matrix(1, Cont.RowNumber(), 1, Cont.RowNumber());
        NbIter = 0;
        Done = false;

        Perform(Cont, Secont, StartingPoint, Nce, Nci, EpsLix, EpsLic, NbIterations);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Perform(math_Matrix Cont, math_Vector Secont, math_Vector StartingPoint, int Nce, int Nci,
                           double EpsLix = 1.0e-06, double EpsLic = 1.0e-06, int NbIterations = 500)
    {
        int i, j, k;
        double scale;
        double Normat, Normli, Xian, Xmax = 0, Xmuian;
        double Rho, Err, Err1, ErrMax = 0, Coef = 1.0 / Math.Sqrt(2.0);
        int Nlig = Cont.RowNumber();
        int Ncol = Cont.ColNumber();

        Standard_DimensionError_Raise_if((Secont.Length() != Nlig) || ((Nce + Nci) != Nlig), " ");

        // Calcul du vecteur Cont*X0 - D:  (erreur initiale)
        //==================================================
        // OCCT: Errinit has Ncol values but is indexed up to Nlig here and below; with more equations than unknowns
        // OCCT writes past it (here: Standard_OutOfRange).
        for (i = 1; i <= Nlig; i++)
        {
            Errinit[i] = Cont[i, 1] * StartingPoint[1] - Secont[i];
            for (j = 2; j <= Ncol; j++)
            {
                Errinit[i] += Cont[i, j] * StartingPoint[j];
            }
        }

        if (Nci == 0)
        {             // cas de resolution directe
            NbIter = 1; //==========================
                        // Calcul de Cont*T(Cont)
            for (i = 1; i <= Nlig; i++)
            {
                for (j = 1; j <= i; j++)
                { // a utiliser avec Crout.
                  //      for (j = 1; j <= Nlig; j++) {        // a utiliser pour Gauss.
                    CTCinv[i, j] = Cont[i, 1] * Cont[j, 1];
                    for (k = 2; k <= Ncol; k++)
                    {
                        CTCinv[i, j] += Cont[i, k] * Cont[j, k];
                    }
                }
            }

            // Calcul de l inverse de CTCinv :
            //================================
            //      CTCinv = CTCinv.Inverse();           // utilisation de Gauss.
            math_Crout inv = new(CTCinv); // utilisation de Crout.
            CTCinv.Initialized(inv.Inverse());
            for (i = 1; i <= Nlig; i++)
            {
                scale = CTCinv[i, 1] * Errinit[1];
                for (j = 2; j <= i; j++)
                {
                    scale += CTCinv[i, j] * Errinit[j];
                }

                for (j = i + 1; j <= Nlig; j++)
                {
                    scale += CTCinv[j, i] * Errinit[j];
                }

                Vardua[i] = scale;
            }

            for (i = 1; i <= Ncol; i++)
            {
                Erruza[i] = -Cont[1, i] * Vardua[1];
                for (j = 2; j <= Nlig; j++)
                {
                    Erruza[i] -= Cont[j, i] * Vardua[j];
                }
            }

            // restitution des valeurs calculees:
            //===================================
            Resul.Initialized(StartingPoint + Erruza);
            Done = true;
            return;
        } // Fin de la resolution directe.
          //==============================

        else
        {
            // Initialisation des variables duales.
            //=====================================
            for (i = 1; i <= Nlig; i++)
            {
                if (i <= Nce)
                {
                    Vardua[i] = 0.0;
                }
                else
                {
                    Vardua[i] = 1.0;
                }
            }

            // Calcul du coefficient Rho:
            //===========================
            Normat = 0.0;
            for (i = 1; i <= Nlig; i++)
            {
                Normli = Cont[i, 1] * Cont[i, 1];
                for (j = 2; j <= Ncol; j++)
                {
                    Normli += Cont[i, j] * Cont[i, j];
                }

                Normat += Normli;
            }

            Rho = Coef / Normat;

            // Boucle des iterations de la methode d Uzawa.
            //=============================================
            for (NbIter = 1; NbIter <= NbIterations; NbIter++)
            {
                for (i = 1; i <= Ncol; i++)
                {
                    Xian = Erruza[i];
                    Erruza[i] = -Cont[1, i] * Vardua[1];
                    for (j = 2; j <= Nlig; j++)
                    {
                        Erruza[i] -= Cont[j, i] * Vardua[j];
                    }

                    if (NbIter > 1)
                    {
                        if (i == 1)
                        {
                            Xmax = Math.Abs(Erruza[i] - Xian);
                        }

                        Xmax = Max(Xmax, Math.Abs(Erruza[i] - Xian));
                    }
                }

                // Calcul de Xmu a l iteration NbIter et evaluation de l erreur sur
                // la verification des contraintes.
                //=================================================================
                for (i = 1; i <= Nlig; i++)
                {
                    Err = Cont[i, 1] * Erruza[1] + Errinit[i];
                    for (j = 2; j <= Ncol; j++)
                    {
                        Err += Cont[i, j] * Erruza[j];
                    }

                    if (i <= Nce)
                    {
                        Vardua[i] += Rho * Err;
                        Err1 = Math.Abs(Rho * Err);
                    }
                    else
                    {
                        Xmuian = Vardua[i];
                        Vardua[i] = Max(0.0, Vardua[i] + Rho * Err);
                        Err1 = Math.Abs(Vardua[i] - Xmuian);
                    }

                    if (i == 1)
                    {
                        ErrMax = Err1;
                    }

                    ErrMax = Max(ErrMax, Err1);
                }

                if (NbIter > 1)
                {
                    if (Xmax <= EpsLix)
                    {
                        if (ErrMax <= EpsLic)
                        {
                            Done = true;
                        }
                        else
                        {
                            Done = false;
                            return;
                        }

                        // Restitution des valeurs calculees
                        //==================================
                        Resul.Initialized(StartingPoint + Erruza);
                        Done = true;
                        return;
                    }
                }
            } // fin de la boucle d iterations.

            Done = false;
        }
    }

    public bool IsDone() => Done;

    /// <summary>The solution (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Vector Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Resul;
    }

    /// <summary>The initial error Cont*StartingPoint-Secont (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Vector InitialError()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Errinit;
    }

    /// <summary>Sets <paramref name="V"/> to the dual variables of the system.</summary>
    public void Duale(math_Vector V)
    {
        V.Initialized(Vardua);
    }

    /// <summary>The difference between the solution and the starting point (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Vector Error()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Erruza;
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return NbIter;
    }

    /// <summary>
    /// The inverse of C * Transposed(C), lower triangle (shared: don't modify), needed for the gradient when
    /// approximating a curve. Throws <see cref="StdFail_NotDone"/> if not done.
    /// </summary>
    public math_Matrix InverseCont()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return CTCinv;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Uzawa");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + NbIter + '\n');
            o.Write(" The solution vector is: " + Resul + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
