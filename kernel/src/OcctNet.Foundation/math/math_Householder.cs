// Port of: src/FoundationClasses/TKMath/math/math_Householder.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Householder.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Householder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The least square solution of n linear equations in m unknowns (n &gt;= m), A.X = B, by the Householder method.
/// More stable than GaussLeastSquare but slower; use it for singular or nearly singular matrices.
/// </summary>
public class math_Householder
{
    private math_Matrix Sol;
    private math_Matrix Q;
    private bool Done;
    private int mylowerArow;
    private int myupperArow;
    private int mylowerAcol;
    private int myupperAcol;

    // Cette classe decrit la methode de Householder qui transforme A en un
    // produit de matrice orthogonale par une triangulaire superieure. Les seconds
    // membres sont modifies dans le meme temps.
    // Les references sur le cote sont celles de l'algorithme explique en page
    // 90 du livre "Introduction a l'analyse numerique matricielle et a
    // l'optimisation." par P.G. CIARLET, edition MASSON. Les secondes
    // references sont celles du sous-programme HOUSEO d'Euclid.
    // A la difference du sous-programme Houseo, la premiere colonne n'est pas
    // traitee separement. Les tests effectues ont montre que le code effectue
    // specialement pour celle-ci etait plus long qu'une simple recopie. C'est
    // donc cette solution de recopie initiale qui a ete retenue.

    /// <summary>
    /// Solves A.X = B in the least square sense. If a column norm is less than <paramref name="EPS"/>, it can't be
    /// done. Throws <see cref="Standard_DimensionError"/> if the length of B differs from the rows of A.
    /// </summary>
    public math_Householder(math_Matrix A, math_Vector B, double EPS = 1.0e-20)
    {
        Sol = new math_Matrix(1, A.ColNumber(), 1, 1);
        Q = new math_Matrix(1, A.RowNumber(), 1, A.ColNumber());
        Done = false;

        mylowerArow = A.LowerRow();
        mylowerAcol = A.LowerCol();
        myupperArow = A.UpperRow();
        myupperAcol = A.UpperCol();
        math_Matrix B1 = new(1, B.Length(), 1, 1);
        B1.SetCol(1, B);
        Perform(A, B1, EPS);
    }

    /// <summary>
    /// Solves A.X = B in the least square sense for each column of B. If a column norm is less than
    /// <paramref name="EPS"/>, it can't be done. Throws <see cref="Standard_DimensionError"/> if the rows of B differ
    /// from the rows of A.
    /// </summary>
    public math_Householder(math_Matrix A, math_Matrix B, double EPS = 1.0e-20)
    {
        Sol = new math_Matrix(1, A.ColNumber(), 1, B.ColNumber());
        // OCCT gives Q A's column range, but Perform indexes its columns from 1: a matrix A whose LowerCol is not 1
        // reads outside Q (out of range here).
        Q = new math_Matrix(1, A.RowNumber(), A.LowerCol(), A.UpperCol());
        Done = false;

        mylowerArow = A.LowerRow();
        mylowerAcol = A.LowerCol();
        myupperArow = A.UpperRow();
        myupperAcol = A.UpperCol();
        Perform(A, B, EPS);
    }

    /// <summary>
    /// Solves A.X = B in the least square sense for each column of B, using only the given rows and columns of A.
    /// </summary>
    public math_Householder(math_Matrix A, math_Matrix B, int lowerArow, int upperArow, int lowerAcol, int upperAcol,
                            double EPS = 1.0e-20)
    {
        Sol = new math_Matrix(1, upperAcol - lowerAcol + 1, 1, B.ColNumber());
        Q = new math_Matrix(1, upperArow - lowerArow + 1, 1, upperAcol - lowerAcol + 1);
        Done = false;

        mylowerArow = lowerArow;
        myupperArow = upperArow;
        mylowerAcol = lowerAcol;
        myupperAcol = upperAcol;
        Perform(A, B, EPS);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Perform(math_Matrix A, math_Matrix B, double EPS)
    {
        int i, j, k, n, l, m;
        double f, g, h = 0.0, alfaii;
        double qki;
        double cj;
        n = Q.ColNumber();
        l = Q.RowNumber();
        m = B.ColNumber();
        math_Matrix B2 = new(1, l, 1, m);

        int lbrow = B.LowerRow();
        for (i = 1; i <= l; i++)
        {
            for (j = 1; j <= n; j++)
            {
                Q[i, j] = A[i + mylowerArow - 1, j + mylowerAcol - 1];
            }

            for (j = 1; j <= m; j++)
            {
                // OCCT indexes B's columns from 1 whatever its LowerCol
                B2[i, j] = B[i + lbrow - 1, j];
            }
        }

        // OCCT checks the dimensions only after copying B, so a B with too few rows reads out of range first
        Standard_DimensionError_Raise_if(l != B.RowNumber() || n > l, " ");

        // Traitement de chaque colonne de A:
        for (i = 1; i <= n; i++)
        {
            h = 0.0;
            for (k = i; k <= l; k++)
            {
                qki = Q[k, i];
                h += qki * qki; // = ||a||*||a||     = EUAI
            }

            f = Q[i, i]; // = a1              = AII
            g = f < 1.0e-15 ? Math.Sqrt(h) : -Math.Sqrt(h);
            if (Math.Abs(g) <= EPS)
            {
                Done = false;
                return;
            }

            h -= f * g;     // = (v*v)/2         = C1
            alfaii = g - f; // = v               = ALFAII
            for (j = i + 1; j <= n; j++)
            {
                double scale = 0.0;
                for (k = i; k <= l; k++)
                {
                    scale += Q[k, i] * Q[k, j]; //                   = SCAL
                }

                cj = (g * Q[i, j] - scale) / h;
                Q[i, j] = Q[i, j] - alfaii * cj;
                for (k = i + 1; k <= l; k++)
                {
                    Q[k, j] = Q[k, j] + cj * Q[k, i];
                }
            }

            // Modification de B:
            for (j = 1; j <= m; j++)
            {
                double scale = Q[i, i] * B2[i, j];
                for (k = i + 1; k <= l; k++)
                {
                    scale += Q[k, i] * B2[k, j];
                }

                cj = (g * B2[i, j] - scale) / h;
                B2[i, j] = B2[i, j] - cj * alfaii;
                for (k = i + 1; k <= l; k++)
                {
                    B2[k, j] = B2[k, j] + cj * Q[k, i];
                }
            }

            Q[i, i] = g;
        }

        // Remontee:
        for (j = 1; j <= m; j++)
        {
            Sol[n, j] = B2[n, j] / Q[n, n];
            for (i = n - 1; i >= 1; i--)
            {
                double scale = 0.0;
                for (k = i + 1; k <= n; k++)
                {
                    scale += Q[i, k] * Sol[k, j];
                }

                Sol[i, j] = (B2[i, j] - scale) / Q[i, i];
            }
        }

        Done = true;
    }

    public bool IsDone() => Done;

    /// <summary>
    /// Sets <paramref name="sol"/> to the solution for column <paramref name="Index"/> of B. Throws
    /// <see cref="StdFail_NotDone"/> if not done and <see cref="Standard_OutOfRange"/> if Index is out of 1..B's columns.
    /// </summary>
    public void Value(math_Vector sol, int Index = 1)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_OutOfRange_Raise_if((Index < 1) || (Index > Sol.ColNumber()), " ");
        sol.Initialized(Sol.Col(Index));
    }

    /// <summary>All the solutions, one per column (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Matrix AllValues()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Sol;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Householder ");
        if (Done)
        {
            o.Write(" Status = Done \n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
