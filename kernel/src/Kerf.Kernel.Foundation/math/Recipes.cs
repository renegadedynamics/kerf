// Port of: src/FoundationClasses/TKMath/math/math_Recipes.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Recipes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The free functions of math_Recipes.hxx (LU, SVD, DACTCL and Jacobi decompositions, after Numerical Recipes).
/// Ported code imports them with <c>using static Kerf.Engine.math_Recipes;</c> and calls them unqualified, as in C++.
/// </summary>
/// <remarks>The matrices and vectors are indexed from 1, as OCCT requires.</remarks>
internal static class Recipes
{
    public const int MathStatusUserAborted = -1;
    public const int MathStatusOK = 0;
    public const int MathStatusSingularMatrix = 1;
    public const int MathStatusArgumentError = 2;
    public const int MathStatusNoConvergence = 3;

    private static double PYTHAG(double a, double b)
    {
        double at = Math.Abs(a), bt = Math.Abs(b), ct = 0.0;
        if (at > bt)
        {
            ct = bt / at;
            ct = at * Math.Sqrt(1.0 + ct * ct);
        }
        else if (bt != 0.0)
        {
            ct = at / bt;
            ct = bt * Math.Sqrt(1.0 + ct * ct);
        }

        return ct;
    }

    private static double SIGN(double a, double b) => b >= 0.0 ? Math.Abs(a) : -Math.Abs(a);

    // The ROTATE macro of math_Recipes.cxx.
    private static void ROTATE(Matrix a, int i, int j, int k, int l, double s, double tau, ref double g, ref double h)
    {
        g = a[i, j];
        h = a[k, l];
        a[i, j] = g - s * (h + g * tau);
        a[k, l] = h + s * (g - h * tau);
    }

    private static void EigenSort(math_Vector d, Matrix v)
    { // descending order
        int k, i, j;
        double p;
        int n = d.Length();

        for (i = 1; i < n; i++)
        {
            p = d[k = i];
            for (j = i + 1; j <= n; j++)
            {
                if (d[j] >= p)
                {
                    p = d[k = j];
                }
            }

            if (k != i)
            {
                d[k] = d[i];
                d[i] = p;
                for (j = 1; j <= n; j++)
                {
                    p = v[j, i];
                    v[j, i] = v[j, k];
                    v[j, k] = p;
                }
            }
        }
    }

    /// <summary>
    /// Computes all eigenvalues and eigenvectors of a real symmetric matrix a(1..n, 1..n). On output, elements of a
    /// above the diagonal are destroyed. d(1..n) returns the eigenvalues, sorted into descending order; the columns of
    /// v(1..n, 1..n) the normalized eigenvectors; nrot the number of Jacobi rotations.
    /// </summary>
    public static int Jacobi(Matrix a, math_Vector d, Matrix v, out int nrot)
        => Jacobi(a, d, v, out nrot, new math_Vector(1, a.RowNumber()), new math_Vector(1, a.RowNumber()));

    /// <summary>
    /// <see cref="Jacobi(Matrix, math_Vector, Matrix, out int)"/> with its two work vectors b(1..n) and z(1..n) given,
    /// for callers that repeat it (unit 10J); their values on entry don't matter.
    /// </summary>
    internal static int Jacobi(Matrix a, math_Vector d, Matrix v, out int nrot, math_Vector b, math_Vector z)
    {
        int n = a.RowNumber();
        int j, iq, ip, i;
        double tresh, theta, tau, t, sm, s, h = 0.0, g = 0.0, c;

        for (ip = 1; ip <= n; ip++)
        {
            for (iq = 1; iq <= n; iq++)
            {
                v[ip, iq] = 0.0;
            }

            v[ip, ip] = 1.0;
        }

        for (ip = 1; ip <= n; ip++)
        {
            b[ip] = d[ip] = a[ip, ip];
            z[ip] = 0.0;
        }

        nrot = 0;
        for (i = 1; i <= 50; i++)
        {
            sm = 0.0;
            for (ip = 1; ip < n; ip++)
            {
                for (iq = ip + 1; iq <= n; iq++)
                {
                    sm += Math.Abs(a[ip, iq]);
                }
            }

            if (sm == 0.0)
            {
                EigenSort(d, v);
                return MathStatusOK;
            }

            if (i < 4)
            {
                tresh = 0.2 * sm / (n * n);
            }
            else
            {
                tresh = 0.0;
            }

            for (ip = 1; ip < n; ip++)
            {
                for (iq = ip + 1; iq <= n; iq++)
                {
                    g = 100.0 * Math.Abs(a[ip, iq]);
                    if (i > 4 && Math.Abs(d[ip]) + g == Math.Abs(d[ip]) && Math.Abs(d[iq]) + g == Math.Abs(d[iq]))
                    {
                        a[ip, iq] = 0.0;
                    }
                    else if (Math.Abs(a[ip, iq]) > tresh)
                    {
                        h = d[iq] - d[ip];
                        if (Math.Abs(h) + g == Math.Abs(h))
                        {
                            t = a[ip, iq] / h;
                        }
                        else
                        {
                            theta = 0.5 * h / a[ip, iq];
                            t = 1.0 / (Math.Abs(theta) + Math.Sqrt(1.0 + theta * theta));
                            if (theta < 0.0)
                            {
                                t = -t;
                            }
                        }

                        c = 1.0 / Math.Sqrt(1 + t * t);
                        s = t * c;
                        tau = s / (1.0 + c);
                        h = t * a[ip, iq];
                        z[ip] -= h;
                        z[iq] += h;
                        d[ip] -= h;
                        d[iq] += h;
                        a[ip, iq] = 0.0;
                        for (j = 1; j < ip; j++)
                        {
                            ROTATE(a, j, ip, j, iq, s, tau, ref g, ref h);
                        }

                        for (j = ip + 1; j < iq; j++)
                        {
                            ROTATE(a, ip, j, j, iq, s, tau, ref g, ref h);
                        }

                        for (j = iq + 1; j <= n; j++)
                        {
                            ROTATE(a, ip, j, iq, j, s, tau, ref g, ref h);
                        }

                        for (j = 1; j <= n; j++)
                        {
                            ROTATE(v, j, ip, j, iq, s, tau, ref g, ref h);
                        }

                        nrot++;
                    }
                }
            }

            for (ip = 1; ip <= n; ip++)
            {
                b[ip] += z[ip];
                d[ip] = b[ip];
                z[ip] = 0.0;
            }
        }

        EigenSort(d, v);
        return MathStatusNoConvergence;
    }

    /// <summary>
    /// The LU decomposition of a(1..n, 1..n) with partial pivoting, replacing a. indx(1..n) records the row
    /// permutation; d is +1 or -1 as the number of row interchanges is even or odd. vv(1..n) is scratch space.
    /// </summary>
    public static int LU_Decompose(Matrix a, math_IntegerVector indx, out double d, math_Vector vv,
                                   double tiny = 1.0e-30, ProgressRange progress = default)
    {
        int i, imax = 0, j, k;
        double big, dum, sum, temp;

        int n = a.RowNumber();
        d = 1.0;

        // None when nothing is watching, nearly always: math_NewtonMinimum comes here at every iteration (unit 11B)
        using ProgressScope? ps = progress.IsActive ? new ProgressScope(progress, "math_Gauss LU_Decompose", n) : null;

        for (i = 1; i <= n; i++)
        {
            big = 0.0;
            for (j = 1; j <= n; j++)
            {
                if ((temp = Math.Abs(a[i, j])) > big)
                {
                    big = temp;
                }
            }

            if (big <= tiny)
            {
                return MathStatusSingularMatrix;
            }

            vv[i] = 1.0 / big;
        }

        for (j = 1; j <= n && (ps is null || ps.More()); j++, ps?.Next())
        {
            for (i = 1; i < j; i++)
            {
                sum = a[i, j];
                for (k = 1; k < i; k++)
                {
                    sum -= a[i, k] * a[k, j];
                }

                a[i, j] = sum;
            }

            big = 0.0;
            for (i = j; i <= n; i++)
            {
                sum = a[i, j];
                for (k = 1; k < j; k++)
                {
                    sum -= a[i, k] * a[k, j];
                }

                a[i, j] = sum;
                // Note that comparison is made so as to have imax updated even if argument is NAN, Inf or
                // IND, see #25559
                if ((dum = vv[i] * Math.Abs(sum)) < big)
                {
                    continue;
                }

                big = dum;
                imax = i;
            }

            if (j != imax)
            {
                for (k = 1; k <= n; k++)
                {
                    dum = a[imax, k];
                    a[imax, k] = a[j, k];
                    a[j, k] = dum;
                }

                d = -d;
                vv[imax] = vv[j];
            }

            indx[j] = imax;
            if (Math.Abs(a[j, j]) <= tiny)
            {
                return MathStatusSingularMatrix;
            }

            if (j != n)
            {
                dum = 1.0 / (a[j, j]);
                for (i = j + 1; i <= n; i++)
                {
                    a[i, j] *= dum;
                }
            }
        }

        if (j <= n)
        {
            return MathStatusUserAborted;
        }

        return MathStatusOK;
    }

    /// <summary>
    /// The LU decomposition of a(1..n, 1..n) with partial pivoting, replacing a. indx(1..n) records the row
    /// permutation; d is +1 or -1 as the number of row interchanges is even or odd.
    /// </summary>
    public static int LU_Decompose(Matrix a, math_IntegerVector indx, out double d,
                                   double tiny = 1.0e-20, ProgressRange progress = default)
    {
        math_Vector vv = new(1, a.RowNumber());
        return LU_Decompose(a, indx, out d, vv, tiny, progress);
    }

    /// <summary>
    /// Solves a * x = b, with a and indx as returned by <see cref="LU_Decompose(Matrix, math_IntegerVector, out double, double, ProgressRange)"/>.
    /// b is replaced by the solution.
    /// </summary>
    public static void LU_Solve(Matrix a, math_IntegerVector indx, math_Vector b)
    {
        int i, ii = 0, ip, j;
        double sum;

        int n = a.RowNumber();
        int nblow = b.Lower() - 1;
        for (i = 1; i <= n; i++)
        {
            ip = indx[i];
            sum = b[ip + nblow];
            b[ip + nblow] = b[i + nblow];
            if (ii != 0)
            {
                for (j = ii; j < i; j++)
                {
                    sum -= a[i, j] * b[j + nblow];
                }
            }
            else if (sum != 0.0)
            {
                ii = i;
            }

            b[i + nblow] = sum;
        }

        for (i = n; i >= 1; i--)
        {
            sum = b[i + nblow];
            for (j = i + 1; j <= n; j++)
            {
                sum -= a[i, j] * b[j + nblow];
            }

            b[i + nblow] = sum / a[i, i];
        }
    }

    /// <summary>Replaces a(1..n, 1..n) by its inverse.</summary>
    public static int LU_Invert(Matrix a)
    {
        int n = a.RowNumber();
        Matrix inv = new(1, n, 1, n);
        math_Vector col = new(1, n);
        math_IntegerVector indx = new(1, n);
        int i, j;

        int error = LU_Decompose(a, indx, out _);
        if (error == 0)
        {
            for (j = 1; j <= n; j++)
            {
                for (i = 1; i <= n; i++)
                {
                    col[i] = 0.0;
                }

                col[j] = 1.0;
                LU_Solve(a, indx, col);
                for (i = 1; i <= n; i++)
                {
                    inv[i, j] = col[i];
                }
            }

            for (j = 1; j <= n; j++)
            {
                for (i = 1; i <= n; i++)
                {
                    a[i, j] = inv[i, j];
                }
            }
        }

        return error;
    }

    /// <summary>
    /// The singular value decomposition a = u * w * transposed(v) of a(1..m, 1..n), m &gt;= n. u replaces a, the
    /// singular values are output in w(1..n), and v(1..n, 1..n).
    /// </summary>
    public static int SVD_Decompose(Matrix a, math_Vector w, Matrix v)
    {
        math_Vector rv1 = new(1, a.ColNumber());
        return SVD_Decompose(a, w, v, rv1);
    }

    /// <summary>As <see cref="SVD_Decompose(Matrix, math_Vector, Matrix)"/>, with rv1 as scratch space.</summary>
    public static int SVD_Decompose(Matrix a, math_Vector w, Matrix v, math_Vector rv1)
    {
        int flag, i, its, j, jj, k, l = 0, nm = 0;
        double ar, aw, aik, aki, f, h, s, x, y, z;
        double anorm = 0.0, g = 0.0, scale = 0.0;
        int m = a.RowNumber();
        int n = a.ColNumber();

        for (i = 1; i <= n; i++)
        {
            l = i + 1;
            rv1[i] = scale * g;
            g = s = scale = 0.0;
            if (i <= m)
            {
                for (k = i; k <= m; k++)
                {
                    aki = a[k, i];
                    if (aki > 0)
                    {
                        scale += aki;
                    }
                    else
                    {
                        scale -= aki;
                    }
                }

                if (scale != 0.0)
                {
                    for (k = i; k <= m; k++)
                    {
                        a[k, i] /= scale;
                        s += a[k, i] * a[k, i];
                    }

                    f = a[i, i];
                    g = -SIGN(Math.Sqrt(s), f);
                    h = f * g - s;
                    a[i, i] = f - g;
                    if (i != n)
                    {
                        for (j = l; j <= n; j++)
                        {
                            for (s = 0.0, k = i; k <= m; k++)
                            {
                                s += a[k, i] * a[k, j];
                            }

                            f = s / h;
                            for (k = i; k <= m; k++)
                            {
                                a[k, j] += f * a[k, i];
                            }
                        }
                    }

                    for (k = i; k <= m; k++)
                    {
                        a[k, i] *= scale;
                    }
                }
            }

            w[i] = scale * g;
            g = s = scale = 0.0;
            if (i <= m && i != n)
            {
                for (k = l; k <= n; k++)
                {
                    aik = a[i, k];
                    if (aik > 0)
                    {
                        scale += aik;
                    }
                    else
                    {
                        scale -= aik;
                    }
                }

                if (scale != 0.0)
                {
                    for (k = l; k <= n; k++)
                    {
                        a[i, k] /= scale;
                        s += a[i, k] * a[i, k];
                    }

                    f = a[i, l];
                    g = -SIGN(Math.Sqrt(s), f);
                    h = f * g - s;
                    a[i, l] = f - g;
                    for (k = l; k <= n; k++)
                    {
                        rv1[k] = a[i, k] / h;
                    }

                    if (i != m)
                    {
                        for (j = l; j <= m; j++)
                        {
                            for (s = 0.0, k = l; k <= n; k++)
                            {
                                s += a[j, k] * a[i, k];
                            }

                            for (k = l; k <= n; k++)
                            {
                                a[j, k] += s * rv1[k];
                            }
                        }
                    }

                    for (k = l; k <= n; k++)
                    {
                        a[i, k] *= scale;
                    }
                }
            }

            aw = w[i];
            if (aw < 0)
            {
                aw = -aw;
            }

            ar = rv1[i];
            if (ar > 0)
            {
                ar = aw + ar;
            }
            else
            {
                ar = aw - ar;
            }

            if (anorm < ar)
            {
                anorm = ar;
            }
        }

        for (i = n; i >= 1; i--)
        {
            if (i < n)
            {
                if (g != 0.0)
                {
                    for (j = l; j <= n; j++)
                    {
                        v[j, i] = (a[i, j] / a[i, l]) / g;
                    }

                    for (j = l; j <= n; j++)
                    {
                        for (s = 0.0, k = l; k <= n; k++)
                        {
                            s += a[i, k] * v[k, j];
                        }

                        for (k = l; k <= n; k++)
                        {
                            v[k, j] += s * v[k, i];
                        }
                    }
                }

                for (j = l; j <= n; j++)
                {
                    v[i, j] = v[j, i] = 0.0;
                }
            }

            v[i, i] = 1.0;
            g = rv1[i];
            l = i;
        }

        for (i = n; i >= 1; i--)
        {
            l = i + 1;
            g = w[i];
            if (i < n)
            {
                for (j = l; j <= n; j++)
                {
                    a[i, j] = 0.0;
                }
            }

            if (g != 0.0)
            {
                g = 1.0 / g;
                if (i != n)
                {
                    for (j = l; j <= n; j++)
                    {
                        for (s = 0.0, k = l; k <= m; k++)
                        {
                            s += a[k, i] * a[k, j];
                        }

                        f = (s / a[i, i]) * g;
                        for (k = i; k <= m; k++)
                        {
                            a[k, j] += f * a[k, i];
                        }
                    }
                }

                for (j = i; j <= m; j++)
                {
                    a[j, i] *= g;
                }
            }
            else
            {
                for (j = i; j <= m; j++)
                {
                    a[j, i] = 0.0;
                }
            }

            ++a[i, i];
        }

        for (k = n; k >= 1; k--)
        {
            for (its = 1; its <= 30; its++)
            {
                flag = 1;
                for (l = k; l >= 1; l--)
                {
                    nm = l - 1;
                    if (Math.Abs(rv1[l]) + anorm == anorm)
                    {
                        flag = 0;
                        break;
                    }

                    if (Math.Abs(w[nm]) + anorm == anorm)
                    {
                        break;
                    }
                }

                if (flag != 0)
                {
                    s = 1.0;
                    for (i = l; i <= k; i++)
                    {
                        f = s * rv1[i];
                        if (Math.Abs(f) + anorm != anorm)
                        {
                            g = w[i];
                            h = PYTHAG(f, g);
                            w[i] = h;
                            h = 1.0 / h;
                            double c0 = g * h;
                            s = (-f * h);
                            for (j = 1; j <= m; j++)
                            {
                                y = a[j, nm];
                                z = a[j, i];
                                a[j, nm] = y * c0 + z * s;
                                a[j, i] = z * c0 - y * s;
                            }
                        }
                    }
                }

                z = w[k];
                if (l == k)
                {
                    if (z < 0.0)
                    {
                        w[k] = -z;
                        for (j = 1; j <= n; j++)
                        {
                            v[j, k] = (-v[j, k]);
                        }
                    }

                    break;
                }

                if (its == 30)
                {
                    return MathStatusNoConvergence;
                }

                x = w[l];
                nm = k - 1;
                y = w[nm];
                g = rv1[nm];
                h = rv1[k];
                f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
                g = PYTHAG(f, 1.0);
                f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

                double c = s = 1.0;
                for (j = l; j <= nm; j++)
                {
                    i = j + 1;
                    g = rv1[i];
                    y = w[i];
                    h = s * g;
                    g = c * g;
                    z = PYTHAG(f, h);
                    rv1[j] = z;
                    c = f / z;
                    s = h / z;
                    f = x * c + g * s;
                    g = g * c - x * s;
                    h = y * s;
                    y = y * c;
                    for (jj = 1; jj <= n; jj++)
                    {
                        x = v[jj, j];
                        z = v[jj, i];
                        v[jj, j] = x * c + z * s;
                        v[jj, i] = z * c - x * s;
                    }

                    z = PYTHAG(f, h);
                    w[j] = z;
                    if (z != 0.0)
                    {
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }

                    f = (c * g) + (s * y);
                    x = (c * y) - (s * g);
                    for (jj = 1; jj <= m; jj++)
                    {
                        y = a[jj, j];
                        z = a[jj, i];
                        a[jj, j] = y * c + z * s;
                        a[jj, i] = z * c - y * s;
                    }
                }

                rv1[l] = 0.0;
                rv1[k] = f;
                w[k] = x;
            }
        }

        return MathStatusOK;
    }

    /// <summary>
    /// Solves a * x = b with u(1..m, 1..n), w(1..n), v(1..n, 1..n) as returned by SVD_Decompose; b(1..m) is the
    /// right-hand side and x(1..n) the solution.
    /// </summary>
    public static void SVD_Solve(Matrix u, math_Vector w, Matrix v, math_Vector b, math_Vector x)
        => SVD_Solve(u, w, v, b, x, new math_Vector(1, u.ColNumber()));

    /// <summary>As <see cref="SVD_Solve(Matrix, math_Vector, Matrix, math_Vector, math_Vector)"/>, with tmp as scratch.</summary>
    public static void SVD_Solve(Matrix u, math_Vector w, Matrix v, math_Vector b, math_Vector x, math_Vector tmp)
    {
        int jj, j, i;
        double s;

        int m = u.RowNumber();
        int n = u.ColNumber();

        for (j = 1; j <= n; j++)
        {
            s = 0.0;
            if (w[j] != 0.0)
            {
                for (i = 1; i <= m; i++)
                {
                    s += u[i, j] * b[i];
                }

                s /= w[j];
            }

            tmp[j] = s;
        }

        for (j = 1; j <= n; j++)
        {
            s = 0.0;
            for (jj = 1; jj <= n; jj++)
            {
                s += v[j, jj] * tmp[jj];
            }

            x[j] = s;
        }
    }

    /// <summary>
    /// The LU decomposition of a SYMMETRIC matrix given by the non-zero components of its upper triangle in a, with
    /// indx the indexes of the diagonal elements. a is replaced by its decomposition.
    /// </summary>
    public static int DACTCL_Decompose(math_Vector a, math_IntegerVector indx, double minPivot = 1.0e-20)
    {
        int i, j, neq = indx.Length();
        int jr, jd, jh, @is, ie, k, ir, id, ih, mh;
        int idot, idot1, idot2;
        double aa, d, dot;
        bool diag;

        jr = 0;
        for (j = 1; j <= neq; j++)
        {
            diag = false;
            jd = indx[j];
            jh = jd - jr;
            @is = j - jh + 2;
            if (jh - 2 == 0)
            {
                diag = true;
            }

            if (jh - 2 > 0)
            {
                ie = j - 1;
                k = jr + 2;
                id = indx[@is - 1];
                // Reduction des coefficients non diagonaux:
                // =========================================
                for (i = @is; i <= ie; i++)
                {
                    ir = id;
                    id = indx[i];
                    ih = id - ir - 1;
                    mh = i - @is + 1;
                    if (ih > mh)
                    {
                        ih = mh;
                    }

                    if (ih > 0.0)
                    {
                        dot = 0.0;
                        idot1 = k - ih - 1;
                        idot2 = id - ih - 1;
                        for (idot = 1; idot <= ih; idot++)
                        {
                            dot = dot + a[idot1 + idot] * a[idot2 + idot];
                        }

                        a[k] = a[k] - dot;
                    }

                    k++;
                }

                diag = true;
            }

            if (diag)
            {
                // Reduction des coefficients diagonaux:
                // =====================================
                ir = jr + 1;
                ie = jd - 1;
                k = j - jd;
                for (i = ir; i <= ie; i++)
                {
                    id = indx[k + i];
                    aa = a[id];
                    if (aa < 0)
                    {
                        aa = -aa;
                    }

                    if (aa <= minPivot)
                    {
                        return MathStatusSingularMatrix;
                    }

                    d = a[i];
                    a[i] = d / a[id];
                    a[jd] = a[jd] - d * a[i];
                }
            }

            jr = jd;
        }

        return MathStatusOK;
    }

    /// <summary>
    /// Solves a * x = b with a coming from <see cref="DACTCL_Decompose"/> and the same indx; b is replaced by the solution.
    /// </summary>
    public static int DACTCL_Solve(math_Vector a, math_Vector b, math_IntegerVector indx, double minPivot = 1.0e-20)
    {
        int i, j, neq = indx.Length();
        int jr, jd, jh, @is, k, id;
        int jh1, idot, idot1, idot2;
        double aa, d, dot;

        jr = 0;
        for (j = 1; j <= neq; j++)
        {
            jd = indx[j];
            jh = jd - jr;
            @is = j - jh + 2;

            // Reduction du second membre:
            // ===========================
            dot = 0.0;
            idot1 = jr;
            idot2 = @is - 2;
            jh1 = jh - 1;
            for (idot = 1; idot <= jh1; idot++)
            {
                dot = dot + a[idot1 + idot] * b[idot2 + idot];
            }

            b[j] = b[j] - dot;

            jr = jd;
        }

        // Division par les pivots diagonaux:
        // ==================================
        for (i = 1; i <= neq; i++)
        {
            id = indx[i];
            aa = a[id];
            if (aa < 0)
            {
                aa = -aa;
            }

            if (aa <= minPivot)
            {
                return MathStatusSingularMatrix;
            }

            b[i] = b[i] / a[id];
        }

        // Substitution arriere:
        // =====================
        jd = indx[neq];
        for (j = neq - 1; j > 0; j--)
        {
            d = b[j + 1];
            jr = indx[j];
            if (jd - jr > 1)
            {
                @is = j - jd + jr + 2;
                k = jr - @is + 1;
                for (i = @is; i <= j; i++)
                {
                    b[i] = b[i] - a[i + k] * d;
                }
            }

            jd = jr;
        }

        return MathStatusOK;
    }
}
