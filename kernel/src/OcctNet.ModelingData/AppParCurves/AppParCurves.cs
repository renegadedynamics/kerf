// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Parallel approximation of n curves: the basis functions shared by the AppParCurves algorithms, which
/// approximate a MultiLine described by a line tool (<see cref="IAppParCurves_LineTool{MultiLine}"/>) with a
/// MultiCurve.
/// </summary>
public static class AppParCurves
{
    /// <summary>
    /// Fills <paramref name="A"/>(i, j) with the Bernstein polynomial j of degree NbPoles - 1 at U(i).
    /// </summary>
    public static void BernsteinMatrix(int NbPoles, math_Vector U, math_Matrix A)
    {
        int i, j, id;
        double u0, u1, y0, y1, xs;
        int first = U.Lower(), last = U.Upper();
        math_Vector B = new(1, NbPoles - 1);

        for (i = first; i <= last; i++)
        {
            B[1] = 1;
            u0 = U[i];
            u1 = 1.0 - u0;

            for (id = 2; id <= NbPoles - 1; id++)
            {
                y0 = B[1];
                y1 = u0 * y0;
                B[1] = y0 - y1;
                for (j = 2; j <= id - 1; j++)
                {
                    xs = y1;
                    y0 = B[j];
                    y1 = u0 * y0;
                    B[j] = y0 - y1 + xs;
                }
                B[id] = y1;
            }
            A[i, 1] = u1 * B[1];
            A[i, NbPoles] = u0 * B[NbPoles - 1];
            for (j = 2; j <= NbPoles - 1; j++)
            {
                A[i, j] = u1 * B[j] + u0 * B[j - 1];
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="A"/>(i, j) and <paramref name="DA"/>(i, j) with the Bernstein polynomial j of degree
    /// NbPoles - 1 and its derivative at U(i).
    /// </summary>
    public static void Bernstein(int NbPoles, math_Vector U, math_Matrix A, math_Matrix DA)
    {
        int i, j, id, Ndeg = NbPoles - 1;
        double u0, u1, y0, y1, xs, bj, bj1;
        int first = U.Lower(), last = U.Upper();
        math_Vector B = new(1, NbPoles - 1);

        for (i = first; i <= last; i++)
        {
            B[1] = 1;
            u0 = U[i];
            u1 = 1.0 - u0;

            for (id = 2; id <= NbPoles - 1; id++)
            {
                y0 = B[1];
                y1 = u0 * y0;
                B[1] = y0 - y1;
                for (j = 2; j <= id - 1; j++)
                {
                    xs = y1;
                    y0 = B[j];
                    y1 = u0 * y0;
                    B[j] = y0 - y1 + xs;
                }
                B[id] = y1;
            }
            DA[i, 1] = -Ndeg * B[1];
            DA[i, NbPoles] = Ndeg * B[NbPoles - 1];
            A[i, 1] = u1 * B[1];
            A[i, NbPoles] = u0 * B[NbPoles - 1];
            for (j = 2; j <= NbPoles - 1; j++)
            {
                bj = B[j];
                bj1 = B[j - 1];
                DA[i, j] = Ndeg * (bj1 - bj);
                A[i, j] = u1 * bj + u0 * bj1;
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="DDA"/> with the second derivatives at <paramref name="U"/> of the Bernstein
    /// polynomials of degree DDA.Length() - 1.
    /// </summary>
    public static void SecondDerivativeBernstein(double U, math_Vector DDA)
    {
        double Y0, Y1, Xs;
        int NbPoles = DDA.Length();
        int id, j, N4, deg = NbPoles - 1;
        N4 = deg * (deg - 1);
        // OCCT: allocates B(1, deg - 1) for every degree; for degree 1 that is an empty range (1, 0), whose B(1)
        // write is out of bounds (unchecked in OCCT's release build). The port allocates at least one element.
        math_Vector B = new(1, Math.Max(deg - 1, 1));
        B[1] = 1.0;

        // Cas particulier si degre = 1:
        if (deg == 1)
        {
            DDA[1] = 0.0;
            DDA[2] = 0.0;
        }
        else if (deg == 2)
        {
            DDA[1] = 2.0;
            DDA[2] = -4.0;
            DDA[3] = 2.0;
        }
        else
        {
            for (id = 2; id <= deg - 1; id++)
            {
                Y0 = B[1];
                Y1 = U * Y0;
                B[1] = Y0 - Y1;
                for (j = 2; j <= id - 1; j++)
                {
                    Xs = Y1;
                    Y0 = B[j];
                    Y1 = U * Y0;
                    B[j] = Y0 - Y1 + Xs;
                }
                B[id] = Y1;
            }

            DDA[1] = N4 * B[1];
            DDA[2] = N4 * (-2 * B[1] + B[2]);
            DDA[deg] = N4 * (B[deg - 2] - 2 * B[deg - 1]);
            DDA[deg + 1] = N4 * B[deg - 1];

            for (j = 2; j <= deg - 2; j++)
            {
                DDA[j + 1] = N4 * (B[j - 1] - 2 * B[j] + B[j + 1]);
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="A"/>(i, j) and <paramref name="DA"/>(i, j) with the B-spline basis function j of degree
    /// <paramref name="deg"/> on the flat knots <paramref name="flatknots"/>, and its derivative, at Parameters(i).
    /// <paramref name="index"/>(i) receives the index before the first non-zero function: they are non-zero from
    /// index(i) + 1 to index(i) + deg + 1.
    /// </summary>
    public static void SplineFunction(int nbpoles, int deg, math_Vector Parameters, math_Vector flatknots, math_Matrix A,
                                      math_Matrix DA, math_IntegerVector index)
    {
        double U, NewU;
        int i, j, deg1 = deg + 1;
        int oldkindex, kindex, theindex;
        math_Vector locpoles = new(1, deg1);
        math_Vector locdpoles = new(1, deg1);
        int firstp = Parameters.Lower(), lastp = Parameters.Upper();

        NCollection_Array1<double> Aflatknots = new(flatknots.Lower(), flatknots.Upper());
        for (i = flatknots.Lower(); i <= flatknots.Upper(); i++)
        {
            Aflatknots[i] = flatknots[i];
        }

        oldkindex = 1;

        int pp, qq;
        double Saved, Inverse, LocalInverse, locqq, locdqq, val;

        for (i = firstp; i <= lastp; i++)
        {
            U = Parameters[i];
            NewU = U;
            kindex = oldkindex;
            BSplCLib.LocateParameter(deg, Aflatknots, U, false, deg1, nbpoles + 1, out kindex, out NewU);

            oldkindex = kindex;

            // On stocke les index:
            index[i] = kindex - deg - 1;

            locpoles[1] = 1.0;

            for (qq = 2; qq <= deg; qq++)
            {
                locpoles[qq] = 0.0;
                for (pp = 1; pp <= qq - 1; pp++)
                {
                    Inverse = 1.0 / (flatknots[kindex + pp] - flatknots[kindex - qq + pp + 1]);
                    Saved = (U - flatknots[kindex - qq + pp + 1]) * Inverse * locpoles[pp];
                    locpoles[pp] *= (flatknots[kindex + pp] - U) * Inverse;
                    locpoles[pp] += locpoles[qq];
                    locpoles[qq] = Saved;
                }
            }

            qq = deg + 1;
            for (pp = 1; pp <= deg; pp++)
            {
                locdpoles[pp] = locpoles[pp];
            }

            locqq = 0.0;
            locdqq = 0.0;
            for (pp = 1; pp <= deg; pp++)
            {
                Inverse = 1.0 / (flatknots[kindex + pp] - flatknots[kindex - qq + pp + 1]);
                Saved = (U - flatknots[kindex - qq + pp + 1]) * Inverse * locpoles[pp];
                locpoles[pp] *= (flatknots[kindex + pp] - U) * Inverse;
                locpoles[pp] += locqq;
                locqq = Saved;
                LocalInverse = (double)(deg) * Inverse;
                Saved = LocalInverse * locdpoles[pp];
                locdpoles[pp] *= -LocalInverse;
                locdpoles[pp] += locdqq;
                locdqq = Saved;
            }

            locpoles[qq] = locqq;
            locdpoles[qq] = locdqq;

            for (j = 1; j <= deg1; j++)
            {
                val = locpoles[j];
                theindex = j + oldkindex - deg1;
                A[i, theindex] = val;
                DA[i, theindex] = locdpoles[j];
            }

            for (j = 1; j < oldkindex - deg; j++)
            {
                A[i, j] = DA[i, j] = 0.0;
            }
            for (j = oldkindex + 1; j <= nbpoles; j++)
            {
                A[i, j] = DA[i, j] = 0.0;
            }
        }
    }
}
