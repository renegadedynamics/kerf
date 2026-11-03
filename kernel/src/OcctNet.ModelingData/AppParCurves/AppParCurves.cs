// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Parallel approximation of n curves: the basis functions shared by the AppParCurves algorithms, which
/// approximate a MultiLine described by a line tool (<see cref="ILineTool{MultiLine}"/>) with a
/// MultiCurve.
/// </summary>
public static class AppParCurves
{
    /// <summary>
    /// Fills <paramref name="a"/>(i, j) with the Bernstein polynomial j of degree NbPoles - 1 at U(i).
    /// </summary>
    public static void BernsteinMatrix(int nbPoles, math_Vector u, Matrix a)
    {
        int i, j, id;
        double u0, u1, y0, y1, xs;
        int first = u.Lower(), last = u.Upper();
        math_Vector b = new(1, nbPoles - 1);

        for (i = first; i <= last; i++)
        {
            b[1] = 1;
            u0 = u[i];
            u1 = 1.0 - u0;

            for (id = 2; id <= nbPoles - 1; id++)
            {
                y0 = b[1];
                y1 = u0 * y0;
                b[1] = y0 - y1;
                for (j = 2; j <= id - 1; j++)
                {
                    xs = y1;
                    y0 = b[j];
                    y1 = u0 * y0;
                    b[j] = y0 - y1 + xs;
                }
                b[id] = y1;
            }
            a[i, 1] = u1 * b[1];
            a[i, nbPoles] = u0 * b[nbPoles - 1];
            for (j = 2; j <= nbPoles - 1; j++)
            {
                a[i, j] = u1 * b[j] + u0 * b[j - 1];
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="a"/>(i, j) and <paramref name="da"/>(i, j) with the Bernstein polynomial j of degree
    /// NbPoles - 1 and its derivative at U(i).
    /// </summary>
    public static void Bernstein(int nbPoles, math_Vector u, Matrix a, Matrix da)
    {
        int i, j, id, ndeg = nbPoles - 1;
        double u0, u1, y0, y1, xs, bj, bj1;
        int first = u.Lower(), last = u.Upper();
        math_Vector b = new(1, nbPoles - 1);

        for (i = first; i <= last; i++)
        {
            b[1] = 1;
            u0 = u[i];
            u1 = 1.0 - u0;

            for (id = 2; id <= nbPoles - 1; id++)
            {
                y0 = b[1];
                y1 = u0 * y0;
                b[1] = y0 - y1;
                for (j = 2; j <= id - 1; j++)
                {
                    xs = y1;
                    y0 = b[j];
                    y1 = u0 * y0;
                    b[j] = y0 - y1 + xs;
                }
                b[id] = y1;
            }
            da[i, 1] = -ndeg * b[1];
            da[i, nbPoles] = ndeg * b[nbPoles - 1];
            a[i, 1] = u1 * b[1];
            a[i, nbPoles] = u0 * b[nbPoles - 1];
            for (j = 2; j <= nbPoles - 1; j++)
            {
                bj = b[j];
                bj1 = b[j - 1];
                da[i, j] = ndeg * (bj1 - bj);
                a[i, j] = u1 * bj + u0 * bj1;
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="dda"/> with the second derivatives at <paramref name="u"/> of the Bernstein
    /// polynomials of degree DDA.Length() - 1.
    /// </summary>
    public static void SecondDerivativeBernstein(double u, math_Vector dda)
    {
        double y0, y1, xs;
        int nbPoles = dda.Length();
        int id, j, n4, deg = nbPoles - 1;
        n4 = deg * (deg - 1);
        // OCCT: allocates B(1, deg - 1) for every degree; for degree 1 that is an empty range (1, 0), whose B(1)
        // write is out of bounds (unchecked in OCCT's release build). The port allocates at least one element.
        math_Vector b = new(1, Math.Max(deg - 1, 1));
        b[1] = 1.0;

        // Cas particulier si degre = 1:
        if (deg == 1)
        {
            dda[1] = 0.0;
            dda[2] = 0.0;
        }
        else if (deg == 2)
        {
            dda[1] = 2.0;
            dda[2] = -4.0;
            dda[3] = 2.0;
        }
        else
        {
            for (id = 2; id <= deg - 1; id++)
            {
                y0 = b[1];
                y1 = u * y0;
                b[1] = y0 - y1;
                for (j = 2; j <= id - 1; j++)
                {
                    xs = y1;
                    y0 = b[j];
                    y1 = u * y0;
                    b[j] = y0 - y1 + xs;
                }
                b[id] = y1;
            }

            dda[1] = n4 * b[1];
            dda[2] = n4 * (-2 * b[1] + b[2]);
            dda[deg] = n4 * (b[deg - 2] - 2 * b[deg - 1]);
            dda[deg + 1] = n4 * b[deg - 1];

            for (j = 2; j <= deg - 2; j++)
            {
                dda[j + 1] = n4 * (b[j - 1] - 2 * b[j] + b[j + 1]);
            }
        }
    }

    /// <summary>
    /// Fills <paramref name="a"/>(i, j) and <paramref name="da"/>(i, j) with the B-spline basis function j of degree
    /// <paramref name="deg"/> on the flat knots <paramref name="flatknots"/>, and its derivative, at Parameters(i).
    /// <paramref name="index"/>(i) receives the index before the first non-zero function: they are non-zero from
    /// index(i) + 1 to index(i) + deg + 1.
    /// </summary>
    public static void SplineFunction(int nbpoles, int deg, math_Vector parameters, math_Vector flatknots, Matrix a,
                                      Matrix da, math_IntegerVector index)
    {
        double u, newU;
        int i, j, deg1 = deg + 1;
        int oldkindex, kindex, theindex;
        math_Vector locpoles = new(1, deg1);
        math_Vector locdpoles = new(1, deg1);
        int firstp = parameters.Lower(), lastp = parameters.Upper();

        NCollection_Array1<double> aflatknots = new(flatknots.Lower(), flatknots.Upper());
        for (i = flatknots.Lower(); i <= flatknots.Upper(); i++)
        {
            aflatknots[i] = flatknots[i];
        }

        oldkindex = 1;

        int pp, qq;
        double saved, inverse, localInverse, locqq, locdqq, val;

        for (i = firstp; i <= lastp; i++)
        {
            u = parameters[i];
            newU = u;
            kindex = oldkindex;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(deg, aflatknots, u, false, deg1, nbpoles + 1, out kindex, out newU);

            oldkindex = kindex;

            // On stocke les index:
            index[i] = kindex - deg - 1;

            locpoles[1] = 1.0;

            for (qq = 2; qq <= deg; qq++)
            {
                locpoles[qq] = 0.0;
                for (pp = 1; pp <= qq - 1; pp++)
                {
                    inverse = 1.0 / (flatknots[kindex + pp] - flatknots[kindex - qq + pp + 1]);
                    saved = (u - flatknots[kindex - qq + pp + 1]) * inverse * locpoles[pp];
                    locpoles[pp] *= (flatknots[kindex + pp] - u) * inverse;
                    locpoles[pp] += locpoles[qq];
                    locpoles[qq] = saved;
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
                inverse = 1.0 / (flatknots[kindex + pp] - flatknots[kindex - qq + pp + 1]);
                saved = (u - flatknots[kindex - qq + pp + 1]) * inverse * locpoles[pp];
                locpoles[pp] *= (flatknots[kindex + pp] - u) * inverse;
                locpoles[pp] += locqq;
                locqq = saved;
                localInverse = (double)(deg) * inverse;
                saved = localInverse * locdpoles[pp];
                locdpoles[pp] *= -localInverse;
                locdpoles[pp] += locdqq;
                locdqq = saved;
            }

            locpoles[qq] = locqq;
            locdpoles[qq] = locdqq;

            for (j = 1; j <= deg1; j++)
            {
                val = locpoles[j];
                theindex = j + oldkindex - deg1;
                a[i, theindex] = val;
                da[i, theindex] = locdpoles[j];
            }

            for (j = 1; j < oldkindex - deg; j++)
            {
                a[i, j] = da[i, j] = 0.0;
            }
            for (j = oldkindex + 1; j <= nbpoles; j++)
            {
                a[i, j] = da[i, j] = 0.0;
            }
        }
    }
}
