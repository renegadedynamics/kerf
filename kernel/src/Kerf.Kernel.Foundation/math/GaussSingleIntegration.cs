// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

/*
Par Gauss le calcul d'une integrale simple se transforme en sommation des
valeurs de la fonction donnee aux <Order> points de Gauss affectee des poids
de Gauss.

Les points et poids de Gauss sont stockes dans GaussPoints.cxx.
Les points sont compris entre les valeurs -1 et +1, ce qui necessite un
changement de variable pour les faire varier dans l'intervalle [Lower, Upper].

On veut calculer Integrale( f(u)* du) entre a et b.

Etapes du calcul:

1- calcul de la fonction au ieme point de Gauss (apres changement de variable).

2- multiplication de cette valeur par le ieme poids de Gauss.

3- sommation de toutes ces valeurs.

4- retour a l'intervalle [Lower, Upper] de notre integrale.
*/

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Gauss-Legendre integration of a function of a single variable between the bounds Lower and Upper. The order
/// is limited to <see cref="NumericTables.GaussPointsMax"/> (61).
/// </summary>
internal class GaussSingleIntegration
{
    private double _val;
    private bool _done;

    public GaussSingleIntegration()
    {
        _done = false;
    }

    /// <summary>Integrates F between Lower and Upper with Order Gauss points.</summary>
    public GaussSingleIntegration(Function f, double lower, double upper, int order)
    {
        int theOrder = Min(NumericTables.GaussPointsMax(), order);
        Perform(f, lower, upper, theOrder);
    }

    /// <summary>
    /// Integrates F between Lower and Upper with Order Gauss points, halving the subintervals until two results
    /// differ by at most Tol (13 iterations at most).
    /// </summary>
    public GaussSingleIntegration(Function f, double lower, double upper, int order, double tol)
    {
        int theOrder = Min(NumericTables.GaussPointsMax(), order);

        const int iterMax = 13; // Max number of iteration
        int nIter = 1;          // current number of iteration
        int nbInterval = 1;     // current number of subintervals
        double dU, oldLen, len;

        Perform(f, lower, upper, theOrder);
        len = _val;
        do
        {
            oldLen = len;
            len = 0.0;
            nbInterval *= 2;
            dU = (upper - lower) / nbInterval;
            for (int i = 1; i <= nbInterval; i++)
            {
                Perform(f, lower + (i - 1) * dU, lower + i * dU, theOrder);
                if (!_done)
                {
                    return;
                }

                len += _val;
            }

            nIter++;
        } while (Math.Abs(oldLen - len) > tol && nIter <= iterMax);

        _val = len;
    }

    /// <summary>True if the integral was computed.</summary>
    public bool IsDone => _done;

    /// <summary>The value of the integral. Throws <see cref="NotDoneException"/> if it was not computed.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _val;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussSingleIntegration ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write("Integration Value = " + _val.ToString(CultureInfo.InvariantCulture) + "\n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    private void Perform(Function f, double lower, double upper, int order)
    {
        double xr, xm, dx;
        int j;
        double f1, f2;
        bool ok1;
        math_Vector gaussP = new(1, order);
        math_Vector gaussW = new(1, order);
        _done = false;

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        NumericTables.GaussPoints(order, gaussP);
        NumericTables.GaussWeights(order, gaussW);

        // Calcul de l'integrale aux points de Gauss :

        // Changement de variable pour la mise a l'echelle [Lower, Upper] :
        xm = 0.5 * (upper + lower);
        xr = 0.5 * (upper - lower);
        _val = 0.0;

        int ind = order / 2, ind1 = (order + 1) / 2;
        if (ind1 > ind)
        { // odder case
            ok1 = f.Value(xm, out _val);
            if (!ok1)
            {
                return;
            }

            _val *= gaussW[ind1];
        }

        // Sommation sur tous les points de Gauss: avec utilisation de la symetrie.
        for (j = 1; j <= ind; j++)
        {
            dx = xr * gaussP[j];
            ok1 = f.Value(xm - dx, out f1);
            if (!ok1)
            {
                return;
            }

            ok1 = f.Value(xm + dx, out f2);
            if (!ok1)
            {
                return;
            }

            // Multiplication par les poids de Gauss.
            double ft = f1 + f2;
            _val += gaussW[j] * ft;
        }

        // Mise a l'echelle de l'intervalle [Lower, Upper]
        _val *= xr;
        _done = true;
    }
}
