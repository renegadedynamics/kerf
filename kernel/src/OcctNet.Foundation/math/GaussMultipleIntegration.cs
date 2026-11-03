// Port of: src/FoundationClasses/TKMath/math/math_GaussMultipleIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussMultipleIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussMultipleIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

/*
Ce programme permet le calcul de l'integrale nieme d'une fonction. La
resolution peut etre vue comme l'integrale de l'integrale de ....(n fois), ce
qui necessite l'emploi de fonction recursive.

Une integrale simple de Gauss est la somme de la fonction donnee sur les points
de Gauss affectee du poids de Gauss correspondant.

I = somme(F(GaussPt) * GaussW)   sur i = 1 jusqu'a l'ordre d'integration.

Une integrale multiple est une somme recursive sur toutes les variables.

Tout le calcul est effectue dans la methode recursive_iteration appelee n fois.
 (n = nombre de variables)

Cette methode fait une sommation sur toutes les variables, sur tous les ordres
d'integration correspondants de la valeur de la fonction aux points et poids
de Gauss.
*/

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Gauss-Legendre integration of a function of several variables between the bounds Lower[a..b] and
/// Upper[a..b]. Each order is limited to <see cref="NumericTables.GaussPointsMax"/> (61).
/// </summary>
public class GaussMultipleIntegration
{
    private double _val;
    private bool _done;

    /// <summary>Integrates F between Lower and Upper with Order(i) Gauss points for variable i.</summary>
    public GaussMultipleIntegration(MultipleVarFunction f, math_Vector lower, math_Vector upper, math_IntegerVector order)
    {
        int maxOrder = NumericTables.GaussPointsMax();

        int i, max = 0;
        int nVar = f.VariableCount;
        math_IntegerVector ord = new(1, nVar);
        math_Vector lowsav = new(1, nVar);
        math_Vector uppsav = new(1, nVar);
        lowsav.Initialized(lower);
        uppsav.Initialized(upper);
        //  Ord = Order;
        _done = false;
        for (i = 1; i <= nVar; i++)
        {
            if (order[i] > maxOrder)
            {
                ord[i] = maxOrder;
            }
            else
            {
                ord[i] = order[i];
            }

            if (ord[i] >= max)
            {
                max = ord[i];
            }
        }

        // Calcul de l'integrale aux points de Gauss.
        //  Il s agit d une somme multiple sur le domaine [Lower, Upper].

        IntegrationFunction func = new(f, max, nVar, ord, lowsav, uppsav);
        if (func.IsDone)
        {
            _val = func.Value();
            _done = true;
        }
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
        o.Write("math_GaussMultipleIntegration ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Integration value = " + _val.ToString(CultureInfo.InvariantCulture) + "\n");
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

    // Cette classe sert a conserver dans les champs les valeurs de tests de
    // la fonction recursive. (The file-local class IntegrationFunction of math_GaussMultipleIntegration.cxx.)
    private sealed class IntegrationFunction
    {
        private MultipleVarFunction _fsav;
        private math_IntegerVector _ordsav;
        private int _nVarsav;
        private math_Vector _xr;
        private math_Vector _xm;
        private Matrix _gaussPoint;
        private Matrix _gaussWeight;
        private double _val;
        private bool _done;

        public IntegrationFunction(MultipleVarFunction f, int maxsav, int nVar, math_IntegerVector ord,
                                   math_Vector lowsav, math_Vector uppsav)
        {
            _ordsav = new math_IntegerVector(1, nVar);
            _xr = new math_Vector(1, nVar);
            _xm = new math_Vector(1, nVar);
            _gaussPoint = new Matrix(1, nVar, 1, maxsav);
            _gaussWeight = new Matrix(1, nVar, 1, maxsav);

            int i, k;
            math_IntegerVector inc = new(1, nVar);
            inc.Init(0);
            _fsav = f;
            _nVarsav = nVar;
            _ordsav.Initialized(ord);
            _done = false;

            // Recuperation des points et poids de Gauss dans le fichier GaussPoints
            for (i = 1; i <= _nVarsav; i++)
            {
                _xm[i] = 0.5 * (lowsav[i] + uppsav[i]);
                _xr[i] = 0.5 * (uppsav[i] - lowsav[i]);
                math_Vector gp = new(1, _ordsav[i]), gw = new(1, _ordsav[i]);
                NumericTables.GaussPoints(_ordsav[i], gp);
                NumericTables.GaussWeights(_ordsav[i], gw);
                for (k = 1; k <= _ordsav[i]; k++)
                {
                    _gaussPoint[i, k] = gp[k];  // kieme point et poids de
                    _gaussWeight[i, k] = gw[k]; // Gauss de la variable i.
                }
            }

            _val = 0.0;
            int iterdeb = 1;
            bool recur = recursive_iteration(ref iterdeb, inc);
            if (recur)
            {
                // On ramene l'integration a la bonne echelle.
                for (i = 1; i <= _nVarsav; i++)
                {
                    _val *= _xr[i];
                }

                _done = true;
            }
        }

        public double Value()
        {
            return _val;
        }

        public bool IsDone
        {
            get
            {
                return _done;
            }
        }

        private bool recursive_iteration(ref int n, math_IntegerVector inc)
        {
            // Termination criterium :
            // Calcul de la valeur de la fonction aux points de Gauss fixes:

            int local;
            if (n == (_nVarsav + 1))
            {
                math_Vector dx = new(1, _nVarsav);
                int j;
                for (j = 1; j <= _nVarsav; j++)
                {
                    dx[j] = _xr[j] * _gaussPoint[j, inc[j]];
                }

                bool Ok = _fsav.Value(_xm + dx, out double f1);
                if (!Ok)
                {
                    return false;
                }

                double interm = 1;
                for (j = 1; j <= _nVarsav; j++)
                {
                    interm *= _gaussWeight[j, inc[j]];
                }

                _val += interm * f1;
                return true;
            }

            // Calcul recursif, pour chaque variable n de la valeur de la fonction aux
            // Ordsav(n) points de Gauss.

            bool ok = false;
            // OCCT keeps only the last iteration's status, so an earlier failure of the function is ignored.
            for (inc[n] = 1; inc[n] <= _ordsav[n]; inc[n]++)
            {
                local = n + 1;
                ok = recursive_iteration(ref local, inc);
            }

            return ok;
        }
    }
}
