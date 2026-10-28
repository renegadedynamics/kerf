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

using System.Globalization;

namespace Occt;

/// <summary>
/// The Gauss-Legendre integration of a function of several variables between the bounds Lower[a..b] and
/// Upper[a..b]. Each order is limited to <see cref="math.GaussPointsMax"/> (61).
/// </summary>
public class math_GaussMultipleIntegration
{
    private double Val;
    private bool Done;

    /// <summary>Integrates F between Lower and Upper with Order(i) Gauss points for variable i.</summary>
    public math_GaussMultipleIntegration(math_MultipleVarFunction F, math_Vector Lower, math_Vector Upper, math_IntegerVector Order)
    {
        int MaxOrder = math.GaussPointsMax();

        int i, max = 0;
        int NVar = F.NbVariables();
        math_IntegerVector Ord = new(1, NVar);
        math_Vector Lowsav = new(1, NVar);
        math_Vector Uppsav = new(1, NVar);
        Lowsav.Initialized(Lower);
        Uppsav.Initialized(Upper);
        //  Ord = Order;
        Done = false;
        for (i = 1; i <= NVar; i++)
        {
            if (Order[i] > MaxOrder)
            {
                Ord[i] = MaxOrder;
            }
            else
            {
                Ord[i] = Order[i];
            }

            if (Ord[i] >= max)
            {
                max = Ord[i];
            }
        }

        // Calcul de l'integrale aux points de Gauss.
        //  Il s agit d une somme multiple sur le domaine [Lower, Upper].

        IntegrationFunction Func = new(F, max, NVar, Ord, Lowsav, Uppsav);
        if (Func.IsDone())
        {
            Val = Func.Value();
            Done = true;
        }
    }

    /// <summary>True if the integral was computed.</summary>
    public bool IsDone() => Done;

    /// <summary>The value of the integral. Throws <see cref="StdFail_NotDone"/> if it was not computed.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Val;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussMultipleIntegration ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Integration value = " + Val.ToString(CultureInfo.InvariantCulture) + "\n");
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

    // Cette classe sert a conserver dans les champs les valeurs de tests de
    // la fonction recursive. (The file-local class IntegrationFunction of math_GaussMultipleIntegration.cxx.)
    private sealed class IntegrationFunction
    {
        private math_MultipleVarFunction Fsav;
        private math_IntegerVector Ordsav;
        private int NVarsav;
        private math_Vector xr;
        private math_Vector xm;
        private math_Matrix GaussPoint;
        private math_Matrix GaussWeight;
        private double Val;
        private bool Done;

        public IntegrationFunction(math_MultipleVarFunction F, int maxsav, int NVar, math_IntegerVector Ord,
                                   math_Vector Lowsav, math_Vector Uppsav)
        {
            Ordsav = new math_IntegerVector(1, NVar);
            xr = new math_Vector(1, NVar);
            xm = new math_Vector(1, NVar);
            GaussPoint = new math_Matrix(1, NVar, 1, maxsav);
            GaussWeight = new math_Matrix(1, NVar, 1, maxsav);

            int i, k;
            math_IntegerVector inc = new(1, NVar);
            inc.Init(0);
            Fsav = F;
            NVarsav = NVar;
            Ordsav.Initialized(Ord);
            Done = false;

            // Recuperation des points et poids de Gauss dans le fichier GaussPoints
            for (i = 1; i <= NVarsav; i++)
            {
                xm[i] = 0.5 * (Lowsav[i] + Uppsav[i]);
                xr[i] = 0.5 * (Uppsav[i] - Lowsav[i]);
                math_Vector GP = new(1, Ordsav[i]), GW = new(1, Ordsav[i]);
                math.GaussPoints(Ordsav[i], GP);
                math.GaussWeights(Ordsav[i], GW);
                for (k = 1; k <= Ordsav[i]; k++)
                {
                    GaussPoint[i, k] = GP[k];  // kieme point et poids de
                    GaussWeight[i, k] = GW[k]; // Gauss de la variable i.
                }
            }

            Val = 0.0;
            int Iterdeb = 1;
            bool recur = recursive_iteration(ref Iterdeb, inc);
            if (recur)
            {
                // On ramene l'integration a la bonne echelle.
                for (i = 1; i <= NVarsav; i++)
                {
                    Val *= xr[i];
                }

                Done = true;
            }
        }

        public double Value()
        {
            return Val;
        }

        public bool IsDone()
        {
            return Done;
        }

        private bool recursive_iteration(ref int n, math_IntegerVector inc)
        {
            // Termination criterium :
            // Calcul de la valeur de la fonction aux points de Gauss fixes:

            int local;
            if (n == (NVarsav + 1))
            {
                math_Vector dx = new(1, NVarsav);
                int j;
                for (j = 1; j <= NVarsav; j++)
                {
                    dx[j] = xr[j] * GaussPoint[j, inc[j]];
                }

                bool Ok = Fsav.Value(xm + dx, out double F1);
                if (!Ok)
                {
                    return false;
                }

                double Interm = 1;
                for (j = 1; j <= NVarsav; j++)
                {
                    Interm *= GaussWeight[j, inc[j]];
                }

                Val += Interm * F1;
                return true;
            }

            // Calcul recursif, pour chaque variable n de la valeur de la fonction aux
            // Ordsav(n) points de Gauss.

            bool OK = false;
            // OCCT keeps only the last iteration's status, so an earlier failure of the function is ignored.
            for (inc[n] = 1; inc[n] <= Ordsav[n]; inc[n]++)
            {
                local = n + 1;
                OK = recursive_iteration(ref local, inc);
            }

            return OK;
        }
    }
}
