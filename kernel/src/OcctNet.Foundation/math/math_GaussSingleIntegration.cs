// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSingleIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

using System.Globalization;

namespace Occt;

/// <summary>
/// The Gauss-Legendre integration of a function of a single variable between the bounds Lower and Upper. The order
/// is limited to <see cref="math.GaussPointsMax"/> (61).
/// </summary>
public class math_GaussSingleIntegration
{
    private double Val;
    private bool Done;

    public math_GaussSingleIntegration()
    {
        Done = false;
    }

    /// <summary>Integrates F between Lower and Upper with Order Gauss points.</summary>
    public math_GaussSingleIntegration(math_Function F, double Lower, double Upper, int Order)
    {
        int theOrder = Min(math.GaussPointsMax(), Order);
        Perform(F, Lower, Upper, theOrder);
    }

    /// <summary>
    /// Integrates F between Lower and Upper with Order Gauss points, halving the subintervals until two results
    /// differ by at most Tol (13 iterations at most).
    /// </summary>
    public math_GaussSingleIntegration(math_Function F, double Lower, double Upper, int Order, double Tol)
    {
        int theOrder = Min(math.GaussPointsMax(), Order);

        const int IterMax = 13; // Max number of iteration
        int NIter = 1;          // current number of iteration
        int NbInterval = 1;     // current number of subintervals
        double dU, OldLen, Len;

        Perform(F, Lower, Upper, theOrder);
        Len = Val;
        do
        {
            OldLen = Len;
            Len = 0.0;
            NbInterval *= 2;
            dU = (Upper - Lower) / NbInterval;
            for (int i = 1; i <= NbInterval; i++)
            {
                Perform(F, Lower + (i - 1) * dU, Lower + i * dU, theOrder);
                if (!Done)
                {
                    return;
                }

                Len += Val;
            }

            NIter++;
        } while (Math.Abs(OldLen - Len) > Tol && NIter <= IterMax);

        Val = Len;
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
        o.Write("math_GaussSingleIntegration ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write("Integration Value = " + Val.ToString(CultureInfo.InvariantCulture) + "\n");
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

    private void Perform(math_Function F, double Lower, double Upper, int Order)
    {
        double xr, xm, dx;
        int j;
        double F1, F2;
        bool Ok1;
        math_Vector GaussP = new(1, Order);
        math_Vector GaussW = new(1, Order);
        Done = false;

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        math.GaussPoints(Order, GaussP);
        math.GaussWeights(Order, GaussW);

        // Calcul de l'integrale aux points de Gauss :

        // Changement de variable pour la mise a l'echelle [Lower, Upper] :
        xm = 0.5 * (Upper + Lower);
        xr = 0.5 * (Upper - Lower);
        Val = 0.0;

        int ind = Order / 2, ind1 = (Order + 1) / 2;
        if (ind1 > ind)
        { // odder case
            Ok1 = F.Value(xm, out Val);
            if (!Ok1)
            {
                return;
            }

            Val *= GaussW[ind1];
        }

        // Sommation sur tous les points de Gauss: avec utilisation de la symetrie.
        for (j = 1; j <= ind; j++)
        {
            dx = xr * GaussP[j];
            Ok1 = F.Value(xm - dx, out F1);
            if (!Ok1)
            {
                return;
            }

            Ok1 = F.Value(xm + dx, out F2);
            if (!Ok1)
            {
                return;
            }

            // Multiplication par les poids de Gauss.
            double FT = F1 + F2;
            Val += GaussW[j] * FT;
        }

        // Mise a l'echelle de l'intervalle [Lower, Upper]
        Val *= xr;
        Done = true;
    }
}
