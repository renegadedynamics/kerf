// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The Gauss-Legendre integration of a set of N functions of M variables between the bounds Lower[a..b] and
/// Upper[a..b]. Only M = 1 is implemented.
/// </summary>
public class math_GaussSetIntegration
{
    private math_Vector Val;
    private bool Done;

    /// <summary>
    /// Integrates F between Lower and Upper with Order Gauss points. Throws <see cref="Standard_NotImplemented"/> if F
    /// has more than one variable or the order exceeds <see cref="math.GaussPointsMax"/>.
    /// </summary>
    public math_GaussSetIntegration(math_FunctionSet F, math_Vector Lower, math_Vector Upper, math_IntegerVector Order)
    {
        Val = new math_Vector(1, F.NbEquations());

        int NbEqua = F.NbEquations(), NbVar = F.NbVariables();
        int i;
        bool IsOk;
        math_Vector FVal1 = new(1, NbEqua), FVal2 = new(1, NbEqua), Tval = new(1, NbVar);

        // Verification
        Standard_NotImplemented_Raise_if(NbVar != 1 || Order.Value(Order.Lower()) > math.GaussPointsMax(), "GaussSetIntegration ");

        // Initialisations
        Done = false;

        double Xdeb = Lower.Value(Lower.Lower());
        double Xfin = Upper.Value(Upper.Lower());
        int Ordre = Order.Value(Order.Lower());
        double Xm, Xr;
        math_Vector GaussP = new(1, Ordre), GaussW = new(1, Ordre);

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        math.GaussPoints(Ordre, GaussP);
        math.GaussWeights(Ordre, GaussW);

        // Changement de variable pour la mise a l'echelle [Lower, Upper] :
        Xm = 0.5 * (Xdeb + Xfin);
        Xr = 0.5 * (Xfin - Xdeb);

        int ind = Ordre / 2, ind1 = (Ordre + 1) / 2;
        if (ind1 > ind)
        {             // odder case
            Tval[1] = Xm; // +  Xr * GaussP(ind1);
            IsOk = F.Value(Tval, Val);
            if (!IsOk)
            {
                return;
            }

            Val *= GaussW[ind1];
        }
        else
        {
            Val.Init(0);
        }

        for (i = 1; i <= ind; i++)
        {
            Tval[1] = Xm + Xr * GaussP[i];
            IsOk = F.Value(Tval, FVal1);
            if (!IsOk)
            {
                return;
            }

            Tval[1] = Xm - Xr * GaussP[i];
            IsOk = F.Value(Tval, FVal2);
            if (!IsOk)
            {
                return;
            }

            FVal1 += FVal2;
            FVal1 *= GaussW[i];
            Val += FVal1;
        }

        Val *= Xr;

        Done = true;
    }

    /// <summary>True if the integrals were computed.</summary>
    public bool IsDone() => Done;

    /// <summary>The values of the integrals (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if they were not computed.</summary>
    public math_Vector Value()
    {
        StdFail_NotDone_Raise_if(!Done, "Integration ");
        return Val;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussSetIntegration ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write("Integration Value = " + Val + "\n");
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
