// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussSetIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Gauss-Legendre integration of a set of N functions of M variables between the bounds Lower[a..b] and
/// Upper[a..b]. Only M = 1 is implemented.
/// </summary>
internal class GaussSetIntegration
{
    private math_Vector _val;
    private bool _done;

    /// <summary>
    /// Integrates F between Lower and Upper with Order Gauss points. Throws <see cref="MethodNotImplementedException"/> if F
    /// has more than one variable or the order exceeds <see cref="NumericTables.GaussPointsMax"/>.
    /// </summary>
    public GaussSetIntegration(FunctionSet f, math_Vector lower, math_Vector upper, math_IntegerVector order)
    {
        _val = new math_Vector(1, f.NbEquations());

        int nbEqua = f.NbEquations(), nbVar = f.NbVariables();
        int i;
        bool isOk;
        math_Vector fVal1 = new(1, nbEqua), fVal2 = new(1, nbEqua), tval = new(1, nbVar);

        // Verification
        Standard_NotImplemented_Raise_if(nbVar != 1 || order.Value(order.Lower()) > NumericTables.GaussPointsMax(), "GaussSetIntegration ");

        // Initialisations
        _done = false;

        double xdeb = lower.Value(lower.Lower());
        double xfin = upper.Value(upper.Lower());
        int ordre = order.Value(order.Lower());
        double xm, xr;
        math_Vector gaussP = new(1, ordre), gaussW = new(1, ordre);

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        NumericTables.GaussPoints(ordre, gaussP);
        NumericTables.GaussWeights(ordre, gaussW);

        // Changement de variable pour la mise a l'echelle [Lower, Upper] :
        xm = 0.5 * (xdeb + xfin);
        xr = 0.5 * (xfin - xdeb);

        int ind = ordre / 2, ind1 = (ordre + 1) / 2;
        if (ind1 > ind)
        {             // odder case
            tval[1] = xm; // +  Xr * GaussP(ind1);
            isOk = f.Value(tval, _val);
            if (!isOk)
            {
                return;
            }

            _val *= gaussW[ind1];
        }
        else
        {
            _val.Init(0);
        }

        for (i = 1; i <= ind; i++)
        {
            tval[1] = xm + xr * gaussP[i];
            isOk = f.Value(tval, fVal1);
            if (!isOk)
            {
                return;
            }

            tval[1] = xm - xr * gaussP[i];
            isOk = f.Value(tval, fVal2);
            if (!isOk)
            {
                return;
            }

            fVal1 += fVal2;
            fVal1 *= gaussW[i];
            _val += fVal1;
        }

        _val *= xr;

        _done = true;
    }

    /// <summary>True if the integrals were computed.</summary>
    public bool IsDone => _done;

    /// <summary>The values of the integrals (shared: don't modify). Throws <see cref="NotDoneException"/> if they were not computed.</summary>
    public math_Vector Value()
    {
        StdFail_NotDone_Raise_if(!_done, "Integration ");
        return _val;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussSetIntegration ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write("Integration Value = " + _val + "\n");
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
}
