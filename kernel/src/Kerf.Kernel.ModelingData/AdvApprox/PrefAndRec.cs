// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefAndRec.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefAndRec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// Cuts an interval with a list of preferential points and a list of recommended points.
/// </summary>
/// <remarks>
/// The cutting value is the preferential point nearest to (a + b) / 2 if one lies in
/// ](r * a + b) / (r + 1), (a + r * b) / (r + 1)[ where r is the weight; otherwise the recommended point nearest
/// to (a + b) / 2 if one is closer to the middle than the ends are; otherwise (a + b) / 2.
/// </remarks>
internal class PrefAndRec : Cutting
{
    private readonly NCollection_Array1<double> _recCutting;
    private readonly NCollection_Array1<double> _prefCutting;
    private readonly double _weight;

    /// <summary>Copies the points. Throws Standard_DomainError if <paramref name="weight"/> &lt;= 1.</summary>
    public PrefAndRec(NCollection_Array1<double> recomendedCut, NCollection_Array1<double> prefferedCut,
                                double weight = 5)
    {
        _recCutting = new NCollection_Array1<double>(1, recomendedCut.Length());
        _prefCutting = new NCollection_Array1<double>(1, prefferedCut.Length());
        _weight = weight;
        _recCutting.Assign(recomendedCut);
        _prefCutting.Assign(prefferedCut);
        if (_weight <= 1)
        {
            throw new DomainErrorException("PrefAndRec : Weight is too small");
        }
    }

    public override bool Value(double a, double b, out double cuttingvalue)
    {
        //  Minimum length of a parametric interval: 10*PConfusion()
        double lgmin = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i;
        double cut, mil = (a + b) / 2, dist;
        bool isfound = false;

        cut = mil;

        // Search for a preferred cutting point
        dist = Math.Abs((a * _weight + b) / (1 + _weight) - mil);
        for (i = 1; i <= _prefCutting.Length(); i++)
        {
            if (dist > Math.Abs(mil - _prefCutting.Value(i)))
            {
                cut = _prefCutting.Value(i);
                dist = Math.Abs(mil - cut);
                isfound = true;
            }
        }

        // Search for a recommended cutting point
        if (!isfound)
        {
            dist = Math.Abs((a - b) / 2);
            for (i = 1; i <= _recCutting.Length(); i++)
            {
                if ((dist - lgmin) > Math.Abs(mil - _recCutting.Value(i)))
                {
                    cut = _recCutting.Value(i);
                    dist = Math.Abs(mil - cut);
                }
            }
        }

        // Result
        cuttingvalue = cut;
        return (Math.Abs(cut - a) >= lgmin && Math.Abs(b - cut) >= lgmin);
    }
}
