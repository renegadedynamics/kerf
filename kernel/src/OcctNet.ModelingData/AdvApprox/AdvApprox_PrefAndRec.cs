// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefAndRec.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefAndRec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Cuts an interval with a list of preferential points and a list of recommended points.
/// </summary>
/// <remarks>
/// The cutting value is the preferential point nearest to (a + b) / 2 if one lies in
/// ](r * a + b) / (r + 1), (a + r * b) / (r + 1)[ where r is the weight; otherwise the recommended point nearest
/// to (a + b) / 2 if one is closer to the middle than the ends are; otherwise (a + b) / 2.
/// </remarks>
public class AdvApprox_PrefAndRec : AdvApprox_Cutting
{
    private readonly NCollection_Array1<double> myRecCutting;
    private readonly NCollection_Array1<double> myPrefCutting;
    private readonly double myWeight;

    /// <summary>Copies the points. Throws Standard_DomainError if <paramref name="Weight"/> &lt;= 1.</summary>
    public AdvApprox_PrefAndRec(NCollection_Array1<double> RecomendedCut, NCollection_Array1<double> PrefferedCut,
                                double Weight = 5)
    {
        myRecCutting = new NCollection_Array1<double>(1, RecomendedCut.Length());
        myPrefCutting = new NCollection_Array1<double>(1, PrefferedCut.Length());
        myWeight = Weight;
        myRecCutting.Assign(RecomendedCut);
        myPrefCutting.Assign(PrefferedCut);
        if (myWeight <= 1)
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
        dist = Math.Abs((a * myWeight + b) / (1 + myWeight) - mil);
        for (i = 1; i <= myPrefCutting.Length(); i++)
        {
            if (dist > Math.Abs(mil - myPrefCutting.Value(i)))
            {
                cut = myPrefCutting.Value(i);
                dist = Math.Abs(mil - cut);
                isfound = true;
            }
        }

        // Search for a recommended cutting point
        if (!isfound)
        {
            dist = Math.Abs((a - b) / 2);
            for (i = 1; i <= myRecCutting.Length(); i++)
            {
                if ((dist - lgmin) > Math.Abs(mil - myRecCutting.Value(i)))
                {
                    cut = myRecCutting.Value(i);
                    dist = Math.Abs(mil - cut);
                }
            }
        }

        // Result
        cuttingvalue = cut;
        return (Math.Abs(cut - a) >= lgmin && Math.Abs(b - cut) >= lgmin);
    }
}
