// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefCutting.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_PrefCutting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// Cuts an interval [a, b] at the preferential point nearest to (a + b) / 2, or at (a + b) / 2 when no
/// preferential point is closer to the middle than the ends are.
/// </summary>
internal class PrefCutting : Cutting
{
    private readonly NCollection_Array1<double> _pntOfCutting;

    /// <summary>Copies the preferential points.</summary>
    public PrefCutting(NCollection_Array1<double> cutPnts)
    {
        _pntOfCutting = new NCollection_Array1<double>(1, cutPnts.Length());
        _pntOfCutting.Assign(cutPnts);
    }

    public override bool Value(double a, double b, out double cuttingvalue)
    {
        //  Minimum length of a parametric interval: PConfusion()
        //                              for F(U,V): EPS1=1.e-9 (cf. MMEPS1)
        double lgmin = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i;
        double cut, mil = (a + b) / 2, dist = Math.Abs((a - b) / 2);
        cut = mil;
        for (i = _pntOfCutting.Lower(); i <= _pntOfCutting.Upper(); i++)
        {
            if ((dist - lgmin) > Math.Abs(mil - _pntOfCutting.Value(i)))
            {
                cut = _pntOfCutting.Value(i);
                dist = Math.Abs(mil - _pntOfCutting.Value(i));
            }
        }
        cuttingvalue = cut;
        return (Math.Abs(cut - a) >= lgmin && Math.Abs(b - cut) >= lgmin);
    }
}
