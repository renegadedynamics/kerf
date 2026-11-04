// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_DichoCutting.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_DichoCutting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>Cuts an interval [a, b] at (a + b) / 2.</summary>
internal class DichoCutting : Cutting
{
    public DichoCutting() { }

    public override bool Value(double a, double b, out double cuttingvalue)
    {
        // Minimum length of an interval for F(U,V): EPS1=1.e-9 (cf. MEPS1)
        double lgmin = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        cuttingvalue = (a + b) / 2;
        return (Math.Abs(b - a) >= 2 * lgmin);
    }
}
