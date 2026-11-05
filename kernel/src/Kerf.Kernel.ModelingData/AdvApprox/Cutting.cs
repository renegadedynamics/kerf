// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_Cutting.hxx
// Port of: src/ModelingData/TKG3d/AdvApprox/AdvApprox_Cutting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>Chooses where an approximation cuts an interval.</summary>
internal abstract class Cutting
{
    /// <summary>
    /// The cutting value for the interval [<paramref name="a"/>, <paramref name="b"/>] in
    /// <paramref name="cuttingvalue"/>; false if the interval must not be cut.
    /// </summary>
    public abstract bool Value(double a, double b, out double cuttingvalue);
}
