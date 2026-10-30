// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The auxiliary statics of the intersection tools.</summary>
/// <remarks>
/// Unit 6A ports the member BOPDS_DS calls (<see cref="IntermediatePoint(double, double)"/>); the unit that ports
/// IntTools_Tools (6B) adds the rest of the class to this file.
/// </remarks>
public static partial class IntTools_Tools
{
    /// <summary>
    /// A parameter inside [<paramref name="aFirst"/>, <paramref name="aLast"/>], at the fraction 10 e^-pi
    /// (0.43213918) of the range.
    /// </summary>
    public static double IntermediatePoint(double aFirst, double aLast)
    {
        // define parameter division number as 10*e^(-M_PI) = 0.43213918
        const double PAR_T = 0.43213918;
        return (1.0 - PAR_T) * aFirst + PAR_T * aLast;
    }
}
