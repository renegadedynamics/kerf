// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to straight line Qualified1 (L1).                     +
    //          center on  circle OnCirc.                                     +
    //          with radius       Radius.                                     +
    //                                                                        +
    //  Initialize table of solutions cirsol and all fields.                  +
    //  Eliminate cases not being the solution.                               +
    //  Create parallel line(s) to L1 in the required direction(s).           +
    //  Intersect parallel line(s) with OnCirc and obtain                     +
    //  center points of found solutions.                                     +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedLin qualified1, in Circle2D onCirc, double radius, double tolerance)
        : this(8)
    {
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Solve(new CircleContact(qualified1), null, qualified1.Qualified(), null, null, onCirc, radius, Math.Abs(tolerance));
    }
}
