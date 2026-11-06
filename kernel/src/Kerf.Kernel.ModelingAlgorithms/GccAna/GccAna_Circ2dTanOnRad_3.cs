// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to a circle   Qualified1 (C1).                        +
    //          center on a circle  OnCirc.                                   +
    //          of radius              Radius.                                +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Create parallel(s) to C1 in the required direction(s).       +
    //  Intersect parallel(s) with OnCirc and obtain the   +
    //  center point(s) of found solution(s).       +
    //  Create solution(s) cirsol.                               +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedCirc qualified1, in Circle2D onCirc, double radius, double tolerance)
        : this(8)
    {
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Solve(new CircleContact(qualified1), qualified1.Qualified(), null, null, null, onCirc, radius, Math.Abs(tolerance));
    }
}
