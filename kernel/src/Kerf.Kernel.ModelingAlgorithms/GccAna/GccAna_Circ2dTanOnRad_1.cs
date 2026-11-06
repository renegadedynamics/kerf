// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to straight line  Qualified1 (L1)                         +
    //          center on straight line OnLine                                  +
    //          of radius Radius.                                              +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Elimine depending on the qualifier the cases not being solutions.     +
    //  Create L1para : parallel to L1 in the direction required by the       +
    //                  qualifier at distance Radius.                         +
    //  Point P of intersection between L1para and OnLine will give the center point +
    //  of the solution.                                                +
    //  Create solutions cirsol with center P and radius Radius.          +
    //  Fill the fields.                                                +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedLin qualified1, in Line2D onLine, double radius, double tolerance)
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
        Solve(new CircleContact(qualified1), null, qualified1.Qualified(), null, onLine, null, radius, Math.Abs(tolerance));
    }
}
