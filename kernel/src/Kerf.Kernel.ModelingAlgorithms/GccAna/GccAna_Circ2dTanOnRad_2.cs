// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //    typedef of handled objects :                                      +
    //=========================================================================

    //=========================================================================
    //   Circle tangent to a point      Point1.                               +
    //          center on straight line OnLine.                               +
    //          radius                  Radius.                               +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Solve the equation of second degree showing that the found center point +
    //  (xc,yc) is at distance Radius from point Point1 and on the straight line OnLine. +
    //  The solutions are represented by circles :                     +
    //                   - of center Pntcen(xc,yc)                            +
    //                   - of radius Radius.                                   +
    //=========================================================================

    /// <summary>Circles of radius Radius passing through a point, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in Point2D point1, in Line2D onLine, double radius, double tolerance)
        : this(8)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Solve(new CircleContact(point1), null, null, point1, onLine, null, radius, Math.Abs(tolerance));
    }
}
