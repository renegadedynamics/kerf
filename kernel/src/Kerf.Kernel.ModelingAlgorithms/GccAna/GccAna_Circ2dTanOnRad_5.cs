// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to a point   Point1.                                  +
    //          center on circle     OnCirc.                                  +
    //          radius               Radius.                                  +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                               +
    //  Create the circle with center in Point1 of radius Radius.             +
    //  Intersect this circle with OnCirc and obtain the center points        +
    //  of found solutions.                                                   +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius passing through a point, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in Point2D point1, in Circle2D onCirc, double radius, double tolerance)
        : this(8)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Solve(new CircleContact(point1), null, null, point1, null, onCirc, radius, Math.Abs(tolerance));
    }
}
