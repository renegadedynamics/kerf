// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_10.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to straight line L1 and a point Point2.   +
    //                        centered on a circle.                           +
    //  We start by making difference between boundary cases that will be    +
    //  processed separately.                                            +
    //  In general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to L1 and Point2 that give us all        +
    //  all possible locations of centers of all circles        +
    //  tangent to L1 and Point2.                                              +
    //  We intersect these bissectrices with circle OnCirc which gives us +
    //  points among which we'll choose the solutions.   +
    //  The choice is made using Qualifiers of L1.        +
    //=========================================================================

    /// <summary>The circles tangent to a line and through a point with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in Point2D point2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(16)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        double tol = Math.Abs(tolerance);
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D l1 = qualified1.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(point2);
        // OCCT's boundary cases: the centres on the line through the point and the locus's centre (OCCT compared
        // one's distance to the line with the other's)
        AddAlong(point2, onCirc, a1, a2, tol);
        // The centres are where the locus meets the bisector; each is kept with the radii that touch both arguments as
        // asked (Kerf decision 17102)
        LinPnt2dBisec bis = new(l1, point2);
        if (bis.IsDone)
        {
            ConstraintBisec sol = bis.ThisSolution();
            ConstraintCurveType type = sol.ArcType();
            AnalyticIntersection intp = new();
            if (type == ConstraintCurveType.Lin)
            {
                intp.Perform(sol.Line(), onCirc);
            }
            else if (type == ConstraintCurveType.Par)
            {
                intp.Perform(onCirc, new Conic(sol.Parabola()));
            }
            AddCandidates(intp, onCirc, a1, a2, tol);
            _wellDone = true;
        }
    }
}
