// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //  Circles tangent to circle C1, passing by point Point2 and centers     +
    //  on a straight line OnLine.                                            +
    //  We start by making difference with boundary cases that will be        +
    //  processed separately.                                                 +
    //  For the general case:                                                 +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and Point2 that give us all           +
    //  possible locations of centers of all circles                          +
    //  tangent to C1 and passing by Point2.                                  +
    //  We intersect these bissectrices with the straight line OnLine which   +
    //  gives us the points among which we'll choose the solutions.           +
    //  The choices are made using Qualifiers of C1 and C2.                   +
    //=========================================================================

    /// <summary>The circles tangent to a circle and through a point with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in Point2D point2,
                               in Line2D onLine,
                               double tolerance)
        : this(16)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        double tol = Math.Abs(tolerance);
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(point2);
        // OCCT's boundary case: the centre at the foot of the point on the line
        AddFoot(point2, onLine, a1, a2, tol);
        // The centres are where the locus meets the bisectors; each is kept with the radii that touch both
        // arguments as asked (Kerf decision 1541)
        CircPnt2dBisec bis = new(c1, point2);
        if (bis.IsDone)
        {
            for (int i = 1; i <= bis.SolutionCount; i++)
            {
                ConstraintBisec sol = bis.ThisSolution(i);
                ConstraintCurveType type = sol.ArcType();
                AnalyticIntersection intp = new();
                if (type == ConstraintCurveType.Lin)
                {
                    intp.Perform(onLine, sol.Line());
                }
                else if (type == ConstraintCurveType.Cir)
                {
                    intp.Perform(onLine, sol.Circle());
                }
                else if (type == ConstraintCurveType.Ell)
                {
                    intp.Perform(onLine, new Conic(sol.Ellipse()));
                }
                else if (type == ConstraintCurveType.Hpr)
                {
                    intp.Perform(onLine, new Conic(sol.Hyperbola()));
                }
                AddCandidates(intp, onLine, a1, a2, tol);
            }
            _wellDone = true;
        }
    }
}
