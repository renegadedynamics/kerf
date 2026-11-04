// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to Circle C1 and a straight line L2.    +
    //                        centered on a straight line.                    +
    //  We start by making difference between cases that we are going to      +
    //  process separately.                                                   +
    //  In general case:                                                      +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and L2 that give us                   +
    //  all possible locations of centers of all circles tangent to C1 and L2 +
    //  We intersect these bissectrices with straight line OnLine which gives +
    //  us points among which we'll choose the solutions.                     +
    //  The choices are made basing on Qualifiers of C1 and L2.               +
    //=========================================================================

    /// <summary>The circles tangent to a circle and a line with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedLin qualified2,
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
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        // OCCT's limit case: the centre at the foot of C1's centre on the line (OCCT's retry for the other side
        // recomputed the same point)
        AddFoot(c1.Location, onLine, a1, a2, tol);
        // The centres are where the locus meets the bisectors; each is kept with the radii that touch both
        // arguments as asked (Kerf decision 1541)
        CircLin2dBisec bis = new(c1, l2);
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
                else if (type == ConstraintCurveType.Par)
                {
                    intp.Perform(onLine, new Conic(sol.Parabola()));
                }
                AddCandidates(intp, onLine, a1, a2, tol);
            }
            _wellDone = true;
        }
    }
}
