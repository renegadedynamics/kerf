// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to two circles C1 and C2.               +
    //                        centered on a circle.                           +
    //  We start with distinguishing various boundary cases that will be      +
    //  processed separately.                                            +
    //  In the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and C2 that give us all           +
    //  possible locations of centers of all circles tangent to C1 and C2. + We intersect these
    //  bissectrices with circle OnCirc which gives us   + points among which we choose the solutions. +
    //  The choice is made basing in Qualifiers of C1 and C2.  +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedCirc qualified2,
                               in Circle2D onCirc,
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
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        // OCCT's boundary cases: the centres on the line through C1's centre and the locus's
        AddAlong(c1.Location, onCirc, a1, a2, tol);
        // The centres are where the locus meets the bisectors; each is kept with the radii that touch both
        // arguments as asked (Kerf decision 1541)
        Circ2dBisec bis = new(c1, c2);
        if (bis.IsDone)
        {
            for (int i = 1; i <= bis.SolutionCount; i++)
            {
                ConstraintBisec sol = bis.ThisSolution(i);
                ConstraintCurveType type = sol.ArcType();
                AnalyticIntersection intp = new();
                if (type == ConstraintCurveType.Cir)
                {
                    intp.Perform(onCirc, sol.Circle());
                }
                else if (type == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol.Line(), onCirc);
                }
                else if (type == ConstraintCurveType.Hpr)
                {
                    intp.Perform(onCirc, new Conic(sol.Hyperbola()));
                }
                else if (type == ConstraintCurveType.Ell)
                {
                    intp.Perform(onCirc, new Conic(sol.Ellipse()));
                }
                AddCandidates(intp, onCirc, a1, a2, tol);
            }
            _wellDone = true;
        }
    }
}
