// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanOn
{
    /// <summary>The circles tangent to two lines with their centre on a circle. The tolerance is not used.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in QualifiedLin qualified2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(16)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        double tol = Math.Abs(tolerance);
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        // The centres are where the locus meets the bisectors; each is kept with the radii that touch both
        // arguments as asked (Kerf decision 1541)
        Lin2dBisec bis = new(l1, l2);
        if (bis.IsDone)
        {
            for (int i = 1; i <= bis.SolutionCount; i++)
            {
                AnalyticIntersection intp = new(bis.ThisSolution(i), onCirc);
                AddCandidates(intp, onCirc, a1, a2, tol);
            }
            _wellDone = true;
        }
    }
}
