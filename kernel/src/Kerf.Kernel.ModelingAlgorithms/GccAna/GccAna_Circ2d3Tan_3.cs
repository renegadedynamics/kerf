// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to three straight lines.                +
    //   Create Bissectrices at Qualified1 and Qualified2 and                 +
    //          Bissectrices at Qualified1 and Qualified3.                    +
    //   Intersect bissectrices calculated in this way ==> Center points      +
    //   Choose the center point that corresponds to qualifiers and           +
    //   construct the solution of radius equal to the distance between the   +
    //   chosen center point and straight line Qualified1.                               +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedLin qualified1,
        in QualifiedLin qualified2,
        in QualifiedLin qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 4);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 4);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 4);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 4);
        _theSame1 = new NCollection_Array1<int>(1, 4);
        _theSame2 = new NCollection_Array1<int>(1, 4);
        _theSame3 = new NCollection_Array1<int>(1, 4);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 4);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 4);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 4);
        _par1sol = new NCollection_Array1<double>(1, 4);
        _par2sol = new NCollection_Array1<double>(1, 4);
        _par3sol = new NCollection_Array1<double>(1, 4);
        _pararg1 = new NCollection_Array1<double>(1, 4);
        _pararg2 = new NCollection_Array1<double>(1, 4);
        _pararg3 = new NCollection_Array1<double>(1, 4);

        _theSame1.Init(0);
        _theSame2.Init(0);
        _theSame3.Init(0);
        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsOutside || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Line2D l3 = qualified3.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        CircleContact a3 = new(qualified3);
        Lin2dBisec bisec1 = new(l1, l2);
        Lin2dBisec bisec2 = new(l1, l3);
        if (bisec1.IsDone && bisec2.IsDone)
        {
            for (int i = 1; i <= bisec1.SolutionCount; i++)
            {
                for (int j = 1; j <= bisec2.SolutionCount; j++)
                {
                    AnalyticIntersection intp = new(bisec1.ThisSolution(i), bisec2.ThisSolution(j));
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int k = 1; k <= intp.PointCount; k++)
                            {
                                Point2D center = intp.Point(k).Value();
                                double radius = (l1.DistanceTo(center) + l2.DistanceTo(center) + l3.DistanceTo(center)) / 3.0;
                                // AddSolution keeps the circles on the sides the qualifiers ask, with their sides as
                                // computed qualifiers (OCCT filtered them with sides some branches never set or set
                                // wrong, and didn't move the qualifiers with the circles: Kerf decision 1540)
                                AddSolution(new Circle2D(new Axis2D(center, dirx), radius), a1, a2, a3, Math.Abs(tolerance));
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }
}
