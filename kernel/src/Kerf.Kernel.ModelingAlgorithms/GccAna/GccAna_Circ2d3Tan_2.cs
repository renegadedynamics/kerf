// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to a circle and two straight lines.          +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedLin qualified2,
        in QualifiedLin qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 8);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 8);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 8);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 8);
        _theSame1 = new NCollection_Array1<int>(1, 8);
        _theSame2 = new NCollection_Array1<int>(1, 8);
        _theSame3 = new NCollection_Array1<int>(1, 8);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 8);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 8);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 8);
        _par1sol = new NCollection_Array1<double>(1, 8);
        _par2sol = new NCollection_Array1<double>(1, 8);
        _par3sol = new NCollection_Array1<double>(1, 8);
        _pararg1 = new NCollection_Array1<double>(1, 8);
        _pararg2 = new NCollection_Array1<double>(1, 8);
        _pararg3 = new NCollection_Array1<double>(1, 8);

        _theSame1.Init(0);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsOutside || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Line2D l3 = qualified3.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        CircleContact a3 = new(qualified3);
        CircLin2dBisec bis1 = new(c1, l2);
        Lin2dBisec bis2 = new(l2, l3);
        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution1 = bis1.SolutionCount;
            int nbsolution2 = bis2.SolutionCount;
            for (int i = 1; i <= nbsolution1; i++)
            {
                ConstraintBisec sol1 = bis1.ThisSolution(i);
                ConstraintCurveType typ1 = sol1.ArcType();
                AnalyticIntersection intp = new();
                for (int k = 1; k <= nbsolution2; k++)
                {
                    if (typ1 == ConstraintCurveType.Lin)
                    {
                        intp.Perform(sol1.Line(), bis2.ThisSolution(k));
                    }
                    else if (typ1 == ConstraintCurveType.Par)
                    {
                        intp.Perform(bis2.ThisSolution(k), new Conic(sol1.Parabola()));
                    }
                    if (intp.IsDone)
                    {
                        if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                        {
                            for (int j = 1; j <= intp.PointCount; j++)
                            {
                                Point2D center = intp.Point(j).Value();
                                // A circle tangent to the second line has the distance to it as its radius;
                                // AddSolution keeps it if it also touches the circle and the third line as asked (OCCT
                                // combined radii from the circle without resetting its flag: Kerf decision 1540)
                                double dist2 = l2.DistanceTo(center);
                                if (dist2 > tol)
                                {
                                    AddSolution(new Circle2D(new Axis2D(center, dirx), dist2), a1, a2, a3, tol);
                                }
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }
}
