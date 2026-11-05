// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to two straight lines and a point.      +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedLin qualified1,
        in QualifiedLin qualified2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 2);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 2);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 2);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 2);
        _theSame1 = new NCollection_Array1<int>(1, 2);
        _theSame2 = new NCollection_Array1<int>(1, 2);
        _theSame3 = new NCollection_Array1<int>(1, 2);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 2);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 2);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 2);
        _par1sol = new NCollection_Array1<double>(1, 2);
        _par2sol = new NCollection_Array1<double>(1, 2);
        _par3sol = new NCollection_Array1<double>(1, 2);
        _pararg1 = new NCollection_Array1<double>(1, 2);
        _pararg2 = new NCollection_Array1<double>(1, 2);
        _pararg3 = new NCollection_Array1<double>(1, 2);

        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        double tol = Math.Abs(tolerance);
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        _pnttg3sol.Init(point3);

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        CircleContact a3 = new(point3);

        Lin2dBisec bis1 = new(l1, l2);
        LinPnt2dBisec bis2 = new(l1, point3);
        if (bis1.IsDone && bis2.IsDone)
        {
            int nbsolution1 = bis1.SolutionCount;
            ConstraintBisec sol2 = bis2.ThisSolution();
            for (int i = 1; i <= nbsolution1; i++)
            {
                Line2D sol1 = bis1.ThisSolution(i);
                ConstraintCurveType typ2 = sol2.ArcType();
                AnalyticIntersection intp = new();
                if (typ2 == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol1, sol2.Line());
                }
                else if (typ2 == ConstraintCurveType.Par)
                {
                    intp.Perform(sol1, new Conic(sol2.Parabola()));
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            // A circle tangent to the first line has the distance to it as its radius; AddSolution
                            // keeps it if it is on the sides asked of both lines and passes through the point (OCCT
                            // kept a centre on the wrong side of a qualified line: Kerf decision 1540)
                            double dist1 = l1.DistanceTo(center);
                            if (dist1 > tol)
                            {
                                AddSolution(new Circle2D(new Axis2D(center, dirx), dist1), a1, a2, a3, tol);
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
