// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    //   Creation of a circle tangent to two circles and a straight line.     +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedCirc qualified2,
        in QualifiedLin qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 16);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 16);
        _theSame1 = new NCollection_Array1<int>(1, 16);
        _theSame2 = new NCollection_Array1<int>(1, 16);
        _theSame3 = new NCollection_Array1<int>(1, 16);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 16);
        _par1sol = new NCollection_Array1<double>(1, 16);
        _par2sol = new NCollection_Array1<double>(1, 16);
        _par3sol = new NCollection_Array1<double>(1, 16);
        _pararg1 = new NCollection_Array1<double>(1, 16);
        _pararg2 = new NCollection_Array1<double>(1, 16);
        _pararg3 = new NCollection_Array1<double>(1, 16);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsOutside || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        Line2D l3 = qualified3.Qualified();
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        CircleContact a3 = new(qualified3);
        CircLin2dBisec bis1 = new(c1, l3);
        CircLin2dBisec bis2 = new(c2, l3);
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
                    ConstraintBisec sol2 = bis2.ThisSolution(k);
                    ConstraintCurveType typ2 = sol2.ArcType();
                    if (typ1 == ConstraintCurveType.Lin)
                    {
                        if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol1.Line(), sol2.Line());
                        }
                        else if (typ2 == ConstraintCurveType.Par)
                        {
                            intp.Perform(sol1.Line(), new Conic(sol2.Parabola()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Par)
                    {
                        if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), new Conic(sol1.Parabola()));
                        }
                        else if (typ2 == ConstraintCurveType.Par)
                        {
                            intp.Perform(sol1.Parabola(), new Conic(sol2.Parabola()));
                        }
                    }
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int j = 1; j <= intp.PointCount; j++)
                            {
                                Point2D center = intp.Point(j).Value();

                                // pop : if the coordinates are too great, no creation
                                if (center.X > 1e10 || center.Y > 1e10)
                                {
                                    break;
                                }

                                double dist3 = l3.DistanceTo(center);

                                // pop : if the coordinates are too great, no creation
                                if (dist3 > 1e10)
                                {
                                    break;
                                }

                                // A circle tangent to the line has the distance to it as its radius; AddSolution
                                // keeps it if it also touches both circles as asked (OCCT combined radii from the
                                // circles without resetting its flag, never checked the line's distance, and tested
                                // Qualified2 for the line: Kerf decision 1540)
                                if (dist3 > tol)
                                {
                                    AddSolution(new Circle2D(new Axis2D(center, dirx), dist3), a1, a2, a3, tol);
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
