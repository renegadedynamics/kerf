// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d3Tan
{
    private const int MaxSol = 20;

    //=========================================================================
    //   Creation of a circle tangent to two circles and a point.           +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedCirc qualified2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, MaxSol);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, MaxSol);
        _theSame1 = new NCollection_Array1<int>(1, MaxSol);
        _theSame2 = new NCollection_Array1<int>(1, MaxSol);
        _theSame3 = new NCollection_Array1<int>(1, MaxSol);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, MaxSol);
        _par1sol = new NCollection_Array1<double>(1, MaxSol);
        _par2sol = new NCollection_Array1<double>(1, MaxSol);
        _par3sol = new NCollection_Array1<double>(1, MaxSol);
        _pararg1 = new NCollection_Array1<double>(1, MaxSol);
        _pararg2 = new NCollection_Array1<double>(1, MaxSol);
        _pararg3 = new NCollection_Array1<double>(1, MaxSol);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        double r1 = c1.Radius;
        double r2 = c2.Radius;
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;

        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        CircleContact a3 = new(point3);
        Circ2dBisec bis1 = new(c1, c2);
        CircPnt2dBisec bis2 = new(c1, point3);
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
                    if (typ1 == ConstraintCurveType.Cir)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol1.Circle(), sol2.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), sol1.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Circle(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Circle(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Ell)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol2.Circle(), new Conic(sol1.Ellipse()));
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), new Conic(sol1.Ellipse()));
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Ellipse(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Ellipse(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Lin)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol1.Line(), sol2.Circle());
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol1.Line(), sol2.Line());
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol1.Line(), new Conic(sol2.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol1.Line(), new Conic(sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == ConstraintCurveType.Hpr)
                    {
                        if (typ2 == ConstraintCurveType.Cir)
                        {
                            intp.Perform(sol2.Circle(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Lin)
                        {
                            intp.Perform(sol2.Line(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Hpr)
                        {
                            intp.Perform(sol2.Hyperbola(), new Conic(sol1.Hyperbola()));
                        }
                        else if (typ2 == ConstraintCurveType.Ell)
                        {
                            intp.Perform(sol2.Ellipse(), new Conic(sol1.Hyperbola()));
                        }
                    }
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int j = 1; j <= intp.PointCount; j++)
                            {
                                Point2D center = intp.Point(j).Value();
                                // A circle through the point has the distance to it as its radius; AddSolution keeps
                                // it if it touches both circles as asked, with the tangency points where it touches
                                // (OCCT combined radii from the circles and put the second tangency point on the
                                // wrong side of an enclosed solution: Kerf decision 17101)
                                double dist3 = center.DistanceTo(point3);
                                if (dist3 > tol)
                                {
                                    AddSolution(new Circle2D(new Axis2D(center, dirx), dist3), a1, a2, a3, tol);
                                }
                                if (_nbrSol == MaxSol)
                                {
                                    break;
                                }
                            }
                        }
                        _wellDone = true;
                        if (_nbrSol == MaxSol)
                        {
                            break;
                        }
                    }
                    if (_nbrSol == MaxSol)
                    {
                        break;
                    }
                }
                if (_nbrSol == MaxSol)
                {
                    break;
                }
            }
            // Done once both bisectors are: IntAna2d reports two hyperbolas that don't meet as not done, and OCCT
            // left the whole construction not done for it (Kerf decision 17101)
            _wellDone = true;
        }
    }
}
