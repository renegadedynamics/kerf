// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d3Tan
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
        Point2D origin1 = l1.Location;
        Direction2D dir1 = l1.Direction;
        Direction2D normL1 = new(-dir1.Y, dir1.X);
        Point2D origin2 = l2.Location;
        Direction2D dir2 = l2.Direction;
        Direction2D normL2 = new(-dir2.Y, dir2.X);
        Point2D origin3 = l3.Location;
        Direction2D dir3 = l3.Direction;
        Direction2D normL3 = new(-dir3.Y, dir3.X);
        double xloc1 = origin1.X;
        double xloc2 = origin2.X;
        double xloc3 = origin3.X;
        double yloc1 = origin1.Y;
        double yloc2 = origin2.Y;
        double yloc3 = origin3.Y;
        double xdir1 = dir1.X;
        double xdir2 = dir2.X;
        double xdir3 = dir3.X;
        double ydir1 = dir1.Y;
        double ydir2 = dir2.Y;
        double ydir3 = dir3.Y;
        Lin2dBisec bisec1 = new(l1, l2);
        Lin2dBisec bisec2 = new(l1, l3);
        int ncote1 = 0;
        int ncote2 = 0;
        int ncote3 = 0;
        NCollection_Array1<double> cote1 = new(1, 2);
        NCollection_Array1<double> cote2 = new(1, 2);
        NCollection_Array1<double> cote3 = new(1, 2);
        int nbsol = 0;
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
                                nbsol++;
                                double radius =
                                  (l1.DistanceTo(intp.Point(k).Value()) + l2.DistanceTo(intp.Point(k).Value())
                                   + l3.DistanceTo(intp.Point(k).Value()))
                                  / 3.0;
                                Point2D center = intp.Point(k).Value();
                                double cx = center.X;
                                double cy = center.Y;
                                _cirsol[nbsol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //             ======================================================
                                Direction2D dc1 = new(origin1.XY() - center.XY());
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[nbsol] = qualified1.Qualifier();
                                }
                                else if (dc1.Dot(normL1) > 0.0)
                                {
                                    _qualifier1[nbsol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[nbsol] = Qualifier.Enclosed;
                                }
                                Direction2D dc2 = new(origin2.XY() - center.XY());
                                if (!qualified2.IsUnqualified)
                                {
                                    _qualifier2[nbsol] = qualified2.Qualifier();
                                }
                                else if (dc2.Dot(normL2) > 0.0)
                                {
                                    _qualifier2[nbsol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier2[nbsol] = Qualifier.Enclosed;
                                }
                                Direction2D dc3 = new(origin3.XY() - center.XY());
                                if (!qualified3.IsUnqualified)
                                {
                                    _qualifier3[nbsol] = qualified3.Qualifier();
                                }
                                else if (dc3.Dot(normL3) > 0.0)
                                {
                                    _qualifier3[nbsol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier3[nbsol] = Qualifier.Enclosed;
                                }

                                double cross1 = new Direction2D(-ydir1, xdir1).Dot(new Direction2D(xloc1 - cx, yloc1 - cy));
                                double cross2 = new Direction2D(-ydir2, xdir2).Dot(new Direction2D(xloc2 - cx, yloc2 - cy));
                                double cross3 = new Direction2D(-ydir3, xdir3).Dot(new Direction2D(xloc3 - cx, yloc3 - cy));
                                if (cross1 != 0.0)
                                {
                                    cross1 = cross1 / Math.Abs(cross1);
                                }
                                _pnttg1sol[nbsol] = new Point2D(new Xy(cx, cy) + cross1 * radius * new Xy(-ydir1, xdir1));
                                if (cross2 != 0.0)
                                {
                                    cross2 = cross2 / Math.Abs(cross2);
                                }
                                _pnttg2sol[nbsol] = new Point2D(new Xy(cx, cy) + cross2 * radius * new Xy(-ydir2, xdir2));
                                if (cross3 != 0.0)
                                {
                                    cross3 = cross3 / Math.Abs(cross3);
                                }
                                _pnttg3sol[nbsol] = new Point2D(new Xy(cx, cy) + cross3 * radius * new Xy(-ydir3, xdir3));
                                _par1sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[nbsol], _pnttg1sol[nbsol]);
                                _pararg1[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[nbsol]);
                                _par2sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[nbsol], _pnttg2sol[nbsol]);
                                _pararg2[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[nbsol]);
                                _par3sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[nbsol], _pnttg3sol[nbsol]);
                                _pararg3[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l3, _pnttg3sol[nbsol]);
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
        if (qualified1.IsEnclosed && qualified2.IsEnclosed &&
            //  =========================================================
            qualified3.IsEnclosed)
        {
            //     ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = 1.0;
            cote3[1] = 1.0;
        }
        else if (qualified1.IsEnclosed && qualified2.IsEnclosed &&
                 // ==============================================================
                 qualified3.IsOutside)
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = 1.0;
            cote3[1] = -1.0;
        }
        else if (qualified1.IsEnclosed && qualified2.IsOutside &&
                 // =============================================================
                 qualified3.IsEnclosed)
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = -1.0;
            cote3[1] = 1.0;
        }
        else if (qualified1.IsEnclosed && qualified2.IsOutside &&
                 // =============================================================
                 qualified3.IsOutside)
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = -1.0;
            cote3[1] = -1.0;
        }
        else if (qualified1.IsOutside && qualified2.IsEnclosed &&
                 // =============================================================
                 qualified3.IsEnclosed)
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = 1.0;
            cote3[1] = 1.0;
        }
        else if (qualified1.IsOutside && qualified2.IsEnclosed &&
                 // =============================================================
                 qualified3.IsOutside)
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = 1.0;
            cote3[1] = -1.0;
        }
        else if (qualified1.IsOutside && qualified2.IsOutside &&
                 // ============================================================
                 qualified3.IsEnclosed)
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = -1.0;
            cote3[1] = 1.0;
        }
        else if (qualified1.IsOutside && qualified2.IsOutside &&
                 // ============================================================
                 qualified3.IsOutside)
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = -1.0;
            cote3[1] = -1.0;
        }
        else
        {
            if (qualified1.IsUnqualified)
            {
                //   ====================================
                ncote1 = 2;
                cote1[1] = 1.0;
                cote1[2] = -1.0;
                if (qualified2.IsUnqualified)
                {
                    //     ===============================
                    ncote2 = 2;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    if (qualified3.IsUnqualified)
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote2 where cote3 is meant
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        _nbrSol = nbsol;
                        _wellDone = true;
                    }
                    else if (qualified3.IsEnclosed)
                    {
                        //       ===============================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (qualified3.IsOutside)
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (qualified2.IsEnclosed)
                {
                    //     =================================
                    ncote2 = 1;
                    cote2[1] = 1.0;
                    if (qualified3.IsUnqualified)
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote3(1) twice, cote3(2) is never set
                        cote3[1] = 1.0;
                        cote3[1] = -1.0;
                    }
                    else if (qualified3.IsEnclosed)
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (qualified3.IsOutside)
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (qualified2.IsOutside)
                {
                    //     ================================
                    ncote2 = 1;
                    cote2[1] = -1.0;
                    if (qualified3.IsUnqualified)
                    {
                        //       ===============================
                        ncote3 = 2;
                        cote3[1] = 1.0;
                        cote3[2] = -1.0;
                    }
                    else if (qualified3.IsEnclosed)
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (qualified3.IsOutside)
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
            }
            else if (qualified2.IsUnqualified)
            {
                //   ===================================
                ncote2 = 2;
                cote2[1] = 1.0;
                cote2[2] = -1.0;
                if (qualified1.IsEnclosed)
                {
                    //     ============================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (qualified3.IsUnqualified)
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote3(1) twice, cote3(2) is never set
                        cote3[1] = -1.0;
                        cote3[1] = -1.0;
                    }
                    else if (qualified3.IsEnclosed)
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (qualified3.IsOutside)
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (qualified1.IsOutside)
                {
                    //     ================================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (qualified3.IsUnqualified)
                    {
                        //       ===============================
                        ncote3 = 2;
                        cote3[1] = 1.0;
                        cote3[2] = -1.0;
                    }
                    else if (qualified3.IsEnclosed)
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (qualified3.IsOutside)
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
            }
            else if (qualified3.IsUnqualified)
            {
                //   ===================================
                ncote3 = 2;
                cote3[1] = 1.0;
                cote3[2] = -1.0;
                if (qualified1.IsEnclosed)
                {
                    //     ============================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (qualified2.IsEnclosed)
                    {
                        //       ============================
                        ncote2 = 1;
                        cote2[1] = 1.0;
                    }
                    else if (qualified2.IsOutside)
                    {
                        //       ===============================
                        ncote2 = 1;
                        cote2[1] = -1.0;
                    }
                }
                else if (qualified1.IsOutside)
                {
                    //     ================================
                    ncote1 = 1;
                    cote1[1] = -1.0;
                    if (qualified2.IsEnclosed)
                    {
                        //       ============================
                        ncote2 = 1;
                        cote2[1] = 1.0;
                    }
                    else if (qualified2.IsOutside)
                    {
                        //       ===============================
                        ncote2 = 1;
                        cote2[1] = -1.0;
                    }
                }
            }
        }
        if (_nbrSol > 0)
        {
            return;
        }
        for (int i = 1; i <= nbsol; i++)
        {
            for (int j1 = 1; j1 <= ncote1; j1++)
            {
                for (int j2 = 1; j2 <= ncote2; j2++)
                {
                    for (int j3 = 1; j3 <= ncote3; j3++)
                    {
                        if ((cote2[j2]
                               * ((_cirsol[i].Location.X - origin2.X) * (-dir2.Y)
                                  + (_cirsol[i].Location.Y - origin2.Y) * (dir2.X))
                             > 0.0)
                            && (cote3[j3]
                                  * ((_cirsol[i].Location.X - origin3.X) * (-dir3.Y)
                                     + (_cirsol[i].Location.Y - origin3.Y) * (dir3.X))
                                > 0.0)
                            && (cote1[j1]
                                  * ((_cirsol[i].Location.X - origin1.X) * (-dir1.Y)
                                     + (_cirsol[i].Location.Y - origin1.Y) * (dir1.X))
                                > 0.0))
                        {
                            _nbrSol++;
                            // OCCT: qualifier1..3 are not copied to the new slot with the circle
                            _cirsol[_nbrSol] = _cirsol[i];
                            //           =====================================
                            double radius = _cirsol[_nbrSol].Radius;
                            Point2D center = _cirsol[_nbrSol].Location;
                            Direction2D dc = new(origin1.XY() - center.XY());
                            double sign = dc.Dot(new Direction2D(-dir1.Y, dir1.X));
                            dc = new Direction2D(sign * new Xy(-dir1.Y, dir1.X));
                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                            dc = new Direction2D(origin2.XY() - center.XY());
                            sign = dc.Dot(new Direction2D(-dir2.Y, dir2.X));
                            dc = new Direction2D(sign * new Xy(-dir2.Y, dir2.X));
                            _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                            dc = new Direction2D(origin3.XY() - center.XY());
                            sign = dc.Dot(new Direction2D(-dir3.Y, dir3.X));
                            dc = new Direction2D(sign * new Xy(-dir3.Y, dir3.X));
                            _pnttg3sol[_nbrSol] = new Point2D(center.XY() + radius * dc.XY());
                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                            _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                            _pararg3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l3, _pnttg3sol[_nbrSol]);
                        }
                    }
                }
            }
        }
    }
}
