// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d3Tan
{
    //=========================================================================
    //   Creation of a circle passing by three points.                       +
    //   Three cases  :                                               +
    //      1/ Three points coincide.                               +
    //      -----------------------------------                               +
    //      The result is the circle with center in Point1 with zero radius.  +
    //      2/ Two of three points coincide.                          +
    //      ----------------------------------------                          +
    //      Create the medium line between two non-coinciding points and      +
    //      the straight line passing by these two points.                               +
    //      The center of the solution is the intersection of two straight lines and the   +
    //      radius is the distance between this center and one of three points.  +
    //      3/ The three points are distinct.                                +
    //      ----------------------------------                                +
    //=========================================================================
    public AnalyticCirc2d3Tan(in Point2D point1,
        in Point2D point2,
        in Point2D point3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 1);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 1);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 1);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 1);
        _theSame1 = new NCollection_Array1<int>(1, 1);
        _theSame2 = new NCollection_Array1<int>(1, 1);
        _theSame3 = new NCollection_Array1<int>(1, 1);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 1);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 1);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 1);
        _par1sol = new NCollection_Array1<double>(1, 1);
        _par2sol = new NCollection_Array1<double>(1, 1);
        _par3sol = new NCollection_Array1<double>(1, 1);
        _pararg1 = new NCollection_Array1<double>(1, 1);
        _pararg2 = new NCollection_Array1<double>(1, 1);
        _pararg3 = new NCollection_Array1<double>(1, 1);

        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        _nbrSol = 0;

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        double dist1 = point1.DistanceTo(point2);
        double dist2 = point1.DistanceTo(point3);
        double dist3 = point2.DistanceTo(point3);

        _qualifier1[1] = Qualifier.Noqualifier;
        _qualifier2[1] = Qualifier.Noqualifier;
        _qualifier3[1] = Qualifier.Noqualifier;

        if ((dist1 < tolerance) && (dist2 < tolerance) && (dist3 < tolerance))
        {
            _nbrSol++;
            _wellDone = true;
            _cirsol[1] = new Circle2D(new Axis2D(point1, dirx), 0.0);
            //   ===============================================
            _theSame1[1] = 0;
            _theSame2[1] = 0;
            _theSame3[1] = 0;
            _pnttg1sol[1] = point1;
            _pnttg2sol[1] = point2;
            _pnttg3sol[1] = point3;
            _par1sol[1] = 0.0;
            _par2sol[1] = 0.0;
            _par3sol[1] = 0.0;
            _pararg1[1] = 0.0;
            _pararg2[1] = 0.0;
            _pararg3[1] = 0.0;
        }
        else
        {
            Line2D l1 = new();
            Line2D l2 = new();
            if (dist1 >= tolerance)
            {
                l1 = new Line2D(new Point2D((point1.XY() + point2.XY()) / 2.0),
                              new Direction2D(point1.Y - point2.Y, point2.X - point1.X));
            }
            if (dist2 >= tolerance)
            {
                l2 = new Line2D(new Point2D((point1.XY() + point3.XY()) / 2.0),
                              new Direction2D(point1.Y - point3.Y, point3.X - point1.X));
            }
            if (dist2 <= tolerance)
            {
                l2 = new Line2D(point1, new Direction2D(point1.Y - point2.Y, point2.X - point1.X));
            }
            else if (dist1 <= tolerance)
            {
                l1 = new Line2D(point1, new Direction2D(point1.Y - point3.Y, point3.X - point1.X));
            }
            else if (dist3 <= tolerance)
            {
                l2 = new Line2D(point1, new Direction2D(point1.Y - point2.Y, point2.X - point1.X));
            }
            AnalyticIntersection intp = new(l1, l2);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    for (int i = 1; i <= intp.PointCount; i++)
                    {
                        _nbrSol++;
                        _cirsol[_nbrSol] =
                          new Circle2D(new Axis2D(intp.Point(i).Value(), dirx),
                                    //         ===============================================================
                                    point1.DistanceTo(intp.Point(i).Value()));
                        //                                    =======================================

                        _theSame1[_nbrSol] = 0;
                        _theSame2[_nbrSol] = 0;
                        _theSame3[_nbrSol] = 0;
                        _pnttg1sol[_nbrSol] = point1;
                        _pnttg2sol[_nbrSol] = point2;
                        _pnttg3sol[_nbrSol] = point3;
                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                        _pararg1[_nbrSol] = 0.0;
                        _pararg2[_nbrSol] = 0.0;
                        _pararg3[_nbrSol] = 0.0;
                    }
                }
                _wellDone = true;
            }
        }
    }
}
