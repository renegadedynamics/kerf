// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to straight line  Qualified1 (L1)                         +
    //          center on straight line OnLine                                  +
    //          of radius Radius.                                              +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Elimine depending on the qualifier the cases not being solutions.     +
    //  Create L1para : parallel to L1 in the direction required by the       +
    //                  qualifier at distance Radius.                         +
    //  Point P of intersection between L1para and OnLine will give the center point +
    //  of the solution.                                                +
    //  Create solutions cirsol with center P and radius Radius.          +
    //  Fill the fields.                                                +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedLin qualified1, in Line2D onLine, double radius, double tolerance)
    {
        _cirsol = new(1, 2);
        _qualifier1 = new(1, 2);
        _theSame1 = new(1, 2);
        _pnttg1sol = new(1, 2);
        _pntcen3 = new(1, 2);
        _par1sol = new(1, 2);
        _pararg1 = new(1, 2);
        _parcen3 = new(1, 2);


        double   tol = Math.Abs(tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        _nbrSol   = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int                     nbsol = 0;
        NCollection_Array1<int> eps = new(1, 2);
        Line2D                l1 = qualified1.Qualified();
        Point2D origin1 = l1.Location();
        Direction2D dir1 = l1.Direction();
        Direction2D normL1 = new(-dir1.Y(), dir1.X());

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if ((onLine.Direction()).IsParallel(dir1, tol))
        {
            _wellDone = true;
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //   ============================
                eps[1] = -1;
                nbsol  = 1;
            }
            else if (qualified1.IsOutside)
            {
                //   ================================
                eps[1] = 1;
                nbsol  = 1;
            }
            else
            {
                //   ====
                eps[1] = 1;
                eps[2] = -1;
                nbsol  = 2;
            }
            double dx1 = dir1.X();
            double dy1 = dir1.Y();
            double lx1 = origin1.X();
            double ly1 = origin1.Y();
            for (int j = 1; j <= nbsol; j++)
            {
                Line2D l1para = new(new Point2D(lx1 + eps[j] * radius * dy1, ly1 - eps[j] * radius * dx1), dir1);
                AnalyticIntersection intp = new(onLine, l1para);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =====================================================
                            Direction2D dc1 = new(origin1.XY() - center.XY());
                            if (!qualified1.IsUnqualified)
                            {
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                            }
                            else if (dc1.Dot(normL1) > 0.0)
                            {
                                _qualifier1[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier1[_nbrSol] = Qualifier.Enclosed;
                            }
                            _theSame1[_nbrSol] = 0;
                            if (new Vector2D(center, origin1).Dot(new Vector2D(new Direction2D(-dy1, dx1))) > 0.0)
                            {
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * new Xy(-dy1, dx1));
                                _pntcen3[_nbrSol]   = _cirsol[1].Location(); // OCCT: cirsol(1), not cirsol(NbrSol)
                            }
                            else
                            {
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() - radius * new Xy(-dy1, dx1));
                                _pntcen3[_nbrSol]   = _cirsol[1].Location(); // OCCT: cirsol(1), not cirsol(NbrSol)
                            }
                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                            _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen3[_nbrSol]);
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
