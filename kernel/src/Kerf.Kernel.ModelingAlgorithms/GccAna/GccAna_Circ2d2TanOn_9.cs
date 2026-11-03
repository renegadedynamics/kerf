// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanOn
{
    /// <summary>The circles tangent to two lines with their centre on a circle. The tolerance is not used.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedLin qualified1,
                               in QualifiedLin qualified2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(4)
    {
        _ = tolerance; // OCCT: unnamed, not used
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;

        Direction2D dirx = new(Direction2D.D.X);
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D l1 = qualified1.Qualified();
        Line2D l2 = qualified2.Qualified();
        Lin2dBisec bis = new(l1, l2);
        int i = 0, j = 0;
        int nbsol = 0;
        double sgn = 1.0;
        double s = 1.0;
        bool ok = false;
        Direction2D d1 = l1.Direction;
        Direction2D d2 = l2.Direction;
        Direction2D dnor1 = new(-d1.Y, d1.X);
        Direction2D dnor2 = new(-d2.Y, d2.X);
        Xy xYnor1 = new(-d1.Y, d1.X);
        Xy xYnor2 = new(-d2.Y, d2.X);
        Point2D originL1 = l1.Location;
        Point2D originL2 = l2.Location;
        Xy dloc = originL1.XY() - originL2.XY();
        // OCCT: Angle is signed, so every pair of lines turning clockwise from D1 to D2 takes this "parallel" branch
        if (d1.AngleTo(d2) <= Constants.Resolution())
        {
            if (qualified1.IsEnclosed)
            {
                ok = dloc.Dot(xYnor1) <= 0.0;
            }
            else if (qualified1.IsOutside)
            {
                ok = dloc.Dot(xYnor1) >= 0.0;
            }
            else
            {
                ok = true;
            }
            // OCCT: overwrites the first line's test instead of combining with it
            if (qualified2.IsEnclosed)
            {
                ok = dloc.Dot(xYnor2) >= 0.0;
            }
            else if (qualified2.IsOutside)
            {
                ok = dloc.Dot(xYnor2) <= 0.0;
            }
            else
            {
                ok = true;
            }
            if (ok)
            {
                AnalyticIntersection intp = new(bis.ThisSolution(1), onCirc);
                if (intp.IsDone)
                {
                    _wellDone = true;
                    if (!intp.IsEmpty)
                    {
                        for (int l = 1; l <= intp.PointCount; l++)
                        {
                            _nbrSol++;
                            Point2D pt = intp.Point(l).Value();
                            Axis2D axe = new(pt, dirx);
                            _cirsol[_nbrSol] = new Circle2D(axe, l1.DistanceTo(pt));
                            //          ===============================================
                            Direction2D dc1 = new(originL1.XY() - pt.XY());
                            Direction2D dc2 = new(originL2.XY() - pt.XY());
                            if (!qualified1.IsUnqualified)
                            {
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                            }
                            else if (dc1.Dot(dnor1) > 0.0)
                            {
                                _qualifier1[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier1[_nbrSol] = Qualifier.Enclosed;
                            }
                            if (!qualified2.IsUnqualified)
                            {
                                _qualifier2[_nbrSol] = qualified2.Qualifier();
                            }
                            else if (dc2.Dot(dnor2) > 0.0)
                            {
                                _qualifier2[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier2[_nbrSol] = Qualifier.Enclosed;
                            }
                        }
                    }
                }
            }
        }
        else if (qualified1.IsEnclosed && qualified2.IsEnclosed)
        {
            //============================================================
            if (bis.IsDone)
            {
                if (bis.SolutionCount == 2)
                {
                    nbsol = 1;
                    i = 2;
                    j = 1;
                    sgn = -1.0;
                }
            }
        }
        else if (qualified1.IsEnclosed && qualified2.IsOutside)
        {
            //===========================================================
            if (bis.IsDone)
            {
                if (bis.SolutionCount >= 1)
                {
                    nbsol = 1;
                    i = 1;
                    j = 1;
                    if (d1.AngleTo(d2) >= 0.0)
                    {
                        sgn = -1.0;
                    }
                }
            }
        }
        else if (qualified1.IsOutside && qualified2.IsEnclosed)
        {
            //===========================================================
            if (bis.IsDone)
            {
                if (bis.SolutionCount >= 1)
                {
                    nbsol = 1;
                    i = 1;
                    j = 1;
                    if (d1.AngleTo(d2) <= 0.0)
                    {
                        sgn = -1.0;
                    }
                }
            }
        }
        else if (qualified1.IsOutside && qualified2.IsOutside)
        {
            //==========================================================
            if (bis.IsDone)
            {
                if (bis.SolutionCount >= 1)
                {
                    nbsol = 1;
                    i = 2;
                    j = 1;
                }
            }
        }
        else if (qualified1.IsUnqualified && qualified2.IsEnclosed)
        {
            //=============================================================
            if (bis.IsDone)
            {
                nbsol = 2;
                if (bis.SolutionCount >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (d1.AngleTo(d2) >= 0.0)
                {
                    s = -1.0;
                }
                else
                {
                    sgn = -1.0;
                }
            }
        }
        else if (qualified1.IsUnqualified && qualified2.IsOutside)
        {
            //==============================================================
            if (bis.IsDone)
            {
                nbsol = 2;
                if (bis.SolutionCount >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (d1.AngleTo(d2) >= 0.0)
                {
                    s = -1.0;
                    sgn = -1.0;
                }
            }
        }
        else if (qualified1.IsEnclosed && qualified2.IsUnqualified)
        {
            //===============================================================
            if (bis.IsDone)
            {
                nbsol = 2;
                if (bis.SolutionCount >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (d1.AngleTo(d2) >= 0.0)
                {
                    sgn = -1.0;
                }
                else
                {
                    s = -1.0;
                }
            }
        }
        else if (qualified1.IsOutside && qualified2.IsUnqualified)
        {
            //==============================================================
            if (bis.IsDone)
            {
                nbsol = 2;
                if (bis.SolutionCount >= 1)
                {
                    i = 1;
                    j = 2;
                }
                if (d1.AngleTo(d2) <= 0.0)
                {
                    s = -1.0;
                    sgn = -1.0;
                }
            }
        }
        else if (qualified1.IsUnqualified && qualified2.IsUnqualified)
        {
            //==================================================================
            nbsol = 4;
            i = 1;
            j = 2;
        }
        if (nbsol >= 1)
        {
            if (bis.IsDone)
            {
                int kk = 0;
                for (int k = i; k <= i + j - 1; k++)
                {
                    kk++;
                    AnalyticIntersection intp = new(bis.ThisSolution(k), onCirc);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int l = 1; l <= intp.PointCount; l++)
                            {
                                Vector2D v = new(intp.Point(l).Value(), bis.ThisSolution(k).Location);
                                if ((kk == 1 && sgn * v.Dot(new Vector2D(bis.ThisSolution(k).Direction)) >= 0.0)
                                    || (kk == 2 && sgn * s * v.Dot(new Vector2D(bis.ThisSolution(k).Direction)) >= 0.0)
                                    || nbsol == 4)
                                {
                                    _nbrSol++;
                                    // OCCT: the centre is the intersection point number i (the first bisector's
                                    // index), not l; native and the port throw Standard_OutOfRange when there is no
                                    // such point
                                    Point2D pt = intp.Point(i).Value();
                                    Axis2D axe = new(pt, dirx);
                                    _cirsol[_nbrSol] = new Circle2D(axe,
                                                                   //              ===============================
                                                                   l1.DistanceTo(intp.Point(l).Value()));
                                    //                                         ===================================
                                    Direction2D dc1 = new(originL1.XY() - pt.XY());
                                    Direction2D dc2 = new(originL2.XY() - pt.XY());
                                    if (!qualified1.IsUnqualified)
                                    {
                                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                                    }
                                    else if (dc1.Dot(dnor1) > 0.0)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    if (!qualified2.IsUnqualified)
                                    {
                                        _qualifier2[_nbrSol] = qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(dnor2) > 0.0)
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                    }
                                }
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
        if (_nbrSol > 0)
        {
            for (i = 1; i <= _nbrSol; i++)
            {
                Point2D pbid = _cirsol[i].Location;
                double radius = _cirsol[i].Radius;
                Direction2D dc2 = new(originL1.XY() - pbid.XY());
                double sign = dc2.Dot(new Direction2D(-l1.Direction.Y, l1.Direction.X));
                dc2 = new Direction2D(sign * new Xy(-l1.Direction.Y, l1.Direction.X));
                _pnttg1sol[i] = new Point2D(pbid.XY() + radius * dc2.XY());
                dc2 = new Direction2D(originL2.XY() - pbid.XY());
                sign = dc2.Dot(new Direction2D(-l2.Direction.Y, l2.Direction.X));
                dc2 = new Direction2D(sign * new Xy(-l2.Direction.Y, l2.Direction.X));
                _pnttg2sol[i] = new Point2D(pbid.XY() + radius * dc2.XY());
                _pntcen[i] = pbid;
                _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
                _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[i]);
                _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
                _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[i]);
                _parcen3[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[i]);
            }
        }
    }
}
