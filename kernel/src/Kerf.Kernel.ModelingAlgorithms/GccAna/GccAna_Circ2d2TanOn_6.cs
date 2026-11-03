// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

public sealed partial class AnalyticCirc2d2TanOn
{
    //=========================================================================
    //   Creation of a circle tangent to two circles C1 and C2.               +
    //                        centered on a circle.                           +
    //  We start with distinguishing various boundary cases that will be      +
    //  processed separately.                                            +
    //  In the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to C1 and C2 that give us all           +
    //  possible locations of centers of all circles tangent to C1 and C2. + We intersect these
    //  bissectrices with circle OnCirc which gives us   + points among which we choose the solutions. +
    //  The choice is made basing in Qualifiers of C1 and C2.  +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a circle.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedCirc qualified2,
                               in Circle2D onCirc,
                               double tolerance)
        : this(8)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double tol = Math.Abs(tolerance);
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        Direction2D dirx = new(Direction2D.D.X);
        NCollection_Array1<double> radius = new(1, 2);
        NCollection_Array1<double> rradius = new(1, 2);
        Point2D center1 = c1.Location();
        Point2D center2 = c2.Location();

        double r1 = c1.Radius();
        double r2 = c2.Radius();

        //=========================================================================
        //   Processing of boundary cases.                                          +
        //=========================================================================

        int nbsol1 = 1;
        int nbsol2 = 0;
        double ron = onCirc.Radius();
        double distcco = onCirc.Location().Distance(center1);
        Direction2D dircc = new(onCirc.Location().XY() - center1.XY());
        Point2D pinterm = new(center1.XY() + (distcco - ron) * dircc.XY());
        double distcc2 = pinterm.Distance(center2);
        double distcc1 = pinterm.Distance(center1);
        double d1 = Math.Abs(distcc2 - r2 - Math.Abs(distcc1 - r1));
        double d2 = Math.Abs(distcc2 + r2 - Math.Abs(distcc1 - r1));
        double d3 = Math.Abs(distcc2 - r2 - (distcc1 + r1));
        double d4 = Math.Abs(distcc2 + r2 - (distcc1 + r1));
        if (d1 > tol || d2 > tol || d3 > tol || d4 > tol)
        {
            pinterm = new Point2D(center1.XY() + (distcco + ron) * dircc.XY());
            distcc2 = pinterm.Distance(center2);
            distcc1 = pinterm.Distance(center1);
            d1 = Math.Abs(distcc2 - r2 - Math.Abs(distcc1 - r1));
            d2 = Math.Abs(distcc2 + r2 - Math.Abs(distcc1 - r1));
            d3 = Math.Abs(distcc2 - r2 - (distcc1 + r1));
            d4 = Math.Abs(distcc2 + r2 - (distcc1 + r1));
            if (d1 > tol || d2 > tol || d3 > tol || d4 > tol)
            {
                nbsol1 = 0;
            }
        }
        if (nbsol1 > 0)
        {
            if (qualified1.IsEnclosed || qualified1.IsOutside)
            {
                nbsol1 = 1;
                radius[1] = Math.Abs(distcc1 - r1);
            }
            else if (qualified1.IsEnclosing)
            {
                nbsol1 = 1;
                radius[1] = r1 + distcc1;
            }
            else if (qualified1.IsUnqualified)
            {
                nbsol1 = 2;
                radius[1] = Math.Abs(distcc1 - r1);
                radius[2] = r1 + distcc1;
            }
            if (qualified2.IsEnclosed || qualified2.IsOutside)
            {
                nbsol2 = 1;
                rradius[1] = Math.Abs(distcc2 - r2);
            }
            else if (qualified2.IsEnclosing)
            {
                nbsol2 = 1;
                rradius[1] = r2 + distcc2;
            }
            else if (qualified2.IsUnqualified)
            {
                nbsol2 = 2;
                rradius[1] = Math.Abs(distcc2 - r2);
                rradius[2] = r2 + distcc2;
            }
            for (int i = 1; i <= nbsol1; i++)
            {
                for (int j = 1; j <= nbsol2; j++)
                {
                    if (Math.Abs(radius[i] - rradius[j]) <= tol)
                    {
                        _wellDone = true;
                        _nbrSol++;
                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(pinterm, dirx), radius[i]);
                        //        ===========================================================
                        Direction2D dc1 = new(center1.XY() - pinterm.XY());
                        Direction2D dc2 = new(center2.XY() - pinterm.XY());
                        distcc1 = pinterm.Distance(center1);
                        distcc2 = pinterm.Distance(center2);
                        if (!qualified1.IsUnqualified)
                        {
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                        }
                        else if (Math.Abs(distcc1 + radius[i] - r1) < tol)
                        {
                            _qualifier1[_nbrSol] = Qualifier.Enclosed;
                        }
                        else if (Math.Abs(distcc1 - r1 - radius[i]) < tol)
                        {
                            _qualifier1[_nbrSol] = Qualifier.Outside;
                        }
                        else
                        {
                            _qualifier1[_nbrSol] = Qualifier.Enclosing;
                        }
                        if (!qualified2.IsUnqualified)
                        {
                            _qualifier2[_nbrSol] = qualified2.Qualifier();
                        }
                        else if (Math.Abs(distcc2 + radius[i] - r2) < tol)
                        {
                            _qualifier2[_nbrSol] = Qualifier.Enclosed;
                        }
                        else if (Math.Abs(distcc2 - r2 - radius[i]) < tol)
                        {
                            _qualifier2[_nbrSol] = Qualifier.Outside;
                        }
                        else
                        {
                            _qualifier2[_nbrSol] = Qualifier.Enclosing;
                        }
                        _pnttg1sol[_nbrSol] = new Point2D(pinterm.XY() + radius[i] * dc1.XY());
                        _pnttg2sol[_nbrSol] = new Point2D(pinterm.XY() + radius[i] * dc2.XY());
                        _pntcen[_nbrSol] = _cirsol[_nbrSol].Location();
                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[_nbrSol]);
                        _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                    }
                }
            }
            if (_wellDone)
            {
                return;
            }
        }

        //=========================================================================
        //   General case.                                                         +
        //=========================================================================

        Circ2dBisec bis = new(c1, c2);
        if (bis.IsDone)
        {
            NCollection_Array1<double> rbid = new(1, 2);
            NCollection_Array1<double> rBid = new(1, 2);
            int nbsolution = bis.SolutionCount;
            for (int i = 1; i <= nbsolution; i++)
            {
                ConstraintBisec sol = bis.ThisSolution(i);
                ConstraintCurveType typ = sol.ArcType();
                AnalyticIntersection intp = new();
                if (typ == ConstraintCurveType.Cir)
                {
                    intp.Perform(onCirc, sol.Circle());
                }
                else if (typ == ConstraintCurveType.Lin)
                {
                    intp.Perform(sol.Line(), onCirc);
                }
                else if (typ == ConstraintCurveType.Hpr)
                {
                    intp.Perform(onCirc, new Conic(sol.Hyperbola()));
                }
                else if (typ == ConstraintCurveType.Ell)
                {
                    intp.Perform(onCirc, new Conic(sol.Ellipse()));
                }
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty) && (!intp.ParallelElements()) && (!intp.IdenticalElements()))
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double dist1 = center.Distance(center1);
                            double dist2 = center.Distance(center2);
                            int nbsol = 0;
                            int nsol = 0;
                            int nnsol = 0;
                            r1 = c1.Radius();
                            r2 = c2.Radius();
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - r1 < tol)
                                {
                                    nbsol = 1;
                                    rbid[1] = Math.Abs(r1 - dist1);
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (r1 - dist1 < tol)
                                {
                                    nbsol = 1;
                                    rbid[1] = Math.Abs(dist1 - r1);
                                }
                            }
                            else if (qualified1.IsEnclosing)
                            {
                                nbsol = 1;
                                rbid[1] = dist1 + r1;
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                nbsol = 2;
                                rbid[1] = dist1 + r1;
                                // OCCT: Rbid(1) is written twice and Rbid(2) never (it keeps the value of an earlier
                                // point, or the array's initial value: uninitialised in C++, 0 in the port)
                                rbid[1] = Math.Abs(dist1 - r1);
                            }
                            if (qualified2.IsEnclosed && nbsol != 0)
                            {
                                if (dist2 - r2 < tol)
                                {
                                    nsol = 1;
                                    rBid[1] = Math.Abs(r2 - dist2);
                                }
                            }
                            else if (qualified2.IsOutside && nbsol != 0)
                            {
                                if (r2 - dist2 < tol)
                                {
                                    nsol = 1;
                                    rBid[1] = Math.Abs(r2 - dist2);
                                }
                            }
                            else if (qualified2.IsEnclosing && nbsol != 0)
                            {
                                nsol = 1;
                                rBid[1] = dist2 + r2;
                            }
                            else if (qualified2.IsUnqualified && nbsol != 0)
                            {
                                nsol = 2;
                                rBid[1] = dist2 + r2;
                                rBid[2] = Math.Abs(r2 - dist2);
                            }
                            for (int isol = 1; isol <= nbsol; isol++)
                            {
                                for (int jsol = 1; jsol <= nsol; jsol++)
                                {
                                    if (Math.Abs(rbid[isol] - rBid[jsol]) <= tol)
                                    {
                                        nnsol++;
                                        // OCCT: up to four matches for two elements (OCCT and the port throw past the end)
                                        radius[nnsol] = (rBid[jsol] + rbid[isol]) / 2.0;
                                    }
                                }
                            }
                            if (nnsol > 0)
                            {
                                for (int k = 1; k <= nnsol; k++)
                                {
                                    _nbrSol++;
                                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[k]);
                                    //              ==========================================================
                                    distcc1 = center.Distance(center1);
                                    distcc2 = center.Distance(center2);
                                    if (!qualified1.IsUnqualified)
                                    {
                                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                                    }
                                    else if (Math.Abs(distcc1 + radius[k] - r1) < tol)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    else if (Math.Abs(distcc1 - r1 - radius[k]) < tol)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                    }
                                    if (!qualified2.IsUnqualified)
                                    {
                                        _qualifier2[_nbrSol] = qualified2.Qualifier();
                                    }
                                    else if (Math.Abs(distcc2 + radius[k] - r2) < tol)
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    else if (Math.Abs(distcc2 - r2 - radius[k]) < tol)
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Enclosing;
                                    }
                                    if (center.Distance(center1) <= tolerance
                                        && Math.Abs(radius[k] - c1.Radius()) <= tolerance)
                                    {
                                        _theSame1[_nbrSol] = 1;
                                    }
                                    else
                                    {
                                        _theSame1[_nbrSol] = 0;
                                        Direction2D dc1 = new(center1.XY() - center.XY());
                                        _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc1.XY());
                                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                    }
                                    if (center.Distance(center2) <= tolerance
                                        && Math.Abs(radius[k] - c2.Radius()) <= tolerance)
                                    {
                                        _theSame2[_nbrSol] = 1;
                                    }
                                    else
                                    {
                                        _theSame2[_nbrSol] = 0;
                                        Direction2D dc2 = new(center2.XY() - center.XY());
                                        _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc2.XY());
                                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                        _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[_nbrSol]);
                                    }
                                    _pntcen[_nbrSol] = center;
                                    _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, _pntcen[_nbrSol]);
                                }
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }
}
