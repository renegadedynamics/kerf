// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Engine.Construction.Constraints.ConstraintCurveType;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The 2D circles tangent to two curves, tangent to a curve and through a point, or through two points, with their
/// centre on a third curve. The curves are lines or circles.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Each constructor fills its arrays as OCCT's does,
/// including OCCT's mistakes (marked <c>// OCCT</c>): <see cref="CenterOn3"/> returns the first tangency point, not
/// the centre, as OCCT's does.
/// </remarks>
public sealed partial class AnalyticCirc2d2TanOn
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<Point2D> _pntcen;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;
    private readonly NCollection_Array1<double> _parcen3;

    // The result arrays, sized as each constructor's initialiser list does in OCCT.
    private AnalyticCirc2d2TanOn(int size)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, size);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, size);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, size);
        _theSame1 = new NCollection_Array1<int>(1, size);
        _theSame2 = new NCollection_Array1<int>(1, size);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, size);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, size);
        _pntcen = new NCollection_Array1<Point2D>(1, size);
        _par1sol = new NCollection_Array1<double>(1, size);
        _par2sol = new NCollection_Array1<double>(1, size);
        _pararg1 = new NCollection_Array1<double>(1, size);
        _pararg2 = new NCollection_Array1<double>(1, size);
        _parcen3 = new NCollection_Array1<double>(1, size);
    }

    //=========================================================================
    //  Circles tangent to two circles C1 and C2 and centered on a straight line.   +
    //  We start by distinguishing various boundary cases that will be processed separately. +
    //  In the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to two circles that give us     +
    //  all possible locations of centers of all circles tangent to C1 and C2. + We intersect these
    //  bissectrices with straight line which gives us  + points among which we are going to find
    //  solutions.   + The choices are made basing on Qualifiers of C1 and C2.  +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedCirc qualified2,
                               in Line2D onLine,
                               double tolerance)
        : this(4)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
        int nbsol = 0;
        double tol = Math.Abs(tolerance);
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        double r1 = c1.Radius;
        double r2 = c2.Radius;
        Direction2D dirx = new(Direction2D.D.X);
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;
        NCollection_Array1<double> radius = new(1, 2);

        double dist1 = onLine.DistanceTo(center1);
        double dist2 = onLine.DistanceTo(center2);
        double d1 = dist1 + r1;
        double d2 = dist2 + r2;
        double d3 = dist1 - r1;
        double d4 = dist2 - r2;

        //=========================================================================
        //  Processing of boundary cases.                                           +
        //=========================================================================

        // OCCT: up to four of these cases can hold together, but Radius has two elements: OCCT's range check throws
        // Standard_OutOfRange past its end, as the port does (decision 778).
        if (Math.Abs(d3 - d4) < tol
            && (qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            && (qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            nbsol++;
            radius[nbsol] = Math.Abs(d3);
            _wellDone = true;
        }
        if (Math.Abs(d1 - d2) < tol && (qualified1.IsEnclosing || qualified1.IsUnqualified)
            && (qualified2.IsEnclosing || qualified2.IsUnqualified))
        {
            nbsol++;
            radius[nbsol] = Math.Abs(d1);
            _wellDone = true;
        }
        if (Math.Abs(d1 - d4) < tol && (qualified1.IsEnclosing || qualified1.IsUnqualified)
            && (qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            nbsol++;
            radius[nbsol] = Math.Abs(d1);
            _wellDone = true;
        }
        if (Math.Abs(d3 - d2) < tol && (qualified2.IsEnclosing || qualified2.IsUnqualified)
            && (qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            nbsol++;
            radius[nbsol] = Math.Abs(d3);
            _wellDone = true;
        }
        Line2D l = new(center1, new Direction2D(center2.XY() - center1.XY()));
        AnalyticIntersection intp = new(onLine, l);
        if (intp.IsDone)
        {
            if (!intp.IsEmpty)
            {
                for (int j = 1; j <= intp.PointCount; j++)
                {
                    Point2D center = intp.Point(j).Value();
                    for (int i = 1; i <= nbsol; i++)
                    {
                        _nbrSol++;
                        _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius[i]);
                        //        ==========================================================
                        _wellDone = true;
                        double distcc1 = center.DistanceTo(center1);
                        double distcc2 = center.DistanceTo(center2);
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
                        Direction2D dc1 = new(center1.XY() - center.XY());
                        Direction2D dc2 = new(center2.XY() - center.XY());
                        _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[i] * dc1.XY());
                        _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius[i] * dc2.XY());
                        _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                        _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c2, _pnttg2sol[_nbrSol]);
                        _pntcen[_nbrSol] = _cirsol[_nbrSol].Location;
                        _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
                    }
                }
            }
        }

        //=========================================================================
        //   General case.                                                       +
        //=========================================================================

        if (!_wellDone)
        {
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

                    if (typ == ConstraintCurveType.Cir)
                    {
                        intp.Perform(onLine, sol.Circle());
                    }
                    else if (typ == ConstraintCurveType.Lin)
                    {
                        intp.Perform(onLine, sol.Line());
                    }
                    else if (typ == ConstraintCurveType.Hpr)
                    {
                        intp.Perform(onLine, new Conic(sol.Hyperbola()));
                    }
                    else if (typ == ConstraintCurveType.Ell)
                    {
                        intp.Perform(onLine, new Conic(sol.Ellipse()));
                    }
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int j = 1; j <= intp.PointCount; j++)
                            {
                                Point2D center = intp.Point(j).Value();
                                dist1 = center.DistanceTo(center1);
                                dist2 = center.DistanceTo(center2);
                                nbsol = 0;
                                int nsol = 0;
                                int nnsol = 0;
                                r1 = c1.Radius;
                                r2 = c2.Radius;
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
                                        //                ==========================================================
                                        double distcc1 = center.DistanceTo(center1);
                                        double distcc2 = center.DistanceTo(center2);
                                        // OCCT: the qualifiers of unqualified arguments are computed with Radius(i),
                                        // i the bisector's index, instead of Radius(k); past Radius's two elements
                                        // OCCT's range check throws Standard_OutOfRange, as the port does
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
                                        if (center.DistanceTo(center1) <= tol && Math.Abs(radius[k] - c1.Radius) <= tol)
                                        {
                                            _theSame1[_nbrSol] = 1;
                                        }
                                        else
                                        {
                                            _theSame1[_nbrSol] = 0;
                                            Direction2D dc1 = new(center1.XY() - center.XY());
                                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius[k] * dc1.XY());
                                            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                            // OCCT: the parameter on C1 is that of the second tangency point
                                            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg2sol[_nbrSol]);
                                        }
                                        if (center.DistanceTo(center2) <= tol && Math.Abs(radius[k] - c2.Radius) <= tol)
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
                                        _parcen3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, _pntcen[_nbrSol]);
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

    //========================================================================

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions (OCCT doesn't check IsDone here).</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>
    /// The solution number Index. As in OCCT, IsDone and NbSolutions are not checked; an index outside the result
    /// arrays throws Standard_OutOfRange.
    /// </summary>
    public Circle2D ThisSolution(int index)
    {
        return _cirsol[index];
    }

    /// <summary>
    /// The qualifiers of the arguments for the solution number Index: the ones given, or the ones computed for
    /// unqualified arguments.
    /// </summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
            qualif2 = _qualifier2[index];
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with the first argument: its parameter on the solution, on the
    /// argument, and the point. Throws StdFail_NotDone if the solution is the argument itself (IsTheSame1).
    /// </summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with the second argument: its parameter on the solution, on the
    /// argument, and the point. Throws StdFail_NotDone if the solution is the argument itself (IsTheSame2).
    /// </summary>
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame2[index] == 0)
            {
                parSol = _par2sol[index];
                parArg = _pararg2[index];
                pntSol = _pnttg2sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>
    /// The centre of the solution number Index on the third argument: its parameter there, and a point. As in OCCT,
    /// the point returned is the first tangency point, not the centre.
    /// </summary>
    public void CenterOn3(int index, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parArg = _parcen3[index];
            pntSol = _pnttg1sol[index]; // OCCT: pnttg1sol instead of pntcen
        }
    }

    /// <summary>True if the solution number Index is the first argument.</summary>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame1[index] == 0)
        {
            return false;
        }
        return true;
    }

    /// <summary>True if the solution number Index is the second argument.</summary>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame2[index] == 0)
        {
            return false;
        }
        return true;
    }
}
