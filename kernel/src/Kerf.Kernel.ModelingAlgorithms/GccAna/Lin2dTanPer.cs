// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPer.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D lines through a point or tangent to a circle, and perpendicular to a line or to a circle. The origin of each
/// solution is its tangency point with the first argument.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). As in OCCT, the qualifier of a point argument, and
/// of a circle in the circle/line case, is never set: WhichQualifier returns GccEnt_unqualified there (uninitialised
/// memory in OCCT).
/// </remarks>
public sealed class Lin2dTanPer
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pntint2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    private Lin2dTanPer(int size)
    {
        _linsol = new NCollection_Array1<Line2D>(1, size);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, size);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, size);
        _pntint2sol = new NCollection_Array1<Point2D>(1, size);
        _par1sol = new NCollection_Array1<double>(1, size);
        _par2sol = new NCollection_Array1<double>(1, size);
        _pararg1 = new NCollection_Array1<double>(1, size);
        _pararg2 = new NCollection_Array1<double>(1, size);
    }

    //=========================================================================
    //   Straight line passing by point  : ThePoint and                       +
    //       orthogonal to straight line : TheLin.                            +
    //   Create the straight line of origin     : ThePoint                           +
    //                  and direction  : TheLin.Direction() turned by 90   +
    //=========================================================================

    /// <summary>The line through ThePnt perpendicular to TheLin.</summary>
    public Lin2dTanPer(in Point2D thePnt, in Line2D theLin) : this(1)
    {
        _linsol[1] = new Line2D(thePnt,
                                 new Direction2D(-(theLin.Direction.Y),
                                              // ===============================================================
                                              theLin.Direction.X));
        //                                      ========================
        _pnttg1sol[1] = thePnt;
        AnalyticIntersection intp = new(_linsol[1], theLin);
        if (intp.IsDone)
        {
            if (!intp.IsEmpty)
            {
                for (int i = 1; i <= intp.PointCount; i++)
                {
                    _pntint2sol[1] = intp.Point(i).Value();
                }
            }
        }
        _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[1], _pnttg1sol[1]);
        _par2sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[1], _pntint2sol[1]);
        _pararg1[1] = 0.0;
        _pararg2[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, _pntint2sol[1]);
        _nbrSol = 1;
        _wellDone = true;
    }

    //=========================================================================
    //   Straight line passing by point      : ThePnt                             +
    //       and orthogonal to circle        : TheCircle.                         +
    //   Create the straight line of origin  : ThePoint                           +
    //                  and direction        : (TheCircle.Location(),ThePnt).     +
    //=========================================================================

    /// <summary>The line through ThePnt and the centre of TheCircle (perpendicular to the circle).</summary>
    public Lin2dTanPer(in Point2D thePnt, in Circle2D theCircle) : this(1)
    {
        _linsol[1] = new Line2D(thePnt,
                                 // ============================
                                 new Direction2D(theCircle.Location.XY() - thePnt.XY()));
        //                      ================================================
        _pnttg1sol[1] = thePnt;
        AnalyticIntersection intp = new(_linsol[1], theCircle);
        if (intp.IsDone)
        {
            if (!intp.IsEmpty)
            {
                double maxdist = RealLast();
                for (int i = 1; i <= intp.PointCount; i++)
                {
                    // OCCT: maxdist is never lowered, so this keeps the last intersection point, not the nearest
                    if (intp.Point(i).Value().DistanceTo(thePnt) < maxdist)
                    {
                        _pntint2sol[1] = intp.Point(i).Value();
                    }
                }
            }
        }
        _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[1], _pnttg1sol[1]);
        _par2sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[1], _pntint2sol[1]);
        _pararg1[1] = 0.0;
        // a circle of radius 0 puts the intersection at its centre
        _pararg2[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCircle, _pntint2sol[1]);
        _nbrSol = 1;
        _wellDone = true;
    }

    //=========================================================================
    //   Straight line tangent to circle     : Qualified1 (C1)                    +
    //   and orthogonal to straight line     : TheLin.                            +
    //   Create straight line of origin      : P1 (on C1)                         +
    //                  and direction        : TheLin.Direction() turned by 90`   +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle and perpendicular to TheLin.</summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed.</exception>
    public Lin2dTanPer(in QualifiedCirc qualified1, in Line2D theLin) : this(2)
    {
        _wellDone = false;
        int nbsol = 0;
        int signe = 0;
        _nbrSol = 0;
        Circle2D c1 = qualified1.Qualified();

        // OCCT: qualifier1 is never set by this constructor
        if (qualified1.IsEnclosed)
        {
            // ============================
            throw new BadQualifierException();
        }
        else if (qualified1.IsEnclosing)
        {
            // ==================================
            nbsol = 1;
            signe = -1;
        }
        else if (qualified1.IsOutside)
        {
            // ================================
            nbsol = 1;
            signe = 1;
        }
        else
        {
            nbsol = 2;
            signe = -1;
        }
        Xy xy = c1.Radius * theLin.Direction.XY();
        for (int j = 1; j <= nbsol; j++)
        {
            signe = -signe;
            _nbrSol++;
            _linsol[_nbrSol] = new Line2D(new Point2D((c1.Location.XY()).Added(signe * xy)),
                                          //   =======================================================================
                                          new Direction2D(-theLin.Direction.Y,
                                                       //                           =================================
                                                       theLin.Direction.X));
            //                                    ========================
            _pnttg1sol[_nbrSol] = new Point2D((c1.Location.XY()).Added(signe * xy));
            AnalyticIntersection intp = new(_linsol[_nbrSol], theLin);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    for (int i = 1; i <= intp.PointCount; i++)
                    {
                        _pntint2sol[_nbrSol] = intp.Point(i).Value();
                    }
                }
            }
            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pnttg1sol[_nbrSol]);
            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pntint2sol[_nbrSol]);
            // a circle of radius 0 puts the tangency point at its centre
            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
            _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, _pntint2sol[_nbrSol]);
            _wellDone = true;
        }
    }

    //=========================================================================
    //   Straight line tangent to circle     : Qualified1 (C1)                    +
    //       and orthogonal to circle        : TheCircle.                         +
    //   Create straight line of origin      : P1 (on C1)                         +
    //                  and direction        : TheLin.Direction() turned by 90`   +
    //=========================================================================

    /// <summary>
    /// Lines tangent to the qualified circle, at its points on the line of centres, so perpendicular to that line
    /// (as OCCT computes them: they don't pass through the centre of TheCircle).
    /// </summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed.</exception>
    public Lin2dTanPer(in QualifiedCirc qualified1, in Circle2D theCircle) : this(2)
    {
        _wellDone = false;
        _nbrSol = 0;
        int signe = 0;
        Circle2D c1 = qualified1.Qualified();

        if (qualified1.IsEnclosed)
        {
            // ============================
            throw new BadQualifierException();
        }
        else if (qualified1.IsEnclosing)
        {
            // ==================================
            signe = -1;
            _qualifier1[1] = Qualifier.Enclosing;
        }
        else if (qualified1.IsOutside)
        {
            // ================================
            signe = 1;
            _qualifier1[1] = Qualifier.Outside;
        }
        else if (qualified1.IsUnqualified)
        {
            // ====================================
            signe = -1;
            _qualifier1[1] = Qualifier.Enclosing;
            _qualifier1[2] = Qualifier.Outside;
        }
        // OCCT: always two solutions, whatever the qualifier (qualifier1(2) is then unset), and the lines are
        // perpendicular to the line of centres instead of passing through TheCircle's centre
        for (int j = 1; j <= 2; j++)
        {
            _nbrSol++;
            signe = -signe;
            Direction2D d1 = new(theCircle.Location.XY() - c1.Location.XY());
            _linsol[_nbrSol] = new Line2D(new Point2D((c1.Location.XY())
                                                       //   ===================================================
                                                       + signe * (d1.XY() * c1.Radius)),
                                          new Direction2D(-d1.Y, d1.X));
            //                     ======================================================
            _pnttg1sol[_nbrSol] = new Point2D((c1.Location.XY()) + signe * (d1.XY() * c1.Radius));
            AnalyticIntersection intp = new(_linsol[_nbrSol], theCircle);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    double maxdist = RealLast();
                    for (int i = 1; i <= intp.PointCount; i++)
                    {
                        // OCCT: maxdist is never lowered, so this keeps the last intersection point
                        if (intp.Point(i).Value().DistanceTo(_pnttg1sol[_nbrSol]) < maxdist)
                        {
                            _pntint2sol[_nbrSol] = intp.Point(i).Value();
                        }
                    }
                }
            }
            _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pnttg1sol[_nbrSol]);
            _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[_nbrSol], _pntint2sol[_nbrSol]);
            // a circle of radius 0 puts the tangency point at its centre, and a line missing TheCircle leaves the
            // intersection at the origin, which may be TheCircle's centre
            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
            _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCircle, _pntint2sol[_nbrSol]);
            _wellDone = true;
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            if (!_wellDone)
            {
                throw new NotDoneException();
            }
            return _nbrSol;
        }
    }

    /// <summary>
    /// The qualifier of the first argument for the solution number Index (see the remarks for the cases OCCT leaves
    /// unset).
    /// </summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
        }
    }

    /// <summary>The solution number Index.</summary>
    /// <exception cref="RangeErrorException">Index is out of range (OCCT throws this base class here).</exception>
    public Line2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new RangeErrorException();
        }
        return _linsol[index];
    }

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pnt)
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
            parSol = _par1sol[index];
            parArg = _pararg1[index];
            pnt = _pnttg1sol[index];
        }
    }

    /// <summary>The intersection of the solution number Index with the second argument, and its parameters.</summary>
    public void Intersection2(int index, out double parSol, out double parArg, out Point2D pntSol)
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
            parSol = _par2sol[index];
            parArg = _pararg2[index];
            pntSol = _pntint2sol[index];
        }
    }
}
