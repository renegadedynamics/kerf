// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanCen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanCen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D circles with a given centre, tangent to a circle (two when unqualified, one when qualified), tangent to a line,
/// or through a point.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
internal sealed class AnalyticCirc2dTanCen
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;

    //========================================================================
    //   Initialization of fields.                                          +
    //========================================================================
    private AnalyticCirc2dTanCen(int size)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, size);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, size);
        _theSame1 = new NCollection_Array1<int>(1, size);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, size);
        _par1sol = new NCollection_Array1<double>(1, size);
        _pararg1 = new NCollection_Array1<double>(1, size);
    }

    //========================================================================
    //     Creation of a circle tangent to a circle with center in a point.      +
    //   - Calculate the distance between the center of the circle and the point of  +
    //     center : dist                                                     +
    //   - Check that this distance is compatible with the qualifier of the circle. +
    //       Si yes, the radius of the solution will be :                           +
    //          C1.Radius()-dist if the qualifier is Enclosed.              +
    //          C1.Radius()+dist if the qualifier is Enclosing.             +
    //          dist-C1.Radius() if the qualifier is Outside.               +
    //          a mix of these values if the qualifier is Unqualified.  +
    //========================================================================

    /// <summary>The circles centred on Pcenter and tangent to the qualified circle.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    public AnalyticCirc2dTanCen(in QualifiedCirc qualified1, in Point2D pcenter, double tolerance) : this(2)
    {
        _nbrSol = 0;
        double radius = 0.0;
        _wellDone = false;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        Circle2D c1 = qualified1.Qualified();
        double r1 = c1.Radius;
        Point2D center1 = c1.Location;
        double dist;
        int signe = 0;
        int signe1 = 0;

        if (!qualified1.IsUnqualified)
        {
            dist = pcenter.DistanceTo(center1);
            if (qualified1.IsEnclosed)
            {
                //   ============================
                if (dist - r1 <= tol)
                {
                    radius = Math.Abs(r1 - dist);
                    signe = 1;
                }
                else
                {
                    _wellDone = true;
                }
            }
            else if (qualified1.IsEnclosing)
            {
                //   =================================
                radius = r1 + dist;
                signe = -1;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                if (dist < r1 - tol)
                {
                    _wellDone = true;
                }
                else
                {
                    radius = Math.Abs(r1 - dist);
                    signe = -1;
                }
            }
            if (signe != 0)
            {
                _nbrSol++;
                _cirsol[_nbrSol] = new Circle2D(new Axis2D(pcenter, dirx), radius);
                //     ========================================================
                _qualifier1[_nbrSol] = qualified1.Qualifier();
                if (dist <= Constants.Resolution())
                {
                    _theSame1[_nbrSol] = 1;
                }
                else
                {
                    _theSame1[_nbrSol] = 0;
                    Direction2D d = new(pcenter.X - center1.X, pcenter.Y - center1.Y);
                    _pnttg1sol[_nbrSol] = new Point2D(pcenter.XY() + signe * radius * d.XY());
                    // a solution of radius 0 (Pcenter on C1) and a circle C1 of radius 0 put the tangency point at
                    // their centre
                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                }
                _wellDone = true;
            }
        }
        else
        {
            // ====
            dist = pcenter.DistanceTo(center1);
            if (dist >= Constants.Resolution())
            {
                signe = 1;
                for (int i = 1; i <= 2; i++)
                {
                    signe = -signe;
                    if (r1 - dist <= 0.0)
                    {
                        signe1 = -1;
                    }
                    else
                    {
                        signe1 = -signe;
                    }
                    radius = Math.Abs(r1 + signe * dist);
                    _nbrSol++;
                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(pcenter, dirx), radius);
                    //       ========================================================
                    double distcc1 = pcenter.DistanceTo(center1);
                    if (!qualified1.IsUnqualified)
                    {
                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                    }
                    else if (Math.Abs(distcc1 + radius - r1) < tol)
                    {
                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                    }
                    else if (Math.Abs(distcc1 - r1 - radius) < tol)
                    {
                        _qualifier1[_nbrSol] = Qualifier.Outside;
                    }
                    else
                    {
                        _qualifier1[_nbrSol] = Qualifier.Enclosing;
                    }
                    _theSame1[_nbrSol] = 0;
                    _wellDone = true;
                    Direction2D d = new(pcenter.X - center1.X, pcenter.Y - center1.Y);
                    _pnttg1sol[_nbrSol] = new Point2D(pcenter.XY() + signe1 * radius * d.XY());
                    // a solution of radius 0 (Pcenter on C1) and a circle C1 of radius 0 put the tangency point at
                    // their centre
                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                }
            }
            else
            {
                _nbrSol++;
                _cirsol[_nbrSol] = c1;
                //     ==============================
                _qualifier1[1] = qualified1.Qualifier();
                _theSame1[_nbrSol] = 1;
                _wellDone = true;
            }
        }
    }

    //=========================================================================
    //   Circle tangent to line Linetan and center in a point Pcenter.    +
    //   Calculate the distance from the point to the line ==> Radius.                +
    //   Create the circle with center Pcenter of radius Radius.                  +
    //=========================================================================

    /// <summary>The circle centred on Pcenter and tangent to Linetan.</summary>
    public AnalyticCirc2dTanCen(in Line2D linetan, in Point2D pcenter) : this(1)
    {
        Direction2D dirx = new(Direction2D.D.X);
        double rayon = linetan.DistanceTo(pcenter);
        _cirsol[1] = new Circle2D(new Axis2D(pcenter, dirx), rayon);
        // ==================================================
        _qualifier1[1] = Qualifier.Noqualifier;
        _theSame1[1] = 0;
        double xloc = linetan.Location.X;
        double yloc = linetan.Location.Y;
        double xdir = linetan.Direction.X;
        double ydir = linetan.Direction.Y;

        if (new Direction2D(xloc - pcenter.X, yloc - pcenter.Y).Dot(new Direction2D(-ydir, xdir)) > 0.0)
        {
            _pnttg1sol[1] = new Point2D(pcenter.XY() + rayon * new Xy(-ydir, xdir));
            _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
            _pararg1[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linetan, _pnttg1sol[1]);
        }
        else
        {
            _pnttg1sol[1] = new Point2D(pcenter.XY() + rayon * new Xy(ydir, -xdir));
            _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
            _pararg1[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linetan, _pnttg1sol[1]);
        }
        _nbrSol = 1;
        _wellDone = true;
    }

    //=========================================================================
    //   Circle tangent to point Point1 and centered in a point Pcenter.      +
    //   Calculate the distance from Pcenter to Point1 ==> Radius.                +
    //   Create the circle with center Pcenter of radius Radius.                  +
    //=========================================================================

    /// <summary>The circle centred on Pcenter through Point1.</summary>
    public AnalyticCirc2dTanCen(in Point2D point1, in Point2D pcenter) : this(1)
    {
        Direction2D dirx = new(Direction2D.D.X);
        double rayon = point1.DistanceTo(pcenter);
        _cirsol[1] = new Circle2D(new Axis2D(pcenter, dirx), rayon);
        // =================================================
        _qualifier1[1] = Qualifier.Noqualifier;
        _theSame1[1] = 0;
        _pnttg1sol[1] = point1;
        // coincident points give a circle of radius 0 through its centre
        _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[1], _pnttg1sol[1]);
        _pararg1[1] = 0.0;
        _nbrSol = 1;
        _wellDone = true;
    }

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

    /// <summary>The solution number Index (OCCT doesn't check IsDone here).</summary>
    public Circle2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>
    /// The qualifier of the argument for the solution number Index: the one given, the one computed for an unqualified
    /// circle, or GccEnt_noqualifier for a line or a point.
    /// </summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
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
        }
    }

    /// <summary>The tangency point of the solution number Index with the argument, and its parameters.</summary>
    /// <exception cref="NotDoneException">The solution is the argument itself (IsTheSame1).</exception>
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
                pntSol = _pnttg1sol[index];
                parSol = _par1sol[index];
                parArg = _pararg1[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>True if the solution number Index is the argument circle itself.</summary>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame1[index] == 0)
        {
            return false;
        }

        return true;
    }
}
