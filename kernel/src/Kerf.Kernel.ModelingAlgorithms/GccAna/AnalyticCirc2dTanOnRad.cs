// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D circles of a given radius tangent to a qualified circle or line, or passing through a point, with their centre
/// on a line or a circle.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 4 solutions.</remarks>
public sealed partial class AnalyticCirc2dTanOnRad
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pntcen3;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _parcen3;

    // PRO12736 : bug quand OnLine // Ox, JCT 20/03/98
    //========================================================================
    //       circular tangent to element of type :      - Circle.            +
    //                                                  - Line.             +
    //                                                  - Point.             +
    //              center on second element of type :  - Circle.  +
    //                                                  - Line.   +
    //              of given radius : Radius.                             +
    //========================================================================



    //=========================================================================
    //  Circle tangent  :  to circle Qualified1 (C1).                         +
    //         center   :  on straight line OnLine.                           +
    //         of radius :  Radius.                                           +
    //                                                                        +
    //  Initialise the table of solutions cirsol and all fields.              +
    //  Eliminate depending on the qualifier the cases not being solutions.   +
    //  Solve the equation of the second degree indicating that the found center +
    //  point (xc,yc) is at a distance Radius from circle C1 and              +
    //                     on straight line OnLine.                           +
    //  The solutions aret represented by circles :                           +
    //                   - with center Pntcen(xc,yc)                          +
    //                   - with radius Radius.                                +
    //=========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2dTanOnRad(in QualifiedCirc qualified1, in Line2D onLine, double radius, double tolerance)
        : this(8)
    {
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Solve(new CircleContact(qualified1), qualified1.Qualified(), null, null, onLine, null, radius, Math.Abs(tolerance));
    }

    // The result arrays.
    private AnalyticCirc2dTanOnRad(int size)
    {
        _cirsol = new(1, size);
        _qualifier1 = new(1, size);
        _theSame1 = new(1, size);
        _pnttg1sol = new(1, size);
        _pntcen3 = new(1, size);
        _par1sol = new(1, size);
        _pararg1 = new(1, size);
        _parcen3 = new(1, size);
    }

    /// <summary>
    /// The circles of radius Radius centred on the line or circle that touch the argument as asked (Kerf decision 17104):
    /// their centres are where the locus meets the curves at Radius from the argument (two parallels of a line, the
    /// circles of radii Radius + r and |Radius - r| about a circle's centre, the circle of radius Radius about a point),
    /// and, where those only touch the locus, the locus's points nearest the argument's centre. OCCT's per-case code
    /// misplaced tangency points and centres, and threw on every generic line and circle.
    /// </summary>
    private void Solve(in CircleContact a, Circle2D? circle, Line2D? line, Point2D? point, Line2D? onLine, Circle2D? onCirc,
                       double radius, double tol)
    {
        _wellDone = true;
        _nbrSol = 0;
        if (radius <= tol)
        {
            return;
        }
        Direction2D dirx = new(Direction2D.D.X);
        if (line is Line2D l)
        {
            Xy n = new(-l.Direction.Y, l.Direction.X);
            foreach (double sign in (ReadOnlySpan<double>)[1.0, -1.0])
            {
                Line2D parallel = new(new Point2D(l.Location.XY() + (sign * radius) * n), l.Direction);
                AnalyticIntersection intp = onLine is Line2D ol ? new(ol, parallel) : new(parallel, onCirc!.Value);
                AddCenters(intp, a, onLine, onCirc, radius, tol);
            }
            return;
        }
        Point2D center = circle is Circle2D c ? c.Location : point!.Value;
        ReadOnlySpan<double> offsets = circle is Circle2D cc ? [radius + cc.Radius, Math.Abs(radius - cc.Radius)] : [radius];
        foreach (double d in offsets)
        {
            if (d <= tol)
            {
                // the argument's centre itself
                AddCenter(center, a, onLine, onCirc, radius, tol);
                continue;
            }
            Circle2D offset = new(new Axis2D(center, dirx), d);
            AnalyticIntersection intp = onLine is Line2D ol ? new(ol, offset) : new(offset, onCirc!.Value);
            AddCenters(intp, a, onLine, onCirc, radius, tol);
        }
        // where the offset circles only touch the locus
        if (onLine is Line2D foot)
        {
            AddCenter(global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(foot, center), foot),
                      a, onLine, onCirc, radius, tol);
        }
        else
        {
            Circle2D on = onCirc!.Value;
            Xy u = center.XY() - on.Location.XY();
            double m = u.Modulus();
            if (m > tol)
            {
                AddCenter(new Point2D(on.Location.XY() + (on.Radius / m) * u), a, onLine, onCirc, radius, tol);
                AddCenter(new Point2D(on.Location.XY() - (on.Radius / m) * u), a, onLine, onCirc, radius, tol);
            }
        }
    }

    private void AddCenters(AnalyticIntersection intp, in CircleContact a, Line2D? onLine, Circle2D? onCirc, double radius, double tol)
    {
        if (intp.IsDone && !intp.IsEmpty)
        {
            for (int j = 1; j <= intp.PointCount; j++)
            {
                AddCenter(intp.Point(j).Value(), a, onLine, onCirc, radius, tol);
            }
        }
    }

    /// <summary>Adds the circle of radius Radius centred at a point if it is on the locus, touches the argument as asked and is new.</summary>
    private void AddCenter(in Point2D center, in CircleContact a, Line2D? onLine, Circle2D? onCirc, double radius, double tol)
    {
        double off = onLine is Line2D l ? l.DistanceTo(center) : Math.Abs(center.DistanceTo(onCirc!.Value.Location) - onCirc.Value.Radius);
        Circle2D sol = new(new Axis2D(center, new Direction2D(Direction2D.D.X)), radius);
        if (off > tol || !a.Touches(sol, tol, out Qualifier q))
        {
            return;
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (_cirsol[i].Location.DistanceTo(center) <= tol)
            {
                return;
            }
        }
        if (_nbrSol == _cirsol.Upper())
        {
            return;
        }
        int k = ++_nbrSol;
        _cirsol[k] = sol;
        _qualifier1[k] = q;
        _theSame1[k] = a.Touch(k, sol, q, tol, _pnttg1sol, _par1sol, _pararg1);
        _pntcen3[k] = center;
        _parcen3[k] = onLine is Line2D ol ? global::Kerf.Engine.Geometry.ElCLib.Parameter(ol, center)
                      : center.DistanceTo(onCirc!.Value.Location) <= Constants.Resolution() ? 0.0
                      : global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc!.Value, center);
    }

    /// <summary>True if the construction succeeded (even with no solution).</summary>
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
            return _nbrSol;
        }
    }

    /// <summary>The solution number Index.</summary>
    public Circle2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
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

    /// <summary>The tangency point of the solution number Index with the first argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
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
            parSol = _par1sol[index];
            parArg = _pararg1[index];
            pntSol = _pnttg1sol[index];
        }
    }

    /// <summary>The centre of the solution number Index on the On-curve: its parameter there, and the point.</summary>
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
            pntSol = _pntcen3[index];
        }
    }

    /// <summary>True if the solution number Index is the first argument.</summary>
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
