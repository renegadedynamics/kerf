// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D circles of a given radius tangent to two arguments, each a qualified circle, a qualified line or a point
/// (passed through). Each solution comes with the qualifiers of its arguments (computed for unqualified ones) and its
/// tangency points.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Up to 8 solutions (two circles), 4 (a circle and a
/// line, two lines) or 2 (with a point).
/// </remarks>
internal sealed partial class AnalyticCirc2d2TanRad
{
    private bool _wellDone;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    /// <summary>
    /// Adds a candidate circle if it touches both arguments as their qualifiers ask, within the tolerance, and isn't a
    /// solution already, with the qualifiers it has and the tangency points where it touches (Kerf decision 1542).
    /// </summary>
    private void AddSolution(in Circle2D sol, in CircleContact a1, in CircleContact a2, double tolerance)
    {
        if (!a1.Touches(sol, tolerance, out Qualifier q1) || !a2.Touches(sol, tolerance, out Qualifier q2))
        {
            return;
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (_cirsol[i].Location.DistanceTo(sol.Location) <= tolerance)
            {
                return;
            }
        }
        if (_nbrSol == _cirsol.Upper())
        {
            return;
        }
        int n = ++_nbrSol;
        _cirsol[n] = sol;
        _qualifier1[n] = q1;
        _qualifier2[n] = q2;
        _theSame1[n] = a1.Touch(n, sol, q1, tolerance, _pnttg1sol, _par1sol, _pararg1);
        _theSame2[n] = a2.Touch(n, sol, q2, tolerance, _pnttg2sol, _par2sol, _pararg2);
    }

    //==================================================================
    // The centre of a circle of radius R tangent to a circle (c, r) is at  +
    // R + r or |R - r| from c: the solutions' centres are where those      +
    // circles about C1 and C2 meet, and, where they only touch, on the     +
    // line of the centres. Each candidate is kept if it touches both       +
    // circles as asked (Kerf decision 1542: OCCT's case analysis lost     +
    // solutions and returned circles tangent to one circle only).          +
    //==================================================================
    /// <summary>Circles of radius Radius tangent to two qualified circles.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedCirc qualified1, in QualifiedCirc qualified2, double radius, double tolerance)
    {
        _qualifier1 = new(1, 8);
        _qualifier2 = new(1, 8);
        _theSame1 = new(1, 8);
        _theSame2 = new(1, 8);
        _cirsol = new(1, 8);
        _pnttg1sol = new(1, 8);
        _pnttg2sol = new(1, 8);
        _par1sol = new(1, 8);
        _par2sol = new(1, 8);
        _pararg1 = new(1, 8);
        _pararg2 = new(1, 8);

        double tol = Math.Abs(tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        Circle2D c1 = qualified1.Qualified();
        Circle2D c2 = qualified2.Qualified();
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;
        _wellDone = true;
        if (center1.IsEqual(center2, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // concentric circles: no solution, or infinitely many
            return;
        }
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        ReadOnlySpan<double> rho1 = [radius + c1.Radius, Math.Abs(radius - c1.Radius)];
        ReadOnlySpan<double> rho2 = [radius + c2.Radius, Math.Abs(radius - c2.Radius)];
        Xy u = center2.XY() - center1.XY();
        double dist = u.Modulus();
        u /= dist;
        foreach (double d1 in rho1)
        {
            foreach (double d2 in rho2)
            {
                if (d1 <= tol)
                {
                    // the solution is centred at C1's centre
                    AddSolution(new Circle2D(new Axis2D(center1, dirx), radius), a1, a2, tol);
                    continue;
                }
                if (d2 <= tol)
                {
                    AddSolution(new Circle2D(new Axis2D(center2, dirx), radius), a1, a2, tol);
                    continue;
                }
                AnalyticIntersection intp = new(new Circle2D(new Axis2D(center1, dirx), d1), new Circle2D(new Axis2D(center2, dirx), d2));
                if (intp.IsDone && !intp.IsEmpty)
                {
                    for (int j = 1; j <= intp.PointCount; j++)
                    {
                        AddSolution(new Circle2D(new Axis2D(intp.Point(j).Value(), dirx), radius), a1, a2, tol);
                    }
                }
                // where the two circles only touch, on the line of the centres
                AddSolution(new Circle2D(new Axis2D(new Point2D(center1.XY() + d1 * u), dirx), radius), a1, a2, tol);
                AddSolution(new Circle2D(new Axis2D(new Point2D(center1.XY() - d1 * u), dirx), radius), a1, a2, tol);
            }
        }
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
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        qualif1 = _qualifier1[index];
        qualif2 = _qualifier2[index];
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

    /// <summary>The tangency point of the solution number Index with the second argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
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

    /// <summary>True if the solution number Index is the second argument.</summary>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
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
