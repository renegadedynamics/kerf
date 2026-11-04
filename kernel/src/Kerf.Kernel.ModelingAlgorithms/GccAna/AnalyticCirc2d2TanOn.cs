// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
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
/// An algorithm object computed by its constructor (decision 117). The constructors find candidate centres on the
/// third curve as OCCT's do (where it meets the bisectors of the arguments), and keep the circles that touch both
/// arguments as asked (Kerf decision 1541).
/// </remarks>
internal sealed partial class AnalyticCirc2d2TanOn
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

    /// <summary>
    /// Adds a candidate circle if it touches both arguments as their qualifiers ask, within the tolerance, and isn't a
    /// solution already: with the qualifiers it has, the tangency points where it touches, and its centre with its
    /// parameter on the locus (Kerf decision 1541).
    /// </summary>
    private void AddSolution(in Circle2D sol, in CircleContact a1, in CircleContact a2, double parcen, double tolerance)
    {
        if (!a1.Touches(sol, tolerance, out Qualifier q1) || !a2.Touches(sol, tolerance, out Qualifier q2))
        {
            return;
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (_cirsol[i].Location.DistanceTo(sol.Location) <= tolerance && Math.Abs(_cirsol[i].Radius - sol.Radius) <= tolerance)
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
        _pntcen[n] = sol.Location;
        _parcen3[n] = parcen;
        _wellDone = true;
    }

    /// <summary>Adds the circles centred at a point of the locus whose radius one of the arguments allows.</summary>
    private void AddCandidates(in Point2D center, in CircleContact a1, in CircleContact a2, double parcen, double tolerance)
    {
        Span<double> radii = stackalloc double[4];
        int nb = a1.Radii(center, radii);
        nb += a2.Radii(center, radii[nb..]);
        Direction2D dirx = new(Direction2D.D.X);
        for (int i = 0; i < nb; i++)
        {
            if (radii[i] > tolerance)
            {
                AddSolution(new Circle2D(new Axis2D(center, dirx), radii[i]), a1, a2, parcen, tolerance);
            }
        }
    }

    /// <summary>The candidates centred at the points of an intersection with a line locus.</summary>
    private void AddCandidates(AnalyticIntersection intp, in Line2D onLine, in CircleContact a1, in CircleContact a2, double tolerance)
    {
        if (intp.IsDone && !intp.IsEmpty)
        {
            for (int j = 1; j <= intp.PointCount; j++)
            {
                Point2D center = intp.Point(j).Value();
                AddCandidates(center, a1, a2, global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, center), tolerance);
            }
        }
    }

    /// <summary>The candidates centred at the points of an intersection with a circle locus.</summary>
    private void AddCandidates(AnalyticIntersection intp, in Circle2D onCirc, in CircleContact a1, in CircleContact a2, double tolerance)
    {
        if (intp.IsDone && !intp.IsEmpty)
        {
            for (int j = 1; j <= intp.PointCount; j++)
            {
                Point2D center = intp.Point(j).Value();
                AddCandidates(center, a1, a2, ParameterOn(onCirc, center), tolerance);
            }
        }
    }

    /// <summary>The parameter of a centre on a circle locus (0 at the centre of a circle of radius 0).</summary>
    private static double ParameterOn(in Circle2D onCirc, in Point2D center)
        => center.DistanceTo(onCirc.Location) <= Constants.Resolution() ? 0.0 : global::Kerf.Engine.Geometry.ElCLib.Parameter(onCirc, center);

    /// <summary>The foot of a point on a line locus, a candidate centre of OCCT's boundary cases.</summary>
    private void AddFoot(in Point2D p, in Line2D onLine, in CircleContact a1, in CircleContact a2, double tolerance)
    {
        double u = global::Kerf.Engine.Geometry.ElCLib.Parameter(onLine, p);
        AddCandidates(global::Kerf.Engine.Geometry.ElCLib.Value(u, onLine), a1, a2, u, tolerance);
    }

    /// <summary>
    /// The two points of a circle locus on the line through its centre and a point, candidate centres of OCCT's
    /// boundary cases.
    /// </summary>
    private void AddAlong(in Point2D p, in Circle2D onCirc, in CircleContact a1, in CircleContact a2, double tolerance)
    {
        Xy d = p.XY() - onCirc.Location.XY();
        double m = d.Modulus();
        if (m <= tolerance)
        {
            return;
        }
        foreach (double sign in (ReadOnlySpan<double>)[1.0, -1.0])
        {
            Point2D center = new(onCirc.Location.XY() + (sign * onCirc.Radius / m) * d);
            AddCandidates(center, a1, a2, ParameterOn(onCirc, center), tolerance);
        }
    }

    //=========================================================================
    //  Circles tangent to two circles C1 and C2 and centered on a straight line.   +
    //  The centres are where the line meets the bisectors of the circles, or the  +
    //  line through their centres (OCCT's boundary cases); each is kept with the  +
    //  radii that touch both circles as asked (Kerf decision 1541).              +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a line.</summary>
    public AnalyticCirc2d2TanOn(in QualifiedCirc qualified1,
                               in QualifiedCirc qualified2,
                               in Line2D onLine,
                               double tolerance)
        : this(16)
    {
        _theSame1.Init(0);
        _theSame2.Init(0);
        _wellDone = false;
        _nbrSol = 0;
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
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        Point2D center1 = c1.Location;
        Point2D center2 = c2.Location;

        // OCCT's boundary cases: centres where the line meets the line of the circles' centres
        if (center1.DistanceTo(center2) > tol)
        {
            AddCandidates(new AnalyticIntersection(onLine, new Line2D(center1, new Direction2D(center2.XY() - center1.XY()))),
                          onLine, a1, a2, tol);
        }

        Circ2dBisec bis = new(c1, c2);
        if (bis.IsDone)
        {
            AnalyticIntersection intp = new();
            for (int i = 1; i <= bis.SolutionCount; i++)
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
                AddCandidates(intp, onLine, a1, a2, tol);
            }
            _wellDone = true;
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

    /// <summary>The number of solutions (0 when the construction failed, as for GccAna's other solvers).</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>The solution number Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbSolutions()] (OCCT checked neither: Kerf decision 1541).</exception>
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
    /// The centre of the solution number Index on the third argument: its parameter there, and the centre (OCCT returned
    /// the first tangency point: Kerf decision 1541).
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
            pntSol = _pntcen[index];
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
