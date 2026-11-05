// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanObl.cxx
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
/// 2D lines through a point or tangent to a circle, making an angle (in radians) with a line. The origin of each
/// solution is its tangency point with the first argument.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
internal sealed class AnalyticLin2dTanObl
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
    private readonly bool[] _parallel;

    private AnalyticLin2dTanObl(int size)
    {
        _parallel = new bool[size + 1];
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
    //   Creation of a straight line passing by a point : ThePoint                +
    //                         making an angle     : TheAngle                +
    //                         with straight line  : TheLine.                +
    //   Subject the straight line (ThePoint,TheLine.Location()) to a rotation +
    //   by angle TheAngle ==> D1.                                            +
    //   create straight line passing through ThePoint of direction D1.              +
    //=========================================================================

    /// <summary>The line through ThePoint making the angle TheAngle with TheLine.</summary>
    public AnalyticLin2dTanObl(in Point2D thePoint, in Line2D theLine, double theAngle) : this(1)
    {
        double cosa = theLine.Direction.X;
        double sina = theLine.Direction.Y;
        _linsol[1] = new Line2D(thePoint,
                                 // ==============================
                                 new Direction2D(cosa * Math.Cos(theAngle) - sina * Math.Sin(theAngle),
                                              //                      ===============================================
                                              sina * Math.Cos(theAngle) + Math.Sin(theAngle) * cosa));
        //                               =======================================
        _qualifier1[1] = Qualifier.Noqualifier;
        _pnttg1sol[1] = thePoint;
        _par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[1], _pnttg1sol[1]);
        _pararg1[1] = 0.0;
        _nbrSol = 1;
        _wellDone = Intersect(1, theLine);
    }

    /// <summary>
    /// The intersection of solution n with TheLine; a solution parallel to it has none, and Intersection2 throws
    /// IsParallel for it (OCCT gave it the data of the default point, the origin: Kerf decision 1544). False if
    /// IntAna2d fails.
    /// </summary>
    private bool Intersect(int n, in Line2D theLine)
    {
        AnalyticIntersection intp = new(_linsol[n], theLine);
        if (!intp.IsDone)
        {
            return false;
        }
        _parallel[n] = intp.IsEmpty || intp.PointCount == 0;
        if (!_parallel[n])
        {
            _pntint2sol[n] = intp.Point(1).Value();
            _par2sol[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[n], _pntint2sol[n]);
            _pararg2[n] = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLine, _pntint2sol[n]);
        }
        return true;
    }

    //=========================================================================
    //   Creation of a straight line tangent to a circle  : Qualified1 (C1)         +
    //                         making angle           : TheAngle                +
    //                         with a straight line   : TheLine.                +
    //   Subject the straight line  (C1.Location,TheLine.Location()) to a       +
    //   rotation by angle TheAngle or -TheAngle ==> D1.                       +
    //   create the straight line passing by C1 of direction D1.                    +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle making the angle TheAngle with TheLine.</summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed.</exception>
    public AnalyticLin2dTanObl(in QualifiedCirc qualified1, in Line2D theLine, double theAngle) : this(2)
    {
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double cosa = theLine.Direction.X;
        double sina = theLine.Direction.Y;
        if (qualified1.IsEnclosed)
        {
            // ============================
            throw new BadQualifierException();
        }
        else
        {
            Circle2D c1 = qualified1.Qualified();
            double r1 = c1.Radius;
            if (qualified1.IsEnclosing)
            {
                //   =============================
                Xy xy = new(Math.Cos(theAngle) * cosa - Math.Sin(theAngle) * sina,
                               Math.Cos(theAngle) * sina + Math.Sin(theAngle) * cosa);
                _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * new Xy(xy.Y(), -xy.X()));
                _linsol[1] = new Line2D(_pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                _qualifier1[1] = qualified1.Qualifier();
                _nbrSol = 1;
                _wellDone = true;
            }
            else if (qualified1.IsOutside)
            {
                //   ================================
                Xy xy = new(Math.Cos(theAngle) * cosa - Math.Sin(theAngle) * sina,
                               Math.Cos(theAngle) * sina + Math.Sin(theAngle) * cosa);
                _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * new Xy(-xy.Y(), xy.X()));
                _linsol[1] = new Line2D(_pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                _qualifier1[1] = qualified1.Qualifier();
                _wellDone = true;
                _nbrSol = 1;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ====================================
                Xy xy = new(Math.Cos(theAngle) * cosa - Math.Sin(theAngle) * sina,
                               Math.Cos(theAngle) * sina + Math.Sin(theAngle) * cosa);
                _pnttg1sol[1] = new Point2D(c1.Location.XY() + r1 * new Xy(xy.Y(), -xy.X()));
                _linsol[1] = new Line2D(_pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                _qualifier1[1] = Qualifier.Enclosing;
                _wellDone = true;
                _nbrSol = 1;
                _pnttg1sol[2] = new Point2D(c1.Location.XY() + r1 * new Xy(-xy.Y(), xy.X()));
                _linsol[2] = new Line2D(_pnttg1sol[2], new Direction2D(xy));
                //     ===============================================
                _qualifier1[2] = Qualifier.Outside;
                _nbrSol++;
                _wellDone = true;
            }

            for (int index = 1; index <= _nbrSol; index++)
            {
                _par1sol[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[index], _pnttg1sol[index]);
                // a circle of radius 0 puts the tangency point at its centre
                _pararg1[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[index]);
                // each solution's own intersection (OCCT intersected the first solution twice: Kerf decision 1544)
                Intersect(index, theLine);
            }
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

    /// <summary>The solution number Index.</summary>
    public Line2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _linsol[index];
    }

    /// <summary>
    /// The qualifier of the first argument for the solution number Index: the one given, the one computed for an
    /// unqualified circle (enclosing or outside), or GccEnt_noqualifier for a point.
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

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
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

    /// <summary>The intersection of the solution number Index with the line, and its parameters on both.</summary>
    /// <exception cref="IsParallelException">The solution is parallel to the line (Kerf decision 1544).</exception>
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
        else if (_parallel[index])
        {
            throw new IsParallelException();
        }
        else
        {
            parSol = _par2sol[index];
            parArg = _pararg2[index];
            pntSol = _pntint2sol[index];
        }
    }
}
