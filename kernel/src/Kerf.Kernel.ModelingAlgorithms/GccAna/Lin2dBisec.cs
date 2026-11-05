// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting lines of two 2D lines. Secant lines have two: the inside one, along D1 + D2, and the outside one,
/// along D1 - D2 (D1 and D2 the lines' directions). Parallel lines have one, equidistant from both; coincident lines
/// give the first line.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
internal sealed class Lin2dBisec
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Point2D> _pntint1sol;
    private readonly NCollection_Array1<Point2D> _pntint2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    //=========================================================================
    //   The first calculated bissectrice is the interior bisectrice, the     +
    //   second is the exterior bissectrice.                                  +
    //   the direction of the first bissectrice is such that its scalar product +
    //   with direction of Lin1 is always positive.             +
    //   The second bissectrice is turned in the positive direction.             +
    //=========================================================================

    /// <summary>Constructs the bisecting lines of Lin1 and Lin2.</summary>
    public Lin2dBisec(in Line2D lin1, in Line2D lin2)
    {
        _linsol = new NCollection_Array1<Line2D>(1, 2);
        _pntint1sol = new NCollection_Array1<Point2D>(1, 2);
        _pntint2sol = new NCollection_Array1<Point2D>(1, 2);
        _par1sol = new NCollection_Array1<double>(1, 2);
        _par2sol = new NCollection_Array1<double>(1, 2);
        _pararg1 = new NCollection_Array1<double>(1, 2);
        _pararg2 = new NCollection_Array1<double>(1, 2);

        _wellDone = false;
        _nbrSol = 0;

        AnalyticIntersection intp = new(lin1, lin2);
        if (intp.IsDone)
        {
            if (intp.ParallelElements())
            {
                if (intp.IdenticalElements())
                {
                    _nbrSol = 1;
                    _wellDone = true;
                    _linsol[_nbrSol] = lin1;
                }
                else
                {
                    // Attention : do not use dist = Lin1.Distance(Lin2);
                    // as straight lines can be concurrent for gp_Lin2d
                    // so dist = 0.0 (test of the angle too strict ?)
                    double dist = lin1.DistanceTo(lin2.Location) / 2.0;
                    double cross = new Vector2D(-lin2.Direction.Y, lin2.Direction.X)
                                       .Dot(new Vector2D(lin2.Location, lin1.Location));
                    if (cross < 0)
                    {
                        dist = -dist;
                    }
                    _nbrSol++;
                    _wellDone = true;
                    _linsol[_nbrSol] =
                        new Line2D(new Point2D(lin2.Location.XY()
                                                  + new Xy(-lin2.Direction.Y * dist, lin2.Direction.X * dist)),
                                     lin2.Direction);
                }
            }
            else
            {
                if (!intp.IsEmpty)
                {
                    for (int i = 1; i <= intp.PointCount; i++)
                    {
                        _nbrSol++;
                        _linsol[_nbrSol] = new Line2D(intp.Point(i).Value(),
                                                      new Direction2D(lin1.Direction.XY() + lin2.Direction.XY()));
                        _nbrSol++;
                        _linsol[_nbrSol] = new Line2D(intp.Point(i).Value(),
                                                      new Direction2D(lin1.Direction.XY() - lin2.Direction.XY()));
                        if (lin1.AngleTo(lin2) >= 0.0)
                        {
                            _linsol[_nbrSol] = _linsol[_nbrSol].Reversed();
                        }
                        _wellDone = true;
                    }
                }
            }
        }

        for (int i = 1; i <= _nbrSol; i++)
        {
            _pntint1sol[i] = _linsol[i].Location;
            _pntint2sol[i] = _pntint1sol[i];
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pntint1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_linsol[i], _pntint2sol[i]);
            _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin1, _pntint1sol[i]);
            _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2, _pntint2sol[i]);
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

    /// <summary>The number of bisecting lines.</summary>
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
    /// The solution number Index: the first is the inside bisector (along D1 + D2), the second the outside one (along
    /// D1 - D2).
    /// </summary>
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

    /// <summary>The intersection of the solution number Index with the first argument.</summary>
    public void Intersection1(int index, out double parSol, out double parArg, out Point2D pntSol)
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
            pntSol = _pntint1sol[index];
        }
    }

    /// <summary>The intersection of the solution number Index with the second argument.</summary>
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
