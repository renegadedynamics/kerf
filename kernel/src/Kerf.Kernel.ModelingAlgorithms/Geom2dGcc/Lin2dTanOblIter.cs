// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanOblIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanOblIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A 2D line tangent to a curve and making a given angle with a line, found by Newton iteration from a parameter on
/// the curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). At most one solution.</remarks>
internal sealed class Lin2dTanOblIter
{
    private readonly bool _wellDone;
    private readonly bool _paral2;
    private readonly Line2D _linsol = new();
    private readonly Qualifier _qualifier1;
    private readonly Point2D _pnttg1sol = new();
    private readonly Point2D _pntint2sol = new();
    private readonly double _par1sol;
    private readonly double _par2sol;
    private readonly double _pararg1;
    private readonly double _pararg2;

    /// <summary>
    /// The line tangent to the qualified curve at an angle Angle to TheLin, starting from the parameter Param1 on the
    /// curve. TolAng is the angular tolerance.
    /// </summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    public Lin2dTanOblIter(in QCurve qualified1, in Line2D theLin, double param1, double tolAng,
                                     double angle = 0)
    {
        _par2sol = 0.0;
        _pararg2 = 0.0;

        _par1sol  = 0.0;
        _pararg1  = 0.0;
        _wellDone = false;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        _paral2                  = false;
        CurveAdaptor2D cu1 = qualified1.Qualified();
        double              u1  = ConstraintCurveTool.FirstParameter(cu1);
        double              u2  = ConstraintCurveTool.LastParameter(cu1);
        Direction2D            dir = theLin.Direction;
        double              a   = dir.X;
        double              b   = dir.Y;
        Direction2D            theDirection = dir;
        if (Math.Abs(angle) > Math.Abs(tolAng))
        {
            if (Math.Abs(Math.Abs(angle) - M_PI) <= Math.Abs(tolAng))
            {
                _paral2       = true;
                theDirection = dir.Reversed();
            }
            else if (Math.Abs(angle - M_PI / 2) <= Math.Abs(tolAng))
            {
                theDirection = new Direction2D(-b, a);
            }
            else if (Math.Abs(angle + M_PI / 2) <= Math.Abs(tolAng))
            {
                theDirection = new Direction2D(b, -a);
            }
            else
            {
                theDirection = new Direction2D(a * Math.Cos(angle) - b * Math.Sin(angle),
                                            a * Math.Sin(angle) + b * Math.Cos(angle));
            }
        }
        else
        {
            _paral2 = true;
        }
        FunctionTanObl func = new(cu1, theDirection);
        FunctionRoot        sol  = new(func,
                                            param1,
                                            ConstraintCurveTool.EpsX(cu1, Math.Abs(tolAng)),
                                            u1,
                                            u2,
                                            100);
        if (sol.IsDone)
        {
            double usol = sol.Root();
            ConstraintCurveTool.D2(cu1, usol, out Point2D origine, out Vector2D vect1, out Vector2D vect2);
            double sign1 = vect1.XY().Dot(theDirection.XY());
            double sign2 = vect2.XY().Crossed(theDirection.XY());
            if (qualified1.IsUnqualified || (qualified1.IsEnclosing && sign2 <= 0.0)
                || (qualified1.IsOutside && sign1 <= 0.0 && sign2 >= 0.0)
                || (qualified1.IsEnclosed && sign1 >= 0.0 && sign2 >= 0.0))
            {
                _wellDone   = true;
                _linsol     = new Line2D(origine, theDirection);
                _pnttg1sol  = origine;
                _qualifier1 = qualified1.Qualifier();
                _pararg1    = usol;
                _par1sol    = 0.0;
                if (!_paral2)
                {
                    AnalyticIntersection intp = new(_linsol, theLin);
                    if (intp.IsDone && !intp.IsEmpty)
                    {
                        if (intp.PointCount == 1)
                        {
                            _pntint2sol = intp.Point(1).Value();
                            _par2sol    = new Vector2D(_linsol.Direction).Dot(new Vector2D(_linsol.Location, _pntint2sol));
                            _pararg2    = new Vector2D(theLin.Direction).Dot(new Vector2D(theLin.Location, _pntint2sol));
                        }
                    }
                }
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

    /// <summary>The solution.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    public Line2D ThisSolution()
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        return _linsol;
    }

    /// <summary>The qualifier of the curve for the solution.</summary>
    public void WhichQualifier(out Qualifier qualif1)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            qualif1 = _qualifier1;
        }
    }

    /// <summary>True if the solution is parallel to the line (the angle is 0 or pi).</summary>
    public bool IsParallel2
    {
        get
        {
            return _paral2;
        }
    }

    /// <summary>The tangency point between the solution and the curve, and its parameters.</summary>
    public void Tangency1(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = _par1sol;
            parArg = _pararg1;
            pntSol = _pnttg1sol;
        }
    }

    /// <summary>The intersection point between the solution and the line, and its parameters.</summary>
    /// <exception cref="IsParallelException">The solution is parallel to the line.</exception>
    public void Intersection2(out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (_paral2)
        {
            throw new IsParallelException();
        }
        else
        {
            pntSol = _pntint2sol;
            parSol = _par2sol;
            parArg = _pararg2;
        }
    }
}
