// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2TanIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2TanIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A 2D line tangent to two curves (or to a circle and a curve, or to a curve and through a point), found by Newton
/// iteration from parameters on the curves.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). At most one solution.</remarks>
internal sealed class Lin2d2TanIter
{
    private readonly bool _wellDone;
    private readonly Line2D _linsol = new();
    private readonly Qualifier _qualifier1;
    private readonly Qualifier _qualifier2;
    private readonly Point2D _pnttg1sol = new();
    private readonly Point2D _pnttg2sol = new();
    private readonly double _par1sol;
    private readonly double _par2sol;
    private readonly double _pararg1;
    private readonly double _pararg2;

    /// <summary>The line tangent to a qualified circle and a qualified curve, from the parameter Param2 on the curve.</summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed.</exception>
    public Lin2d2TanIter(in QualifiedCirc qualified1, in QCurve qualified2, double param2,
                                   double tolang)
    {
        _par1sol = 0.0;
        _pararg1 = 0.0;
        _par2sol = 0.0;
        _pararg2 = 0.0;
        // double Tol = std::abs(Tolang);

        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (qualified1.IsEnclosed)
        {
            throw new BadQualifierException();
        }
        Circle2D                  c1   = qualified1.Qualified();
        CurveAdaptor2D        cu2  = qualified2.Qualified();
        double                     u1   = ConstraintCurveTool.FirstParameter(cu2);
        double                     u2   = ConstraintCurveTool.LastParameter(cu2);
        FunctionTanCirCu func = new(c1, cu2);
        FunctionRoot          sol  = new(func,
                                              param2,
                                              ConstraintCurveTool.EpsX(cu2, Math.Abs(tolang)),
                                              u1,
                                              u2,
                                              100);
        if (sol.IsDone)
        {
            double usol = sol.Root();
            func.Value(usol, out double norm);
            if (Math.Abs(norm) < tolang)
            {
                ConstraintCurveTool.D2(cu2, usol, out Point2D origine, out Vector2D vect1, out Vector2D vect2);
                Vector2D vdir  = new(c1.Location.XY() - origine.XY());
                double   sign1 = vect1.Dot(vdir);
                if (sign1 <= 0.0)
                {
                    vect1 = vect1.Reversed();
                }
                double sign2 = vect2.Crossed(vect1);
                if (qualified2.IsUnqualified || (qualified2.IsEnclosing && sign2 <= 0.0)
                    || (qualified2.IsOutside && sign1 <= 0.0 && sign2 >= 0.0)
                    || (qualified2.IsEnclosed && sign1 >= 0.0 && sign2 >= 0.0))
                {
                    if (qualified1.IsUnqualified || (qualified1.IsOutside && vect1.AngleTo(vdir) <= 0.0)
                        || (qualified1.IsEnclosing && vect1.AngleTo(vdir) >= 0.0))
                    {
                        Direction2D direc  = new(vect1);
                        double   r1     = c1.Radius;
                        Xy    normal = new(-r1 * direc.Y, r1 * direc.X);
                        sign1           = vect1.Crossed(vdir);
                        if (qualified1.IsEnclosing)
                        {
                            _pnttg1sol = new Point2D(c1.Location.XY() - normal);
                        }
                        else if (qualified1.IsOutside)
                        {
                            _pnttg1sol = new Point2D(c1.Location.XY() + normal);
                        }
                        else
                        {
                            if (sign1 >= 0.0)
                            {
                                _pnttg1sol = new Point2D(c1.Location.XY() - normal);
                            }
                            else
                            {
                                _pnttg1sol = new Point2D(c1.Location.XY() + normal);
                            }
                        }
                        // 	 if (gp_Vec2d(direc.XY()).Angle(gp_Vec2d(pnttg1sol,Origine)) <= Tol) {
                        _pnttg2sol  = origine;
                        _linsol     = new Line2D(_pnttg1sol, direc);
                        _wellDone   = true;
                        _qualifier1 = qualified1.Qualifier();
                        _qualifier2 = qualified2.Qualifier();
                        _pararg2    = usol;
                        _par1sol    = 0.0;
                        _par2sol    = _pnttg2sol.DistanceTo(_pnttg1sol);
                        _pararg1    = 0.0;
                    }
                }
            }
        }
    }

    /// <summary>The line tangent to two qualified curves, from the parameters Param1 and Param2 on them.</summary>
    /// <exception cref="BadQualifierException">A qualifier is GccEnt_noqualifier.</exception>
    public Lin2d2TanIter(in QCurve qualified1, in QCurve qualified2, double param1,
                                   double param2, double tolang)
    {
        _par1sol    = 0.0;
        _pararg1    = 0.0;
        _par2sol    = 0.0;
        _pararg2    = 0.0;
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        CurveAdaptor2D       cu1    = qualified1.Qualified();
        CurveAdaptor2D       cu2    = qualified2.Qualified();
        FunctionTanCuCu func   = new(cu1, cu2);
        math_Vector               umin   = new(1, 2);
        math_Vector               umax   = new(1, 2);
        math_Vector               ufirst = new(1, 2);
        math_Vector               tol    = new(1, 2);
        umin[1]   = ConstraintCurveTool.FirstParameter(cu1);
        umin[2]   = ConstraintCurveTool.FirstParameter(cu2);
        umax[1]   = ConstraintCurveTool.LastParameter(cu1);
        umax[2]   = ConstraintCurveTool.LastParameter(cu2);
        ufirst[1] = param1;
        ufirst[2] = param2;
        tol[1]    = ConstraintCurveTool.EpsX(cu1, Math.Abs(tolang));
        tol[2]    = ConstraintCurveTool.EpsX(cu2, Math.Abs(tolang));
        FunctionSetRoot root = new(func, tol);
        root.Perform(func, ufirst, umin, umax);
        if (root.IsDone)
        {
            root.Root(ufirst);
            math_Vector norm = new(1, 2);
            func.Value(ufirst, norm);
            if (Math.Abs(norm[1]) < tolang && Math.Abs(norm[2]) < tolang)
            {
                ConstraintCurveTool.D2(cu1, ufirst[1], out Point2D point1, out Vector2D vect11, out Vector2D vect12);
                ConstraintCurveTool.D2(cu2, ufirst[2], out Point2D point2, out Vector2D vect21, out Vector2D vect22);
                Vector2D vec    = new(new Point2D(point1.XY()), new Point2D(point2.XY()));
                double   angle1 = vec.AngleTo(vect12);
                double   sign1  = vect11.Dot(vec);
                if (qualified1.IsUnqualified || (qualified1.IsEnclosing && angle1 >= 0.0)
                    || (qualified1.IsOutside && angle1 <= 0.0 && sign1 <= 0.0)
                    || (qualified1.IsEnclosed && angle1 <= 0.0 && sign1 >= 0.0))
                {
                    angle1 = vec.AngleTo(vect22);
                    sign1  = vect21.Dot(vec);
                    if (qualified2.IsUnqualified || (qualified2.IsEnclosing && angle1 >= 0.0)
                        || (qualified2.IsOutside && angle1 <= 0.0 && sign1 <= 0.0)
                        || (qualified2.IsEnclosed && angle1 <= 0.0 && sign1 >= 0.0))
                    {
                        _qualifier1   = qualified1.Qualifier();
                        _qualifier2   = qualified2.Qualifier();
                        _pararg1      = ufirst[1];
                        _par1sol      = 0.0;
                        _pnttg1sol    = point1;
                        _pararg2      = ufirst[2];
                        _pnttg2sol    = point2;
                        _par2sol      = _pnttg2sol.DistanceTo(_pnttg1sol);
                        Direction2D dir = new(_pnttg2sol.X - _pnttg1sol.X, _pnttg2sol.Y - _pnttg1sol.Y);
                        _linsol       = new Line2D(_pnttg1sol, dir);
                        _wellDone     = true;
                    }
                }
            }
        }
    }

    /// <summary>The line tangent to a qualified curve through a point, from the parameter Param1 on the curve.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    public Lin2d2TanIter(in QCurve qualified1, in Point2D thePoint, double param1, double tolang)
    {
        _par1sol    = 0.0;
        _pararg1    = 0.0;
        _par2sol    = 0.0;
        _pararg2    = 0.0;
        _wellDone   = false;
        _qualifier1 = Qualifier.Noqualifier;
        _qualifier2 = Qualifier.Noqualifier;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        CurveAdaptor2D        cu1  = qualified1.Qualified();
        double                     u1   = ConstraintCurveTool.FirstParameter(cu1);
        double                     u2   = ConstraintCurveTool.LastParameter(cu1);
        FunctionTanCuPnt func = new(cu1, thePoint);
        FunctionRoot          sol  = new(func,
                                              param1,
                                              ConstraintCurveTool.EpsX(cu1, Math.Abs(tolang)),
                                              u1,
                                              u2,
                                              100);
        if (sol.IsDone)
        {
            double usol = sol.Root();
            func.Value(usol, out double norm);
            if (Math.Abs(norm) < tolang)
            {
                ConstraintCurveTool.D2(cu1, usol, out Point2D origine, out Vector2D vect1, out Vector2D vect2);
                Vector2D vdir  = new(thePoint.XY() - origine.XY());
                double   sign1 = vect1.Dot(vdir);
                double   sign2 = vect2.Crossed(vdir);
                if (qualified1.IsUnqualified
                    || (qualified1.IsEnclosing
                        && ((sign1 >= 0.0 && sign2 <= 0.0) || (sign1 <= 0.0 && sign2 <= 0.0)))
                    || (qualified1.IsOutside && sign1 <= 0.0 && sign2 >= 0.0)
                    || (qualified1.IsEnclosed && sign1 >= 0.0 && sign2 >= 0.0))
                {
                    _wellDone   = true;
                    _linsol     = new Line2D(origine, new Direction2D(vdir));
                    _qualifier1 = qualified1.Qualifier();
                    _qualifier2 = Qualifier.Noqualifier;
                    _pnttg1sol  = origine;
                    _pnttg2sol  = thePoint;
                    _pararg1    = usol;
                    _par1sol    = 0.0;
                    _pararg2    = thePoint.DistanceTo(origine);
                    _par2sol    = 0.0;
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

    /// <summary>The qualifiers of the arguments for the solution.</summary>
    public void WhichQualifier(out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            qualif1 = _qualifier1;
            qualif2 = _qualifier2;
        }
    }

    /// <summary>The tangency point between the solution and the first argument, and its parameters.</summary>
    public void Tangency1(out double parSol, out double parArg, out Point2D pnt)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = _par1sol;
            parArg = _pararg1;
            pnt    = _pnttg1sol;
        }
    }

    /// <summary>The tangency point between the solution and the second argument, and its parameters.</summary>
    public void Tangency2(out double parSol, out double parArg, out Point2D pnt)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else
        {
            parSol = _par2sol;
            parArg = _pararg2;
            pnt    = _pnttg2sol;
        }
    }
}
