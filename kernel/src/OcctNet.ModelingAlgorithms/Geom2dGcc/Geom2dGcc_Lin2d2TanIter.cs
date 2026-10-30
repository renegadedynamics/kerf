// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2TanIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2TanIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// A 2D line tangent to two curves (or to a circle and a curve, or to a curve and through a point), found by Newton
/// iteration from parameters on the curves.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). At most one solution.</remarks>
public sealed class Geom2dGcc_Lin2d2TanIter
{
    private readonly bool WellDone;
    private readonly gp_Lin2d linsol = new();
    private readonly GccEnt_Position qualifier1;
    private readonly GccEnt_Position qualifier2;
    private readonly gp_Pnt2d pnttg1sol = new();
    private readonly gp_Pnt2d pnttg2sol = new();
    private readonly double par1sol;
    private readonly double par2sol;
    private readonly double pararg1;
    private readonly double pararg2;

    /// <summary>The line tangent to a qualified circle and a qualified curve, from the parameter Param2 on the curve.</summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed.</exception>
    public Geom2dGcc_Lin2d2TanIter(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, double Param2,
                                   double Tolang)
    {
        par1sol = 0.0;
        pararg1 = 0.0;
        par2sol = 0.0;
        pararg2 = 0.0;
        // double Tol = std::abs(Tolang);

        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (Qualified1.IsEnclosed())
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Circ2d                  C1   = Qualified1.Qualified();
        Geom2dAdaptor_Curve        Cu2  = Qualified2.Qualified();
        double                     U1   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        double                     U2   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Geom2dGcc_FunctionTanCirCu func = new(C1, Cu2);
        math_FunctionRoot          sol  = new(func,
                                              Param2,
                                              Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolang)),
                                              U1,
                                              U2,
                                              100);
        if (sol.IsDone())
        {
            double Usol = sol.Root();
            func.Value(Usol, out double Norm);
            if (Math.Abs(Norm) < Tolang)
            {
                Geom2dGcc_CurveTool.D2(Cu2, Usol, out gp_Pnt2d Origine, out gp_Vec2d Vect1, out gp_Vec2d Vect2);
                gp_Vec2d Vdir  = new(C1.Location().XY() - Origine.XY());
                double   sign1 = Vect1.Dot(Vdir);
                if (sign1 <= 0.0)
                {
                    Vect1.Reverse();
                }
                double sign2 = Vect2.Crossed(Vect1);
                if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && sign2 <= 0.0)
                    || (Qualified2.IsOutside() && sign1 <= 0.0 && sign2 >= 0.0)
                    || (Qualified2.IsEnclosed() && sign1 >= 0.0 && sign2 >= 0.0))
                {
                    if (Qualified1.IsUnqualified() || (Qualified1.IsOutside() && Vect1.Angle(Vdir) <= 0.0)
                        || (Qualified1.IsEnclosing() && Vect1.Angle(Vdir) >= 0.0))
                    {
                        gp_Dir2d direc  = new(Vect1);
                        double   R1     = C1.Radius();
                        gp_XY    normal = new(-R1 * direc.Y(), R1 * direc.X());
                        sign1           = Vect1.Crossed(Vdir);
                        if (Qualified1.IsEnclosing())
                        {
                            pnttg1sol = new gp_Pnt2d(C1.Location().XY() - normal);
                        }
                        else if (Qualified1.IsOutside())
                        {
                            pnttg1sol = new gp_Pnt2d(C1.Location().XY() + normal);
                        }
                        else
                        {
                            if (sign1 >= 0.0)
                            {
                                pnttg1sol = new gp_Pnt2d(C1.Location().XY() - normal);
                            }
                            else
                            {
                                pnttg1sol = new gp_Pnt2d(C1.Location().XY() + normal);
                            }
                        }
                        // 	 if (gp_Vec2d(direc.XY()).Angle(gp_Vec2d(pnttg1sol,Origine)) <= Tol) {
                        pnttg2sol  = Origine;
                        linsol     = new gp_Lin2d(pnttg1sol, direc);
                        WellDone   = true;
                        qualifier1 = Qualified1.Qualifier();
                        qualifier2 = Qualified2.Qualifier();
                        pararg2    = Usol;
                        par1sol    = 0.0;
                        par2sol    = pnttg2sol.Distance(pnttg1sol);
                        pararg1    = 0.0;
                    }
                }
            }
        }
    }

    /// <summary>The line tangent to two qualified curves, from the parameters Param1 and Param2 on them.</summary>
    /// <exception cref="GccEnt_BadQualifier">A qualifier is GccEnt_noqualifier.</exception>
    public Geom2dGcc_Lin2d2TanIter(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, double Param1,
                                   double Param2, double Tolang)
    {
        par1sol    = 0.0;
        pararg1    = 0.0;
        par2sol    = 0.0;
        pararg2    = 0.0;
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Geom2dAdaptor_Curve       Cu1    = Qualified1.Qualified();
        Geom2dAdaptor_Curve       Cu2    = Qualified2.Qualified();
        Geom2dGcc_FunctionTanCuCu Func   = new(Cu1, Cu2);
        math_Vector               Umin   = new(1, 2);
        math_Vector               Umax   = new(1, 2);
        math_Vector               Ufirst = new(1, 2);
        math_Vector               tol    = new(1, 2);
        Umin[1]   = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        Umin[2]   = Geom2dGcc_CurveTool.FirstParameter(Cu2);
        Umax[1]   = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Umax[2]   = Geom2dGcc_CurveTool.LastParameter(Cu2);
        Ufirst[1] = Param1;
        Ufirst[2] = Param2;
        tol[1]    = Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolang));
        tol[2]    = Geom2dGcc_CurveTool.EpsX(Cu2, Math.Abs(Tolang));
        math_FunctionSetRoot Root = new(Func, tol);
        Root.Perform(Func, Ufirst, Umin, Umax);
        if (Root.IsDone())
        {
            Root.Root(Ufirst);
            math_Vector Norm = new(1, 2);
            Func.Value(Ufirst, Norm);
            if (Math.Abs(Norm[1]) < Tolang && Math.Abs(Norm[2]) < Tolang)
            {
                Geom2dGcc_CurveTool.D2(Cu1, Ufirst[1], out gp_Pnt2d point1, out gp_Vec2d Vect11, out gp_Vec2d Vect12);
                Geom2dGcc_CurveTool.D2(Cu2, Ufirst[2], out gp_Pnt2d point2, out gp_Vec2d Vect21, out gp_Vec2d Vect22);
                gp_Vec2d Vec    = new(new gp_Pnt2d(point1.XY()), new gp_Pnt2d(point2.XY()));
                double   Angle1 = Vec.Angle(Vect12);
                double   sign1  = Vect11.Dot(Vec);
                if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && Angle1 >= 0.0)
                    || (Qualified1.IsOutside() && Angle1 <= 0.0 && sign1 <= 0.0)
                    || (Qualified1.IsEnclosed() && Angle1 <= 0.0 && sign1 >= 0.0))
                {
                    Angle1 = Vec.Angle(Vect22);
                    sign1  = Vect21.Dot(Vec);
                    if (Qualified2.IsUnqualified() || (Qualified2.IsEnclosing() && Angle1 >= 0.0)
                        || (Qualified2.IsOutside() && Angle1 <= 0.0 && sign1 <= 0.0)
                        || (Qualified2.IsEnclosed() && Angle1 <= 0.0 && sign1 >= 0.0))
                    {
                        qualifier1   = Qualified1.Qualifier();
                        qualifier2   = Qualified2.Qualifier();
                        pararg1      = Ufirst[1];
                        par1sol      = 0.0;
                        pnttg1sol    = point1;
                        pararg2      = Ufirst[2];
                        pnttg2sol    = point2;
                        par2sol      = pnttg2sol.Distance(pnttg1sol);
                        gp_Dir2d dir = new(pnttg2sol.X() - pnttg1sol.X(), pnttg2sol.Y() - pnttg1sol.Y());
                        linsol       = new gp_Lin2d(pnttg1sol, dir);
                        WellDone     = true;
                    }
                }
            }
        }
    }

    /// <summary>The line tangent to a qualified curve through a point, from the parameter Param1 on the curve.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    public Geom2dGcc_Lin2d2TanIter(in Geom2dGcc_QCurve Qualified1, in gp_Pnt2d ThePoint, double Param1, double Tolang)
    {
        par1sol    = 0.0;
        pararg1    = 0.0;
        par2sol    = 0.0;
        pararg2    = 0.0;
        WellDone   = false;
        qualifier1 = GccEnt_noqualifier;
        qualifier2 = GccEnt_noqualifier;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Geom2dAdaptor_Curve        Cu1  = Qualified1.Qualified();
        double                     U1   = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        double                     U2   = Geom2dGcc_CurveTool.LastParameter(Cu1);
        Geom2dGcc_FunctionTanCuPnt func = new(Cu1, ThePoint);
        math_FunctionRoot          sol  = new(func,
                                              Param1,
                                              Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(Tolang)),
                                              U1,
                                              U2,
                                              100);
        if (sol.IsDone())
        {
            double Usol = sol.Root();
            func.Value(Usol, out double Norm);
            if (Math.Abs(Norm) < Tolang)
            {
                Geom2dGcc_CurveTool.D2(Cu1, Usol, out gp_Pnt2d Origine, out gp_Vec2d Vect1, out gp_Vec2d Vect2);
                gp_Vec2d Vdir  = new(ThePoint.XY() - Origine.XY());
                double   sign1 = Vect1.Dot(Vdir);
                double   sign2 = Vect2.Crossed(Vdir);
                if (Qualified1.IsUnqualified()
                    || (Qualified1.IsEnclosing()
                        && ((sign1 >= 0.0 && sign2 <= 0.0) || (sign1 <= 0.0 && sign2 <= 0.0)))
                    || (Qualified1.IsOutside() && sign1 <= 0.0 && sign2 >= 0.0)
                    || (Qualified1.IsEnclosed() && sign1 >= 0.0 && sign2 >= 0.0))
                {
                    WellDone   = true;
                    linsol     = new gp_Lin2d(Origine, new gp_Dir2d(Vdir));
                    qualifier1 = Qualified1.Qualifier();
                    qualifier2 = GccEnt_noqualifier;
                    pnttg1sol  = Origine;
                    pnttg2sol  = ThePoint;
                    pararg1    = Usol;
                    par1sol    = 0.0;
                    pararg2    = ThePoint.Distance(Origine);
                    par2sol    = 0.0;
                }
            }
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The solution.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    public gp_Lin2d ThisSolution()
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        return linsol;
    }

    /// <summary>The qualifiers of the arguments for the solution.</summary>
    public void WhichQualifier(out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            Qualif1 = qualifier1;
            Qualif2 = qualifier2;
        }
    }

    /// <summary>The tangency point between the solution and the first argument, and its parameters.</summary>
    public void Tangency1(out double ParSol, out double ParArg, out gp_Pnt2d Pnt)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            ParSol = par1sol;
            ParArg = pararg1;
            Pnt    = pnttg1sol;
        }
    }

    /// <summary>The tangency point between the solution and the second argument, and its parameters.</summary>
    public void Tangency2(out double ParSol, out double ParArg, out gp_Pnt2d Pnt)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            ParSol = par2sol;
            ParArg = pararg2;
            Pnt    = pnttg2sol;
        }
    }
}
