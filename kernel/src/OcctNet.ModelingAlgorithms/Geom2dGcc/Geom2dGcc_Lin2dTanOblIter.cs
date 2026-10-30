// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanOblIter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanOblIter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A 2D line tangent to a curve and making a given angle with a line, found by Newton iteration from a parameter on
/// the curve.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). At most one solution.</remarks>
public sealed class Geom2dGcc_Lin2dTanOblIter
{
    private readonly bool WellDone;
    private readonly bool Paral2;
    private readonly gp_Lin2d linsol = new();
    private readonly GccEnt_Position qualifier1;
    private readonly gp_Pnt2d pnttg1sol = new();
    private readonly gp_Pnt2d pntint2sol = new();
    private readonly double par1sol;
    private readonly double par2sol;
    private readonly double pararg1;
    private readonly double pararg2;

    /// <summary>
    /// The line tangent to the qualified curve at an angle Angle to TheLin, starting from the parameter Param1 on the
    /// curve. TolAng is the angular tolerance.
    /// </summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    public Geom2dGcc_Lin2dTanOblIter(in Geom2dGcc_QCurve Qualified1, in gp_Lin2d TheLin, double Param1, double TolAng,
                                     double Angle = 0)
    {
        par2sol = 0.0;
        pararg2 = 0.0;

        par1sol  = 0.0;
        pararg1  = 0.0;
        WellDone = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Paral2                  = false;
        Geom2dAdaptor_Curve Cu1 = Qualified1.Qualified();
        double              U1  = Geom2dGcc_CurveTool.FirstParameter(Cu1);
        double              U2  = Geom2dGcc_CurveTool.LastParameter(Cu1);
        gp_Dir2d            Dir = TheLin.Direction();
        double              A   = Dir.X();
        double              B   = Dir.Y();
        gp_Dir2d            TheDirection = Dir;
        if (Math.Abs(Angle) > Math.Abs(TolAng))
        {
            if (Math.Abs(Math.Abs(Angle) - M_PI) <= Math.Abs(TolAng))
            {
                Paral2       = true;
                TheDirection = Dir.Reversed();
            }
            else if (Math.Abs(Angle - M_PI / 2) <= Math.Abs(TolAng))
            {
                TheDirection = new gp_Dir2d(-B, A);
            }
            else if (Math.Abs(Angle + M_PI / 2) <= Math.Abs(TolAng))
            {
                TheDirection = new gp_Dir2d(B, -A);
            }
            else
            {
                TheDirection = new gp_Dir2d(A * Math.Cos(Angle) - B * Math.Sin(Angle),
                                            A * Math.Sin(Angle) + B * Math.Cos(Angle));
            }
        }
        else
        {
            Paral2 = true;
        }
        Geom2dGcc_FunctionTanObl func = new(Cu1, TheDirection);
        math_FunctionRoot        sol  = new(func,
                                            Param1,
                                            Geom2dGcc_CurveTool.EpsX(Cu1, Math.Abs(TolAng)),
                                            U1,
                                            U2,
                                            100);
        if (sol.IsDone())
        {
            double Usol = sol.Root();
            Geom2dGcc_CurveTool.D2(Cu1, Usol, out gp_Pnt2d Origine, out gp_Vec2d Vect1, out gp_Vec2d Vect2);
            double sign1 = Vect1.XY().Dot(TheDirection.XY());
            double sign2 = Vect2.XY().Crossed(TheDirection.XY());
            if (Qualified1.IsUnqualified() || (Qualified1.IsEnclosing() && sign2 <= 0.0)
                || (Qualified1.IsOutside() && sign1 <= 0.0 && sign2 >= 0.0)
                || (Qualified1.IsEnclosed() && sign1 >= 0.0 && sign2 >= 0.0))
            {
                WellDone   = true;
                linsol     = new gp_Lin2d(Origine, TheDirection);
                pnttg1sol  = Origine;
                qualifier1 = Qualified1.Qualifier();
                pararg1    = Usol;
                par1sol    = 0.0;
                if (!Paral2)
                {
                    IntAna2d_AnaIntersection Intp = new(linsol, TheLin);
                    if (Intp.IsDone() && !Intp.IsEmpty())
                    {
                        if (Intp.NbPoints() == 1)
                        {
                            pntint2sol = Intp.Point(1).Value();
                            par2sol    = new gp_Vec2d(linsol.Direction()).Dot(new gp_Vec2d(linsol.Location(), pntint2sol));
                            pararg2    = new gp_Vec2d(TheLin.Direction()).Dot(new gp_Vec2d(TheLin.Location(), pntint2sol));
                        }
                    }
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

    /// <summary>The qualifier of the curve for the solution.</summary>
    public void WhichQualifier(out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            Qualif1 = qualifier1;
        }
    }

    /// <summary>True if the solution is parallel to the line (the angle is 0 or pi).</summary>
    public bool IsParallel2()
    {
        return Paral2;
    }

    /// <summary>The tangency point between the solution and the curve, and its parameters.</summary>
    public void Tangency1(out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else
        {
            ParSol = par1sol;
            ParArg = pararg1;
            PntSol = pnttg1sol;
        }
    }

    /// <summary>The intersection point between the solution and the line, and its parameters.</summary>
    /// <exception cref="Geom2dGcc_IsParallel">The solution is parallel to the line.</exception>
    public void Intersection2(out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Paral2)
        {
            throw new Geom2dGcc_IsParallel();
        }
        else
        {
            PntSol = pntint2sol;
            ParSol = par2sol;
            ParArg = pararg2;
        }
    }
}
