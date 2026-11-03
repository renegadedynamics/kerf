// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanCen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanCen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D circles with a given centre, tangent to a circle (two when unqualified, one when qualified), tangent to a line,
/// or through a point.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_Circ2dTanCen
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;

    //========================================================================
    //   Initialization of fields.                                          +
    //========================================================================
    private GccAna_Circ2dTanCen(int theSize)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, theSize);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        TheSame1 = new NCollection_Array1<int>(1, theSize);
        pnttg1sol = new NCollection_Array1<Point2D>(1, theSize);
        par1sol = new NCollection_Array1<double>(1, theSize);
        pararg1 = new NCollection_Array1<double>(1, theSize);
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
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    public GccAna_Circ2dTanCen(in GccEnt_QualifiedCirc Qualified1, in Point2D Pcenter, double Tolerance) : this(2)
    {
        NbrSol = 0;
        double Radius = 0.0;
        WellDone = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);
        Circle2D C1 = Qualified1.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();
        double dist;
        int signe = 0;
        int signe1 = 0;

        if (!Qualified1.IsUnqualified())
        {
            dist = Pcenter.Distance(center1);
            if (Qualified1.IsEnclosed())
            {
                //   ============================
                if (dist - R1 <= Tol)
                {
                    Radius = Math.Abs(R1 - dist);
                    signe = 1;
                }
                else
                {
                    WellDone = true;
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //   =================================
                Radius = R1 + dist;
                signe = -1;
            }
            else if (Qualified1.IsOutside())
            {
                //   ===============================
                if (dist < R1 - Tol)
                {
                    WellDone = true;
                }
                else
                {
                    Radius = Math.Abs(R1 - dist);
                    signe = -1;
                }
            }
            if (signe != 0)
            {
                NbrSol++;
                cirsol[NbrSol] = new Circle2D(new Axis2D(Pcenter, dirx), Radius);
                //     ========================================================
                qualifier1[NbrSol] = Qualified1.Qualifier();
                if (dist <= Constants.Resolution())
                {
                    TheSame1[NbrSol] = 1;
                }
                else
                {
                    TheSame1[NbrSol] = 0;
                    Direction2D d = new(Pcenter.X() - center1.X(), Pcenter.Y() - center1.Y());
                    pnttg1sol[NbrSol] = new Point2D(Pcenter.XY() + signe * Radius * d.XY());
                    // a solution of radius 0 (Pcenter on C1) and a circle C1 of radius 0 put the tangency point at
                    // their centre
                    par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                    pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                }
                WellDone = true;
            }
        }
        else
        {
            // ====
            dist = Pcenter.Distance(center1);
            if (dist >= Constants.Resolution())
            {
                signe = 1;
                for (int i = 1; i <= 2; i++)
                {
                    signe = -signe;
                    if (R1 - dist <= 0.0)
                    {
                        signe1 = -1;
                    }
                    else
                    {
                        signe1 = -signe;
                    }
                    Radius = Math.Abs(R1 + signe * dist);
                    NbrSol++;
                    cirsol[NbrSol] = new Circle2D(new Axis2D(Pcenter, dirx), Radius);
                    //       ========================================================
                    double distcc1 = Pcenter.Distance(center1);
                    if (!Qualified1.IsUnqualified())
                    {
                        qualifier1[NbrSol] = Qualified1.Qualifier();
                    }
                    else if (Math.Abs(distcc1 + Radius - R1) < Tol)
                    {
                        qualifier1[NbrSol] = GccEnt_enclosed;
                    }
                    else if (Math.Abs(distcc1 - R1 - Radius) < Tol)
                    {
                        qualifier1[NbrSol] = GccEnt_outside;
                    }
                    else
                    {
                        qualifier1[NbrSol] = GccEnt_enclosing;
                    }
                    TheSame1[NbrSol] = 0;
                    WellDone = true;
                    Direction2D d = new(Pcenter.X() - center1.X(), Pcenter.Y() - center1.Y());
                    pnttg1sol[NbrSol] = new Point2D(Pcenter.XY() + signe1 * Radius * d.XY());
                    // a solution of radius 0 (Pcenter on C1) and a circle C1 of radius 0 put the tangency point at
                    // their centre
                    par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                    pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                }
            }
            else
            {
                NbrSol++;
                cirsol[NbrSol] = C1;
                //     ==============================
                qualifier1[1] = Qualified1.Qualifier();
                TheSame1[NbrSol] = 1;
                WellDone = true;
            }
        }
    }

    //=========================================================================
    //   Circle tangent to line Linetan and center in a point Pcenter.    +
    //   Calculate the distance from the point to the line ==> Radius.                +
    //   Create the circle with center Pcenter of radius Radius.                  +
    //=========================================================================

    /// <summary>The circle centred on Pcenter and tangent to Linetan.</summary>
    public GccAna_Circ2dTanCen(in Line2D Linetan, in Point2D Pcenter) : this(1)
    {
        Direction2D dirx = new(Direction2D.D.X);
        double rayon = Linetan.Distance(Pcenter);
        cirsol[1] = new Circle2D(new Axis2D(Pcenter, dirx), rayon);
        // ==================================================
        qualifier1[1] = GccEnt_noqualifier;
        TheSame1[1] = 0;
        double xloc = Linetan.Location().X();
        double yloc = Linetan.Location().Y();
        double xdir = Linetan.Direction().X();
        double ydir = Linetan.Direction().Y();

        if (new Direction2D(xloc - Pcenter.X(), yloc - Pcenter.Y()).Dot(new Direction2D(-ydir, xdir)) > 0.0)
        {
            pnttg1sol[1] = new Point2D(Pcenter.XY() + rayon * new Xy(-ydir, xdir));
            par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
            pararg1[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(Linetan, pnttg1sol[1]);
        }
        else
        {
            pnttg1sol[1] = new Point2D(Pcenter.XY() + rayon * new Xy(ydir, -xdir));
            par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
            pararg1[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(Linetan, pnttg1sol[1]);
        }
        NbrSol = 1;
        WellDone = true;
    }

    //=========================================================================
    //   Circle tangent to point Point1 and centered in a point Pcenter.      +
    //   Calculate the distance from Pcenter to Point1 ==> Radius.                +
    //   Create the circle with center Pcenter of radius Radius.                  +
    //=========================================================================

    /// <summary>The circle centred on Pcenter through Point1.</summary>
    public GccAna_Circ2dTanCen(in Point2D Point1, in Point2D Pcenter) : this(1)
    {
        Direction2D dirx = new(Direction2D.D.X);
        double rayon = Point1.Distance(Pcenter);
        cirsol[1] = new Circle2D(new Axis2D(Pcenter, dirx), rayon);
        // =================================================
        qualifier1[1] = GccEnt_noqualifier;
        TheSame1[1] = 0;
        pnttg1sol[1] = Point1;
        // coincident points give a circle of radius 0 through its centre
        par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
        pararg1[1] = 0.0;
        NbrSol = 1;
        WellDone = true;
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions (OCCT doesn't check IsDone here).</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>The solution number Index (OCCT doesn't check IsDone here).</summary>
    public Circle2D ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new OutOfRangeException();
        }
        return cirsol[Index];
    }

    /// <summary>
    /// The qualifier of the argument for the solution number Index: the one given, the one computed for an unqualified
    /// circle, or GccEnt_noqualifier for a line or a point.
    /// </summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            Qualif1 = qualifier1[Index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the argument, and its parameters.</summary>
    /// <exception cref="NotDoneException">The solution is the argument itself (IsTheSame1).</exception>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (TheSame1[Index] == 0)
            {
                PntSol = pnttg1sol[Index];
                ParSol = par1sol[Index];
                ParArg = pararg1[Index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>True if the solution number Index is the argument circle itself.</summary>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }

        if (TheSame1[Index] == 0)
        {
            return false;
        }

        return true;
    }
}
