// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPar.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPar.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D lines parallel to a line and passing through a point (one solution) or tangent to a circle (two when
/// unqualified, one when enclosing or outside).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_Lin2dTanPar
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<Line2D> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;

    //========================================================================
    //   Passing by a point :                                              +
    //   Create the straight line with origin ThePoint and                             +
    //                          direction Lin1.Direction().                    +
    //========================================================================

    /// <summary>The line through ThePoint parallel to Lin1.</summary>
    public GccAna_Lin2dTanPar(in Point2D ThePoint, in Line2D Lin1)
    {
        linsol = new NCollection_Array1<Line2D>(1, 1);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 1);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 1);
        par1sol = new NCollection_Array1<double>(1, 1);
        pararg1 = new NCollection_Array1<double>(1, 1);

        linsol[1] = new Line2D(ThePoint, Lin1.Direction());
        // ===============================================
        qualifier1[1] = GccEnt_noqualifier;
        pnttg1sol[1] = ThePoint;
        par1sol[1] = 0.0;
        pararg1[1] = 0.0;
        NbrSol = 1;
        WellDone = true;
    }

    //========================================================================
    //   Tangent to a circle :                                               +
    //   Create following the qualifier the straight line                    +
    //          - with origin P1 (P1 is a point of intersection between C1 and +
    //                       a straight line passing by the center of C1 and  +
    //                         direction the normal to Lin1).                 +
    //                         the choice of the point of intersection depends +
    //                         on the qualifier.                            +
    //          - with direction the direction of Lin1.                         +
    //========================================================================

    /// <summary>The lines tangent to the qualified circle and parallel to Lin1.</summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed.</exception>
    public GccAna_Lin2dTanPar(in GccEnt_QualifiedCirc Qualified1, in Line2D Lin1)
    {
        linsol = new NCollection_Array1<Line2D>(1, 2);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 2);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 2);
        par1sol = new NCollection_Array1<double>(1, 2);
        pararg1 = new NCollection_Array1<double>(1, 2);

        WellDone = false;
        int signe = 0;
        int nbsol = 0;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D C1 = Qualified1.Qualified();
        double xdir = (Lin1.Direction()).X();
        double ydir = (Lin1.Direction()).Y();

        if (Qualified1.IsEnclosed())
        {
            throw new GccEnt_BadQualifier();
        }
        // ============================
        else if (Qualified1.IsEnclosing())
        {
            // ==================================
            nbsol = 1;
            signe = 1;
            qualifier1[1] = GccEnt_enclosing;
        }
        else if (Qualified1.IsOutside())
        {
            // ===================================
            nbsol = 1;
            signe = -1;
            qualifier1[1] = GccEnt_outside;
        }
        else
        {
            nbsol = 2;
            signe = -1;
            qualifier1[1] = GccEnt_outside;
            qualifier1[2] = GccEnt_enclosing;
        }
        Xy xy = new(-C1.Radius() * ydir, C1.Radius() * xdir);
        for (int j = 1; j <= nbsol; j++)
        {
            signe = -signe;
            NbrSol++;
            linsol[NbrSol] = new Line2D(new Point2D((C1.Location().XY()).Added(signe * xy)),
                                          //   =========================================================================
                                          Lin1.Direction());
            //                             =================
            pnttg1sol[NbrSol] = new Point2D((C1.Location().XY()).Added(signe * xy));
            par1sol[NbrSol] = 0.0;
            // a circle of radius 0 puts the tangency point at its centre
            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
            WellDone = true;
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        return NbrSol;
    }

    /// <summary>The solution number Index.</summary>
    public Line2D ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        return linsol[Index];
    }

    /// <summary>
    /// The qualifier of the argument for the solution number Index: the one given, the one computed for an unqualified
    /// circle (enclosing or outside), or GccEnt_noqualifier for a point.
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

    /// <summary>The tangency point of the solution number Index with the argument, and its parameters on both.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out Point2D Pnt)
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
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
            Pnt = pnttg1sol[Index];
        }
    }
}
