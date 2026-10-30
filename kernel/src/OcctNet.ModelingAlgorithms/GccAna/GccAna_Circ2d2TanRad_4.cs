// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanRad
{
    // circular tangent to two lines of given radius
    //===============================================
    //========================================================================
    // Initialize WellDone to false.                                       +
    // Return two lines L1 and L2.                                 +
    // Leave with error if the construction is impossible.     +
    // Create parallel lines to L1 and L2 in the proper direction.                   +
    // Intersect parallels ==> The center point of the solution.   +
    // Create the solution to be added to the already found solutions.         +
    // Fill the fields.                                                +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to two qualified lines. The tolerance is not used.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedLin Qualified1, in GccEnt_QualifiedLin Qualified2, double Radius, double Tolerance)
    {
        qualifier1 = new(1, 4);
        qualifier2 = new(1, 4);
        TheSame1 = new(1, 4);
        TheSame2 = new(1, 4);
        cirsol = new(1, 4);
        pnttg1sol = new(1, 4);
        pnttg2sol = new(1, 4);
        par1sol = new(1, 4);
        par2sol = new(1, 4);
        pararg1 = new(1, 4);
        pararg2 = new(1, 4);


        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        NCollection_Array1<double> cote1 = new(1, 2);
        NCollection_Array1<double> cote2 = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        NbrSol                              = 0;
        WellDone                            = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Lin2d L1     = Qualified1.Qualified();
        gp_Lin2d L2     = Qualified2.Qualified();
        double   x1dir  = (L1.Direction()).X();
        double   y1dir  = (L1.Direction()).Y();
        double   lx1loc = (L1.Location()).X();
        double   ly1loc = (L1.Location()).Y();
        double   x2dir  = (L2.Direction()).X();
        double   y2dir  = (L2.Direction()).Y();
        double   lx2loc = (L2.Location()).X();
        double   ly2loc = (L2.Location()).Y();
        gp_Pnt2d origin1 = new(lx1loc, ly1loc);
        gp_Pnt2d origin2 = new(lx2loc, ly2loc);
        // OCCT: normL1 and normL2 are the lines' directions, not their normals, so the qualifiers computed below for
        // unqualified lines compare the side of the centre along the line, not across it
        gp_Dir2d normL1 = new(x1dir, y1dir);
        gp_Dir2d normL2 = new(x2dir, y2dir);
        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            if (L1.Direction().IsParallel(L2.Direction(), Precision.Angular()))
            {
                WellDone = true;
            }
            else
            {
                if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed())
                {
                    //    =======================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote2[1] = 1.0;
                }
                else if (Qualified1.IsEnclosed() && Qualified2.IsOutside())
                {
                    //    ==========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote2[1] = -1.0;
                }
                else if (Qualified1.IsOutside() && Qualified2.IsEnclosed())
                {
                    //    ===========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = -1.0;
                    cote2[1] = 1.0;
                }
                else if (Qualified1.IsOutside() && Qualified2.IsOutside())
                {
                    //    =========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = -1.0;
                    cote2[1] = -1.0;
                }
                if (Qualified1.IsEnclosed() && Qualified2.IsUnqualified())
                {
                    //    =========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 2;
                    cote1[1] = 1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                if (Qualified1.IsUnqualified() && Qualified2.IsEnclosed())
                {
                    //    =========================================================
                    nbrcote1 = 2;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = 1.0;
                }
                else if (Qualified1.IsOutside() && Qualified2.IsUnqualified())
                {
                    //    =============================================================
                    nbrcote1 = 1;
                    nbrcote2 = 2;
                    cote1[1] = -1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                if (Qualified1.IsUnqualified() && Qualified2.IsOutside())
                {
                    //    ========================================================
                    nbrcote1 = 2;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = -1.0;
                }
                else if (Qualified1.IsUnqualified() && Qualified2.IsUnqualified())
                {
                    //    =================================================================
                    nbrcote1 = 2;
                    nbrcote2 = 2;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
                {
                    for (int jcote2 = 1; jcote2 <= nbrcote2; jcote2++)
                    {
                        gp_Lin2d linint1 = new(new gp_Pnt2d(lx1loc - cote1[jcote1] * y1dir * Radius,
                                                                            ly1loc + cote1[jcote1] * x1dir * Radius),
                                                          L1.Direction());
                        gp_Lin2d linint2 = new(new gp_Pnt2d(lx2loc - cote2[jcote2] * y2dir * Radius,
                                                                            ly2loc + cote2[jcote2] * x2dir * Radius),
                                                          L2.Direction());
                        IntAna2d_AnaIntersection Intp = new(linint1, linint2);
                        if (Intp.IsDone())
                        {
                            if (!Intp.IsEmpty())
                            {
                                for (int i = 1; i <= Intp.NbPoints(); i++)
                                {
                                    NbrSol++;
                                    gp_Pnt2d Center = Intp.Point(i).Value();
                                    cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                    //              =======================================================
                                    gp_Dir2d dc1 = new(origin1.XY() - Center.XY());
                                    gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (dc1.Dot(normL1) > 0.0)
                                    {
                                        qualifier1[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    if (!Qualified2.IsUnqualified())
                                    {
                                        qualifier2[NbrSol] = Qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(normL2) > 0.0)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                    TheSame1[NbrSol] = 0;
                                    TheSame2[NbrSol] = 0;
                                    pnttg1sol[NbrSol] =
                                        new gp_Pnt2d(Center.XY() + cote1[jcote1] * Radius * new gp_XY(y1dir, -x1dir));
                                    pnttg2sol[NbrSol] =
                                        new gp_Pnt2d(Center.XY() + cote2[jcote2] * Radius * new gp_XY(y2dir, -x2dir));
                                }
                            }
                            WellDone = true;
                        }
                    }
                }
            }
        }
        for (int i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            pararg1[i] = ElCLib.Parameter(L1, pnttg1sol[i]);
            par2sol[i] = ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
            pararg2[i] = ElCLib.Parameter(L2, pnttg2sol[i]);
        }
    }
}
