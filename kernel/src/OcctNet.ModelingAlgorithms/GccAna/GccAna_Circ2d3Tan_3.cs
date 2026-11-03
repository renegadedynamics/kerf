// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to three straight lines.                +
    //   Create Bissectrices at Qualified1 and Qualified2 and                 +
    //          Bissectrices at Qualified1 and Qualified3.                    +
    //   Intersect bissectrices calculated in this way ==> Center points      +
    //   Choose the center point that corresponds to qualifiers and           +
    //   construct the solution of radius equal to the distance between the   +
    //   chosen center point and straight line Qualified1.                               +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedLin Qualified1,
        in GccEnt_QualifiedLin Qualified2,
        in GccEnt_QualifiedLin Qualified3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, 4);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 4);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 4);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 4);
        TheSame1 = new NCollection_Array1<int>(1, 4);
        TheSame2 = new NCollection_Array1<int>(1, 4);
        TheSame3 = new NCollection_Array1<int>(1, 4);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 4);
        pnttg2sol = new NCollection_Array1<Point2D>(1, 4);
        pnttg3sol = new NCollection_Array1<Point2D>(1, 4);
        par1sol = new NCollection_Array1<double>(1, 4);
        par2sol = new NCollection_Array1<double>(1, 4);
        par3sol = new NCollection_Array1<double>(1, 4);
        pararg1 = new NCollection_Array1<double>(1, 4);
        pararg2 = new NCollection_Array1<double>(1, 4);
        pararg3 = new NCollection_Array1<double>(1, 4);

        TheSame1.Init(0);
        TheSame2.Init(0);
        TheSame3.Init(0);
        Direction2D dirx = new(Direction2D.D.X);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified())
            || !(Qualified3.IsEnclosed() || Qualified3.IsOutside() || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Line2D L1 = Qualified1.Qualified();
        Line2D L2 = Qualified2.Qualified();
        Line2D L3 = Qualified3.Qualified();
        Point2D origin1 = L1.Location();
        Direction2D dir1 = L1.Direction();
        Direction2D normL1 = new(-dir1.Y(), dir1.X());
        Point2D origin2 = L2.Location();
        Direction2D dir2 = L2.Direction();
        Direction2D normL2 = new(-dir2.Y(), dir2.X());
        Point2D origin3 = L3.Location();
        Direction2D dir3 = L3.Direction();
        Direction2D normL3 = new(-dir3.Y(), dir3.X());
        double xloc1 = origin1.X();
        double xloc2 = origin2.X();
        double xloc3 = origin3.X();
        double yloc1 = origin1.Y();
        double yloc2 = origin2.Y();
        double yloc3 = origin3.Y();
        double xdir1 = dir1.X();
        double xdir2 = dir2.X();
        double xdir3 = dir3.X();
        double ydir1 = dir1.Y();
        double ydir2 = dir2.Y();
        double ydir3 = dir3.Y();
        GccAna_Lin2dBisec Bisec1 = new(L1, L2);
        GccAna_Lin2dBisec Bisec2 = new(L1, L3);
        int ncote1 = 0;
        int ncote2 = 0;
        int ncote3 = 0;
        NCollection_Array1<double> cote1 = new(1, 2);
        NCollection_Array1<double> cote2 = new(1, 2);
        NCollection_Array1<double> cote3 = new(1, 2);
        int nbsol = 0;
        if (Bisec1.IsDone() && Bisec2.IsDone())
        {
            for (int i = 1; i <= Bisec1.NbSolutions(); i++)
            {
                for (int j = 1; j <= Bisec2.NbSolutions(); j++)
                {
                    AnalyticIntersection Intp = new(Bisec1.ThisSolution(i), Bisec2.ThisSolution(j));
                    if (Intp.IsDone)
                    {
                        if (!Intp.IsEmpty)
                        {
                            for (int k = 1; k <= Intp.PointCount; k++)
                            {
                                nbsol++;
                                double Radius =
                                  (L1.Distance(Intp.Point(k).Value()) + L2.Distance(Intp.Point(k).Value())
                                   + L3.Distance(Intp.Point(k).Value()))
                                  / 3.0;
                                Point2D Center = Intp.Point(k).Value();
                                double cx = Center.X();
                                double cy = Center.Y();
                                cirsol[nbsol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             ======================================================
                                Direction2D dc1 = new(origin1.XY() - Center.XY());
                                if (!Qualified1.IsUnqualified())
                                {
                                    qualifier1[nbsol] = Qualified1.Qualifier();
                                }
                                else if (dc1.Dot(normL1) > 0.0)
                                {
                                    qualifier1[nbsol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier1[nbsol] = GccEnt_enclosed;
                                }
                                Direction2D dc2 = new(origin2.XY() - Center.XY());
                                if (!Qualified2.IsUnqualified())
                                {
                                    qualifier2[nbsol] = Qualified2.Qualifier();
                                }
                                else if (dc2.Dot(normL2) > 0.0)
                                {
                                    qualifier2[nbsol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier2[nbsol] = GccEnt_enclosed;
                                }
                                Direction2D dc3 = new(origin3.XY() - Center.XY());
                                if (!Qualified3.IsUnqualified())
                                {
                                    qualifier3[nbsol] = Qualified3.Qualifier();
                                }
                                else if (dc3.Dot(normL3) > 0.0)
                                {
                                    qualifier3[nbsol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier3[nbsol] = GccEnt_enclosed;
                                }

                                double cross1 = new Direction2D(-ydir1, xdir1).Dot(new Direction2D(xloc1 - cx, yloc1 - cy));
                                double cross2 = new Direction2D(-ydir2, xdir2).Dot(new Direction2D(xloc2 - cx, yloc2 - cy));
                                double cross3 = new Direction2D(-ydir3, xdir3).Dot(new Direction2D(xloc3 - cx, yloc3 - cy));
                                if (cross1 != 0.0)
                                {
                                    cross1 = cross1 / Math.Abs(cross1);
                                }
                                pnttg1sol[nbsol] = new Point2D(new Xy(cx, cy) + cross1 * Radius * new Xy(-ydir1, xdir1));
                                if (cross2 != 0.0)
                                {
                                    cross2 = cross2 / Math.Abs(cross2);
                                }
                                pnttg2sol[nbsol] = new Point2D(new Xy(cx, cy) + cross2 * Radius * new Xy(-ydir2, xdir2));
                                if (cross3 != 0.0)
                                {
                                    cross3 = cross3 / Math.Abs(cross3);
                                }
                                pnttg3sol[nbsol] = new Point2D(new Xy(cx, cy) + cross3 * Radius * new Xy(-ydir3, xdir3));
                                par1sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[nbsol], pnttg1sol[nbsol]);
                                pararg1[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[nbsol]);
                                par2sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[nbsol], pnttg2sol[nbsol]);
                                pararg2[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[nbsol]);
                                par3sol[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[nbsol], pnttg3sol[nbsol]);
                                pararg3[nbsol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L3, pnttg3sol[nbsol]);
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
        if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed() &&
            //  =========================================================
            Qualified3.IsEnclosed())
        {
            //     ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = 1.0;
            cote3[1] = 1.0;
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed() &&
                 // ==============================================================
                 Qualified3.IsOutside())
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = 1.0;
            cote3[1] = -1.0;
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsOutside() &&
                 // =============================================================
                 Qualified3.IsEnclosed())
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = -1.0;
            cote3[1] = 1.0;
        }
        else if (Qualified1.IsEnclosed() && Qualified2.IsOutside() &&
                 // =============================================================
                 Qualified3.IsOutside())
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = 1.0;
            cote2[1] = -1.0;
            cote3[1] = -1.0;
        }
        else if (Qualified1.IsOutside() && Qualified2.IsEnclosed() &&
                 // =============================================================
                 Qualified3.IsEnclosed())
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = 1.0;
            cote3[1] = 1.0;
        }
        else if (Qualified1.IsOutside() && Qualified2.IsEnclosed() &&
                 // =============================================================
                 Qualified3.IsOutside())
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = 1.0;
            cote3[1] = -1.0;
        }
        else if (Qualified1.IsOutside() && Qualified2.IsOutside() &&
                 // ============================================================
                 Qualified3.IsEnclosed())
        {
            //          ========================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = -1.0;
            cote3[1] = 1.0;
        }
        else if (Qualified1.IsOutside() && Qualified2.IsOutside() &&
                 // ============================================================
                 Qualified3.IsOutside())
        {
            //          =======================
            ncote1 = 1;
            ncote2 = 1;
            ncote3 = 1;
            cote1[1] = -1.0;
            cote2[1] = -1.0;
            cote3[1] = -1.0;
        }
        else
        {
            if (Qualified1.IsUnqualified())
            {
                //   ====================================
                ncote1 = 2;
                cote1[1] = 1.0;
                cote1[2] = -1.0;
                if (Qualified2.IsUnqualified())
                {
                    //     ===============================
                    ncote2 = 2;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    if (Qualified3.IsUnqualified())
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote2 where cote3 is meant
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        NbrSol = nbsol;
                        WellDone = true;
                    }
                    else if (Qualified3.IsEnclosed())
                    {
                        //       ===============================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (Qualified3.IsOutside())
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (Qualified2.IsEnclosed())
                {
                    //     =================================
                    ncote2 = 1;
                    cote2[1] = 1.0;
                    if (Qualified3.IsUnqualified())
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote3(1) twice, cote3(2) is never set
                        cote3[1] = 1.0;
                        cote3[1] = -1.0;
                    }
                    else if (Qualified3.IsEnclosed())
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (Qualified3.IsOutside())
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (Qualified2.IsOutside())
                {
                    //     ================================
                    ncote2 = 1;
                    cote2[1] = -1.0;
                    if (Qualified3.IsUnqualified())
                    {
                        //       ===============================
                        ncote3 = 2;
                        cote3[1] = 1.0;
                        cote3[2] = -1.0;
                    }
                    else if (Qualified3.IsEnclosed())
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (Qualified3.IsOutside())
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
            }
            else if (Qualified2.IsUnqualified())
            {
                //   ===================================
                ncote2 = 2;
                cote2[1] = 1.0;
                cote2[2] = -1.0;
                if (Qualified1.IsEnclosed())
                {
                    //     ============================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (Qualified3.IsUnqualified())
                    {
                        //       ===============================
                        ncote3 = 2;
                        // OCCT: cote3(1) twice, cote3(2) is never set
                        cote3[1] = -1.0;
                        cote3[1] = -1.0;
                    }
                    else if (Qualified3.IsEnclosed())
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (Qualified3.IsOutside())
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
                else if (Qualified1.IsOutside())
                {
                    //     ================================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (Qualified3.IsUnqualified())
                    {
                        //       ===============================
                        ncote3 = 2;
                        cote3[1] = 1.0;
                        cote3[2] = -1.0;
                    }
                    else if (Qualified3.IsEnclosed())
                    {
                        //       =================================
                        ncote3 = 1;
                        cote3[1] = 1.0;
                    }
                    else if (Qualified3.IsOutside())
                    {
                        //       ================================
                        ncote3 = 1;
                        cote3[1] = -1.0;
                    }
                }
            }
            else if (Qualified3.IsUnqualified())
            {
                //   ===================================
                ncote3 = 2;
                cote3[1] = 1.0;
                cote3[2] = -1.0;
                if (Qualified1.IsEnclosed())
                {
                    //     ============================
                    ncote1 = 1;
                    cote1[1] = 1.0;
                    if (Qualified2.IsEnclosed())
                    {
                        //       ============================
                        ncote2 = 1;
                        cote2[1] = 1.0;
                    }
                    else if (Qualified2.IsOutside())
                    {
                        //       ===============================
                        ncote2 = 1;
                        cote2[1] = -1.0;
                    }
                }
                else if (Qualified1.IsOutside())
                {
                    //     ================================
                    ncote1 = 1;
                    cote1[1] = -1.0;
                    if (Qualified2.IsEnclosed())
                    {
                        //       ============================
                        ncote2 = 1;
                        cote2[1] = 1.0;
                    }
                    else if (Qualified2.IsOutside())
                    {
                        //       ===============================
                        ncote2 = 1;
                        cote2[1] = -1.0;
                    }
                }
            }
        }
        if (NbrSol > 0)
        {
            return;
        }
        for (int i = 1; i <= nbsol; i++)
        {
            for (int j1 = 1; j1 <= ncote1; j1++)
            {
                for (int j2 = 1; j2 <= ncote2; j2++)
                {
                    for (int j3 = 1; j3 <= ncote3; j3++)
                    {
                        if ((cote2[j2]
                               * ((cirsol[i].Location().X() - origin2.X()) * (-dir2.Y())
                                  + (cirsol[i].Location().Y() - origin2.Y()) * (dir2.X()))
                             > 0.0)
                            && (cote3[j3]
                                  * ((cirsol[i].Location().X() - origin3.X()) * (-dir3.Y())
                                     + (cirsol[i].Location().Y() - origin3.Y()) * (dir3.X()))
                                > 0.0)
                            && (cote1[j1]
                                  * ((cirsol[i].Location().X() - origin1.X()) * (-dir1.Y())
                                     + (cirsol[i].Location().Y() - origin1.Y()) * (dir1.X()))
                                > 0.0))
                        {
                            NbrSol++;
                            // OCCT: qualifier1..3 are not copied to the new slot with the circle
                            cirsol[NbrSol] = cirsol[i];
                            //           =====================================
                            double Radius = cirsol[NbrSol].Radius();
                            Point2D Center = cirsol[NbrSol].Location();
                            Direction2D dc = new(origin1.XY() - Center.XY());
                            double sign = dc.Dot(new Direction2D(-dir1.Y(), dir1.X()));
                            dc = new Direction2D(sign * new Xy(-dir1.Y(), dir1.X()));
                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dc.XY());
                            dc = new Direction2D(origin2.XY() - Center.XY());
                            sign = dc.Dot(new Direction2D(-dir2.Y(), dir2.X()));
                            dc = new Direction2D(sign * new Xy(-dir2.Y(), dir2.X()));
                            pnttg2sol[NbrSol] = new Point2D(Center.XY() + Radius * dc.XY());
                            dc = new Direction2D(origin3.XY() - Center.XY());
                            sign = dc.Dot(new Direction2D(-dir3.Y(), dir3.X()));
                            dc = new Direction2D(sign * new Xy(-dir3.Y(), dir3.X()));
                            pnttg3sol[NbrSol] = new Point2D(Center.XY() + Radius * dc.XY());
                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                            par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                            par3sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                            pararg3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L3, pnttg3sol[NbrSol]);
                        }
                    }
                }
            }
        }
    }
}
