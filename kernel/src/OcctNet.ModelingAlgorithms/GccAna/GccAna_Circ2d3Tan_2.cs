// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //=========================================================================
    //   Creation of a circle tangent to a circle and two straight lines.          +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in GccEnt_QualifiedLin Qualified2,
        in GccEnt_QualifiedLin Qualified3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, 8);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 8);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 8);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 8);
        TheSame1 = new NCollection_Array1<int>(1, 8);
        TheSame2 = new NCollection_Array1<int>(1, 8);
        TheSame3 = new NCollection_Array1<int>(1, 8);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 8);
        pnttg2sol = new NCollection_Array1<Point2D>(1, 8);
        pnttg3sol = new NCollection_Array1<Point2D>(1, 8);
        par1sol = new NCollection_Array1<double>(1, 8);
        par2sol = new NCollection_Array1<double>(1, 8);
        par3sol = new NCollection_Array1<double>(1, 8);
        pararg1 = new NCollection_Array1<double>(1, 8);
        pararg2 = new NCollection_Array1<double>(1, 8);
        pararg3 = new NCollection_Array1<double>(1, 8);

        TheSame1.Init(0);

        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified())
            || !(Qualified3.IsEnclosed() || Qualified3.IsOutside() || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D C1 = Qualified1.Qualified();
        Line2D L2 = Qualified2.Qualified();
        Line2D L3 = Qualified3.Qualified();
        double R1 = C1.Radius();
        Point2D center1 = C1.Location();
        Point2D origin2 = L2.Location();
        Direction2D dir2 = L2.Direction();
        Direction2D normL2 = new(-dir2.Y(), dir2.X());
        Point2D origin3 = L3.Location();
        Direction2D dir3 = L3.Direction();
        Direction2D normL3 = new(-dir3.Y(), dir3.X());

        NCollection_Array1<double> Radius = new(1, 2);
        GccAna_CircLin2dBisec Bis1 = new(C1, L2);
        GccAna_Lin2dBisec Bis2 = new(L2, L3);
        if (Bis1.IsDone() && Bis2.IsDone())
        {
            int nbsolution1 = Bis1.NbSolutions();
            int nbsolution2 = Bis2.NbSolutions();
            for (int i = 1; i <= nbsolution1; i++)
            {
                GccInt_Bisec Sol1 = Bis1.ThisSolution(i);
                GccInt_IType typ1 = Sol1.ArcType();
                IntAna2d_AnaIntersection Intp = new();
                for (int k = 1; k <= nbsolution2; k++)
                {
                    if (typ1 == GccInt_Lin)
                    {
                        Intp.Perform(Sol1.Line(), Bis2.ThisSolution(k));
                    }
                    else if (typ1 == GccInt_Par)
                    {
                        Intp.Perform(Bis2.ThisSolution(k), new IntAna2d_Conic(Sol1.Parabola()));
                    }
                    if (Intp.IsDone())
                    {
                        if ((!Intp.IsEmpty()) && (!Intp.ParallelElements()) && (!Intp.IdenticalElements()))
                        {
                            for (int j = 1; j <= Intp.NbPoints(); j++)
                            {
                                Point2D Center = Intp.Point(j).Value();
                                double dist1 = Center.Distance(center1);
                                double dist2 = L2.Distance(Center);
                                double dist3 = L3.Distance(Center);
                                int nbsol1 = 0;
                                int nbsol3 = 0;
                                bool ok = false;
                                if (Qualified1.IsEnclosed())
                                {
                                    if (dist1 - R1 < Tolerance)
                                    {
                                        Radius[1] = Math.Abs(R1 - dist1);
                                        nbsol1 = 1;
                                        ok = true;
                                    }
                                }
                                else if (Qualified1.IsOutside())
                                {
                                    if (R1 - dist1 < Tolerance)
                                    {
                                        Radius[1] = Math.Abs(R1 - dist1);
                                        nbsol1 = 1;
                                        ok = true;
                                    }
                                }
                                else if (Qualified1.IsEnclosing())
                                {
                                    ok = true;
                                    nbsol1 = 1;
                                    Radius[1] = Math.Abs(R1 - dist1);
                                }
                                else if (Qualified1.IsUnqualified())
                                {
                                    ok = true;
                                    nbsol1 = 2;
                                    Radius[1] = Math.Abs(R1 - dist1);
                                    Radius[2] = R1 + dist1;
                                }
                                if (Qualified2.IsEnclosed() && ok)
                                {
                                    if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                         + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        <= 0)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                            {
                                                ok = true;
                                                Radius[1] = Radius[ii];
                                            }
                                        }
                                    }
                                }
                                else if (Qualified2.IsOutside() && ok)
                                {
                                    if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                         + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        >= 0)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                            {
                                                ok = true;
                                                Radius[1] = Radius[ii];
                                            }
                                        }
                                    }
                                }
                                else if (Qualified2.IsUnqualified() && ok)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(dist2 - Radius[ii]) < Tol)
                                        {
                                            ok = true;
                                            Radius[1] = Radius[ii];
                                        }
                                    }
                                }
                                if (Qualified3.IsEnclosed() && ok)
                                {
                                    if ((((origin3.X() - Center.X()) * (-dir3.Y()))
                                         + ((origin3.Y() - Center.Y()) * (dir3.X())))
                                        <= 0)
                                    {
                                        if (Math.Abs(dist3 - Radius[1]) < Tol)
                                        {
                                            ok = true;
                                            nbsol3 = 1;
                                        }
                                    }
                                }
                                else if (Qualified3.IsOutside() && ok)
                                {
                                    if ((((origin3.X() - Center.X()) * (-dir3.Y()))
                                         + ((origin3.Y() - Center.Y()) * (dir3.X())))
                                        >= 0)
                                    {
                                        if (Math.Abs(dist3 - Radius[1]) < Tol)
                                        {
                                            ok = true;
                                            nbsol3 = 1;
                                        }
                                    }
                                }
                                else if (Qualified3.IsUnqualified() && ok)
                                {
                                    if (Math.Abs(dist3 - Radius[1]) < Tol)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                if (ok)
                                {
                                    for (int m = 1; m <= nbsol3; m++)
                                    {
                                        NbrSol++;
                                        cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius[m]);
                                        //                 ==========================================================
                                        double distcc1 = Center.Distance(center1);
                                        if (!Qualified1.IsUnqualified())
                                        {
                                            qualifier1[NbrSol] = Qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + Radius[m] - R1) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - R1 - Radius[m]) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosing;
                                        }
                                        Direction2D dc2 = new(origin2.XY() - Center.XY());
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
                                        Direction2D dc3 = new(origin3.XY() - Center.XY());
                                        if (!Qualified3.IsUnqualified())
                                        {
                                            qualifier3[NbrSol] = Qualified3.Qualifier();
                                        }
                                        else if (dc3.Dot(normL3) > 0.0)
                                        {
                                            qualifier3[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier3[NbrSol] = GccEnt_enclosed;
                                        }
                                        if (Center.Distance(center1) <= Tolerance
                                            && Math.Abs(Radius[m] - R1) <= Tolerance)
                                        {
                                            TheSame1[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame1[NbrSol] = 0;
                                            Direction2D dc1 = new(center1.XY() - Center.XY());
                                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[m] * dc1.XY());
                                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                        }
                                        TheSame2[NbrSol] = 0;
                                        TheSame3[NbrSol] = 0;
                                        Direction2D dc = new(origin2.XY() - Center.XY());
                                        double sign = dc.Dot(new Direction2D(-dir2.Y(), dir2.X()));
                                        dc = new Direction2D(sign * new Xy(-dir2.Y(), dir2.X()));
                                        pnttg2sol[NbrSol] = new Point2D(Center.XY() + Radius[m] * dc.XY());
                                        par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                        pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                        dc = new Direction2D(origin3.XY() - Center.XY());
                                        sign = dc.Dot(new Direction2D(-dir3.Y(), dir3.X()));
                                        dc = new Direction2D(sign * new Xy(-dir3.Y(), dir3.X()));
                                        pnttg3sol[NbrSol] = new Point2D(Center.XY() + Radius[m] * dc.XY());
                                        par3sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                                        pararg3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L3, pnttg3sol[NbrSol]);
                                    }
                                }
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }
}
