// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_1.cxx
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
    //   Creation of a circle tangent to two circles and a straight line.     +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in GccEnt_QualifiedCirc Qualified2,
        in GccEnt_QualifiedLin Qualified3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, 16);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 16);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 16);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 16);
        TheSame1 = new NCollection_Array1<int>(1, 16);
        TheSame2 = new NCollection_Array1<int>(1, 16);
        TheSame3 = new NCollection_Array1<int>(1, 16);
        pnttg1sol = new NCollection_Array1<Point2D>(1, 16);
        pnttg2sol = new NCollection_Array1<Point2D>(1, 16);
        pnttg3sol = new NCollection_Array1<Point2D>(1, 16);
        par1sol = new NCollection_Array1<double>(1, 16);
        par2sol = new NCollection_Array1<double>(1, 16);
        par3sol = new NCollection_Array1<double>(1, 16);
        pararg1 = new NCollection_Array1<double>(1, 16);
        pararg2 = new NCollection_Array1<double>(1, 16);
        pararg3 = new NCollection_Array1<double>(1, 16);

        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified())
            || !(Qualified3.IsEnclosed() || Qualified3.IsOutside() || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D C1 = Qualified1.Qualified();
        Circle2D C2 = Qualified2.Qualified();
        Line2D L3 = Qualified3.Qualified();
        double R1 = C1.Radius();
        double R2 = C2.Radius();
        Point2D center1 = C1.Location();
        Point2D center2 = C2.Location();
        _ = center2;

        Point2D origin3 = L3.Location();
        Direction2D dir3 = L3.Direction();
        Direction2D normL3 = new(-dir3.Y(), dir3.X());

        NCollection_Array1<double> Radius = new(1, 2);
        GccAna_CircLin2dBisec Bis1 = new(C1, L3);
        GccAna_CircLin2dBisec Bis2 = new(C2, L3);
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
                    GccInt_Bisec Sol2 = Bis2.ThisSolution(k);
                    GccInt_IType typ2 = Sol2.ArcType();
                    if (typ1 == GccInt_Lin)
                    {
                        if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol1.Line(), Sol2.Line());
                        }
                        else if (typ2 == GccInt_Par)
                        {
                            Intp.Perform(Sol1.Line(), new IntAna2d_Conic(Sol2.Parabola()));
                        }
                    }
                    else if (typ1 == GccInt_Par)
                    {
                        if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol2.Line(), new IntAna2d_Conic(Sol1.Parabola()));
                        }
                        else if (typ2 == GccInt_Par)
                        {
                            Intp.Perform(Sol1.Parabola(), new IntAna2d_Conic(Sol2.Parabola()));
                        }
                    }
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int j = 1; j <= Intp.NbPoints(); j++)
                            {
                                double Rradius = 0;
                                Point2D Center = Intp.Point(j).Value();

                                // pop : if the coordinates are too great, no creation
                                if (Center.X() > 1e10 || Center.Y() > 1e10)
                                {
                                    break;
                                }

                                double dist1 = Center.Distance(C1.Location());
                                double dist2 = Center.Distance(C2.Location());
                                double dist3 = L3.Distance(Center);

                                // pop : if the coordinates are too great, no creation
                                if (dist3 > 1e10)
                                {
                                    break;
                                }

                                int nbsol1 = 0;
                                int nbsol2 = 0;
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
                                    if (dist2 - R2 < Tolerance)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(Radius[ii] - Math.Abs(R2 - dist2)) < Tol)
                                            {
                                                Radius[1] = Math.Abs(R2 - dist2);
                                                ok = true;
                                                nbsol2 = 1;
                                            }
                                        }
                                    }
                                }
                                else if (Qualified2.IsOutside() && ok)
                                {
                                    if (R2 - dist2 < Tolerance)
                                    {
                                        for (int ii = 1; ii <= nbsol1; ii++)
                                        {
                                            if (Math.Abs(Radius[ii] - Math.Abs(R2 - dist2)) < Tol)
                                            {
                                                Radius[1] = Math.Abs(R2 - dist2);
                                                ok = true;
                                                nbsol2 = 1;
                                            }
                                        }
                                    }
                                }
                                else if (Qualified2.IsEnclosing() && ok)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(Radius[ii] - R2 - dist2) < Tol)
                                        {
                                            Radius[1] = R2 + dist2;
                                            ok = true;
                                            nbsol2 = 1;
                                        }
                                    }
                                }
                                else if (Qualified2.IsUnqualified() && ok)
                                {
                                    for (int ii = 1; ii <= nbsol1; ii++)
                                    {
                                        if (Math.Abs(Radius[ii] - Math.Abs(R2 - dist2)) < Tol)
                                        {
                                            Rradius = Math.Abs(R2 - dist2);
                                            ok = true;
                                            nbsol2++;
                                        }
                                        else if (Math.Abs(Radius[ii] - R2 - dist2) < Tol)
                                        {
                                            Rradius = R2 + dist2;
                                            ok = true;
                                            nbsol2++;
                                        }
                                    }
                                    if (nbsol2 == 1)
                                    {
                                        Radius[1] = Rradius;
                                    }
                                    else if (nbsol2 == 2)
                                    {
                                        Radius[1] = Math.Abs(R2 - dist2);
                                        Radius[2] = R2 + dist2;
                                    }
                                }
                                if (Qualified3.IsEnclosed() && ok)
                                {
                                    if ((((L3.Location().X() - Center.X()) * (-L3.Direction().Y()))
                                         + ((L3.Location().Y() - Center.Y()) * (L3.Direction().X())))
                                        <= 0)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                // OCCT: Qualified2 where the line's Qualified3 is meant
                                else if (Qualified2.IsOutside() && ok)
                                {
                                    if ((((L3.Location().X() - Center.X()) * (-L3.Direction().Y()))
                                         + ((L3.Location().Y() - Center.Y()) * (L3.Direction().X())))
                                        >= 0)
                                    {
                                        ok = true;
                                        nbsol3 = 1;
                                    }
                                }
                                // OCCT: Qualified2 where the line's Qualified3 is meant
                                else if (Qualified2.IsUnqualified() && ok)
                                {
                                    ok = true;
                                    nbsol3 = 1;
                                }
                                // OCCT: ok is not reset when the second circle's or the line's check fails, and the distance to the line is never compared with the radius: solutions not tangent to all three arguments are kept
                                if (ok)
                                {
                                    for (int ind3 = 1; ind3 <= nbsol3; ind3++)
                                    {
                                        NbrSol++;
                                        cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius[ind3]);
                                        //                ==========================================================
                                        double distcc1 = Center.Distance(center1);
                                        if (!Qualified1.IsUnqualified())
                                        {
                                            qualifier1[NbrSol] = Qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + Radius[ind3] - R1) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - R1 - Radius[ind3]) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosing;
                                        }
                                        // OCCT: measured from center1, not center2
                                        double distcc2 = Center.Distance(center1);
                                        if (!Qualified2.IsUnqualified())
                                        {
                                            qualifier2[NbrSol] = Qualified2.Qualifier();
                                        }
                                        else if (Math.Abs(distcc2 + Radius[ind3] - R2) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc2 - R2 - Radius[ind3]) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosing;
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
                                        if (Center.Distance(C1.Location()) <= Tolerance
                                            && Math.Abs(Radius[ind3] - R1) <= Tolerance)
                                        {
                                            TheSame1[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame1[NbrSol] = 0;
                                            Direction2D dc1 = new(C1.Location().XY() - Center.XY());
                                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[ind3] * dc1.XY());
                                            // POP for protection if cirsol[NbrSol].Location == pnttg1sol[NbrSol]
                                            if (cirsol[NbrSol].Location().IsEqual(pnttg1sol[NbrSol],
                                                                                  global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                par1sol[NbrSol] = 1;
                                            }
                                            else
                                            {
                                                par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                            }
                                            // POP for protection if C1.Location == pnttg1sol[NbrSol]
                                            if (C1.Location().IsEqual(pnttg1sol[NbrSol], global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                pararg1[NbrSol] = 1;
                                            }
                                            else
                                            {
                                                pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                            }
                                        }
                                        if (Center.Distance(C2.Location()) <= Tolerance
                                            && Math.Abs(Radius[ind3] - R2) <= Tolerance)
                                        {
                                            TheSame2[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame2[NbrSol] = 0;
                                            Direction2D dc2 = new(C2.Location().XY() - Center.XY());
                                            pnttg2sol[NbrSol] = new Point2D(Center.XY() + Radius[ind3] * dc2.XY());
                                            // POP for protection if cirsol[NbrSol].Location == pnttg1sol[NbrSol]
                                            if (cirsol[NbrSol].Location().IsEqual(pnttg1sol[NbrSol],
                                                                                  global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                // OCCT: tests pnttg1sol and writes par1sol in the second tangency's block
                                                par1sol[NbrSol] = 1;
                                            }
                                            else
                                            {
                                                par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                            }
                                            // POP for protection if C2.Location == pnttg2sol[NbrSol]
                                            if (C2.Location().IsEqual(pnttg2sol[NbrSol], global::Kerf.Engine.Foundation.Precision.Confusion()))
                                            {
                                                pararg2[NbrSol] = 1;
                                            }
                                            else
                                            {
                                                pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                                            }
                                        }
                                        TheSame3[NbrSol] = 0;
                                        Direction2D dc = new(L3.Location().XY() - Center.XY());
                                        double sign = dc.Dot(new Direction2D(-L3.Direction().Y(), L3.Direction().X()));
                                        dc = new Direction2D(sign * new Xy(-L3.Direction().Y(), L3.Direction().X()));
                                        pnttg3sol[NbrSol] = new Point2D(Center.XY() + Radius[ind3] * dc.XY());
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
