// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    private const int MaxSol = 20;

    //=========================================================================
    //   Creation of a circle tangent to two circles and a point.           +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in GccEnt_QualifiedCirc Qualified2,
        in Point2D Point3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<Circle2D>(1, MaxSol);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, MaxSol);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, MaxSol);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, MaxSol);
        TheSame1 = new NCollection_Array1<int>(1, MaxSol);
        TheSame2 = new NCollection_Array1<int>(1, MaxSol);
        TheSame3 = new NCollection_Array1<int>(1, MaxSol);
        pnttg1sol = new NCollection_Array1<Point2D>(1, MaxSol);
        pnttg2sol = new NCollection_Array1<Point2D>(1, MaxSol);
        pnttg3sol = new NCollection_Array1<Point2D>(1, MaxSol);
        par1sol = new NCollection_Array1<double>(1, MaxSol);
        par2sol = new NCollection_Array1<double>(1, MaxSol);
        par3sol = new NCollection_Array1<double>(1, MaxSol);
        pararg1 = new NCollection_Array1<double>(1, MaxSol);
        pararg2 = new NCollection_Array1<double>(1, MaxSol);
        pararg3 = new NCollection_Array1<double>(1, MaxSol);

        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D C1 = Qualified1.Qualified();
        Circle2D C2 = Qualified2.Qualified();
        double R1 = C1.Radius();
        double R2 = C2.Radius();
        Point2D center1 = C1.Location();
        Point2D center2 = C2.Location();

        NCollection_Array1<double> Radius = new(1, 2);
        GccAna_Circ2dBisec Bis1 = new(C1, C2);
        GccAna_CircPnt2dBisec Bis2 = new(C1, Point3);
        if (Bis1.IsDone() && Bis2.IsDone())
        {
            int nbsolution1 = Bis1.NbSolutions();
            int nbsolution2 = Bis2.NbSolutions();
            for (int i = 1; i <= nbsolution1; i++)
            {
                GccInt_Bisec Sol1 = Bis1.ThisSolution(i);
                GccInt_IType typ1 = Sol1.ArcType();
                AnalyticIntersection Intp = new();
                for (int k = 1; k <= nbsolution2; k++)
                {
                    GccInt_Bisec Sol2 = Bis2.ThisSolution(k);
                    GccInt_IType typ2 = Sol2.ArcType();
                    if (typ1 == GccInt_Cir)
                    {
                        if (typ2 == GccInt_Cir)
                        {
                            Intp.Perform(Sol1.Circle(), Sol2.Circle());
                        }
                        else if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol2.Line(), Sol1.Circle());
                        }
                        else if (typ2 == GccInt_Hpr)
                        {
                            Intp.Perform(Sol1.Circle(), new Conic(Sol2.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Ell)
                        {
                            Intp.Perform(Sol1.Circle(), new Conic(Sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == GccInt_Ell)
                    {
                        if (typ2 == GccInt_Cir)
                        {
                            Intp.Perform(Sol2.Circle(), new Conic(Sol1.Ellipse()));
                        }
                        else if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol2.Line(), new Conic(Sol1.Ellipse()));
                        }
                        else if (typ2 == GccInt_Hpr)
                        {
                            Intp.Perform(Sol1.Ellipse(), new Conic(Sol2.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Ell)
                        {
                            Intp.Perform(Sol1.Ellipse(), new Conic(Sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == GccInt_Lin)
                    {
                        if (typ2 == GccInt_Cir)
                        {
                            Intp.Perform(Sol1.Line(), Sol2.Circle());
                        }
                        else if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol1.Line(), Sol2.Line());
                        }
                        else if (typ2 == GccInt_Hpr)
                        {
                            Intp.Perform(Sol1.Line(), new Conic(Sol2.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Ell)
                        {
                            Intp.Perform(Sol1.Line(), new Conic(Sol2.Ellipse()));
                        }
                    }
                    else if (typ1 == GccInt_Hpr)
                    {
                        if (typ2 == GccInt_Cir)
                        {
                            Intp.Perform(Sol2.Circle(), new Conic(Sol1.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Lin)
                        {
                            Intp.Perform(Sol2.Line(), new Conic(Sol1.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Hpr)
                        {
                            Intp.Perform(Sol2.Hyperbola(), new Conic(Sol1.Hyperbola()));
                        }
                        else if (typ2 == GccInt_Ell)
                        {
                            Intp.Perform(Sol2.Ellipse(), new Conic(Sol1.Hyperbola()));
                        }
                    }
                    if (Intp.IsDone)
                    {
                        if (!Intp.IsEmpty)
                        {
                            for (int j = 1; j <= Intp.PointCount; j++)
                            {
                                double Rradius = 0;
                                Point2D Center = Intp.Point(j).Value();
                                double dist1 = Center.Distance(center1);
                                double dist2 = Center.Distance(center2);
                                double dist3 = Center.Distance(Point3);
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
                                    Radius[1] = R1 + dist1;
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
                                for (int ii = 1; ii <= nbsol2; ii++)
                                {
                                    if (Math.Abs(dist3 - Radius[ii]) <= Tol)
                                    {
                                        nbsol3++;
                                        ok = true;
                                    }
                                }
                                if (ok)
                                {
                                    for (int k1 = 1; k1 <= nbsol3; k1++)
                                    {
                                        NbrSol++;
                                        cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius[k1]);
                                        //                 ==========================================================
                                        double distcc1 = Center.Distance(center1);
                                        if (!Qualified1.IsUnqualified())
                                        {
                                            qualifier1[NbrSol] = Qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + Radius[k1] - R1) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - R1 - Radius[k1]) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosing;
                                        }

                                        //                   double distcc2 = Center.Distance(center1);
                                        double distcc2 = Center.Distance(center2);
                                        if (!Qualified2.IsUnqualified())
                                        {
                                            qualifier2[NbrSol] = Qualified2.Qualifier();
                                        }
                                        else if (Math.Abs(distcc2 + Radius[k1] - R2) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc2 - R2 - Radius[k1]) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosing;
                                        }
                                        qualifier3[NbrSol] = GccEnt_noqualifier;
                                        if (Center.Distance(center1) <= Tolerance
                                            && Math.Abs(Radius[k1] - R1) <= Tolerance)
                                        {
                                            TheSame1[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame1[NbrSol] = 0;
                                            Direction2D dc = new(center1.XY() - Center.XY());
                                            if (qualifier1[NbrSol] == GccEnt_enclosed)
                                            {
                                                // if tangent circle is inside the source circle, moving to edge of source circle
                                                dc.Reverse();
                                            }
                                            pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius[k1] * dc.XY());
                                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                        }
                                        if (Center.Distance(center2) <= Tolerance
                                            && Math.Abs(Radius[k1] - R2) <= Tolerance)
                                        {
                                            TheSame2[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame2[NbrSol] = 0;
                                            Direction2D dc = new(center2.XY() - Center.XY());
                                            // case of concentric circles :
                                            // 2nd tangency point is at the other side of the circle solution
                                            double alpha = 1.0;
                                            if (center1.Distance(center2) <= Tolerance)
                                            {
                                                alpha = -1;
                                            }
                                            pnttg2sol[NbrSol] = new Point2D(Center.XY() + alpha * Radius[k1] * dc.XY());
                                            par2sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                            pararg2[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                                        }
                                        TheSame3[NbrSol] = 0;
                                        pnttg3sol[NbrSol] = Point3;
                                        par3sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                                        pararg3[NbrSol] = 0.0;
                                        WellDone = true;
                                        if (NbrSol == MaxSol)
                                        {
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        WellDone = true;
                        if (NbrSol == MaxSol)
                        {
                            break;
                        }
                    }
                    if (NbrSol == MaxSol)
                    {
                        break;
                    }
                }
                if (NbrSol == MaxSol)
                {
                    break;
                }
            }
        }

        // Debug to create the point on the solution circles.

        int kk;
        for (kk = 1; kk <= NbrSol; kk++)
        {
            Circle2D CC = cirsol[kk];
            double NR = CC.Location().Distance(Point3);
            if (Math.Abs(NR - CC.Radius()) > Tol)
            {
                cirsol[kk].SetRadius(NR);
            }
        }

        // Debug to eliminate multiple solution.
        // this happens in case of intersection line hyperbola.
        double Tol2 = Tol * Tol;
        for (kk = 1; kk < NbrSol; kk++)
        {
            Point2D PK = cirsol[kk].Location();
            for (int ll = kk + 1; ll <= NbrSol; ll++)
            {
                Point2D PL = cirsol[ll].Location();
                if (PK.SquareDistance(PL) < Tol2)
                {
                    for (int mm = ll + 1; mm <= NbrSol; mm++)
                    {
                        cirsol[mm - 1] = cirsol[mm];
                        // OCCT: TheSame1..3 are not shifted with the other arrays
                        pnttg1sol[mm - 1] = pnttg1sol[mm];
                        pnttg2sol[mm - 1] = pnttg2sol[mm];
                        pnttg3sol[mm - 1] = pnttg3sol[mm];
                        par1sol[mm - 1] = par1sol[mm];
                        par2sol[mm - 1] = par2sol[mm];
                        par3sol[mm - 1] = par3sol[mm];
                        pararg1[mm - 1] = pararg1[mm];
                        pararg2[mm - 1] = pararg2[mm];
                        pararg3[mm - 1] = pararg3[mm];
                        qualifier1[mm - 1] = qualifier1[mm];
                        qualifier2[mm - 1] = qualifier2[mm];
                        qualifier3[mm - 1] = qualifier3[mm];
                    }
                    NbrSol--;
                }
            }
        }
    }
}
