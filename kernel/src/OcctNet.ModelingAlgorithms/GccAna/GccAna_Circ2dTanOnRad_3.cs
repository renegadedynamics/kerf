// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to a circle   Qualified1 (C1).                        +
    //          center on a circle  OnCirc.                                   +
    //          of radius              Radius.                                +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Create parallel(s) to C1 in the required direction(s).       +
    //  Intersect parallel(s) with OnCirc and obtain the   +
    //  center point(s) of found solution(s).       +
    //  Create solution(s) cirsol.                               +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in GccEnt_QualifiedCirc Qualified1, in Circle2D OnCirc, double Radius, double Tolerance)
    {
        cirsol = new(1, 4);
        // OCCT: qualifier1(NbrSol) is written with NbrSol = 0 when sign != 0 (below), outside the array (1, 4): OCCT's
        // range check throws Standard_OutOfRange there, and so does the port
        qualifier1 = new(1, 4);
        TheSame1 = new(1, 4);
        pnttg1sol = new(1, 4);
        pntcen3 = new(1, 4);
        par1sol = new(1, 4);
        pararg1 = new(1, 4);
        parcen3 = new(1, 4);


        TheSame1.Init(0);
        Direction2D dirx = new(Direction2D.D.X);
        double   Tol = Math.Abs(Tolerance);
        Span<int> signe = stackalloc int[5];
        signe[0] = 0;
        signe[1] = 0;
        signe[2] = 0;
        signe[3] = 0;
        signe[4] = 0;
        Span<double> disparal = stackalloc double[2];
        int    nparal = 0;
        int    sign   = 0;
        WellDone      = false;
        NbrSol        = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D                    C1 = Qualified1.Qualified();
        NCollection_Array1<Point2D> Center = new(1, 4);
        NCollection_Array1<Direction2D> dir1on = new(1, 4);

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            double   R1  = C1.Radius();
            double   R2  = OnCirc.Radius();
            double   c1x = C1.Location().X();
            double   c1y = C1.Location().Y();
            Point2D center1 = new(c1x, c1y);
            double   dist = OnCirc.Location().Distance(center1);
            double   onx  = OnCirc.Location().X();
            double   ony  = OnCirc.Location().Y();
            if (Qualified1.IsEnclosed())
            {
                //   ============================
                if ((Radius - R1 > Tol) || (R1 - dist - R2 - Radius > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(Radius - R1) < Tol)
                    {
                        if (OnCirc.Distance(center1) < Tol)
                        {
                            NbrSol         = 1;
                            cirsol[NbrSol] = C1;
                            //           ==============
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            TheSame1[NbrSol]   = 1;
                            pntcen3[NbrSol]    = center1;
                            parcen3[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[NbrSol]);
                            WellDone           = true;
                        }
                        else
                        {
                            WellDone = false;
                        }
                    }
                    else if (Math.Abs(R1 - dist - R2 - Radius) <= Tol)
                    {
                        dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                        sign      = -1;
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(R1 - Radius);
                    }
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //   ==================================
                if ((Tol < R1 - Radius) || (Tol < R1 + dist - R2 - Radius) || (Radius - R1 - dist - R2 > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(Radius - R1) < Tol)
                    {
                        if (OnCirc.Distance(center1) < Tol)
                        {
                            NbrSol         = 1;
                            cirsol[NbrSol] = C1;
                            //           ==============
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            TheSame1[NbrSol]   = 1;
                            pntcen3[NbrSol]    = center1;
                            parcen3[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[NbrSol]);
                            WellDone           = true;
                        }
                        else
                        {
                            WellDone = true;
                        }
                    }
                    else if ((Math.Abs(R1 + dist - R2 - Radius) < Tol)
                                      || (Math.Abs(Radius - R1 - dist - R2) < Tol))
                    {
                        dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                        if (Math.Abs(R1 + dist - R2 - Radius) < Tol)
                        {
                            sign = 1;
                        }
                        else
                        {
                            sign = -1;
                        }
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(R1 - Radius);
                    }
                }
            }
            else if (Qualified1.IsOutside())
            {
                //   ================================
                if ((dist - R2 - R1 - Radius > Tol) || (Radius - dist - R2 + R1 > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                    if (Math.Abs(R1 - dist - R2 + Radius) < Tol)
                    {
                        sign = -1;
                    }
                    else if (Math.Abs(dist - R1 - R2 - Radius) < Tol)
                    {
                        sign = 1;
                    }
                    else
                    {
                        nparal      = 1;
                        disparal[0] = Math.Abs(R1 + Radius);
                    }
                }
            }
            else
            {
                if ((dist - R2 - R1 - Radius > Tol) || (Radius - dist - R1 - R2 > Tol))
                {
                    WellDone = true;
                }
                else if ((R1 - dist - R2 > Tol) && (Tol < R1 - R2 - dist - Radius))
                {
                    WellDone = true;
                }
                else
                {
                    dir1on[1] = new Direction2D(c1x - onx, c1y - ony);
                    if (Math.Abs(dist - R1 - R2 - Radius) < Tol)
                    {
                        sign = 1;
                    }
                    else if (dist + R1 + R2 - Radius < 0.0)
                    {
                        sign = -1;
                    }
                    else if ((R1 - dist + R2 > 0.0) && (R1 - R2 - dist - Radius > 0.0))
                    {
                        sign = -1;
                    }
                    else
                    {
                        nparal      = 2;
                        disparal[0] = Math.Abs(R1 + Radius);
                        disparal[1] = Math.Abs(R1 - Radius);
                    }
                }
            }
            for (int jj = 0; jj < nparal; jj++)
            {
                IntAna2d_AnaIntersection Intp = new(OnCirc, new Circle2D(new Axis2D(center1, dirx), disparal[jj]));
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints(); i++)
                        {
                            NbrSol++;
                            Center[NbrSol] = Intp.Point(i).Value();
                            cirsol[NbrSol] = new Circle2D(new Axis2D(Center[NbrSol], dirx), Radius);
                            //           =============================================================
                            double distcc1 = Center[NbrSol].Distance(center1);
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
                            dir1on[NbrSol] = new Direction2D(c1x - Center[NbrSol].X(), c1y - Center[NbrSol].Y());
                            if ((R1 > Radius) && (Center[NbrSol].Distance(center1) <= R1))
                            {
                                signe[NbrSol - 1] = -1;
                            }
                            else
                            {
                                signe[NbrSol - 1] = 1;
                            }
                        }
                    }
                    WellDone = true;
                }
            }
            if (sign != 0)
            {
                Center[1] = new Point2D(OnCirc.Location().XY() + sign * R2 * dir1on[1].XY());
                cirsol[1] = new Circle2D(new Axis2D(Center[1], dirx), Radius);
                //     =====================================================
                double distcc1 = Center[1].Distance(center1);
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
                NbrSol            = 1;
                signe[NbrSol - 1] = 1;
                WellDone          = true;
            }
            int ii = 0;
            while (signe[ii] != 0)
            {
                pnttg1sol[ii + 1] = new Point2D(Center[ii + 1].XY() + signe[ii] * Radius * dir1on[ii + 1].XY());
                pntcen3[ii + 1]   = cirsol[ii + 1].Location();
                pararg1[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[ii + 1]);
                par1sol[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[ii + 1], pnttg1sol[ii + 1]);
                parcen3[ii + 1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[ii + 1]);
                ii++;
            }
        }
    }
}
