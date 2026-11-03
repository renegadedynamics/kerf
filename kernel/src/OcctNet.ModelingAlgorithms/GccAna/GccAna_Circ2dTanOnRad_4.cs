// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to straight line Qualified1 (L1).                     +
    //          center on  circle OnCirc.                                     +
    //          with radius       Radius.                                     +
    //                                                                        +
    //  Initialize table of solutions cirsol and all fields.                  +
    //  Eliminate cases not being the solution.                               +
    //  Create parallel line(s) to L1 in the required direction(s).           +
    //  Intersect parallel line(s) with OnCirc and obtain                     +
    //  center points of found solutions.                                     +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in GccEnt_QualifiedLin Qualified1, in Circle2D OnCirc, double Radius, double Tolerance)
    {
        cirsol = new(1, 4);
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
        WellDone     = false;
        NbrSol       = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //  Initialisation of various variables.                                 +
        //=========================================================================

        int      nbsol = 0;
        int      sign  = 0;
        Line2D L1    = Qualified1.Qualified();
        Point2D origin1 = L1.Location();
        Direction2D dir1 = L1.Direction();
        Direction2D normL1 = new(-dir1.Y(), dir1.X());
        double   dist1 = L1.Distance(OnCirc.Location()) - OnCirc.Radius();
        double   dist2 = L1.Distance(OnCirc.Location()) + OnCirc.Radius();

        //=========================================================================
        //  Processing.                                                           +
        //=========================================================================

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            L1 = Qualified1.Qualified();
            if ((dist1 - Radius > Tol) || (Tol < Radius - dist2))
            {
                WellDone = true;
            }
            else
            {

                // to modify later

                if (dist1 - Radius > 0.0)
                {
                    dist1 = Radius;
                }
                else if (dist2 - Radius < 0.0)
                {
                    dist2 = Radius;
                }

                if (Qualified1.IsEnclosed())
                {
                    //     ============================
                    nbsol = 1;
                    sign  = -1;
                }
                else if (Qualified1.IsOutside())
                {
                    //     ================================
                    nbsol = 1;
                    sign  = 1;
                }
                else
                {
                    //     ====
                    nbsol = 2;
                    sign  = 1;
                }
                for (int j = 1; j <= nbsol; j++)
                {
                    sign = -sign;
                    Line2D L = new(
                        new Point2D(origin1.X() - sign * Radius * dir1.Y(), origin1.Y() + sign * Radius * dir1.X()),
                        dir1);
                    IntAna2d_AnaIntersection Intp = new(L, OnCirc);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int i = 1; i <= Intp.NbPoints(); i++)
                            {
                                NbrSol++;
                                Point2D Center = Intp.Point(i).Value();
                                Axis2D axe = new(Center, dirx);
                                cirsol[NbrSol] = new Circle2D(axe, Radius);
                                //             ======================================
                                Direction2D dc1 = new(origin1.XY() - Center.XY());
                                sign = (int)dc1.Dot(normL1); // OCCT: truncates the dot product (0 unless it is exactly 1 or -1) and clobbers the loop's side
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
                                pntcen3[NbrSol]   = cirsol[NbrSol].Location();
                                pnttg1sol[NbrSol] = new Point2D(
                                    pntcen3[NbrSol].XY() + new Xy(sign * Radius * dir1.Y(), -sign * Radius * dir1.X()));
                                pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                                par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[NbrSol]);
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }
}
