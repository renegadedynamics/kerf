// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_1.cxx
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
    //   Circle tangent to straight line  Qualified1 (L1)                         +
    //          center on straight line OnLine                                  +
    //          of radius Radius.                                              +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Elimine depending on the qualifier the cases not being solutions.     +
    //  Create L1para : parallel to L1 in the direction required by the       +
    //                  qualifier at distance Radius.                         +
    //  Point P of intersection between L1para and OnLine will give the center point +
    //  of the solution.                                                +
    //  Create solutions cirsol with center P and radius Radius.          +
    //  Fill the fields.                                                +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in GccEnt_QualifiedLin Qualified1, in Line2D OnLine, double Radius, double Tolerance)
    {
        cirsol = new(1, 2);
        qualifier1 = new(1, 2);
        TheSame1 = new(1, 2);
        pnttg1sol = new(1, 2);
        pntcen3 = new(1, 2);
        par1sol = new(1, 2);
        pararg1 = new(1, 2);
        parcen3 = new(1, 2);


        double   Tol = Math.Abs(Tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        WellDone = false;
        NbrSol   = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int                     nbsol = 0;
        NCollection_Array1<int> eps = new(1, 2);
        Line2D                L1 = Qualified1.Qualified();
        Point2D origin1 = L1.Location();
        Direction2D dir1 = L1.Direction();
        Direction2D normL1 = new(-dir1.Y(), dir1.X());

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if ((OnLine.Direction()).IsParallel(dir1, Tol))
        {
            WellDone = true;
        }
        else
        {
            if (Qualified1.IsEnclosed())
            {
                //   ============================
                eps[1] = -1;
                nbsol  = 1;
            }
            else if (Qualified1.IsOutside())
            {
                //   ================================
                eps[1] = 1;
                nbsol  = 1;
            }
            else
            {
                //   ====
                eps[1] = 1;
                eps[2] = -1;
                nbsol  = 2;
            }
            double dx1 = dir1.X();
            double dy1 = dir1.Y();
            double lx1 = origin1.X();
            double ly1 = origin1.Y();
            for (int j = 1; j <= nbsol; j++)
            {
                Line2D L1para = new(new Point2D(lx1 + eps[j] * Radius * dy1, ly1 - eps[j] * Radius * dx1), dir1);
                IntAna2d_AnaIntersection Intp = new(OnLine, L1para);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints(); i++)
                        {
                            NbrSol++;
                            Point2D Center = Intp.Point(i).Value();
                            cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                            //           =====================================================
                            Direction2D dc1 = new(origin1.XY() - Center.XY());
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
                            TheSame1[NbrSol] = 0;
                            if (new Vector2D(Center, origin1).Dot(new Vector2D(new Direction2D(-dy1, dx1))) > 0.0)
                            {
                                pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * new Xy(-dy1, dx1));
                                pntcen3[NbrSol]   = cirsol[1].Location(); // OCCT: cirsol(1), not cirsol(NbrSol)
                            }
                            else
                            {
                                pnttg1sol[NbrSol] = new Point2D(Center.XY() - Radius * new Xy(-dy1, dx1));
                                pntcen3[NbrSol]   = cirsol[1].Location(); // OCCT: cirsol(1), not cirsol(NbrSol)
                            }
                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                            parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen3[NbrSol]);
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
