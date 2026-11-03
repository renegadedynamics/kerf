// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2dTanOnRad
{
    //=========================================================================
    //   Circle tangent to a point   Point1.                                  +
    //          center on circle     OnCirc.                                  +
    //          radius               Radius.                                  +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                               +
    //  Create the circle with center in Point1 of radius Radius.             +
    //  Intersect this circle with OnCirc and obtain the center points        +
    //  of found solutions.                                                   +
    //  Create solutions cirsol.                                              +
    //=========================================================================
    /// <summary>Circles of radius Radius passing through a point, centred on a circle.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in Point2D Point1, in Circle2D OnCirc, double Radius, double Tolerance)
    {
        cirsol = new(1, 2);
        qualifier1 = new(1, 2);
        TheSame1 = new(1, 2);
        pnttg1sol = new(1, 2);
        pntcen3 = new(1, 2);
        par1sol = new(1, 2);
        pararg1 = new(1, 2);
        parcen3 = new(1, 2);


        Direction2D dirx = new(Direction2D.D.X);
        double   Tol   = Math.Abs(Tolerance);
        WellDone       = false;
        NbrSol         = 0;
        double Roncirc = OnCirc.Radius();
        double dist1   = Point1.Distance(OnCirc.Location()) - Roncirc;
        double dist2   = Point1.Distance(OnCirc.Location()) + Roncirc;

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if ((dist1 - Radius > Tol) || (Tol < Radius - dist2))
        {
            WellDone = true;
        }
        else
        {
            int signe = 0;
            if (Math.Abs(dist1 - Radius) < Tol)
            {
                signe = 1;
            }
            else if (Math.Abs(dist2 - Radius) < Tol)
            {
                signe = -1;
            }
            if (signe != 0)
            {
                Axis2D axe = new(new Point2D(OnCirc.Location().XY()
                                                          - Roncirc
                                                                  * new Direction2D(OnCirc.Location().X() - signe * Point1.X(), // OCCT: Location - signe * Point1 is a sum when signe is -1
                                                                                        OnCirc.Location().Y() - signe * Point1.Y())
                                                                          .XY()),
                                        dirx);
                cirsol[1] = new Circle2D(axe, Radius);
                //      ================================
                qualifier1[1] = GccEnt_noqualifier;
                TheSame1[1]   = 0;
                pnttg1sol[1]  = Point1;
                pntcen3[1]    = cirsol[1].Location();
                pararg1[1]    = 0.0;
                par1sol[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
                parcen3[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[1]);
                WellDone      = true;
                NbrSol        = 1;
            }
            else
            {
                AnalyticIntersection Intp = new(OnCirc, new Circle2D(new Axis2D(Point1, dirx), Radius));
                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int i = 1; i <= Intp.PointCount; i++)
                        {
                            NbrSol++;
                            Point2D Center = Intp.Point(i).Value();
                            cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[1] = GccEnt_noqualifier; // OCCT: index 1, not NbrSol, here and below
                            TheSame1[1]   = 0;
                            pnttg1sol[1]  = Point1;
                            pntcen3[1]    = cirsol[1].Location();
                            par1sol[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
                            parcen3[1]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnCirc, pntcen3[1]);
                            pararg1[1]    = 0.0;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }
}
