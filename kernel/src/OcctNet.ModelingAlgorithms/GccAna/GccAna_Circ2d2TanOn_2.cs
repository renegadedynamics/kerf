// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    /// <summary>The circles tangent to two lines with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedLin Qualified1,
                               in GccEnt_QualifiedLin Qualified2,
                               in Line2D OnLine,
                               double Tolerance)
        : this(2)
    {
        // initialisations
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;

        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        Direction2D dirx = new(Direction2D.D.X);
        double Tol = Math.Abs(Tolerance);

        // calculation of bisectrices of L1 and L2
        Line2D L1 = Qualified1.Qualified();
        Line2D L2 = Qualified2.Qualified();
        Point2D originL1 = L1.Location();
        Point2D originL2 = L2.Location();
        GccAna_Lin2dBisec Bis = new(L1, L2);

        if (Bis.IsDone())
        {
            if (Bis.NbSolutions() == 1 || Bis.NbSolutions() == 2)
            {
                // if 1 bisectrice, L1 and L2 are parallel
                // if 2 bisectrices, L1 and L2 are intersected

                for (int k = 1; k <= Bis.NbSolutions(); k++)
                {
                    AnalyticIntersection Intp = new(Bis.ThisSolution(k), OnLine);
                    if (Intp.IsDone)
                    {
                        WellDone = true;
                        // for degenerated cases, no acceptable solution
                        // (OnLine and bisectrice strictly parallel or not)
                        if (!Intp.IdenticalElements() && !Intp.ParallelElements() && !Intp.IsEmpty)
                        {
                            // at maximum 1 point of intersection !
                            for (int l = 1; l <= Intp.PointCount; l++)
                            {
                                Point2D pt = Intp.Point(l).Value();
                                Axis2D axe = new(pt, dirx);
                                double Radius = L1.Distance(pt);
                                if (!L1.Contains(pt, Tol) && Radius < 1.0 / Tol && NbrSol < 2)
                                {
                                    // acceptable solution : the radius is correct
                                    NbrSol++;
                                    cirsol[NbrSol] = new Circle2D(axe, Radius);
                                }
                            }
                        }
                    }
                }
            }
        }

        // parce following the qualifiers NbrSol acceptable solutions

        for (int i = 1; i <= NbrSol; i++)
        {
            Point2D pbid = cirsol[i].Location();
            double Radius = cirsol[i].Radius();
            bool ok = false;

            // solution Outside or Enclosed / L1
            Direction2D dc1 = new(originL1.XY() - pbid.XY());
            double sign1 = dc1.Dot(new Direction2D(-L1.Direction().Y(), L1.Direction().X()));
            if (sign1 > 0.0)
            {
                ok = (Qualified1.IsUnqualified() || Qualified1.IsOutside());
            }
            else
            {
                ok = (Qualified1.IsUnqualified() || Qualified1.IsEnclosed());
            }

            // solution Outside or Enclosed / L2
            Direction2D dc2 = new(originL2.XY() - pbid.XY());
            double sign2 = dc2.Dot(new Direction2D(-L2.Direction().Y(), L2.Direction().X()));
            if (sign2 > 0.0)
            {
                ok = ok && (Qualified2.IsUnqualified() || Qualified2.IsOutside());
            }
            else
            {
                ok = ok && (Qualified2.IsUnqualified() || Qualified2.IsEnclosed());
            }

            if (ok)
            {
                // solution to be preserved
                dc1 = new Direction2D(sign1 * new Xy(-L1.Direction().Y(), L1.Direction().X()));
                pnttg1sol[i] = new Point2D(pbid.XY() + Radius * dc1.XY());
                if (sign1 > 0.0)
                {
                    qualifier1[i] = GccEnt_outside;
                }
                else
                {
                    qualifier1[i] = GccEnt_enclosed;
                }
                dc2 = new Direction2D(sign2 * new Xy(-L2.Direction().Y(), L2.Direction().X()));
                pnttg2sol[i] = new Point2D(pbid.XY() + Radius * dc2.XY());
                if (sign2 > 0.0)
                {
                    qualifier2[i] = GccEnt_outside;
                }
                else
                {
                    qualifier2[i] = GccEnt_enclosed;
                }
                pntcen[i] = pbid;
                par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
                pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[i]);
                par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
                pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[i]);
                parcen3[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen[i]);
            }
            else
            {
                // solution to be rejected
                if (i == NbrSol)
                {
                    NbrSol--;
                }
                else
                {
                    for (int k = i + 1; k <= NbrSol; k++)
                    {
                        cirsol[k - 1] = cirsol[k];
                    }
                    NbrSol--;
                    i--;
                }
            }
        }
    }
}
