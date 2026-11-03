// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanObl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D lines through a point or tangent to a circle, making an angle (in radians) with a line. The origin of each
/// solution is its tangency point with the first argument.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_Lin2dTanObl
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<Line2D> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pntint2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    private GccAna_Lin2dTanObl(int theSize)
    {
        linsol = new NCollection_Array1<Line2D>(1, theSize);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        pnttg1sol = new NCollection_Array1<Point2D>(1, theSize);
        pntint2sol = new NCollection_Array1<Point2D>(1, theSize);
        par1sol = new NCollection_Array1<double>(1, theSize);
        par2sol = new NCollection_Array1<double>(1, theSize);
        pararg1 = new NCollection_Array1<double>(1, theSize);
        pararg2 = new NCollection_Array1<double>(1, theSize);
    }

    //=========================================================================
    //   Creation of a straight line passing by a point : ThePoint                +
    //                         making an angle     : TheAngle                +
    //                         with straight line  : TheLine.                +
    //   Subject the straight line (ThePoint,TheLine.Location()) to a rotation +
    //   by angle TheAngle ==> D1.                                            +
    //   create straight line passing through ThePoint of direction D1.              +
    //=========================================================================

    /// <summary>The line through ThePoint making the angle TheAngle with TheLine.</summary>
    public GccAna_Lin2dTanObl(in Point2D ThePoint, in Line2D TheLine, double TheAngle) : this(1)
    {
        double Cosa = TheLine.Direction().X();
        double Sina = TheLine.Direction().Y();
        linsol[1] = new Line2D(ThePoint,
                                 // ==============================
                                 new Direction2D(Cosa * Math.Cos(TheAngle) - Sina * Math.Sin(TheAngle),
                                              //                      ===============================================
                                              Sina * Math.Cos(TheAngle) + Math.Sin(TheAngle) * Cosa));
        //                               =======================================
        qualifier1[1] = GccEnt_noqualifier;
        pnttg1sol[1] = ThePoint;
        AnalyticIntersection Intp = new(linsol[1], TheLine);
        if (Intp.IsDone)
        {
            if (!Intp.IsEmpty)
            {
                for (int i = 1; i <= Intp.PointCount; i++)
                {
                    pntint2sol[1] = Intp.Point(i).Value();
                }
            }
            // OCCT: when the solution is parallel to TheLine there is no intersection, and the intersection data is
            // that of the default point (the origin)
            par1sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol[1], pnttg1sol[1]);
            par2sol[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol[1], pntint2sol[1]);
            pararg1[1] = 0.0;
            pararg2[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(TheLine, pntint2sol[1]);
            NbrSol = 1;
            WellDone = true;
        }
        else
        {
            WellDone = false;
            NbrSol = 0;
        }
    }

    //=========================================================================
    //   Creation of a straight line tangent to a circle  : Qualified1 (C1)         +
    //                         making angle           : TheAngle                +
    //                         with a straight line   : TheLine.                +
    //   Subject the straight line  (C1.Location,TheLine.Location()) to a       +
    //   rotation by angle TheAngle or -TheAngle ==> D1.                       +
    //   create the straight line passing by C1 of direction D1.                    +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle making the angle TheAngle with TheLine.</summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed.</exception>
    public GccAna_Lin2dTanObl(in GccEnt_QualifiedCirc Qualified1, in Line2D TheLine, double TheAngle) : this(2)
    {
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double Cosa = TheLine.Direction().X();
        double Sina = TheLine.Direction().Y();
        if (Qualified1.IsEnclosed())
        {
            // ============================
            throw new GccEnt_BadQualifier();
        }
        else
        {
            Circle2D C1 = Qualified1.Qualified();
            double R1 = C1.Radius();
            if (Qualified1.IsEnclosing())
            {
                //   =============================
                Xy xy = new(Math.Cos(TheAngle) * Cosa - Math.Sin(TheAngle) * Sina,
                               Math.Cos(TheAngle) * Sina + Math.Sin(TheAngle) * Cosa);
                pnttg1sol[1] = new Point2D(C1.Location().XY() + R1 * new Xy(xy.Y(), -xy.X()));
                linsol[1] = new Line2D(pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                qualifier1[1] = Qualified1.Qualifier();
                AnalyticIntersection Intp = new(linsol[1], TheLine);
                NbrSol = 1;
                WellDone = true;
                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int i = 1; i <= Intp.PointCount; i++)
                        {
                            pntint2sol[1] = Intp.Point(i).Value();
                        }
                    }
                }
            }
            else if (Qualified1.IsOutside())
            {
                //   ================================
                Xy xy = new(Math.Cos(TheAngle) * Cosa - Math.Sin(TheAngle) * Sina,
                               Math.Cos(TheAngle) * Sina + Math.Sin(TheAngle) * Cosa);
                pnttg1sol[1] = new Point2D(C1.Location().XY() + R1 * new Xy(-xy.Y(), xy.X()));
                linsol[1] = new Line2D(pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                qualifier1[1] = Qualified1.Qualifier();
                AnalyticIntersection Intp = new(linsol[1], TheLine);
                WellDone = true;
                NbrSol = 1;
                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int i = 1; i <= Intp.PointCount; i++)
                        {
                            pntint2sol[1] = Intp.Point(i).Value();
                        }
                    }
                }
            }
            else if (Qualified1.IsUnqualified())
            {
                //   ====================================
                Xy xy = new(Math.Cos(TheAngle) * Cosa - Math.Sin(TheAngle) * Sina,
                               Math.Cos(TheAngle) * Sina + Math.Sin(TheAngle) * Cosa);
                pnttg1sol[1] = new Point2D(C1.Location().XY() + R1 * new Xy(xy.Y(), -xy.X()));
                linsol[1] = new Line2D(pnttg1sol[1], new Direction2D(xy));
                //     ===============================================
                qualifier1[1] = GccEnt_enclosing;
                AnalyticIntersection Intp = new(linsol[1], TheLine);
                WellDone = true;
                NbrSol = 1;
                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int i = 1; i <= Intp.PointCount; i++)
                        {
                            pntint2sol[1] = Intp.Point(i).Value();
                        }
                    }
                }
                pnttg1sol[2] = new Point2D(C1.Location().XY() + R1 * new Xy(-xy.Y(), xy.X()));
                linsol[2] = new Line2D(pnttg1sol[2], new Direction2D(xy));
                //     ===============================================
                qualifier1[2] = GccEnt_outside;
                // OCCT: intersects the first solution again, not linsol(2), so the second solution's intersection
                // point is the first's
                Intp = new AnalyticIntersection(linsol[1], TheLine);
                NbrSol++;
                WellDone = true;
                if (Intp.IsDone)
                {
                    if (!Intp.IsEmpty)
                    {
                        for (int i = 1; i <= Intp.PointCount; i++)
                        {
                            pntint2sol[2] = Intp.Point(i).Value();
                        }
                    }
                }
            }

            for (int index = 1; index <= NbrSol; index++)
            {
                par1sol[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol[index], pnttg1sol[index]);
                // a circle of radius 0 puts the tangency point at its centre
                pararg1[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[index]);
                par2sol[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol[index], pntint2sol[index]);
                pararg2[index] = global::Kerf.Engine.Geometry.ElCLib.Parameter(TheLine, pntint2sol[index]);
            }
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        return NbrSol;
    }

    /// <summary>The solution number Index.</summary>
    public Line2D ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        return linsol[Index];
    }

    /// <summary>
    /// The qualifier of the first argument for the solution number Index: the one given, the one computed for an
    /// unqualified circle (enclosing or outside), or GccEnt_noqualifier for a point.
    /// </summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            Qualif1 = qualifier1[Index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
            PntSol = pnttg1sol[Index];
        }
    }

    /// <summary>The intersection of the solution number Index with the line, and its parameters on both.</summary>
    public void Intersection2(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            ParSol = par2sol[Index];
            ParArg = pararg2[Index];
            PntSol = pntint2sol[Index];
        }
    }
}
