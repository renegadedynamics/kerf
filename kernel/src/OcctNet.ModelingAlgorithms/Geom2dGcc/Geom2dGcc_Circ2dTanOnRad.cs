// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// 2D circles of a given radius tangent to a qualified curve (or through a point) whose centre lies on a curve. Lines
/// and circles go to GccAna_Circ2dTanOnRad, other curves to Geom2dGcc_Circ2dTanOnRadGeo.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 8 solutions.</remarks>
public sealed class Geom2dGcc_Circ2dTanOnRad
{
    private readonly bool WellDone;
    private readonly int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<Point2D> pntcen3;
    private readonly NCollection_Array1<double> parcen3;

    // The fields' initialiser list.
    private Geom2dGcc_Circ2dTanOnRad(int nbSol)
    {
        cirsol     = new(1, nbSol);
        qualifier1 = new(1, nbSol);
        TheSame1   = new(1, nbSol);
        pnttg1sol  = new(1, nbSol);
        par1sol    = new(1, nbSol);
        pararg1    = new(1, nbSol);
        pntcen3    = new(1, nbSol);
        parcen3    = new(1, nbSol);
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on OnCurve.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRad(in Geom2dGcc_QualifiedCurve Qualified1, Geom2dAdaptor_Curve OnCurve, double Radius,
                                    double Tolerance)
        : this(8)
    {
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
            CurveKind   Type1 = C1.GetType();
            CurveKind   Type2 = OnCurve.GetType();
            Geom2d_Curve?       CC1   = C1.Curve();
            Geom2d_Curve?       Con   = OnCurve.Curve();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            NbrSol = 0;
            if ((Type1 == Line || Type1 == Circle)
                && (Type2 == Line || Type2 == Circle))
            {
                if (Type1 == Circle)
                {
                    Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
                    Circle2D            c1   = CCC1.Circ2d();
                    GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        Geom2d_Circle         CCon = (Con as Geom2d_Circle)!;
                        Circle2D             con  = CCon.Circ2d();
                        GccAna_Circ2dTanOnRad Circ = new(Qc1, con, Radius, Tolerance);
                        WellDone                   = Circ.IsDone();
                        NbrSol                     = Circ.NbSolutions();
                        Results(Circ);
                    }
                    else
                    {
                        Geom2d_Line           LLon = (Con as Geom2d_Line)!;
                        Line2D              lon  = LLon.Lin2d();
                        GccAna_Circ2dTanOnRad Circ = new(Qc1, lon, Radius, Tolerance);
                        WellDone                   = Circ.IsDone();
                        NbrSol                     = Circ.NbSolutions();
                        Results(Circ);
                    }
                }
                else
                {
                    Geom2d_Line         LL1 = (CC1 as Geom2d_Line)!;
                    Line2D            l1  = LL1.Lin2d();
                    GccEnt_QualifiedLin Ql1 = new(l1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        Geom2d_Circle         CCon = (Con as Geom2d_Circle)!;
                        Circle2D             con  = CCon.Circ2d();
                        GccAna_Circ2dTanOnRad Circ = new(Ql1, con, Radius, Tolerance);
                        WellDone                   = Circ.IsDone();
                        NbrSol                     = Circ.NbSolutions();
                        Results(Circ);
                    }
                    else
                    {
                        Geom2d_Line           LLon = (Con as Geom2d_Line)!;
                        Line2D              lon  = LLon.Lin2d();
                        GccAna_Circ2dTanOnRad Circ = new(Ql1, lon, Radius, Tolerance);
                        WellDone                   = Circ.IsDone();
                        NbrSol                     = Circ.NbSolutions();
                        Results(Circ);
                    }
                }
            }

            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                if (Type1 == Circle)
                {
                    Geom2d_Circle               CCC1    = (CC1 as Geom2d_Circle)!;
                    Circle2D                   c1      = CCC1.Circ2d();
                    GccEnt_QualifiedCirc        Qc1     = new(c1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2dTanOnRadGeo CircGeo = new(Qc1, OnCurve, Radius, Tolerance);
                    WellDone                            = CircGeo.IsDone();
                    NbrSol                              = CircGeo.NbSolutions();
                    Results(CircGeo);
                }
                else if (Type1 == Line)
                {
                    Geom2d_Line                 LL1     = (CC1 as Geom2d_Line)!;
                    Line2D                    l1      = LL1.Lin2d();
                    GccEnt_QualifiedLin         Ql1     = new(l1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2dTanOnRadGeo CircGeo = new(Ql1, OnCurve, Radius, Tolerance);
                    WellDone                            = CircGeo.IsDone();
                    NbrSol                              = CircGeo.NbSolutions();
                    Results(CircGeo);
                }
                else
                {
                    Geom2dGcc_QCurve            Qc1     = new(C1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2dTanOnRadGeo CircGeo = new(Qc1, OnCurve, Radius, Tolerance);
                    WellDone                            = CircGeo.IsDone();
                    NbrSol                              = CircGeo.NbSolutions();
                    Results(CircGeo);
                }
            }
        }
    }

    /// <summary>Circles of radius Radius through Point1, centred on OnCurve.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRad(Geom2d_Point Point1, Geom2dAdaptor_Curve OnCurve, double Radius, double Tolerance)
        : this(8)
    {
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            Point2D          point1 = Point1.Pnt2d();
            CurveKind Type2  = OnCurve.GetType();
            Geom2d_Curve?     Con    = OnCurve.Curve();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            NbrSol = 0;
            if (Type2 == Line || Type2 == Circle)
            {
                if (Type2 == Circle)
                {
                    Geom2d_Circle         CCon = (Con as Geom2d_Circle)!;
                    Circle2D             con  = CCon.Circ2d();
                    GccAna_Circ2dTanOnRad Circ = new(point1, con, Radius, Tolerance);
                    WellDone                   = Circ.IsDone();
                    NbrSol                     = Circ.NbSolutions();
                    Results(Circ);
                }
                else
                {
                    Geom2d_Line           LLon = (Con as Geom2d_Line)!;
                    Line2D              lon  = LLon.Lin2d();
                    GccAna_Circ2dTanOnRad Circ = new(point1, lon, Radius, Tolerance);
                    WellDone                   = Circ.IsDone();
                    NbrSol                     = Circ.NbSolutions();
                    Results(Circ);
                }
            }

            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                Geom2dGcc_Circ2dTanOnRadGeo CircGeo = new(point1, OnCurve, Radius, Tolerance);
                WellDone                            = CircGeo.IsDone();
                NbrSol                              = CircGeo.NbSolutions();
                Results(CircGeo);
            }
        }
    }

    /// <summary>Copies the analytic solver's solutions (OCCT's public Results; it writes this solver).</summary>
    internal void Results(GccAna_Circ2dTanOnRad Circ)
    {
        for (int j = 1; j <= NbrSol; j++)
        {
            cirsol[j] = Circ.ThisSolution(j);
            if (Circ.IsTheSame1(j))
            {
                TheSame1[j] = 1;
            }
            else
            {
                TheSame1[j] = 0;
            }
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.CenterOn3(j, out parcen3[j], out pntcen3[j]);
            Circ.WhichQualifier(j, out qualifier1[j]);
        }
    }

    /// <summary>Copies the geometric solver's solutions (OCCT's public Results; it writes this solver).</summary>
    internal void Results(Geom2dGcc_Circ2dTanOnRadGeo Circ)
    {
        for (int j = 1; j <= NbrSol; j++)
        {
            cirsol[j] = Circ.ThisSolution(j);
            if (Circ.IsTheSame1(j))
            {
                TheSame1[j] = 1;
            }
            else
            {
                TheSame1[j] = 0;
            }
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.CenterOn3(j, out parcen3[j], out pntcen3[j]);
            Circ.WhichQualifier(j, out qualifier1[j]);
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
        return NbrSol;
    }

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Circle2D ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        return cirsol[Index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
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

    /// <summary>The tangency point between the solution of index Index and the argument, and its parameters.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself.</exception>
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
            if (TheSame1[Index] == 0)
            {
                ParSol = par1sol[Index];
                ParArg = pararg1[Index];
                PntSol = pnttg1sol[Index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The centre of the solution of index Index and its parameter on the curve it lies on.</summary>
    public void CenterOn3(int Index, out double ParArg, out Point2D PntSol)
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
            ParArg = parcen3[Index];
            PntSol = pntcen3[Index];
        }
    }

    /// <summary>True if the solution of index Index is the argument itself.</summary>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        if (TheSame1[Index] == 0)
        {
            return false;
        }
        return true;
    }
}
