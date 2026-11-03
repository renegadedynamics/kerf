// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// 2D circles tangent to a qualified curve and centred on a point. Lines and circles go to GccAna_Circ2dTanCen, other
/// curves to Geom2dGcc_Circ2dTanCenGeo.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 2 solutions.</remarks>
public sealed class Geom2dGcc_Circ2dTanCen
{
    private readonly bool WellDone;
    private readonly int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;

    /// <summary>Circles centred on PCenter tangent to the qualified curve.</summary>
    public Geom2dGcc_Circ2dTanCen(in Geom2dGcc_QualifiedCurve Qualified1, Point2 PCenter, double Tolerance)
    {
        cirsol     = new(1, 2);
        qualifier1 = new(1, 2);
        TheSame1   = new(1, 2);
        pnttg1sol  = new(1, 2);
        par1sol    = new(1, 2);
        pararg1    = new(1, 2);

        CurveAdaptor2D C1    = Qualified1.Qualified();
        Curve2D?       CC1   = C1.Curve();
        CurveKind   Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Point2D pcenter = PCenter.Pnt2d();
        NbrSol           = 0;
        if ((Type1 == Line || Type1 == Circle))
        {
            if (Type1 == Circle)
            {
                CircleCurve2D        CCC1 = (CC1 as CircleCurve2D)!;
                Circle2D            c1   = CCC1.Circ2d();
                GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
                GccAna_Circ2dTanCen  Circ = new(Qc1, pcenter, Tolerance);
                WellDone                  = Circ.IsDone();
                NbrSol                    = Circ.NbSolutions();
                for (int j = 1; j <= NbrSol; j++)
                {
                    cirsol[j] = Circ.ThisSolution(j);
                    Circ.WhichQualifier(j, out qualifier1[j]);
                    if (Circ.IsTheSame1(j))
                    {
                        TheSame1[j] = 1;
                    }
                    else
                    {
                        TheSame1[j] = 0;
                    }
                    Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                }
            }
            else
            {
                LineCurve2D         LL1  = (CC1 as LineCurve2D)!;
                Line2D            l1   = LL1.Lin2d();
                GccAna_Circ2dTanCen Circ = new(l1, pcenter);
                WellDone                 = Circ.IsDone();
                NbrSol                   = Circ.NbSolutions();
                for (int j = 1; j <= NbrSol; j++)
                {
                    cirsol[j] = Circ.ThisSolution(j);
                    Circ.WhichQualifier(j, out qualifier1[j]);
                    if (Circ.IsTheSame1(j))
                    {
                        TheSame1[j] = 1;
                    }
                    else
                    {
                        TheSame1[j] = 0;
                    }
                    Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                }
            }
        }

        //=============================================================================
        //                            Appel a GccGeo.                                 +
        //=============================================================================

        else
        {
            Geom2dGcc_QCurve          Qc1  = new(C1, Qualified1.Qualifier());
            Geom2dGcc_Circ2dTanCenGeo Circ = new(Qc1, pcenter, Tolerance);
            WellDone                       = Circ.IsDone();
            NbrSol                         = Circ.NbSolutions();
            for (int j = 1; j <= NbrSol; j++)
            {
                cirsol[j]   = Circ.ThisSolution(j);
                TheSame1[j] = 0;
                Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                Circ.WhichQualifier(j, out qualifier1[j]);
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
