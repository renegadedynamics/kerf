// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// 2D circles of a given radius tangent to two arguments, each a qualified curve or a point (passed through). Lines
/// and circles go to GccAna_Circ2d2TanRad; a free-form argument to Geom2dGcc_Circ2d2TanRadGeo (the intersections of
/// the arguments' offsets). Each solution comes with the qualifiers of its arguments and its tangency points.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117), up to 16 solutions (2 for two points). Where the
/// second argument is the line or circle and the first is not, the solver is called with the arguments swapped and
/// the accessors swap them back.
/// </remarks>
public sealed class Geom2dGcc_Circ2d2TanRad
{
    private const int aNbSolMAX = 16;

    private bool WellDone;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private int NbrSol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pnttg2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;
    private bool Invert;

    // The arrays, sized as each constructor's initialiser list sizes them.
    private Geom2dGcc_Circ2d2TanRad(int theNbSol)
    {
        cirsol     = new(1, theNbSol);
        qualifier1 = new(1, theNbSol);
        qualifier2 = new(1, theNbSol);
        TheSame1   = new(1, theNbSol);
        TheSame2   = new(1, theNbSol);
        pnttg1sol  = new(1, theNbSol);
        pnttg2sol  = new(1, theNbSol);
        par1sol    = new(1, theNbSol);
        par2sol    = new(1, theNbSol);
        pararg1    = new(1, theNbSol);
        pararg2    = new(1, theNbSol);
    }

    // circulaire tangent a deux cercles et de rayon donne
    //====================================================
    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere le cercle C1 et le cercle C2.                             +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On distingue les cas limites pour les triater separement.             +
    // On fait la parallele a C1 dans le bon sens.                           +
    // On fait la parallele a C2 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to two qualified curves.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    /// <exception cref="GccEnt_BadQualifier">A line is qualified enclosing.</exception>
    public Geom2dGcc_Circ2d2TanRad(in Geom2dGcc_QualifiedCurve Qualified1,
                                   in Geom2dGcc_QualifiedCurve Qualified2,
                                   double Radius,
                                   double Tolerance)
        : this(aNbSolMAX)
    {
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            CurveAdaptor2D C1    = Qualified1.Qualified();
            CurveAdaptor2D C2    = Qualified2.Qualified();
            Curve2D?       CC1   = C1.Curve();
            Curve2D?       CC2   = C2.Curve();
            CurveKind   Type1 = C1.GetType();
            CurveKind   Type2 = C2.GetType();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            Invert = false;
            NbrSol = 0;
            if ((Type1 == Line || Type1 == Circle)
                && (Type2 == Line || Type2 == Circle))
            {
                if (Type1 == Circle)
                {
                    CircleCurve2D        CCC1 = (CC1 as CircleCurve2D)!;
                    Circle2D            c1   = CCC1.Circ2d();
                    GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        CircleCurve2D        CCC2    = (CC2 as CircleCurve2D)!;
                        Circle2D            c2      = CCC2.Circ2d();
                        GccAna_Circ2d2TanRad CircAna = new(Qc1, new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()), Radius, Tolerance);
                        WellDone = CircAna.IsDone();
                        NbrSol   = CircAna.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                        }
                        Results(CircAna);
                    }
                    else
                    {
                        LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                        Line2D    l2  = LL2.Lin2d();
                        if (!Qualified2.IsEnclosing())
                        {
                            GccAna_Circ2d2TanRad CircAna = new(Qc1, new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()), Radius, Tolerance);
                            WellDone = CircAna.IsDone();
                            NbrSol   = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                        else
                        {
                            WellDone = false;
                            throw new GccEnt_BadQualifier();
                        }
                    }
                }
                else
                {
                    LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                    Line2D    l1  = LL1.Lin2d();
                    if (Qualified1.IsEnclosing())
                    {
                        WellDone = false;
                        throw new GccEnt_BadQualifier();
                    }
                    else
                    {
                        GccEnt_QualifiedLin Ql1 = new(l1, Qualified1.Qualifier());
                        if (Type2 == Circle)
                        {
                            CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                            Circle2D     c2   = CCC2.Circ2d();
                            Invert             = true;
                            GccAna_Circ2d2TanRad CircAna = new(new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()), Ql1, Radius, Tolerance);
                            WellDone = CircAna.IsDone();
                            NbrSol   = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                        else
                        {
                            LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                            Line2D    l2  = LL2.Lin2d();
                            if (!Qualified2.IsEnclosing())
                            {
                                GccAna_Circ2d2TanRad CircAna = new(Ql1, new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()), Radius, Tolerance);
                                WellDone = CircAna.IsDone();
                                NbrSol   = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                            else
                            {
                                WellDone = false;
                                throw new GccEnt_BadQualifier();
                            }
                        }
                    }
                }
            }
            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================
            else
            {
                if (Type1 == Line)
                {
                    LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                    Line2D    l1  = LL1.Lin2d();
                    if (Qualified1.IsEnclosing())
                    {
                        WellDone = false;
                        throw new GccEnt_BadQualifier();
                    }
                    else
                    {
                        GccEnt_QualifiedLin        Ql1     = new(l1, Qualified1.Qualifier());
                        Geom2dGcc_QCurve           Qc2     = new(C2, Qualified2.Qualifier());
                        Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Ql1, Qc2, Radius, Tolerance);
                        WellDone = CircGeo.IsDone();
                        NbrSol   = CircGeo.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                        }
                        Results(CircGeo);
                    }
                }
                else if (Type1 == Circle)
                {
                    CircleCurve2D              CCC1    = (CC1 as CircleCurve2D)!;
                    Circle2D                  c1      = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1     = new(c1, Qualified1.Qualifier());
                    Geom2dGcc_QCurve           Qc2     = new(C2, Qualified2.Qualifier());
                    Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Qc1, Qc2, Radius, Tolerance);
                    WellDone = CircGeo.IsDone();
                    NbrSol   = CircGeo.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircGeo);
                }
                else if (Type2 == Line)
                {
                    Invert          = true;
                    LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                    Line2D    l2  = LL2.Lin2d();
                    if (Qualified2.IsEnclosing())
                    {
                        WellDone = false;
                        throw new GccEnt_BadQualifier();
                    }
                    else
                    {
                        GccEnt_QualifiedLin        Ql2     = new(l2, Qualified2.Qualifier());
                        Geom2dGcc_QCurve           Qc1     = new(C1, Qualified1.Qualifier());
                        Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Ql2, Qc1, Radius, Tolerance);
                        WellDone = CircGeo.IsDone();
                        NbrSol   = CircGeo.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                        }
                        Results(CircGeo);
                    }
                }
                else if (Type2 == Circle)
                {
                    Invert                             = true;
                    CircleCurve2D              CCC2    = (CC2 as CircleCurve2D)!;
                    Circle2D                  c2      = CCC2.Circ2d();
                    GccEnt_QualifiedCirc       Qc2     = new(c2, Qualified2.Qualifier());
                    Geom2dGcc_QCurve           Qc1     = new(C1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Qc2, Qc1, Radius, Tolerance);
                    WellDone = CircGeo.IsDone();
                    NbrSol   = CircGeo.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircGeo);
                }
                else
                {
                    Geom2dGcc_QCurve           Qc1     = new(C1, Qualified1.Qualifier());
                    Geom2dGcc_QCurve           Qc2     = new(C2, Qualified2.Qualifier());
                    Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Qc1, Qc2, Radius, Tolerance);
                    WellDone = CircGeo.IsDone();
                    NbrSol   = CircGeo.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircGeo);
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to a qualified curve and passing through a point.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRad(in Geom2dGcc_QualifiedCurve Qualified1, Point2 Point, double Radius, double Tolerance)
        : this(aNbSolMAX)
    {
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            CurveAdaptor2D C1    = Qualified1.Qualified();
            Curve2D?       CC1   = C1.Curve();
            CurveKind   Type1 = C1.GetType();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            Invert = false;
            NbrSol = 0;
            if (Type1 == Line || Type1 == Circle)
            {
                if (Type1 == Circle)
                {
                    CircleCurve2D        CCC1    = (CC1 as CircleCurve2D)!;
                    Circle2D            c1      = CCC1.Circ2d();
                    GccEnt_QualifiedCirc Qc1     = new(c1, Qualified1.Qualifier());
                    GccAna_Circ2d2TanRad CircAna = new(Qc1, Point.Pnt2d(), Radius, Tolerance);
                    WellDone = CircAna.IsDone();
                    NbrSol   = CircAna.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircAna);
                }
                else
                {
                    LineCurve2D          LLL1    = (CC1 as LineCurve2D)!;
                    Line2D             l1      = LLL1.Lin2d();
                    GccEnt_QualifiedLin  Ql1     = new(l1, Qualified1.Qualifier());
                    GccAna_Circ2d2TanRad CircAna = new(Ql1, Point.Pnt2d(), Radius, Tolerance);
                    WellDone = CircAna.IsDone();
                    NbrSol   = CircAna.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircAna);
                }
            }
            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================
            else
            {
                Geom2dGcc_QCurve           Qc1     = new(C1, Qualified1.Qualifier());
                Geom2dGcc_Circ2d2TanRadGeo CircGeo = new(Qc1, Point.Pnt2d(), Radius, Tolerance);
                WellDone = CircGeo.IsDone();
                NbrSol   = CircGeo.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                }
                Results(CircGeo);
            }
        }
    }

    /// <summary>Circles of radius Radius passing through two points.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRad(Point2 Point1, Point2 Point2, double Radius, double Tolerance)
        : this(2)
    {
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            Invert = false;
            NbrSol = 0;
            GccAna_Circ2d2TanRad CircAna = new(Point1.Pnt2d(), Point2.Pnt2d(), Radius, Tolerance);
            WellDone = CircAna.IsDone();
            NbrSol   = CircAna.NbSolutions();
            for (int i = 1; i <= NbrSol; i++)
            {
                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
            }
            Results(CircAna);
        }
    }

    /// <summary>Copies the analytic solver's solutions (OCCT's public Results; it writes the solver, so internal).</summary>
    internal void Results(GccAna_Circ2d2TanRad Circ)
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
            if (Circ.IsTheSame2(j))
            {
                TheSame2[j] = 1;
            }
            else
            {
                TheSame2[j] = 0;
            }
            // OCCT: asks for both tangencies even when the solution is an argument (TheSame), where GccAna's
            // accessors throw StdFail_NotDone: the constructor then throws.
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.Tangency2(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
        }
    }

    /// <summary>Copies the geometric solver's solutions (OCCT's public Results; it writes the solver, so internal).</summary>
    internal void Results(Geom2dGcc_Circ2d2TanRadGeo Circ)
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
            if (Circ.IsTheSame2(j))
            {
                TheSame2[j] = 1;
            }
            else
            {
                TheSame2[j] = 0;
            }
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.Tangency2(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
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

    /// <summary>The solution of index Index (1 to NbSolutions).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The qualifiers of the two arguments for the solution of index Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
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
            if (Invert)
            {
                Qualif1 = qualifier2[Index];
                Qualif2 = qualifier1[Index];
            }
            else
            {
                Qualif1 = qualifier1[Index];
                Qualif2 = qualifier2[Index];
            }
        }
    }

    /// <summary>The tangency point on the first argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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
            if (Invert)
            {
                if (TheSame2[Index] == 0)
                {
                    ParSol = par2sol[Index];
                    ParArg = pararg2[Index];
                    PntSol = pnttg2sol[Index];
                }
                else
                {
                    throw new NotDoneException();
                }
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
    }

    /// <summary>The tangency point on the second argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
    public void Tangency2(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
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
            if (!Invert)
            {
                if (TheSame2[Index] == 0)
                {
                    ParSol = par2sol[Index];
                    ParArg = pararg2[Index];
                    PntSol = pnttg2sol[Index];
                }
                else
                {
                    throw new NotDoneException();
                }
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
    }

    /// <summary>True if the solution of index Index is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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
        if (Invert)
        {
            return TheSame2[Index] != 0;
        }
        else
        {
            return TheSame1[Index] != 0;
        }
    }

    /// <summary>True if the solution of index Index is the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
    public bool IsTheSame2(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        if (!Invert)
        {
            return TheSame2[Index] != 0;
        }
        else
        {
            return TheSame1[Index] != 0;
        }
    }
}
