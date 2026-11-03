// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRadGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRadGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// Circles of a given radius tangent to two arguments, at least one of them a free-form curve (the other a qualified
/// line, circle or curve, or a point): the centres are the intersections of the arguments' offsets by the radius
/// (Adaptor2d_OffsetCurve and Geom2dInt).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117), up to 16 solutions.</remarks>
public sealed class Geom2dGcc_Circ2d2TanRadGeo
{
    private const int aNbSolMAX = 16;

    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<GccEnt_Position> qualifier1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<GccEnt_Position> qualifier2 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<int> TheSame1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<int> TheSame2 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<Point2D> pnttg1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<Point2D> pnttg2sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> par1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> par2sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> pararg1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> pararg2 = new(1, aNbSolMAX);

    // circulaire tant a une courbe et une droite ,de rayon donne
    //==============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu2 et la droite L1.                            +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On fait la parallele a L1 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified line and a qualified curve.</summary>
    /// <exception cref="GccEnt_BadQualifier">The line is qualified enclosing.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRadGeo(in GccEnt_QualifiedLin Qualified1, in Geom2dGcc_QCurve Qualified2, double Radius, double Tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     Tol      = Math.Abs(Tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        WellDone                            = false;
        NbrSol                              = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Line2D            L1      = Qualified1.Qualified();
        double              x1dir   = (L1.Direction()).X();
        double              y1dir   = (L1.Direction()).Y();
        double              lxloc   = (L1.Location()).X();
        double              lyloc   = (L1.Location()).Y();
        Point2D            origin1 = new(lxloc, lyloc);
        Direction2D            normL1  = new(-y1dir, x1dir);
        CurveAdaptor2D Cu2     = Qualified2.Qualified();
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed())
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsEnclosed() && Qualified2.IsOutside())
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsEnclosed())
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsOutside())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = -Radius;
            }
            if (Qualified1.IsEnclosed() && Qualified2.IsUnqualified())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsEnclosed())
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsUnqualified())
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsOutside())
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsUnqualified() && Qualified2.IsUnqualified())
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            Direction2D Dir = new(-y1dir, x1dir);
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Point2D        Point = new(L1.Location().XY() + cote1[jcote1] * Dir.XY());
                Line2D        Line  = new(Point, L1.Direction()); // ligne avec deport.
                IntRes2d_Domain D1    = new();
                for (int jcote2 = 1; jcote2 <= nbrcote2 && NbrSol < aNbSolMAX; jcote2++)
                {
                    CurveAdaptor2D HCu2 = (CurveAdaptor2D)Cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D C2 = new(HCu2, -cote2[jcote2]);
                    firstparam = Max(C2.FirstParameter(), thefirst);
                    lastparam  = Min(C2.LastParameter(), thelast);
                    IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
                    Geom2dInt_TheIntConicCurveOfGInter Intp = new(Line, D1, C2, D2, Tol, Tol);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                            {
                                NbrSol++;
                                Point2D Center = Intp.Point(i).Value();
                                cirsol[NbrSol]  = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             =======================================================
                                Direction2D dc1       = new(origin1.XY() - Center.XY());
                                qualifier2[NbrSol] = Qualified2.Qualifier();
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
                                TheSame1[NbrSol]  = 0;
                                TheSame2[NbrSol]  = 0;
                                pararg1[NbrSol]   = Intp.Point(i).ParamOnFirst();
                                pararg2[NbrSol]   = Intp.Point(i).ParamOnSecond();
                                pnttg1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Value(pararg1[NbrSol], L1);
                                pnttg2sol[NbrSol] = Geom2dGcc_CurveTool.Value(Cu2, pararg2[NbrSol]);
                                par1sol[NbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                par2sol[NbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }

    // circulaire tant a une courbe et un cercle ,de rayon donne
    //=============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu2 et le cercle C1.                            +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On fait la parallele a C1 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified circle and a qualified curve.</summary>
    /// <exception cref="GccEnt_BadQualifier">A qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRadGeo(in GccEnt_QualifiedCirc Qualified1, in Geom2dGcc_QCurve Qualified2, double Radius, double Tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     Tol      = Math.Abs(Tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        WellDone                            = false;
        NbrSol                              = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        Circle2D           C1      = Qualified1.Qualified();
        Point2D            center1 = C1.Location();
        CurveAdaptor2D Cu2     = Qualified2.Qualified();
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed())
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsEnclosed() && Qualified2.IsOutside())
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsEnclosed())
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsOutside())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = -Radius;
            }
            if (Qualified1.IsEnclosed() && Qualified2.IsUnqualified())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsEnclosed())
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsUnqualified())
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsOutside())
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsUnqualified() && Qualified2.IsUnqualified())
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            double                             R1   = C1.Radius();
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1 && NbrSol < aNbSolMAX; jcote1++)
            {
                Circle2D       Circ = new(C1.XAxis(), R1 + cote1[jcote1]);
                IntRes2d_Domain D1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Circ), 0.0, Tol, global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, Circ), 2.0 * M_PI, Tol);
                D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                for (int jcote2 = 1; jcote2 <= nbrcote2; jcote2++)
                {
                    CurveAdaptor2D HCu2 = (CurveAdaptor2D)Cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D C2 = new(HCu2, -cote2[jcote2]);
                    firstparam = Max(C2.FirstParameter(), thefirst);
                    lastparam  = Min(C2.LastParameter(), thelast);
                    IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
                    Intp.Perform(Circ, D1, C2, D2, Tol, Tol);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                            {
                                NbrSol++;
                                Point2D Center = Intp.Point(i).Value();
                                cirsol[NbrSol]  = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             =======================================================
                                double distcc1 = Center.Distance(center1);
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
                                qualifier2[NbrSol] = Qualified2.Qualifier();
                                TheSame1[NbrSol]   = 0;
                                TheSame2[NbrSol]   = 0;
                                pararg1[NbrSol]    = Intp.Point(i).ParamOnFirst();
                                pararg2[NbrSol]    = Intp.Point(i).ParamOnSecond();
                                pnttg1sol[NbrSol]  = global::Kerf.Engine.Geometry.ElCLib.Value(pararg1[NbrSol], C1);
                                pnttg2sol[NbrSol]  = Geom2dGcc_CurveTool.Value(Cu2, pararg2[NbrSol]);
                                par1sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                par2sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }

    // circulaire tant a une courbe et un point ,de rayon donne
    //============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu1 et le point P2.                             +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu1 dans le bon sens.                          +
    // On fait la parallele a P2 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified curve and passing through a point.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRadGeo(in Geom2dGcc_QCurve Qualified1, in Point2D Point2, double Radius, double Tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     Tol      = Math.Abs(Tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        int                        nbrcote1 = 0;
        WellDone                            = false;
        NbrSol                              = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (Qualified1.IsEnclosed())
            {
                //    ===========================
                nbrcote1 = 1;
                cote1[1] = Radius;
            }
            else if (Qualified1.IsOutside())
            {
                //   ===============================
                nbrcote1 = 1;
                cote1[1] = -Radius;
            }
            else if (Qualified1.IsUnqualified())
            {
                //   ===================================
                nbrcote1 = 2;
                cote1[1] = Radius;
                cote1[2] = -Radius;
            }
            Circle2D       Circ = new(new Axis2D(Point2, new Direction2D(Direction2D.D.X)), Radius);
            IntRes2d_Domain D1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, Circ), 0.0, Tol, global::Kerf.Engine.Geometry.ElCLib.Value(M_PI + M_PI, Circ), M_PI + M_PI, Tol);
            D1.SetEquivalentParameters(0.0, M_PI + M_PI);
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1 && NbrSol < aNbSolMAX; jcote1++)
            {
                CurveAdaptor2D HCu1 = (CurveAdaptor2D)Cu1.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu1)
                // Adaptor2d_OffsetCurve Cu2(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D Cu2 = new(HCu1, -cote1[jcote1]);
                firstparam = Max(Cu2.FirstParameter(), thefirst);
                lastparam  = Min(Cu2.LastParameter(), thelast);
                IntRes2d_Domain D2 = new(Cu2.Value(firstparam), firstparam, Tol, Cu2.Value(lastparam), lastparam, Tol);
                Intp.Perform(Circ, D1, Cu2, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            Point2D Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new Circle2D(new Axis2D(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                            TheSame1[NbrSol]   = 0;
                            TheSame2[NbrSol]   = 0;
                            pararg1[NbrSol]    = Intp.Point(i).ParamOnSecond();
                            pararg2[NbrSol]    = 0.0;
                            pnttg1sol[NbrSol]  = Geom2dGcc_CurveTool.Value(Cu1, pararg1[NbrSol]);
                            pnttg2sol[NbrSol]  = Point2;
                            par1sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            par2sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    //=======================================================================
    // function : PrecRoot
    // purpose  : In case, when curves has tangent zones, intersection point
    //            found may be precised. This function uses precision algorithm
    //            of Extrema Curve-Curve method (dot product between every
    //            tangent vector and vector between points in two curves must
    //            be equal to zero).
    //=======================================================================
    private static void PrecRoot(OffsetCurveAdaptor2D theC1,
                                 OffsetCurveAdaptor2D theC2,
                                 double                theU0,
                                 double                theV0,
                                 out double            theUfinal,
                                 out double            theVfinal)
    {
        // It is necessary for precision to solve the system
        //     (x1(u) - x2(v)) * x1'(u) + (y1(u) - y2(v)) * y1'(u) = 0
        //     (x1(u) - x2(v)) * x2'(v) + (y1(u) - y2(v)) * y2'(v) = 0
        // by Newton's method (OCCT's comment gives the iteration of any 2 x 2 system).

        theUfinal = theU0;
        theVfinal = theV0;

        const int aNbIterMax = 100;

        double aU = theU0, aV = theV0;

        int aNbIter = 0;

        double aStepU = 0.0, aStepV = 0.0;

        double aSQDistPrev = RealFirst();

        theC1.D2(aU, out Point2D aPu, out Vector2D aD1u, out Vector2D aD2u);
        theC2.D2(aV, out Point2D aPv, out Vector2D aD1v, out Vector2D aD2v);

        double aCrProd = Math.Abs(aD1u.Crossed(aD1v));
        if (aCrProd * aCrProd > 1.0e-6 * aD1u.SquareMagnitude() * aD1v.SquareMagnitude())
        {
            // Curves are not tangent. Therefore, we consider that
            // 2D-intersection algorithm have found good point which
            // did not need in more precision.
            return;
        }

        do
        {
            aNbIter++;

            Vector2D aVuv = new(aPv, aPu);

            double aSQDist = aVuv.SquareMagnitude();
            if (IsEqual(aSQDist, 0.0))
            {
                break;
            }

            if ((aNbIter == 1) || (aSQDist < aSQDistPrev))
            {
                aSQDistPrev = aSQDist;
                theUfinal   = aU;
                theVfinal   = aV;
            }

            double aG1 = aD1u.Magnitude();
            double aG2 = aD1v.Magnitude();

            if (IsEqual(aG1, 0.0) || IsEqual(aG2, 0.0))
            { // Here we do not processing singular cases.
                break;
            }

            double aF1 = aVuv.Dot(aD1u);
            double aF2 = aVuv.Dot(aD1v);

            double aFIu  = aVuv.Dot(aD2u);
            double aFIv  = aVuv.Dot(aD2v);
            double aPSIu = aD1u.Dot(aD2u);
            double aPSIv = aD1v.Dot(aD2v);

            double aTheta = aD1u * aD1v;

            double aS1 = aF1 / aG1;
            double aS2 = aF2 / aG2;

            double aDS1u = (aG1 * aG1 + aFIu) / aG1 - (aS1 * aPSIu / (aG1 * aG1));
            double aDS1v = -aTheta / aG1;
            double aDS2u = aTheta / aG2;
            double aDS2v = (aFIv - aG2 * aG2) / aG2 - (aS2 * aPSIv / (aG2 * aG2));

            double aDet = aDS1u * aDS2v - aDS1v * aDS2u;

            if (IsEqual(aDet, 0.0))
            {
                if (!IsEqual(aStepV, 0.0) && !IsEqual(aDS1u, 0.0))
                {
                    aV += aStepV;
                    aU = aU - (aDS1v * aStepV - aS1) / aDS1u;
                }
                else if (!IsEqual(aStepU, 0.0) && !IsEqual(aDS1v, 0.0))
                {
                    aU += aStepU;
                    aV = aV - (aDS1u * aStepU - aS1) / aDS1v;
                }
                else
                {
                    break;
                }
            }
            else
            {
                aStepU = -(aS1 * aDS2v - aS2 * aDS1v) / aDet;
                aStepV = -(aS2 * aDS1u - aS1 * aDS2u) / aDet;

                if (Math.Abs(aStepU) < Epsilon(Math.Abs(aU)))
                {
                    if (Math.Abs(aStepV) < Epsilon(Math.Abs(aV)))
                    {
                        break;
                    }
                }

                aU += aStepU;
                aV += aStepV;
            }

            theC1.D2(aU, out aPu, out aD1u, out aD2u);
            theC2.D2(aV, out aPv, out aD1v, out aD2v);
        } while (aNbIter <= aNbIterMax);
    }

    // circulaire tant a deux courbes ,de rayon donne
    //==================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere les courbes Cu1 et Cu2.                                   +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu1 dans le bon sens.                          +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to two qualified curves.</summary>
    /// <exception cref="GccEnt_BadQualifier">A qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Geom2dGcc_Circ2d2TanRadGeo(in Geom2dGcc_QCurve Qualified1, in Geom2dGcc_QCurve Qualified2, double Radius, double Tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     Tol      = Math.Abs(Tolerance);
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        WellDone                            = false;
        NbrSol                              = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        CurveAdaptor2D Cu1 = Qualified1.Qualified();
        CurveAdaptor2D Cu2 = Qualified2.Qualified();
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (Qualified1.IsEnclosed() && Qualified2.IsEnclosed())
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsEnclosed() && Qualified2.IsOutside())
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsEnclosed())
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsOutside())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -Radius;
                cote2[1] = -Radius;
            }
            if (Qualified1.IsEnclosed() && Qualified2.IsUnqualified())
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsEnclosed())
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
            }
            else if (Qualified1.IsOutside() && Qualified2.IsUnqualified())
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            if (Qualified1.IsUnqualified() && Qualified2.IsOutside())
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = -Radius;
            }
            else if (Qualified1.IsUnqualified() && Qualified2.IsUnqualified())
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = Radius;
                cote1[2] = -Radius;
                cote2[1] = Radius;
                cote2[2] = -Radius;
            }
            Geom2dInt_GInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                CurveAdaptor2D HCu1 = (CurveAdaptor2D)Cu1.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu1)
                // Adaptor2d_OffsetCurve C1(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D C1 = new(HCu1, -cote1[jcote1]);
                for (int jcote2 = 1; jcote2 <= nbrcote2 && NbrSol < aNbSolMAX; jcote2++)
                {
                    CurveAdaptor2D HCu2 = (CurveAdaptor2D)Cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D C2 = new(HCu2, -cote2[jcote2]);
                    Intp.Perform(C1, C2, Tol, Tol);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            double aSQApproxTol = global::Kerf.Engine.Foundation.Precision.Approximation() * global::Kerf.Engine.Foundation.Precision.Approximation();
                            for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                            {
                                double aU0 = Intp.Point(i).ParamOnFirst();
                                double aV0 = Intp.Point(i).ParamOnSecond();

                                double aU1 = aU0 - global::Kerf.Engine.Foundation.Precision.PApproximation();
                                double aV1 = aV0 - global::Kerf.Engine.Foundation.Precision.PApproximation();

                                double aU2 = aU0 + global::Kerf.Engine.Foundation.Precision.PApproximation();
                                double aV2 = aV0 + global::Kerf.Engine.Foundation.Precision.PApproximation();

                                Point2D P11 = C1.Value(aU1);
                                Point2D P12 = C2.Value(aV1);
                                Point2D P21 = C1.Value(aU2);
                                Point2D P22 = C2.Value(aV2);

                                double aDist1112 = P11.SquareDistance(P12);
                                double aDist1122 = P11.SquareDistance(P22);

                                double aDist1221 = P12.SquareDistance(P21);
                                double aDist2122 = P21.SquareDistance(P22);

                                if ((Min(aDist1112, aDist1122) <= aSQApproxTol)
                                    && (Min(aDist1221, aDist2122) <= aSQApproxTol))
                                {
                                    PrecRoot(C1, C2, aU0, aV0, out aU0, out aV0);
                                }

                                NbrSol++;
                                Point2D Center = C1.Value(aU0);
                                cirsol[NbrSol]  = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             =======================================================
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                                // OCCT: sets qualifier1 twice and never qualifier2, which stays uninitialised
                                // natively (0, GccEnt_unqualified, in the port).
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                                TheSame1[NbrSol]   = 0;
                                TheSame2[NbrSol]   = 0;
                                // The tangency data are taken at the intersection's parameters, not PrecRoot's.
                                pararg1[NbrSol]    = Intp.Point(i).ParamOnFirst();
                                pararg2[NbrSol]    = Intp.Point(i).ParamOnSecond();
                                pnttg1sol[NbrSol]  = Geom2dGcc_CurveTool.Value(Cu1, pararg1[NbrSol]);
                                pnttg2sol[NbrSol]  = Geom2dGcc_CurveTool.Value(Cu2, pararg2[NbrSol]);
                                par1sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                par2sol[NbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            }
                        }

                        WellDone = true;
                    }
                }
            }
        }
    }

    //=========================================================================

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

    /// <summary>The qualifiers of the arguments for the solution of index Index.</summary>
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
            Qualif1 = qualifier1[Index];
            Qualif2 = qualifier2[Index];
        }
    }

    /// <summary>The tangency point on the first argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument.</exception>
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

    /// <summary>The tangency point on the second argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument.</exception>
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

        if (TheSame1[Index] == 0)
        {
            return false;
        }
        return true;
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

        if (TheSame2[Index] == 0)
        {
            return false;
        }
        return true;
    }
}
