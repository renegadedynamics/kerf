// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

//=========================================================================
//   Creation d un cercle tangent a deux elements : Droite.               +
//                                                  Cercle.               +
//                                                  Point.                +
//                                                  Courbes.              +
//                        centre sur un troisieme : Droite.               +
//                                                  Cercle.               +
//                                                  Courbes.              +
//=========================================================================

/// <summary>
/// Circles tangent to two arguments (qualified circles or lines, or points) whose centres lie on a curve: the
/// bisectors of the two arguments (GccAna) intersected with the curve (Geom2dInt).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117), up to 8 solutions.</remarks>
public sealed class Geom2dGcc_Circ2d2TanOnGeo
{
    private const int aNbSolMAX = 8;

    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Circ2d> cirsol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<GccEnt_Position> qualifier1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<GccEnt_Position> qualifier2 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<int> TheSame1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<int> TheSame2 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<gp_Pnt2d> pnttg2sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<gp_Pnt2d> pntcen = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> par1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> par2sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> pararg1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> pararg2 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> parcen3 = new(1, aNbSolMAX);

    /// <summary>Circles tangent to two qualified circles, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedCirc Qualified2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {
        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double Tol = Math.Abs(Tolerance);
        NbrSol     = 0;
        NCollection_Array1<double> Rbid = new(1, 2); // OCCT: Rbid(2) and RBid(2) are read (nbsol 2) where unset: uninitialised natively, 0 in the port
        NCollection_Array1<double> RBid = new(1, 2);
        NCollection_Array1<double> Radius = new(1, 2);
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Circ2d          C1 = Qualified1.Qualified();
        gp_Circ2d          C2 = Qualified2.Qualified();
        double             R1 = C1.Radius();
        double             R2 = C2.Radius();
        gp_Dir2d           dirx = new(gp_Dir2d.D.X);
        gp_Pnt2d           center1 = C1.Location();
        gp_Pnt2d           center2 = C2.Location();
        GccAna_Circ2dBisec Bis = new(C1, C2);
        if (Bis.IsDone())
        {
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            int                                nbsolution = Bis.NbSolutions();
            Geom2dAdaptor_Curve   HCu2       = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              Cu2 = new(HCu2, 0.0);
            firstparam = Max(Cu2.FirstParameter(), thefirst);
            lastparam  = Min(Cu2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(Cu2.Value(firstparam),
                                                  firstparam,
                                                  Tol,
                                                  Cu2.Value(lastparam),
                                                  lastparam,
                                                  Tol);
            double          Tol1 = Math.Abs(Tolerance);
            double          Tol2 = Tol1;
            for (int i = 1; i <= nbsolution; i++)
            {
                GccInt_Bisec              Sol  = Bis.ThisSolution(i);
                GccInt_IType              type = Sol.ArcType();
                switch (type)
                {
                    case GccInt_Cir:
                    {
                        gp_Circ2d       Circ = Sol.Circle();
                        IntRes2d_Domain D1 = new(ElCLib.Value(0.0, Circ),
                                                              0.0,
                                                              Tol1,
                                                              ElCLib.Value(2.0 * M_PI, Circ),
                                                              2.0 * M_PI,
                                                              Tol2);
                        D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        Intp.Perform(Circ, D1, Cu2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Ell:
                    {
                        gp_Elips2d      Elips = Sol.Ellipse();
                        IntRes2d_Domain D1 = new(ElCLib.Value(0.0, Elips),
                                                              0.0,
                                                              Tol1,
                                                              ElCLib.Value(2.0 * M_PI, Elips),
                                                              2.0 * M_PI,
                                                              Tol2);
                        D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        Intp.Perform(Elips, D1, Cu2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Hpr:
                    {
                        gp_Hypr2d       Hypr = Sol.Hyperbola();
                        IntRes2d_Domain D1 = new(ElCLib.Value(-4.0, Hypr),
                                                              -4.0,
                                                              Tol1,
                                                              ElCLib.Value(4.0, Hypr),
                                                              4.0,
                                                              Tol2);
                        Intp.Perform(Hypr, D1, Cu2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Lin:
                    {
                        gp_Lin2d        Line = Sol.Line();
                        IntRes2d_Domain D1 = new();
                        Intp.Perform(Line, D1, Cu2, D2, Tol1, Tol2);
                    }
                    break;
                    default:
                    {
                        throw new Standard_ConstructionError();
                    }
                }
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()))
                    {
                        for (int j = 1; j <= Intp.NbPoints(); j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double   dist1 = Center.Distance(C1.Location());
                            double   dist2 = Center.Distance(C2.Location());
                            int      nbsol = 0;
                            int      nnsol = 0;
                            R1             = C1.Radius();
                            R2             = C2.Radius();
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 < Tol)
                                {
                                    nbsol   = 1;
                                    Rbid[1] = Math.Abs(R1 - dist1);
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 < Tol)
                                {
                                    nbsol   = 1;
                                    Rbid[1] = Math.Abs(dist1 - R1);
                                }
                            }
                            else if (Qualified1.IsEnclosing())
                            {
                                nbsol   = 1;
                                Rbid[1] = dist1 + R1;
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                nbsol   = 2;
                                Rbid[1] = dist1 + R1;
                                Rbid[1] = Math.Abs(dist1 - R1); // OCCT: overwrites Rbid(1); Rbid(2) is never set
                            }
                            if (Qualified2.IsEnclosed() && nbsol != 0)
                            {
                                if (dist2 - R2 < Tol)
                                {
                                    RBid[1] = Math.Abs(R2 - dist2);
                                }
                            }
                            else if (Qualified2.IsOutside() && nbsol != 0)
                            {
                                if (R2 - dist2 < Tol)
                                {
                                    RBid[1] = Math.Abs(R2 - dist2);
                                }
                            }
                            else if (Qualified2.IsEnclosing() && nbsol != 0)
                            {
                                RBid[1] = dist2 + R2;
                            }
                            else if (Qualified2.IsUnqualified() && nbsol != 0)
                            {
                                RBid[1] = dist2 + R2;
                                RBid[2] = Math.Abs(R2 - dist2); // OCCT: set only here; read whenever nbsol is 2
                            }
                            for (int isol = 1; isol <= nbsol; isol++)
                            {
                                for (int jsol = 1; jsol <= nbsol; jsol++)
                                {
                                    if (Math.Abs(Rbid[isol] - RBid[jsol]) <= Tol)
                                    {
                                        nnsol++;
                                        Radius[nnsol] = (RBid[jsol] + Rbid[isol]) / 2.0; // OCCT: up to 4 radii into an array of 2 (Standard_OutOfRange, natively too)
                                    }
                                }
                            }
                            if (nnsol > 0)
                            {
                                for (int k = 1; k <= nnsol && NbrSol < aNbSolMAX; k++)
                                {
                                    NbrSol++;
                                    cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius[k]);
                                    //              ==========================================================
                                    double distcc1 = Center.Distance(center1);
                                    double distcc2 = Center.Distance(center2);
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (Math.Abs(distcc1 + Radius[i] - R1) < Tol) // OCCT: Radius(i) with the bisector's index i, not k (out of range for i > 2)
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    else if (Math.Abs(distcc1 - R1 - Radius[i]) < Tol)
                                    {
                                        qualifier1[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosing;
                                    }
                                    if (!Qualified2.IsUnqualified())
                                    {
                                        qualifier2[NbrSol] = Qualified2.Qualifier();
                                    }
                                    else if (Math.Abs(distcc2 + Radius[i] - R2) < Tol) // OCCT: Radius(i), as above
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                    else if (Math.Abs(distcc2 - R2 - Radius[i]) < Tol)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosing;
                                    }
                                    if (dist1 <= Tol && Math.Abs(Radius[k] - C1.Radius()) <= Tol)
                                    {
                                        TheSame1[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame1[NbrSol] = 0;
                                        gp_Dir2d dc1 = new(C1.Location().XY() - Center.XY());
                                        pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc1.XY());
                                        par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                        pararg1[NbrSol]   = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                    }
                                    if (dist2 <= Tol && Math.Abs(Radius[k] - C2.Radius()) <= Tol)
                                    {
                                        TheSame2[NbrSol] = 1;
                                    }
                                    else
                                    {
                                        TheSame2[NbrSol] = 0;
                                        gp_Dir2d dc2 = new(C2.Location().XY() - Center.XY());
                                        pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc2.XY());
                                        par2sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                        pararg2[NbrSol]   = ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                                    }
                                    pntcen[NbrSol]  = Center;
                                    parcen3[NbrSol] = Intp.Point(j).ParamOnSecond();
                                }
                                WellDone = true;
                            }
                        }
                    }
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tangent a un Cercle C1 et a une Droite L2.      +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a C1 et L2 qui nous donnent           +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tangents a C1 et L2.                                                  +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant C1 et L2.  +
    //=========================================================================
    /// <summary>Circles tangent to a qualified circle and a qualified line, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedLin Qualified2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {

        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        NbrSol     = 0;
        double Tol = Math.Abs(Tolerance);
        double Radius;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Dir2d  dirx = new(gp_Dir2d.D.X);
        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Lin2d  L2 = Qualified2.Qualified();
        double    R1 = C1.Radius();
        gp_Pnt2d  center1 = C1.Location();
        gp_Pnt2d  origin2 = L2.Location();
        gp_Dir2d  dir2 = L2.Direction();
        gp_Dir2d  normL2 = new(-dir2.Y(), dir2.X());

        GccAna_CircLin2dBisec Bis = new(C1, L2);
        if (Bis.IsDone())
        {
            double                             Tol1 = Math.Abs(Tolerance);
            double                             Tol2 = Tol1;
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            int                                nbsolution = Bis.NbSolutions();
            Geom2dAdaptor_Curve   HCu2       = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              C2 = new(HCu2, 0.0);
            firstparam = Max(C2.FirstParameter(), thefirst);
            lastparam  = Min(C2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
            for (int i = 1; i <= nbsolution; i++)
            {
                GccInt_Bisec              Sol  = Bis.ThisSolution(i);
                GccInt_IType              type = Sol.ArcType();
                switch (type)
                {
                    case GccInt_Lin:
                    {
                        gp_Lin2d        Line = Sol.Line();
                        IntRes2d_Domain D1 = new();
                        Intp.Perform(Line, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Par:
                    {
                        gp_Parab2d      Parab = Sol.Parabola();
                        IntRes2d_Domain D1 = new(ElCLib.Value(-40, Parab),
                                                              -40,
                                                              Tol1,
                                                              ElCLib.Value(40, Parab),
                                                              40,
                                                              Tol1);
                        Intp.Perform(Parab, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    default:
                    {
                        throw new Standard_ConstructionError();
                    }
                }
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int j = 1; j <= Intp.NbPoints() && NbrSol < aNbSolMAX; j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double   dist1 = Center.Distance(center1);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 < Tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 < Tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsEnclosing() || Qualified1.IsUnqualified())
                            {
                                ok = true;
                            }
                            Radius = L2.Distance(Center);
                            if (Qualified2.IsEnclosed() && ok)
                            {
                                ok = false;
                                if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                          + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        <= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified2.IsOutside() && ok)
                            {
                                ok = false;
                                if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                          + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        >= 0)
                                {
                                    ok = true;
                                }
                            }
                            if (Qualified1.IsEnclosing() && dist1 > Radius)
                            {
                                ok = false;
                            }
                            if (ok)
                            {
                                NbrSol++;
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //            =======================================================
                                gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
                                double   distcc1 = Center.Distance(center1);
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
                                if (!Qualified2.IsUnqualified())
                                {
                                    qualifier2[NbrSol] = Qualified2.Qualifier();
                                }
                                else if (dc2.Dot(normL2) > 0.0)
                                {
                                    qualifier2[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier2[NbrSol] = GccEnt_enclosed;
                                }
                                if (dist1 <= Tol && Math.Abs(Radius - C1.Radius()) <= Tol)
                                {
                                    TheSame1[NbrSol] = 1;
                                }
                                else
                                {
                                    TheSame1[NbrSol] = 0;
                                    gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc1.XY());
                                    par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                    pararg1[NbrSol]   = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                }
                                TheSame2[NbrSol]  = 0;
                                double sign       = dc2.Dot(new gp_Dir2d(-dir2.Y(), dir2.X()));
                                dc2               = new gp_Dir2d(sign * new gp_XY(-dir2.Y(), dir2.X()));
                                pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc2.XY());
                                par2sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol]   = ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                pntcen[NbrSol]    = Center;
                                parcen3[NbrSol]   = Intp.Point(j).ParamOnSecond();
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a deux Droites L1 et L2.                   +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a L1 et L2 qui nous donnent           +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a L1 et L2.                                                     +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant L1 et L2.  +
    //=========================================================================
    /// <summary>Circles tangent to two qualified lines, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in GccEnt_QualifiedLin Qualified1, in GccEnt_QualifiedLin Qualified2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {

        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double            Tol    = Math.Abs(Tolerance);
        double            Radius = 0;
        gp_Dir2d          dirx = new(gp_Dir2d.D.X);
        gp_Lin2d          L1 = Qualified1.Qualified();
        gp_Lin2d          L2 = Qualified2.Qualified();
        gp_Dir2d          dir1 = L1.Direction();
        gp_Dir2d          dir2 = L2.Direction();
        gp_Dir2d          Dnor1 = new(-dir1.Y(), dir1.X());
        gp_Dir2d          Dnor2 = new(-dir2.Y(), dir2.X());
        gp_Pnt2d          origin1 = L1.Location();
        gp_Pnt2d          origin2 = L2.Location();
        GccAna_Lin2dBisec Bis = new(L1, L2);
        if (Bis.IsDone())
        {
            double                             Tol1 = Math.Abs(Tolerance);
            double                             Tol2 = Tol1;
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            int                                nbsolution = Bis.NbSolutions();
            Geom2dAdaptor_Curve   HCu2       = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              C2 = new(HCu2, 0.0);
            firstparam = Max(C2.FirstParameter(), thefirst);
            lastparam  = Min(C2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
            IntRes2d_Domain D1 = new();
            for (int i = 1; i <= nbsolution; i++)
            {
                Intp.Perform(Bis.ThisSolution(i), D1, C2, D2, Tol1, Tol2);
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()))
                    {
                        for (int j = 1; j <= Intp.NbPoints() && NbrSol < aNbSolMAX; j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double   dist1 = L1.Distance(Center);
                            double   dist2 = L2.Distance(Center);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (Qualified1.IsEnclosed())
                            {
                                if ((((origin1.X() - Center.X()) * (-dir1.Y()))
                                          + ((origin1.Y() - Center.Y()) * (dir1.X())))
                                        <= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if ((((origin1.X() - Center.X()) * (-dir1.Y()))
                                          + ((origin1.Y() - Center.Y()) * (dir1.X())))
                                        >= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                ok = true;
                            }
                            if (Qualified2.IsEnclosed() && ok)
                            {
                                ok = false;
                                if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                          + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        <= 0)
                                {
                                    ok     = true;
                                    Radius = (dist1 + dist2) / 2.0;
                                }
                            }
                            else if (Qualified2.IsOutside() && ok)
                            {
                                ok = false;
                                if ((((origin2.X() - Center.X()) * (-dir2.Y()))
                                          + ((origin2.Y() - Center.Y()) * (dir2.X())))
                                        >= 0)
                                {
                                    ok     = true;
                                    Radius = (dist1 + dist2) / 2.0;
                                }
                            }
                            else if (Qualified2.IsUnqualified() && ok)
                            {
                                Radius = (dist1 + dist2) / 2.0;
                            }
                            if (ok)
                            {
                                NbrSol++;
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //            =======================================================
                                gp_Dir2d dc1 = new(origin1.XY() - Center.XY());
                                gp_Dir2d dc2 = new(origin2.XY() - Center.XY());
                                if (!Qualified1.IsUnqualified())
                                {
                                    qualifier1[NbrSol] = Qualified1.Qualifier();
                                }
                                else if (dc1.Dot(Dnor1) > 0.0)
                                {
                                    qualifier1[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier1[NbrSol] = GccEnt_enclosed;
                                }
                                if (!Qualified2.IsUnqualified())
                                {
                                    qualifier2[NbrSol] = Qualified2.Qualifier();
                                }
                                else if (dc2.Dot(Dnor2) > 0.0)
                                {
                                    qualifier2[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier2[NbrSol] = GccEnt_enclosed;
                                }
                                TheSame1[NbrSol]  = 0;
                                TheSame2[NbrSol]  = 0;
                                double sign       = dc1.Dot(Dnor1);
                                dc1               = new gp_Dir2d(sign * new gp_XY(-dir1.Y(), dir1.X()));
                                pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc1.XY());
                                par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                pararg1[NbrSol]   = ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                                sign              = dc2.Dot(new gp_Dir2d(-dir2.Y(), dir2.X()));
                                dc2               = new gp_Dir2d(sign * new gp_XY(-dir2.Y(), dir2.X()));
                                pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc2.XY());
                                par2sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                pararg2[NbrSol]   = ElCLib.Parameter(L2, pnttg2sol[NbrSol]);
                                pntcen[NbrSol]    = Center;
                                parcen3[NbrSol]   = Intp.Point(j).ParamOnSecond();
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a un Cercle C1, passant par un point P2    +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a C1 et Point2 qui nous donnent       +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a C1 et Point2.                                                 +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant C1.        +
    //=========================================================================
    /// <summary>Circles tangent to a qualified circle and through a point, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in GccEnt_QualifiedCirc Qualified1, in gp_Pnt2d Point2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {

        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        double                Tol = Math.Abs(Tolerance);
        double                Radius;
        gp_Dir2d              dirx = new(gp_Dir2d.D.X);
        gp_Circ2d             C1 = Qualified1.Qualified();
        double                R1 = C1.Radius();
        gp_Pnt2d              center1 = C1.Location();
        GccAna_CircPnt2dBisec Bis = new(C1, Point2);
        if (Bis.IsDone())
        {
            double                             Tol1 = Math.Abs(Tolerance);
            double                             Tol2 = Tol1;
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            int                                nbsolution = Bis.NbSolutions();
            Geom2dAdaptor_Curve   HCu2       = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              C2 = new(HCu2, 0.0);
            firstparam = Max(C2.FirstParameter(), thefirst);
            lastparam  = Min(C2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
            for (int i = 1; i <= nbsolution; i++)
            {
                GccInt_Bisec              Sol  = Bis.ThisSolution(i);
                GccInt_IType              type = Sol.ArcType();
                switch (type)
                {
                    case GccInt_Cir:
                    {
                        gp_Circ2d       Circ = Sol.Circle();
                        IntRes2d_Domain D1 = new(ElCLib.Value(0.0, Circ),
                                                              0.0,
                                                              Tol1,
                                                              ElCLib.Value(2.0 * M_PI, Circ),
                                                              2.0 * M_PI,
                                                              Tol2);
                        D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        Intp.Perform(Circ, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Lin:
                    {
                        gp_Lin2d        Line = Sol.Line();
                        IntRes2d_Domain D1 = new();
                        Intp.Perform(Line, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Ell:
                    {
                        gp_Elips2d      Elips = Sol.Ellipse();
                        IntRes2d_Domain D1 = new(ElCLib.Value(0.0, Elips),
                                                              0.0,
                                                              Tol1,
                                                              ElCLib.Value(2.0 * M_PI, Elips),
                                                              2.0 * M_PI,
                                                              Tol2);
                        D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        Intp.Perform(Elips, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    case GccInt_Hpr:
                    {
                        gp_Hypr2d       Hypr = Sol.Hyperbola();
                        IntRes2d_Domain D1 = new(ElCLib.Value(-4.0, Hypr),
                                                              -4.0,
                                                              Tol1,
                                                              ElCLib.Value(4.0, Hypr),
                                                              4.0,
                                                              Tol2);
                        Intp.Perform(Hypr, D1, C2, D2, Tol1, Tol2);
                    }
                    break;
                    default:
                    {
                        throw new Standard_ConstructionError();
                    }
                }
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()))
                    {
                        for (int j = 1; j <= Intp.NbPoints() && NbrSol < aNbSolMAX; j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            Radius       = Center.Distance(Point2);
                            double dist1 = center1.Distance(Center);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (Qualified1.IsEnclosed())
                            {
                                if (dist1 - R1 <= Tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsOutside())
                            {
                                if (R1 - dist1 <= Tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (Qualified1.IsEnclosing())
                            {
                                ok = true;
                            }
                            else if (Qualified1.IsUnqualified())
                            {
                                ok = true;
                            }
                            if (ok)
                            {
                                NbrSol++;
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //            =======================================================
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
                                qualifier2[NbrSol] = GccEnt_noqualifier;
                                if (dist1 <= Tol && Math.Abs(Radius - R1) <= Tol)
                                {
                                    TheSame1[NbrSol] = 1;
                                }
                                else
                                {
                                    TheSame1[NbrSol] = 0;
                                    gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc1.XY());
                                    par1sol[NbrSol]   = 0.0;
                                    par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                    pararg1[NbrSol]   = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                                }
                                TheSame2[NbrSol]  = 0;
                                pnttg2sol[NbrSol] = Point2;
                                pntcen[NbrSol]    = Center;
                                parcen3[NbrSol]   = Intp.Point(j).ParamOnSecond();
                                pararg2[NbrSol]   = 0.0;
                                par2sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            }
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a une ligne L1, passant par un point P2    +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a L1 et Point2 qui nous donnent       +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a L1 et passant par Point2.                                     +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant L1.        +
    //=========================================================================
    /// <summary>Circles tangent to a qualified line and through a point, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in GccEnt_QualifiedLin Qualified1, in gp_Pnt2d Point2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {

        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double Tol = Math.Abs(Tolerance);
        NbrSol     = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Dir2d             dirx = new(gp_Dir2d.D.X);
        gp_Lin2d             L1 = Qualified1.Qualified();
        gp_Pnt2d             origin1 = L1.Location();
        gp_Dir2d             dir1 = L1.Direction();
        gp_Dir2d             normal = new(-dir1.Y(), dir1.X());
        GccAna_LinPnt2dBisec Bis = new(L1, Point2);
        if (Bis.IsDone())
        {
            double                             Tol1 = Math.Abs(Tolerance);
            double                             Tol2 = Tol1;
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            Geom2dAdaptor_Curve   HCu2 = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              C2 = new(HCu2, 0.0);
            firstparam = Max(C2.FirstParameter(), thefirst);
            lastparam  = Min(C2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(C2.Value(firstparam), firstparam, Tol, C2.Value(lastparam), lastparam, Tol);
            GccInt_Bisec              Sol  = Bis.ThisSolution();
            GccInt_IType              type = Sol.ArcType();
            switch (type)
            {
                case GccInt_Lin:
                {
                    gp_Lin2d        Line = Sol.Line();
                    IntRes2d_Domain D1 = new();
                    Intp.Perform(Line, D1, C2, D2, Tol1, Tol2);
                }
                break;
                case GccInt_Par:
                {
                    gp_Parab2d      Parab = Sol.Parabola();
                    IntRes2d_Domain D1 = new(ElCLib.Value(-40, Parab),
                                                          -40,
                                                          Tol1,
                                                          ElCLib.Value(40, Parab),
                                                          40,
                                                          Tol1);
                    Intp.Perform(Parab, D1, C2, D2, Tol1, Tol2);
                }
                break;
                default:
                {
                    throw new Standard_ConstructionError();
                }
            }
            if (Intp.IsDone())
            {
                if ((!Intp.IsEmpty()))
                {
                    for (int j = 1; j <= Intp.NbPoints() && NbrSol < aNbSolMAX; j++)
                    {
                        gp_Pnt2d Center = Intp.Point(j).Value();
                        double   Radius = L1.Distance(Center);
                        //	  int nbsol = 1;
                        bool ok = false;
                        if (Qualified1.IsEnclosed())
                        {
                            if ((((origin1.X() - Center.X()) * (-dir1.Y()))
                                      + ((origin1.Y() - Center.Y()) * (dir1.X())))
                                    <= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (Qualified1.IsOutside())
                        {
                            if ((((origin1.X() - Center.X()) * (-dir1.Y()))
                                      + ((origin1.Y() - Center.Y()) * (dir1.X())))
                                    >= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (Qualified1.IsUnqualified())
                        {
                            ok = true;
                        }
                        if (ok)
                        {
                            NbrSol++;
                            cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //          =======================================================
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                            gp_Dir2d dc2 = new(origin1.XY() - Center.XY());
                            if (!Qualified1.IsUnqualified())
                            {
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                            }
                            else if (dc2.Dot(normal) > 0.0)
                            {
                                qualifier1[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier1[NbrSol] = GccEnt_enclosed;
                            }
                            TheSame1[NbrSol] = 0;
                            TheSame2[NbrSol] = 0;
                            gp_Dir2d dc1 = new(origin1.XY() - Center.XY());
                            double   sign     = dc1.Dot(new gp_Dir2d(-dir1.Y(), dir1.X()));
                            dc1               = new gp_Dir2d(sign * new gp_XY(-dir1.Y(), dir1.X()));
                            pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius * dc1.XY());
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol]   = ElCLib.Parameter(L1, pnttg1sol[NbrSol]);
                            pnttg2sol[NbrSol] = Point2;
                            par2sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            pararg2[NbrSol]   = 0.0;
                            pntcen[NbrSol]    = Center;
                            parcen3[NbrSol]   = Intp.Point(j).ParamOnSecond();
                        }
                    }
                }
                WellDone = true;
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle passant par deux point Point1 et Point2         +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a Point1 et Point2 qui nous donnent   +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  passant par Point1 et Point2.                                         +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //=========================================================================
    /// <summary>Circles through two points, centred on a curve.</summary>
    public Geom2dGcc_Circ2d2TanOnGeo(in gp_Pnt2d Point1, in gp_Pnt2d Point2, Geom2dAdaptor_Curve OnCurv, double Tolerance)
    {

        WellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double Tol = Math.Abs(Tolerance);
        NbrSol     = 0;
        gp_Dir2d          dirx = new(gp_Dir2d.D.X);
        GccAna_Pnt2dBisec Bis = new(Point1, Point2);
        if (Bis.IsDone())
        {
            double                             Tol1 = Math.Abs(Tolerance);
            double                             Tol2 = Tol1;
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            Geom2dAdaptor_Curve   HCu2 = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            Adaptor2d_OffsetCurve              Cu2 = new(HCu2, 0.0);
            firstparam = Max(Cu2.FirstParameter(), thefirst);
            lastparam  = Min(Cu2.LastParameter(), thelast);
            IntRes2d_Domain D2 = new(Cu2.Value(firstparam),
                                                  firstparam,
                                                  Tol,
                                                  Cu2.Value(lastparam),
                                                  lastparam,
                                                  Tol);
            IntRes2d_Domain D1 = new();
            if (Bis.HasSolution())
            {
                Intp.Perform(Bis.ThisSolution(), D1, Cu2, D2, Tol1, Tol2);
                if (Intp.IsDone())
                {
                    if ((!Intp.IsEmpty()))
                    {
                        for (int j = 1; j <= Intp.NbPoints() && NbrSol < aNbSolMAX; j++)
                        {
                            gp_Pnt2d Center = Intp.Point(j).Value();
                            double   Radius = Point2.Distance(Center);
                            NbrSol++;
                            cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = GccEnt_noqualifier;
                            qualifier2[NbrSol] = GccEnt_noqualifier;
                            TheSame1[NbrSol]   = 0;
                            TheSame2[NbrSol]   = 0;
                            pntcen[NbrSol]     = Center;
                            pnttg1sol[NbrSol]  = Point1;
                            pnttg2sol[NbrSol]  = Point2;
                            pararg1[NbrSol]    = 0.0;
                            pararg2[NbrSol]    = 0.0;
                            par1sol[NbrSol]    = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            par2sol[NbrSol]    = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                            parcen3[NbrSol]    = Intp.Point(j).ParamOnSecond();
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>True if the construction succeeded (possibly with no solution).</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>Solution Index.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public gp_Circ2d ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        return cirsol[Index];
    }

    /// <summary>The qualifiers of solution Index relative to the two arguments.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            Qualif1 = qualifier1[Index];
            Qualif2 = qualifier2[Index];
        }
    }

    /// <summary>The tangency point of solution Index on the first argument, with its parameters on both.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
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
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>The tangency point of solution Index on the second argument, with its parameters on both.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public void Tangency2(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
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
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>The centre of solution Index and its parameter on the centre's curve.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public void CenterOn3(int Index, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            ParArg = parcen3[Index];
            PntSol = pntcen[Index];
        }
    }

    /// <summary>True if solution Index is the first argument (a circle).</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame1[Index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if solution Index is the second argument (a circle).</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame2(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame2[Index] == 0)
        {
            return false;
        }

        return true;
    }
}
