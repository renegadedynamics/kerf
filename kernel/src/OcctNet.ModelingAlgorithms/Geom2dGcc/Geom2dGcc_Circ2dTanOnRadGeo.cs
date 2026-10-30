// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRadGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRadGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D circles of a given radius tangent to an argument (a curve, a circle, a line or a point) whose centre lies on a
/// line, a circle or a curve: the centres are the intersections of the curve the centre lies on with the argument's
/// offsets at the radius.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 8 solutions.</remarks>
public sealed class Geom2dGcc_Circ2dTanOnRadGeo
{
    private const int aNbSolMAX = 8;

    private readonly bool WellDone;
    private readonly int NbrSol;
    private readonly NCollection_Array1<gp_Circ2d> cirsol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<GccEnt_Position> qualifier1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<int> TheSame1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<gp_Pnt2d> pntcen3 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> par1sol = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> pararg1 = new(1, aNbSolMAX);
    private readonly NCollection_Array1<double> parcen3 = new(1, aNbSolMAX);

    //=========================================================================
    //  Cercle tangent  :  a un cercle Qualified1 (C1).                       +
    //         centre   :  sur une droite OnLine.                             +
    //         de rayon :  Radius.                                            +
    //                                                                        +
    //  On initialise le tableau de solutions cirsol ainsi que tous les       +
    //  champs.                                                               +
    //  On elimine en fonction du qualifieur les cas ne presentant pas de     +
    //  solutions.                                                            +
    //  On resoud l equation du second degre indiquant que le point de centre +
    //  recherche (xc,yc) est a une distance Radius du cercle C1 et           +
    //                        sur la droite OnLine.                           +
    //  Les solutions sont representees par les cercles :                     +
    //                   - de centre Pntcen(xc,yc)                            +
    //                   - de rayon Radius.                                   +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the line OnLine.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in Geom2dGcc_QCurve Qualified1, in gp_Lin2d OnLine, double Radius, double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   Tol      = Math.Abs(Tolerance);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        WellDone = false;
        NbrSol   = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int                        nbrcote1 = 0;
        NCollection_Array1<double> Coef     = new(1, 2);
        Geom2dAdaptor_Curve        Cu1      = Qualified1.Qualified();

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            if (Qualified1.IsEnclosed())
            {
                //    ===========================
                nbrcote1 = 1;
                Coef[1]  = Radius;
            }
            else if (Qualified1.IsOutside())
            {
                //   ===============================
                nbrcote1 = 1;
                Coef[1]  = -Radius;
            }
            else if (Qualified1.IsUnqualified())
            {
                //   ===================================
                nbrcote1 = 2;
                Coef[1]  = Radius;
                Coef[2]  = -Radius;
            }
            IntRes2d_Domain                    D1   = new();
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Geom2dAdaptor_Curve HCu1 = (Geom2dAdaptor_Curve)Cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C2(HCu1,Coef(jcote1));
                Adaptor2d_OffsetCurve C2 = new(HCu1, -Coef[jcote1]);
                firstparam               = Max(C2.FirstParameter(), thefirst);
                lastparam                = Min(C2.LastParameter(), thelast);
                IntRes2d_Domain D2 = new(C2.Value(firstparam),
                                         firstparam,
                                         Tol,
                                         C2.Value(lastparam),
                                         lastparam,
                                         Tol);
                Intp.Perform(OnLine, D1, C2, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            gp_Pnt2d Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            TheSame1[NbrSol]   = 0;
                            pararg1[NbrSol]    = Intp.Point(i).ParamOnSecond();
                            parcen3[NbrSol]    = Intp.Point(i).ParamOnFirst();
                            // OCCT: reads pnttg1sol(NbrSol) before it is set (the origin, as the array is
                            // default-constructed)
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pnttg1sol[NbrSol] = Geom2dGcc_CurveTool.Value(Cu1, pararg1[NbrSol]);
                            pntcen3[NbrSol]   = Center;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the circle OnCirc.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in Geom2dGcc_QCurve Qualified1, in gp_Circ2d OnCirc, double Radius, double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   Tol      = Math.Abs(Tolerance);
        int      nbrcote1 = 0;
        WellDone          = false;
        NbrSol            = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        NCollection_Array1<double> cote1 = new(1, 2);
        Geom2dAdaptor_Curve        Cu1   = Qualified1.Qualified();

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
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
            IntRes2d_Domain D1 = new(ElCLib.Value(0.0, OnCirc),
                                     0.0,
                                     Tol,
                                     ElCLib.Value(2.0 * M_PI, OnCirc),
                                     2.0 * M_PI,
                                     Tol);
            D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Geom2dAdaptor_Curve HCu1 = (Geom2dAdaptor_Curve)Cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C2(HCu1,cote1(jcote1));
                Adaptor2d_OffsetCurve C2 = new(HCu1, -cote1[jcote1]);
                firstparam               = Max(C2.FirstParameter(), thefirst);
                lastparam                = Min(C2.LastParameter(), thelast);
                IntRes2d_Domain D2 = new(C2.Value(firstparam),
                                         firstparam,
                                         Tol,
                                         C2.Value(lastparam),
                                         lastparam,
                                         Tol);
                Intp.Perform(OnCirc, D1, C2, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            gp_Pnt2d Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            TheSame1[NbrSol]   = 0;
                            pararg1[NbrSol]    = Intp.Point(i).ParamOnSecond();
                            parcen3[NbrSol]    = Intp.Point(i).ParamOnFirst();
                            // OCCT: reads pnttg1sol(NbrSol) before it is set
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pnttg1sol[NbrSol] = Geom2dGcc_CurveTool.Value(Cu1, pararg1[NbrSol]);
                            pntcen3[NbrSol]   = Center;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to the qualified circle, centred on the curve OnCurv.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in GccEnt_QualifiedCirc Qualified1, Geom2dAdaptor_Curve OnCurv, double Radius,
                                       double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   Tol      = Math.Abs(Tolerance);
        int      nbrcote1 = 0;
        WellDone          = false;
        NbrSol            = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        NCollection_Array1<double> cote1   = new(1, 2);
        gp_Circ2d                  C1      = Qualified1.Qualified();
        gp_Pnt2d                   center1 = C1.Location();
        double                     R1      = C1.Radius();

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
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
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                gp_Circ2d       Circ = new(C1.XAxis(), R1 + cote1[jcote1]);
                IntRes2d_Domain D1   = new(ElCLib.Value(0.0, Circ),
                                           0.0,
                                           Tol,
                                           ElCLib.Value(2.0 * M_PI, Circ),
                                           2.0 * M_PI,
                                           Tol);
                D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                firstparam         = Max(Geom2dGcc_CurveTool.FirstParameter(OnCurv), thefirst);
                lastparam          = Min(Geom2dGcc_CurveTool.LastParameter(OnCurv), thelast);
                IntRes2d_Domain D2 = new(Geom2dGcc_CurveTool.Value(OnCurv, firstparam),
                                         firstparam,
                                         Tol,
                                         Geom2dGcc_CurveTool.Value(OnCurv, lastparam),
                                         lastparam,
                                         Tol);
                Intp.Perform(Circ, D1, OnCurv, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            gp_Pnt2d Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
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
                            TheSame1[NbrSol] = 0;
                            pararg1[NbrSol]  = Intp.Point(i).ParamOnFirst();
                            parcen3[NbrSol]  = Intp.Point(i).ParamOnSecond();
                            // OCCT: reads pnttg1sol(NbrSol) before it is set
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pnttg1sol[NbrSol] = ElCLib.Value(pararg1[NbrSol], C1);
                            pntcen3[NbrSol]   = Center;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to the qualified line, centred on the curve OnCurv.</summary>
    /// <exception cref="GccEnt_BadQualifier">The line is qualified enclosing or has no qualifier.</exception>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in GccEnt_QualifiedLin Qualified1, Geom2dAdaptor_Curve OnCurv, double Radius,
                                       double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   Tol      = Math.Abs(Tolerance);
        WellDone          = false;
        NbrSol            = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int                        nbrcote1 = 0;
        NCollection_Array1<double> cote1    = new(1, 2);
        gp_Lin2d                   L1       = Qualified1.Qualified();
        gp_Pnt2d                   origin1  = L1.Location();
        gp_Dir2d                   dir1     = L1.Direction();
        gp_Dir2d                   norm1    = new(-dir1.Y(), dir1.X());

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
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
            Geom2dInt_TheIntConicCurveOfGInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                // OCCT: offsets the line's direction, not its origin, so the line is in the wrong place unless the
                // qualified line passes through dir1 (and pararg1 below is a parameter on that line, not on L1)
                gp_Pnt2d        Point = new(dir1.XY() + cote1[jcote1] * norm1.XY());
                gp_Lin2d        Line  = new(Point, dir1); // ligne avec deport.
                IntRes2d_Domain D1    = new();
                firstparam            = Max(Geom2dGcc_CurveTool.FirstParameter(OnCurv), thefirst);
                lastparam             = Min(Geom2dGcc_CurveTool.LastParameter(OnCurv), thelast);
                IntRes2d_Domain D2    = new(Geom2dGcc_CurveTool.Value(OnCurv, firstparam),
                                            firstparam,
                                            Tol,
                                            Geom2dGcc_CurveTool.Value(OnCurv, lastparam),
                                            lastparam,
                                            Tol);
                Intp.Perform(Line, D1, OnCurv, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            gp_Pnt2d Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            gp_Dir2d dc1 = new(origin1.XY() - Center.XY());
                            if (!Qualified1.IsUnqualified())
                            {
                                qualifier1[NbrSol] = Qualified1.Qualifier();
                            }
                            else if (dc1.Dot(norm1) > 0.0)
                            {
                                qualifier1[NbrSol] = GccEnt_outside;
                            }
                            else
                            {
                                qualifier1[NbrSol] = GccEnt_enclosed;
                            }
                            TheSame1[NbrSol] = 0;
                            pararg1[NbrSol]  = Intp.Point(i).ParamOnFirst();
                            parcen3[NbrSol]  = Intp.Point(i).ParamOnSecond();
                            // OCCT: reads pnttg1sol(NbrSol) before it is set
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pnttg1sol[NbrSol] = ElCLib.Value(pararg1[NbrSol], L1);
                            pntcen3[NbrSol]   = Center;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the curve OnCurv.</summary>
    /// <exception cref="GccEnt_BadQualifier">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in Geom2dGcc_QCurve Qualified1, Geom2dAdaptor_Curve OnCurv, double Radius,
                                       double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   Tol      = Math.Abs(Tolerance);
        int      nbrcote1 = 0;
        WellDone          = false;
        NbrSol            = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        NCollection_Array1<double> cote1 = new(1, 2);
        Geom2dAdaptor_Curve        Cu1   = Qualified1.Qualified();

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
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
            Geom2dInt_GInter Intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Geom2dAdaptor_Curve HCu1 = (Geom2dAdaptor_Curve)Cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C1(HCu1,cote1(jcote1));
                Adaptor2d_OffsetCurve C1 = new(HCu1, -cote1[jcote1]);
                firstparam               = Max(C1.FirstParameter(), thefirst);
                lastparam                = Min(C1.LastParameter(), thelast);
                IntRes2d_Domain D1 = new(C1.Value(firstparam),
                                         firstparam,
                                         Tol,
                                         C1.Value(lastparam),
                                         lastparam,
                                         Tol);
                Geom2dAdaptor_Curve   HOnCurv = (Geom2dAdaptor_Curve)OnCurv.ShallowCopy();
                Adaptor2d_OffsetCurve C2      = new(HOnCurv);
                firstparam                    = Max(C2.FirstParameter(), thefirst);
                lastparam                     = Min(C2.LastParameter(), thelast);
                IntRes2d_Domain D2 = new(C2.Value(firstparam),
                                         firstparam,
                                         Tol,
                                         C2.Value(lastparam),
                                         lastparam,
                                         Tol);
                Intp.Perform(C1, D1, C2, D2, Tol, Tol);
                if (Intp.IsDone())
                {
                    if (!Intp.IsEmpty())
                    {
                        for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                        {
                            NbrSol++;
                            gp_Pnt2d Center = Intp.Point(i).Value();
                            cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                            TheSame1[NbrSol]   = 0;
                            pararg1[NbrSol]    = Intp.Point(i).ParamOnFirst();
                            parcen3[NbrSol]    = Intp.Point(i).ParamOnSecond();
                            // OCCT: reads pnttg1sol(NbrSol) before it is set
                            par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pnttg1sol[NbrSol] = Geom2dGcc_CurveTool.Value(Cu1, pararg1[NbrSol]);
                            pntcen3[NbrSol]   = Center;
                        }
                    }
                    WellDone = true;
                }
            }
        }
    }

    /// <summary>Circles of radius Radius through Point1, centred on the curve OnCurv.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public Geom2dGcc_Circ2dTanOnRadGeo(in gp_Pnt2d Point1, Geom2dAdaptor_Curve OnCurv, double Radius, double Tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        gp_Dir2d dirx     = new(gp_Dir2d.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   Tol      = Math.Abs(Tolerance);
        WellDone          = false;
        NbrSol            = 0;

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            //     gp_Dir2d Dir(-y1dir,x1dir);
            gp_Circ2d       Circ = new(new gp_Ax2d(Point1, new gp_Dir2d(gp_Dir2d.D.X)), Radius);
            IntRes2d_Domain D1   = new(ElCLib.Value(0.0, Circ),
                                       0.0,
                                       Tol,
                                       ElCLib.Value(2.0 * M_PI, Circ),
                                       2 * M_PI,
                                       Tol);
            D1.SetEquivalentParameters(0.0, 2.0 * M_PI);
            firstparam         = Max(Geom2dGcc_CurveTool.FirstParameter(OnCurv), thefirst);
            lastparam          = Min(Geom2dGcc_CurveTool.LastParameter(OnCurv), thelast);
            IntRes2d_Domain D2 = new(Geom2dGcc_CurveTool.Value(OnCurv, firstparam),
                                     firstparam,
                                     Tol,
                                     Geom2dGcc_CurveTool.Value(OnCurv, lastparam),
                                     lastparam,
                                     Tol);
            Geom2dInt_TheIntConicCurveOfGInter Intp = new(Circ, D1, OnCurv, D2, Tol, Tol);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    for (int i = 1; i <= Intp.NbPoints() && NbrSol < aNbSolMAX; i++)
                    {
                        NbrSol++;
                        gp_Pnt2d Center = Intp.Point(i).Value();
                        cirsol[NbrSol]  = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                        //         =======================================================
                        qualifier1[NbrSol] = GccEnt_noqualifier;
                        TheSame1[NbrSol]   = 0;
                        pararg1[NbrSol]    = Intp.Point(i).ParamOnFirst();
                        parcen3[NbrSol]    = Intp.Point(i).ParamOnSecond();
                        // OCCT: reads pnttg1sol(NbrSol) before it is set
                        par1sol[NbrSol]   = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        pnttg1sol[NbrSol] = Point1;
                        pntcen3[NbrSol]   = Center;
                    }
                    WellDone = true;
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

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is not in 1..NbSolutions().</exception>
    public gp_Circ2d ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new Standard_OutOfRange();
        }
        return cirsol[Index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
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
        }
    }

    /// <summary>The tangency point between the solution of index Index and the argument, and its parameters.</summary>
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
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
            PntSol = pnttg1sol[Index];
        }
    }

    /// <summary>The parameter of the centre of the solution of index Index on the curve it lies on, and (OCCT) the tangency point.</summary>
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
            PntSol = pnttg1sol[Index]; // OCCT: returns the tangency point, not the centre (pntcen3)
        }
    }

    /// <summary>True if the solution of index Index is the argument itself.</summary>
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
}
