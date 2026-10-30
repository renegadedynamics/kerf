// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace Occt;

/// <summary>Tools and data for filling and sweep surfaces: ruled surfaces and the circular sections of blends.</summary>
public static class GeomFill
{
    /// <summary>
    /// The ruled surface between Curve1 and Curve2: a plane, cylinder or cone where the curves allow, else a B-spline
    /// surface (GeomFill_Generator). The result is frozen.
    /// </summary>
    public static Geom_Surface? Surface(Geom_Curve Curve1, Geom_Curve Curve2)
    {
        Geom_Surface? Surf = InternalSurface(Curve1, Curve2);
        Surf?.Freeze();
        return Surf;
    }

    /// <summary>The ruled surface, not frozen, for ported callers.</summary>
    internal static Geom_Surface? InternalSurface(Geom_Curve Curve1, Geom_Curve Curve2)
    {
        Geom_Curve TheCurve1, TheCurve2;
        Geom_Surface? Surf = null;

        // recherche du type de la surface resultat:
        // les surfaces reglees particulieres sont :
        //     - les plans
        //     - les cylindres
        //     - les cones
        // dans ces trois cas les courbes doivent etre de meme type :
        //     - ou 2 droites
        //     - ou 2 cercles

        double a1 = 0, a2 = 0, b1 = 0, b2 = 0;
        bool Trim1 = false, Trim2 = false;
        if (Curve1 is Geom_TrimmedCurve Ctrim1)
        {
            TheCurve1 = Ctrim1.BasisCurve();
            a1 = Ctrim1.FirstParameter();
            b1 = Ctrim1.LastParameter();
            Trim1 = true;
        }
        else
        {
            TheCurve1 = (Geom_Curve)Curve1.Copy();
        }
        if (Curve2 is Geom_TrimmedCurve Ctrim2)
        {
            TheCurve2 = Ctrim2.BasisCurve();
            a2 = Ctrim2.FirstParameter();
            b2 = Ctrim2.LastParameter();
            Trim2 = true;
        }
        else
        {
            TheCurve2 = (Geom_Curve)Curve2.Copy();
        }

        bool IsDone = false;
        // Les deux courbes sont des droites.
        if (TheCurve1 is Geom_Line aLine1 && TheCurve2 is Geom_Line aLine2 && Trim1 && Trim2)
        {
            gp_Lin L1 = aLine1.Lin();
            gp_Lin L2 = aLine2.Lin();
            gp_Dir D1 = L1.Direction();
            gp_Dir D2 = L2.Direction();

            if (D1.IsParallel(D2, Precision.Angular()))
            {
                gp_Vec P1P2 = new gp_Vec(L1.Location(), L2.Location());
                double proj = P1P2.Dot(new gp_Vec(D1));

                if (D1.IsEqual(D2, Precision.Angular()))
                {
                    if (Math.Abs(a1 - proj - a2) <= Precision.Confusion()
                        && Math.Abs(b1 - proj - b2) <= Precision.Confusion())
                    {
                        gp_Ax3 Ax = new gp_Ax3(L1.Location(), D1.Crossed(new gp_Dir(P1P2)), D1);
                        Geom_Plane P = new Geom_Plane(Ax);
                        double V = P1P2.Dot(new gp_Vec(Ax.YDirection()));
                        Surf = new Geom_RectangularTrimmedSurface(P, a1, b1, Min(0.0, V), Max(0.0, V));
                        IsDone = true;
                    }
                }
                if (D1.IsOpposite(D2, Precision.Angular()))
                {
                    if (Math.Abs(a1 - proj + b2) <= Precision.Confusion()
                        && Math.Abs(b1 - proj + a2) <= Precision.Confusion())
                    {
                        gp_Ax3 Ax = new gp_Ax3(L1.Location(), D1.Crossed(new gp_Dir(P1P2)), D1);
                        Geom_Plane P = new Geom_Plane(Ax);
                        double V = P1P2.Dot(new gp_Vec(Ax.YDirection()));
                        Surf = new Geom_RectangularTrimmedSurface(P, a1, b1, Min(0.0, V), Max(0.0, V));
                        IsDone = true;
                    }
                }
            }
        }

        // Les deux courbes sont des cercles.
        else if (TheCurve1 is Geom_Circle aCircle1 && TheCurve2 is Geom_Circle aCircle2)
        {
            gp_Circ C1 = aCircle1.Circ();
            gp_Circ C2 = aCircle2.Circ();

            gp_Ax3 A1 = new gp_Ax3(C1.Position());
            gp_Ax3 A2 = new gp_Ax3(C2.Position());

            // first, A1 & A2 must be coaxials
            if (A1.Axis().IsCoaxial(A2.Axis(), Precision.Angular(), Precision.Confusion()))
            {
                double V = new gp_Vec(A1.Location(), A2.Location()).Dot(new gp_Vec(A1.Direction()));
                if (!Trim1 && !Trim2)
                {
                    if (Math.Abs(C1.Radius() - C2.Radius()) < Precision.Confusion())
                    {
                        Geom_CylindricalSurface C = new Geom_CylindricalSurface(A1, C1.Radius());
                        Surf = new Geom_RectangularTrimmedSurface(C, Min(0.0, V), Max(0.0, V), false);
                    }
                    else
                    {
                        double Rad = C2.Radius() - C1.Radius();
                        double Ang = Math.Atan(Rad / V);
                        if (Ang < 0.0)
                        {
                            A1.ZReverse();
                            V = -V;
                            Ang = -Ang;
                        }
                        Geom_ConicalSurface C = new Geom_ConicalSurface(A1, Ang, C1.Radius());
                        V /= Math.Cos(Ang);
                        Surf = new Geom_RectangularTrimmedSurface(C, Min(0.0, V), Max(0.0, V), false);
                    }
                    IsDone = true;
                }
                else if (Trim1 && Trim2)
                {
                }
            }
        }

        if (!IsDone)
        {
            GeomFill_Generator Generator = new GeomFill_Generator();
            Generator.AddCurve(Curve1);
            Generator.AddCurve(Curve2);
            Generator.Perform(Precision.PConfusion());
            Surf = Generator.InternalSurface();
        }

        return Surf;
    }

    /// <summary>
    /// The shape of the B-spline of an arc of angle MaxAng in the conversion TypeConv: Convert_TgtThetaOver2 becomes
    /// Convert_TgtThetaOver2_1 to _3 when the arc needs one to three spans.
    /// </summary>
    public static void GetShape(double MaxAng, out int NbPoles, out int NbKnots, out int Degree,
                                ref Convert_ParameterisationType TConv)
    {
        switch (TConv)
        {
            case Convert_QuasiAngular:
                {
                    NbPoles = 7;
                    NbKnots = 2;
                    Degree = 6;
                }
                break;
            case Convert_Polynomial:
                {
                    NbPoles = 8;
                    NbKnots = 2;
                    Degree = 7;
                }
                break;
            default:
                {
                    int NbSpan = (int)(Math.Ceiling(3.0 * Math.Abs(MaxAng) / 2.0 / M_PI));
                    NbPoles = 2 * NbSpan + 1;
                    NbKnots = NbSpan + 1;
                    Degree = 2;
                    if (NbSpan == 1)
                    {
                        TConv = Convert_TgtThetaOver2_1;
                    }
                    else if (NbSpan == 2)
                    {
                        TConv = Convert_TgtThetaOver2_2;
                    }
                    else if (NbSpan == 3)
                    {
                        TConv = Convert_TgtThetaOver2_3;
                    }
                }
                break;
        }
    }

    //=======================================================================
    // function : GetMinimalWeights
    // purpose  : On suppose les extremum de poids sont obtenus pour les
    //           extremums d'angles (A verifier eventuelement pour Quasi-Angular)
    //=======================================================================

    /// <summary>The smallest weights of the arcs between MinAng and MaxAng in the conversion TConv.</summary>
    public static void GetMinimalWeights(Convert_ParameterisationType TConv, double MinAng, double MaxAng,
                                         NCollection_Array1<double> Weights)
    {
        if (TConv == Convert_Polynomial)
        {
            Weights.Init(1);
        }
        else
        {
            gp_Ax2 popAx2 = new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z));
            gp_Circ C = new gp_Circ(popAx2, 1);
            Geom_TrimmedCurve Sect1 = new Geom_TrimmedCurve(new Geom_Circle(C), 0.0, MaxAng);
            Geom_BSplineCurve CtoBspl = GeomConvert.CurveToBSplineCurve(Sect1, TConv);
            Weights.Assign(CtoBspl.InternalWeightsArray());

            double angle_min = Max(Precision.PConfusion(), MinAng);
            Geom_TrimmedCurve Sect2 = new Geom_TrimmedCurve(new Geom_Circle(C), 0.0, angle_min);
            CtoBspl = GeomConvert.CurveToBSplineCurve(Sect2, TConv);
            NCollection_Array1<double> poids = CtoBspl.InternalWeightsArray();

            for (int ii = Weights.Lower(); ii <= Weights.Upper(); ii++)
            {
                if (poids[ii] < Weights[ii])
                {
                    Weights[ii] = poids[ii];
                }
            }
        }
    }

    /// <summary>The knots of the circular sections in the conversion TConv.</summary>
    public static void Knots(Convert_ParameterisationType TConv, NCollection_Array1<double> TKnots)
    {
        if ((TConv != Convert_QuasiAngular) && (TConv != Convert_Polynomial))
        {
            int i;
            double val = 0.0;
            for (i = TKnots.Lower(); i <= TKnots.Upper(); i++)
            {
                TKnots[i] = val;
                val = val + 1.0;
            }
        }
        else
        {
            TKnots[1] = 0.0;
            TKnots[2] = 1.0;
        }
    }

    /// <summary>The multiplicities of the circular sections in the conversion TConv.</summary>
    public static void Mults(Convert_ParameterisationType TConv, NCollection_Array1<int> TMults)
    {
        switch (TConv)
        {
            case Convert_QuasiAngular:
                {
                    TMults[1] = 7;
                    TMults[2] = 7;
                }
                break;
            case Convert_Polynomial:
                {
                    TMults[1] = 8;
                    TMults[2] = 8;
                }
                break;

            default:
                {
                    // Cas rational classsique
                    int i;
                    TMults[TMults.Lower()] = 3;
                    for (i = TMults.Lower() + 1; i <= TMults.Upper() - 1; i++)
                    {
                        TMults[i] = 2;
                    }
                    TMults[TMults.Upper()] = 3;
                }
                break;
        }
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determiner la Tolerance 3d permetant de respecter la Tolerance
    //           de continuite G1.
    //=======================================================================

    /// <summary>The 3D tolerance of the approximation that keeps the G1 tolerance AngularTol (blends).</summary>
    public static double GetTolerance(Convert_ParameterisationType TConv, double AngleMin, double Radius,
                                      double AngularTol, double SpatialTol)
    {
        gp_Ax2 popAx2 = new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z));
        gp_Circ C = new gp_Circ(popAx2, Radius);
        Geom_Circle popCircle = new Geom_Circle(C);
        Geom_TrimmedCurve Sect = new Geom_TrimmedCurve(popCircle, 0.0, Max(AngleMin, 0.02));
        // 0.02 est proche d'1 degree, en desous on ne se preocupe pas de la tngence
        // afin d'eviter des tolerances d'approximation tendant vers 0 !
        Geom_BSplineCurve CtoBspl = GeomConvert.CurveToBSplineCurve(Sect, TConv);
        double Dist;
        Dist = CtoBspl.Pole(1).Distance(CtoBspl.Pole(2)) + SpatialTol;
        return Dist * AngularTol / 2;
    }

    //===========================================================================
    // function : GetCircle
    // purpose  : Calculs les poles et poids d'un cercle definie par ses extremites
    //           et son rayon.
    //   On evite (si possible) de passer par les convertions pour
    //  1) Des problemes de performances.
    //  2) Assurer la coherance entre cette methode est celle qui donne la derive
    //============================================================================

    /// <summary>
    /// The poles and weights of the arc of radius Rayon about Center from pts1 to pts2 (ns1, ns2: the inward normals
    /// at the ends; nplan: the normal to the plane), in the conversion TConv.
    /// </summary>
    public static void GetCircle(Convert_ParameterisationType TConv,
                                 in gp_Vec ns1, // Normal rentrente au premier point
                                 in gp_Vec ns2, // Normal rentrente au second point
                                 in gp_Vec nplan, // Normal au plan
                                 in gp_Pnt pts1,
                                 in gp_Pnt pts2,
                                 double Rayon, // Rayon (doit etre positif)
                                 in gp_Pnt Center,
                                 NCollection_Array1<gp_Pnt> Poles,
                                 NCollection_Array1<double> Weights)
    {
        // La classe de convertion

        int i, jj;
        double Cosa, Sina, Angle, Alpha, Cosas2, lambda;
        gp_Vec temp = new gp_Vec(), np2;
        int low = Poles.Lower();
        int upp = Poles.Upper();

        Cosa = ns1.Dot(ns2);
        Sina = nplan.Dot(ns1.Crossed(ns2));

        if (Cosa < -1.0)
        {
            Cosa = -1;
            Sina = 0;
        }
        if (Cosa > 1.0)
        {
            Cosa = 1;
            Sina = 0;
        }
        Angle = Math.Acos(Cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (Sina < 0.0)
        {
            if (Cosa > 0.0)
            {
                Angle = -Angle;
            }
            else
            {
                Angle = 2.0 * M_PI - Angle;
            }
        }

        switch (TConv)
        {
            case Convert_QuasiAngular:
                {
                    GeomFill_QuasiAngularConvertor QConvertor = new GeomFill_QuasiAngularConvertor();
                    QConvertor.Init();
                    QConvertor.Section(pts1, Center, nplan, Angle, Poles, Weights);
                    break;
                }
            case Convert_Polynomial:
                {
                    GeomFill_PolynomialConvertor PConvertor = new GeomFill_PolynomialConvertor();
                    PConvertor.Init();
                    PConvertor.Section(pts1, Center, nplan, Angle, Poles);
                    Weights.Init(1);
                    break;
                }
            default:
                {
                    // Cas Rational, on utilise une expression directe beaucoup plus
                    // performente que GeomConvert
                    int NbSpan = (Poles.Length() - 1) / 2;

                    Poles[low] = pts1;
                    Poles[upp] = pts2;
                    Weights[low] = 1;
                    Weights[upp] = 1;

                    np2 = nplan.Crossed(ns1);

                    Alpha = Angle / ((double)(NbSpan));
                    Cosas2 = Math.Cos(Alpha / 2);

                    for (i = 1, jj = low + 2; i <= NbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * Alpha;
                        Cosa = Math.Cos(lambda);
                        Sina = Math.Sin(lambda);
                        temp.SetLinearForm(Cosa - 1, ns1, Sina, np2);
                        Poles[jj].SetXYZ(pts1.XYZ() + Rayon * temp.XYZ());
                        Weights[jj] = 1;
                    }

                    lambda = 1.0 / (2.0 * Cosas2 * Cosas2);
                    for (i = 1, jj = low + 1; i <= NbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(Poles[jj - 1].XYZ() + Poles[jj + 1].XYZ() - 2.0 * Center.XYZ());
                        Poles[jj].SetXYZ(Center.XYZ() + lambda * temp.XYZ());
                        Weights[jj] = Cosas2;
                    }
                    break;
                }
        }
    }

    /// <summary>The poles and weights of the arc and their first derivatives.</summary>
    public static bool GetCircle(Convert_ParameterisationType TConv,
                                 in gp_Vec ns1,
                                 in gp_Vec ns2,
                                 in gp_Vec dn1w,
                                 in gp_Vec dn2w,
                                 in gp_Vec nplan,
                                 in gp_Vec dnplan,
                                 in gp_Pnt pts1,
                                 in gp_Pnt pts2,
                                 in gp_Vec tang1,
                                 in gp_Vec tang2,
                                 double Rayon,
                                 double DRayon,
                                 in gp_Pnt Center,
                                 in gp_Vec DCenter,
                                 NCollection_Array1<gp_Pnt> Poles,
                                 NCollection_Array1<gp_Vec> DPoles,
                                 NCollection_Array1<double> Weights,
                                 NCollection_Array1<double> DWeights)
    {
        double Cosa, Sina, Cosas2, Sinas2, Angle, DAngle, Alpha, lambda, Dlambda;
        gp_Vec temp = new gp_Vec(), np2, dnp2;
        int i, jj;
        int NbSpan = (Poles.Length() - 1) / 2;
        int low = Poles.Lower();
        int upp = Poles.Upper();

        Cosa = ns1.Dot(ns2);
        Sina = nplan.Dot(ns1.Crossed(ns2));

        if (Cosa < -1.0)
        {
            Cosa = -1;
            Sina = 0;
        }
        if (Cosa > 1.0)
        {
            Cosa = 1;
            Sina = 0;
        }
        Angle = Math.Acos(Cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (Sina < 0.0)
        {
            if (Cosa > 0.0)
            {
                Angle = -Angle;
            }
            else
            {
                Angle = 2.0 * M_PI - Angle;
            }
        }

        if (Math.Abs(Sina) > Math.Abs(Cosa))
        {
            DAngle = -(dn1w.Dot(ns2) + ns1.Dot(dn2w)) / Sina;
        }
        else
        {
            DAngle = (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(dn1w.Crossed(ns2) + ns1.Crossed(dn2w))) / Cosa;
        }

        // Aux Extremites.
        Poles[low] = pts1;
        Poles[upp] = pts2;
        Weights[low] = 1;
        Weights[upp] = 1;

        DPoles[low] = tang1;
        DPoles[upp] = tang2;
        DWeights[low] = 0;
        DWeights[upp] = 0;

        switch (TConv)
        {
            case Convert_QuasiAngular:
                {
                    GeomFill_QuasiAngularConvertor QConvertor = new GeomFill_QuasiAngularConvertor();
                    QConvertor.Init();
                    QConvertor.Section(pts1,
                                       tang1,
                                       Center,
                                       DCenter,
                                       nplan,
                                       dnplan,
                                       Angle,
                                       DAngle,
                                       Poles,
                                       DPoles,
                                       Weights,
                                       DWeights);
                    return true;
                }
            case Convert_Polynomial:
                {
                    GeomFill_PolynomialConvertor PConvertor = new GeomFill_PolynomialConvertor();
                    PConvertor.Init();
                    PConvertor.Section(pts1, tang1, Center, DCenter, nplan, dnplan, Angle, DAngle, Poles, DPoles);
                    Weights.Init(1);
                    DWeights.Init(0);
                    return true;
                }

            default:
                // Cas rationel classique
                {
                    np2 = nplan.Crossed(ns1);
                    dnp2 = dnplan.Crossed(ns1).Added(nplan.Crossed(dn1w));

                    Alpha = Angle / ((double)(NbSpan));
                    Cosas2 = Math.Cos(Alpha / 2);
                    Sinas2 = Math.Sin(Alpha / 2);

                    for (i = 1, jj = low + 2; i <= NbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * Alpha;
                        Cosa = Math.Cos(lambda);
                        Sina = Math.Sin(lambda);
                        temp.SetLinearForm(Cosa - 1, ns1, Sina, np2);
                        Poles[jj].SetXYZ(pts1.XYZ() + Rayon * temp.XYZ());

                        DPoles[jj].SetLinearForm(DRayon, temp, tang1);
                        temp.SetLinearForm(-Sina, ns1, Cosa, np2);
                        temp.Multiply(((double)(i)) / ((double)(NbSpan)) * DAngle);
                        temp.Add(((Cosa - 1) * dn1w).Added(Sina * dnp2));
                        DPoles[jj] += Rayon * temp;
                    }

                    lambda = 1.0 / (2.0 * Cosas2 * Cosas2);
                    Dlambda = (lambda * Sinas2 * DAngle) / (Cosas2 * NbSpan);

                    for (i = 1, jj = low; i <= NbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(Poles[jj].XYZ() + Poles[jj + 2].XYZ() - 2.0 * Center.XYZ());
                        Poles[jj + 1].SetXYZ(Center.XYZ() + lambda * temp.XYZ());
                        DPoles[jj + 1].SetLinearForm(Dlambda,
                                                     temp,
                                                     1.0 - 2 * lambda,
                                                     DCenter,
                                                     lambda,
                                                     (DPoles[jj] + DPoles[jj + 2]));
                    }

                    // Les poids
                    Dlambda = -Sinas2 * DAngle / (2 * NbSpan);
                    for (i = low; i < upp; i += 2)
                    {
                        Weights[i] = 1.0;
                        Weights[i + 1] = Cosas2;
                        DWeights[i] = 0.0;
                        DWeights[i + 1] = Dlambda;
                    }
                }
                return true;
        }
        //  return false;
    }

    /// <summary>The poles and weights of the arc and their first and second derivatives.</summary>
    public static bool GetCircle(Convert_ParameterisationType TConv,
                                 in gp_Vec ns1,
                                 in gp_Vec ns2,
                                 in gp_Vec dn1w,
                                 in gp_Vec dn2w,
                                 in gp_Vec d2n1w,
                                 in gp_Vec d2n2w,
                                 in gp_Vec nplan,
                                 in gp_Vec dnplan,
                                 in gp_Vec d2nplan,
                                 in gp_Pnt pts1,
                                 in gp_Pnt pts2,
                                 in gp_Vec tang1,
                                 in gp_Vec tang2,
                                 in gp_Vec Dtang1,
                                 in gp_Vec Dtang2,
                                 double Rayon,
                                 double DRayon,
                                 double D2Rayon,
                                 in gp_Pnt Center,
                                 in gp_Vec DCenter,
                                 in gp_Vec D2Center,
                                 NCollection_Array1<gp_Pnt> Poles,
                                 NCollection_Array1<gp_Vec> DPoles,
                                 NCollection_Array1<gp_Vec> D2Poles,
                                 NCollection_Array1<double> Weights,
                                 NCollection_Array1<double> DWeights,
                                 NCollection_Array1<double> D2Weights)
    {
        double Cosa, Sina, Cosas2, Sinas2;
        double Angle, DAngle, D2Angle, Alpha;
        double lambda, Dlambda, D2lambda, aux;
        gp_Vec temp = new gp_Vec(), dtemp = new gp_Vec(), np2, dnp2, d2np2;
        int i, jj;
        int NbSpan = (Poles.Length() - 1) / 2;
        int low = Poles.Lower();
        int upp = Poles.Upper();

        Cosa = ns1.Dot(ns2);
        Sina = nplan.Dot(ns1.Crossed(ns2));

        if (Cosa < -1.0)
        {
            Cosa = -1;
            Sina = 0;
        }
        if (Cosa > 1.0)
        {
            Cosa = 1;
            Sina = 0;
        }
        Angle = Math.Acos(Cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (Sina < 0.0)
        {
            if (Cosa > 0.0)
            {
                Angle = -Angle;
            }
            else
            {
                Angle = 2.0 * M_PI - Angle;
            }
        }

        if (Math.Abs(Sina) > Math.Abs(Cosa))
        {
            aux = dn1w.Dot(ns2) + ns1.Dot(dn2w);
            DAngle = -aux / Sina;
            D2Angle = -(d2n1w.Dot(ns2) + 2 * dn1w.Dot(dn2w) + ns1.Dot(d2n2w)) / Sina
                      + aux * (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(dn1w.Crossed(ns2) + ns1.Crossed(dn2w)))
                            / (Sina * Sina);
        }
        else
        {
            temp = dn1w.Crossed(ns2) + ns1.Crossed(dn2w);
            DAngle = (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(temp)) / Cosa;
            D2Angle = (d2nplan.Dot(ns1.Crossed(ns2)) + 2 * dnplan.Dot(temp)
                       + nplan.Dot(d2n1w.Crossed(ns2) + 2 * dn1w.Crossed(dn2w) + ns1.Crossed(d2n2w)))
                          / Cosa
                      - (dn1w.Dot(ns2) + ns1.Dot(dn2w)) * (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(temp))
                            / (Cosa * Cosa);
        }

        // Aux Extremites.
        Poles[low] = pts1;
        Poles[upp] = pts2;
        Weights[low] = 1;
        Weights[upp] = 1;

        DPoles[low] = tang1;
        DPoles[upp] = tang2;
        DWeights[low] = 0;
        DWeights[upp] = 0;

        D2Poles[low] = Dtang1;
        D2Poles[upp] = Dtang2;
        D2Weights[low] = 0;
        D2Weights[upp] = 0;

        switch (TConv)
        {
            case Convert_QuasiAngular:
                {
                    GeomFill_QuasiAngularConvertor QConvertor = new GeomFill_QuasiAngularConvertor();
                    QConvertor.Init();
                    QConvertor.Section(pts1,
                                       tang1,
                                       Dtang1,
                                       Center,
                                       DCenter,
                                       D2Center,
                                       nplan,
                                       dnplan,
                                       d2nplan,
                                       Angle,
                                       DAngle,
                                       D2Angle,
                                       Poles,
                                       DPoles,
                                       D2Poles,
                                       Weights,
                                       DWeights,
                                       D2Weights);
                    return true;
                }
            case Convert_Polynomial:
                {
                    GeomFill_PolynomialConvertor PConvertor = new GeomFill_PolynomialConvertor();
                    PConvertor.Init();
                    PConvertor.Section(pts1,
                                       tang1,
                                       Dtang1,
                                       Center,
                                       DCenter,
                                       D2Center,
                                       nplan,
                                       dnplan,
                                       d2nplan,
                                       Angle,
                                       DAngle,
                                       D2Angle,
                                       Poles,
                                       DPoles,
                                       D2Poles);
                    Weights.Init(1);
                    DWeights.Init(0);
                    D2Weights.Init(0);
                    return true;
                }

            default:
                {
                    np2 = nplan.Crossed(ns1);
                    dnp2 = dnplan.Crossed(ns1).Added(nplan.Crossed(dn1w));
                    // OCCT: nplan ^ dn2w where the second derivative of nplan ^ ns1 needs nplan ^ d2n1w (occt-issues.md).
                    d2np2 = d2nplan.Crossed(ns1).Added(nplan.Crossed(dn2w));
                    d2np2 += 2 * dnplan.Crossed(dn1w);

                    Alpha = Angle / ((double)(NbSpan));
                    Cosas2 = Math.Cos(Alpha / 2);
                    Sinas2 = Math.Sin(Alpha / 2);

                    for (i = 1, jj = low + 2; i <= NbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * Alpha;
                        Cosa = Math.Cos(lambda);
                        Sina = Math.Sin(lambda);
                        temp.SetLinearForm(Cosa - 1, ns1, Sina, np2);
                        Poles[jj].SetXYZ(pts1.XYZ() + Rayon * temp.XYZ());

                        DPoles[jj].SetLinearForm(DRayon, temp, tang1);
                        dtemp.SetLinearForm(-Sina, ns1, Cosa, np2);
                        aux = ((double)(i)) / ((double)(NbSpan));
                        dtemp.Multiply(aux * DAngle);
                        dtemp.Add(((Cosa - 1) * dn1w).Added(Sina * dnp2));
                        DPoles[jj] += Rayon * dtemp;

                        D2Poles[jj].SetLinearForm(D2Rayon, temp, 2 * DRayon, dtemp, Dtang1);
                        temp.SetLinearForm(Cosa - 1, dn2w, Sina, d2np2);
                        dtemp.SetLinearForm(-Sina, ns1, Cosa, np2);
                        temp += (aux * aux * D2Angle) * dtemp;
                        dtemp.SetLinearForm(-Sina, dn1w + np2, Cosa, dnp2, -Cosa, ns1);
                        temp += (aux * DAngle) * dtemp;
                        D2Poles[jj] += Rayon * temp;
                    }

                    lambda = 1.0 / (2.0 * Cosas2 * Cosas2);
                    Dlambda = (lambda * Sinas2 * DAngle) / (Cosas2 * NbSpan);
                    aux = Sinas2 / Cosas2;
                    D2lambda = (Dlambda * aux * DAngle + D2Angle * aux * lambda
                                + (1 + aux * aux) * (DAngle / (2 * NbSpan)) * DAngle * lambda)
                               / NbSpan;
                    for (i = 1, jj = low; i <= NbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(Poles[jj].XYZ() + Poles[jj + 2].XYZ() - 2.0 * Center.XYZ());
                        Poles[jj + 1].SetXYZ(Center.XYZ() + lambda * temp.XYZ());

                        dtemp.SetXYZ(DPoles[jj].XYZ() + DPoles[jj + 2].XYZ() - 2.0 * DCenter.XYZ());
                        DPoles[jj + 1].SetLinearForm(Dlambda, temp, lambda, dtemp, DCenter);
                        D2Poles[jj + 1].SetLinearForm(D2lambda,
                                                      temp,
                                                      2 * Dlambda,
                                                      dtemp,
                                                      lambda,
                                                      (D2Poles[jj] + D2Poles[jj + 2]));
                        D2Poles[jj + 1] += (1 - 2 * lambda) * D2Center;
                    }

                    // Les poids
                    Dlambda = -Sinas2 * DAngle / (2 * NbSpan);
                    D2lambda = -Sinas2 * D2Angle / (2 * NbSpan) - Cosas2 * Math.Pow(DAngle / (2 * NbSpan), 2);

                    for (i = low; i < upp; i += 2)
                    {
                        Weights[i] = 1.0;
                        Weights[i + 1] = Cosas2;
                        DWeights[i] = 0.0;
                        DWeights[i + 1] = Dlambda;
                        D2Weights[i] = 0.0;
                        D2Weights[i + 1] = D2lambda;
                    }
                }
                return true;
        }
    }
}
