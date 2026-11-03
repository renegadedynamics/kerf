// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>Tools and data for filling and sweep surfaces: ruled surfaces and the circular sections of blends.</summary>
public static class GeomFill
{
    /// <summary>
    /// The ruled surface between Curve1 and Curve2: a plane, cylinder or cone where the curves allow, else a B-spline
    /// surface (GeomFill_Generator). The result is frozen.
    /// </summary>
    public static Surface? Surface(Curve3D curve1, Curve3D curve2)
    {
        Surface? surf = InternalSurface(curve1, curve2);
        surf?.Freeze();
        return surf;
    }

    /// <summary>The ruled surface, not frozen, for ported callers.</summary>
    internal static Surface? InternalSurface(Curve3D curve1, Curve3D curve2)
    {
        Curve3D theCurve1, theCurve2;
        Surface? surf = null;

        // recherche du type de la surface resultat:
        // les surfaces reglees particulieres sont :
        //     - les plans
        //     - les cylindres
        //     - les cones
        // dans ces trois cas les courbes doivent etre de meme type :
        //     - ou 2 droites
        //     - ou 2 cercles

        double a1 = 0, a2 = 0, b1 = 0, b2 = 0;
        bool trim1 = false, trim2 = false;
        if (curve1 is TrimmedCurve3D ctrim1)
        {
            theCurve1 = ctrim1.BasisCurve();
            a1 = ctrim1.FirstParameter();
            b1 = ctrim1.LastParameter();
            trim1 = true;
        }
        else
        {
            theCurve1 = (Curve3D)curve1.Copy();
        }
        if (curve2 is TrimmedCurve3D ctrim2)
        {
            theCurve2 = ctrim2.BasisCurve();
            a2 = ctrim2.FirstParameter();
            b2 = ctrim2.LastParameter();
            trim2 = true;
        }
        else
        {
            theCurve2 = (Curve3D)curve2.Copy();
        }

        bool isDone = false;
        // Les deux courbes sont des droites.
        if (theCurve1 is LineCurve3D line1 && theCurve2 is LineCurve3D line2 && trim1 && trim2)
        {
            Line3D l1 = line1.Lin();
            Line3D l2 = line2.Lin();
            Direction3D d1 = l1.Direction();
            Direction3D d2 = l2.Direction();

            if (d1.IsParallel(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                Vector3D p1P2 = new Vector3D(l1.Location(), l2.Location());
                double proj = p1P2.Dot(new Vector3D(d1));

                if (d1.IsEqual(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    if (Math.Abs(a1 - proj - a2) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                        && Math.Abs(b1 - proj - b2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        CoordinateSystem ax = new CoordinateSystem(l1.Location(), d1.Crossed(new Direction3D(p1P2)), d1);
                        PlaneSurface p = new PlaneSurface(ax);
                        double v = p1P2.Dot(new Vector3D(ax.YDirection()));
                        surf = new TrimmedSurface(p, a1, b1, Min(0.0, v), Max(0.0, v));
                        isDone = true;
                    }
                }
                if (d1.IsOpposite(d2, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    if (Math.Abs(a1 - proj + b2) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                        && Math.Abs(b1 - proj + a2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        CoordinateSystem ax = new CoordinateSystem(l1.Location(), d1.Crossed(new Direction3D(p1P2)), d1);
                        PlaneSurface p = new PlaneSurface(ax);
                        double v = p1P2.Dot(new Vector3D(ax.YDirection()));
                        surf = new TrimmedSurface(p, a1, b1, Min(0.0, v), Max(0.0, v));
                        isDone = true;
                    }
                }
            }
        }

        // Les deux courbes sont des cercles.
        else if (theCurve1 is CircleCurve3D circle1 && theCurve2 is CircleCurve3D circle2)
        {
            Circle3D c1 = circle1.Circ();
            Circle3D c2 = circle2.Circ();

            CoordinateSystem A1 = new CoordinateSystem(c1.Position());
            CoordinateSystem A2 = new CoordinateSystem(c2.Position());

            // first, A1 & A2 must be coaxials
            if (A1.Axis().IsCoaxial(A2.Axis(), global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                double v = new Vector3D(A1.Location(), A2.Location()).Dot(new Vector3D(A1.Direction()));
                if (!trim1 && !trim2)
                {
                    if (Math.Abs(c1.Radius() - c2.Radius()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        CylindricalSurface c = new CylindricalSurface(A1, c1.Radius());
                        surf = new TrimmedSurface(c, Min(0.0, v), Max(0.0, v), false);
                    }
                    else
                    {
                        double rad = c2.Radius() - c1.Radius();
                        double ang = Math.Atan(rad / v);
                        if (ang < 0.0)
                        {
                            A1.ZReverse();
                            v = -v;
                            ang = -ang;
                        }
                        ConicalSurface c = new ConicalSurface(A1, ang, c1.Radius());
                        v /= Math.Cos(ang);
                        surf = new TrimmedSurface(c, Min(0.0, v), Max(0.0, v), false);
                    }
                    isDone = true;
                }
                else if (trim1 && trim2)
                {
                }
            }
        }

        if (!isDone)
        {
            SurfaceGenerator generator = new SurfaceGenerator();
            generator.AddCurve(curve1);
            generator.AddCurve(curve2);
            generator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
            surf = generator.InternalSurface();
        }

        return surf;
    }

    /// <summary>
    /// The shape of the B-spline of an arc of angle MaxAng in the conversion TypeConv: Convert_TgtThetaOver2 becomes
    /// Convert_TgtThetaOver2_1 to _3 when the arc needs one to three spans.
    /// </summary>
    public static void GetShape(double maxAng, out int nbPoles, out int nbKnots, out int degree,
                                ref ParameterisationType tConv)
    {
        switch (tConv)
        {
            case QuasiAngular:
                {
                    nbPoles = 7;
                    nbKnots = 2;
                    degree = 6;
                }
                break;
            case Polynomial:
                {
                    nbPoles = 8;
                    nbKnots = 2;
                    degree = 7;
                }
                break;
            default:
                {
                    int nbSpan = (int)(Math.Ceiling(3.0 * Math.Abs(maxAng) / 2.0 / M_PI));
                    nbPoles = 2 * nbSpan + 1;
                    nbKnots = nbSpan + 1;
                    degree = 2;
                    if (nbSpan == 1)
                    {
                        tConv = TgtThetaOver2_1;
                    }
                    else if (nbSpan == 2)
                    {
                        tConv = TgtThetaOver2_2;
                    }
                    else if (nbSpan == 3)
                    {
                        tConv = TgtThetaOver2_3;
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
    public static void GetMinimalWeights(ParameterisationType tConv, double minAng, double maxAng,
                                         NCollection_Array1<double> weights)
    {
        if (tConv == Polynomial)
        {
            weights.Init(1);
        }
        else
        {
            Frame3D popAx2 = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
            Circle3D c = new Circle3D(popAx2, 1);
            TrimmedCurve3D sect1 = new TrimmedCurve3D(new CircleCurve3D(c), 0.0, maxAng);
            BSplineCurve3D ctoBspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(sect1, tConv);
            weights.Assign(ctoBspl.InternalWeightsArray());

            double angle_min = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), minAng);
            TrimmedCurve3D sect2 = new TrimmedCurve3D(new CircleCurve3D(c), 0.0, angle_min);
            ctoBspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(sect2, tConv);
            NCollection_Array1<double> poids = ctoBspl.InternalWeightsArray();

            for (int ii = weights.Lower(); ii <= weights.Upper(); ii++)
            {
                if (poids[ii] < weights[ii])
                {
                    weights[ii] = poids[ii];
                }
            }
        }
    }

    /// <summary>The knots of the circular sections in the conversion TConv.</summary>
    public static void Knots(ParameterisationType tConv, NCollection_Array1<double> tKnots)
    {
        if ((tConv != QuasiAngular) && (tConv != Polynomial))
        {
            int i;
            double val = 0.0;
            for (i = tKnots.Lower(); i <= tKnots.Upper(); i++)
            {
                tKnots[i] = val;
                val = val + 1.0;
            }
        }
        else
        {
            tKnots[1] = 0.0;
            tKnots[2] = 1.0;
        }
    }

    /// <summary>The multiplicities of the circular sections in the conversion TConv.</summary>
    public static void Mults(ParameterisationType tConv, NCollection_Array1<int> tMults)
    {
        switch (tConv)
        {
            case QuasiAngular:
                {
                    tMults[1] = 7;
                    tMults[2] = 7;
                }
                break;
            case Polynomial:
                {
                    tMults[1] = 8;
                    tMults[2] = 8;
                }
                break;

            default:
                {
                    // Cas rational classsique
                    int i;
                    tMults[tMults.Lower()] = 3;
                    for (i = tMults.Lower() + 1; i <= tMults.Upper() - 1; i++)
                    {
                        tMults[i] = 2;
                    }
                    tMults[tMults.Upper()] = 3;
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
    public static double GetTolerance(ParameterisationType tConv, double angleMin, double radius,
                                      double angularTol, double spatialTol)
    {
        Frame3D popAx2 = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
        Circle3D c = new Circle3D(popAx2, radius);
        CircleCurve3D popCircle = new CircleCurve3D(c);
        TrimmedCurve3D sect = new TrimmedCurve3D(popCircle, 0.0, Max(angleMin, 0.02));
        // 0.02 est proche d'1 degree, en desous on ne se preocupe pas de la tngence
        // afin d'eviter des tolerances d'approximation tendant vers 0 !
        BSplineCurve3D ctoBspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(sect, tConv);
        double dist;
        dist = ctoBspl.Pole(1).Distance(ctoBspl.Pole(2)) + spatialTol;
        return dist * angularTol / 2;
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
    public static void GetCircle(ParameterisationType tConv,
                                 in Vector3D ns1, // Normal rentrente au premier point
                                 in Vector3D ns2, // Normal rentrente au second point
                                 in Vector3D nplan, // Normal au plan
                                 in Point3D pts1,
                                 in Point3D pts2,
                                 double rayon, // Rayon (doit etre positif)
                                 in Point3D center,
                                 NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<double> weights)
    {
        // La classe de convertion

        int i, jj;
        double cosa, sina, angle, alpha, cosas2, lambda;
        Vector3D temp = new Vector3D(), np2;
        int low = poles.Lower();
        int upp = poles.Upper();

        cosa = ns1.Dot(ns2);
        sina = nplan.Dot(ns1.Crossed(ns2));

        if (cosa < -1.0)
        {
            cosa = -1;
            sina = 0;
        }
        if (cosa > 1.0)
        {
            cosa = 1;
            sina = 0;
        }
        angle = Math.Acos(cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (sina < 0.0)
        {
            if (cosa > 0.0)
            {
                angle = -angle;
            }
            else
            {
                angle = 2.0 * M_PI - angle;
            }
        }

        switch (tConv)
        {
            case QuasiAngular:
                {
                    QuasiAngularConvertor qConvertor = new QuasiAngularConvertor();
                    qConvertor.Init();
                    qConvertor.Section(pts1, center, nplan, angle, poles, weights);
                    break;
                }
            case Polynomial:
                {
                    PolynomialConvertor pConvertor = new PolynomialConvertor();
                    pConvertor.Init();
                    pConvertor.Section(pts1, center, nplan, angle, poles);
                    weights.Init(1);
                    break;
                }
            default:
                {
                    // Cas Rational, on utilise une expression directe beaucoup plus
                    // performente que GeomConvert
                    // OCCT: the interior poles below are on the circle for ns1 running from the centre to pts1, not
                    // for the inward normal the header documents (occt-issues.md).
                    int nbSpan = (poles.Length() - 1) / 2;

                    poles[low] = pts1;
                    poles[upp] = pts2;
                    weights[low] = 1;
                    weights[upp] = 1;

                    np2 = nplan.Crossed(ns1);

                    alpha = angle / ((double)(nbSpan));
                    cosas2 = Math.Cos(alpha / 2);

                    for (i = 1, jj = low + 2; i <= nbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * alpha;
                        cosa = Math.Cos(lambda);
                        sina = Math.Sin(lambda);
                        temp.SetLinearForm(cosa - 1, ns1, sina, np2);
                        poles[jj].SetXYZ(pts1.XYZ() + rayon * temp.XYZ());
                        weights[jj] = 1;
                    }

                    lambda = 1.0 / (2.0 * cosas2 * cosas2);
                    for (i = 1, jj = low + 1; i <= nbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(poles[jj - 1].XYZ() + poles[jj + 1].XYZ() - 2.0 * center.XYZ());
                        poles[jj].SetXYZ(center.XYZ() + lambda * temp.XYZ());
                        weights[jj] = cosas2;
                    }
                    break;
                }
        }
    }

    /// <summary>The poles and weights of the arc and their first derivatives.</summary>
    public static bool GetCircle(ParameterisationType tConv,
                                 in Vector3D ns1,
                                 in Vector3D ns2,
                                 in Vector3D dn1w,
                                 in Vector3D dn2w,
                                 in Vector3D nplan,
                                 in Vector3D dnplan,
                                 in Point3D pts1,
                                 in Point3D pts2,
                                 in Vector3D tang1,
                                 in Vector3D tang2,
                                 double rayon,
                                 double dRayon,
                                 in Point3D center,
                                 in Vector3D dCenter,
                                 NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<double> weights,
                                 NCollection_Array1<double> dWeights)
    {
        double cosa, sina, cosas2, sinas2, angle, dAngle, alpha, lambda, dlambda;
        Vector3D temp = new Vector3D(), np2, dnp2;
        int i, jj;
        int nbSpan = (poles.Length() - 1) / 2;
        int low = poles.Lower();
        int upp = poles.Upper();

        cosa = ns1.Dot(ns2);
        sina = nplan.Dot(ns1.Crossed(ns2));

        if (cosa < -1.0)
        {
            cosa = -1;
            sina = 0;
        }
        if (cosa > 1.0)
        {
            cosa = 1;
            sina = 0;
        }
        angle = Math.Acos(cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (sina < 0.0)
        {
            if (cosa > 0.0)
            {
                angle = -angle;
            }
            else
            {
                angle = 2.0 * M_PI - angle;
            }
        }

        if (Math.Abs(sina) > Math.Abs(cosa))
        {
            dAngle = -(dn1w.Dot(ns2) + ns1.Dot(dn2w)) / sina;
        }
        else
        {
            dAngle = (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(dn1w.Crossed(ns2) + ns1.Crossed(dn2w))) / cosa;
        }

        // Aux Extremites.
        poles[low] = pts1;
        poles[upp] = pts2;
        weights[low] = 1;
        weights[upp] = 1;

        dPoles[low] = tang1;
        dPoles[upp] = tang2;
        dWeights[low] = 0;
        dWeights[upp] = 0;

        switch (tConv)
        {
            case QuasiAngular:
                {
                    QuasiAngularConvertor qConvertor = new QuasiAngularConvertor();
                    qConvertor.Init();
                    qConvertor.Section(pts1,
                                       tang1,
                                       center,
                                       dCenter,
                                       nplan,
                                       dnplan,
                                       angle,
                                       dAngle,
                                       poles,
                                       dPoles,
                                       weights,
                                       dWeights);
                    return true;
                }
            case Polynomial:
                {
                    PolynomialConvertor pConvertor = new PolynomialConvertor();
                    pConvertor.Init();
                    pConvertor.Section(pts1, tang1, center, dCenter, nplan, dnplan, angle, dAngle, poles, dPoles);
                    weights.Init(1);
                    dWeights.Init(0);
                    return true;
                }

            default:
                // Cas rationel classique
                {
                    np2 = nplan.Crossed(ns1);
                    dnp2 = dnplan.Crossed(ns1).Added(nplan.Crossed(dn1w));

                    alpha = angle / ((double)(nbSpan));
                    cosas2 = Math.Cos(alpha / 2);
                    sinas2 = Math.Sin(alpha / 2);

                    for (i = 1, jj = low + 2; i <= nbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * alpha;
                        cosa = Math.Cos(lambda);
                        sina = Math.Sin(lambda);
                        temp.SetLinearForm(cosa - 1, ns1, sina, np2);
                        poles[jj].SetXYZ(pts1.XYZ() + rayon * temp.XYZ());

                        dPoles[jj].SetLinearForm(dRayon, temp, tang1);
                        temp.SetLinearForm(-sina, ns1, cosa, np2);
                        temp.Multiply(((double)(i)) / ((double)(nbSpan)) * dAngle);
                        temp.Add(((cosa - 1) * dn1w).Added(sina * dnp2));
                        dPoles[jj] += rayon * temp;
                    }

                    lambda = 1.0 / (2.0 * cosas2 * cosas2);
                    dlambda = (lambda * sinas2 * dAngle) / (cosas2 * nbSpan);

                    for (i = 1, jj = low; i <= nbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(poles[jj].XYZ() + poles[jj + 2].XYZ() - 2.0 * center.XYZ());
                        poles[jj + 1].SetXYZ(center.XYZ() + lambda * temp.XYZ());
                        dPoles[jj + 1].SetLinearForm(dlambda,
                                                     temp,
                                                     1.0 - 2 * lambda,
                                                     dCenter,
                                                     lambda,
                                                     (dPoles[jj] + dPoles[jj + 2]));
                    }

                    // Les poids
                    dlambda = -sinas2 * dAngle / (2 * nbSpan);
                    for (i = low; i < upp; i += 2)
                    {
                        weights[i] = 1.0;
                        weights[i + 1] = cosas2;
                        dWeights[i] = 0.0;
                        dWeights[i + 1] = dlambda;
                    }
                }
                return true;
        }
        //  return false;
    }

    /// <summary>The poles and weights of the arc and their first and second derivatives.</summary>
    public static bool GetCircle(ParameterisationType tConv,
                                 in Vector3D ns1,
                                 in Vector3D ns2,
                                 in Vector3D dn1w,
                                 in Vector3D dn2w,
                                 in Vector3D d2n1w,
                                 in Vector3D d2n2w,
                                 in Vector3D nplan,
                                 in Vector3D dnplan,
                                 in Vector3D d2nplan,
                                 in Point3D pts1,
                                 in Point3D pts2,
                                 in Vector3D tang1,
                                 in Vector3D tang2,
                                 in Vector3D dtang1,
                                 in Vector3D dtang2,
                                 double rayon,
                                 double dRayon,
                                 double d2Rayon,
                                 in Point3D center,
                                 in Vector3D dCenter,
                                 in Vector3D d2Center,
                                 NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles,
                                 NCollection_Array1<double> weights,
                                 NCollection_Array1<double> dWeights,
                                 NCollection_Array1<double> d2Weights)
    {
        double cosa, sina, cosas2, sinas2;
        double angle, dAngle, d2Angle, alpha;
        double lambda, dlambda, d2lambda, aux;
        Vector3D temp = new Vector3D(), dtemp = new Vector3D(), np2, dnp2, d2np2;
        int i, jj;
        int nbSpan = (poles.Length() - 1) / 2;
        int low = poles.Lower();
        int upp = poles.Upper();

        cosa = ns1.Dot(ns2);
        sina = nplan.Dot(ns1.Crossed(ns2));

        if (cosa < -1.0)
        {
            cosa = -1;
            sina = 0;
        }
        if (cosa > 1.0)
        {
            cosa = 1;
            sina = 0;
        }
        angle = Math.Acos(cosa);
        // Recadrage sur ]-pi/2, 3pi/2]
        if (sina < 0.0)
        {
            if (cosa > 0.0)
            {
                angle = -angle;
            }
            else
            {
                angle = 2.0 * M_PI - angle;
            }
        }

        if (Math.Abs(sina) > Math.Abs(cosa))
        {
            aux = dn1w.Dot(ns2) + ns1.Dot(dn2w);
            dAngle = -aux / sina;
            d2Angle = -(d2n1w.Dot(ns2) + 2 * dn1w.Dot(dn2w) + ns1.Dot(d2n2w)) / sina
                      + aux * (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(dn1w.Crossed(ns2) + ns1.Crossed(dn2w)))
                            / (sina * sina);
        }
        else
        {
            temp = dn1w.Crossed(ns2) + ns1.Crossed(dn2w);
            dAngle = (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(temp)) / cosa;
            d2Angle = (d2nplan.Dot(ns1.Crossed(ns2)) + 2 * dnplan.Dot(temp)
                       + nplan.Dot(d2n1w.Crossed(ns2) + 2 * dn1w.Crossed(dn2w) + ns1.Crossed(d2n2w)))
                          / cosa
                      - (dn1w.Dot(ns2) + ns1.Dot(dn2w)) * (dnplan.Dot(ns1.Crossed(ns2)) + nplan.Dot(temp))
                            / (cosa * cosa);
        }

        // Aux Extremites.
        poles[low] = pts1;
        poles[upp] = pts2;
        weights[low] = 1;
        weights[upp] = 1;

        dPoles[low] = tang1;
        dPoles[upp] = tang2;
        dWeights[low] = 0;
        dWeights[upp] = 0;

        d2Poles[low] = dtang1;
        d2Poles[upp] = dtang2;
        d2Weights[low] = 0;
        d2Weights[upp] = 0;

        switch (tConv)
        {
            case QuasiAngular:
                {
                    QuasiAngularConvertor qConvertor = new QuasiAngularConvertor();
                    qConvertor.Init();
                    qConvertor.Section(pts1,
                                       tang1,
                                       dtang1,
                                       center,
                                       dCenter,
                                       d2Center,
                                       nplan,
                                       dnplan,
                                       d2nplan,
                                       angle,
                                       dAngle,
                                       d2Angle,
                                       poles,
                                       dPoles,
                                       d2Poles,
                                       weights,
                                       dWeights,
                                       d2Weights);
                    return true;
                }
            case Polynomial:
                {
                    PolynomialConvertor pConvertor = new PolynomialConvertor();
                    pConvertor.Init();
                    pConvertor.Section(pts1,
                                       tang1,
                                       dtang1,
                                       center,
                                       dCenter,
                                       d2Center,
                                       nplan,
                                       dnplan,
                                       d2nplan,
                                       angle,
                                       dAngle,
                                       d2Angle,
                                       poles,
                                       dPoles,
                                       d2Poles);
                    weights.Init(1);
                    dWeights.Init(0);
                    d2Weights.Init(0);
                    return true;
                }

            default:
                {
                    np2 = nplan.Crossed(ns1);
                    dnp2 = dnplan.Crossed(ns1).Added(nplan.Crossed(dn1w));
                    // OCCT: nplan ^ dn2w where the second derivative of nplan ^ ns1 needs nplan ^ d2n1w (occt-issues.md).
                    d2np2 = d2nplan.Crossed(ns1).Added(nplan.Crossed(dn2w));
                    d2np2 += 2 * dnplan.Crossed(dn1w);

                    alpha = angle / ((double)(nbSpan));
                    cosas2 = Math.Cos(alpha / 2);
                    sinas2 = Math.Sin(alpha / 2);

                    for (i = 1, jj = low + 2; i <= nbSpan - 1; i++, jj += 2)
                    {
                        lambda = ((double)(i)) * alpha;
                        cosa = Math.Cos(lambda);
                        sina = Math.Sin(lambda);
                        temp.SetLinearForm(cosa - 1, ns1, sina, np2);
                        poles[jj].SetXYZ(pts1.XYZ() + rayon * temp.XYZ());

                        dPoles[jj].SetLinearForm(dRayon, temp, tang1);
                        dtemp.SetLinearForm(-sina, ns1, cosa, np2);
                        aux = ((double)(i)) / ((double)(nbSpan));
                        dtemp.Multiply(aux * dAngle);
                        dtemp.Add(((cosa - 1) * dn1w).Added(sina * dnp2));
                        dPoles[jj] += rayon * dtemp;

                        d2Poles[jj].SetLinearForm(d2Rayon, temp, 2 * dRayon, dtemp, dtang1);
                        temp.SetLinearForm(cosa - 1, dn2w, sina, d2np2);
                        dtemp.SetLinearForm(-sina, ns1, cosa, np2);
                        temp += (aux * aux * d2Angle) * dtemp;
                        dtemp.SetLinearForm(-sina, dn1w + np2, cosa, dnp2, -cosa, ns1);
                        temp += (aux * dAngle) * dtemp;
                        d2Poles[jj] += rayon * temp;
                    }

                    lambda = 1.0 / (2.0 * cosas2 * cosas2);
                    dlambda = (lambda * sinas2 * dAngle) / (cosas2 * nbSpan);
                    aux = sinas2 / cosas2;
                    d2lambda = (dlambda * aux * dAngle + d2Angle * aux * lambda
                                + (1 + aux * aux) * (dAngle / (2 * nbSpan)) * dAngle * lambda)
                               / nbSpan;
                    for (i = 1, jj = low; i <= nbSpan; i++, jj += 2)
                    {
                        temp.SetXYZ(poles[jj].XYZ() + poles[jj + 2].XYZ() - 2.0 * center.XYZ());
                        poles[jj + 1].SetXYZ(center.XYZ() + lambda * temp.XYZ());

                        dtemp.SetXYZ(dPoles[jj].XYZ() + dPoles[jj + 2].XYZ() - 2.0 * dCenter.XYZ());
                        dPoles[jj + 1].SetLinearForm(dlambda, temp, lambda, dtemp, dCenter);
                        d2Poles[jj + 1].SetLinearForm(d2lambda,
                                                      temp,
                                                      2 * dlambda,
                                                      dtemp,
                                                      lambda,
                                                      (d2Poles[jj] + d2Poles[jj + 2]));
                        d2Poles[jj + 1] += (1 - 2 * lambda) * d2Center;
                    }

                    // Les poids
                    dlambda = -sinas2 * dAngle / (2 * nbSpan);
                    d2lambda = -sinas2 * d2Angle / (2 * nbSpan) - cosas2 * Math.Pow(dAngle / (2 * nbSpan), 2);

                    for (i = low; i < upp; i += 2)
                    {
                        weights[i] = 1.0;
                        weights[i + 1] = cosas2;
                        dWeights[i] = 0.0;
                        dWeights[i + 1] = dlambda;
                        d2Weights[i] = 0.0;
                        d2Weights[i + 1] = d2lambda;
                    }
                }
                return true;
        }
    }
}
