// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CircularBlendFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CircularBlendFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The sweep function of a circular blend of constant radius between two guide curves around a path (a "pipe on
/// an edge"): at each parameter, the arc of radius Radius from Curve1's point to Curve2's, centred near the path.
/// </summary>
/// <remarks>
/// A working object (decision 21): it keeps the caller's adaptors and <see cref="SetInterval"/> trims them into new
/// adaptors.
/// </remarks>
public class GeomFill_CircularBlendFunc : Approx_SweepFunction
{
    private const double TolAng = 1.0e-6;

    private Point3D myBary = new Point3D();
    private readonly double myRadius;
    private double maxang;
    private double minang;
    private double distmin;
    private readonly Adaptor3d_Curve myPath;
    private readonly Adaptor3d_Curve myCurve1;
    private readonly Adaptor3d_Curve myCurve2;
    private Adaptor3d_Curve myTPath;
    private Adaptor3d_Curve myTCurve1;
    private Adaptor3d_Curve myTCurve2;
    private readonly int myDegree;
    private readonly int myNbKnots;
    private readonly int myNbPoles;
    private readonly ParameterisationType myTConv;
    private bool myreverse;

    private static Continuity GeomFillNextShape(Continuity S)
    {
        switch (S)
        {
            case C0:
                return C1;
            case C1:
                return C2;
            case C2:
                return C3;
            case C3:
                return CN;
            default:
                return CN;
        }
    }

    private static void GeomFillFusInt(NCollection_Array1<double> I1, NCollection_Array1<double> I2,
                                       NCollection_Sequence<double> Seq)
    {
        int ind1 = 1, ind2 = 1;
        double Epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // en suposant que le positionement fonctionne a PConfusion()/2
        double v1, v2;
        // Initialisations : les IND1 et IND2 pointent sur le 1er element
        // de chacune des 2 tables a traiter.

        //--- On remplit TABSOR en parcourant TABLE1 et TABLE2 simultanement ---
        //------------------ en eliminant les occurrences multiples ------------

        while ((ind1 <= I1.Upper()) && (ind2 <= I2.Upper()))
        {
            v1 = I1[ind1];
            v2 = I2[ind2];
            if (Math.Abs(v1 - v2) <= Epspar)
            {
                // Ici les elements de I1 et I2 conviennent .
                Seq.Append((v1 + v2) / 2);
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Ici l' element de I1 convient.
                Seq.Append(v1);
                ind1++;
            }
            else
            {
                // Ici l' element de TABLE2 convient.
                Seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > I1.Upper())
        {
            //----- Ici I1 est epuise, on complete avec la fin de TABLE2 -------

            for (; ind2 <= I2.Upper(); ind2++)
            {
                Seq.Append(I2[ind2]);
            }
        }

        if (ind2 > I2.Upper())
        {
            //----- Ici I2 est epuise, on complete avec la fin de I1 -------

            for (; ind1 <= I1.Upper(); ind1++)
            {
                Seq.Append(I1[ind1]);
            }
        }
    }

    /// <summary>
    /// The blend of radius Radius between Curve1 and Curve2 around Path, polynomial if Polynomial, else rational
    /// (quasi-angular when the arcs exceed 0.65 pi, else in tan(theta/2)).
    /// </summary>
    public GeomFill_CircularBlendFunc(Adaptor3d_Curve Path, Adaptor3d_Curve Curve1, Adaptor3d_Curve Curve2,
                                      double Radius, bool Polynomial = false)
    {
        maxang = RealFirst();
        minang = RealLast();
        distmin = RealLast();

        // Recopie des arguments
        myPath = myTPath = Path;
        myCurve1 = myTCurve1 = Curve1;
        myCurve2 = myTCurve2 = Curve2;
        myRadius = Radius;

        // Estimations numeriques
        Discret();

        // Type de convertion ?
        if (Polynomial)
        {
            myTConv = global::Kerf.Engine.Geometry.Conversion.ParameterisationType.Polynomial;
        }
        else if (maxang > 0.65 * M_PI)
        {
            myTConv = QuasiAngular; // car c'est Continue
        }
        else
        {
            myTConv = TgtThetaOver2;
        }
        // car c'est le plus performant

        // On en deduit la structure
        GeomFill.GetShape(maxang, out myNbPoles, out myNbKnots, out myDegree, ref myTConv);
    }

    private void Discret()
    {
        double TFirst = myPath.FirstParameter();
        double TLast = myPath.LastParameter(), T;
        int ii;
        double L1, L2;
        Adaptor3d_Curve C;
        Point3D P1, P2, P3, Center;
        Vector3D DCenter;

        P1 = myCurve1.Value(TFirst);
        P2 = myCurve1.Value((TFirst + TLast) / 2.0);
        P3 = myCurve1.Value(TLast);
        L1 = P1.Distance(P2) + P2.Distance(P3);

        P1 = myCurve2.Value(TFirst);
        P2 = myCurve2.Value((TFirst + TLast) / 2.0);
        P3 = myCurve2.Value(TLast);
        L2 = P1.Distance(P2) + P2.Distance(P3);

        double L;
        if (L1 > L2)
        {
            L = L1;
            C = myCurve1;
        }
        else
        {
            L = L2;
            C = myCurve2;
        }

        double Fleche = 1.0e-2 * L;
        double Angle, Cosa, Percent;
        GCPnts_QuasiUniformDeflection Samp = new GCPnts_QuasiUniformDeflection();
        Samp.Initialize(C, Fleche);
        myBary.SetCoord(0.0, 0.0, 0.0);
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D();

        if (Samp.IsDone())
        {
            Percent = ((double)1) / (2 * Samp.NbPoints());
            for (ii = 1; ii <= Samp.NbPoints(); ii++)
            {
                T = Samp.Parameter(ii);
                myCurve1.D0(T, out P1);
                myCurve2.D0(T, out P2);
                myPath.D0(T, out Center);
                ns1.SetXYZ(Center.XYZ() - P1.XYZ());
                ns2.SetXYZ(Center.XYZ() - P2.XYZ());
                ns1.Normalize();
                ns2.Normalize();
                Cosa = ns1.Dot(ns2);
                if (Cosa > 1.0)
                {
                    Cosa = 1.0;
                }
                Angle = Math.Abs(Math.Acos(Cosa));
                if (Angle > maxang)
                {
                    maxang = Angle;
                }
                if (Angle < minang)
                {
                    minang = Angle;
                }
                distmin = Min(distmin, P1.Distance(P2));
                myBary.ChangeCoord() += (P1.XYZ() + P2.XYZ());
            }
        }
        else
        {
            double Delta = (TLast - TFirst) / 20;
            Percent = ((double)1) / 42;
            for (ii = 0, T = TFirst; ii <= 20; ii++, T += Delta)
            {
                myCurve1.D0(T, out P1);
                myCurve2.D0(T, out P2);
                myPath.D0(T, out Center);

                ns1.SetXYZ(Center.XYZ() - P1.XYZ());
                ns2.SetXYZ(Center.XYZ() - P2.XYZ());
                ns1.Normalize();
                ns2.Normalize();
                Cosa = ns1.Dot(ns2);
                if (Cosa > 1.0)
                {
                    Cosa = 1.0;
                }
                Angle = Math.Abs(Math.Acos(Cosa));

                if (Angle > maxang)
                {
                    maxang = Angle;
                }
                if (Angle < minang)
                {
                    minang = Angle;
                }
                distmin = Min(distmin, P1.Distance(P2));
                myBary.ChangeCoord() += (P1.XYZ() + P2.XYZ());
            }
        }
        myBary.ChangeCoord() *= Percent;

        // Faut il inverser la trajectoire ?
        T = (TFirst + TLast) / 2;
        myCurve1.D0(T, out P1);
        myCurve2.D0(T, out P2);
        myPath.D1(T, out Center, out DCenter);

        ns1.SetXYZ(Center.XYZ() - P1.XYZ());
        ns2.SetXYZ(Center.XYZ() - P2.XYZ());

        // myreverse = (DCenter.Dot(ns1.Crossed(ns2)) < 0);
        myreverse = false;
    }

    public override bool D0(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Point2D> Poles2d,
                            NCollection_Array1<double> Weigths)
    {
        Point3D P1, P2, Center;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), nplan;
        Xyz temp = new Xyz();

        // Positionnement
        myTPath.D0(Param, out Center);
        myTCurve1.D0(Param, out P1);
        myTCurve2.D0(Param, out P2);
        ns1.SetXYZ(Center.XYZ() - P1.XYZ());
        ns2.SetXYZ(Center.XYZ() - P2.XYZ());
        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
        }
        else
        {
            myTPath.D1(Param, out Center, out nplan);
            if (myreverse)
            {
                nplan.Reverse();
            }
        }

        // Normalisation
        ns1.Normalize();
        ns2.Normalize();
        nplan.Normalize();

        temp.SetLinearForm(myRadius, ns1.XYZ(), myRadius, ns2.XYZ(), 1, P1.XYZ(), P2.XYZ());
        Center.ChangeCoord() = 0.5 * temp;

        // Section
        GeomFill.GetCircle(myTConv, ns1, ns2, nplan, P1, P2, myRadius, Center, Poles, Weigths);

        return true;
    }

    public override bool D1(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<Point2D> Poles2d,
                            NCollection_Array1<Vector2D> DPoles2d,
                            NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths)
    {
        Point3D P1, P2, Center;
        double invnorm1, invnorm2, invnormp;
        Vector3D DCenter, nplan, dnplan, DP1, DP2;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), Dns1, Dns2;
        Xyz temp = new Xyz();

        // Positionemment
        myTPath.D1(Param, out Center, out DCenter);
        myTCurve1.D1(Param, out P1, out DP1);
        myTCurve2.D1(Param, out P2, out DP2);

        ns1.SetXYZ(Center.XYZ() - P1.XYZ());
        ns2.SetXYZ(Center.XYZ() - P2.XYZ());
        Dns1 = DCenter - DP1;
        Dns2 = DCenter - DP2;

        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
            dnplan = Dns1.Crossed(ns2).Added(ns1.Crossed(Dns2));
        }
        else
        {
            myTPath.D2(Param, out Center, out nplan, out dnplan);
            if (myreverse)
            {
                nplan.Reverse();
                dnplan.Reverse();
            }
        }

        // Normalisation
        invnorm1 = ((double)1) / ns1.Magnitude();
        invnorm2 = ((double)1) / ns2.Magnitude();

        ns1 *= invnorm1;
        Dns1.SetLinearForm(-Dns1.Dot(ns1), ns1, Dns1);
        Dns1 *= invnorm1;

        ns2 *= invnorm2;
        Dns2.SetLinearForm(-Dns2.Dot(ns2), ns2, Dns2);
        Dns2 *= invnorm2;

        temp.SetLinearForm(myRadius, ns1.XYZ(), myRadius, ns2.XYZ(), 1, P1.XYZ(), P2.XYZ());
        Center.ChangeCoord() = 0.5 * temp;
        DCenter.SetLinearForm(myRadius, Dns1, myRadius, Dns2, 1, DP1, DP2);
        DCenter *= 0.5;

        invnormp = ((double)1) / nplan.Magnitude();
        nplan *= invnormp;
        dnplan.SetLinearForm(-dnplan.Dot(nplan), nplan, dnplan);
        dnplan *= invnormp;

        GeomFill.GetCircle(myTConv,
                           ns1,
                           ns2,
                           Dns1,
                           Dns2,
                           nplan,
                           dnplan,
                           P1,
                           P2,
                           DP1,
                           DP2,
                           myRadius,
                           0,
                           Center,
                           DCenter,
                           Poles,
                           DPoles,
                           Weigths,
                           DWeigths);
        return true;
    }

    public override bool D2(double Param,
                            double First,
                            double Last,
                            NCollection_Array1<Point3D> Poles,
                            NCollection_Array1<Vector3D> DPoles,
                            NCollection_Array1<Vector3D> D2Poles,
                            NCollection_Array1<Point2D> Poles2d,
                            NCollection_Array1<Vector2D> DPoles2d,
                            NCollection_Array1<Vector2D> D2Poles2d,
                            NCollection_Array1<double> Weigths,
                            NCollection_Array1<double> DWeigths,
                            NCollection_Array1<double> D2Weigths)
    {
        Point3D P1, P2, Center;
        double invnorm1, invnorm2, invnormp, sc;
        Vector3D DCenter, D2Center, DP1, DP2, D2P1, D2P2;
        Vector3D nplan, dnplan, d2nplan;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), Dns1, Dns2, D2ns1, D2ns2;
        Xyz temp = new Xyz();

        // Positionement
        myTPath.D2(Param, out Center, out DCenter, out D2Center);
        myTCurve1.D2(Param, out P1, out DP1, out D2P1);
        myTCurve2.D2(Param, out P2, out DP2, out D2P2);

        ns1.SetXYZ(Center.XYZ() - P1.XYZ());
        Dns1 = DCenter - DP1;
        D2ns1 = D2Center - D2P1;
        ns2.SetXYZ(Center.XYZ() - P2.XYZ());
        Dns2 = DCenter - DP2;
        D2ns2 = D2Center - D2P2;

        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
            dnplan = Dns1.Crossed(ns2).Added(ns1.Crossed(Dns2));
            d2nplan = new Vector3D();
            d2nplan.SetLinearForm(1, D2ns1.Crossed(ns2), 2, Dns1.Crossed(Dns2), ns1.Crossed(D2ns2));
        }
        else
        {
            myTPath.D3(Param, out Center, out nplan, out dnplan, out d2nplan);
            if (myreverse)
            {
                nplan.Reverse();
                dnplan.Reverse();
                d2nplan.Reverse();
            }
        }

        // Normalisation
        invnorm1 = ((double)1) / ns1.Magnitude();
        invnorm2 = ((double)1) / ns2.Magnitude();

        ns1 *= invnorm1;
        sc = Dns1.Dot(ns1);
        D2ns1.SetLinearForm(3 * sc * sc * invnorm1 - D2ns1.Dot(ns1) - invnorm1 * Dns1.SquareMagnitude(),
                            ns1,
                            -2 * sc * invnorm1,
                            Dns1,
                            D2ns1);
        Dns1.SetLinearForm(-Dns1.Dot(ns1), ns1, Dns1);
        D2ns1 *= invnorm1;
        Dns1 *= invnorm1;

        ns2 *= invnorm2;
        sc = Dns2.Dot(ns2);
        D2ns2.SetLinearForm(3 * sc * sc * invnorm2 - D2ns2.Dot(ns2) - invnorm2 * Dns2.SquareMagnitude(),
                            ns2,
                            -2 * sc * invnorm2,
                            Dns2,
                            D2ns2);
        Dns2.SetLinearForm(-sc, ns2, Dns2);
        D2ns2 *= invnorm2;
        Dns2 *= invnorm2;

        temp.SetLinearForm(myRadius, ns1.XYZ(), myRadius, ns2.XYZ(), 1, P1.XYZ(), P2.XYZ());
        Center.ChangeCoord() = 0.5 * temp;
        DCenter.SetLinearForm(myRadius, Dns1, myRadius, Dns2, 1, DP1, DP2);
        DCenter *= 0.5;
        D2Center.SetLinearForm(myRadius, D2ns1, myRadius, D2ns2, 1, D2P1, D2P2);
        D2Center *= 0.5;

        invnormp = ((double)1) / nplan.Magnitude();
        nplan *= invnormp;
        sc = dnplan.Dot(nplan);
        d2nplan.SetLinearForm(3 * sc * sc * invnormp - d2nplan.Dot(nplan) - invnormp * dnplan.SquareMagnitude(),
                              nplan,
                              -2 * sc * invnormp,
                              dnplan,
                              d2nplan);
        dnplan.SetLinearForm(-sc, nplan, dnplan);
        dnplan *= invnormp;
        d2nplan *= invnormp;

        GeomFill.GetCircle(myTConv,
                           ns1,
                           ns2,
                           Dns1,
                           Dns2,
                           D2ns1,
                           D2ns2,
                           nplan,
                           dnplan,
                           d2nplan,
                           P1,
                           P2,
                           DP1,
                           DP2,
                           D2P1,
                           D2P2,
                           myRadius,
                           0,
                           0,
                           Center,
                           DCenter,
                           D2Center,
                           Poles,
                           DPoles,
                           D2Poles,
                           Weigths,
                           DWeigths,
                           D2Weigths);
        return true;
    }

    public override int Nb2dCurves() => 0;

    public override void SectionShape(out int NbPoles, out int NbKnots, out int Degree)
    {
        NbPoles = myNbPoles;
        NbKnots = myNbKnots;
        Degree = myDegree;
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        GeomFill.Knots(myTConv, TKnots);
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        GeomFill.Mults(myTConv, TMults);
    }

    public override bool IsRational() => (myTConv != Polynomial);

    public override int NbIntervals(Continuity S)
    {
        int NbI_Center, NbI_Cb1, NbI_Cb2, ii;
        NbI_Center = myPath.NbIntervals(GeomFillNextShape(S));
        NbI_Cb1 = myCurve1.NbIntervals(S);
        NbI_Cb2 = myCurve2.NbIntervals(S);

        NCollection_Array1<double> ICenter = new(1, NbI_Center + 1);
        NCollection_Array1<double> ICb1 = new(1, NbI_Cb1 + 1);
        NCollection_Array1<double> ICb2 = new(1, NbI_Cb2 + 1);
        NCollection_Sequence<double> Inter = new();

        myPath.Intervals(ICenter, GeomFillNextShape(S));
        myCurve1.Intervals(ICb1, S);
        myCurve2.Intervals(ICb2, S);

        GeomFillFusInt(ICb1, ICb2, Inter);

        NCollection_Array1<double> ICbs = new(1, Inter.Length());
        for (ii = 1; ii <= ICbs.Length(); ii++)
        {
            ICbs[ii] = Inter[ii];
        }

        Inter.Clear();
        GeomFillFusInt(ICenter, ICbs, Inter);

        return Inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        int NbI_Center, NbI_Cb1, NbI_Cb2, ii;
        NbI_Center = myPath.NbIntervals(GeomFillNextShape(S));
        NbI_Cb1 = myCurve1.NbIntervals(S);
        NbI_Cb2 = myCurve2.NbIntervals(S);

        NCollection_Array1<double> ICenter = new(1, NbI_Center + 1);
        NCollection_Array1<double> ICb1 = new(1, NbI_Cb1 + 1);
        NCollection_Array1<double> ICb2 = new(1, NbI_Cb2 + 1);
        NCollection_Sequence<double> Inter = new();

        myPath.Intervals(ICenter, GeomFillNextShape(S));
        myCurve1.Intervals(ICb1, S);
        myCurve2.Intervals(ICb2, S);

        GeomFillFusInt(ICb1, ICb2, Inter);

        NCollection_Array1<double> ICbs = new(1, Inter.Length());
        for (ii = 1; ii <= ICbs.Length(); ii++)
        {
            ICbs[ii] = Inter[ii];
        }

        Inter.Clear();
        GeomFillFusInt(ICenter, ICbs, Inter);

        // Recopie du resultat
        for (ii = 1; ii <= Inter.Length(); ii++)
        {
            T[ii] = Inter[ii];
        }
    }

    public override void SetInterval(double First, double Last)
    {
        double Eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        myTPath = myPath.Trim(First, Last, Eps);
        myTCurve1 = myCurve1.Trim(First, Last, Eps);
        myTCurve2 = myCurve2.Trim(First, Last, Eps);
    }

    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, NCollection_Array1<double> Tol3d)
    {
        int low = Tol3d.Lower(), up = Tol3d.Upper();
        double Tol;

        Tol = GeomFill.GetTolerance(myTConv, minang, myRadius, AngleTol, SurfTol);
        Tol3d.Init(SurfTol);
        Tol3d[low + 1] = Tol3d[up - 1] = Min(Tol, SurfTol);
        Tol3d[low] = Tol3d[up] = Min(Tol, BoundTol);
    }

    public override void SetTolerance(double Tol3d, double Tol2d)
    {
        // y rien a faire !
    }

    public override Point3D BarycentreOfSurf() => myBary;

    public override double MaximalSection() => maxang * myRadius;

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        GeomFill.GetMinimalWeights(myTConv, minang, maxang, Weigths);
    }
}
