// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CircularBlendFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_CircularBlendFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The sweep function of a circular blend of constant radius between two guide curves around a path (a "pipe on
/// an edge"): at each parameter, the arc of radius Radius from Curve1's point to Curve2's, centred near the path.
/// </summary>
/// <remarks>
/// A working object (decision 21): it keeps the caller's adaptors and <see cref="SetInterval"/> trims them into new
/// adaptors.
/// </remarks>
internal class CircularBlendFunc : SweepFunction
{
    private const double TolAng = 1.0e-6;

    private Point3D _bary = new Point3D();
    private readonly double _radius;
    private double _maxang;
    private double _minang;
    private double _distmin;
    private readonly CurveAdaptorBase _path;
    private readonly CurveAdaptorBase _curve1;
    private readonly CurveAdaptorBase _curve2;
    private CurveAdaptorBase _tPath;
    private CurveAdaptorBase _tCurve1;
    private CurveAdaptorBase _tCurve2;
    private readonly int _degree;
    private readonly int _nbKnots;
    private readonly int _nbPoles;
    private readonly ParameterisationType _tConv;
    private bool _myreverse;

    private static Continuity GeomFillNextShape(Continuity s)
    {
        switch (s)
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

    private static void GeomFillFusInt(NCollection_Array1<double> i1, NCollection_Array1<double> i2,
                                       NCollection_Sequence<double> seq)
    {
        int ind1 = 1, ind2 = 1;
        double epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // en suposant que le positionement fonctionne a PConfusion()/2
        double v1, v2;
        // Initialisations : les IND1 et IND2 pointent sur le 1er element
        // de chacune des 2 tables a traiter.

        //--- On remplit TABSOR en parcourant TABLE1 et TABLE2 simultanement ---
        //------------------ en eliminant les occurrences multiples ------------

        while ((ind1 <= i1.Upper()) && (ind2 <= i2.Upper()))
        {
            v1 = i1[ind1];
            v2 = i2[ind2];
            if (Math.Abs(v1 - v2) <= epspar)
            {
                // Ici les elements de I1 et I2 conviennent .
                seq.Append((v1 + v2) / 2);
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Ici l' element de I1 convient.
                seq.Append(v1);
                ind1++;
            }
            else
            {
                // Ici l' element de TABLE2 convient.
                seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > i1.Upper())
        {
            //----- Ici I1 est epuise, on complete avec la fin de TABLE2 -------

            for (; ind2 <= i2.Upper(); ind2++)
            {
                seq.Append(i2[ind2]);
            }
        }

        if (ind2 > i2.Upper())
        {
            //----- Ici I2 est epuise, on complete avec la fin de I1 -------

            for (; ind1 <= i1.Upper(); ind1++)
            {
                seq.Append(i1[ind1]);
            }
        }
    }

    /// <summary>
    /// The blend of radius Radius between Curve1 and Curve2 around Path, polynomial if Polynomial, else rational
    /// (quasi-angular when the arcs exceed 0.65 pi, else in tan(theta/2)).
    /// </summary>
    public CircularBlendFunc(CurveAdaptorBase path, CurveAdaptorBase curve1, CurveAdaptorBase curve2,
                                      double radius, bool polynomial = false)
    {
        _maxang = RealFirst();
        _minang = RealLast();
        _distmin = RealLast();

        // Recopie des arguments
        _path = _tPath = path;
        _curve1 = _tCurve1 = curve1;
        _curve2 = _tCurve2 = curve2;
        _radius = radius;

        // Estimations numeriques
        Discret();

        // Type de convertion ?
        if (polynomial)
        {
            _tConv = global::Kerf.Engine.Geometry.Conversion.ParameterisationType.Polynomial;
        }
        else if (_maxang > 0.65 * M_PI)
        {
            _tConv = QuasiAngular; // car c'est Continue
        }
        else
        {
            _tConv = TgtThetaOver2;
        }
        // car c'est le plus performant

        // On en deduit la structure
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetShape(_maxang, out _nbPoles, out _nbKnots, out _degree, ref _tConv);
    }

    private void Discret()
    {
        double tFirst = _path.FirstParameter();
        double tLast = _path.LastParameter(), t;
        int ii;
        double l1, l2;
        CurveAdaptorBase c;
        Point3D p1, p2, p3, center;
        Vector3D dCenter;

        p1 = _curve1.Value(tFirst);
        p2 = _curve1.Value((tFirst + tLast) / 2.0);
        p3 = _curve1.Value(tLast);
        l1 = p1.DistanceTo(p2) + p2.DistanceTo(p3);

        p1 = _curve2.Value(tFirst);
        p2 = _curve2.Value((tFirst + tLast) / 2.0);
        p3 = _curve2.Value(tLast);
        l2 = p1.DistanceTo(p2) + p2.DistanceTo(p3);

        double l;
        if (l1 > l2)
        {
            l = l1;
            c = _curve1;
        }
        else
        {
            l = l2;
            c = _curve2;
        }

        double fleche = 1.0e-2 * l;
        double angle, cosa, percent;
        QuasiUniformDeflection samp = new QuasiUniformDeflection();
        samp.Initialize(c, fleche);
        _bary = _bary.WithCoord(0.0, 0.0, 0.0);
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D();

        if (samp.IsDone)
        {
            percent = ((double)1) / (2 * samp.PointCount);
            for (ii = 1; ii <= samp.PointCount; ii++)
            {
                t = samp.Parameter(ii);
                _curve1.D0(t, out p1);
                _curve2.D0(t, out p2);
                _path.D0(t, out center);
                ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
                ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());
                ns1 = ns1.Normalized();
                ns2 = ns2.Normalized();
                cosa = ns1.Dot(ns2);
                if (cosa > 1.0)
                {
                    cosa = 1.0;
                }
                angle = Math.Abs(Math.Acos(cosa));
                if (angle > _maxang)
                {
                    _maxang = angle;
                }
                if (angle < _minang)
                {
                    _minang = angle;
                }
                _distmin = Min(_distmin, p1.DistanceTo(p2));
                _bary = new Point3D(_bary.XYZ() + ((p1.XYZ() + p2.XYZ())));
            }
        }
        else
        {
            double delta = (tLast - tFirst) / 20;
            percent = ((double)1) / 42;
            for (ii = 0, t = tFirst; ii <= 20; ii++, t += delta)
            {
                _curve1.D0(t, out p1);
                _curve2.D0(t, out p2);
                _path.D0(t, out center);

                ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
                ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());
                ns1 = ns1.Normalized();
                ns2 = ns2.Normalized();
                cosa = ns1.Dot(ns2);
                if (cosa > 1.0)
                {
                    cosa = 1.0;
                }
                angle = Math.Abs(Math.Acos(cosa));

                if (angle > _maxang)
                {
                    _maxang = angle;
                }
                if (angle < _minang)
                {
                    _minang = angle;
                }
                _distmin = Min(_distmin, p1.DistanceTo(p2));
                _bary = new Point3D(_bary.XYZ() + ((p1.XYZ() + p2.XYZ())));
            }
        }
        _bary = new Point3D(_bary.XYZ() * percent);

        // Faut il inverser la trajectoire ?
        t = (tFirst + tLast) / 2;
        _curve1.D0(t, out p1);
        _curve2.D0(t, out p2);
        _path.D1(t, out center, out dCenter);

        ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
        ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());

        // myreverse = (DCenter.Dot(ns1.Crossed(ns2)) < 0);
        _myreverse = false;
    }

    public override bool D0(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<double> weigths)
    {
        Point3D p1, p2, center;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), nplan;
        Xyz temp = new Xyz();

        // Positionnement
        _tPath.D0(param, out center);
        _tCurve1.D0(param, out p1);
        _tCurve2.D0(param, out p2);
        ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
        ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());
        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
        }
        else
        {
            _tPath.D1(param, out center, out nplan);
            if (_myreverse)
            {
                nplan = nplan.Reversed();
            }
        }

        // Normalisation
        ns1 = ns1.Normalized();
        ns2 = ns2.Normalized();
        nplan = nplan.Normalized();

        temp.SetLinearForm(_radius, ns1.XYZ(), _radius, ns2.XYZ(), 1, p1.XYZ(), p2.XYZ());
        center = new Point3D(0.5 * temp);

        // Section
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, nplan, p1, p2, _radius, center, poles, weigths);

        return true;
    }

    public override bool D1(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths)
    {
        Point3D p1, p2, center;
        double invnorm1, invnorm2, invnormp;
        Vector3D dCenter, nplan, dnplan, dp1, dp2;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), dns1, dns2;
        Xyz temp = new Xyz();

        // Positionemment
        _tPath.D1(param, out center, out dCenter);
        _tCurve1.D1(param, out p1, out dp1);
        _tCurve2.D1(param, out p2, out dp2);

        ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
        ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());
        dns1 = dCenter - dp1;
        dns2 = dCenter - dp2;

        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
            dnplan = dns1.Crossed(ns2).Added(ns1.Crossed(dns2));
        }
        else
        {
            _tPath.D2(param, out center, out nplan, out dnplan);
            if (_myreverse)
            {
                nplan = nplan.Reversed();
                dnplan = dnplan.Reversed();
            }
        }

        // Normalisation
        invnorm1 = ((double)1) / ns1.Magnitude;
        invnorm2 = ((double)1) / ns2.Magnitude;

        ns1 *= invnorm1;
        dns1 = Vector3D.LinearForm(-dns1.Dot(ns1), ns1, dns1);
        dns1 *= invnorm1;

        ns2 *= invnorm2;
        dns2 = Vector3D.LinearForm(-dns2.Dot(ns2), ns2, dns2);
        dns2 *= invnorm2;

        temp.SetLinearForm(_radius, ns1.XYZ(), _radius, ns2.XYZ(), 1, p1.XYZ(), p2.XYZ());
        center = new Point3D(0.5 * temp);
        dCenter = Vector3D.LinearForm(_radius, dns1, _radius, dns2, 1, dp1, dp2);
        dCenter *= 0.5;

        invnormp = ((double)1) / nplan.Magnitude;
        nplan *= invnormp;
        dnplan = Vector3D.LinearForm(-dnplan.Dot(nplan), nplan, dnplan);
        dnplan *= invnormp;

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv,
                           ns1,
                           ns2,
                           dns1,
                           dns2,
                           nplan,
                           dnplan,
                           p1,
                           p2,
                           dp1,
                           dp2,
                           _radius,
                           0,
                           center,
                           dCenter,
                           poles,
                           dPoles,
                           weigths,
                           dWeigths);
        return true;
    }

    public override bool D2(double param,
                            double first,
                            double last,
                            NCollection_Array1<Point3D> poles,
                            NCollection_Array1<Vector3D> dPoles,
                            NCollection_Array1<Vector3D> d2Poles,
                            NCollection_Array1<Point2D> poles2d,
                            NCollection_Array1<Vector2D> dPoles2d,
                            NCollection_Array1<Vector2D> d2Poles2d,
                            NCollection_Array1<double> weigths,
                            NCollection_Array1<double> dWeigths,
                            NCollection_Array1<double> d2Weigths)
    {
        Point3D p1, p2, center;
        double invnorm1, invnorm2, invnormp, sc;
        Vector3D dCenter, d2Center, dp1, dp2, d2P1, d2P2;
        Vector3D nplan, dnplan, d2nplan;
        Vector3D ns1 = new Vector3D(), ns2 = new Vector3D(), dns1, dns2, d2ns1, d2ns2;
        Xyz temp = new Xyz();

        // Positionement
        _tPath.D2(param, out center, out dCenter, out d2Center);
        _tCurve1.D2(param, out p1, out dp1, out d2P1);
        _tCurve2.D2(param, out p2, out dp2, out d2P2);

        ns1 = ns1.WithXYZ(center.XYZ() - p1.XYZ());
        dns1 = dCenter - dp1;
        d2ns1 = d2Center - d2P1;
        ns2 = ns2.WithXYZ(center.XYZ() - p2.XYZ());
        dns2 = dCenter - dp2;
        d2ns2 = d2Center - d2P2;

        if (!ns1.IsParallel(ns2, TolAng))
        {
            nplan = ns1.Crossed(ns2);
            dnplan = dns1.Crossed(ns2).Added(ns1.Crossed(dns2));
            d2nplan = new Vector3D();
            d2nplan = Vector3D.LinearForm(1, d2ns1.Crossed(ns2), 2, dns1.Crossed(dns2), ns1.Crossed(d2ns2));
        }
        else
        {
            _tPath.D3(param, out center, out nplan, out dnplan, out d2nplan);
            if (_myreverse)
            {
                nplan = nplan.Reversed();
                dnplan = dnplan.Reversed();
                d2nplan = d2nplan.Reversed();
            }
        }

        // Normalisation
        invnorm1 = ((double)1) / ns1.Magnitude;
        invnorm2 = ((double)1) / ns2.Magnitude;

        ns1 *= invnorm1;
        sc = dns1.Dot(ns1);
        d2ns1 = Vector3D.LinearForm(3 * sc * sc * invnorm1 - d2ns1.Dot(ns1) - invnorm1 * dns1.SquareMagnitude,
                            ns1,
                            -2 * sc * invnorm1,
                            dns1,
                            d2ns1);
        dns1 = Vector3D.LinearForm(-dns1.Dot(ns1), ns1, dns1);
        d2ns1 *= invnorm1;
        dns1 *= invnorm1;

        ns2 *= invnorm2;
        sc = dns2.Dot(ns2);
        d2ns2 = Vector3D.LinearForm(3 * sc * sc * invnorm2 - d2ns2.Dot(ns2) - invnorm2 * dns2.SquareMagnitude,
                            ns2,
                            -2 * sc * invnorm2,
                            dns2,
                            d2ns2);
        dns2 = Vector3D.LinearForm(-sc, ns2, dns2);
        d2ns2 *= invnorm2;
        dns2 *= invnorm2;

        temp.SetLinearForm(_radius, ns1.XYZ(), _radius, ns2.XYZ(), 1, p1.XYZ(), p2.XYZ());
        center = new Point3D(0.5 * temp);
        dCenter = Vector3D.LinearForm(_radius, dns1, _radius, dns2, 1, dp1, dp2);
        dCenter *= 0.5;
        d2Center = Vector3D.LinearForm(_radius, d2ns1, _radius, d2ns2, 1, d2P1, d2P2);
        d2Center *= 0.5;

        invnormp = ((double)1) / nplan.Magnitude;
        nplan *= invnormp;
        sc = dnplan.Dot(nplan);
        d2nplan = Vector3D.LinearForm(3 * sc * sc * invnormp - d2nplan.Dot(nplan) - invnormp * dnplan.SquareMagnitude,
                              nplan,
                              -2 * sc * invnormp,
                              dnplan,
                              d2nplan);
        dnplan = Vector3D.LinearForm(-sc, nplan, dnplan);
        dnplan *= invnormp;
        d2nplan *= invnormp;

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv,
                           ns1,
                           ns2,
                           dns1,
                           dns2,
                           d2ns1,
                           d2ns2,
                           nplan,
                           dnplan,
                           d2nplan,
                           p1,
                           p2,
                           dp1,
                           dp2,
                           d2P1,
                           d2P2,
                           _radius,
                           0,
                           0,
                           center,
                           dCenter,
                           d2Center,
                           poles,
                           dPoles,
                           d2Poles,
                           weigths,
                           dWeigths,
                           d2Weigths);
        return true;
    }

    public override int Nb2dCurves() => 0;

    public override void SectionShape(out int nbPoles, out int nbKnots, out int degree)
    {
        nbPoles = _nbPoles;
        nbKnots = _nbKnots;
        degree = _degree;
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Knots(_tConv, tKnots);
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Mults(_tConv, tMults);
    }

    public override bool IsRational => (_tConv != Polynomial);

    public override int NbIntervals(Continuity s)
    {
        int NbI_Center, NbI_Cb1, NbI_Cb2, ii;
        NbI_Center = _path.NbIntervals(GeomFillNextShape(s));
        NbI_Cb1 = _curve1.NbIntervals(s);
        NbI_Cb2 = _curve2.NbIntervals(s);

        NCollection_Array1<double> iCenter = new(1, NbI_Center + 1);
        NCollection_Array1<double> iCb1 = new(1, NbI_Cb1 + 1);
        NCollection_Array1<double> iCb2 = new(1, NbI_Cb2 + 1);
        NCollection_Sequence<double> inter = new();

        _path.Intervals(iCenter, GeomFillNextShape(s));
        _curve1.Intervals(iCb1, s);
        _curve2.Intervals(iCb2, s);

        GeomFillFusInt(iCb1, iCb2, inter);

        NCollection_Array1<double> iCbs = new(1, inter.Length());
        for (ii = 1; ii <= iCbs.Length(); ii++)
        {
            iCbs[ii] = inter[ii];
        }

        inter.Clear();
        GeomFillFusInt(iCenter, iCbs, inter);

        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int NbI_Center, NbI_Cb1, NbI_Cb2, ii;
        NbI_Center = _path.NbIntervals(GeomFillNextShape(s));
        NbI_Cb1 = _curve1.NbIntervals(s);
        NbI_Cb2 = _curve2.NbIntervals(s);

        NCollection_Array1<double> iCenter = new(1, NbI_Center + 1);
        NCollection_Array1<double> iCb1 = new(1, NbI_Cb1 + 1);
        NCollection_Array1<double> iCb2 = new(1, NbI_Cb2 + 1);
        NCollection_Sequence<double> inter = new();

        _path.Intervals(iCenter, GeomFillNextShape(s));
        _curve1.Intervals(iCb1, s);
        _curve2.Intervals(iCb2, s);

        GeomFillFusInt(iCb1, iCb2, inter);

        NCollection_Array1<double> iCbs = new(1, inter.Length());
        for (ii = 1; ii <= iCbs.Length(); ii++)
        {
            iCbs[ii] = inter[ii];
        }

        inter.Clear();
        GeomFillFusInt(iCenter, iCbs, inter);

        // Recopie du resultat
        for (ii = 1; ii <= inter.Length(); ii++)
        {
            t[ii] = inter[ii];
        }
    }

    public override void SetInterval(double first, double last)
    {
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _tPath = _path.Trim(first, last, eps);
        _tCurve1 = _curve1.Trim(first, last, eps);
        _tCurve2 = _curve2.Trim(first, last, eps);
    }

    public override void GetTolerance(double boundTol, double surfTol, double angleTol, NCollection_Array1<double> tol3d)
    {
        int low = tol3d.Lower(), up = tol3d.Upper();
        double tol;

        tol = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(_tConv, _minang, _radius, angleTol, surfTol);
        tol3d.Init(surfTol);
        tol3d[low + 1] = tol3d[up - 1] = Min(tol, surfTol);
        tol3d[low] = tol3d[up] = Min(tol, boundTol);
    }

    public override void SetTolerance(double tol3d, double tol2d)
    {
        // y rien a faire !
    }

    public override Point3D BarycentreOfSurf() => _bary;

    public override double MaximalSection() => _maxang * _radius;

    public override void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetMinimalWeights(_tConv, _minang, _maxang, weigths);
    }
}
