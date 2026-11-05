// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronAC.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronAC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A trihedron law with a guide, by curvilinear abscissa: the normal points from the path to the guide's point of
/// the same relative curvilinear abscissa.
/// </summary>
/// <remarks>
/// As in OCCT, the class has its own <c>myCurve</c> (hiding the base's, which stays null); the base's
/// <see cref="TrihedronLaw.GetInterval"/> reads <c>myTrimmed</c>, which <see cref="SetCurve"/> sets.
/// </remarks>
internal class GuideTrihedronAC : TrihedronWithGuide
{
    private readonly CurvlinFunc _guideAC;
    private readonly double _lguide;
    private CurvlinFunc? _curveAC;
    private double _l;
    private new CurveAdaptorBase? _curve;
    private readonly double _uTol;
    private readonly double _sTol;
    private double _orig1;
    private double _orig2;

    public GuideTrihedronAC(CurveAdaptorBase guide)
    {
        _curve = null;
        _guide = guide;
        _trimG = guide;
        _guideAC = new CurvlinFunc(_guide, 1.0e-7);
        _lguide = _guideAC.GetLength();
        _uTol = _sTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _orig1 = 0; // origines pour le cas path multi-edges
        _orig2 = 1;
    }

    public override CurveAdaptorBase Guide() => _guide!;

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        double s = _curveAC!.GetSParameter(param); // abscisse curviligne <=> Param
        double origG = _orig1 + s * (_orig2 - _orig1); // abscisse curv sur le guide (cas multi-edges)
        double tG = _guideAC.GetUParameter(_guide!, origG, 1); // param <=> s sur theGuide

        Vector3D b;
        _trimmed!.D1(param, out Point3D p, out Vector3D to); // point et derivee au parametre Param sur myCurve
        _trimG!.D0(tG, out Point3D pg); // point au parametre tG sur myGuide
        _curPointOnGuide = pg;

        Vector3D n = new Vector3D(p, pg); // vecteur definissant la normale

        // OCCT TODO: finding #8 - no zero-magnitude guard before Normalized()/Magnitude().
        normal = n.Normalized();
        b = to.Crossed(normal);
        biNormal = b / b.Magnitude;
        tangent = normal.Crossed(biNormal);
        tangent = tangent.Normalized();

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        // triedre
        double s, origG, tG, dtg;
        // abscisse curviligne <=> Param
        s = _curveAC!.GetSParameter(param);
        // parametre <=> s sur theGuide
        origG = _orig1 + s * (_orig2 - _orig1);
        // parametre <=> s sur  theGuide
        tG = _guideAC.GetUParameter(_guide!, origG, 1);

        Vector3D b, bPrim;

        _trimmed!.D2(param, out Point3D p, out Vector3D to, out Vector3D dTo);
        _trimG!.D1(tG, out Point3D pg, out Vector3D tg);
        _curPointOnGuide = pg;

        Vector3D n = new Vector3D(p, pg), dn = new Vector3D();
        double norm = n.Magnitude;
        if (norm < 1.0e-12)
        {
            norm = 1;
        }
        n /= norm;

        // OCCT TODO: finding #8 - no zero-magnitude guard before TG.Magnitude()/L division.
        dtg = (_orig2 - _orig1) * (to.Magnitude / tg.Magnitude) * (_lguide / _l);
        dn = Vector3D.LinearForm(dtg, tg, -1, to);
        dn /= norm;

        // triedre
        normal = n;
        b = to.Crossed(normal);
        double normB = b.Magnitude;
        b /= normB;

        biNormal = b;

        tangent = normal.Crossed(biNormal);
        tangent = tangent.Normalized();

        // derivee premiere
        dNormal = Vector3D.LinearForm(-(n.Dot(dn)), n, dn);

        bPrim = new Vector3D();
        bPrim = Vector3D.LinearForm(dTo.Crossed(normal), to.Crossed(dNormal));

        dBiNormal = Vector3D.LinearForm(-(b.Dot(bPrim)), b, bPrim);
        dBiNormal /= normB;

        dTangent = Vector3D.LinearForm(normal.Crossed(dBiNormal), dNormal.Crossed(biNormal));

        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        // abscisse curviligne <=> Param
        double s = _curveAC!.GetSParameter(param);
        // parametre <=> s sur theGuide
        double origG = _orig1 + s * (_orig2 - _orig1);
        double tG = _guideAC.GetUParameter(_guide!, origG, 1);

        _trimmed!.D3(param, out Point3D p, out Vector3D to, out Vector3D dTo, out Vector3D d2To);
        _trimG!.D2(tG, out Point3D pg, out Vector3D tg, out Vector3D dtg);
        _curPointOnGuide = pg;

        double nTo = to.Magnitude;
        double n2To = to.SquareMagnitude;
        double ntg = tg.Magnitude;
        double n2Tp = tg.SquareMagnitude;
        double d2tp_dt2, dtg_dt;
        dtg_dt = (_orig2 - _orig1) * (nTo / ntg) * (_lguide / _l);

        Vector3D n = new Vector3D(p, pg); // vecteur definissant la normale
        double norm = n.Magnitude, ndn;
        // derivee de n par rapport a Param
        Vector3D dn = new Vector3D(), d2n = new Vector3D();
        dn = Vector3D.LinearForm(dtg_dt, tg, -1, to);

        // derivee seconde de tG par rapport a Param
        d2tp_dt2 = (_orig2 - _orig1) * (_lguide / _l)
                   * (dTo.Dot(to) / (nTo * ntg) - n2To * tg * dtg * (_lguide / _l) / (n2Tp * n2Tp));
        // derivee seconde de n par rapport a Param
        d2n = Vector3D.LinearForm(dtg_dt * dtg_dt, dtg, d2tp_dt2, tg, -1, dTo);

        if (norm > 1.0e-9)
        {
            n /= norm;
            dn /= norm;
            d2n /= norm;
        }

        // triedre
        normal = n;

        Vector3D tn, dtn = new Vector3D(), d2TN = new Vector3D();
        tn = to.Crossed(normal);

        double norma = tn.Magnitude;
        if (norma > 1.0e-9)
        {
            tn /= norma;
        }

        biNormal = tn;

        tangent = normal.Crossed(biNormal);

        // derivee premiere du triedre
        ndn = n.Dot(dn);
        dNormal = Vector3D.LinearForm(-ndn, n, dn);

        dtn = Vector3D.LinearForm(dTo.Crossed(normal), to.Crossed(dNormal));
        dtn /= norma;
        double tndtn = tn.Dot(dtn);

        dBiNormal = Vector3D.LinearForm(-tndtn, tn, dtn);

        dTangent = Vector3D.LinearForm(normal.Crossed(dBiNormal), dNormal.Crossed(biNormal));

        // derivee seconde du triedre
        double tn2 = tn.SquareMagnitude;

        d2Normal = Vector3D.LinearForm(-2 * ndn, dn, 3 * ndn * ndn - (dn.SquareMagnitude + n.Dot(d2n)), n, d2n);

        d2TN = Vector3D.LinearForm(1, d2To.Crossed(normal), 2, dTo.Crossed(dNormal), to.Crossed(d2Normal));
        d2TN /= norma;

        // OCCT: TN2 is |TN|^2 (1); the second derivative of the unit vector needs |DTN|^2, so D2BiNormal (and
        // D2Tangent) are off by (|DTN|^2 - 1) * TN
        d2BiNormal = Vector3D.LinearForm(-2 * tndtn, dtn, 3 * tndtn * tndtn - (tn2 + tn.Dot(d2TN)), tn, d2TN);

        d2Tangent = Vector3D.LinearForm(1, d2Normal.Crossed(biNormal), 2, dNormal.Crossed(dBiNormal),
                                normal.Crossed(d2BiNormal));

        return true;
    }

    public override TrihedronLaw Copy()
    {
        GuideTrihedronAC copy = new GuideTrihedronAC(_guide!);
        // OCCT sets the curve even when it is null
        copy.SetCurve(_curve!);
        copy.Origine(_orig1, _orig2);
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase c)
    {
        _curve = c;
        _trimmed = c;
        if (_curve is not null)
        {
            _curveAC = new CurvlinFunc(c, 1.0e-7);
            _l = _curveAC.GetLength();
        }
        return true;
    }

    public override int NbIntervals(Continuity s)
    {
        int nb;
        nb = _curveAC!.NbIntervals(s);
        NCollection_Array1<double> discC = new NCollection_Array1<double>(1, nb + 1);
        _curveAC.Intervals(discC, s);
        nb = _guideAC.NbIntervals(s);
        NCollection_Array1<double> discG = new NCollection_Array1<double>(1, nb + 1);
        _guideAC.Intervals(discG, s);

        NCollection_Sequence<double> seq = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(discC, discG, seq);

        return seq.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> tt, Continuity s)
    {
        int nb, ii;
        nb = _curveAC!.NbIntervals(s);
        NCollection_Array1<double> discC = new NCollection_Array1<double>(1, nb + 1);
        _curveAC.Intervals(discC, s);
        nb = _guideAC.NbIntervals(s);
        NCollection_Array1<double> discG = new NCollection_Array1<double>(1, nb + 1);
        _guideAC.Intervals(discG, s);

        NCollection_Sequence<double> seq = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(discC, discG, seq);
        nb = seq.Length();

        for (ii = 1; ii <= nb; ii++)
        {
            tt[ii] = _curveAC.GetUParameter(_curve!, seq[ii], 1);
        }
    }

    public override void SetInterval(double first, double last)
    {
        _trimmed = _curve!.Trim(first, last, _uTol);
        double sf, sl, u;

        sf = _curveAC!.GetSParameter(first);
        sl = _curveAC.GetSParameter(last);

        u = _orig1 + sf * (_orig2 - _orig1);
        sf = _guideAC.GetUParameter(_guide!, u, 1);
        u = _orig1 + sl * (_orig2 - _orig1);
        sl = _guideAC.GetUParameter(_guide!, u, 1);
        _trimG = _guide!.Trim(sf, sl, _uTol);
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        int ii;
        double t, delta = (_curve!.LastParameter() - _curve.FirstParameter()) / 20.001;

        aTangent = aTangent.WithCoord(0.0, 0.0, 0.0);
        aNormal = aNormal.WithCoord(0.0, 0.0, 0.0);
        aBiNormal = aBiNormal.WithCoord(0.0, 0.0, 0.0);
        Vector3D T = new Vector3D(), n = new Vector3D(), b = new Vector3D();

        for (ii = 1; ii <= 20; ii++)
        {
            t = _curve.FirstParameter() + (ii - 1) * delta;
            D0(t, ref T, ref n, ref b);
            aTangent += T;
            aNormal += n;
            aBiNormal += b;
        }
        aTangent /= 20;
        aNormal /= 20;
        aBiNormal /= 20;
    }

    /// <summary>Returns false.</summary>
    public override bool IsConstant => false;

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve => false;

    /// <summary>Sets the relative curvilinear origins on the guide (for a path of several edges).</summary>
    public override void Origine(double orACR1, double orACR2)
    {
        _orig1 = orACR1;
        _orig2 = orACR2;
    }
}
