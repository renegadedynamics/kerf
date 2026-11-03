// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronAC.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_GuideTrihedronAC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A trihedron law with a guide, by curvilinear abscissa: the normal points from the path to the guide's point of
/// the same relative curvilinear abscissa.
/// </summary>
/// <remarks>
/// As in OCCT, the class has its own <c>myCurve</c> (hiding the base's, which stays null); the base's
/// <see cref="GeomFill_TrihedronLaw.GetInterval"/> reads <c>myTrimmed</c>, which <see cref="SetCurve"/> sets.
/// </remarks>
public class GeomFill_GuideTrihedronAC : GeomFill_TrihedronWithGuide
{
    private readonly CurvlinFunc myGuideAC;
    private readonly double Lguide;
    private CurvlinFunc? myCurveAC;
    private double L;
    private new CurveAdaptorBase? myCurve;
    private readonly double UTol;
    private readonly double STol;
    private double Orig1;
    private double Orig2;

    public GeomFill_GuideTrihedronAC(CurveAdaptorBase guide)
    {
        myCurve = null;
        myGuide = guide;
        myTrimG = guide;
        myGuideAC = new CurvlinFunc(myGuide, 1.0e-7);
        Lguide = myGuideAC.GetLength();
        UTol = STol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Orig1 = 0; // origines pour le cas path multi-edges
        Orig2 = 1;
    }

    public override CurveAdaptorBase Guide() => myGuide!;

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        double s = myCurveAC!.GetSParameter(Param); // abscisse curviligne <=> Param
        double OrigG = Orig1 + s * (Orig2 - Orig1); // abscisse curv sur le guide (cas multi-edges)
        double tG = myGuideAC.GetUParameter(myGuide!, OrigG, 1); // param <=> s sur theGuide

        Vector3D B;
        myTrimmed!.D1(Param, out Point3D P, out Vector3D To); // point et derivee au parametre Param sur myCurve
        myTrimG!.D0(tG, out Point3D PG); // point au parametre tG sur myGuide
        myCurPointOnGuide = PG;

        Vector3D n = new Vector3D(P, PG); // vecteur definissant la normale

        // OCCT TODO: finding #8 - no zero-magnitude guard before Normalized()/Magnitude().
        Normal = n.Normalized();
        B = To.Crossed(Normal);
        BiNormal = B / B.Magnitude();
        Tangent = Normal.Crossed(BiNormal);
        Tangent.Normalize();

        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        // triedre
        double s, OrigG, tG, dtg;
        // abscisse curviligne <=> Param
        s = myCurveAC!.GetSParameter(Param);
        // parametre <=> s sur theGuide
        OrigG = Orig1 + s * (Orig2 - Orig1);
        // parametre <=> s sur  theGuide
        tG = myGuideAC.GetUParameter(myGuide!, OrigG, 1);

        Vector3D B, BPrim;

        myTrimmed!.D2(Param, out Point3D P, out Vector3D To, out Vector3D DTo);
        myTrimG!.D1(tG, out Point3D PG, out Vector3D TG);
        myCurPointOnGuide = PG;

        Vector3D n = new Vector3D(P, PG), dn = new Vector3D();
        double Norm = n.Magnitude();
        if (Norm < 1.0e-12)
        {
            Norm = 1;
        }
        n /= Norm;

        // OCCT TODO: finding #8 - no zero-magnitude guard before TG.Magnitude()/L division.
        dtg = (Orig2 - Orig1) * (To.Magnitude() / TG.Magnitude()) * (Lguide / L);
        dn.SetLinearForm(dtg, TG, -1, To);
        dn /= Norm;

        // triedre
        Normal = n;
        B = To.Crossed(Normal);
        double NormB = B.Magnitude();
        B /= NormB;

        BiNormal = B;

        Tangent = Normal.Crossed(BiNormal);
        Tangent.Normalize();

        // derivee premiere
        DNormal.SetLinearForm(-(n.Dot(dn)), n, dn);

        BPrim = new Vector3D();
        BPrim.SetLinearForm(DTo.Crossed(Normal), To.Crossed(DNormal));

        DBiNormal.SetLinearForm(-(B.Dot(BPrim)), B, BPrim);
        DBiNormal /= NormB;

        DTangent.SetLinearForm(Normal.Crossed(DBiNormal), DNormal.Crossed(BiNormal));

        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        // abscisse curviligne <=> Param
        double s = myCurveAC!.GetSParameter(Param);
        // parametre <=> s sur theGuide
        double OrigG = Orig1 + s * (Orig2 - Orig1);
        double tG = myGuideAC.GetUParameter(myGuide!, OrigG, 1);

        myTrimmed!.D3(Param, out Point3D P, out Vector3D To, out Vector3D DTo, out Vector3D D2To);
        myTrimG!.D2(tG, out Point3D PG, out Vector3D TG, out Vector3D DTG);
        myCurPointOnGuide = PG;

        double NTo = To.Magnitude();
        double N2To = To.SquareMagnitude();
        double NTG = TG.Magnitude();
        double N2Tp = TG.SquareMagnitude();
        double d2tp_dt2, dtg_dt;
        dtg_dt = (Orig2 - Orig1) * (NTo / NTG) * (Lguide / L);

        Vector3D n = new Vector3D(P, PG); // vecteur definissant la normale
        double Norm = n.Magnitude(), ndn;
        // derivee de n par rapport a Param
        Vector3D dn = new Vector3D(), d2n = new Vector3D();
        dn.SetLinearForm(dtg_dt, TG, -1, To);

        // derivee seconde de tG par rapport a Param
        d2tp_dt2 = (Orig2 - Orig1) * (Lguide / L)
                   * (DTo.Dot(To) / (NTo * NTG) - N2To * TG * DTG * (Lguide / L) / (N2Tp * N2Tp));
        // derivee seconde de n par rapport a Param
        d2n.SetLinearForm(dtg_dt * dtg_dt, DTG, d2tp_dt2, TG, -1, DTo);

        if (Norm > 1.0e-9)
        {
            n /= Norm;
            dn /= Norm;
            d2n /= Norm;
        }

        // triedre
        Normal = n;

        Vector3D TN, DTN = new Vector3D(), D2TN = new Vector3D();
        TN = To.Crossed(Normal);

        double Norma = TN.Magnitude();
        if (Norma > 1.0e-9)
        {
            TN /= Norma;
        }

        BiNormal = TN;

        Tangent = Normal.Crossed(BiNormal);

        // derivee premiere du triedre
        ndn = n.Dot(dn);
        DNormal.SetLinearForm(-ndn, n, dn);

        DTN.SetLinearForm(DTo.Crossed(Normal), To.Crossed(DNormal));
        DTN /= Norma;
        double TNDTN = TN.Dot(DTN);

        DBiNormal.SetLinearForm(-TNDTN, TN, DTN);

        DTangent.SetLinearForm(Normal.Crossed(DBiNormal), DNormal.Crossed(BiNormal));

        // derivee seconde du triedre
        double TN2 = TN.SquareMagnitude();

        D2Normal.SetLinearForm(-2 * ndn, dn, 3 * ndn * ndn - (dn.SquareMagnitude() + n.Dot(d2n)), n, d2n);

        D2TN.SetLinearForm(1, D2To.Crossed(Normal), 2, DTo.Crossed(DNormal), To.Crossed(D2Normal));
        D2TN /= Norma;

        // OCCT: TN2 is |TN|^2 (1); the second derivative of the unit vector needs |DTN|^2, so D2BiNormal (and
        // D2Tangent) are off by (|DTN|^2 - 1) * TN
        D2BiNormal.SetLinearForm(-2 * TNDTN, DTN, 3 * TNDTN * TNDTN - (TN2 + TN.Dot(D2TN)), TN, D2TN);

        D2Tangent.SetLinearForm(1, D2Normal.Crossed(BiNormal), 2, DNormal.Crossed(DBiNormal),
                                Normal.Crossed(D2BiNormal));

        return true;
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_GuideTrihedronAC copy = new GeomFill_GuideTrihedronAC(myGuide!);
        // OCCT sets the curve even when it is null
        copy.SetCurve(myCurve!);
        copy.Origine(Orig1, Orig2);
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase C)
    {
        myCurve = C;
        myTrimmed = C;
        if (myCurve is not null)
        {
            myCurveAC = new CurvlinFunc(C, 1.0e-7);
            L = myCurveAC.GetLength();
        }
        return true;
    }

    public override int NbIntervals(Continuity S)
    {
        int Nb;
        Nb = myCurveAC!.NbIntervals(S);
        NCollection_Array1<double> DiscC = new NCollection_Array1<double>(1, Nb + 1);
        myCurveAC.Intervals(DiscC, S);
        Nb = myGuideAC.NbIntervals(S);
        NCollection_Array1<double> DiscG = new NCollection_Array1<double>(1, Nb + 1);
        myGuideAC.Intervals(DiscG, S);

        NCollection_Sequence<double> Seq = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(DiscC, DiscG, Seq);

        return Seq.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> TT, Continuity S)
    {
        int Nb, ii;
        Nb = myCurveAC!.NbIntervals(S);
        NCollection_Array1<double> DiscC = new NCollection_Array1<double>(1, Nb + 1);
        myCurveAC.Intervals(DiscC, S);
        Nb = myGuideAC.NbIntervals(S);
        NCollection_Array1<double> DiscG = new NCollection_Array1<double>(1, Nb + 1);
        myGuideAC.Intervals(DiscG, S);

        NCollection_Sequence<double> Seq = new NCollection_Sequence<double>();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(DiscC, DiscG, Seq);
        Nb = Seq.Length();

        for (ii = 1; ii <= Nb; ii++)
        {
            TT[ii] = myCurveAC.GetUParameter(myCurve!, Seq[ii], 1);
        }
    }

    public override void SetInterval(double First, double Last)
    {
        myTrimmed = myCurve!.Trim(First, Last, UTol);
        double Sf, Sl, U;

        Sf = myCurveAC!.GetSParameter(First);
        Sl = myCurveAC.GetSParameter(Last);

        U = Orig1 + Sf * (Orig2 - Orig1);
        Sf = myGuideAC.GetUParameter(myGuide!, U, 1);
        U = Orig1 + Sl * (Orig2 - Orig1);
        Sl = myGuideAC.GetUParameter(myGuide!, U, 1);
        myTrimG = myGuide!.Trim(Sf, Sl, UTol);
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        int ii;
        double t, Delta = (myCurve!.LastParameter() - myCurve.FirstParameter()) / 20.001;

        ATangent.SetCoord(0.0, 0.0, 0.0);
        ANormal.SetCoord(0.0, 0.0, 0.0);
        ABiNormal.SetCoord(0.0, 0.0, 0.0);
        Vector3D T = new Vector3D(), N = new Vector3D(), B = new Vector3D();

        for (ii = 1; ii <= 20; ii++)
        {
            t = myCurve.FirstParameter() + (ii - 1) * Delta;
            D0(t, ref T, ref N, ref B);
            ATangent += T;
            ANormal += N;
            ABiNormal += B;
        }
        ATangent /= 20;
        ANormal /= 20;
        ABiNormal /= 20;
    }

    /// <summary>Returns false.</summary>
    public override bool IsConstant() => false;

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve() => false;

    /// <summary>Sets the relative curvilinear origins on the guide (for a path of several edges).</summary>
    public override void Origine(double OrACR1, double OrACR2)
    {
        Orig1 = OrACR1;
        Orig2 = OrACR2;
    }
}
