// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A trihedron whose normal makes a constant draft angle with a fixed direction (the binormal given), used by
/// the draft sweeps (GeomFill_LocationDraft).
/// </summary>
public class GeomFill_DraftTrihedron : GeomFill_TrihedronLaw
{
    private gp_Vec B;
    private double myAngle;
    private double myCos;

    // computes (F/|F|)''
    private static gp_Vec DDeriv(in gp_Vec F, in gp_Vec DF, in gp_Vec D2F)
    {
        double Norma = F.Magnitude();
        gp_Vec Result = (D2F - 2 * DF * (F * DF) / (Norma * Norma)) / Norma
                        - F * ((DF.SquareMagnitude() + F * D2F - 3 * (F * DF) * (F * DF) / (Norma * Norma))
                               / (Norma * Norma * Norma));
        return Result;
    }

    public GeomFill_DraftTrihedron(in gp_Vec BiNormal, double Angle)
    {
        B = BiNormal;
        B.Normalize();
        SetAngle(Angle);
    }

    public void SetAngle(double Angle)
    {
        myAngle = M_PI / 2 + Angle;
        myCos = Math.Cos(myAngle);
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        myTrimmed!.D1(Param, out _, out gp_Vec T);
        T.Normalize();
        gp_Vec b = T.Crossed(B);
        double normb = b.Magnitude();
        b /= normb;
        if (normb < 1.0e-12)
        {
            return false;
        }

        gp_Vec v = b.Crossed(T);

        double mu = myCos;
        mu = myCos;

        // La Normal est portee par la regle
        Normal.SetLinearForm(Math.Sqrt(1 - mu * mu), b, mu, v);

        // Le reste suit....
        // La tangente est perpendiculaire a la normale et a la direction de depouille
        Tangent = Normal.Crossed(B);
        Tangent.Normalize();

        BiNormal = Tangent;
        BiNormal.Cross(Normal);

        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        gp_Vec DT = new gp_Vec();
        myTrimmed!.D2(Param, out _, out gp_Vec T, out gp_Vec aux);

        double normT, normb;
        normT = T.Magnitude();
        T /= normT;
        DT.SetLinearForm(-(T.Dot(aux)), T, aux);
        DT /= normT;

        gp_Vec db = new gp_Vec(), b = T.Crossed(B);
        normb = b.Magnitude();
        if (normb < 1.0e-12)
        {
            return false;
        }
        b /= normb;
        aux = DT.Crossed(B);
        db.SetLinearForm(-(b.Dot(aux)), b, aux);
        db /= normb;

        gp_Vec v = b.Crossed(T);
        gp_Vec dv = db.Crossed(T) + b.Crossed(DT);

        double mu = myCos;

        Normal.SetLinearForm(Math.Sqrt(1 - mu * mu), b, mu, v);
        DNormal.SetLinearForm(Math.Sqrt(1 - mu * mu), db, mu, dv);

        Tangent = Normal.Crossed(B);
        normT = Tangent.Magnitude();
        Tangent /= normT;
        aux = DNormal.Crossed(B);
        DTangent.SetLinearForm(-(Tangent.Dot(aux)), Tangent, aux);
        DTangent /= normT;

        BiNormal = Tangent;
        BiNormal.Cross(Normal);
        DBiNormal.SetLinearForm(DTangent.Crossed(Normal), Tangent.Crossed(DNormal));

        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        gp_Vec DT = new gp_Vec(), D2T;
        double dot;

        myTrimmed!.D3(Param, out _, out gp_Vec T, out gp_Vec aux, out gp_Vec aux2);

        double normT, normb;

        D2T = DDeriv(T, aux, aux2);
        normT = T.Magnitude();
        T /= normT;
        dot = T.Dot(aux);
        DT.SetLinearForm(-dot, T, aux);
        DT /= normT;

        gp_Vec db = new gp_Vec(), d2b, b = T.Crossed(B);
        normb = b.Magnitude();
        if (normb < 1.0e-12)
        {
            return false;
        }

        aux = DT.Crossed(B);
        aux2 = D2T.Crossed(B);
        d2b = DDeriv(b, aux, aux2);
        b /= normb;
        dot = b.Dot(aux);
        db.SetLinearForm(-dot, b, aux);
        db /= normb;

        gp_Vec v = b.Crossed(T);
        gp_Vec dv = db.Crossed(T) + b.Crossed(DT);
        gp_Vec d2v = d2b.Crossed(T) + 2 * db.Crossed(DT) + b.Crossed(D2T);

        double mu = myCos, rac;
        rac = Math.Sqrt(1 - mu * mu);

        Normal.SetLinearForm(rac, b, mu, v);
        DNormal.SetLinearForm(rac, db, mu, dv);
        D2Normal.SetLinearForm(rac, d2b, mu, d2v);

        Tangent = Normal.Crossed(B);
        normT = Tangent.Magnitude();

        aux = DNormal.Crossed(B);
        aux2 = D2Normal.Crossed(B);
        D2Tangent = DDeriv(Tangent, aux, aux2);
        Tangent /= normT;
        dot = Tangent.Dot(aux);
        DTangent.SetLinearForm(-dot, Tangent, aux);
        DTangent /= normT;

        BiNormal = Tangent;
        BiNormal.Cross(Normal);
        DBiNormal.SetLinearForm(DTangent.Crossed(Normal), Tangent.Crossed(DNormal));
        D2BiNormal.SetLinearForm(1, D2Tangent.Crossed(Normal), 2, DTangent.Crossed(DNormal), Tangent.Crossed(D2Normal));

        return true;
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_DraftTrihedron copy = new GeomFill_DraftTrihedron(B, myAngle - M_PI / 2);
        copy.SetCurve(myCurve!);
        return copy;
    }

    private static GeomAbs_Shape RaisedShape(GeomAbs_Shape S)
    {
        GeomAbs_Shape tmpS;
        switch (S)
        {
            case GeomAbs_C0:
                tmpS = GeomAbs_C2;
                break;
            case GeomAbs_C1:
                tmpS = GeomAbs_C3;
                break;
            case GeomAbs_C2:
            case GeomAbs_C3:
            case GeomAbs_CN:
                tmpS = GeomAbs_CN;
                break;
            default:
                throw new Standard_OutOfRange();
        }
        return tmpS;
    }

    public override int NbIntervals(GeomAbs_Shape S) => myCurve!.NbIntervals(RaisedShape(S));

    public override void Intervals(NCollection_Array1<double> TT, GeomAbs_Shape S)
    {
        myCurve!.Intervals(TT, RaisedShape(S));
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        int Num = 20; // order of digitalization
        gp_Vec T = new gp_Vec(), N = new gp_Vec(), BN = new gp_Vec();
        ATangent = new gp_Vec(0, 0, 0);
        ANormal = new gp_Vec(0, 0, 0);
        ABiNormal = new gp_Vec(0, 0, 0);
        double Step = (myTrimmed!.LastParameter() - myTrimmed.FirstParameter()) / Num;
        double Param;
        for (int i = 0; i <= Num; i++)
        {
            Param = myTrimmed.FirstParameter() + i * Step;
            if (Param > myTrimmed.LastParameter())
            {
                Param = myTrimmed.LastParameter();
            }
            D0(Param, ref T, ref N, ref BN);
            ATangent += T;
            ANormal += N;
            ABiNormal += BN;
        }

        ANormal /= Num + 1;
        ABiNormal /= Num + 1;
        ATangent /= Num + 1;
    }

    public override bool IsConstant() => myCurve!.GetType() == GeomAbs_Line;

    /// <summary>True when the plane of the curve is perpendicular to the draft direction.</summary>
    public override bool IsOnlyBy3dCurve()
    {
        GeomAbs_CurveType TheType = myCurve!.GetType();
        gp_Ax1 TheAxe;
        switch (TheType)
        {
            case GeomAbs_Circle:
                TheAxe = myCurve.Circle().Axis();
                break;
            case GeomAbs_Ellipse:
                TheAxe = myCurve.Ellipse().Axis();
                break;
            case GeomAbs_Hyperbola:
                TheAxe = myCurve.Hyperbola().Axis();
                break;
            case GeomAbs_Parabola:
                TheAxe = myCurve.Parabola().Axis();
                break;
            case GeomAbs_Line:
            { // La normale du plan de la courbe est il perpendiculaire a la BiNormale ?
                gp_Vec V = new gp_Vec();
                V.SetXYZ(myCurve.Line().Direction().XYZ());
                return V.IsParallel(B, Precision.Angular());
            }
            default:
                return false; // pas de risques
        }

        // La normale du plan de la courbe est il // a la BiNormale ?
        gp_Vec V2 = new gp_Vec();
        V2.SetXYZ(TheAxe.Direction().XYZ());
        return V2.IsParallel(B, Precision.Angular());
    }
}
