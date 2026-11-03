// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A trihedron whose normal makes a constant draft angle with a fixed direction (the binormal given), used by
/// the draft sweeps (GeomFill_LocationDraft).
/// </summary>
public class GeomFill_DraftTrihedron : GeomFill_TrihedronLaw
{
    private Vector3D B;
    private double myAngle;
    private double myCos;

    // computes (F/|F|)''
    private static Vector3D DDeriv(in Vector3D F, in Vector3D DF, in Vector3D D2F)
    {
        double Norma = F.Magnitude();
        Vector3D Result = (D2F - 2 * DF * (F * DF) / (Norma * Norma)) / Norma
                        - F * ((DF.SquareMagnitude() + F * D2F - 3 * (F * DF) * (F * DF) / (Norma * Norma))
                               / (Norma * Norma * Norma));
        return Result;
    }

    public GeomFill_DraftTrihedron(in Vector3D BiNormal, double Angle)
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

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        myTrimmed!.D1(Param, out _, out Vector3D T);
        T.Normalize();
        Vector3D b = T.Crossed(B);
        double normb = b.Magnitude();
        b /= normb;
        if (normb < 1.0e-12)
        {
            return false;
        }

        Vector3D v = b.Crossed(T);

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

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        Vector3D DT = new Vector3D();
        myTrimmed!.D2(Param, out _, out Vector3D T, out Vector3D aux);

        double normT, normb;
        normT = T.Magnitude();
        T /= normT;
        DT.SetLinearForm(-(T.Dot(aux)), T, aux);
        DT /= normT;

        Vector3D db = new Vector3D(), b = T.Crossed(B);
        normb = b.Magnitude();
        if (normb < 1.0e-12)
        {
            return false;
        }
        b /= normb;
        aux = DT.Crossed(B);
        db.SetLinearForm(-(b.Dot(aux)), b, aux);
        db /= normb;

        Vector3D v = b.Crossed(T);
        Vector3D dv = db.Crossed(T) + b.Crossed(DT);

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

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        Vector3D DT = new Vector3D(), D2T;
        double dot;

        myTrimmed!.D3(Param, out _, out Vector3D T, out Vector3D aux, out Vector3D aux2);

        double normT, normb;

        D2T = DDeriv(T, aux, aux2);
        normT = T.Magnitude();
        T /= normT;
        dot = T.Dot(aux);
        DT.SetLinearForm(-dot, T, aux);
        DT /= normT;

        Vector3D db = new Vector3D(), d2b, b = T.Crossed(B);
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

        Vector3D v = b.Crossed(T);
        Vector3D dv = db.Crossed(T) + b.Crossed(DT);
        Vector3D d2v = d2b.Crossed(T) + 2 * db.Crossed(DT) + b.Crossed(D2T);

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

    private static Continuity RaisedShape(Continuity S)
    {
        Continuity tmpS;
        switch (S)
        {
            case C0:
                tmpS = C2;
                break;
            case C1:
                tmpS = C3;
                break;
            case C2:
            case C3:
            case CN:
                tmpS = CN;
                break;
            default:
                throw new OutOfRangeException();
        }
        return tmpS;
    }

    public override int NbIntervals(Continuity S) => myCurve!.NbIntervals(RaisedShape(S));

    public override void Intervals(NCollection_Array1<double> TT, Continuity S)
    {
        myCurve!.Intervals(TT, RaisedShape(S));
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        int Num = 20; // order of digitalization
        Vector3D T = new Vector3D(), N = new Vector3D(), BN = new Vector3D();
        ATangent = new Vector3D(0, 0, 0);
        ANormal = new Vector3D(0, 0, 0);
        ABiNormal = new Vector3D(0, 0, 0);
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

    public override bool IsConstant() => myCurve!.GetType() == Line;

    /// <summary>True when the plane of the curve is perpendicular to the draft direction.</summary>
    public override bool IsOnlyBy3dCurve()
    {
        CurveKind TheType = myCurve!.GetType();
        Axis3D TheAxe;
        switch (TheType)
        {
            case Circle:
                TheAxe = myCurve.Circle().Axis();
                break;
            case Ellipse:
                TheAxe = myCurve.Ellipse().Axis();
                break;
            case Hyperbola:
                TheAxe = myCurve.Hyperbola().Axis();
                break;
            case Parabola:
                TheAxe = myCurve.Parabola().Axis();
                break;
            case Line:
            { // La normale du plan de la courbe est il perpendiculaire a la BiNormale ?
                Vector3D V = new Vector3D();
                V.SetXYZ(myCurve.Line().Direction().XYZ());
                return V.IsParallel(B, global::Kerf.Engine.Foundation.Precision.Angular());
            }
            default:
                return false; // pas de risques
        }

        // La normale du plan de la courbe est il // a la BiNormale ?
        Vector3D V2 = new Vector3D();
        V2.SetXYZ(TheAxe.Direction().XYZ());
        return V2.IsParallel(B, global::Kerf.Engine.Foundation.Precision.Angular());
    }
}
