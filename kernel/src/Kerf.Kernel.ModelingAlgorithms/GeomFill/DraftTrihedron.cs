// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_DraftTrihedron.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A trihedron whose normal makes a constant draft angle with a fixed direction (the binormal given), used by
/// the draft sweeps (GeomFill_LocationDraft).
/// </summary>
internal class DraftTrihedron : TrihedronLaw
{
    private Vector3D _b;
    private double _angle;
    private double _cos;

    // computes (F/|F|)''
    private static Vector3D DDeriv(in Vector3D f, in Vector3D df, in Vector3D d2F)
    {
        double norma = f.Magnitude;
        Vector3D result = (d2F - 2 * df * (f * df) / (norma * norma)) / norma
                        - f * ((df.SquareMagnitude + f * d2F - 3 * (f * df) * (f * df) / (norma * norma))
                               / (norma * norma * norma));
        return result;
    }

    public DraftTrihedron(in Vector3D biNormal, double angle)
    {
        _b = biNormal;
        _b = _b.Normalized();
        SetAngle(angle);
    }

    public void SetAngle(double angle)
    {
        _angle = M_PI / 2 + angle;
        _cos = Math.Cos(_angle);
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        _trimmed!.D1(param, out _, out Vector3D t);
        t = t.Normalized();
        Vector3D b = t.Crossed(_b);
        double normb = b.Magnitude;
        b /= normb;
        if (normb < 1.0e-12)
        {
            return false;
        }

        Vector3D v = b.Crossed(t);

        double mu = _cos;
        mu = _cos;

        // La Normal est portee par la regle
        normal = Vector3D.LinearForm(Math.Sqrt(1 - mu * mu), b, mu, v);

        // Le reste suit....
        // La tangente est perpendiculaire a la normale et a la direction de depouille
        tangent = normal.Crossed(_b);
        tangent = tangent.Normalized();

        biNormal = tangent;
        biNormal = biNormal.Crossed(normal);

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        Vector3D dt = new Vector3D();
        _trimmed!.D2(param, out _, out Vector3D t, out Vector3D aux);

        double normT, normb;
        normT = t.Magnitude;
        t /= normT;
        dt = Vector3D.LinearForm(-(t.Dot(aux)), t, aux);
        dt /= normT;

        Vector3D db = new Vector3D(), b = t.Crossed(_b);
        normb = b.Magnitude;
        if (normb < 1.0e-12)
        {
            return false;
        }
        b /= normb;
        aux = dt.Crossed(_b);
        db = Vector3D.LinearForm(-(b.Dot(aux)), b, aux);
        db /= normb;

        Vector3D v = b.Crossed(t);
        Vector3D dv = db.Crossed(t) + b.Crossed(dt);

        double mu = _cos;

        normal = Vector3D.LinearForm(Math.Sqrt(1 - mu * mu), b, mu, v);
        dNormal = Vector3D.LinearForm(Math.Sqrt(1 - mu * mu), db, mu, dv);

        tangent = normal.Crossed(_b);
        normT = tangent.Magnitude;
        tangent /= normT;
        aux = dNormal.Crossed(_b);
        dTangent = Vector3D.LinearForm(-(tangent.Dot(aux)), tangent, aux);
        dTangent /= normT;

        biNormal = tangent;
        biNormal = biNormal.Crossed(normal);
        dBiNormal = Vector3D.LinearForm(dTangent.Crossed(normal), tangent.Crossed(dNormal));

        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        Vector3D dt = new Vector3D(), d2T;
        double dot;

        _trimmed!.D3(param, out _, out Vector3D t, out Vector3D aux, out Vector3D aux2);

        double normT, normb;

        d2T = DDeriv(t, aux, aux2);
        normT = t.Magnitude;
        t /= normT;
        dot = t.Dot(aux);
        dt = Vector3D.LinearForm(-dot, t, aux);
        dt /= normT;

        Vector3D db = new Vector3D(), d2b, b = t.Crossed(_b);
        normb = b.Magnitude;
        if (normb < 1.0e-12)
        {
            return false;
        }

        aux = dt.Crossed(_b);
        aux2 = d2T.Crossed(_b);
        d2b = DDeriv(b, aux, aux2);
        b /= normb;
        dot = b.Dot(aux);
        db = Vector3D.LinearForm(-dot, b, aux);
        db /= normb;

        Vector3D v = b.Crossed(t);
        Vector3D dv = db.Crossed(t) + b.Crossed(dt);
        Vector3D d2v = d2b.Crossed(t) + 2 * db.Crossed(dt) + b.Crossed(d2T);

        double mu = _cos, rac;
        rac = Math.Sqrt(1 - mu * mu);

        normal = Vector3D.LinearForm(rac, b, mu, v);
        dNormal = Vector3D.LinearForm(rac, db, mu, dv);
        d2Normal = Vector3D.LinearForm(rac, d2b, mu, d2v);

        tangent = normal.Crossed(_b);
        normT = tangent.Magnitude;

        aux = dNormal.Crossed(_b);
        aux2 = d2Normal.Crossed(_b);
        d2Tangent = DDeriv(tangent, aux, aux2);
        tangent /= normT;
        dot = tangent.Dot(aux);
        dTangent = Vector3D.LinearForm(-dot, tangent, aux);
        dTangent /= normT;

        biNormal = tangent;
        biNormal = biNormal.Crossed(normal);
        dBiNormal = Vector3D.LinearForm(dTangent.Crossed(normal), tangent.Crossed(dNormal));
        d2BiNormal = Vector3D.LinearForm(1, d2Tangent.Crossed(normal), 2, dTangent.Crossed(dNormal), tangent.Crossed(d2Normal));

        return true;
    }

    public override TrihedronLaw Copy()
    {
        DraftTrihedron copy = new DraftTrihedron(_b, _angle - M_PI / 2);
        copy.SetCurve(_curve!);
        return copy;
    }

    private static Continuity RaisedShape(Continuity s)
    {
        Continuity tmpS;
        switch (s)
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

    public override int NbIntervals(Continuity s) => _curve!.NbIntervals(RaisedShape(s));

    public override void Intervals(NCollection_Array1<double> tt, Continuity s)
    {
        _curve!.Intervals(tt, RaisedShape(s));
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        int num = 20; // order of digitalization
        Vector3D t = new Vector3D(), n = new Vector3D(), bn = new Vector3D();
        aTangent = new Vector3D(0, 0, 0);
        aNormal = new Vector3D(0, 0, 0);
        aBiNormal = new Vector3D(0, 0, 0);
        double step = (_trimmed!.LastParameter() - _trimmed.FirstParameter()) / num;
        double param;
        for (int i = 0; i <= num; i++)
        {
            param = _trimmed.FirstParameter() + i * step;
            if (param > _trimmed.LastParameter())
            {
                param = _trimmed.LastParameter();
            }
            D0(param, ref t, ref n, ref bn);
            aTangent += t;
            aNormal += n;
            aBiNormal += bn;
        }

        aNormal /= num + 1;
        aBiNormal /= num + 1;
        aTangent /= num + 1;
    }

    public override bool IsConstant => _curve!.GetType() == Line;

    /// <summary>True when the plane of the curve is perpendicular to the draft direction.</summary>
    public override bool IsOnlyBy3dCurve
    {
        get
        {
            CurveKind theType = _curve!.GetType();
            Axis3D theAxe;
            switch (theType)
            {
                case Circle:
                    theAxe = _curve.Circle().Axis;
                    break;
                case Ellipse:
                    theAxe = _curve.Ellipse().Axis;
                    break;
                case Hyperbola:
                    theAxe = _curve.Hyperbola().Axis;
                    break;
                case Parabola:
                    theAxe = _curve.Parabola().Axis;
                    break;
                case Line:
                { // La normale du plan de la courbe est il perpendiculaire a la BiNormale ?
                    Vector3D v = new Vector3D().WithXYZ(_curve.Line().Direction.XYZ());
                    return v.IsParallel(_b, global::Kerf.Engine.Foundation.Precision.Angular());
                }
                default:
                    return false; // pas de risques
            }

            // La normale du plan de la courbe est il // a la BiNormale ?
            Vector3D v2 = new Vector3D().WithXYZ(theAxe.Direction.XYZ());
            return v2.IsParallel(_b, global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }
}
