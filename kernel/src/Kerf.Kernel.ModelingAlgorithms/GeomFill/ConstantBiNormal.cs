// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>A trihedron law with a constant binormal: the Frenet tangent made normal to the binormal.</summary>
public class ConstantBiNormal : TrihedronLaw
{
    private Vector3D _bn;
    private readonly Frenet _frenet;

    /// <summary>Computes (F/|F|)'.</summary>
    private static Vector3D FDeriv(in Vector3D f, in Vector3D df)
    {
        double norma = f.Magnitude();
        Vector3D result = (df - f * (f * df) / (norma * norma)) / norma;
        return result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static Vector3D DDeriv(in Vector3D f, in Vector3D df, in Vector3D d2F)
    {
        double norma = f.Magnitude();
        Vector3D result = (d2F - 2 * df * (f * df) / (norma * norma)) / norma
                        - f * ((df.SquareMagnitude() + f * d2F - 3 * (f * df) * (f * df) / (norma * norma))
                               / (norma * norma * norma));
        return result;
    }

    public ConstantBiNormal(in Direction3D biNormal)
    {
        _bn = new Vector3D(biNormal);
        _frenet = new Frenet();
    }

    public override TrihedronLaw Copy()
    {
        TrihedronLaw copy = new ConstantBiNormal(new Direction3D(_bn));
        if (_curve is not null)
        {
            copy.SetCurve(_curve);
        }
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase c)
    {
        bool isOK = false;
        base.SetCurve(c);
        if (c is not null)
        {
            isOK = _frenet.SetCurve(c);
        }
        return isOK;
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        // if BN^T != 0 then N = (BN^T).Normalized ; T = N^BN
        // else T = (N^BN).Normalized ; N = BN^T
        _frenet.D0(param, ref tangent, ref normal, ref biNormal);
        biNormal = _bn;
        if (biNormal.Crossed(tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            normal = biNormal.Crossed(tangent).Normalized();
            tangent = normal.Crossed(biNormal);
        }
        else
        {
            tangent = normal.Crossed(biNormal).Normalized();
            normal = biNormal.Crossed(tangent);
        }
        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        Vector3D f, df;
        _frenet.D1(param, ref tangent, ref dTangent, ref normal, ref dNormal, ref biNormal, ref dBiNormal);
        biNormal = _bn;
        dBiNormal = new Vector3D(0, 0, 0);
        if (biNormal.Crossed(tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            f = biNormal.Crossed(tangent);
            df = biNormal.Crossed(dTangent);
            normal = f.Normalized();
            dNormal = FDeriv(f, df);

            tangent = normal.Crossed(biNormal);
            dTangent = dNormal.Crossed(biNormal);
        }
        else
        {
            f = normal.Crossed(biNormal);
            df = dNormal.Crossed(biNormal);
            tangent = f.Normalized();
            dTangent = FDeriv(f, df);

            normal = biNormal.Crossed(tangent);
            dNormal = biNormal.Crossed(dTangent);
        }
        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        Vector3D f, df, d2F;
        _frenet.D2(param, ref tangent, ref dTangent, ref d2Tangent, ref normal, ref dNormal, ref d2Normal, ref biNormal,
                  ref dBiNormal, ref d2BiNormal);
        biNormal = _bn;
        dBiNormal = new Vector3D(0, 0, 0);
        d2BiNormal = new Vector3D(0, 0, 0);
        if (biNormal.Crossed(tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            f = biNormal.Crossed(tangent);
            df = biNormal.Crossed(dTangent);
            d2F = biNormal.Crossed(d2Tangent);
            normal = f.Normalized();
            dNormal = FDeriv(f, df);
            d2Normal = DDeriv(f, df, d2F);

            tangent = normal.Crossed(biNormal);
            dTangent = dNormal.Crossed(biNormal);
            d2Tangent = d2Normal.Crossed(biNormal);
        }
        else
        {
            f = normal.Crossed(biNormal);
            df = dNormal.Crossed(biNormal);
            d2F = d2Normal.Crossed(biNormal);
            tangent = f.Normalized();
            dTangent = FDeriv(f, df);
            d2Tangent = DDeriv(f, df, d2F);

            normal = biNormal.Crossed(tangent);
            dNormal = biNormal.Crossed(dTangent);
            d2Normal = biNormal.Crossed(d2Tangent);
        }
        return true;
    }

    public override int NbIntervals(Continuity s) => _frenet.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _frenet.Intervals(t, s);
    }

    public override void GetAverageLaw(ref Vector3D aTangent, ref Vector3D aNormal, ref Vector3D aBiNormal)
    {
        _frenet.GetAverageLaw(ref aTangent, ref aNormal, ref aBiNormal);
        aBiNormal = _bn;
        if (aBiNormal.Crossed(aTangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            aNormal = aBiNormal.Crossed(aTangent).Normalized();
            aTangent = aNormal.Crossed(aBiNormal);
        }
        else
        {
            aTangent = aNormal.Crossed(aBiNormal).Normalized();
            aNormal = aBiNormal.Crossed(aTangent);
        }
    }

    public override bool IsConstant => _frenet.IsConstant;

    public override bool IsOnlyBy3dCurve
    {
        get
        {
            CurveKind theType = _curve!.GetType();
            Axis3D theAxe;

            switch (theType)
            {
                case Circle:
                {
                    theAxe = _curve.Circle().Axis();
                    break;
                }
                case Ellipse:
                {
                    theAxe = _curve.Ellipse().Axis();
                    break;
                }
                case Hyperbola:
                {
                    theAxe = _curve.Hyperbola().Axis();
                    break;
                }
                case Parabola:
                {
                    theAxe = _curve.Parabola().Axis();
                    break;
                }
                case Line:
                { // Is the normal of the plane of the curve perpendicular to the binormal?
                    Vector3D v0 = new Vector3D();
                    v0 = v0.WithXYZ(_curve.Line().Direction().XYZ());
                    return v0.IsNormal(_bn, global::Kerf.Engine.Foundation.Precision.Angular());
                }
                default:
                    return false; // pas de risques
            }

            // Is the normal of the plane of the curve parallel to the binormal?
            Vector3D v = new Vector3D();
            v = v.WithXYZ(theAxe.Direction().XYZ());
            return v.IsParallel(_bn, global::Kerf.Engine.Foundation.Precision.Angular());
        }
    }
}
