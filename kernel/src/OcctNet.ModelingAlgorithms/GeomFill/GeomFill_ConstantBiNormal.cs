// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>A trihedron law with a constant binormal: the Frenet tangent made normal to the binormal.</summary>
public class GeomFill_ConstantBiNormal : GeomFill_TrihedronLaw
{
    private Vector3D BN;
    private readonly GeomFill_Frenet frenet;

    /// <summary>Computes (F/|F|)'.</summary>
    private static Vector3D FDeriv(in Vector3D F, in Vector3D DF)
    {
        double Norma = F.Magnitude();
        Vector3D Result = (DF - F * (F * DF) / (Norma * Norma)) / Norma;
        return Result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static Vector3D DDeriv(in Vector3D F, in Vector3D DF, in Vector3D D2F)
    {
        double Norma = F.Magnitude();
        Vector3D Result = (D2F - 2 * DF * (F * DF) / (Norma * Norma)) / Norma
                        - F * ((DF.SquareMagnitude() + F * D2F - 3 * (F * DF) * (F * DF) / (Norma * Norma))
                               / (Norma * Norma * Norma));
        return Result;
    }

    public GeomFill_ConstantBiNormal(in Direction3D BiNormal)
    {
        BN = new Vector3D(BiNormal);
        frenet = new GeomFill_Frenet();
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_TrihedronLaw copy = new GeomFill_ConstantBiNormal(new Direction3D(BN));
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    public override bool SetCurve(CurveAdaptorBase C)
    {
        bool isOK = false;
        base.SetCurve(C);
        if (C is not null)
        {
            isOK = frenet.SetCurve(C);
        }
        return isOK;
    }

    public override bool D0(double Param, ref Vector3D Tangent, ref Vector3D Normal, ref Vector3D BiNormal)
    {
        // if BN^T != 0 then N = (BN^T).Normalized ; T = N^BN
        // else T = (N^BN).Normalized ; N = BN^T
        frenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);
        BiNormal = BN;
        if (BiNormal.Crossed(Tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Normal = BiNormal.Crossed(Tangent).Normalized();
            Tangent = Normal.Crossed(BiNormal);
        }
        else
        {
            Tangent = Normal.Crossed(BiNormal).Normalized();
            Normal = BiNormal.Crossed(Tangent);
        }
        return true;
    }

    public override bool D1(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D Normal, ref Vector3D DNormal,
                            ref Vector3D BiNormal, ref Vector3D DBiNormal)
    {
        Vector3D F, DF;
        frenet.D1(Param, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal);
        BiNormal = BN;
        DBiNormal = new Vector3D(0, 0, 0);
        if (BiNormal.Crossed(Tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            F = BiNormal.Crossed(Tangent);
            DF = BiNormal.Crossed(DTangent);
            Normal = F.Normalized();
            DNormal = FDeriv(F, DF);

            Tangent = Normal.Crossed(BiNormal);
            DTangent = DNormal.Crossed(BiNormal);
        }
        else
        {
            F = Normal.Crossed(BiNormal);
            DF = DNormal.Crossed(BiNormal);
            Tangent = F.Normalized();
            DTangent = FDeriv(F, DF);

            Normal = BiNormal.Crossed(Tangent);
            DNormal = BiNormal.Crossed(DTangent);
        }
        return true;
    }

    public override bool D2(double Param, ref Vector3D Tangent, ref Vector3D DTangent, ref Vector3D D2Tangent, ref Vector3D Normal,
                            ref Vector3D DNormal, ref Vector3D D2Normal, ref Vector3D BiNormal, ref Vector3D DBiNormal,
                            ref Vector3D D2BiNormal)
    {
        Vector3D F, DF, D2F;
        frenet.D2(Param, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal, ref BiNormal,
                  ref DBiNormal, ref D2BiNormal);
        BiNormal = BN;
        DBiNormal = new Vector3D(0, 0, 0);
        D2BiNormal = new Vector3D(0, 0, 0);
        if (BiNormal.Crossed(Tangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            F = BiNormal.Crossed(Tangent);
            DF = BiNormal.Crossed(DTangent);
            D2F = BiNormal.Crossed(D2Tangent);
            Normal = F.Normalized();
            DNormal = FDeriv(F, DF);
            D2Normal = DDeriv(F, DF, D2F);

            Tangent = Normal.Crossed(BiNormal);
            DTangent = DNormal.Crossed(BiNormal);
            D2Tangent = D2Normal.Crossed(BiNormal);
        }
        else
        {
            F = Normal.Crossed(BiNormal);
            DF = DNormal.Crossed(BiNormal);
            D2F = D2Normal.Crossed(BiNormal);
            Tangent = F.Normalized();
            DTangent = FDeriv(F, DF);
            D2Tangent = DDeriv(F, DF, D2F);

            Normal = BiNormal.Crossed(Tangent);
            DNormal = BiNormal.Crossed(DTangent);
            D2Normal = BiNormal.Crossed(D2Tangent);
        }
        return true;
    }

    public override int NbIntervals(Continuity S) => frenet.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        frenet.Intervals(T, S);
    }

    public override void GetAverageLaw(ref Vector3D ATangent, ref Vector3D ANormal, ref Vector3D ABiNormal)
    {
        frenet.GetAverageLaw(ref ATangent, ref ANormal, ref ABiNormal);
        ABiNormal = BN;
        if (ABiNormal.Crossed(ATangent).Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ANormal = ABiNormal.Crossed(ATangent).Normalized();
            ATangent = ANormal.Crossed(ABiNormal);
        }
        else
        {
            ATangent = ANormal.Crossed(ABiNormal).Normalized();
            ANormal = ABiNormal.Crossed(ATangent);
        }
    }

    public override bool IsConstant() => frenet.IsConstant();

    public override bool IsOnlyBy3dCurve()
    {
        CurveKind TheType = myCurve!.GetType();
        Axis3D TheAxe;

        switch (TheType)
        {
            case Circle:
            {
                TheAxe = myCurve.Circle().Axis();
                break;
            }
            case Ellipse:
            {
                TheAxe = myCurve.Ellipse().Axis();
                break;
            }
            case Hyperbola:
            {
                TheAxe = myCurve.Hyperbola().Axis();
                break;
            }
            case Parabola:
            {
                TheAxe = myCurve.Parabola().Axis();
                break;
            }
            case Line:
            { // Is the normal of the plane of the curve perpendicular to the binormal?
                Vector3D V0 = new Vector3D();
                V0.SetXYZ(myCurve.Line().Direction().XYZ());
                return V0.IsNormal(BN, global::Kerf.Engine.Foundation.Precision.Angular());
            }
            default:
                return false; // pas de risques
        }

        // Is the normal of the plane of the curve parallel to the binormal?
        Vector3D V = new Vector3D();
        V.SetXYZ(TheAxe.Direction().XYZ());
        return V.IsParallel(BN, global::Kerf.Engine.Foundation.Precision.Angular());
    }
}
