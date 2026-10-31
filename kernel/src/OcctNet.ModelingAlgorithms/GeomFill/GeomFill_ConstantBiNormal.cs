// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstantBiNormal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>A trihedron law with a constant binormal: the Frenet tangent made normal to the binormal.</summary>
public class GeomFill_ConstantBiNormal : GeomFill_TrihedronLaw
{
    private gp_Vec BN;
    private readonly GeomFill_Frenet frenet;

    /// <summary>Computes (F/|F|)'.</summary>
    private static gp_Vec FDeriv(in gp_Vec F, in gp_Vec DF)
    {
        double Norma = F.Magnitude();
        gp_Vec Result = (DF - F * (F * DF) / (Norma * Norma)) / Norma;
        return Result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static gp_Vec DDeriv(in gp_Vec F, in gp_Vec DF, in gp_Vec D2F)
    {
        double Norma = F.Magnitude();
        gp_Vec Result = (D2F - 2 * DF * (F * DF) / (Norma * Norma)) / Norma
                        - F * ((DF.SquareMagnitude() + F * D2F - 3 * (F * DF) * (F * DF) / (Norma * Norma))
                               / (Norma * Norma * Norma));
        return Result;
    }

    public GeomFill_ConstantBiNormal(in gp_Dir BiNormal)
    {
        BN = new gp_Vec(BiNormal);
        frenet = new GeomFill_Frenet();
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_TrihedronLaw copy = new GeomFill_ConstantBiNormal(new gp_Dir(BN));
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    public override bool SetCurve(Adaptor3d_Curve C)
    {
        bool isOK = false;
        base.SetCurve(C);
        if (C is not null)
        {
            isOK = frenet.SetCurve(C);
        }
        return isOK;
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        // if BN^T != 0 then N = (BN^T).Normalized ; T = N^BN
        // else T = (N^BN).Normalized ; N = BN^T
        frenet.D0(Param, ref Tangent, ref Normal, ref BiNormal);
        BiNormal = BN;
        if (BiNormal.Crossed(Tangent).Magnitude() > Precision.Confusion())
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

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        gp_Vec F, DF;
        frenet.D1(Param, ref Tangent, ref DTangent, ref Normal, ref DNormal, ref BiNormal, ref DBiNormal);
        BiNormal = BN;
        DBiNormal = new gp_Vec(0, 0, 0);
        if (BiNormal.Crossed(Tangent).Magnitude() > Precision.Confusion())
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

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        gp_Vec F, DF, D2F;
        frenet.D2(Param, ref Tangent, ref DTangent, ref D2Tangent, ref Normal, ref DNormal, ref D2Normal, ref BiNormal,
                  ref DBiNormal, ref D2BiNormal);
        BiNormal = BN;
        DBiNormal = new gp_Vec(0, 0, 0);
        D2BiNormal = new gp_Vec(0, 0, 0);
        if (BiNormal.Crossed(Tangent).Magnitude() > Precision.Confusion())
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

    public override int NbIntervals(GeomAbs_Shape S) => frenet.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        frenet.Intervals(T, S);
    }

    public override void GetAverageLaw(ref gp_Vec ATangent, ref gp_Vec ANormal, ref gp_Vec ABiNormal)
    {
        frenet.GetAverageLaw(ref ATangent, ref ANormal, ref ABiNormal);
        ABiNormal = BN;
        if (ABiNormal.Crossed(ATangent).Magnitude() > Precision.Confusion())
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
        GeomAbs_CurveType TheType = myCurve!.GetType();
        gp_Ax1 TheAxe;

        switch (TheType)
        {
            case GeomAbs_Circle:
            {
                TheAxe = myCurve.Circle().Axis();
                break;
            }
            case GeomAbs_Ellipse:
            {
                TheAxe = myCurve.Ellipse().Axis();
                break;
            }
            case GeomAbs_Hyperbola:
            {
                TheAxe = myCurve.Hyperbola().Axis();
                break;
            }
            case GeomAbs_Parabola:
            {
                TheAxe = myCurve.Parabola().Axis();
                break;
            }
            case GeomAbs_Line:
            { // Is the normal of the plane of the curve perpendicular to the binormal?
                gp_Vec V0 = new gp_Vec();
                V0.SetXYZ(myCurve.Line().Direction().XYZ());
                return V0.IsNormal(BN, Precision.Angular());
            }
            default:
                return false; // pas de risques
        }

        // Is the normal of the plane of the curve parallel to the binormal?
        gp_Vec V = new gp_Vec();
        V.SetXYZ(TheAxe.Direction().XYZ());
        return V.IsParallel(BN, Precision.Angular());
    }
}
