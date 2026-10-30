// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Darboux.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Darboux.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.CSLib_NormalStatus;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The Darboux trihedron law of a curve on a surface: the tangent, the surface normal (as binormal) and their
/// cross product.
/// </summary>
/// <remarks>The curve must be an <see cref="Adaptor3d_CurveOnSurface"/> (OCCT casts it unchecked; the port checks).</remarks>
public class GeomFill_Darboux : GeomFill_TrihedronLaw
{
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

    /// <summary>The normal to the surface (OCCT's file-static NormalD0).</summary>
    private static void NormalD0(double U, double V, Adaptor3d_Surface Surf, ref gp_Dir Normal, out int OrderU,
                                 out int OrderV)
    {
        GeomAbs_Shape Cont = (Surf.UContinuity() < Surf.VContinuity()) ? (Surf.UContinuity()) : (Surf.VContinuity());
        OrderU = OrderV = 0;

        Surf.D1(U, V, out _, out gp_Vec D1U, out gp_Vec D1V);
        double MagTol = 0.000000001;

        CSLib.Normal(D1U, D1V, MagTol, out CSLib_NormalStatus NStatus, ref Normal);

        if (NStatus != CSLib_Defined)
        {
            if (Cont == GeomAbs_C0 || Cont == GeomAbs_C1)
            {
                throw new Geom_UndefinedValue();
            }
            int MaxOrder = 3;
            NCollection_Array2<gp_Vec> DerNUV = new NCollection_Array2<gp_Vec>(0, MaxOrder, 0, MaxOrder);
            NCollection_Array2<gp_Vec> DerSurf = new NCollection_Array2<gp_Vec>(0, MaxOrder + 1, 0, MaxOrder + 1);
            int i, j;
            double Umin, Umax, Vmin, Vmax;
            Umin = Surf.FirstUParameter();
            Umax = Surf.LastUParameter();
            Vmin = Surf.FirstVParameter();
            Vmax = Surf.LastVParameter();
            for (i = 1; i <= MaxOrder + 1; i++)
            {
                DerSurf.SetValue(i, 0, Surf.DN(U, V, i, 0));
            }

            for (i = 0; i <= MaxOrder + 1; i++)
            {
                for (j = 1; j <= MaxOrder + 1; j++)
                {
                    DerSurf.SetValue(i, j, Surf.DN(U, V, i, j));
                }
            }

            for (i = 0; i <= MaxOrder; i++)
            {
                for (j = 0; j <= MaxOrder; j++)
                {
                    DerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurf));
                }
            }

            CSLib.Normal(MaxOrder, DerNUV, MagTol, U, V, Umin, Umax, Vmin, Vmax, out NStatus, ref Normal, out OrderU,
                         out OrderV);

            if (NStatus != CSLib_Defined)
            {
                throw new Geom_UndefinedValue();
            }
        }
    }

    /// <summary>The normal to the surface and its first derivatives (OCCT's file-static NormalD1).</summary>
    private static void NormalD1(double U, double V, Adaptor3d_Surface Surf, ref gp_Dir Normal, out gp_Vec D1UNormal,
                                 out gp_Vec D1VNormal)
    {
        Surf.D2(U, V, out _, out D1UNormal, out D1VNormal, out gp_Vec d2u, out gp_Vec d2v, out gp_Vec d2uv);
        double MagTol = 0.000000001;
        CSLib.Normal(D1UNormal, D1VNormal, MagTol, out CSLib_NormalStatus NStatus, ref Normal);
        int MaxOrder;
        if (NStatus == CSLib_Defined)
        {
            MaxOrder = 0;
        }
        else
        {
            MaxOrder = 3;
        }
        NCollection_Array2<gp_Vec> DerNUV = new NCollection_Array2<gp_Vec>(0, MaxOrder + 1, 0, MaxOrder + 1);
        NCollection_Array2<gp_Vec> DerSurf = new NCollection_Array2<gp_Vec>(0, MaxOrder + 2, 0, MaxOrder + 2);
        int i, j;
        double Umin, Umax, Vmin, Vmax;
        Umin = Surf.FirstUParameter();
        Umax = Surf.LastUParameter();
        Vmin = Surf.FirstVParameter();
        Vmax = Surf.LastVParameter();

        DerSurf.SetValue(1, 0, D1UNormal);
        DerSurf.SetValue(0, 1, D1VNormal);
        DerSurf.SetValue(1, 1, d2uv);
        DerSurf.SetValue(2, 0, d2u);
        DerSurf.SetValue(0, 2, d2v);
        for (i = 0; i <= MaxOrder + 1; i++)
        {
            for (j = i; j <= MaxOrder + 2; j++)
            {
                if (i + j > 2)
                {
                    DerSurf.SetValue(i, j, Surf.DN(U, V, i, j));
                    if (i != j)
                    {
                        DerSurf.SetValue(j, i, Surf.DN(U, V, j, i));
                    }
                }
            }
        }

        for (i = 0; i <= MaxOrder + 1; i++)
        {
            for (j = 0; j <= MaxOrder + 1; j++)
            {
                DerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurf));
            }
        }

        CSLib.Normal(MaxOrder, DerNUV, MagTol, U, V, Umin, Umax, Vmin, Vmax, out NStatus, ref Normal, out int OrderU,
                     out int OrderV);
        if (NStatus != CSLib_Defined)
        {
            throw new Geom_UndefinedValue();
        }

        D1UNormal = CSLib.DNNormal(1, 0, DerNUV, OrderU, OrderV);
        D1VNormal = CSLib.DNNormal(0, 1, DerNUV, OrderU, OrderV);
    }

    /// <summary>The normal to the surface and its first and second derivatives (OCCT's file-static NormalD2).</summary>
    private static void NormalD2(double U, double V, Adaptor3d_Surface Surf, ref gp_Dir Normal, out gp_Vec D1UNormal,
                                 out gp_Vec D1VNormal, out gp_Vec D2UNormal, out gp_Vec D2VNormal,
                                 out gp_Vec D2UVNormal)
    {
        Surf.D3(U, V, out _, out D1UNormal, out D1VNormal, out D2UNormal, out D2VNormal, out D2UVNormal, out gp_Vec d3u,
                out gp_Vec d3v, out gp_Vec d3uuv, out gp_Vec d3uvv);
        double MagTol = 0.000000001;
        CSLib.Normal(D1UNormal, D1VNormal, MagTol, out CSLib_NormalStatus NStatus, ref Normal);
        int MaxOrder;
        if (NStatus == CSLib_Defined)
        {
            MaxOrder = 0;
        }
        else
        {
            MaxOrder = 3;
        }
        NCollection_Array2<gp_Vec> DerNUV = new NCollection_Array2<gp_Vec>(0, MaxOrder + 2, 0, MaxOrder + 2);
        NCollection_Array2<gp_Vec> DerSurf = new NCollection_Array2<gp_Vec>(0, MaxOrder + 3, 0, MaxOrder + 3);
        int i, j;
        double Umin, Umax, Vmin, Vmax;
        Umin = Surf.FirstUParameter();
        Umax = Surf.LastUParameter();
        Vmin = Surf.FirstVParameter();
        Vmax = Surf.LastVParameter();

        DerSurf.SetValue(1, 0, D1UNormal);
        DerSurf.SetValue(0, 1, D1VNormal);
        DerSurf.SetValue(1, 1, D2UVNormal);
        DerSurf.SetValue(2, 0, D2UNormal);
        DerSurf.SetValue(0, 2, D2VNormal);
        DerSurf.SetValue(3, 0, d3u);
        DerSurf.SetValue(2, 1, d3uuv);
        DerSurf.SetValue(1, 2, d3uvv);
        DerSurf.SetValue(0, 3, d3v);
        for (i = 0; i <= MaxOrder + 2; i++)
        {
            for (j = i; j <= MaxOrder + 3; j++)
            {
                if (i + j > 3)
                {
                    DerSurf.SetValue(i, j, Surf.DN(U, V, i, j));
                    if (i != j)
                    {
                        DerSurf.SetValue(j, i, Surf.DN(U, V, j, i));
                    }
                }
            }
        }

        for (i = 0; i <= MaxOrder + 2; i++)
        {
            for (j = 0; j <= MaxOrder + 2; j++)
            {
                DerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurf));
            }
        }

        CSLib.Normal(MaxOrder, DerNUV, MagTol, U, V, Umin, Umax, Vmin, Vmax, out NStatus, ref Normal, out int OrderU,
                     out int OrderV);
        if (NStatus != CSLib_Defined)
        {
            throw new Geom_UndefinedValue();
        }

        D1UNormal = CSLib.DNNormal(1, 0, DerNUV, OrderU, OrderV);
        D1VNormal = CSLib.DNNormal(0, 1, DerNUV, OrderU, OrderV);
        D2UNormal = CSLib.DNNormal(2, 0, DerNUV, OrderU, OrderV);
        D2VNormal = CSLib.DNNormal(0, 2, DerNUV, OrderU, OrderV);
        D2UVNormal = CSLib.DNNormal(1, 1, DerNUV, OrderU, OrderV);
    }

    public GeomFill_Darboux()
    {
    }

    public override GeomFill_TrihedronLaw Copy()
    {
        GeomFill_Darboux copy = new GeomFill_Darboux();
        if (myCurve is not null)
        {
            copy.SetCurve(myCurve);
        }
        return copy;
    }

    public override bool D0(double Param, ref gp_Vec Tangent, ref gp_Vec Normal, ref gp_Vec BiNormal)
    {
        Adaptor2d_Curve2d myCurve2d = ((Adaptor3d_CurveOnSurface)myTrimmed!).GetCurve()!;
        Adaptor3d_Surface mySupport = ((Adaptor3d_CurveOnSurface)myTrimmed).GetSurface()!;

        myCurve2d.D1(Param, out gp_Pnt2d C2d, out gp_Vec2d D2d);

        gp_Dir NormalDir = new gp_Dir();
        NormalD0(C2d.X(), C2d.Y(), mySupport, ref NormalDir, out _, out _);
        BiNormal.SetXYZ(NormalDir.XYZ());

        mySupport.D1(C2d.X(), C2d.Y(), out _, out gp_Vec dS_du, out gp_Vec dS_dv);

        Tangent = D2d.X() * dS_du + D2d.Y() * dS_dv;
        Tangent.Normalize();

        Normal = BiNormal;
        Normal ^= Tangent;

        return true;
    }

    public override bool D1(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec Normal, ref gp_Vec DNormal,
                            ref gp_Vec BiNormal, ref gp_Vec DBiNormal)
    {
        gp_Vec F, DF;
        Adaptor2d_Curve2d myCurve2d = ((Adaptor3d_CurveOnSurface)myTrimmed!).GetCurve()!;
        Adaptor3d_Surface mySupport = ((Adaptor3d_CurveOnSurface)myTrimmed).GetSurface()!;

        myCurve2d.D2(Param, out gp_Pnt2d C2d, out gp_Vec2d D2d, out gp_Vec2d D2_2d);
        mySupport.D2(C2d.X(), C2d.Y(), out _, out gp_Vec dS_du, out gp_Vec dS_dv, out gp_Vec d2S_du, out gp_Vec d2S_dv,
                     out gp_Vec d2S_duv);

        F = D2d.X() * dS_du + D2d.Y() * dS_dv;
        Tangent = F.Normalized();
        DF = D2_2d.X() * dS_du + D2_2d.Y() * dS_dv + d2S_du * D2d.X() * D2d.X()
             + 2 * d2S_duv * D2d.X() * D2d.Y() + d2S_dv * D2d.Y() * D2d.Y();
        DTangent = FDeriv(F, DF);

        gp_Dir NormalDir = new gp_Dir();
        NormalD1(C2d.X(), C2d.Y(), mySupport, ref NormalDir, out gp_Vec D1UNormal, out gp_Vec D1VNormal);
        BiNormal.SetXYZ(NormalDir.XYZ());
        DBiNormal = D1UNormal * D2d.X() + D1VNormal * D2d.Y();

        Normal = BiNormal;
        Normal ^= Tangent;
        DNormal = BiNormal.Crossed(DTangent) + DBiNormal.Crossed(Tangent);

        return true;
    }

    public override bool D2(double Param, ref gp_Vec Tangent, ref gp_Vec DTangent, ref gp_Vec D2Tangent, ref gp_Vec Normal,
                            ref gp_Vec DNormal, ref gp_Vec D2Normal, ref gp_Vec BiNormal, ref gp_Vec DBiNormal,
                            ref gp_Vec D2BiNormal)
    {
        gp_Vec F, DF, D2F;
        Adaptor2d_Curve2d myCurve2d = ((Adaptor3d_CurveOnSurface)myTrimmed!).GetCurve()!;
        Adaptor3d_Surface mySupport = ((Adaptor3d_CurveOnSurface)myTrimmed).GetSurface()!;

        myCurve2d.D3(Param, out gp_Pnt2d C2d, out gp_Vec2d D2d, out gp_Vec2d D2_2d, out gp_Vec2d D3_2d);
        mySupport.D3(C2d.X(), C2d.Y(), out _, out gp_Vec dS_du, out gp_Vec dS_dv, out gp_Vec d2S_du, out gp_Vec d2S_dv,
                     out gp_Vec d2S_duv, out gp_Vec d3S_du, out gp_Vec d3S_dv, out gp_Vec d3S_duuv, out gp_Vec d3S_duvv);

        F = D2d.X() * dS_du + D2d.Y() * dS_dv;
        Tangent = F.Normalized();
        DF = D2_2d.X() * dS_du + D2_2d.Y() * dS_dv + d2S_du * D2d.X() * D2d.X()
             + 2 * d2S_duv * D2d.X() * D2d.Y() + d2S_dv * D2d.Y() * D2d.Y();
        D2F = D3_2d.X() * dS_du + D3_2d.Y() * dS_dv + D2_2d.X() * (D2d.X() * d2S_du + D2d.Y() * d2S_duv)
              + D2_2d.Y() * (D2d.X() * d2S_duv + D2d.Y() * d2S_dv)
              + D2d.X() * D2d.X() * (D2d.X() * d3S_du + D2d.Y() * d3S_duuv)
              + D2d.Y() * D2d.Y() * (D2d.X() * d3S_duvv + D2d.Y() * d3S_dv)
              + 2 * (D2d.X() * D2_2d.X() * d2S_du + D2d.Y() * D2_2d.Y() * d2S_dv
                     + D2d.X() * D2d.Y() * (D2d.X() * d3S_duuv + D2d.Y() * d3S_duvv)
                     + d2S_duv * (D2_2d.X() * D2d.Y() + D2d.X() * D2_2d.Y()));
        DTangent = FDeriv(F, DF);
        D2Tangent = DDeriv(F, DF, D2F);

        gp_Dir NormalDir = new gp_Dir();
        NormalD2(C2d.X(), C2d.Y(), mySupport, ref NormalDir, out gp_Vec D1UNormal, out gp_Vec D1VNormal,
                 out gp_Vec D2UNormal, out gp_Vec D2VNormal, out gp_Vec D2UVNormal);
        BiNormal.SetXYZ(NormalDir.XYZ());
        DBiNormal = D1UNormal * D2d.X() + D1VNormal * D2d.Y();
        D2BiNormal = D1UNormal * D2_2d.X() + D1VNormal * D2_2d.Y() + D2UNormal * D2d.X() * D2d.X()
                     + 2 * D2UVNormal * D2d.X() * D2d.Y() + D2VNormal * D2d.Y() * D2d.Y();

        Normal = BiNormal;
        Normal ^= Tangent;
        DNormal = BiNormal.Crossed(DTangent) + DBiNormal.Crossed(Tangent);
        D2Normal = BiNormal.Crossed(D2Tangent) + 2 * DBiNormal.Crossed(DTangent) + D2BiNormal.Crossed(Tangent);

        return true;
    }

    public override int NbIntervals(GeomAbs_Shape S) => myCurve!.NbIntervals(S);

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        myCurve!.Intervals(T, S);
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
        ATangent /= Num + 1;
        ANormal /= Num + 1;

        ATangent.Normalize();
        ABiNormal = ATangent.Crossed(ANormal).Normalized();
        ANormal = ABiNormal.Crossed(ATangent);
    }

    public override bool IsConstant() => myCurve!.GetType() == GeomAbs_CurveType.GeomAbs_Line;

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve() => false;
}
