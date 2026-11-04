// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Darboux.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Darboux.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The Darboux trihedron law of a curve on a surface: the tangent, the surface normal (as binormal) and their
/// cross product.
/// </summary>
/// <remarks>The curve must be an <see cref="CurveOnSurface"/> (OCCT casts it unchecked; the port checks).</remarks>
internal class Darboux : TrihedronLaw
{
    /// <summary>Computes (F/|F|)'.</summary>
    private static Vector3D FDeriv(in Vector3D f, in Vector3D df)
    {
        double norma = f.Magnitude;
        Vector3D result = (df - f * (f * df) / (norma * norma)) / norma;
        return result;
    }

    /// <summary>Computes (F/|F|)''.</summary>
    private static Vector3D DDeriv(in Vector3D f, in Vector3D df, in Vector3D d2F)
    {
        double norma = f.Magnitude;
        Vector3D result = (d2F - 2 * df * (f * df) / (norma * norma)) / norma
                        - f * ((df.SquareMagnitude + f * d2F - 3 * (f * df) * (f * df) / (norma * norma))
                               / (norma * norma * norma));
        return result;
    }

    /// <summary>The normal to the surface (OCCT's file-static NormalD0).</summary>
    private static void NormalD0(double u, double v, SurfaceAdaptorBase surf, ref Direction3D normal, out int orderU,
                                 out int orderV)
    {
        Continuity cont = (surf.UContinuity() < surf.VContinuity()) ? (surf.UContinuity()) : (surf.VContinuity());
        orderU = orderV = 0;

        surf.D1(u, v, out _, out Vector3D d1U, out Vector3D d1V);
        double magTol = 0.000000001;

        global::Kerf.Engine.Geometry.CSLib.Normal(d1U, d1V, magTol, out NormalStatus nStatus, ref normal);

        if (nStatus != Defined)
        {
            if (cont == C0 || cont == C1)
            {
                throw new UndefinedValue3DException();
            }
            int maxOrder = 3;
            NCollection_Array2<Vector3D> derNUV = new NCollection_Array2<Vector3D>(0, maxOrder, 0, maxOrder);
            NCollection_Array2<Vector3D> derSurf = new NCollection_Array2<Vector3D>(0, maxOrder + 1, 0, maxOrder + 1);
            int i, j;
            double umin, umax, vmin, vmax;
            umin = surf.FirstUParameter();
            umax = surf.LastUParameter();
            vmin = surf.FirstVParameter();
            vmax = surf.LastVParameter();
            for (i = 1; i <= maxOrder + 1; i++)
            {
                derSurf.SetValue(i, 0, surf.DN(u, v, i, 0));
            }

            for (i = 0; i <= maxOrder + 1; i++)
            {
                for (j = 1; j <= maxOrder + 1; j++)
                {
                    derSurf.SetValue(i, j, surf.DN(u, v, i, j));
                }
            }

            for (i = 0; i <= maxOrder; i++)
            {
                for (j = 0; j <= maxOrder; j++)
                {
                    derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
                }
            }

            global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder, derNUV, magTol, u, v, umin, umax, vmin, vmax, out nStatus, ref normal, out orderU,
                         out orderV);

            if (nStatus != Defined)
            {
                throw new UndefinedValue3DException();
            }
        }
    }

    /// <summary>The normal to the surface and its first derivatives (OCCT's file-static NormalD1).</summary>
    private static void NormalD1(double u, double v, SurfaceAdaptorBase surf, ref Direction3D normal, out Vector3D d1UNormal,
                                 out Vector3D d1VNormal)
    {
        surf.D2(u, v, out _, out d1UNormal, out d1VNormal, out Vector3D d2u, out Vector3D d2v, out Vector3D d2uv);
        double magTol = 0.000000001;
        global::Kerf.Engine.Geometry.CSLib.Normal(d1UNormal, d1VNormal, magTol, out NormalStatus nStatus, ref normal);
        int maxOrder;
        if (nStatus == Defined)
        {
            maxOrder = 0;
        }
        else
        {
            maxOrder = 3;
        }
        NCollection_Array2<Vector3D> derNUV = new NCollection_Array2<Vector3D>(0, maxOrder + 1, 0, maxOrder + 1);
        NCollection_Array2<Vector3D> derSurf = new NCollection_Array2<Vector3D>(0, maxOrder + 2, 0, maxOrder + 2);
        int i, j;
        double umin, umax, vmin, vmax;
        umin = surf.FirstUParameter();
        umax = surf.LastUParameter();
        vmin = surf.FirstVParameter();
        vmax = surf.LastVParameter();

        derSurf.SetValue(1, 0, d1UNormal);
        derSurf.SetValue(0, 1, d1VNormal);
        derSurf.SetValue(1, 1, d2uv);
        derSurf.SetValue(2, 0, d2u);
        derSurf.SetValue(0, 2, d2v);
        for (i = 0; i <= maxOrder + 1; i++)
        {
            for (j = i; j <= maxOrder + 2; j++)
            {
                if (i + j > 2)
                {
                    derSurf.SetValue(i, j, surf.DN(u, v, i, j));
                    if (i != j)
                    {
                        derSurf.SetValue(j, i, surf.DN(u, v, j, i));
                    }
                }
            }
        }

        for (i = 0; i <= maxOrder + 1; i++)
        {
            for (j = 0; j <= maxOrder + 1; j++)
            {
                derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
            }
        }

        global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder, derNUV, magTol, u, v, umin, umax, vmin, vmax, out nStatus, ref normal, out int orderU,
                     out int orderV);
        if (nStatus != Defined)
        {
            throw new UndefinedValue3DException();
        }

        d1UNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV);
        d1VNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV);
    }

    /// <summary>The normal to the surface and its first and second derivatives (OCCT's file-static NormalD2).</summary>
    private static void NormalD2(double u, double v, SurfaceAdaptorBase surf, ref Direction3D normal, out Vector3D d1UNormal,
                                 out Vector3D d1VNormal, out Vector3D d2UNormal, out Vector3D d2VNormal,
                                 out Vector3D d2UVNormal)
    {
        surf.D3(u, v, out _, out d1UNormal, out d1VNormal, out d2UNormal, out d2VNormal, out d2UVNormal, out Vector3D d3u,
                out Vector3D d3v, out Vector3D d3uuv, out Vector3D d3uvv);
        double magTol = 0.000000001;
        global::Kerf.Engine.Geometry.CSLib.Normal(d1UNormal, d1VNormal, magTol, out NormalStatus nStatus, ref normal);
        int maxOrder;
        if (nStatus == Defined)
        {
            maxOrder = 0;
        }
        else
        {
            maxOrder = 3;
        }
        NCollection_Array2<Vector3D> derNUV = new NCollection_Array2<Vector3D>(0, maxOrder + 2, 0, maxOrder + 2);
        NCollection_Array2<Vector3D> derSurf = new NCollection_Array2<Vector3D>(0, maxOrder + 3, 0, maxOrder + 3);
        int i, j;
        double umin, umax, vmin, vmax;
        umin = surf.FirstUParameter();
        umax = surf.LastUParameter();
        vmin = surf.FirstVParameter();
        vmax = surf.LastVParameter();

        derSurf.SetValue(1, 0, d1UNormal);
        derSurf.SetValue(0, 1, d1VNormal);
        derSurf.SetValue(1, 1, d2UVNormal);
        derSurf.SetValue(2, 0, d2UNormal);
        derSurf.SetValue(0, 2, d2VNormal);
        derSurf.SetValue(3, 0, d3u);
        derSurf.SetValue(2, 1, d3uuv);
        derSurf.SetValue(1, 2, d3uvv);
        derSurf.SetValue(0, 3, d3v);
        for (i = 0; i <= maxOrder + 2; i++)
        {
            for (j = i; j <= maxOrder + 3; j++)
            {
                if (i + j > 3)
                {
                    derSurf.SetValue(i, j, surf.DN(u, v, i, j));
                    if (i != j)
                    {
                        derSurf.SetValue(j, i, surf.DN(u, v, j, i));
                    }
                }
            }
        }

        for (i = 0; i <= maxOrder + 2; i++)
        {
            for (j = 0; j <= maxOrder + 2; j++)
            {
                derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
            }
        }

        global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder, derNUV, magTol, u, v, umin, umax, vmin, vmax, out nStatus, ref normal, out int orderU,
                     out int orderV);
        if (nStatus != Defined)
        {
            throw new UndefinedValue3DException();
        }

        d1UNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV);
        d1VNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV);
        d2UNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(2, 0, derNUV, orderU, orderV);
        d2VNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 2, derNUV, orderU, orderV);
        d2UVNormal = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 1, derNUV, orderU, orderV);
    }

    public Darboux()
    {
    }

    public override TrihedronLaw Copy()
    {
        Darboux copy = new Darboux();
        if (_curve is not null)
        {
            copy.SetCurve(_curve);
        }
        return copy;
    }

    public override bool D0(double param, ref Vector3D tangent, ref Vector3D normal, ref Vector3D biNormal)
    {
        Curve2DAdaptorBase myCurve2d = ((CurveOnSurface)_trimmed!).GetCurve()!;
        SurfaceAdaptorBase mySupport = ((CurveOnSurface)_trimmed).GetSurface()!;

        myCurve2d.D1(param, out Point2D c2d, out Vector2D d2d);

        Direction3D normalDir = new Direction3D();
        NormalD0(c2d.X, c2d.Y, mySupport, ref normalDir, out _, out _);
        biNormal = biNormal.WithXYZ(normalDir.XYZ());

        mySupport.D1(c2d.X, c2d.Y, out _, out Vector3D dS_du, out Vector3D dS_dv);

        tangent = d2d.X * dS_du + d2d.Y * dS_dv;
        tangent = tangent.Normalized();

        normal = biNormal;
        normal ^= tangent;

        return true;
    }

    public override bool D1(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D normal, ref Vector3D dNormal,
                            ref Vector3D biNormal, ref Vector3D dBiNormal)
    {
        Vector3D f, df;
        Curve2DAdaptorBase myCurve2d = ((CurveOnSurface)_trimmed!).GetCurve()!;
        SurfaceAdaptorBase mySupport = ((CurveOnSurface)_trimmed).GetSurface()!;

        myCurve2d.D2(param, out Point2D c2d, out Vector2D d2d, out Vector2D D2_2d);
        mySupport.D2(c2d.X, c2d.Y, out _, out Vector3D dS_du, out Vector3D dS_dv, out Vector3D d2S_du, out Vector3D d2S_dv,
                     out Vector3D d2S_duv);

        f = d2d.X * dS_du + d2d.Y * dS_dv;
        tangent = f.Normalized();
        df = D2_2d.X * dS_du + D2_2d.Y * dS_dv + d2S_du * d2d.X * d2d.X
             + 2 * d2S_duv * d2d.X * d2d.Y + d2S_dv * d2d.Y * d2d.Y;
        dTangent = FDeriv(f, df);

        Direction3D normalDir = new Direction3D();
        NormalD1(c2d.X, c2d.Y, mySupport, ref normalDir, out Vector3D d1UNormal, out Vector3D d1VNormal);
        biNormal = biNormal.WithXYZ(normalDir.XYZ());
        dBiNormal = d1UNormal * d2d.X + d1VNormal * d2d.Y;

        normal = biNormal;
        normal ^= tangent;
        dNormal = biNormal.Crossed(dTangent) + dBiNormal.Crossed(tangent);

        return true;
    }

    public override bool D2(double param, ref Vector3D tangent, ref Vector3D dTangent, ref Vector3D d2Tangent, ref Vector3D normal,
                            ref Vector3D dNormal, ref Vector3D d2Normal, ref Vector3D biNormal, ref Vector3D dBiNormal,
                            ref Vector3D d2BiNormal)
    {
        Vector3D f, df, d2F;
        Curve2DAdaptorBase myCurve2d = ((CurveOnSurface)_trimmed!).GetCurve()!;
        SurfaceAdaptorBase mySupport = ((CurveOnSurface)_trimmed).GetSurface()!;

        myCurve2d.D3(param, out Point2D c2d, out Vector2D d2d, out Vector2D D2_2d, out Vector2D D3_2d);
        mySupport.D3(c2d.X, c2d.Y, out _, out Vector3D dS_du, out Vector3D dS_dv, out Vector3D d2S_du, out Vector3D d2S_dv,
                     out Vector3D d2S_duv, out Vector3D d3S_du, out Vector3D d3S_dv, out Vector3D d3S_duuv, out Vector3D d3S_duvv);

        f = d2d.X * dS_du + d2d.Y * dS_dv;
        tangent = f.Normalized();
        df = D2_2d.X * dS_du + D2_2d.Y * dS_dv + d2S_du * d2d.X * d2d.X
             + 2 * d2S_duv * d2d.X * d2d.Y + d2S_dv * d2d.Y * d2d.Y;
        d2F = D3_2d.X * dS_du + D3_2d.Y * dS_dv + D2_2d.X * (d2d.X * d2S_du + d2d.Y * d2S_duv)
              + D2_2d.Y * (d2d.X * d2S_duv + d2d.Y * d2S_dv)
              + d2d.X * d2d.X * (d2d.X * d3S_du + d2d.Y * d3S_duuv)
              + d2d.Y * d2d.Y * (d2d.X * d3S_duvv + d2d.Y * d3S_dv)
              + 2 * (d2d.X * D2_2d.X * d2S_du + d2d.Y * D2_2d.Y * d2S_dv
                     + d2d.X * d2d.Y * (d2d.X * d3S_duuv + d2d.Y * d3S_duvv)
                     + d2S_duv * (D2_2d.X * d2d.Y + d2d.X * D2_2d.Y));
        dTangent = FDeriv(f, df);
        d2Tangent = DDeriv(f, df, d2F);

        Direction3D normalDir = new Direction3D();
        NormalD2(c2d.X, c2d.Y, mySupport, ref normalDir, out Vector3D d1UNormal, out Vector3D d1VNormal,
                 out Vector3D d2UNormal, out Vector3D d2VNormal, out Vector3D d2UVNormal);
        biNormal = biNormal.WithXYZ(normalDir.XYZ());
        dBiNormal = d1UNormal * d2d.X + d1VNormal * d2d.Y;
        d2BiNormal = d1UNormal * D2_2d.X + d1VNormal * D2_2d.Y + d2UNormal * d2d.X * d2d.X
                     + 2 * d2UVNormal * d2d.X * d2d.Y + d2VNormal * d2d.Y * d2d.Y;

        normal = biNormal;
        normal ^= tangent;
        dNormal = biNormal.Crossed(dTangent) + dBiNormal.Crossed(tangent);
        d2Normal = biNormal.Crossed(d2Tangent) + 2 * dBiNormal.Crossed(dTangent) + d2BiNormal.Crossed(tangent);

        return true;
    }

    public override int NbIntervals(Continuity s) => _curve!.NbIntervals(s);

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        _curve!.Intervals(t, s);
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
        aTangent /= num + 1;
        aNormal /= num + 1;

        aTangent = aTangent.Normalized();
        aBiNormal = aTangent.Crossed(aNormal).Normalized();
        aNormal = aBiNormal.Crossed(aTangent);
    }

    public override bool IsConstant => _curve!.GetType() == CurveKind.Line;

    /// <summary>Returns false.</summary>
    public override bool IsOnlyBy3dCurve => false;
}
