// Port of: src/ModelingData/TKG3d/Geom/Geom_ExtrusionUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// Evaluation of surfaces of linear extrusion, P(U, V) = BasisCurve(U) + V * Direction, shared by
/// Geom_SurfaceOfLinearExtrusion, GeomAdaptor_Surface and GeomGridEval: direct calculations from pre-computed
/// curve values, and evaluators over a basis curve.
/// </summary>
/// <remarks>
/// OCCT's evaluators are templates over the curve type; OCCT instantiates them only with Adaptor3d_Curve
/// (GeomAdaptor_Surface), so they are ported for that type (rules, section 9).
/// </remarks>
internal static class ExtrusionUtils
{
    /// <summary>The point of the surface from the curve point C(U).</summary>
    internal static void CalculateD0(in Point3D curvePt,
                                     double v,
                                     in Xyz dir,
                                     out Point3D p)
    {
        p = new Point3D();
        p = p.WithXYZ(curvePt.XYZ() + v * dir);
    }

    /// <summary>The point and first derivatives from C(U) and C'(U).</summary>
    internal static void CalculateD1(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     double v,
                                     in Xyz dir,
                                     out Point3D p,
                                     out Vector3D d1U,
                                     out Vector3D d1V)
    {
        p = new Point3D();
        p = p.WithXYZ(curvePt.XYZ() + v * dir);
        d1U = curveD1;
        d1V = new Vector3D();
        d1V = d1V.WithXYZ(dir);
    }

    /// <summary>The point, first and second derivatives from C(U), C'(U) and C''(U).</summary>
    internal static void CalculateD2(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     in Vector3D curveD2,
                                     double v,
                                     in Xyz dir,
                                     out Point3D p,
                                     out Vector3D d1U,
                                     out Vector3D d1V,
                                     out Vector3D d2U,
                                     out Vector3D d2V,
                                     out Vector3D d2UV)
    {
        p = new Point3D();
        p = p.WithXYZ(curvePt.XYZ() + v * dir);
        d1U = curveD1;
        d1V = new Vector3D();
        d1V = d1V.WithXYZ(dir);
        d2U = curveD2;
        d2V = new Vector3D(0.0, 0.0, 0.0);
        d2UV = new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>The point and the derivatives up to the third order from C(U) to C'''(U).</summary>
    internal static void CalculateD3(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     in Vector3D curveD2,
                                     in Vector3D curveD3,
                                     double v,
                                     in Xyz dir,
                                     out Point3D p,
                                     out Vector3D d1U,
                                     out Vector3D d1V,
                                     out Vector3D d2U,
                                     out Vector3D d2V,
                                     out Vector3D d2UV,
                                     out Vector3D d3U,
                                     out Vector3D d3V,
                                     out Vector3D d3UUV,
                                     out Vector3D d3UVV)
    {
        p = new Point3D();
        p = p.WithXYZ(curvePt.XYZ() + v * dir);
        d1U = curveD1;
        d1V = new Vector3D();
        d1V = d1V.WithXYZ(dir);
        d2U = curveD2;
        d2V = new Vector3D(0.0, 0.0, 0.0);
        d2UV = new Vector3D(0.0, 0.0, 0.0);
        d3U = curveD3;
        d3V = new Vector3D(0.0, 0.0, 0.0);
        d3UUV = new Vector3D(0.0, 0.0, 0.0);
        d3UVV = new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>
    /// The derivative of orders (theDerU, theDerV): the curve derivative theCurveDN when theDerV is 0, the
    /// direction for (0, 1), zero otherwise.
    /// </summary>
    internal static Vector3D CalculateDN(in Vector3D curveDN,
                                       in Xyz dir,
                                       int derU,
                                       int derV)
    {
        if (derV == 0)
        {
            return curveDN;
        }
        else if (derU == 0 && derV == 1)
        {
            return new Vector3D(dir);
        }
        return new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>The point of parameters (theU, theV) on the extrusion of theBasis along theDir.</summary>
    internal static bool D0(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Xyz dir,
                            out Point3D p)
    {
        Point3D curveD0 = basis.EvalD0(u);
        CalculateD0(curveD0, v, dir, out p);
        return true;
    }

    /// <summary>The point and first derivatives.</summary>
    internal static bool D1(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Xyz dir,
                            out Point3D p,
                            out Vector3D d1U,
                            out Vector3D d1V)
    {
        Curve3D.ResD1 curveD1 = basis.EvalD1(u);
        CalculateD1(curveD1.Point, curveD1.D1, v, dir, out p, out d1U, out d1V);
        return true;
    }

    /// <summary>The point, first and second derivatives.</summary>
    internal static bool D2(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Xyz dir,
                            out Point3D p,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV)
    {
        Curve3D.ResD2 curveD2 = basis.EvalD2(u);
        CalculateD2(curveD2.Point,
                    curveD2.D1,
                    curveD2.D2,
                    v,
                    dir,
                    out p,
                    out d1U,
                    out d1V,
                    out d2U,
                    out d2V,
                    out d2UV);
        return true;
    }

    /// <summary>The point and the derivatives up to the third order.</summary>
    internal static bool D3(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Xyz dir,
                            out Point3D p,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV,
                            out Vector3D d3U,
                            out Vector3D d3V,
                            out Vector3D d3UUV,
                            out Vector3D d3UVV)
    {
        Curve3D.ResD3 curveD3 = basis.EvalD3(u);
        CalculateD3(curveD3.Point,
                    curveD3.D1,
                    curveD3.D2,
                    curveD3.D3,
                    v,
                    dir,
                    out p,
                    out d1U,
                    out d1V,
                    out d2U,
                    out d2V,
                    out d2UV,
                    out d3U,
                    out d3V,
                    out d3UUV,
                    out d3UVV);
        return true;
    }

    /// <summary>The derivative of orders (theDerU, theDerV); the curve is evaluated only when theDerV is 0.</summary>
    internal static bool DN(double u,
                            CurveAdaptorBase basis,
                            in Xyz dir,
                            int derU,
                            int derV,
                            out Vector3D dn)
    {
        Vector3D curveDN = new Vector3D();
        if (derV == 0)
        {
            curveDN = basis.EvalDN(u, derU);
        }
        dn = CalculateDN(curveDN, dir, derU, derV);
        return true;
    }
}
