// Port of: src/ModelingData/TKG3d/Geom/Geom_RevolutionUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// Evaluation of surfaces of revolution, P(U, V) = Rotation(Axis, U) * BasisCurve(V), shared by
/// Geom_SurfaceOfRevolution, GeomAdaptor_Surface and GeomGridEval: direct calculations from pre-computed curve
/// values, and evaluators over a basis curve.
/// </summary>
/// <remarks>
/// OCCT's evaluators are templates over the curve type; OCCT instantiates them only with Adaptor3d_Curve
/// (GeomAdaptor_Surface), so they are ported for that type (rules, section 9).
/// </remarks>
internal static class RevolutionUtils
{
    /// <summary>The point of the surface from the curve point C(V) and the rotation angle theU.</summary>
    internal static void CalculateD0(in Point3D curvePt,
                                     double u,
                                     in Axis3D axis,
                                     out Point3D p)
    {
        p = curvePt;
        Transform3D rotation = new Transform3D();
        rotation.SetRotation(axis, u);
        p.Transform(rotation);
    }

    /// <summary>The point and first derivatives from C(V) and C'(V).</summary>
    internal static void CalculateD1(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     double u,
                                     in Axis3D axis,
                                     out Point3D p,
                                     out Vector3D d1U,
                                     out Vector3D d1V)
    {
        p = curvePt;

        // Vector from center of rotation to the point on rotated curve
        Xyz cq = curvePt.XYZ() - axis.Location().XYZ();
        d1U = new Vector3D(axis.Direction().XYZ().Crossed(cq));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (d1U.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            d1U.SetCoord(0.0, 0.0, 0.0);
        }

        Transform3D rotation = new Transform3D();
        rotation.SetRotation(axis, u);
        p.Transform(rotation);
        d1U.Transform(rotation);
        d1V = curveD1;
        d1V.Transform(rotation);
    }

    /// <summary>The point, first and second derivatives from C(V), C'(V) and C''(V).</summary>
    internal static void CalculateD2(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     in Vector3D curveD2,
                                     double u,
                                     in Axis3D axis,
                                     out Point3D p,
                                     out Vector3D d1U,
                                     out Vector3D d1V,
                                     out Vector3D d2U,
                                     out Vector3D d2V,
                                     out Vector3D d2UV)
    {
        p = curvePt;

        // Vector from center of rotation to the point on rotated curve
        Xyz cq = curvePt.XYZ() - axis.Location().XYZ();
        Xyz dir = axis.Direction().XYZ();
        d1U = new Vector3D(dir.Crossed(cq));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (d1U.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            d1U.SetCoord(0.0, 0.0, 0.0);
        }

        d2U = new Vector3D(dir.Dot(cq) * dir - cq);
        d2UV = new Vector3D(dir.Crossed(curveD1.XYZ()));

        Transform3D rotation = new Transform3D();
        rotation.SetRotation(axis, u);
        p.Transform(rotation);
        d1U.Transform(rotation);
        d1V = curveD1;
        d1V.Transform(rotation);
        d2U.Transform(rotation);
        d2V = curveD2;
        d2V.Transform(rotation);
        d2UV.Transform(rotation);
    }

    /// <summary>The point and the derivatives up to the third order from C(V) to C'''(V).</summary>
    internal static void CalculateD3(in Point3D curvePt,
                                     in Vector3D curveD1,
                                     in Vector3D curveD2,
                                     in Vector3D curveD3,
                                     double u,
                                     in Axis3D axis,
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
        p = curvePt;

        // Vector from center of rotation to the point on rotated curve
        Xyz cq = curvePt.XYZ() - axis.Location().XYZ();
        Xyz dir = axis.Direction().XYZ();
        d1U = new Vector3D(dir.Crossed(cq));

        // If the point is placed on the axis of revolution then derivatives on U are undefined.
        // Manually set them to zero.
        if (d1U.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            d1U.SetCoord(0.0, 0.0, 0.0);
        }

        d2U = new Vector3D(dir.Dot(cq) * dir - cq);
        d2UV = new Vector3D(dir.Crossed(curveD1.XYZ()));
        d3U = -d1U;
        d3UUV = new Vector3D(dir.Dot(curveD1.XYZ()) * dir - curveD1.XYZ());
        d3UVV = new Vector3D(dir.Crossed(curveD2.XYZ()));

        Transform3D rotation = new Transform3D();
        rotation.SetRotation(axis, u);
        p.Transform(rotation);
        d1U.Transform(rotation);
        d1V = curveD1;
        d1V.Transform(rotation);
        d2U.Transform(rotation);
        d2V = curveD2;
        d2V.Transform(rotation);
        d2UV.Transform(rotation);
        d3U.Transform(rotation);
        d3V = curveD3;
        d3V.Transform(rotation);
        d3UUV.Transform(rotation);
        d3UVV.Transform(rotation);
    }

    /// <summary>
    /// The derivative of orders (theDerU, theDerV). theCurvePtOrDN is C(V) - axis location when theDerV is 0,
    /// and the curve derivative of order theDerV otherwise.
    /// </summary>
    internal static Vector3D CalculateDN(in Vector3D curvePtOrDN,
                                       double u,
                                       in Axis3D axis,
                                       int derU,
                                       int derV)
    {
        // theDerV is part of the interface contract - caller provides different data based on its value
        _ = derV;

        Transform3D rotation = new Transform3D();
        rotation.SetRotation(axis, u);

        Vector3D result;
        if (derU == 0)
        {
            // Pure V derivative: just rotate the curve derivative
            result = curvePtOrDN;
        }
        else
        {
            // For theDerV == 0: theCurvePtOrDN is (P - AxisLocation) as a vector
            // For theDerV > 0: theCurvePtOrDN is the curve derivative
            Xyz dir = axis.Direction().XYZ();
            if (derU % 4 == 1)
            {
                result = new Vector3D(dir.Crossed(curvePtOrDN.XYZ()));
            }
            else if (derU % 4 == 2)
            {
                result = new Vector3D(dir.Dot(curvePtOrDN.XYZ()) * dir - curvePtOrDN.XYZ());
            }
            else if (derU % 4 == 3)
            {
                result = new Vector3D(dir.Crossed(curvePtOrDN.XYZ())) * (-1.0);
            }
            else
            {
                result = new Vector3D(curvePtOrDN.XYZ() - dir.Dot(curvePtOrDN.XYZ()) * dir);
            }
        }

        result.Transform(rotation);
        return result;
    }

    /// <summary>The point of parameters (theU, theV) on the revolution of theBasis about theAxis.</summary>
    internal static bool D0(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Axis3D axis,
                            out Point3D p)
    {
        Point3D curveD0 = basis.EvalD0(v);
        CalculateD0(curveD0, u, axis, out p);
        return true;
    }

    /// <summary>The point and first derivatives.</summary>
    internal static bool D1(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Axis3D axis,
                            out Point3D p,
                            out Vector3D d1U,
                            out Vector3D d1V)
    {
        Curve3D.ResD1 curveD1 = basis.EvalD1(v);
        CalculateD1(curveD1.Point, curveD1.D1, u, axis, out p, out d1U, out d1V);
        return true;
    }

    /// <summary>The point, first and second derivatives.</summary>
    internal static bool D2(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Axis3D axis,
                            out Point3D p,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV)
    {
        Curve3D.ResD2 curveD2 = basis.EvalD2(v);
        CalculateD2(curveD2.Point,
                    curveD2.D1,
                    curveD2.D2,
                    u,
                    axis,
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
                            in Axis3D axis,
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
        Curve3D.ResD3 curveD3 = basis.EvalD3(v);
        CalculateD3(curveD3.Point,
                    curveD3.D1,
                    curveD3.D2,
                    curveD3.D3,
                    u,
                    axis,
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

    /// <summary>The derivative of orders (theDerU, theDerV).</summary>
    internal static bool DN(double u,
                            double v,
                            CurveAdaptorBase basis,
                            in Axis3D axis,
                            int derU,
                            int derV,
                            out Vector3D dn)
    {
        Vector3D curvePtOrDN;
        if (derU == 0)
        {
            curvePtOrDN = basis.EvalDN(v, derV);
        }
        else if (derV == 0)
        {
            Point3D p = basis.EvalD0(v);
            curvePtOrDN = new Vector3D(p.XYZ() - axis.Location().XYZ());
        }
        else
        {
            curvePtOrDN = basis.EvalDN(v, derV);
        }
        dn = CalculateDN(curvePtOrDN, u, axis, derU, derV);
        return true;
    }
}
