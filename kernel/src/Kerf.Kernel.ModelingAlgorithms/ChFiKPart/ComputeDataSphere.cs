// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The spherical corner fillet whose boundaries are not all isoparametric.</summary>
internal static class ComputeDataSphere
{
    //=======================================================================
    // function : ChFiKPart_Sphere
    // purpose  : Construction of a spherical fillet the contours which of
    //           are not all isos, from three tops.
    //=======================================================================

    /// <summary>The sphere of radius <paramref name="rad"/> through the point <paramref name="ps1"/> of
    /// <paramref name="s1"/> (its south pole) and the points <paramref name="p1S2"/> and <paramref name="p2S2"/> of
    /// <paramref name="s2"/>.</summary>
    internal static bool ChFiKPart_Sphere(DsDataStructure dStr, SurfData data, SurfaceAdaptorBase s1,
                                          SurfaceAdaptorBase s2, Orientation orFace1,
                                          Orientation orFace2, Orientation or1,
                                          Orientation or2, double rad, in Point2D ps1, in Point2D p1S2,
                                          in Point2D p2S2)
    {
        // Construction of the sphere :
        // - pole south on PS1
        // - origine of u given by P1S2
        // - u+ to P2S2
        double ptol = global::Kerf.Engine.Foundation.Precision.Confusion();
        s1.D1(ps1.X, ps1.Y, out Point3D p1, out Vector3D v1, out Vector3D v2);
        Direction3D ds1 = new(v1.Crossed(v2));
        Direction3D df1 = ds1;
        Direction3D dnat1 = ds1;
        if (or1 == Reversed)
        {
            ds1 = ds1.Reversed();
        }
        if (orFace1 == Reversed)
        {
            df1 = df1.Reversed();
        }
        s2.D0(p1S2.X, p1S2.Y, out Point3D p2);
        s2.D0(p2S2.X, p2S2.Y, out Point3D p3);
        Circle3D ci = new MakeCirc(p1, p2, p3).Value();
        Direction3D di = ci.Axis.Direction;
        Point3D pp = ci.Location;
        double rr = ci.Radius;
        double delta = Math.Sqrt(rad * rad - rr * rr);
        Point3D cen = new(pp.X + delta * di.X, pp.Y + delta * di.Y, pp.Z + delta * di.Z);
        Direction3D dz = new(new Vector3D(p1, cen));
        if (Math.Abs(ds1.Dot(dz) - 1.0) > ptol)
        {
            cen = cen.WithCoord(pp.X - delta * di.X, pp.Y - delta * di.Y, pp.Z - delta * di.Z);
            dz = new Direction3D(new Vector3D(p1, cen));
            if (Math.Abs(ds1.Dot(dz) - 1.0) > ptol)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        Direction3D ddx = new(new Vector3D(cen, p2));
        Direction3D dddx = ddx;
        Direction3D ddy = new(new Vector3D(cen, p3));
        Direction3D dx = dz.Crossed(ddx.Crossed(dz));
        Direction3D ddz = dz.Reversed();
        CoordinateSystem filAx3 = new(cen, dz, dx);
        if (filAx3.YDirection.Dot(ddy) <= 0.0)
        {
            filAx3 = filAx3.YReversed();
            ddz = ddz.Reversed();
            dddx = dddx.Reversed();
        }
        SphericalSurface gsph = new(filAx3, rad);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gsph, dStr));
        // the normal of the sphere is compared to the normal of the face
        // oriented to determine the final orientation of the fillet.
        bool toreverse = (ddz.Dot(df1) <= 0.0);
        if (toreverse)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }
        // Parameters of p2 and p3 are calculated on the Sphere to have
        // ranges of curves.
        double uu1, vv1, uu2, vv2;
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(filAx3, rad, p2, out uu1, out vv1);
        uu1 = 0.0;
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(filAx3, rad, p3, out uu2, out vv2);
        // FaceInterferences are loaded with pcurves and curves 3d.
        // Pointed side.
        Curve3D? c = null;
        Curve2D? c2d = null;
        Point2D p2dFil = new(0.0, -Math.PI / 2.0);
        Line2D lin2dFil = new(p2dFil, Constants.DX2d());
        Curve2D? c2dFil = new LineCurve2D(lin2dFil);
        toreverse = (ddz.Dot(dnat1) <= 0.0);
        Orientation trans = Reversed;
        if (toreverse)
        {
            trans = Forward;
        }
        data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(c, dStr), trans, c2d, c2dFil);
        // The other side.
        double ang = ddx.AngleTo(ddy);
        Direction3D dci = ddx.Crossed(ddy);
        Frame3D axci = new(cen, dci, ddx);
        Circle3D ci2 = new(axci, rad);
        c = new CircleCurve3D(ci2);
        SurfaceAdaptor AS = new(gsph);
        CurveAdaptor ac = new(c, 0.0, ang);
        ChFiKPart_ProjPC(ac, AS, out c2dFil);
        Point2D p2dbid = c2dFil!.Value(0.0);
        Point2D pp2dbid = new(uu1, vv1);
        if (!pp2dbid.IsEqual(p2dbid, ptol))
        {
            Vector2D v2dbid = new(p2dbid, pp2dbid);
            c2dFil.Translate(v2dbid);
        }
        Vector2D v2d = new(p1S2, p2S2);
        Direction2D d2d = new(v2d);
        if (Math.Abs(v2d.Magnitude - ang) <= ptol)
        {
            Line2D l2d = new(p1S2, d2d);
            c2d = new LineCurve2D(l2d);
        }
        else
        {
            c2d = ChFiKPart_PCurve(p1S2, p2S2, 0.0, ang);
        }
        s2.D1(p1S2.X, p1S2.Y, out Point3D pp1, out v1, out v2);
        Direction3D ds2 = new(v1.Crossed(v2));
        toreverse = (ds2.Dot(dddx) <= 0.0);
        trans = Reversed;
        if (!toreverse)
        {
            trans = Forward;
        }
        data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(c, dStr), trans, c2d, c2dFil);
        data.ChangeVertexFirstOnS1().SetPoint(p1);
        data.ChangeVertexLastOnS1().SetPoint(p1);
        data.ChangeVertexFirstOnS2().SetPoint(p2);
        data.ChangeVertexLastOnS2().SetPoint(p3);
        data.ChangeInterferenceOnS1().SetFirstParameter(0.0);
        data.ChangeInterferenceOnS1().SetLastParameter(uu2);
        data.ChangeInterferenceOnS2().SetFirstParameter(0.0);
        data.ChangeInterferenceOnS2().SetLastParameter(ang);
        return true;
    }
}
