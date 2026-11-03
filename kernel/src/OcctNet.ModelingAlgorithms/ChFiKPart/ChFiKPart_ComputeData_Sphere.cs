// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The spherical corner fillet whose boundaries are not all isoparametric.</summary>
internal static class ChFiKPart_ComputeData_Sphere
{
    //=======================================================================
    // function : ChFiKPart_Sphere
    // purpose  : Construction of a spherical fillet the contours which of
    //           are not all isos, from three tops.
    //=======================================================================

    /// <summary>The sphere of radius <paramref name="Rad"/> through the point <paramref name="PS1"/> of
    /// <paramref name="S1"/> (its south pole) and the points <paramref name="P1S2"/> and <paramref name="P2S2"/> of
    /// <paramref name="S2"/>.</summary>
    internal static bool ChFiKPart_Sphere(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, Adaptor3d_Surface S1,
                                          Adaptor3d_Surface S2, TopAbs_Orientation OrFace1,
                                          TopAbs_Orientation OrFace2, TopAbs_Orientation Or1,
                                          TopAbs_Orientation Or2, double Rad, in Point2D PS1, in Point2D P1S2,
                                          in Point2D P2S2)
    {
        // Construction of the sphere :
        // - pole south on PS1
        // - origine of u given by P1S2
        // - u+ to P2S2
        double ptol = global::Kerf.Engine.Foundation.Precision.Confusion();
        S1.D1(PS1.X(), PS1.Y(), out Point3D p1, out Vector3D v1, out Vector3D v2);
        Direction3D ds1 = new(v1.Crossed(v2));
        Direction3D df1 = ds1;
        Direction3D dnat1 = ds1;
        if (Or1 == TopAbs_REVERSED)
        {
            ds1.Reverse();
        }
        if (OrFace1 == TopAbs_REVERSED)
        {
            df1.Reverse();
        }
        S2.D0(P1S2.X(), P1S2.Y(), out Point3D p2);
        S2.D0(P2S2.X(), P2S2.Y(), out Point3D p3);
        Circle3D ci = new gce_MakeCirc(p1, p2, p3).Value();
        Direction3D di = ci.Axis().Direction();
        Point3D pp = ci.Location();
        double rr = ci.Radius();
        double delta = Math.Sqrt(Rad * Rad - rr * rr);
        Point3D cen = new(pp.X() + delta * di.X(), pp.Y() + delta * di.Y(), pp.Z() + delta * di.Z());
        Direction3D dz = new(new Vector3D(p1, cen));
        if (Math.Abs(ds1.Dot(dz) - 1.0) > ptol)
        {
            cen.SetCoord(pp.X() - delta * di.X(), pp.Y() - delta * di.Y(), pp.Z() - delta * di.Z());
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
        CoordinateSystem FilAx3 = new(cen, dz, dx);
        if (FilAx3.YDirection().Dot(ddy) <= 0.0)
        {
            FilAx3.YReverse();
            ddz.Reverse();
            dddx.Reverse();
        }
        Geom_SphericalSurface gsph = new(FilAx3, Rad);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gsph, DStr));
        // the normal of the sphere is compared to the normal of the face
        // oriented to determine the final orientation of the fillet.
        bool toreverse = (ddz.Dot(df1) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }
        // Parameters of p2 and p3 are calculated on the Sphere to have
        // ranges of curves.
        double uu1, vv1, uu2, vv2;
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(FilAx3, Rad, p2, out uu1, out vv1);
        uu1 = 0.0;
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(FilAx3, Rad, p3, out uu2, out vv2);
        // FaceInterferences are loaded with pcurves and curves 3d.
        // Pointed side.
        Geom_Curve? C = null;
        Geom2d_Curve? C2d = null;
        Point2D p2dFil = new(0.0, -Math.PI / 2.0);
        Line2D lin2dFil = new(p2dFil, Constants.DX2d());
        Geom2d_Curve? C2dFil = new Geom2d_Line(lin2dFil);
        toreverse = (ddz.Dot(dnat1) <= 0.0);
        TopAbs_Orientation trans = TopAbs_REVERSED;
        if (toreverse)
        {
            trans = TopAbs_FORWARD;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(C, DStr), trans, C2d, C2dFil);
        // The other side.
        double ang = ddx.Angle(ddy);
        Direction3D dci = ddx.Crossed(ddy);
        Frame3D axci = new(cen, dci, ddx);
        Circle3D ci2 = new(axci, Rad);
        C = new Geom_Circle(ci2);
        GeomAdaptor_Surface AS = new(gsph);
        GeomAdaptor_Curve AC = new(C, 0.0, ang);
        ChFiKPart_ProjPC(AC, AS, out C2dFil);
        Point2D p2dbid = C2dFil!.Value(0.0);
        Point2D pp2dbid = new(uu1, vv1);
        if (!pp2dbid.IsEqual(p2dbid, ptol))
        {
            Vector2D v2dbid = new(p2dbid, pp2dbid);
            C2dFil.Translate(v2dbid);
        }
        Vector2D v2d = new(P1S2, P2S2);
        Direction2D d2d = new(v2d);
        if (Math.Abs(v2d.Magnitude() - ang) <= ptol)
        {
            Line2D l2d = new(P1S2, d2d);
            C2d = new Geom2d_Line(l2d);
        }
        else
        {
            C2d = ChFiKPart_PCurve(P1S2, P2S2, 0.0, ang);
        }
        S2.D1(P1S2.X(), P1S2.Y(), out Point3D pp1, out v1, out v2);
        Direction3D ds2 = new(v1.Crossed(v2));
        toreverse = (ds2.Dot(dddx) <= 0.0);
        trans = TopAbs_REVERSED;
        if (!toreverse)
        {
            trans = TopAbs_FORWARD;
        }
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(C, DStr), trans, C2d, C2dFil);
        Data.ChangeVertexFirstOnS1().SetPoint(p1);
        Data.ChangeVertexLastOnS1().SetPoint(p1);
        Data.ChangeVertexFirstOnS2().SetPoint(p2);
        Data.ChangeVertexLastOnS2().SetPoint(p3);
        Data.ChangeInterferenceOnS1().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS1().SetLastParameter(uu2);
        Data.ChangeInterferenceOnS2().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS2().SetLastParameter(ang);
        return true;
    }
}
