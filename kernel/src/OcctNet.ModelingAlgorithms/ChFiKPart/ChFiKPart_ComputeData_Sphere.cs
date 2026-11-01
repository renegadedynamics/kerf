// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
                                          TopAbs_Orientation Or2, double Rad, in gp_Pnt2d PS1, in gp_Pnt2d P1S2,
                                          in gp_Pnt2d P2S2)
    {
        // Construction of the sphere :
        // - pole south on PS1
        // - origine of u given by P1S2
        // - u+ to P2S2
        double ptol = Precision.Confusion();
        S1.D1(PS1.X(), PS1.Y(), out gp_Pnt p1, out gp_Vec v1, out gp_Vec v2);
        gp_Dir ds1 = new(v1.Crossed(v2));
        gp_Dir df1 = ds1;
        gp_Dir dnat1 = ds1;
        if (Or1 == TopAbs_REVERSED)
        {
            ds1.Reverse();
        }
        if (OrFace1 == TopAbs_REVERSED)
        {
            df1.Reverse();
        }
        S2.D0(P1S2.X(), P1S2.Y(), out gp_Pnt p2);
        S2.D0(P2S2.X(), P2S2.Y(), out gp_Pnt p3);
        gp_Circ ci = new gce_MakeCirc(p1, p2, p3).Value();
        gp_Dir di = ci.Axis().Direction();
        gp_Pnt pp = ci.Location();
        double rr = ci.Radius();
        double delta = Math.Sqrt(Rad * Rad - rr * rr);
        gp_Pnt cen = new(pp.X() + delta * di.X(), pp.Y() + delta * di.Y(), pp.Z() + delta * di.Z());
        gp_Dir dz = new(new gp_Vec(p1, cen));
        if (Math.Abs(ds1.Dot(dz) - 1.0) > ptol)
        {
            cen.SetCoord(pp.X() - delta * di.X(), pp.Y() - delta * di.Y(), pp.Z() - delta * di.Z());
            dz = new gp_Dir(new gp_Vec(p1, cen));
            if (Math.Abs(ds1.Dot(dz) - 1.0) > ptol)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        gp_Dir ddx = new(new gp_Vec(cen, p2));
        gp_Dir dddx = ddx;
        gp_Dir ddy = new(new gp_Vec(cen, p3));
        gp_Dir dx = dz.Crossed(ddx.Crossed(dz));
        gp_Dir ddz = dz.Reversed();
        gp_Ax3 FilAx3 = new(cen, dz, dx);
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
        ElSLib.SphereParameters(FilAx3, Rad, p2, out uu1, out vv1);
        uu1 = 0.0;
        ElSLib.SphereParameters(FilAx3, Rad, p3, out uu2, out vv2);
        // FaceInterferences are loaded with pcurves and curves 3d.
        // Pointed side.
        Geom_Curve? C = null;
        Geom2d_Curve? C2d = null;
        gp_Pnt2d p2dFil = new(0.0, -Math.PI / 2.0);
        gp_Lin2d lin2dFil = new(p2dFil, gp.DX2d());
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
        gp_Dir dci = ddx.Crossed(ddy);
        gp_Ax2 axci = new(cen, dci, ddx);
        gp_Circ ci2 = new(axci, Rad);
        C = new Geom_Circle(ci2);
        GeomAdaptor_Surface AS = new(gsph);
        GeomAdaptor_Curve AC = new(C, 0.0, ang);
        ChFiKPart_ProjPC(AC, AS, out C2dFil);
        gp_Pnt2d p2dbid = C2dFil!.Value(0.0);
        gp_Pnt2d pp2dbid = new(uu1, vv1);
        if (!pp2dbid.IsEqual(p2dbid, ptol))
        {
            gp_Vec2d v2dbid = new(p2dbid, pp2dbid);
            C2dFil.Translate(v2dbid);
        }
        gp_Vec2d v2d = new(P1S2, P2S2);
        gp_Dir2d d2d = new(v2d);
        if (Math.Abs(v2d.Magnitude() - ang) <= ptol)
        {
            gp_Lin2d l2d = new(P1S2, d2d);
            C2d = new Geom2d_Line(l2d);
        }
        else
        {
            C2d = ChFiKPart_PCurve(P1S2, P2S2, 0.0, ang);
        }
        S2.D1(P1S2.X(), P1S2.Y(), out gp_Pnt pp1, out v1, out v2);
        gp_Dir ds2 = new(v1.Crossed(v2));
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
