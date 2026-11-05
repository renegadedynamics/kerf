// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCon.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The constant radius fillet between a plane and a cone along a circle: a torus or a sphere.</summary>
internal static class ComputeDataFilPlnCon
{
    //=======================================================================
    // function : MakeFillet
    // purpose  : case cone/plane or plane/cone.
    //=======================================================================

    /// <summary>The fillet between a plane and a cone orthogonal to it, along their circle.</summary>
    internal static bool ChFiKPart_MakeFillet(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cone3D con, double fu, double lu, Orientation or1,
                                              Orientation or2, double radius, in Circle3D spine, double first,
                                              Orientation ofpl, bool plandab)
    {
        // calculate the fillet (torus or sphere).
        bool c1sphere = false;
        CoordinateSystem posPl = pln.Position;
        Direction3D dpnat = posPl.XDirection.Crossed(posPl.YDirection);
        Direction3D dp = dpnat;
        Direction3D df = dp;
        if (or1 == Reversed)
        {
            dp = dp.Reversed();
        }
        if (ofpl == Reversed)
        {
            df = df.Reversed();
        }
        Point3D or = con.Location;
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, or, out u, out v);
        Point2D c2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out or);
        Point3D cPln = or;
        or = or.WithCoord(or.X + radius * dp.X, or.Y + radius * dp.Y, or.Z + radius * dp.Z);
        global::Kerf.Engine.Geometry.ElCLib.D1(first, spine, out Point3D ptSp, out Vector3D dSp);
        QuadQuadGeo cInt = new(pln, con, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D pv;
        if (cInt.IsDone)
        {
            // The origin of the fillet is set at the start point on the
            // guideline.
            pv = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(cInt.Circle(1), ptSp), cInt.Circle(1));
        }
        else
        {
            return false;
        }
        Direction3D dx = new(new Vector3D(cPln, pv));
        Direction3D dy = new(dSp);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, pv, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out Point3D ptCon, out Vector3D vu, out Vector3D vv);
        Direction3D dc = new(vu.Crossed(vv));
        if (or2 == Reversed)
        {
            dc = dc.Reversed();
        }
        Direction3D dz = dp;
        Point3D pp = new(pv.X + dc.X, pv.Y + dc.Y, pv.Z + dc.Z);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, pp, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out pp);
        Direction3D ddp = new(new Vector3D(pv, pp));
        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, pv, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out pp, out Vector3D dcu, out Vector3D dcv);
        Direction3D ddc = new(dcv);
        if (ddc.Dot(dp) < 0.0)
        {
            ddc = ddc.Reversed();
        }
        double ang = ddp.AngleTo(ddc);
        double rabio = radius / Math.Tan(ang / 2);
        double maxrad = cPln.DistanceTo(pv);
        double rad;
        bool dedans = dx.Dot(dc) <= 0.0;
        if (dedans)
        {
            if (!plandab)
            {
                dz = dz.Reversed();
            }
            rad = maxrad - rabio;
            if (Math.Abs(rad) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                c1sphere = true;
            }
            else if (rad < 0)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        else
        {
            if (plandab)
            {
                dz = dz.Reversed();
            }
            rad = maxrad + rabio;
        }
        CoordinateSystem filAx3 = new(or, dz, dx);
        if (filAx3.YDirection.Dot(dy) <= 0.0)
        {
            filAx3 = filAx3.YReversed();
        }
        if (c1sphere)
        {
            SphericalSurface gsph = new(filAx3, radius);
            data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gsph, dStr));
        }
        else
        {
            ToroidalSurface gtor = new(filAx3, rad, radius);
            data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gtor, dStr));
        }
        // It is checked if the orientation of the fillet is the same
        // as of the faces.
        Point3D p = new(), PP;
        Vector3D deru, derv;
        p = p.WithCoord(cPln.X + rad * dx.X, cPln.Y + rad * dx.Y, cPln.Z + rad * dx.Z);
        if (c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(filAx3, rad, p, out u, out v);
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, filAx3, rad, out PP, out deru, out derv);
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusParameters(filAx3, rad, radius, p, out u, out v);
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, filAx3, rad, radius, out PP, out deru, out derv);
            if (!plandab && ang < Math.PI / 2 && dedans)
            {
                v = v + 2 * Math.PI;
            }
        }
        Point2D p2dFil = new(0.0, v);
        Direction3D norFil = new(deru.Crossed(derv));
        bool toreverse = (norFil.Dot(df) <= 0.0);
        if (toreverse)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }
        // FaceInterferences are loaded with pcurves and curves 3d.
        // ---------------------------------------------------------------
        // The plane face.
        // --------------
        CircleCurve2D? gCirc2dPln = null;
        CircleCurve3D? gCircPln = null;
        Frame3D circAx2 = filAx3.Frame;
        if (!c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, p, out u, out v);
            Point2D p2dPln = new(u, v);
            Direction2D d2d = new(dSp.Dot(new Vector3D(posPl.XDirection)), dSp.Dot(new Vector3D(posPl.YDirection)));
            Frame2D ax2dPln = new(c2dPln, new Direction2D(new Vector2D(c2dPln, p2dPln)), d2d);
            Circle2D circ2dPln = new(ax2dPln, rad);
            gCirc2dPln = new CircleCurve2D(circ2dPln);
            circAx2 = circAx2.WithLocation(cPln);
            Circle3D circPln = new(circAx2, rad);
            gCircPln = new CircleCurve3D(circPln);
        }
        Line2D lin2dFil = new(p2dFil, Constants.DX2d());
        LineCurve2D gLin2dFil1 = new(lin2dFil);
        toreverse = (norFil.Dot(dpnat) <= 0.0);
        Orientation trans;
        if ((toreverse && plandab) || (!toreverse && !plandab))
        {
            trans = Forward;
        }
        else
        {
            trans = Reversed;
        }
        if (plandab)
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCircPln, dStr), trans,
                                                          gCirc2dPln, gLin2dFil1);
        }
        else
        {
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCircPln, dStr), trans,
                                                          gCirc2dPln, gLin2dFil1);
        }
        // The conic face.
        // ----------------
        p = p.WithCoord(pv.X + rabio * ddc.X, pv.Y + rabio * ddc.Y, pv.Z + rabio * ddc.Z);
        if (c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(filAx3, radius, p, out u, out v);
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, filAx3, radius, out PP, out deru, out derv);
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusParameters(filAx3, rad, radius, p, out u, out v);
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, filAx3, rad, radius, out PP, out deru, out derv);
            if (plandab && ang < Math.PI / 2 && dedans)
            {
                v = v + 2 * Math.PI;
            }
        }
        norFil = new Direction3D(deru.Crossed(derv));
        p2dFil = p2dFil.WithCoord(0.0, v);
        lin2dFil = lin2dFil.WithLocation(p2dFil);
        LineCurve2D gLin2dFil2 = new(lin2dFil);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, p, out u, out v);
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        bool careaboutsens = false;
        if (Math.Abs(lu - fu - 2 * Math.PI) < tol)
        {
            careaboutsens = true;
        }
        if (u >= fu - tol && u < fu)
        {
            u = fu;
        }
        if (u <= lu + tol && u > lu)
        {
            u = lu;
        }
        if (u < fu || u > lu)
        {
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, fu, fu + 2 * Math.PI);
        }
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out PP, out deru, out derv);
        Direction3D norCon = new(deru.Crossed(derv));
        Direction2D d2dCon = Constants.DX2d();
        if (deru.Dot(new Vector3D(dy)) < 0.0)
        {
            d2dCon = d2dCon.Reversed();
            if (careaboutsens && Math.Abs(fu - u) < tol)
            {
                u = lu;
            }
        }
        else if (careaboutsens && Math.Abs(lu - u) < tol)
        {
            u = fu;
        }
        Point2D p2dCon = new(u, v);
        Line2D lin2dCon = new(p2dCon, d2dCon);
        LineCurve2D gLin2dCon = new(lin2dCon);
        double scal = new Vector3D(dp).Dot(new Vector3D(pv, p));
        PP = PP.WithCoord(cPln.X + scal * dp.X, cPln.Y + scal * dp.Y, cPln.Z + scal * dp.Z);
        circAx2 = circAx2.WithLocation(PP);
        Circle3D circCon = new(circAx2, p.DistanceTo(PP));
        CircleCurve3D gCircCon = new(circCon);
        toreverse = (norFil.Dot(norCon) <= 0.0);
        if ((toreverse && plandab) || (!toreverse && !plandab))
        {
            trans = Reversed;
        }
        else
        {
            trans = Forward;
        }
        if (plandab)
        {
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCircCon, dStr), trans,
                                                          gLin2dCon, gLin2dFil2);
        }
        else
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCircCon, dStr), trans,
                                                          gLin2dCon, gLin2dFil2);
        }
        return true;
    }
}
