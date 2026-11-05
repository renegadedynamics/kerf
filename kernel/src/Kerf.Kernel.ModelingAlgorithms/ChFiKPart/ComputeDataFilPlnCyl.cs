// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.cxx
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

/// <summary>The constant radius fillet between a plane and a cylinder: a cylinder along a generatrix, a torus (or a
/// sphere) along a circle.</summary>
internal static class ComputeDataFilPlnCyl
{
    //=======================================================================
    // function : MakeFillet
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The fillet along a generatrix of the cylinder (a straight guide).</summary>
    internal static bool ChFiKPart_MakeFillet(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cylinder3D cyl, double fu, double lu, Orientation or1,
                                              Orientation or2, double radius, in Line3D spine, double first,
                                              Orientation ofpl, bool plandab)
    {
        // calculate the cylinder fillet.
        // plane deviated from radius
        CoordinateSystem axPln = pln.Position;
        Direction3D norPln = axPln.XDirection.Crossed(axPln.YDirection);
        Direction3D norF = norPln;
        CoordinateSystem axCyl = cyl.Position;
        if (or1 == Reversed)
        {
            norF = norF.Reversed();
        }
        Plane3D planOffset = pln.Translated(radius * new Vector3D(norF));
        // Parallel cylinder
        double rOff = cyl.Radius;
        if ((or2 == Forward && cyl.IsDirect) || (or2 == Reversed && !cyl.IsDirect))
        {
            rOff += radius;
        }
        else if (radius < rOff)
        {
            rOff -= radius;
        }
        else
        {
            // (OCCT_DEBUG trace left out)
            return false;
        }
        // intersection of the parallel plane and of the parallel cylinder.
        Cylinder3D cylOffset = new(cyl.Position, rOff);
        QuadQuadGeo lInt = new(planOffset, cylOffset, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D orSpine = global::Kerf.Engine.Geometry.ElCLib.Value(first, spine);
        Point3D orFillet;
        Direction3D dirFillet;
        if (lInt.IsDone)
        {
            dirFillet = lInt.Line(1).Direction;
            Point3D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(lInt.Line(1), orSpine), lInt.Line(1));
            if (lInt.SolutionCount == 2)
            {
                Point3D p2 = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(lInt.Line(2), orSpine), lInt.Line(2));
                if (p1.SquareDistanceTo(orSpine) < p2.SquareDistanceTo(orSpine))
                {
                    orFillet = p1;
                }
                else
                {
                    orFillet = p2;
                }
            }
            else
            {
                orFillet = p1;
            }
        }
        else
        {
            return false;
        }
        // Construction fillet
        if (dirFillet.Dot(spine.Direction) < 0.0)
        {
            dirFillet = dirFillet.Reversed();
        }
        double uOnCyl, vOnCyl, uOnPln, vOnPln;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, orFillet, out uOnCyl, out vOnCyl);
        double tesp = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (uOnCyl < fu - tesp || uOnCyl > lu + tesp)
        {
            uOnCyl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uOnCyl, fu, fu + 2 * Math.PI);
        }
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, orFillet, out uOnPln, out vOnPln);
        Vector3D xDir, otherDir;
        if (plandab)
        {
            xDir = new Vector3D(norF.Reversed());
            otherDir = new Vector3D(orFillet, global::Kerf.Engine.Geometry.ElSLib.Value(uOnCyl, vOnCyl, cyl));
            otherDir = otherDir.Normalized();
        }
        else
        {
            otherDir = new Vector3D(norF.Reversed());
            xDir = new Vector3D(orFillet, global::Kerf.Engine.Geometry.ElSLib.Value(uOnCyl, vOnCyl, cyl));
            xDir = xDir.Normalized();
        }
        CoordinateSystem axFil = new(orFillet, dirFillet, new Direction3D(xDir));
        Vector3D prod = xDir.Crossed(otherDir);
        if (prod.Dot(new Vector3D(dirFillet)) < 0.0)
        {
            axFil = axFil.YReversed();
        }
        CylindricalSurface fillet = new(axFil, radius);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(fillet, dStr));
        // FaceInterferences are loaded with pcurves and curves 3D.
        // edge plane-Fillet
        Point2D pPln2d = new(uOnPln, vOnPln);
        Direction2D vPln2d = new(dirFillet.Dot(axPln.XDirection), dirFillet.Dot(axPln.YDirection));
        Line2D lin2dPln = new(pPln2d, vPln2d);
        Point3D pOnPln = global::Kerf.Engine.Geometry.ElSLib.Value(uOnPln, vOnPln, pln);
        Line3D c3d = new(pOnPln, dirFillet);
        double uOnFillet, v;
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(axFil, radius, pOnPln, out uOnFillet, out v);
        if (uOnFillet > Math.PI)
        {
            uOnFillet = 0.0;
        }
        Line2D lOnFillet = new(new Point2D(uOnFillet, v), Constants.DY2d());
        LineCurve3D l3d = new(c3d);
        LineCurve2D lFac = new(lin2dPln);
        LineCurve2D lFil = new(lOnFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(uOnFillet, v, axFil, radius, out Point3D p, out Vector3D deru, out Vector3D derv);
        Direction3D norFil = new(deru.Crossed(derv));
        bool toreverse = (norFil.Dot(norPln) <= 0.0);
        // It is checked if the orientation of the cylinder is the same as of the plane.
        if (toreverse)
        {
            data.ChangeOrientation() = global::Kerf.Engine.Topology.TopAbs.Reverse(ofpl);
        }
        else
        {
            data.ChangeOrientation() = ofpl;
        }
        Orientation trans;
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
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        else
        {
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        // edge cylinder-Fillet.
        Point2D pCyl2d = new(uOnCyl, vOnCyl);
        Direction2D dpc = Constants.DY2d();
        if (dirFillet.Dot(axCyl.Direction) < 0.0)
        {
            dpc = dpc.Reversed();
        }
        Line2D lin2dCyl = new(pCyl2d, dpc);
        Point3D pOnCyl = global::Kerf.Engine.Geometry.ElSLib.Value(uOnCyl, vOnCyl, cyl);
        c3d = new Line3D(pOnCyl, dirFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(axFil, radius, pOnCyl, out uOnFillet, out v);
        if (uOnFillet > Math.PI)
        {
            uOnFillet = 0.0;
        }
        lOnFillet = new Line2D(new Point2D(uOnFillet, v), Constants.DY2d());
        l3d = new LineCurve3D(c3d);
        lFac = new LineCurve2D(lin2dCyl);
        lFil = new LineCurve2D(lOnFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(uOnFillet, v, axFil, radius, out p, out deru, out derv);
        norFil = new Direction3D(deru.Crossed(derv));
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(uOnCyl, vOnCyl, axCyl, cyl.Radius, out p, out deru, out derv);
        Direction3D norCyl = new(deru.Crossed(derv));
        toreverse = (norFil.Dot(norCyl) <= 0.0);
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
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        else
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        return true;
    }

    //=======================================================================
    // function : MakeFillet
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The fillet along a circle of the cylinder: a torus, or a sphere when the radii are equal.</summary>
    internal static bool ChFiKPart_MakeFillet(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cylinder3D cyl, double fu, double lu, Orientation or1,
                                              Orientation or2, double radius, in Circle3D spine, double first,
                                              Orientation ofpl, bool plandab)
    {
        // calculation of the fillet (torus or sphere).
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
        Point3D or = cyl.Location;
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, or, out u, out v);
        Point2D c2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out or);
        Point3D cPln = or;
        or = or.WithCoord(or.X + radius * dp.X, or.Y + radius * dp.Y, or.Z + radius * dp.Z);
        Point3D ptSp;
        Vector3D dSp;
        // Modification for the PtSp found at the wrong side of the sewing edge.
        Point3D ptSp2;
        Vector3D dSp2;
        double acote = 1e-7;
        global::Kerf.Engine.Geometry.ElCLib.D1(first, spine, out ptSp, out dSp);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, ptSp, out u, out v);
        if ((Math.Abs(u) < acote) || (Math.Abs(u - (2 * Math.PI)) < acote))
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(first + 0.2, spine, out ptSp2, out dSp2);
            double u2, v2;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, ptSp2, out u2, out v2);
            if (Math.Abs(u2 - u) > Math.PI)
            {
                u = (2 * Math.PI) - u;
                ptSp = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, cyl);
                double pr;
                pr = global::Kerf.Engine.Geometry.ElCLib.Parameter(spine, ptSp);
                global::Kerf.Engine.Geometry.ElCLib.D1(pr, spine, out ptSp2, out dSp);
            }
        }
        // end of modif
        Direction3D dx = new(new Vector3D(or, ptSp));
        dx = dp.Crossed(dx.Crossed(dp));
        Direction3D dy = new(dSp);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cyl, out Point3D ptCyl, out Vector3D vu, out Vector3D vv);
        Direction3D dc = new(vu.Crossed(vv));
        if (or2 == Reversed)
        {
            dc = dc.Reversed();
        }
        Direction3D dz = dp;
        double rad, cylrad = cyl.Radius;
        bool dedans = (dx.Dot(dc) <= 0.0);
        if (dedans)
        {
            if (!plandab)
            {
                dz = dz.Reversed();
            }
            rad = cylrad - radius;
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
            rad = cylrad + radius;
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
        // It is checked if the orientation of the fillet is the same as of faces.
        Point3D p = new(), pp;
        Vector3D deru, derv;
        p = p.WithCoord(cPln.X + rad * dx.X, cPln.Y + rad * dx.Y, cPln.Z + rad * dx.Z);
        u = 0.0;
        if ((dedans && plandab) || (!dedans && !plandab))
        {
            if (c1sphere)
            {
                v = -Math.PI / 2;
            }
            else
            {
                v = 3 * Math.PI / 2;
            }
        }
        else
        {
            v = Math.PI / 2;
        }
        Direction3D norFil;
        if (c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, filAx3, cylrad, out pp, out deru, out derv);
            norFil = filAx3.XDirection.Crossed(filAx3.YDirection);
            if (v < 0.0)
            {
                norFil = norFil.Reversed();
            }
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, filAx3, rad, radius, out pp, out deru, out derv);
            norFil = new Direction3D(deru.Crossed(derv));
        }
        Point2D p2dFil = new(0.0, v);
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
        // The plane face.
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
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, p, out u, out v);
            Point2D p2dPln = new(u, v), pbid = new();
            if (plandab)
            {
                data.Set2dPoints(p2dPln, p2dPln, pbid, pbid);
            }
            else
            {
                data.Set2dPoints(pbid, pbid, p2dPln, p2dPln);
            }
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
        // The cylindrical face.
        p = p.WithCoord(or.X + cylrad * dx.X, or.Y + cylrad * dx.Y, or.Z + cylrad * dx.Z);
        u = 0.0;
        if (dedans)
        {
            if (plandab && !c1sphere)
            {
                v = 2 * Math.PI;
            }
            else
            {
                v = 0.0;
            }
        }
        else
        {
            v = Math.PI;
        }
        p2dFil = p2dFil.WithCoord(u, v);
        if (c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, filAx3, cylrad, out pp, out deru, out derv);
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, filAx3, rad, radius, out pp, out deru, out derv);
        }
        norFil = new Direction3D(deru.Crossed(derv));
        lin2dFil = lin2dFil.WithLocation(p2dFil);
        LineCurve2D gLin2dFil2 = new(lin2dFil);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, p, out u, out v);
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
            u = ChFiKPart_InPeriod(u, fu, fu + 2 * Math.PI, tol);
        }
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cyl, out pp, out deru, out derv);
        Direction3D norcyl = new(deru.Crossed(derv));
        Direction2D d2dCyl = Constants.DX2d();
        if (deru.Dot(new Vector3D(dy)) < 0.0)
        {
            d2dCyl = d2dCyl.Reversed();
            if (careaboutsens && Math.Abs(fu - u) < tol)
            {
                u = lu;
            }
        }
        else if (careaboutsens && Math.Abs(lu - u) < tol)
        {
            u = fu;
        }
        Point2D p2dCyl = new(u, v);
        Line2D lin2dCyl = new(p2dCyl, d2dCyl);
        LineCurve2D gLin2dCyl = new(lin2dCyl);
        circAx2 = circAx2.WithLocation(or);
        Circle3D circCyl = new(circAx2, cylrad);
        CircleCurve3D gCircCyl = new(circCyl);
        toreverse = (norFil.Dot(norcyl) <= 0.0);
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
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCircCyl, dStr), trans,
                                                          gLin2dCyl, gLin2dFil2);
        }
        else
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCircCyl, dStr), trans,
                                                          gLin2dCyl, gLin2dFil2);
        }
        return true;
    }
}
