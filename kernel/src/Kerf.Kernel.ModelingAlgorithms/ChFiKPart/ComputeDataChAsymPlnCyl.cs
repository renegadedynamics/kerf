// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The distance-angle chamfer between a plane and a cylinder: a cone along a circle, a plane along a
/// generatrix.</summary>
internal static class ComputeDataChAsymPlnCyl
{
    //=======================================================================
    // function : MakeChAsym
    // purpose  : Compute the chamfer in the particular case Plane/Cylinder
    //            or Cylinder/Plane
    //            Compute the SurfData <Data> of the chamfer build on the <Spine>
    //            between the plane <Pln> and the cylinder <Cyl>, with the
    //            distances <Dis> and <Angle> on <Pln> on <Cyl>.
    //            <Or1> and <Or2> are the orientations of <Pln> and <Cyl>
    //            and <Ofpl> this of the face carried by <Pln>.
    //            <First> is the start point on the <Spine>
    //            <Plandab> is equal to True if the plane is the surface S1
    //            <fu> and <lu> are the first and last u parameters of the
    //            cylinder
    //            DisOnPlan is equal to True if Dis on plan, else False
    // out      : True if the chanfer has been computed
    //            False else
    //=======================================================================

    /// <summary>The conical distance-angle chamfer along a circle of the cylinder.</summary>
    internal static bool ChFiKPart_MakeChAsym(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cylinder cyl, double fu, double lu, Orientation or1,
                                              Orientation or2, double dis, double angle, in Circle3D spine,
                                              double first, Orientation ofpl, bool plandab, bool disOnP)
    {
        // compute the chamfer surface(cone)
        // compute the normals to the plane surface & to the plane face
        CoordinateSystem posPl = pln.Position();
        Direction3D dpl = posPl.XDirection().Crossed(posPl.YDirection());
        Direction3D norf = dpl;
        if (ofpl == Reversed)
        {
            norf = norf.Reversed();
        }
        if (or1 == Reversed)
        {
            dpl = dpl.Reversed();
        }
        // compute the origin Or of the cone
        Point3D or = cyl.Location();
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, or, out u, out v);
        Point2D pt2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out or);
        Point3D ptPl = or; // projection of the cylinder origin
                          // on the plane
        // start 3d point on the Spine, tangent vector to the spine on PtSp
        global::Kerf.Engine.Geometry.ElCLib.D1(first, spine, out Point3D ptSp, out Vector3D dSp);
        Direction3D dx = new(new Vector3D(or, ptSp));
        Direction3D dy = new(dSp);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, ptSp, out u, out v);
        // point on the cylinder and on the Spine
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cyl, out Point3D ptCyl, out Vector3D vu, out Vector3D vv);
        Direction3D dcyl = new(vu.Crossed(vv)); // normal to the cylinder in PtSp
        if (or2 == Reversed)
        {
            dcyl = dcyl.Reversed();
        }
        bool dedans = (dcyl.Dot(dx) <= 0.0);
        bool pointu = false;
        double conRad, rad, semiAngl;
        // Calculation of distance
        double dis1, dis2, cosNPCyl, sinNPCyl;
        if ((plandab && disOnP) || (!plandab && !disOnP))
        {
            dis1 = dis;
            cosNPCyl = dpl.Dot(dcyl);
            sinNPCyl = Math.Sqrt(1.0 - cosNPCyl * cosNPCyl);
            dis2 = dis / (sinNPCyl / Math.Tan(angle) - cosNPCyl);
        }
        else
        {
            dis2 = dis;
            cosNPCyl = dpl.Dot(dcyl);
            sinNPCyl = Math.Sqrt(1.0 - cosNPCyl * cosNPCyl);
            dis1 = dis / (sinNPCyl / Math.Tan(angle) - cosNPCyl);
        }
        or = or.WithCoord(or.X() + dis2 * dpl.X(), or.Y() + dis2 * dpl.Y(), or.Z() + dis2 * dpl.Z());
        // variables used to compute the semiangle of the cone
        Direction3D vec1 = new(or.X() - ptPl.X(), or.Y() - ptPl.Y(), or.Z() - ptPl.Z());
        Point3D pt = new(or.X() + dis1 * posPl.XDirection().X(), or.Y() + dis1 * posPl.XDirection().Y(),
                        or.Z() + dis1 * posPl.XDirection().Z());
        Direction3D vec2 = new(pt.X() - ptPl.X(), pt.Y() - ptPl.Y(), pt.Z() - ptPl.Z());
        // compute the parameters of the conical surface
        if (dedans)
        {
            rad = cyl.Radius() - dis1;
            if (Math.Abs(rad) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                pointu = true;
            }
            if (rad < 0)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        else
        {
            rad = cyl.Radius() + dis1;
            Direction3D dplr = dpl.Reversed();
            dpl = dplr;
        }
        conRad = cyl.Radius();
        semiAngl = vec1.Angle(vec2);
        CoordinateSystem conAx3 = new(or, dpl, dx);
        ConicalSurface gcon = new(conAx3, semiAngl, conRad);
        // changes due to the fact the parameters of the chamfer must go increasing
        // from surface S1 to surface S2
        if ((dedans && !plandab) || (!dedans && plandab))
        {
            gcon.VReverse(); // be careful : the SemiAngle was changed
            conAx3 = gcon.Position();
            semiAngl = gcon.SemiAngle();
        }
        // changes due to the fact we have reversed the V direction of
        // parametrization
        if (conAx3.YDirection().Dot(new Direction3D(dSp)) <= 0.0)
        {
            conAx3 = conAx3.YReversed();
            gcon.SetPosition(conAx3);
        }
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcon, dStr));
        // compute the chamfer's orientation according to the orientation
        // of the faces
        // search the normal to the cone
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, 0.0, conAx3, conRad, semiAngl, out pt, out Vector3D deru, out Vector3D derv);
        Direction3D norCon = new(deru.Crossed(derv));
        bool toreverse = (norCon.Dot(norf) <= 0.0);
        if (toreverse)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }
        // we load of the faceInterference with the pcurves and
        //  the 3d curves
        // Case of the plane face
        // NB: in the case 'pointu', no pcurve on the plane surface
        // and no intersection plane-chamfer are needed
        CircleCurve2D? gCir2dPln = null;
        CircleCurve3D? gCirPln = null;
        Frame3D cirAx2 = conAx3.Ax2();
        cirAx2 = cirAx2.WithLocation(ptPl);
        if (!pointu)
        {
            // intersection plane-chamfer
            Circle3D cirPln = new(cirAx2, rad);
            gCirPln = new CircleCurve3D(cirPln);
            // pcurve on the plane
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, pt, out u, out v);
            Point2D p2dPln = new(u, v);
            Direction2D d2d = new(dSp.Dot(new Vector3D(posPl.XDirection())), dSp.Dot(new Vector3D(posPl.YDirection())));
            Frame2D ax2dPln = new(pt2dPln, new Direction2D(new Vector2D(pt2dPln, p2dPln)), d2d);
            Circle2D cir2dPln = new(ax2dPln, rad);
            gCir2dPln = new CircleCurve2D(cir2dPln);
        }
        // pcurve on the chamfer
        Point2D p2dch = new();
        if (plandab)
        {
            v = -Math.Sqrt(dis1 * dis1 + dis2 * dis2);
        }
        else
        {
            v = Math.Sqrt(dis1 * dis1 + dis2 * dis2);
        }
        p2dch = p2dch.WithCoord(0.0, v);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, v, conAx3, conRad, semiAngl, out pt, out deru, out derv);
        Line2D lin2dch = new(p2dch, Constants.DX2d());
        LineCurve2D gLin2dCh1 = new(lin2dch);
        // orientation
        Orientation trans;
        Direction3D norpl = posPl.XDirection().Crossed(posPl.YDirection());
        toreverse = (norCon.Dot(norpl) <= 0.0);
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
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCirPln, dStr), trans,
                                                          gCir2dPln, gLin2dCh1);
        }
        else
        {
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCirPln, dStr), trans,
                                                          gCir2dPln, gLin2dCh1);
        }
        // Case of the cylindrical face
        // intersection cylinder-chamfer
        cirAx2 = cirAx2.WithLocation(or);
        Circle3D cirCyl = new(cirAx2, conRad);
        CircleCurve3D gCirCyl = new(cirCyl);
        // pcurve on the chamfer
        p2dch = p2dch.WithCoord(0.0, 0.0);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, 0.0, conAx3, conRad, semiAngl, out pt, out deru, out derv);
        lin2dch = lin2dch.WithLocation(p2dch);
        LineCurve2D gLin2dCh2 = new(lin2dch);
        // pcurve on the cylinder
        norCon = norCon.WithXYZ(deru.Crossed(derv).XYZ());
        pt = pt.WithCoord(or.X() + conRad * dx.X(), or.Y() + conRad * dx.Y(), or.Z() + conRad * dx.Z());
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, pt, out u, out v);
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
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cyl, out pt, out deru, out derv);
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
        // orientation
        toreverse = (norCon.Dot(norcyl) <= 0.0);
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
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCirCyl, dStr), trans,
                                                          gLin2dCyl, gLin2dCh2);
        }
        else
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCirCyl, dStr), trans,
                                                          gLin2dCyl, gLin2dCh2);
        }
        return true;
    }

    //=======================================================================
    // function : MakeChAsym
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The plane distance-angle chamfer along a generatrix of the cylinder.</summary>
    internal static bool ChFiKPart_MakeChAsym(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cylinder cyl, double fu, double lu, Orientation or1,
                                              Orientation or2, double dis, double angle, in Line3D spine,
                                              double first, Orientation ofpl, bool plandab, bool disOnP)
    {
        // calculation of the fillet plane.
        // or1 and or2 permit to determine in which of four sides created by
        // intersection of 2 surfaces we are
        //        _|_          Ofpl is orientation of the plane face allowing
        //         |4          to determine the side of the material
        Point3D orSpine = global::Kerf.Engine.Geometry.ElCLib.Value(first, spine);
        Point3D pOnCyl = new(), pOnPln = new(), orCyl = new();
        Direction3D xDir = spine.Direction();
        CoordinateSystem axPln = pln.Position();
        Direction3D norPln = axPln.XDirection().Crossed(axPln.YDirection());
        Direction3D norF = norPln;
        if (or1 == Reversed)
        {
            norF = norF.Reversed();
        }
        CoordinateSystem axCyl = cyl.Position();
        // OrCyl is the point on axis of cylinder in the plane normal to the
        // axis containing OrSpine
        Point3D loc = axCyl.Location();
        Vector3D locSp = new(loc, orSpine);
        Xyz temp = axCyl.Direction().XYZ();
        // OCCT: gp_XYZ::Multiplied(gp_XYZ) multiplies coordinate by coordinate (it is not the dot product), so OrCyl
        // is the projection on the axis only when the axis is parallel to a coordinate axis (occt-issues 7N1).
        temp = temp.Multiplied(locSp.XYZ().Multiplied(temp));
        orCyl = orCyl.WithXYZ((loc.XYZ()).Added(temp));
        // construction of POnPln
        Vector3D vecTranslPln, tmp;
        tmp = new Vector3D(orSpine, orCyl);
        if ((or2 == Forward && cyl.Direct()) || (or2 == Reversed && !cyl.Direct()))
        {
            tmp = tmp.Reversed();
        }
        vecTranslPln = new Vector3D(xDir.Crossed(norPln));
        if (vecTranslPln.Dot(tmp) <= 0.0)
        {
            vecTranslPln = vecTranslPln.Reversed();
        }
        Vector3D vecTranslCyl;
        vecTranslCyl = new Vector3D(orSpine, orCyl);
        // Calculation of distances dis1 and dis2, depending on Dis and Angle
        Vector3D dirSOrC = vecTranslCyl.Normalized();
        double cosA1 = dirSOrC.Dot(vecTranslPln.Normalized());
        double sinA1 = Math.Sqrt(1.0 - cosA1 * cosA1);
        double dis1 = 0.0;
        double dis2, ray = cyl.Radius();
        bool isDisOnP = ((plandab && disOnP) || (!plandab && !disOnP));
        if (isDisOnP)
        {
            dis1 = dis;
            double sinAl = Math.Sin(angle), cosAl = Math.Cos(angle);
            double h = dis1 * sinAl;
            double cosAhOC = cosA1 * sinAl + sinA1 * cosAl;
            double sinAhOC = sinA1 * sinAl - cosA1 * cosAl;
            if (cosA1 > 0)
            {
                sinAhOC = -sinAhOC;
            }
            double temp1 = h / ray, temp2 = sinAhOC * sinAhOC + temp1 * cosAhOC;
            dis2 = temp2 + temp1 * (cosAhOC - temp1);
            if (dis2 < -1.0E-09)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
            else if (dis2 < 1.0E-09)
            {
                dis2 = ray * Math.Sqrt(2.0 * temp2);
            }
            else
            {
                dis2 = ray * Math.Sqrt(2.0 * (temp2 - sinAhOC * Math.Sqrt(dis2)));
            }
        }
        else
        {
            dis2 = dis;
        }
        // construction of POnCyl
        double alpha = (2 * Math.Asin(dis2 * 0.5 / ray));
        Vector3D vecTemp = vecTranslCyl.Reversed();
        if ((xDir.Crossed(new Direction3D(vecTranslCyl))).Dot(norF) < 0.0)
        {
            vecTemp = vecTemp.Rotated(new Axis3D(orCyl, xDir), alpha);
        }
        else
        {
            vecTemp = vecTemp.Rotated(new Axis3D(orCyl, xDir.Reversed()), alpha);
        }
        pOnCyl = pOnCyl.WithXYZ(orCyl.XYZ().Added(vecTemp.XYZ()));
        double uOnCyl, vOnCyl, uOnPln, vOnPln;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cyl, pOnCyl, out uOnCyl, out vOnCyl);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(uOnCyl, vOnCyl, axCyl, cyl.Radius(), out pOnCyl, out Vector3D duOnCyl, out Vector3D dvOnCyl);
        // Construction of the point on the plane
        if (!isDisOnP)
        {
            Vector3D corde = new(pOnCyl, orSpine);
            Vector3D tCyl = duOnCyl.Crossed(dvOnCyl);
            tCyl = new Vector3D(xDir.Crossed(new Direction3D(tCyl)));
            tCyl = tCyl.Normalized();
            corde = corde.Normalized();
            double cosCorTan = tCyl.Dot(corde);
            double tgCorTan = 1.0 / (cosCorTan * cosCorTan);
            tgCorTan = Math.Sqrt(tgCorTan - 1.0);
            double tgAng = Math.Tan(angle);
            tgAng = (tgAng + tgCorTan) / (1.0 - tgAng * tgCorTan);
            double cosA11 = dis2 / (2.0 * ray);
            double sinA11 = Math.Sqrt(1.0 - cosA11 * cosA11);
            if (cosA1 > 0)
            {
                sinA11 = -sinA11;
            }
            dis1 = (sinA1 + cosA1 * tgAng) * cosA11;
            dis1 -= (cosA1 - sinA1 * tgAng) * sinA11;
            dis1 = (dis2 * tgAng) / dis1;
        }
        vecTranslPln = vecTranslPln.Multiplied(dis1);
        pOnPln = pOnPln.WithXYZ((orSpine.XYZ()).Added(vecTranslPln.XYZ()));
        // construction of the chamfer
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pln, pOnPln, out uOnPln, out vOnPln);
        pOnPln = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(uOnPln, vOnPln, axPln);
        // construction of YDir to go from face1 to face2.
        Vector3D yDir = new(pOnPln, pOnCyl);
        if (!plandab)
        {
            yDir = yDir.Reversed();
        }
        CoordinateSystem axCh = new(pOnPln, xDir.Crossed(new Direction3D(yDir)), xDir);
        PlaneSurface chamfer = new(axCh);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(chamfer, dStr));
        // FaceInterferences are loaded with pcurves and curves 3d.
        //----------- edge plan-Chamfer
        Point2D pPln2d = new(uOnPln, vOnPln);
        Direction2D vPln2d = new(xDir.Dot(axPln.XDirection()), xDir.Dot(axPln.YDirection()));
        Line2D lin2dPln = new(pPln2d, vPln2d);
        pOnPln = global::Kerf.Engine.Geometry.ElSLib.Value(uOnPln, vOnPln, pln);
        Line3D c3d = new(pOnPln, xDir);
        double u, vOnChamfer;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(axCh, pOnPln, out u, out vOnChamfer);
        Line2D lOnChamfer = new(new Point2D(u, vOnChamfer), Constants.DX2d());
        LineCurve3D l3d = new(c3d);
        LineCurve2D lFac = new(lin2dPln);
        LineCurve2D lFil = new(lOnChamfer);
        Direction3D norFil = axCh.Direction();
        bool toreverse = (norFil.Dot(norPln) <= 0.0);
        Direction3D dirPlnCyl = new(new Vector3D(pOnPln, pOnCyl));
        Direction3D dirSPln = new(new Vector3D(orSpine, pOnPln));
        bool posChamfPln = dirPlnCyl.Dot(dirSPln) > 0;
        if (!isDisOnP && posChamfPln)
        {
            toreverse = !toreverse;
        }
        // It is checked if the orientation of the Chamfer is the same as of the plane
        if (toreverse)
        {
            data.ChangeOrientation() = global::Kerf.Engine.Topology.TopAbs.Reverse(ofpl);
        }
        else
        {
            data.ChangeOrientation() = ofpl;
        }
        Orientation trans = Forward;
        if ((!plandab && toreverse) || (plandab && !toreverse))
        {
            trans = Reversed;
        }
        // trans allows to determine the "material" side on S1(2) limited by L3d
        if (plandab)
        {
            data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        else
        {
            data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(l3d, dStr), trans, lFac, lFil);
        }
        //------------edge cylindre-Chamfer
        Point2D pCyl2d = new(uOnCyl, vOnCyl);
        Direction2D vCyl2d = Constants.DY2d();
        if (xDir.Dot(axCyl.Direction()) < 0)
        {
            vCyl2d = vCyl2d.Reversed();
        }
        Line2D lin2dCyl = new(pCyl2d, vCyl2d);
        pOnCyl = global::Kerf.Engine.Geometry.ElSLib.Value(uOnCyl, vOnCyl, cyl);
        c3d = new Line3D(pOnCyl, xDir);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(axCh, pOnCyl, out u, out vOnChamfer);
        lOnChamfer = new Line2D(new Point2D(u, vOnChamfer), Constants.DX2d());
        l3d = new LineCurve3D(c3d);
        lFac = new LineCurve2D(lin2dCyl);
        lFil = new LineCurve2D(lOnChamfer);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(uOnCyl, vOnCyl, axCyl, cyl.Radius(), out pOnCyl, out Vector3D deru, out Vector3D derv);
        Direction3D norCyl = new(deru.Crossed(derv));
        Direction3D dirSCyl = new(new Vector3D(orSpine, pOnCyl));
        bool posChamfCyl = dirPlnCyl.Dot(dirSCyl) < 0;
        toreverse = (norFil.Dot(norCyl) <= 0.0);
        if (isDisOnP && posChamfCyl)
        {
            toreverse = !toreverse;
        }
        trans = Reversed;
        if ((!plandab && toreverse) || (plandab && !toreverse))
        {
            trans = Forward;
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
}
