// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCon.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCon.cxx
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

/// <summary>The distance-angle chamfer between a plane and a cone along a circle: a cone or a cylinder.</summary>
internal static class ComputeDataChAsymPlnCon
{
    //=======================================================================
    // function : MakeChAsym
    // purpose  : Compute the chamfer in the particular case Plane/Cone or
    //           Cylinder/Plane
    //           Compute the SurfData <Data> of the chamfer build on the <Spine>
    //           between the plane <Pln> and the cone <Con>, with the
    //           distances <Dis1> on <Pln> and <Dis2> on <Con>.
    //           <Or1> and <Or2> are the orientations of <Pln> and <Con>
    //           and <Ofpl> this of the face carried by <Pln>.
    //           <First> is the start point on the <Spine>
    //           <Plandab> is equal to True if the plane is the surface S1
    //           <fu> and <lu> are the first and last u parameters of the
    //           cone
    // out      : True if the chanfer has been computed
    //           False else
    //=======================================================================

    /// <summary>The chamfer at the distance <paramref name="Dis"/> (on the plane if <paramref name="disOnP"/>) and
    /// the angle <paramref name="angle"/>.</summary>
    internal static bool ChFiKPart_MakeChAsym(DsDataStructure dStr, SurfData data, in Plane3D pln,
                                              in Cone con, double fu, double lu, Orientation or1,
                                              Orientation or2, double Dis, double angle, in Circle3D spine,
                                              double first, Orientation ofpl, bool plandab, bool disOnP)
    {
        // Compute the chamfer surface(cone)
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
        // compute the origin of the conical chamfer PtPl
        Point3D or = con.Location();
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, or, out u, out v);
        Point2D pt2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, posPl, out or);
        Point3D ptPl = or;
        global::Kerf.Engine.Geometry.ElCLib.D1(first, spine, out Point3D ptSp, out Vector3D dSp);
        Direction3D dx = new(new Vector3D(ptPl, ptSp));
        // compute the normal to the cone in PtSp
        Vector3D deru, derv;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(con, ptSp, out u, out v);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out Point3D ptCon, out deru, out derv);
        Direction3D dcon = new(deru.Crossed(derv));
        if (or2 == Reversed)
        {
            dcon = dcon.Reversed();
        }
        bool dedans = (dx.Dot(dcon) <= 0.0);
        bool ouvert = (dpl.Dot(dcon) >= 0.0);
        // variables used to compute the semiangle of the chamfer
        double angCon = con.SemiAngle();
        double move;
        double chamfRad, semiAngl;
        bool pointu = false;
        double dis;
        bool iscylinder = false;
        bool isConPar = false;
        if ((plandab && disOnP) || (!plandab && !disOnP))
        {
            double tgang = Math.Tan(angle), dis11;
            double tgCon = Math.Abs(Math.Tan(angCon));
            if (ouvert)
            {
                move = Dis * tgang / (1.0 - tgCon * tgang);
                dis11 = move * tgCon;
                dis = Dis + dis11;
            }
            else
            {
                move = Dis * tgang / (1.0 + tgCon * tgang);
                dis11 = move * tgCon;
                dis = Dis - dis11;
            }
            // compute the parameters of the conical chamfer
            if (dedans)
            {
                chamfRad = spine.Radius() - Dis;
                if (Math.Abs(chamfRad) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    pointu = true;
                }
                if (chamfRad < 0)
                {
                    // (OCCT_DEBUG trace left out)
                    return false;
                }
                semiAngl = Math.PI / 2.0 - angle;
            }
            else
            {
                chamfRad = spine.Radius() + Dis;
                semiAngl = angle - Math.PI / 2.0;
            }
            if (ouvert)
            {
                if (Math.Abs(angCon) - Math.Abs(semiAngl) > -global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    // (OCCT_DEBUG trace left out)
                    return false;
                }
            }
        }
        else
        {
            double dis1;
            move = Dis * Math.Cos(angCon);
            if (ouvert)
            {
                semiAngl = Math.Abs(angCon) + angle;
                if ((Math.PI / 2.0 - semiAngl) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    // (OCCT_DEBUG trace left out)
                    return false;
                }
                dis1 = move * Math.Tan(semiAngl) - Dis * Math.Abs(Math.Sin(angCon));
                if (!dedans)
                {
                    semiAngl = -semiAngl;
                }
            }
            else
            {
                semiAngl = Math.Abs(angCon) - angle;
                if (Math.Abs(semiAngl) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    iscylinder = true;
                    dis1 = Dis * Math.Abs(Math.Sin(angCon));
                }
                else
                {
                    dis1 = Dis * Math.Abs(Math.Sin(angCon)) - move * Math.Tan(semiAngl);
                }
                if (semiAngl > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isConPar = true;
                }
                if (dedans)
                {
                    semiAngl = -semiAngl;
                }
            }
            // compute the parameters of the conical chamfer
            if (dedans)
            {
                chamfRad = spine.Radius() - dis1;
                if (Math.Abs(chamfRad) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    pointu = true;
                }
                if (chamfRad < 0)
                {
                    // (OCCT_DEBUG trace left out)
                    return false;
                }
            }
            else
            {
                chamfRad = spine.Radius() + dis1;
                // Dpl.Reverse();
            }
            if (ouvert)
            {
                dis = dis1 + Dis * Math.Abs(Math.Sin(angCon));
            }
            else
            {
                dis = dis1 - Dis * Math.Abs(Math.Sin(angCon));
            }
        }
        or = or.WithCoord(or.X() + move * dpl.X(), or.Y() + move * dpl.Y(), or.Z() + move * dpl.Z());
        Point3D pt = new(or.X() + dis * posPl.XDirection().X(), or.Y() + dis * posPl.XDirection().Y(),
                        or.Z() + dis * posPl.XDirection().Z());
        CoordinateSystem chamfAx3 = new(ptPl, dpl, dx);
        if (iscylinder)
        {
            CylindricalSurface gcyl = new(chamfAx3, chamfRad);
            // changes due to the fact the parameters of the chamfer must go increasing
            // from surface S1 to surface S2
            if (!plandab)
            {
                gcyl.VReverse(); // be careful : the SemiAngle was changed
                chamfAx3 = gcyl.Position();
            }
            // changes due to the fact we have reversed the V direction of
            // parametrization
            if (chamfAx3.YDirection().Dot(new Direction3D(dSp)) <= 0.0)
            {
                chamfAx3 = chamfAx3.YReversed();
                gcyl.SetPosition(chamfAx3);
            }
            data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcyl, dStr));
            bool torevcha = !chamfAx3.Direct();
            Direction3D cylaxe = (chamfAx3.Axis()).Direction();
            torevcha = ((torevcha && !plandab) || (!torevcha && plandab));
            if (torevcha)
            {
                cylaxe = cylaxe.Reversed();
            }
            bool toreverse = (norf.Dot(cylaxe) < 0.0);
            if ((toreverse && dedans) || (!toreverse && !dedans))
            {
                data.ChangeOrientation() = Reversed;
            }
            else
            {
                data.ChangeOrientation() = Forward;
            }
            // we load the faceInterference with the pcurves and
            //  the 3d curves
            // Case of the plane face
            // NB: in the case 'pointu', no pcurve on the plane surface
            // and no intersection plane-chamfer are needed
            // intersection plane-chamfer
            CircleCurve3D gCirPln;
            CircleCurve2D gCir2dPln;
            Frame3D cirAx2 = chamfAx3.Ax2();
            cirAx2 = cirAx2.WithLocation(ptPl);
            pt = pt.WithCoord(ptPl.X() + chamfRad * dx.X(), ptPl.Y() + chamfRad * dx.Y(), ptPl.Z() + chamfRad * dx.Z());
            Circle3D cirPln = new(cirAx2, chamfRad);
            gCirPln = new CircleCurve3D(cirPln);
            // pcurve on the plane
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, pt, out u, out v);
            Point2D p2dPln = new(u, v);
            Direction2D d2d = new(dSp.Dot(new Vector3D(posPl.XDirection())), dSp.Dot(new Vector3D(posPl.YDirection())));
            Frame2D ax2dPln = new(pt2dPln, new Direction2D(new Vector2D(pt2dPln, p2dPln)), d2d);
            Circle2D cir2dPln = new(ax2dPln, chamfRad);
            gCir2dPln = new CircleCurve2D(cir2dPln);
            // pcurve on chamfer
            Point2D p2dch = new Point2D().WithCoord(0.0, 0.0);
            // ElSLib::CylinderD1(0.,0.,ChamfAx3,ChamfRad,Pt,deru,derv);
            Line2D lin2dch = new(p2dch, Constants.DX2d());
            LineCurve2D gLin2dCh1 = new(lin2dch);
            // orientation
            Orientation trans;
            Direction3D norpl = posPl.XDirection().Crossed(posPl.YDirection());
            toreverse = (norpl.Dot(cylaxe) < 0.0);
            toreverse = (toreverse && plandab) || (!toreverse && !plandab);
            if ((toreverse && dedans) || (!toreverse && !dedans))
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
            // Case of the conical face
            // intersection cone-chamfer
            double rad;
            if (dedans)
            {
                rad = chamfRad + dis;
            }
            else
            {
                rad = chamfRad - dis;
            }
            cirAx2 = cirAx2.WithLocation(or);
            Circle3D cirCon = new(cirAx2, rad);
            CircleCurve3D gCirCon = new(cirCon);
            // pcurve on chamfer
            if (plandab)
            {
                v = Math.Sqrt(dis * dis + move * move);
            }
            else
            {
                v = -Math.Sqrt(dis * dis + move * move);
            }
            p2dch = p2dch.WithCoord(0.0, v);
            global::Kerf.Engine.Geometry.ElSLib.CylinderD1(0.0, v, chamfAx3, chamfRad, out pt, out deru, out derv);
            lin2dch = lin2dch.WithLocation(p2dch);
            LineCurve2D gLin2dCh2 = new(lin2dch);
            // pcurve on cone
            pt = pt.WithCoord(or.X() + rad * dx.X(), or.Y() + rad * dx.Y(), or.Z() + rad * dx.Z());
            global::Kerf.Engine.Geometry.ElSLib.Parameters(con, pt, out u, out v);
            double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (u >= 2 * Math.PI - tol && u <= 2 * Math.PI)
            {
                u = 0.0;
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
            global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out pt, out deru, out derv);
            Point2D p2dCon = new(u, v);
            Direction2D d2dCon;
            if (deru.Dot(dSp) <= 0.0)
            {
                d2dCon = Constants.DX2d().Reversed();
            }
            else
            {
                d2dCon = Constants.DX2d();
            }
            Line2D lin2dCon = new(p2dCon, d2dCon);
            LineCurve2D gLin2dCon = new(lin2dCon);
            // orientation
            Direction3D norcon = new(deru.Crossed(derv));
            Direction3D dirCon = (con.Axis()).Direction();
            if (angCon > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                dirCon = dirCon.Reversed();
            }
            bool torevcon = (norcon.Dot(dirCon) < 0.0);
            if ((torevcon && dedans) || (!torevcon && !dedans))
            {
                trans = Reversed;
            }
            else
            {
                trans = Forward;
            }
            if (plandab)
            {
                data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCirCon, dStr), trans,
                                                              gLin2dCon, gLin2dCh2);
            }
            else
            {
                data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCirCon, dStr), trans,
                                                              gLin2dCon, gLin2dCh2);
            }
        }
        else
        {
            ConicalSurface gcon = new(chamfAx3, semiAngl, chamfRad);
            // changes due to the fact the parameters of the chamfer must go increasing
            // from surface S1 to surface S2
            if (!plandab)
            {
                gcon.VReverse(); // be careful : the SemiAngle was changed
                chamfAx3 = gcon.Position();
                semiAngl = gcon.SemiAngle();
            }
            // changes due to the fact we have reversed the V direction of
            // parametrization
            if (chamfAx3.YDirection().Dot(new Direction3D(dSp)) <= 0.0)
            {
                chamfAx3 = chamfAx3.YReversed();
                gcon.SetPosition(chamfAx3);
            }
            data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcon, dStr));
            // compute the chamfer's orientation according to the orientation
            //  of the faces
            // search the normal to the conical chamfer
            u = 0.0;
            if (plandab)
            {
                v = Math.Sqrt(dis * dis + move * move);
            }
            else
            {
                v = -Math.Sqrt(dis * dis + move * move);
            }
            global::Kerf.Engine.Geometry.ElSLib.ConeD1(u, v, chamfAx3, chamfRad, semiAngl, out Point3D p, out deru, out derv);
            Direction3D norchamf = new(deru.Crossed(derv));
            bool toreverse = (norf.Dot(norchamf) < 0.0);
            if (isConPar)
            {
                toreverse = !toreverse;
            }
            if (toreverse)
            {
                data.ChangeOrientation() = Reversed;
            }
            else
            {
                data.ChangeOrientation() = Forward;
            }
            // we load the faceInterference with the pcurves and
            //  the 3d curves
            // Case of the plane face
            // NB: in the case 'pointu', no pcurve on the plane surface
            // and no intersection plane-chamfer are needed
            // intersection plane-chamfer
            CircleCurve3D? gCirPln = null;
            CircleCurve2D? gCir2dPln = null;
            Frame3D cirAx2 = chamfAx3.Ax2();
            cirAx2 = cirAx2.WithLocation(ptPl);
            if (!pointu)
            {
                pt = pt.WithCoord(ptPl.X() + chamfRad * dx.X(), ptPl.Y() + chamfRad * dx.Y(),
                            ptPl.Z() + chamfRad * dx.Z());
                Circle3D cirPln = new(cirAx2, chamfRad);
                gCirPln = new CircleCurve3D(cirPln);
                // pcurve on the plane
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(posPl, pt, out u, out v);
                Point2D p2dPln = new(u, v);
                Direction2D d2d = new(dSp.Dot(new Vector3D(posPl.XDirection())),
                                   dSp.Dot(new Vector3D(posPl.YDirection())));
                Frame2D ax2dPln = new(pt2dPln, new Direction2D(new Vector2D(pt2dPln, p2dPln)), d2d);
                Circle2D cir2dPln = new(ax2dPln, chamfRad);
                gCir2dPln = new CircleCurve2D(cir2dPln);
            }
            // pcurve on chamfer
            Point2D p2dch = new Point2D().WithCoord(0.0, 0.0);
            global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, 0.0, chamfAx3, chamfRad, semiAngl, out pt, out deru, out derv);
            Line2D lin2dch = new(p2dch, Constants.DX2d());
            LineCurve2D gLin2dCh1 = new(lin2dch);
            // orientation
            Orientation trans;
            Direction3D norpl = posPl.XDirection().Crossed(posPl.YDirection());
            if (!pointu)
            {
                norchamf = norchamf.WithXYZ(deru.Crossed(derv).XYZ());
            }
            toreverse = (norchamf.Dot(norpl) <= 0.0);
            if (isConPar)
            {
                toreverse = !toreverse;
            }
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
            // Case of the conical face
            // intersection cone-chamfer
            double rad;
            if (dedans)
            {
                rad = chamfRad + dis;
            }
            else
            {
                rad = chamfRad - dis;
            }
            cirAx2 = cirAx2.WithLocation(or);
            Circle3D cirCon = new(cirAx2, rad);
            CircleCurve3D gCirCon = new(cirCon);
            // pcurve on chamfer
            if (plandab)
            {
                v = Math.Sqrt(dis * dis + move * move);
            }
            else
            {
                v = -Math.Sqrt(dis * dis + move * move);
            }
            p2dch = p2dch.WithCoord(0.0, v);
            global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, v, chamfAx3, chamfRad, semiAngl, out pt, out deru, out derv);
            lin2dch = lin2dch.WithLocation(p2dch);
            LineCurve2D gLin2dCh2 = new(lin2dch);
            // pcurve on cone
            norchamf = norchamf.WithXYZ(deru.Crossed(derv).XYZ());
            pt = pt.WithCoord(or.X() + rad * dx.X(), or.Y() + rad * dx.Y(), or.Z() + rad * dx.Z());
            global::Kerf.Engine.Geometry.ElSLib.Parameters(con, pt, out u, out v);
            double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
            if (u >= 2 * Math.PI - tol && u <= 2 * Math.PI)
            {
                u = 0.0;
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
            global::Kerf.Engine.Geometry.ElSLib.D1(u, v, con, out pt, out deru, out derv);
            Point2D p2dCon = new(u, v);
            Direction2D d2dCon;
            if (deru.Dot(dSp) <= 0.0)
            {
                d2dCon = Constants.DX2d().Reversed();
            }
            else
            {
                d2dCon = Constants.DX2d();
            }
            Line2D lin2dCon = new(p2dCon, d2dCon);
            LineCurve2D gLin2dCon = new(lin2dCon);
            // orientation
            Direction3D norcon = new(deru.Crossed(derv));
            Direction3D dirCon = (con.Axis()).Direction();
            Direction3D dirChamf = (gcon.Axis()).Direction();
            if (angCon > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                dirCon = dirCon.Reversed();
            }
            if (semiAngl > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                dirChamf = dirChamf.Reversed();
            }
            bool torevcon = (norcon.Dot(dirCon) > 0.0);
            bool torevcha = (norchamf.Dot(dirChamf) > 0.0);
            toreverse = ((torevcon && !torevcha) || (!torevcon && torevcha));
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
                data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gCirCon, dStr), trans,
                                                              gLin2dCon, gLin2dCh2);
            }
            else
            {
                data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gCirCon, dStr), trans,
                                                              gLin2dCon, gLin2dCh2);
            }
        }
        return true;
    }
}
