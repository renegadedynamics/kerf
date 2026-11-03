// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The distance-angle chamfer between a plane and a cylinder: a cone along a circle, a plane along a
/// generatrix.</summary>
internal static class ChFiKPart_ComputeData_ChAsymPlnCyl
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
    internal static bool ChFiKPart_MakeChAsym(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D Pln,
                                              in Cylinder Cyl, double fu, double lu, Orientation Or1,
                                              Orientation Or2, double Dis, double Angle, in Circle3D Spine,
                                              double First, Orientation Ofpl, bool plandab, bool DisOnP)
    {
        // compute the chamfer surface(cone)
        // compute the normals to the plane surface & to the plane face
        CoordinateSystem PosPl = Pln.Position();
        Direction3D Dpl = PosPl.XDirection().Crossed(PosPl.YDirection());
        Direction3D norf = Dpl;
        if (Ofpl == Reversed)
        {
            norf.Reverse();
        }
        if (Or1 == Reversed)
        {
            Dpl.Reverse();
        }
        // compute the origin Or of the cone
        Point3D Or = Cyl.Location();
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        Point2D pt2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, PosPl, out Or);
        Point3D PtPl = Or; // projection of the cylinder origin
                          // on the plane
        // start 3d point on the Spine, tangent vector to the spine on PtSp
        global::Kerf.Engine.Geometry.ElCLib.D1(First, Spine, out Point3D PtSp, out Vector3D DSp);
        Direction3D Dx = new(new Vector3D(Or, PtSp));
        Direction3D Dy = new(DSp);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, PtSp, out u, out v);
        // point on the cylinder and on the Spine
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, Cyl, out Point3D PtCyl, out Vector3D Vu, out Vector3D Vv);
        Direction3D Dcyl = new(Vu.Crossed(Vv)); // normal to the cylinder in PtSp
        if (Or2 == Reversed)
        {
            Dcyl.Reverse();
        }
        bool dedans = (Dcyl.Dot(Dx) <= 0.0);
        bool pointu = false;
        double ConRad, Rad, SemiAngl;
        // Calculation of distance
        double dis1, dis2, cosNPCyl, sinNPCyl;
        if ((plandab && DisOnP) || (!plandab && !DisOnP))
        {
            dis1 = Dis;
            cosNPCyl = Dpl.Dot(Dcyl);
            sinNPCyl = Math.Sqrt(1.0 - cosNPCyl * cosNPCyl);
            dis2 = Dis / (sinNPCyl / Math.Tan(Angle) - cosNPCyl);
        }
        else
        {
            dis2 = Dis;
            cosNPCyl = Dpl.Dot(Dcyl);
            sinNPCyl = Math.Sqrt(1.0 - cosNPCyl * cosNPCyl);
            dis1 = Dis / (sinNPCyl / Math.Tan(Angle) - cosNPCyl);
        }
        Or.SetCoord(Or.X() + dis2 * Dpl.X(), Or.Y() + dis2 * Dpl.Y(), Or.Z() + dis2 * Dpl.Z());
        // variables used to compute the semiangle of the cone
        Direction3D Vec1 = new(Or.X() - PtPl.X(), Or.Y() - PtPl.Y(), Or.Z() - PtPl.Z());
        Point3D Pt = new(Or.X() + dis1 * PosPl.XDirection().X(), Or.Y() + dis1 * PosPl.XDirection().Y(),
                        Or.Z() + dis1 * PosPl.XDirection().Z());
        Direction3D Vec2 = new(Pt.X() - PtPl.X(), Pt.Y() - PtPl.Y(), Pt.Z() - PtPl.Z());
        // compute the parameters of the conical surface
        if (dedans)
        {
            Rad = Cyl.Radius() - dis1;
            if (Math.Abs(Rad) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                pointu = true;
            }
            if (Rad < 0)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        else
        {
            Rad = Cyl.Radius() + dis1;
            Direction3D Dplr = Dpl.Reversed();
            Dpl = Dplr;
        }
        ConRad = Cyl.Radius();
        SemiAngl = Vec1.Angle(Vec2);
        CoordinateSystem ConAx3 = new(Or, Dpl, Dx);
        ConicalSurface gcon = new(ConAx3, SemiAngl, ConRad);
        // changes due to the fact the parameters of the chamfer must go increasing
        // from surface S1 to surface S2
        if ((dedans && !plandab) || (!dedans && plandab))
        {
            gcon.VReverse(); // be careful : the SemiAngle was changed
            ConAx3 = gcon.Position();
            SemiAngl = gcon.SemiAngle();
        }
        // changes due to the fact we have reversed the V direction of
        // parametrization
        if (ConAx3.YDirection().Dot(new Direction3D(DSp)) <= 0.0)
        {
            ConAx3.YReverse();
            gcon.SetPosition(ConAx3);
        }
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcon, DStr));
        // compute the chamfer's orientation according to the orientation
        // of the faces
        // search the normal to the cone
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, 0.0, ConAx3, ConRad, SemiAngl, out Pt, out Vector3D deru, out Vector3D derv);
        Direction3D norCon = new(deru.Crossed(derv));
        bool toreverse = (norCon.Dot(norf) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = Reversed;
        }
        else
        {
            Data.ChangeOrientation() = Forward;
        }
        // we load of the faceInterference with the pcurves and
        //  the 3d curves
        // Case of the plane face
        // NB: in the case 'pointu', no pcurve on the plane surface
        // and no intersection plane-chamfer are needed
        CircleCurve2D? GCir2dPln = null;
        CircleCurve3D? GCirPln = null;
        Frame3D CirAx2 = ConAx3.Ax2();
        CirAx2.SetLocation(PtPl);
        if (!pointu)
        {
            // intersection plane-chamfer
            Circle3D CirPln = new(CirAx2, Rad);
            GCirPln = new CircleCurve3D(CirPln);
            // pcurve on the plane
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, Pt, out u, out v);
            Point2D p2dPln = new(u, v);
            Direction2D d2d = new(DSp.Dot(new Vector3D(PosPl.XDirection())), DSp.Dot(new Vector3D(PosPl.YDirection())));
            Frame2D ax2dPln = new(pt2dPln, new Direction2D(new Vector2D(pt2dPln, p2dPln)), d2d);
            Circle2D cir2dPln = new(ax2dPln, Rad);
            GCir2dPln = new CircleCurve2D(cir2dPln);
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
        p2dch.SetCoord(0.0, v);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, v, ConAx3, ConRad, SemiAngl, out Pt, out deru, out derv);
        Line2D lin2dch = new(p2dch, Constants.DX2d());
        LineCurve2D GLin2dCh1 = new(lin2dch);
        // orientation
        Orientation trans;
        Direction3D norpl = PosPl.XDirection().Crossed(PosPl.YDirection());
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
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GCirPln, DStr), trans,
                                                          GCir2dPln, GLin2dCh1);
        }
        else
        {
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GCirPln, DStr), trans,
                                                          GCir2dPln, GLin2dCh1);
        }
        // Case of the cylindrical face
        // intersection cylinder-chamfer
        CirAx2.SetLocation(Or);
        Circle3D CirCyl = new(CirAx2, ConRad);
        CircleCurve3D GCirCyl = new(CirCyl);
        // pcurve on the chamfer
        p2dch.SetCoord(0.0, 0.0);
        global::Kerf.Engine.Geometry.ElSLib.ConeD1(0.0, 0.0, ConAx3, ConRad, SemiAngl, out Pt, out deru, out derv);
        lin2dch.SetLocation(p2dch);
        LineCurve2D GLin2dCh2 = new(lin2dch);
        // pcurve on the cylinder
        norCon.SetXYZ(deru.Crossed(derv).XYZ());
        Pt.SetCoord(Or.X() + ConRad * Dx.X(), Or.Y() + ConRad * Dx.Y(), Or.Z() + ConRad * Dx.Z());
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, Pt, out u, out v);
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
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, Cyl, out Pt, out deru, out derv);
        Direction3D norcyl = new(deru.Crossed(derv));
        Direction2D d2dCyl = Constants.DX2d();
        if (deru.Dot(new Vector3D(Dy)) < 0.0)
        {
            d2dCyl.Reverse();
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
        LineCurve2D GLin2dCyl = new(lin2dCyl);
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
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GCirCyl, DStr), trans,
                                                          GLin2dCyl, GLin2dCh2);
        }
        else
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GCirCyl, DStr), trans,
                                                          GLin2dCyl, GLin2dCh2);
        }
        return true;
    }

    //=======================================================================
    // function : MakeChAsym
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The plane distance-angle chamfer along a generatrix of the cylinder.</summary>
    internal static bool ChFiKPart_MakeChAsym(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D Pln,
                                              in Cylinder Cyl, double fu, double lu, Orientation Or1,
                                              Orientation Or2, double Dis, double Angle, in Line3D Spine,
                                              double First, Orientation Ofpl, bool plandab, bool DisOnP)
    {
        // calculation of the fillet plane.
        // or1 and or2 permit to determine in which of four sides created by
        // intersection of 2 surfaces we are
        //        _|_          Ofpl is orientation of the plane face allowing
        //         |4          to determine the side of the material
        Point3D OrSpine = global::Kerf.Engine.Geometry.ElCLib.Value(First, Spine);
        Point3D POnCyl = new(), POnPln = new(), OrCyl = new();
        Direction3D XDir = Spine.Direction();
        CoordinateSystem AxPln = Pln.Position();
        Direction3D NorPln = AxPln.XDirection().Crossed(AxPln.YDirection());
        Direction3D NorF = NorPln;
        if (Or1 == Reversed)
        {
            NorF.Reverse();
        }
        CoordinateSystem AxCyl = Cyl.Position();
        // OrCyl is the point on axis of cylinder in the plane normal to the
        // axis containing OrSpine
        Point3D Loc = AxCyl.Location();
        Vector3D LocSp = new(Loc, OrSpine);
        Xyz temp = AxCyl.Direction().XYZ();
        // OCCT: gp_XYZ::Multiplied(gp_XYZ) multiplies coordinate by coordinate (it is not the dot product), so OrCyl
        // is the projection on the axis only when the axis is parallel to a coordinate axis (occt-issues 7N1).
        temp = temp.Multiplied(LocSp.XYZ().Multiplied(temp));
        OrCyl.SetXYZ((Loc.XYZ()).Added(temp));
        // construction of POnPln
        Vector3D VecTranslPln, tmp;
        tmp = new Vector3D(OrSpine, OrCyl);
        if ((Or2 == Forward && Cyl.Direct()) || (Or2 == Reversed && !Cyl.Direct()))
        {
            tmp.Reverse();
        }
        VecTranslPln = new Vector3D(XDir.Crossed(NorPln));
        if (VecTranslPln.Dot(tmp) <= 0.0)
        {
            VecTranslPln.Reverse();
        }
        Vector3D VecTranslCyl;
        VecTranslCyl = new Vector3D(OrSpine, OrCyl);
        // Calculation of distances dis1 and dis2, depending on Dis and Angle
        Vector3D DirSOrC = VecTranslCyl.Normalized();
        double cosA1 = DirSOrC.Dot(VecTranslPln.Normalized());
        double sinA1 = Math.Sqrt(1.0 - cosA1 * cosA1);
        double dis1 = 0.0;
        double dis2, ray = Cyl.Radius();
        bool IsDisOnP = ((plandab && DisOnP) || (!plandab && !DisOnP));
        if (IsDisOnP)
        {
            dis1 = Dis;
            double sinAl = Math.Sin(Angle), cosAl = Math.Cos(Angle);
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
            dis2 = Dis;
        }
        // construction of POnCyl
        double alpha = (2 * Math.Asin(dis2 * 0.5 / ray));
        Vector3D VecTemp = VecTranslCyl.Reversed();
        if ((XDir.Crossed(new Direction3D(VecTranslCyl))).Dot(NorF) < 0.0)
        {
            VecTemp.Rotate(new Axis3D(OrCyl, XDir), alpha);
        }
        else
        {
            VecTemp.Rotate(new Axis3D(OrCyl, XDir.Reversed()), alpha);
        }
        POnCyl.SetXYZ(OrCyl.XYZ().Added(VecTemp.XYZ()));
        double UOnCyl, VOnCyl, UOnPln, VOnPln;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, POnCyl, out UOnCyl, out VOnCyl);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(UOnCyl, VOnCyl, AxCyl, Cyl.Radius(), out POnCyl, out Vector3D DUOnCyl, out Vector3D DVOnCyl);
        // Construction of the point on the plane
        if (!IsDisOnP)
        {
            Vector3D Corde = new(POnCyl, OrSpine);
            Vector3D TCyl = DUOnCyl.Crossed(DVOnCyl);
            TCyl = new Vector3D(XDir.Crossed(new Direction3D(TCyl)));
            TCyl.Normalize();
            Corde.Normalize();
            double cosCorTan = TCyl.Dot(Corde);
            double tgCorTan = 1.0 / (cosCorTan * cosCorTan);
            tgCorTan = Math.Sqrt(tgCorTan - 1.0);
            double tgAng = Math.Tan(Angle);
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
        VecTranslPln.Multiply(dis1);
        POnPln.SetXYZ((OrSpine.XYZ()).Added(VecTranslPln.XYZ()));
        // construction of the chamfer
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Pln, POnPln, out UOnPln, out VOnPln);
        POnPln = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(UOnPln, VOnPln, AxPln);
        // construction of YDir to go from face1 to face2.
        Vector3D YDir = new(POnPln, POnCyl);
        if (!plandab)
        {
            YDir.Reverse();
        }
        CoordinateSystem AxCh = new(POnPln, XDir.Crossed(new Direction3D(YDir)), XDir);
        PlaneSurface Chamfer = new(AxCh);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(Chamfer, DStr));
        // FaceInterferences are loaded with pcurves and curves 3d.
        //----------- edge plan-Chamfer
        Point2D PPln2d = new(UOnPln, VOnPln);
        Direction2D VPln2d = new(XDir.Dot(AxPln.XDirection()), XDir.Dot(AxPln.YDirection()));
        Line2D Lin2dPln = new(PPln2d, VPln2d);
        POnPln = global::Kerf.Engine.Geometry.ElSLib.Value(UOnPln, VOnPln, Pln);
        Line3D C3d = new(POnPln, XDir);
        double U, VOnChamfer;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(AxCh, POnPln, out U, out VOnChamfer);
        Line2D LOnChamfer = new(new Point2D(U, VOnChamfer), Constants.DX2d());
        LineCurve3D L3d = new(C3d);
        LineCurve2D LFac = new(Lin2dPln);
        LineCurve2D LFil = new(LOnChamfer);
        Direction3D NorFil = AxCh.Direction();
        bool toreverse = (NorFil.Dot(NorPln) <= 0.0);
        Direction3D DirPlnCyl = new(new Vector3D(POnPln, POnCyl));
        Direction3D DirSPln = new(new Vector3D(OrSpine, POnPln));
        bool PosChamfPln = DirPlnCyl.Dot(DirSPln) > 0;
        if (!IsDisOnP && PosChamfPln)
        {
            toreverse = !toreverse;
        }
        // It is checked if the orientation of the Chamfer is the same as of the plane
        if (toreverse)
        {
            Data.ChangeOrientation() = global::Kerf.Engine.Topology.TopAbs.Reverse(Ofpl);
        }
        else
        {
            Data.ChangeOrientation() = Ofpl;
        }
        Orientation trans = Forward;
        if ((!plandab && toreverse) || (plandab && !toreverse))
        {
            trans = Reversed;
        }
        // trans allows to determine the "material" side on S1(2) limited by L3d
        if (plandab)
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        else
        {
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        //------------edge cylindre-Chamfer
        Point2D PCyl2d = new(UOnCyl, VOnCyl);
        Direction2D VCyl2d = Constants.DY2d();
        if (XDir.Dot(AxCyl.Direction()) < 0)
        {
            VCyl2d.Reverse();
        }
        Line2D Lin2dCyl = new(PCyl2d, VCyl2d);
        POnCyl = global::Kerf.Engine.Geometry.ElSLib.Value(UOnCyl, VOnCyl, Cyl);
        C3d = new Line3D(POnCyl, XDir);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(AxCh, POnCyl, out U, out VOnChamfer);
        LOnChamfer = new Line2D(new Point2D(U, VOnChamfer), Constants.DX2d());
        L3d = new LineCurve3D(C3d);
        LFac = new LineCurve2D(Lin2dCyl);
        LFil = new LineCurve2D(LOnChamfer);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(UOnCyl, VOnCyl, AxCyl, Cyl.Radius(), out POnCyl, out Vector3D deru, out Vector3D derv);
        Direction3D NorCyl = new(deru.Crossed(derv));
        Direction3D DirSCyl = new(new Vector3D(OrSpine, POnCyl));
        bool PosChamfCyl = DirPlnCyl.Dot(DirSCyl) < 0;
        toreverse = (NorFil.Dot(NorCyl) <= 0.0);
        if (IsDisOnP && PosChamfCyl)
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
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        else
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        return true;
    }
}
