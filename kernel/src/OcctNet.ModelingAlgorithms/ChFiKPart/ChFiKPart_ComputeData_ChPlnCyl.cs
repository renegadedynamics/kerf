// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The chamfer between a plane and a cylinder: a cone along a circle, a plane along a generatrix.</summary>
internal static class ChFiKPart_ComputeData_ChPlnCyl
{
    //=======================================================================
    // function : MakeChamfer
    // purpose  : Compute the chamfer in the particular case Plane/Cylinder
    //            or Cylinder/Plane
    //            Compute the SurfData <Data> of the chamfer build on the <Spine>
    //            between the plane <Pln> and the cylinder <Cyl>, with the
    //            distances <Dis1> on <Pln> and <Dis2> on <Cyl>.
    //            <Or1> and <Or2> are the orientations of <Pln> and <Cyl>
    //            and <Ofpl> this of the face carried by <Pln>.
    //            <First> is the start point on the <Spine>
    //            <Plandab> is equal to True if the plane is the surface S1
    //            <fu> and <lu> are the first and last u parameters of the
    //            cylinder
    // out      : True if the chanfer has been computed
    //            False else
    //=======================================================================

    /// <summary>The conical chamfer along a circle of the cylinder.</summary>
    internal static bool ChFiKPart_MakeChamfer(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data,
                                               ChFiDS_ChamfMode theMode, in Plane3D Pln, in Cylinder Cyl,
                                               double fu, double lu, Orientation Or1,
                                               Orientation Or2, double theDis1, double theDis2,
                                               in Circle3D Spine, double First, Orientation Ofpl,
                                               bool plandab)
    {
        // compute the chamfer surface(cone)
        double Dis1 = theDis1, Dis2 = theDis2;
        if (theMode == ChFiDS_ConstThroatChamfer)
        {
            Dis1 = Dis2 = theDis1 * Math.Sqrt(2.0);
        }
        else if (theMode == ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            double aDis2 = Math.Min(theDis1, theDis2);
            double aDis1 = Math.Max(theDis1, theDis2);
            Dis2 = Math.Sqrt(aDis1 * aDis1 - aDis2 * aDis2);
            Dis1 = aDis1 * aDis1 / aDis2 - aDis2;
        }
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
        Or.SetCoord(Or.X() + Dis2 * Dpl.X(), Or.Y() + Dis2 * Dpl.Y(), Or.Z() + Dis2 * Dpl.Z());
        // variables used to compute the semiangle of the cone
        Direction3D Vec1 = new(Or.X() - PtPl.X(), Or.Y() - PtPl.Y(), Or.Z() - PtPl.Z());
        Point3D Pt = new(Or.X() + Dis1 * PosPl.XDirection().X(), Or.Y() + Dis1 * PosPl.XDirection().Y(),
                        Or.Z() + Dis1 * PosPl.XDirection().Z());
        Direction3D Vec2 = new(Pt.X() - PtPl.X(), Pt.Y() - PtPl.Y(), Pt.Z() - PtPl.Z());
        // compute the parameters of the conical surface
        if (dedans)
        {
            Rad = Cyl.Radius() - Dis1;
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
            Rad = Cyl.Radius() + Dis1;
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
            v = -Math.Sqrt(Dis1 * Dis1 + Dis2 * Dis2);
        }
        else
        {
            v = Math.Sqrt(Dis1 * Dis1 + Dis2 * Dis2);
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
    // function : MakeChamfer
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The plane chamfer along a generatrix of the cylinder (classic chamfers only).</summary>
    internal static bool ChFiKPart_MakeChamfer(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data,
                                               ChFiDS_ChamfMode theMode, in Plane3D Pln, in Cylinder Cyl,
                                               double fu, double lu, Orientation Or1,
                                               Orientation Or2, double dis1, double dis2, in Line3D Spine,
                                               double First, Orientation Ofpl, bool plandab)
    {
        // calculation of the fillet plane.
        // or1 and or2 permit to determine in which of four sides created by
        // intersection of 2 surfaces we are
        //        _|_          Ofpl is orientation of the plane face allowing
        //         |4          to determine the side of the material
        if (theMode != ChFiDS_ClassicChamfer)
        {
            return false;
        }
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
        // Project <OrSpine> onto <AxCyl>
        Xyz AxLoc = AxCyl.Location().XYZ(); // aLine.Location().XYZ();
        Xyz AxDir = AxCyl.Direction().XYZ();
        double Parameter = OrSpine.XYZ().Subtracted(AxLoc).Dot(AxDir);
        OrCyl.SetXYZ(AxLoc.Added(AxDir.Multiplied(Parameter)));
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
        VecTranslPln.Multiply(dis1);
        POnPln.SetXYZ((OrSpine.XYZ()).Added(VecTranslPln.XYZ()));
        // construction of POnCyl
        double alpha = (2 * Math.Asin(dis2 * 0.5 / Cyl.Radius()));
        //  gp_Vec VecTranslCyl;
        //  VecTranslCyl = gp_Vec(OrSpine,OrCyl);
        //  if ( ( XDir.Crossed(gp_Dir(VecTranslCyl)) ).Dot(NorF) <=0. )
        //    {VecTranslCyl.Rotate(gp_Ax1(OrSpine,XDir),alpha);}
        //  else
        //    {VecTranslCyl.Rotate(gp_Ax1(OrSpine,XDir.Reversed()),alpha);}
        //  POnCyl.SetXYZ( OrCyl.XYZ().Added(VecTranslCyl.XYZ()) );
        Vector3D VecCylTransl = new(OrCyl, OrSpine);
        if ((XDir.Crossed(new Direction3D(VecCylTransl))).Dot(NorF) > 0.0)
        {
            VecCylTransl.Rotate(new Axis3D(OrCyl, XDir), alpha);
        }
        else
        {
            VecCylTransl.Rotate(new Axis3D(OrCyl, XDir.Reversed()), alpha);
        }
        POnCyl.SetXYZ(OrCyl.XYZ().Added(VecCylTransl.XYZ()));
        // construction of chamfer
        double UOnCyl, VOnCyl, UOnPln, VOnPln;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, POnCyl, out UOnCyl, out VOnCyl);
        POnCyl = global::Kerf.Engine.Geometry.ElSLib.CylinderValue(UOnCyl, VOnCyl, AxCyl, Cyl.Radius());
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Pln, POnPln, out UOnPln, out VOnPln);
        POnPln = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(UOnPln, VOnPln, AxPln);
        // construction of YDir to go to face1 from face2.
        Vector3D YDir = new(POnPln, POnCyl);
        if (!plandab)
        {
            YDir.Reverse();
        }
        CoordinateSystem AxCh = new(POnPln, XDir.Crossed(new Direction3D(YDir)), XDir);
        PlaneSurface Chamfer = new(AxCh);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(Chamfer, DStr));
        // FaceInterferences are loaded with pcurves and curves 3d.
        //----------- edge plane-Chamfer
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
        if (PosChamfPln)
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
        // trans permits to determine the "material" side on S1(2) limited by L3d
        if (plandab)
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        else
        {
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        //------------edge cylinder-Chamfer
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
        toreverse = (NorFil.Dot(NorCyl) <= 0.0);
        Direction3D DirSCyl = new(new Vector3D(OrSpine, POnCyl));
        bool PosChamfCyl = DirPlnCyl.Dot(DirSCyl) < 0;
        if (PosChamfCyl)
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
