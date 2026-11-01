// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

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
                                               ChFiDS_ChamfMode theMode, in gp_Pln Pln, in gp_Cylinder Cyl,
                                               double fu, double lu, TopAbs_Orientation Or1,
                                               TopAbs_Orientation Or2, double theDis1, double theDis2,
                                               in gp_Circ Spine, double First, TopAbs_Orientation Ofpl,
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
        gp_Ax3 PosPl = Pln.Position();
        gp_Dir Dpl = PosPl.XDirection().Crossed(PosPl.YDirection());
        gp_Dir norf = Dpl;
        if (Ofpl == TopAbs_REVERSED)
        {
            norf.Reverse();
        }
        if (Or1 == TopAbs_REVERSED)
        {
            Dpl.Reverse();
        }
        // compute the origin Or of the cone
        gp_Pnt Or = Cyl.Location();
        double u, v;
        ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        gp_Pnt2d pt2dPln = new(u, v);
        ElSLib.PlaneD0(u, v, PosPl, out Or);
        gp_Pnt PtPl = Or; // projection of the cylinder origin
                          // on the plane
        // start 3d point on the Spine, tangent vector to the spine on PtSp
        ElCLib.D1(First, Spine, out gp_Pnt PtSp, out gp_Vec DSp);
        gp_Dir Dx = new(new gp_Vec(Or, PtSp));
        gp_Dir Dy = new(DSp);
        ElSLib.Parameters(Cyl, PtSp, out u, out v);
        // point on the cylinder and on the Spine
        ElSLib.D1(u, v, Cyl, out gp_Pnt PtCyl, out gp_Vec Vu, out gp_Vec Vv);
        gp_Dir Dcyl = new(Vu.Crossed(Vv)); // normal to the cylinder in PtSp
        if (Or2 == TopAbs_REVERSED)
        {
            Dcyl.Reverse();
        }
        bool dedans = (Dcyl.Dot(Dx) <= 0.0);
        bool pointu = false;
        double ConRad, Rad, SemiAngl;
        Or.SetCoord(Or.X() + Dis2 * Dpl.X(), Or.Y() + Dis2 * Dpl.Y(), Or.Z() + Dis2 * Dpl.Z());
        // variables used to compute the semiangle of the cone
        gp_Dir Vec1 = new(Or.X() - PtPl.X(), Or.Y() - PtPl.Y(), Or.Z() - PtPl.Z());
        gp_Pnt Pt = new(Or.X() + Dis1 * PosPl.XDirection().X(), Or.Y() + Dis1 * PosPl.XDirection().Y(),
                        Or.Z() + Dis1 * PosPl.XDirection().Z());
        gp_Dir Vec2 = new(Pt.X() - PtPl.X(), Pt.Y() - PtPl.Y(), Pt.Z() - PtPl.Z());
        // compute the parameters of the conical surface
        if (dedans)
        {
            Rad = Cyl.Radius() - Dis1;
            if (Math.Abs(Rad) <= Precision.Confusion())
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
            gp_Dir Dplr = Dpl.Reversed();
            Dpl = Dplr;
        }
        ConRad = Cyl.Radius();
        SemiAngl = Vec1.Angle(Vec2);
        gp_Ax3 ConAx3 = new(Or, Dpl, Dx);
        Geom_ConicalSurface gcon = new(ConAx3, SemiAngl, ConRad);
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
        if (ConAx3.YDirection().Dot(new gp_Dir(DSp)) <= 0.0)
        {
            ConAx3.YReverse();
            gcon.SetPosition(ConAx3);
        }
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcon, DStr));
        // compute the chamfer's orientation according to the orientation
        // of the faces
        // search the normal to the cone
        ElSLib.ConeD1(0.0, 0.0, ConAx3, ConRad, SemiAngl, out Pt, out gp_Vec deru, out gp_Vec derv);
        gp_Dir norCon = new(deru.Crossed(derv));
        bool toreverse = (norCon.Dot(norf) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }
        // we load of the faceInterference with the pcurves and
        //  the 3d curves
        // Case of the plane face
        // NB: in the case 'pointu', no pcurve on the plane surface
        // and no intersection plane-chamfer are needed
        Geom2d_Circle? GCir2dPln = null;
        Geom_Circle? GCirPln = null;
        gp_Ax2 CirAx2 = ConAx3.Ax2();
        CirAx2.SetLocation(PtPl);
        if (!pointu)
        {
            // intersection plane-chamfer
            gp_Circ CirPln = new(CirAx2, Rad);
            GCirPln = new Geom_Circle(CirPln);
            // pcurve on the plane
            ElSLib.PlaneParameters(PosPl, Pt, out u, out v);
            gp_Pnt2d p2dPln = new(u, v);
            gp_Dir2d d2d = new(DSp.Dot(new gp_Vec(PosPl.XDirection())), DSp.Dot(new gp_Vec(PosPl.YDirection())));
            gp_Ax22d ax2dPln = new(pt2dPln, new gp_Dir2d(new gp_Vec2d(pt2dPln, p2dPln)), d2d);
            gp_Circ2d cir2dPln = new(ax2dPln, Rad);
            GCir2dPln = new Geom2d_Circle(cir2dPln);
        }
        // pcurve on the chamfer
        gp_Pnt2d p2dch = new();
        if (plandab)
        {
            v = -Math.Sqrt(Dis1 * Dis1 + Dis2 * Dis2);
        }
        else
        {
            v = Math.Sqrt(Dis1 * Dis1 + Dis2 * Dis2);
        }
        p2dch.SetCoord(0.0, v);
        ElSLib.ConeD1(0.0, v, ConAx3, ConRad, SemiAngl, out Pt, out deru, out derv);
        gp_Lin2d lin2dch = new(p2dch, gp.DX2d());
        Geom2d_Line GLin2dCh1 = new(lin2dch);
        // orientation
        TopAbs_Orientation trans;
        gp_Dir norpl = PosPl.XDirection().Crossed(PosPl.YDirection());
        toreverse = (norCon.Dot(norpl) <= 0.0);
        if ((toreverse && plandab) || (!toreverse && !plandab))
        {
            trans = TopAbs_FORWARD;
        }
        else
        {
            trans = TopAbs_REVERSED;
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
        gp_Circ CirCyl = new(CirAx2, ConRad);
        Geom_Circle GCirCyl = new(CirCyl);
        // pcurve on the chamfer
        p2dch.SetCoord(0.0, 0.0);
        ElSLib.ConeD1(0.0, 0.0, ConAx3, ConRad, SemiAngl, out Pt, out deru, out derv);
        lin2dch.SetLocation(p2dch);
        Geom2d_Line GLin2dCh2 = new(lin2dch);
        // pcurve on the cylinder
        norCon.SetXYZ(deru.Crossed(derv).XYZ());
        Pt.SetCoord(Or.X() + ConRad * Dx.X(), Or.Y() + ConRad * Dx.Y(), Or.Z() + ConRad * Dx.Z());
        ElSLib.Parameters(Cyl, Pt, out u, out v);
        double tol = Precision.PConfusion();
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
        ElSLib.D1(u, v, Cyl, out Pt, out deru, out derv);
        gp_Dir norcyl = new(deru.Crossed(derv));
        gp_Dir2d d2dCyl = gp.DX2d();
        if (deru.Dot(new gp_Vec(Dy)) < 0.0)
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
        gp_Pnt2d p2dCyl = new(u, v);
        gp_Lin2d lin2dCyl = new(p2dCyl, d2dCyl);
        Geom2d_Line GLin2dCyl = new(lin2dCyl);
        // orientation
        toreverse = (norCon.Dot(norcyl) <= 0.0);
        if ((toreverse && plandab) || (!toreverse && !plandab))
        {
            trans = TopAbs_REVERSED;
        }
        else
        {
            trans = TopAbs_FORWARD;
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
                                               ChFiDS_ChamfMode theMode, in gp_Pln Pln, in gp_Cylinder Cyl,
                                               double fu, double lu, TopAbs_Orientation Or1,
                                               TopAbs_Orientation Or2, double dis1, double dis2, in gp_Lin Spine,
                                               double First, TopAbs_Orientation Ofpl, bool plandab)
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
        gp_Pnt OrSpine = ElCLib.Value(First, Spine);
        gp_Pnt POnCyl = new(), POnPln = new(), OrCyl = new();
        gp_Dir XDir = Spine.Direction();
        gp_Ax3 AxPln = Pln.Position();
        gp_Dir NorPln = AxPln.XDirection().Crossed(AxPln.YDirection());
        gp_Dir NorF = NorPln;
        if (Or1 == TopAbs_REVERSED)
        {
            NorF.Reverse();
        }
        gp_Ax3 AxCyl = Cyl.Position();
        // OrCyl is the point on axis of cylinder in the plane normal to the
        // axis containing OrSpine
        // Project <OrSpine> onto <AxCyl>
        gp_XYZ AxLoc = AxCyl.Location().XYZ(); // aLine.Location().XYZ();
        gp_XYZ AxDir = AxCyl.Direction().XYZ();
        double Parameter = OrSpine.XYZ().Subtracted(AxLoc).Dot(AxDir);
        OrCyl.SetXYZ(AxLoc.Added(AxDir.Multiplied(Parameter)));
        // construction of POnPln
        gp_Vec VecTranslPln, tmp;
        tmp = new gp_Vec(OrSpine, OrCyl);
        if ((Or2 == TopAbs_FORWARD && Cyl.Direct()) || (Or2 == TopAbs_REVERSED && !Cyl.Direct()))
        {
            tmp.Reverse();
        }
        VecTranslPln = new gp_Vec(XDir.Crossed(NorPln));
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
        gp_Vec VecCylTransl = new(OrCyl, OrSpine);
        if ((XDir.Crossed(new gp_Dir(VecCylTransl))).Dot(NorF) > 0.0)
        {
            VecCylTransl.Rotate(new gp_Ax1(OrCyl, XDir), alpha);
        }
        else
        {
            VecCylTransl.Rotate(new gp_Ax1(OrCyl, XDir.Reversed()), alpha);
        }
        POnCyl.SetXYZ(OrCyl.XYZ().Added(VecCylTransl.XYZ()));
        // construction of chamfer
        double UOnCyl, VOnCyl, UOnPln, VOnPln;
        ElSLib.Parameters(Cyl, POnCyl, out UOnCyl, out VOnCyl);
        POnCyl = ElSLib.CylinderValue(UOnCyl, VOnCyl, AxCyl, Cyl.Radius());
        ElSLib.Parameters(Pln, POnPln, out UOnPln, out VOnPln);
        POnPln = ElSLib.PlaneValue(UOnPln, VOnPln, AxPln);
        // construction of YDir to go to face1 from face2.
        gp_Vec YDir = new(POnPln, POnCyl);
        if (!plandab)
        {
            YDir.Reverse();
        }
        gp_Ax3 AxCh = new(POnPln, XDir.Crossed(new gp_Dir(YDir)), XDir);
        Geom_Plane Chamfer = new(AxCh);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(Chamfer, DStr));
        // FaceInterferences are loaded with pcurves and curves 3d.
        //----------- edge plane-Chamfer
        gp_Pnt2d PPln2d = new(UOnPln, VOnPln);
        gp_Dir2d VPln2d = new(XDir.Dot(AxPln.XDirection()), XDir.Dot(AxPln.YDirection()));
        gp_Lin2d Lin2dPln = new(PPln2d, VPln2d);
        POnPln = ElSLib.Value(UOnPln, VOnPln, Pln);
        gp_Lin C3d = new(POnPln, XDir);
        double U, VOnChamfer;
        ElSLib.PlaneParameters(AxCh, POnPln, out U, out VOnChamfer);
        gp_Lin2d LOnChamfer = new(new gp_Pnt2d(U, VOnChamfer), gp.DX2d());
        Geom_Line L3d = new(C3d);
        Geom2d_Line LFac = new(Lin2dPln);
        Geom2d_Line LFil = new(LOnChamfer);
        gp_Dir NorFil = AxCh.Direction();
        bool toreverse = (NorFil.Dot(NorPln) <= 0.0);
        gp_Dir DirPlnCyl = new(new gp_Vec(POnPln, POnCyl));
        gp_Dir DirSPln = new(new gp_Vec(OrSpine, POnPln));
        bool PosChamfPln = DirPlnCyl.Dot(DirSPln) > 0;
        if (PosChamfPln)
        {
            toreverse = !toreverse;
        }
        // It is checked if the orientation of the Chamfer is the same as of the plane
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs.Reverse(Ofpl);
        }
        else
        {
            Data.ChangeOrientation() = Ofpl;
        }
        TopAbs_Orientation trans = TopAbs_FORWARD;
        if ((!plandab && toreverse) || (plandab && !toreverse))
        {
            trans = TopAbs_REVERSED;
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
        gp_Pnt2d PCyl2d = new(UOnCyl, VOnCyl);
        gp_Dir2d VCyl2d = gp.DY2d();
        if (XDir.Dot(AxCyl.Direction()) < 0)
        {
            VCyl2d.Reverse();
        }
        gp_Lin2d Lin2dCyl = new(PCyl2d, VCyl2d);
        POnCyl = ElSLib.Value(UOnCyl, VOnCyl, Cyl);
        C3d = new gp_Lin(POnCyl, XDir);
        ElSLib.PlaneParameters(AxCh, POnCyl, out U, out VOnChamfer);
        LOnChamfer = new gp_Lin2d(new gp_Pnt2d(U, VOnChamfer), gp.DX2d());
        L3d = new Geom_Line(C3d);
        LFac = new Geom2d_Line(Lin2dCyl);
        LFil = new Geom2d_Line(LOnChamfer);
        ElSLib.CylinderD1(UOnCyl, VOnCyl, AxCyl, Cyl.Radius(), out POnCyl, out gp_Vec deru, out gp_Vec derv);
        gp_Dir NorCyl = new(deru.Crossed(derv));
        toreverse = (NorFil.Dot(NorCyl) <= 0.0);
        gp_Dir DirSCyl = new(new gp_Vec(OrSpine, POnCyl));
        bool PosChamfCyl = DirPlnCyl.Dot(DirSCyl) < 0;
        if (PosChamfCyl)
        {
            toreverse = !toreverse;
        }
        trans = TopAbs_REVERSED;
        if ((!plandab && toreverse) || (plandab && !toreverse))
        {
            trans = TopAbs_FORWARD;
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
