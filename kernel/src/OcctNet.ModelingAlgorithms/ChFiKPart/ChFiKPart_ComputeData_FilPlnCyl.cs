// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The constant radius fillet between a plane and a cylinder: a cylinder along a generatrix, a torus (or a
/// sphere) along a circle.</summary>
internal static class ChFiKPart_ComputeData_FilPlnCyl
{
    //=======================================================================
    // function : MakeFillet
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The fillet along a generatrix of the cylinder (a straight guide).</summary>
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln Pln,
                                              in gp_Cylinder Cyl, double fu, double lu, TopAbs_Orientation Or1,
                                              TopAbs_Orientation Or2, double Radius, in gp_Lin Spine, double First,
                                              TopAbs_Orientation Ofpl, bool plandab)
    {
        // calculate the cylinder fillet.
        // plane deviated from radius
        gp_Ax3 AxPln = Pln.Position();
        gp_Dir NorPln = AxPln.XDirection().Crossed(AxPln.YDirection());
        gp_Dir NorF = NorPln;
        gp_Ax3 AxCyl = Cyl.Position();
        if (Or1 == TopAbs_REVERSED)
        {
            NorF.Reverse();
        }
        gp_Pln PlanOffset = Pln.Translated(Radius * new gp_Vec(NorF));
        // Parallel cylinder
        double ROff = Cyl.Radius();
        if ((Or2 == TopAbs_FORWARD && Cyl.Direct()) || (Or2 == TopAbs_REVERSED && !Cyl.Direct()))
        {
            ROff += Radius;
        }
        else if (Radius < ROff)
        {
            ROff -= Radius;
        }
        else
        {
            // (OCCT_DEBUG trace left out)
            return false;
        }
        // intersection of the parallel plane and of the parallel cylinder.
        gp_Cylinder CylOffset = new(Cyl.Position(), ROff);
        IntAna_QuadQuadGeo LInt = new(PlanOffset, CylOffset, Precision.Angular(), Precision.Confusion());
        gp_Pnt OrSpine = ElCLib.Value(First, Spine);
        gp_Pnt OrFillet;
        gp_Dir DirFillet;
        if (LInt.IsDone())
        {
            DirFillet = LInt.Line(1).Direction();
            gp_Pnt P1 = ElCLib.Value(ElCLib.Parameter(LInt.Line(1), OrSpine), LInt.Line(1));
            if (LInt.NbSolutions() == 2)
            {
                gp_Pnt P2 = ElCLib.Value(ElCLib.Parameter(LInt.Line(2), OrSpine), LInt.Line(2));
                if (P1.SquareDistance(OrSpine) < P2.SquareDistance(OrSpine))
                {
                    OrFillet = P1;
                }
                else
                {
                    OrFillet = P2;
                }
            }
            else
            {
                OrFillet = P1;
            }
        }
        else
        {
            return false;
        }
        // Construction fillet
        if (DirFillet.Dot(Spine.Direction()) < 0.0)
        {
            DirFillet.Reverse();
        }
        double UOnCyl, VOnCyl, UOnPln, VOnPln;
        ElSLib.Parameters(Cyl, OrFillet, out UOnCyl, out VOnCyl);
        double tesp = Precision.Confusion();
        if (UOnCyl < fu - tesp || UOnCyl > lu + tesp)
        {
            UOnCyl = ElCLib.InPeriod(UOnCyl, fu, fu + 2 * Math.PI);
        }
        ElSLib.Parameters(Pln, OrFillet, out UOnPln, out VOnPln);
        gp_Vec XDir, OtherDir;
        if (plandab)
        {
            XDir = new gp_Vec(NorF.Reversed());
            OtherDir = new gp_Vec(OrFillet, ElSLib.Value(UOnCyl, VOnCyl, Cyl));
            OtherDir.Normalize();
        }
        else
        {
            OtherDir = new gp_Vec(NorF.Reversed());
            XDir = new gp_Vec(OrFillet, ElSLib.Value(UOnCyl, VOnCyl, Cyl));
            XDir.Normalize();
        }
        gp_Ax3 AxFil = new(OrFillet, DirFillet, new gp_Dir(XDir));
        gp_Vec aProd = XDir.Crossed(OtherDir);
        if (aProd.Dot(new gp_Vec(DirFillet)) < 0.0)
        {
            AxFil.YReverse();
        }
        Geom_CylindricalSurface Fillet = new(AxFil, Radius);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(Fillet, DStr));
        // FaceInterferences are loaded with pcurves and curves 3D.
        // edge plane-Fillet
        gp_Pnt2d PPln2d = new(UOnPln, VOnPln);
        gp_Dir2d VPln2d = new(DirFillet.Dot(AxPln.XDirection()), DirFillet.Dot(AxPln.YDirection()));
        gp_Lin2d Lin2dPln = new(PPln2d, VPln2d);
        gp_Pnt POnPln = ElSLib.Value(UOnPln, VOnPln, Pln);
        gp_Lin C3d = new(POnPln, DirFillet);
        double UOnFillet, V;
        ElSLib.CylinderParameters(AxFil, Radius, POnPln, out UOnFillet, out V);
        if (UOnFillet > Math.PI)
        {
            UOnFillet = 0.0;
        }
        gp_Lin2d LOnFillet = new(new gp_Pnt2d(UOnFillet, V), gp.DY2d());
        Geom_Line L3d = new(C3d);
        Geom2d_Line LFac = new(Lin2dPln);
        Geom2d_Line LFil = new(LOnFillet);
        ElSLib.CylinderD1(UOnFillet, V, AxFil, Radius, out gp_Pnt P, out gp_Vec deru, out gp_Vec derv);
        gp_Dir NorFil = new(deru.Crossed(derv));
        bool toreverse = (NorFil.Dot(NorPln) <= 0.0);
        // It is checked if the orientation of the cylinder is the same as of the plane.
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs.Reverse(Ofpl);
        }
        else
        {
            Data.ChangeOrientation() = Ofpl;
        }
        TopAbs_Orientation trans;
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
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        else
        {
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        // edge cylinder-Fillet.
        gp_Pnt2d PCyl2d = new(UOnCyl, VOnCyl);
        gp_Dir2d DPC = gp.DY2d();
        if (DirFillet.Dot(AxCyl.Direction()) < 0.0)
        {
            DPC.Reverse();
        }
        gp_Lin2d Lin2dCyl = new(PCyl2d, DPC);
        gp_Pnt POnCyl = ElSLib.Value(UOnCyl, VOnCyl, Cyl);
        C3d = new gp_Lin(POnCyl, DirFillet);
        ElSLib.CylinderParameters(AxFil, Radius, POnCyl, out UOnFillet, out V);
        if (UOnFillet > Math.PI)
        {
            UOnFillet = 0.0;
        }
        LOnFillet = new gp_Lin2d(new gp_Pnt2d(UOnFillet, V), gp.DY2d());
        L3d = new Geom_Line(C3d);
        LFac = new Geom2d_Line(Lin2dCyl);
        LFil = new Geom2d_Line(LOnFillet);
        ElSLib.CylinderD1(UOnFillet, V, AxFil, Radius, out P, out deru, out derv);
        NorFil = new gp_Dir(deru.Crossed(derv));
        ElSLib.CylinderD1(UOnCyl, VOnCyl, AxCyl, Cyl.Radius(), out P, out deru, out derv);
        gp_Dir NorCyl = new(deru.Crossed(derv));
        toreverse = (NorFil.Dot(NorCyl) <= 0.0);
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
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        else
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(L3d, DStr), trans, LFac, LFil);
        }
        return true;
    }

    //=======================================================================
    // function : MakeFillet
    // purpose  : case cylinder/plane or plane/cylinder.
    //=======================================================================

    /// <summary>The fillet along a circle of the cylinder: a torus, or a sphere when the radii are equal.</summary>
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln Pln,
                                              in gp_Cylinder Cyl, double fu, double lu, TopAbs_Orientation Or1,
                                              TopAbs_Orientation Or2, double Radius, in gp_Circ Spine, double First,
                                              TopAbs_Orientation Ofpl, bool plandab)
    {
        // calculation of the fillet (torus or sphere).
        bool c1sphere = false;
        gp_Ax3 PosPl = Pln.Position();
        gp_Dir Dpnat = PosPl.XDirection().Crossed(PosPl.YDirection());
        gp_Dir Dp = Dpnat;
        gp_Dir Df = Dp;
        if (Or1 == TopAbs_REVERSED)
        {
            Dp.Reverse();
        }
        if (Ofpl == TopAbs_REVERSED)
        {
            Df.Reverse();
        }
        gp_Pnt Or = Cyl.Location();
        double u, v;
        ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        gp_Pnt2d c2dPln = new(u, v);
        ElSLib.PlaneD0(u, v, PosPl, out Or);
        gp_Pnt cPln = Or;
        Or.SetCoord(Or.X() + Radius * Dp.X(), Or.Y() + Radius * Dp.Y(), Or.Z() + Radius * Dp.Z());
        gp_Pnt PtSp;
        gp_Vec DSp;
        // Modification for the PtSp found at the wrong side of the sewing edge.
        gp_Pnt PtSp2;
        gp_Vec DSp2;
        double acote = 1e-7;
        ElCLib.D1(First, Spine, out PtSp, out DSp);
        ElSLib.Parameters(Cyl, PtSp, out u, out v);
        if ((Math.Abs(u) < acote) || (Math.Abs(u - (2 * Math.PI)) < acote))
        {
            ElCLib.D1(First + 0.2, Spine, out PtSp2, out DSp2);
            double u2, v2;
            ElSLib.Parameters(Cyl, PtSp2, out u2, out v2);
            if (Math.Abs(u2 - u) > Math.PI)
            {
                u = (2 * Math.PI) - u;
                PtSp = ElSLib.Value(u, v, Cyl);
                double PR;
                PR = ElCLib.Parameter(Spine, PtSp);
                ElCLib.D1(PR, Spine, out PtSp2, out DSp);
            }
        }
        // end of modif
        gp_Dir Dx = new(new gp_Vec(Or, PtSp));
        Dx = Dp.Crossed(Dx.Crossed(Dp));
        gp_Dir Dy = new(DSp);
        ElSLib.D1(u, v, Cyl, out gp_Pnt PtCyl, out gp_Vec Vu, out gp_Vec Vv);
        gp_Dir Dc = new(Vu.Crossed(Vv));
        if (Or2 == TopAbs_REVERSED)
        {
            Dc.Reverse();
        }
        gp_Dir Dz = Dp;
        double Rad, cylrad = Cyl.Radius();
        bool dedans = (Dx.Dot(Dc) <= 0.0);
        if (dedans)
        {
            if (!plandab)
            {
                Dz.Reverse();
            }
            Rad = cylrad - Radius;
            if (Math.Abs(Rad) <= Precision.Confusion())
            {
                c1sphere = true;
            }
            else if (Rad < 0)
            {
                // (OCCT_DEBUG trace left out)
                return false;
            }
        }
        else
        {
            if (plandab)
            {
                Dz.Reverse();
            }
            Rad = cylrad + Radius;
        }
        gp_Ax3 FilAx3 = new(Or, Dz, Dx);
        if (FilAx3.YDirection().Dot(Dy) <= 0.0)
        {
            FilAx3.YReverse();
        }
        if (c1sphere)
        {
            Geom_SphericalSurface gsph = new(FilAx3, Radius);
            Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gsph, DStr));
        }
        else
        {
            Geom_ToroidalSurface gtor = new(FilAx3, Rad, Radius);
            Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gtor, DStr));
        }
        // It is checked if the orientation of the fillet is the same as of faces.
        gp_Pnt P = new(), PP;
        gp_Vec deru, derv;
        P.SetCoord(cPln.X() + Rad * Dx.X(), cPln.Y() + Rad * Dx.Y(), cPln.Z() + Rad * Dx.Z());
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
        gp_Dir norFil;
        if (c1sphere)
        {
            ElSLib.SphereD1(u, v, FilAx3, cylrad, out PP, out deru, out derv);
            norFil = FilAx3.XDirection().Crossed(FilAx3.YDirection());
            if (v < 0.0)
            {
                norFil.Reverse();
            }
        }
        else
        {
            ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
            norFil = new gp_Dir(deru.Crossed(derv));
        }
        gp_Pnt2d p2dFil = new(0.0, v);
        bool toreverse = (norFil.Dot(Df) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }
        // FaceInterferences are loaded with pcurves and curves 3d.
        // The plane face.
        Geom2d_Circle? GCirc2dPln = null;
        Geom_Circle? GCircPln = null;
        gp_Ax2 circAx2 = FilAx3.Ax2();
        if (!c1sphere)
        {
            ElSLib.PlaneParameters(PosPl, P, out u, out v);
            gp_Pnt2d p2dPln = new(u, v);
            gp_Dir2d d2d = new(DSp.Dot(new gp_Vec(PosPl.XDirection())), DSp.Dot(new gp_Vec(PosPl.YDirection())));
            gp_Ax22d ax2dPln = new(c2dPln, new gp_Dir2d(new gp_Vec2d(c2dPln, p2dPln)), d2d);
            gp_Circ2d circ2dPln = new(ax2dPln, Rad);
            GCirc2dPln = new Geom2d_Circle(circ2dPln);
            circAx2.SetLocation(cPln);
            gp_Circ circPln = new(circAx2, Rad);
            GCircPln = new Geom_Circle(circPln);
        }
        else
        {
            ElSLib.PlaneParameters(PosPl, P, out u, out v);
            gp_Pnt2d p2dPln = new(u, v), pbid = new();
            if (plandab)
            {
                Data.Set2dPoints(p2dPln, p2dPln, pbid, pbid);
            }
            else
            {
                Data.Set2dPoints(pbid, pbid, p2dPln, p2dPln);
            }
        }
        gp_Lin2d lin2dFil = new(p2dFil, gp.DX2d());
        Geom2d_Line GLin2dFil1 = new(lin2dFil);
        toreverse = (norFil.Dot(Dpnat) <= 0.0);
        TopAbs_Orientation trans;
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
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GCircPln, DStr), trans,
                                                          GCirc2dPln, GLin2dFil1);
        }
        else
        {
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GCircPln, DStr), trans,
                                                          GCirc2dPln, GLin2dFil1);
        }
        // The cylindrical face.
        P.SetCoord(Or.X() + cylrad * Dx.X(), Or.Y() + cylrad * Dx.Y(), Or.Z() + cylrad * Dx.Z());
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
        p2dFil.SetCoord(u, v);
        if (c1sphere)
        {
            ElSLib.SphereD1(u, v, FilAx3, cylrad, out PP, out deru, out derv);
        }
        else
        {
            ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
        }
        norFil = new gp_Dir(deru.Crossed(derv));
        lin2dFil.SetLocation(p2dFil);
        Geom2d_Line GLin2dFil2 = new(lin2dFil);
        ElSLib.Parameters(Cyl, P, out u, out v);
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
        ElSLib.D1(u, v, Cyl, out PP, out deru, out derv);
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
        circAx2.SetLocation(Or);
        gp_Circ circCyl = new(circAx2, cylrad);
        Geom_Circle GCircCyl = new(circCyl);
        toreverse = (norFil.Dot(norcyl) <= 0.0);
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
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GCircCyl, DStr), trans,
                                                          GLin2dCyl, GLin2dFil2);
        }
        else
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GCircCyl, DStr), trans,
                                                          GLin2dCyl, GLin2dFil2);
        }
        return true;
    }
}
