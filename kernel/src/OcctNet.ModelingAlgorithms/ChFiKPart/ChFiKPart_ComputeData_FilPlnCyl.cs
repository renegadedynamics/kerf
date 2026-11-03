// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCyl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
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
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D Pln,
                                              in Cylinder Cyl, double fu, double lu, TopAbs_Orientation Or1,
                                              TopAbs_Orientation Or2, double Radius, in Line3D Spine, double First,
                                              TopAbs_Orientation Ofpl, bool plandab)
    {
        // calculate the cylinder fillet.
        // plane deviated from radius
        CoordinateSystem AxPln = Pln.Position();
        Direction3D NorPln = AxPln.XDirection().Crossed(AxPln.YDirection());
        Direction3D NorF = NorPln;
        CoordinateSystem AxCyl = Cyl.Position();
        if (Or1 == TopAbs_REVERSED)
        {
            NorF.Reverse();
        }
        Plane3D PlanOffset = Pln.Translated(Radius * new Vector3D(NorF));
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
        Cylinder CylOffset = new(Cyl.Position(), ROff);
        IntAna_QuadQuadGeo LInt = new(PlanOffset, CylOffset, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D OrSpine = global::Kerf.Engine.Geometry.ElCLib.Value(First, Spine);
        Point3D OrFillet;
        Direction3D DirFillet;
        if (LInt.IsDone())
        {
            DirFillet = LInt.Line(1).Direction();
            Point3D P1 = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(LInt.Line(1), OrSpine), LInt.Line(1));
            if (LInt.NbSolutions() == 2)
            {
                Point3D P2 = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(LInt.Line(2), OrSpine), LInt.Line(2));
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
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, OrFillet, out UOnCyl, out VOnCyl);
        double tesp = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (UOnCyl < fu - tesp || UOnCyl > lu + tesp)
        {
            UOnCyl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(UOnCyl, fu, fu + 2 * Math.PI);
        }
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Pln, OrFillet, out UOnPln, out VOnPln);
        Vector3D XDir, OtherDir;
        if (plandab)
        {
            XDir = new Vector3D(NorF.Reversed());
            OtherDir = new Vector3D(OrFillet, global::Kerf.Engine.Geometry.ElSLib.Value(UOnCyl, VOnCyl, Cyl));
            OtherDir.Normalize();
        }
        else
        {
            OtherDir = new Vector3D(NorF.Reversed());
            XDir = new Vector3D(OrFillet, global::Kerf.Engine.Geometry.ElSLib.Value(UOnCyl, VOnCyl, Cyl));
            XDir.Normalize();
        }
        CoordinateSystem AxFil = new(OrFillet, DirFillet, new Direction3D(XDir));
        Vector3D aProd = XDir.Crossed(OtherDir);
        if (aProd.Dot(new Vector3D(DirFillet)) < 0.0)
        {
            AxFil.YReverse();
        }
        Geom_CylindricalSurface Fillet = new(AxFil, Radius);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(Fillet, DStr));
        // FaceInterferences are loaded with pcurves and curves 3D.
        // edge plane-Fillet
        Point2D PPln2d = new(UOnPln, VOnPln);
        Direction2D VPln2d = new(DirFillet.Dot(AxPln.XDirection()), DirFillet.Dot(AxPln.YDirection()));
        Line2D Lin2dPln = new(PPln2d, VPln2d);
        Point3D POnPln = global::Kerf.Engine.Geometry.ElSLib.Value(UOnPln, VOnPln, Pln);
        Line3D C3d = new(POnPln, DirFillet);
        double UOnFillet, V;
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(AxFil, Radius, POnPln, out UOnFillet, out V);
        if (UOnFillet > Math.PI)
        {
            UOnFillet = 0.0;
        }
        Line2D LOnFillet = new(new Point2D(UOnFillet, V), Constants.DY2d());
        Geom_Line L3d = new(C3d);
        Geom2d_Line LFac = new(Lin2dPln);
        Geom2d_Line LFil = new(LOnFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(UOnFillet, V, AxFil, Radius, out Point3D P, out Vector3D deru, out Vector3D derv);
        Direction3D NorFil = new(deru.Crossed(derv));
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
        Point2D PCyl2d = new(UOnCyl, VOnCyl);
        Direction2D DPC = Constants.DY2d();
        if (DirFillet.Dot(AxCyl.Direction()) < 0.0)
        {
            DPC.Reverse();
        }
        Line2D Lin2dCyl = new(PCyl2d, DPC);
        Point3D POnCyl = global::Kerf.Engine.Geometry.ElSLib.Value(UOnCyl, VOnCyl, Cyl);
        C3d = new Line3D(POnCyl, DirFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(AxFil, Radius, POnCyl, out UOnFillet, out V);
        if (UOnFillet > Math.PI)
        {
            UOnFillet = 0.0;
        }
        LOnFillet = new Line2D(new Point2D(UOnFillet, V), Constants.DY2d());
        L3d = new Geom_Line(C3d);
        LFac = new Geom2d_Line(Lin2dCyl);
        LFil = new Geom2d_Line(LOnFillet);
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(UOnFillet, V, AxFil, Radius, out P, out deru, out derv);
        NorFil = new Direction3D(deru.Crossed(derv));
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(UOnCyl, VOnCyl, AxCyl, Cyl.Radius(), out P, out deru, out derv);
        Direction3D NorCyl = new(deru.Crossed(derv));
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
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D Pln,
                                              in Cylinder Cyl, double fu, double lu, TopAbs_Orientation Or1,
                                              TopAbs_Orientation Or2, double Radius, in Circle3D Spine, double First,
                                              TopAbs_Orientation Ofpl, bool plandab)
    {
        // calculation of the fillet (torus or sphere).
        bool c1sphere = false;
        CoordinateSystem PosPl = Pln.Position();
        Direction3D Dpnat = PosPl.XDirection().Crossed(PosPl.YDirection());
        Direction3D Dp = Dpnat;
        Direction3D Df = Dp;
        if (Or1 == TopAbs_REVERSED)
        {
            Dp.Reverse();
        }
        if (Ofpl == TopAbs_REVERSED)
        {
            Df.Reverse();
        }
        Point3D Or = Cyl.Location();
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        Point2D c2dPln = new(u, v);
        global::Kerf.Engine.Geometry.ElSLib.PlaneD0(u, v, PosPl, out Or);
        Point3D cPln = Or;
        Or.SetCoord(Or.X() + Radius * Dp.X(), Or.Y() + Radius * Dp.Y(), Or.Z() + Radius * Dp.Z());
        Point3D PtSp;
        Vector3D DSp;
        // Modification for the PtSp found at the wrong side of the sewing edge.
        Point3D PtSp2;
        Vector3D DSp2;
        double acote = 1e-7;
        global::Kerf.Engine.Geometry.ElCLib.D1(First, Spine, out PtSp, out DSp);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, PtSp, out u, out v);
        if ((Math.Abs(u) < acote) || (Math.Abs(u - (2 * Math.PI)) < acote))
        {
            global::Kerf.Engine.Geometry.ElCLib.D1(First + 0.2, Spine, out PtSp2, out DSp2);
            double u2, v2;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, PtSp2, out u2, out v2);
            if (Math.Abs(u2 - u) > Math.PI)
            {
                u = (2 * Math.PI) - u;
                PtSp = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, Cyl);
                double PR;
                PR = global::Kerf.Engine.Geometry.ElCLib.Parameter(Spine, PtSp);
                global::Kerf.Engine.Geometry.ElCLib.D1(PR, Spine, out PtSp2, out DSp);
            }
        }
        // end of modif
        Direction3D Dx = new(new Vector3D(Or, PtSp));
        Dx = Dp.Crossed(Dx.Crossed(Dp));
        Direction3D Dy = new(DSp);
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, Cyl, out Point3D PtCyl, out Vector3D Vu, out Vector3D Vv);
        Direction3D Dc = new(Vu.Crossed(Vv));
        if (Or2 == TopAbs_REVERSED)
        {
            Dc.Reverse();
        }
        Direction3D Dz = Dp;
        double Rad, cylrad = Cyl.Radius();
        bool dedans = (Dx.Dot(Dc) <= 0.0);
        if (dedans)
        {
            if (!plandab)
            {
                Dz.Reverse();
            }
            Rad = cylrad - Radius;
            if (Math.Abs(Rad) <= global::Kerf.Engine.Foundation.Precision.Confusion())
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
        CoordinateSystem FilAx3 = new(Or, Dz, Dx);
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
        Point3D P = new(), PP;
        Vector3D deru, derv;
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
        Direction3D norFil;
        if (c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, FilAx3, cylrad, out PP, out deru, out derv);
            norFil = FilAx3.XDirection().Crossed(FilAx3.YDirection());
            if (v < 0.0)
            {
                norFil.Reverse();
            }
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
            norFil = new Direction3D(deru.Crossed(derv));
        }
        Point2D p2dFil = new(0.0, v);
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
        Frame3D circAx2 = FilAx3.Ax2();
        if (!c1sphere)
        {
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, P, out u, out v);
            Point2D p2dPln = new(u, v);
            Direction2D d2d = new(DSp.Dot(new Vector3D(PosPl.XDirection())), DSp.Dot(new Vector3D(PosPl.YDirection())));
            Frame2D ax2dPln = new(c2dPln, new Direction2D(new Vector2D(c2dPln, p2dPln)), d2d);
            Circle2D circ2dPln = new(ax2dPln, Rad);
            GCirc2dPln = new Geom2d_Circle(circ2dPln);
            circAx2.SetLocation(cPln);
            Circle3D circPln = new(circAx2, Rad);
            GCircPln = new Geom_Circle(circPln);
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PosPl, P, out u, out v);
            Point2D p2dPln = new(u, v), pbid = new();
            if (plandab)
            {
                Data.Set2dPoints(p2dPln, p2dPln, pbid, pbid);
            }
            else
            {
                Data.Set2dPoints(pbid, pbid, p2dPln, p2dPln);
            }
        }
        Line2D lin2dFil = new(p2dFil, Constants.DX2d());
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
            global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, FilAx3, cylrad, out PP, out deru, out derv);
        }
        else
        {
            global::Kerf.Engine.Geometry.ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
        }
        norFil = new Direction3D(deru.Crossed(derv));
        lin2dFil.SetLocation(p2dFil);
        Geom2d_Line GLin2dFil2 = new(lin2dFil);
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cyl, P, out u, out v);
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
        global::Kerf.Engine.Geometry.ElSLib.D1(u, v, Cyl, out PP, out deru, out derv);
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
        Geom2d_Line GLin2dCyl = new(lin2dCyl);
        circAx2.SetLocation(Or);
        Circle3D circCyl = new(circAx2, cylrad);
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
