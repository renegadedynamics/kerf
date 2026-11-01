// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCon.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnCon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The constant radius fillet between a plane and a cone along a circle: a torus or a sphere.</summary>
internal static class ChFiKPart_ComputeData_FilPlnCon
{
    //=======================================================================
    // function : MakeFillet
    // purpose  : case cone/plane or plane/cone.
    //=======================================================================

    /// <summary>The fillet between a plane and a cone orthogonal to it, along their circle.</summary>
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln Pln,
                                              in gp_Cone Con, double fu, double lu, TopAbs_Orientation Or1,
                                              TopAbs_Orientation Or2, double Radius, in gp_Circ Spine, double First,
                                              TopAbs_Orientation Ofpl, bool plandab)
    {
        // calculate the fillet (torus or sphere).
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
        gp_Pnt Or = Con.Location();
        double u, v;
        ElSLib.PlaneParameters(PosPl, Or, out u, out v);
        gp_Pnt2d c2dPln = new(u, v);
        ElSLib.PlaneD0(u, v, PosPl, out Or);
        gp_Pnt cPln = Or;
        Or.SetCoord(Or.X() + Radius * Dp.X(), Or.Y() + Radius * Dp.Y(), Or.Z() + Radius * Dp.Z());
        ElCLib.D1(First, Spine, out gp_Pnt PtSp, out gp_Vec DSp);
        IntAna_QuadQuadGeo CInt = new(Pln, Con, Precision.Angular(), Precision.Confusion());
        gp_Pnt Pv;
        if (CInt.IsDone())
        {
            // The origin of the fillet is set at the start point on the
            // guideline.
            Pv = ElCLib.Value(ElCLib.Parameter(CInt.Circle(1), PtSp), CInt.Circle(1));
        }
        else
        {
            return false;
        }
        gp_Dir Dx = new(new gp_Vec(cPln, Pv));
        gp_Dir Dy = new(DSp);
        ElSLib.Parameters(Con, Pv, out u, out v);
        ElSLib.D1(u, v, Con, out gp_Pnt PtCon, out gp_Vec Vu, out gp_Vec Vv);
        gp_Dir Dc = new(Vu.Crossed(Vv));
        if (Or2 == TopAbs_REVERSED)
        {
            Dc.Reverse();
        }
        gp_Dir Dz = Dp;
        gp_Pnt pp = new(Pv.X() + Dc.X(), Pv.Y() + Dc.Y(), Pv.Z() + Dc.Z());
        ElSLib.PlaneParameters(PosPl, pp, out u, out v);
        ElSLib.PlaneD0(u, v, PosPl, out pp);
        gp_Dir ddp = new(new gp_Vec(Pv, pp));
        ElSLib.Parameters(Con, Pv, out u, out v);
        ElSLib.D1(u, v, Con, out pp, out gp_Vec dcu, out gp_Vec dcv);
        gp_Dir ddc = new(dcv);
        if (ddc.Dot(Dp) < 0.0)
        {
            ddc.Reverse();
        }
        double Ang = ddp.Angle(ddc);
        double Rabio = Radius / Math.Tan(Ang / 2);
        double Maxrad = cPln.Distance(Pv);
        double Rad;
        bool dedans = Dx.Dot(Dc) <= 0.0;
        if (dedans)
        {
            if (!plandab)
            {
                Dz.Reverse();
            }
            Rad = Maxrad - Rabio;
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
            Rad = Maxrad + Rabio;
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
        // It is checked if the orientation of the fillet is the same
        // as of the faces.
        gp_Pnt P = new(), PP;
        gp_Vec deru, derv;
        P.SetCoord(cPln.X() + Rad * Dx.X(), cPln.Y() + Rad * Dx.Y(), cPln.Z() + Rad * Dx.Z());
        if (c1sphere)
        {
            ElSLib.SphereParameters(FilAx3, Rad, P, out u, out v);
            ElSLib.SphereD1(u, v, FilAx3, Rad, out PP, out deru, out derv);
        }
        else
        {
            ElSLib.TorusParameters(FilAx3, Rad, Radius, P, out u, out v);
            ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
            if (!plandab && Ang < Math.PI / 2 && dedans)
            {
                v = v + 2 * Math.PI;
            }
        }
        gp_Pnt2d p2dFil = new(0.0, v);
        gp_Dir norFil = new(deru.Crossed(derv));
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
        // ---------------------------------------------------------------
        // The plane face.
        // --------------
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
        // The conic face.
        // ----------------
        P.SetCoord(Pv.X() + Rabio * ddc.X(), Pv.Y() + Rabio * ddc.Y(), Pv.Z() + Rabio * ddc.Z());
        if (c1sphere)
        {
            ElSLib.SphereParameters(FilAx3, Radius, P, out u, out v);
            ElSLib.SphereD1(u, v, FilAx3, Radius, out PP, out deru, out derv);
        }
        else
        {
            ElSLib.TorusParameters(FilAx3, Rad, Radius, P, out u, out v);
            ElSLib.TorusD1(u, v, FilAx3, Rad, Radius, out PP, out deru, out derv);
            if (plandab && Ang < Math.PI / 2 && dedans)
            {
                v = v + 2 * Math.PI;
            }
        }
        norFil = new gp_Dir(deru.Crossed(derv));
        p2dFil.SetCoord(0.0, v);
        lin2dFil.SetLocation(p2dFil);
        Geom2d_Line GLin2dFil2 = new(lin2dFil);
        ElSLib.Parameters(Con, P, out u, out v);
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
            u = ElCLib.InPeriod(u, fu, fu + 2 * Math.PI);
        }
        ElSLib.D1(u, v, Con, out PP, out deru, out derv);
        gp_Dir norCon = new(deru.Crossed(derv));
        gp_Dir2d d2dCon = gp.DX2d();
        if (deru.Dot(new gp_Vec(Dy)) < 0.0)
        {
            d2dCon.Reverse();
            if (careaboutsens && Math.Abs(fu - u) < tol)
            {
                u = lu;
            }
        }
        else if (careaboutsens && Math.Abs(lu - u) < tol)
        {
            u = fu;
        }
        gp_Pnt2d p2dCon = new(u, v);
        gp_Lin2d lin2dCon = new(p2dCon, d2dCon);
        Geom2d_Line GLin2dCon = new(lin2dCon);
        double scal = new gp_Vec(Dp).Dot(new gp_Vec(Pv, P));
        PP.SetCoord(cPln.X() + scal * Dp.X(), cPln.Y() + scal * Dp.Y(), cPln.Z() + scal * Dp.Z());
        circAx2.SetLocation(PP);
        gp_Circ circCon = new(circAx2, P.Distance(PP));
        Geom_Circle GCircCon = new(circCon);
        toreverse = (norFil.Dot(norCon) <= 0.0);
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
            Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GCircCon, DStr), trans,
                                                          GLin2dCon, GLin2dFil2);
        }
        else
        {
            Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GCircCon, DStr), trans,
                                                          GLin2dCon, GLin2dFil2);
        }
        return true;
    }
}
