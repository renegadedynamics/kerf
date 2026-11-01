// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The torus joint ("rotule") between a plane and the edge of two other planes.</summary>
internal static class ChFiKPart_ComputeData_Rotule
{
    //=================================================================================================

    /// <summary>The torus of radii <paramref name="r"/> and <paramref name="r"/> on the plane <paramref name="pl"/>
    /// around the edge of <paramref name="pl1"/> and <paramref name="pl2"/>.</summary>
    internal static bool ChFiKPart_MakeRotule(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln pl,
                                              in gp_Pln pl1, in gp_Pln pl2, TopAbs_Orientation opl,
                                              TopAbs_Orientation opl1, TopAbs_Orientation opl2, double r,
                                              TopAbs_Orientation ofpl)
    {
        // calcul du tore.
        //---------------
        gp_Ax3 pos = pl.Position();
        gp_Dir dpl = pos.XDirection().Crossed(pos.YDirection());
        gp_Dir dfpl = dpl;
        gp_Dir dplnat = dpl;
        if (opl == TopAbs_REVERSED)
        {
            dpl.Reverse();
        }
        if (ofpl == TopAbs_REVERSED)
        {
            dfpl.Reverse();
        }
        pos = pl1.Position();
        gp_Dir dpl1 = pos.XDirection().Crossed(pos.YDirection());
        if (opl1 == TopAbs_REVERSED)
        {
            dpl1.Reverse();
        }
        pos = pl2.Position();
        gp_Dir dpl2 = pos.XDirection().Crossed(pos.YDirection());
        if (opl2 == TopAbs_REVERSED)
        {
            dpl2.Reverse();
        }
        double alpha = dpl1.Angle(dpl2);
        IntAna_QuadQuadGeo LInt = new(pl1, pl2, Precision.Angular(), Precision.Confusion());
        gp_Pnt ptor = new(), pcirc;
        if (LInt.IsDone())
        {
            pcirc = ElCLib.Value(ElCLib.Parameter(LInt.Line(1), pl.Location()), LInt.Line(1));
            ptor.SetCoord(pcirc.X() + r * dpl.X(), pcirc.Y() + r * dpl.Y(), pcirc.Z() + r * dpl.Z());
        }
        else
        {
            return false;
        }
        gp_Ax3 ppos = new(ptor, dpl.Reversed(), dpl1);
        if (ppos.YDirection().Dot(dpl2) < 0.0)
        {
            ppos.YReverse();
        }
        Geom_ToroidalSurface gtor = new(ppos, r, r);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gtor, DStr));
        // on compare l orientation du tore a celle de la face en bout.
        //------------------------------------------------------------
        ElSLib.TorusD1(0.0, Math.PI / 2, ppos, r, r, out gp_Pnt pp, out gp_Vec du, out gp_Vec dv);
        gp_Dir drot = new(du.Crossed(dv));
        bool reversecur = (drot.Dot(dplnat) <= 0.0);
        bool reversefil = (drot.Dot(dfpl) <= 0.0);
        if (reversefil)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }
        // on charge les FaceInterferences avec les pcurves et courbes 3d.
        //-----------------------------------------------------------------
        // du cote du plan
        //---------------
        gp_Ax2 circAx2 = ppos.Ax2();
        circAx2.SetLocation(pcirc);
        Geom_Circle GC = new(circAx2, r);
        double u, v;
        ElSLib.Parameters(pl, pcirc, out u, out v);
        gp_Pnt2d p2dcirc = new(u, v);
        gp_Dir2d dx2d = new(dpl1.Dot(pl.Position().XDirection()), dpl1.Dot(pl.Position().YDirection()));
        gp_Dir2d dy2d = new(ppos.YDirection().Dot(pl.Position().XDirection()),
                            ppos.YDirection().Dot(pl.Position().YDirection()));
        gp_Ax22d circ2dax = new(p2dcirc, dx2d, dy2d);
        Geom2d_Circle GC2d = new(circ2dax, r);
        gp_Pnt2d p2dlin = new(0.0, Math.PI / 2);
        Geom2d_Line GL2d = new(p2dlin, gp.DX2d());
        TopAbs_Orientation trans = TopAbs_REVERSED;
        if (reversecur)
        {
            trans = TopAbs_FORWARD;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GC, DStr), trans, GC2d, GL2d);
        // du cote pointu
        //--------------
        Geom_Curve? bid = null;
        Geom2d_Curve? bid2d = null;
        p2dlin.SetCoord(0.0, Math.PI);
        Geom2d_Line GL2dcoin = new(p2dlin, gp.DX2d());
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(bid, DStr), trans, bid2d, GL2dcoin);
        // et les points
        //-------------
        Data.ChangeVertexFirstOnS1().SetPoint(pp);
        ElSLib.TorusD0(alpha, Math.PI / 2, ppos, r, r, out pp);
        Data.ChangeVertexLastOnS1().SetPoint(pp);
        Data.ChangeInterferenceOnS1().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS1().SetLastParameter(alpha);
        Data.ChangeInterferenceOnS2().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS2().SetLastParameter(alpha);
        return true;
    }
}
