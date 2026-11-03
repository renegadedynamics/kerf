// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The torus joint ("rotule") between a plane and the edge of two other planes.</summary>
internal static class ChFiKPart_ComputeData_Rotule
{
    //=================================================================================================

    /// <summary>The torus of radii <paramref name="r"/> and <paramref name="r"/> on the plane <paramref name="pl"/>
    /// around the edge of <paramref name="pl1"/> and <paramref name="pl2"/>.</summary>
    internal static bool ChFiKPart_MakeRotule(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D pl,
                                              in Plane3D pl1, in Plane3D pl2, Orientation opl,
                                              Orientation opl1, Orientation opl2, double r,
                                              Orientation ofpl)
    {
        // calcul du tore.
        //---------------
        CoordinateSystem pos = pl.Position();
        Direction3D dpl = pos.XDirection().Crossed(pos.YDirection());
        Direction3D dfpl = dpl;
        Direction3D dplnat = dpl;
        if (opl == Reversed)
        {
            dpl.Reverse();
        }
        if (ofpl == Reversed)
        {
            dfpl.Reverse();
        }
        pos = pl1.Position();
        Direction3D dpl1 = pos.XDirection().Crossed(pos.YDirection());
        if (opl1 == Reversed)
        {
            dpl1.Reverse();
        }
        pos = pl2.Position();
        Direction3D dpl2 = pos.XDirection().Crossed(pos.YDirection());
        if (opl2 == Reversed)
        {
            dpl2.Reverse();
        }
        double alpha = dpl1.Angle(dpl2);
        QuadQuadGeo LInt = new(pl1, pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D ptor = new(), pcirc;
        if (LInt.IsDone)
        {
            pcirc = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(LInt.Line(1), pl.Location()), LInt.Line(1));
            ptor.SetCoord(pcirc.X() + r * dpl.X(), pcirc.Y() + r * dpl.Y(), pcirc.Z() + r * dpl.Z());
        }
        else
        {
            return false;
        }
        CoordinateSystem ppos = new(ptor, dpl.Reversed(), dpl1);
        if (ppos.YDirection().Dot(dpl2) < 0.0)
        {
            ppos.YReverse();
        }
        ToroidalSurface gtor = new(ppos, r, r);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gtor, DStr));
        // on compare l orientation du tore a celle de la face en bout.
        //------------------------------------------------------------
        global::Kerf.Engine.Geometry.ElSLib.TorusD1(0.0, Math.PI / 2, ppos, r, r, out Point3D pp, out Vector3D du, out Vector3D dv);
        Direction3D drot = new(du.Crossed(dv));
        bool reversecur = (drot.Dot(dplnat) <= 0.0);
        bool reversefil = (drot.Dot(dfpl) <= 0.0);
        if (reversefil)
        {
            Data.ChangeOrientation() = Reversed;
        }
        else
        {
            Data.ChangeOrientation() = Forward;
        }
        // on charge les FaceInterferences avec les pcurves et courbes 3d.
        //-----------------------------------------------------------------
        // du cote du plan
        //---------------
        Frame3D circAx2 = ppos.Ax2();
        circAx2.SetLocation(pcirc);
        CircleCurve3D GC = new(circAx2, r);
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pcirc, out u, out v);
        Point2D p2dcirc = new(u, v);
        Direction2D dx2d = new(dpl1.Dot(pl.Position().XDirection()), dpl1.Dot(pl.Position().YDirection()));
        Direction2D dy2d = new(ppos.YDirection().Dot(pl.Position().XDirection()),
                            ppos.YDirection().Dot(pl.Position().YDirection()));
        Frame2D circ2dax = new(p2dcirc, dx2d, dy2d);
        CircleCurve2D GC2d = new(circ2dax, r);
        Point2D p2dlin = new(0.0, Math.PI / 2);
        LineCurve2D GL2d = new(p2dlin, Constants.DX2d());
        Orientation trans = Reversed;
        if (reversecur)
        {
            trans = Forward;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GC, DStr), trans, GC2d, GL2d);
        // du cote pointu
        //--------------
        Curve3D? bid = null;
        Curve2D? bid2d = null;
        p2dlin.SetCoord(0.0, Math.PI);
        LineCurve2D GL2dcoin = new(p2dlin, Constants.DX2d());
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(bid, DStr), trans, bid2d, GL2dcoin);
        // et les points
        //-------------
        Data.ChangeVertexFirstOnS1().SetPoint(pp);
        global::Kerf.Engine.Geometry.ElSLib.TorusD0(alpha, Math.PI / 2, ppos, r, r, out pp);
        Data.ChangeVertexLastOnS1().SetPoint(pp);
        Data.ChangeInterferenceOnS1().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS1().SetLastParameter(alpha);
        Data.ChangeInterferenceOnS2().SetFirstParameter(0.0);
        Data.ChangeInterferenceOnS2().SetLastParameter(alpha);
        return true;
    }
}
