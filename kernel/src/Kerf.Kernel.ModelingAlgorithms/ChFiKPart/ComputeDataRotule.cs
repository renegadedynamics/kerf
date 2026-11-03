// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_Rotule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The torus joint ("rotule") between a plane and the edge of two other planes.</summary>
internal static class ComputeDataRotule
{
    //=================================================================================================

    /// <summary>The torus of radii <paramref name="r"/> and <paramref name="r"/> on the plane <paramref name="pl"/>
    /// around the edge of <paramref name="pl1"/> and <paramref name="pl2"/>.</summary>
    internal static bool ChFiKPart_MakeRotule(DsDataStructure dStr, SurfData data, in Plane3D pl,
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
            dpl = dpl.Reversed();
        }
        if (ofpl == Reversed)
        {
            dfpl = dfpl.Reversed();
        }
        pos = pl1.Position();
        Direction3D dpl1 = pos.XDirection().Crossed(pos.YDirection());
        if (opl1 == Reversed)
        {
            dpl1 = dpl1.Reversed();
        }
        pos = pl2.Position();
        Direction3D dpl2 = pos.XDirection().Crossed(pos.YDirection());
        if (opl2 == Reversed)
        {
            dpl2 = dpl2.Reversed();
        }
        double alpha = dpl1.Angle(dpl2);
        QuadQuadGeo lInt = new(pl1, pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D ptor = new(), pcirc;
        if (lInt.IsDone)
        {
            pcirc = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(lInt.Line(1), pl.Location()), lInt.Line(1));
            ptor = ptor.WithCoord(pcirc.X() + r * dpl.X(), pcirc.Y() + r * dpl.Y(), pcirc.Z() + r * dpl.Z());
        }
        else
        {
            return false;
        }
        CoordinateSystem ppos = new(ptor, dpl.Reversed(), dpl1);
        if (ppos.YDirection().Dot(dpl2) < 0.0)
        {
            ppos = ppos.YReversed();
        }
        ToroidalSurface gtor = new(ppos, r, r);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gtor, dStr));
        // on compare l orientation du tore a celle de la face en bout.
        //------------------------------------------------------------
        global::Kerf.Engine.Geometry.ElSLib.TorusD1(0.0, Math.PI / 2, ppos, r, r, out Point3D pp, out Vector3D du, out Vector3D dv);
        Direction3D drot = new(du.Crossed(dv));
        bool reversecur = (drot.Dot(dplnat) <= 0.0);
        bool reversefil = (drot.Dot(dfpl) <= 0.0);
        if (reversefil)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }
        // on charge les FaceInterferences avec les pcurves et courbes 3d.
        //-----------------------------------------------------------------
        // du cote du plan
        //---------------
        Frame3D circAx2 = ppos.Ax2();
        circAx2 = circAx2.WithLocation(pcirc);
        CircleCurve3D gc = new(circAx2, r);
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pcirc, out u, out v);
        Point2D p2dcirc = new(u, v);
        Direction2D dx2d = new(dpl1.Dot(pl.Position().XDirection()), dpl1.Dot(pl.Position().YDirection()));
        Direction2D dy2d = new(ppos.YDirection().Dot(pl.Position().XDirection()),
                            ppos.YDirection().Dot(pl.Position().YDirection()));
        Frame2D circ2dax = new(p2dcirc, dx2d, dy2d);
        CircleCurve2D gc2d = new(circ2dax, r);
        Point2D p2dlin = new(0.0, Math.PI / 2);
        LineCurve2D gl2d = new(p2dlin, Constants.DX2d());
        Orientation trans = Reversed;
        if (reversecur)
        {
            trans = Forward;
        }
        data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gc, dStr), trans, gc2d, gl2d);
        // du cote pointu
        //--------------
        Curve3D? bid = null;
        Curve2D? bid2d = null;
        p2dlin = p2dlin.WithCoord(0.0, Math.PI);
        LineCurve2D gl2dcoin = new(p2dlin, Constants.DX2d());
        data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(bid, dStr), trans, bid2d, gl2dcoin);
        // et les points
        //-------------
        data.ChangeVertexFirstOnS1().SetPoint(pp);
        global::Kerf.Engine.Geometry.ElSLib.TorusD0(alpha, Math.PI / 2, ppos, r, r, out pp);
        data.ChangeVertexLastOnS1().SetPoint(pp);
        data.ChangeInterferenceOnS1().SetFirstParameter(0.0);
        data.ChangeInterferenceOnS1().SetLastParameter(alpha);
        data.ChangeInterferenceOnS2().SetFirstParameter(0.0);
        data.ChangeInterferenceOnS2().SetLastParameter(alpha);
        return true;
    }
}
