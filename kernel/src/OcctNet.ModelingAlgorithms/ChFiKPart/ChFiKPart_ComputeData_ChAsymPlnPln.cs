// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnPln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The chamfer by a distance and an angle between two planes: a plane.</summary>
internal static class ChFiKPart_ComputeData_ChAsymPlnPln
{
    //=======================================================================
    // function : MakeChAsym
    // Purpose  : Compute the chamfer in the particular case plane/plane.
    //           Compute the SurfData <Data> of the chamfer on the <Spine>
    //           between the plane <Pl1> and the plane <Pl2>, with distances
    //           <Dis> and Angle on <Pl1> .
    //           <First> is the parameter of the start point on the <Spine>
    //           <Or1> and <Or2> are the orientations of the plane <Pl1> and
    //           <Pl2>, and <Of1> the orientation of the face build on the
    //           plane <Pl1>.
    // Out      : True if the chamfer has been computed
    //           False else
    //=======================================================================

    /// <summary>The chamfer plane between <paramref name="Pl1"/> and <paramref name="Pl2"/> at the distance
    /// <paramref name="Dis"/> on the first (or the second, by <paramref name="DisOnP1"/>) and the angle
    /// <paramref name="Angle"/>.</summary>
    internal static bool ChFiKPart_MakeChAsym(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln Pl1,
                                              in gp_Pln Pl2, TopAbs_Orientation Or1, TopAbs_Orientation Or2,
                                              double Dis, double Angle, in gp_Lin Spine, double First,
                                              TopAbs_Orientation Of1, bool DisOnP1)
    {
        // Creation of the plane which carry the chamfer
        // compute the normals to the planes Pl1 and Pl2
        gp_Ax3 Pos1 = Pl1.Position();
        gp_Dir D1 = Pos1.XDirection().Crossed(Pos1.YDirection());
        if (Or1 == TopAbs_REVERSED)
        {
            D1.Reverse();
        }
        gp_Ax3 Pos2 = Pl2.Position();
        gp_Dir D2 = Pos2.XDirection().Crossed(Pos2.YDirection());
        if (Or2 == TopAbs_REVERSED)
        {
            D2.Reverse();
        }
        // compute the intersection line of Pl1 and Pl2
        IntAna_QuadQuadGeo LInt = new(Pl1, Pl2, Precision.Angular(), Precision.Confusion());
        gp_Pnt P;
        double Fint;
        if (LInt.IsDone())
        {
            Fint = ElCLib.Parameter(LInt.Line(1), ElCLib.Value(First, Spine));
            P = ElCLib.Value(Fint, LInt.Line(1));
        }
        else
        {
            return false;
        }
        gp_Dir LinAx1 = Spine.Direction();
        gp_Dir VecTransl1 = LinAx1.Crossed(D1);
        if (VecTransl1.Dot(D2) < 0.0)
        {
            VecTransl1.Reverse();
        }
        gp_Dir VecTransl2 = LinAx1.Crossed(D2);
        if (VecTransl2.Dot(D1) < 0.0)
        {
            VecTransl2.Reverse();
        }
        double cosP, sinP, dis1, dis2;
        cosP = VecTransl1.Dot(VecTransl2);
        sinP = Math.Sqrt(1.0 - cosP * cosP);
        if (DisOnP1)
        {
            dis1 = Dis;
            dis2 = Dis / (cosP + sinP / Math.Tan(Angle));
        }
        else
        {
            dis1 = Dis / (cosP + sinP / Math.Tan(Angle));
            dis2 = Dis;
        }
        // Compute a point on the plane Pl1 and on the chamfer
        gp_Pnt P1 = new(P.X() + dis1 * VecTransl1.X(), P.Y() + dis1 * VecTransl1.Y(), P.Z() + dis1 * VecTransl1.Z());
        // Point on the plane Pl2 and on the chamfer
        gp_Pnt P2 = new(P.X() + dis2 * VecTransl2.X(), P.Y() + dis2 * VecTransl2.Y(), P.Z() + dis2 * VecTransl2.Z());
        // the middle point of P1 P2 is the origin of the chamfer
        gp_Pnt Po = new((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0, (P1.Z() + P2.Z()) / 2.0);
        // compute a second point on the plane Pl2
        gp_Pnt Pp = ElCLib.Value(Fint + 10.0, LInt.Line(1));
        gp_Pnt P22 = new(Pp.X() + dis2 * VecTransl2.X(), Pp.Y() + dis2 * VecTransl2.Y(),
                         Pp.Z() + dis2 * VecTransl2.Z());
        // Compute the normal vector <AxisPlan> to the chamfer's plane
        gp_Dir V1 = new(P2.X() - P1.X(), P2.Y() - P1.Y(), P2.Z() - P1.Z());
        gp_Dir V2 = new(P22.X() - P1.X(), P22.Y() - P1.Y(), P22.Z() - P1.Z());
        gp_Dir AxisPlan = V1.Crossed(V2);
        gp_Dir xdir = LinAx1; // u axis
        gp_Ax3 PlanAx3 = new(Po, AxisPlan, xdir);
        if (PlanAx3.YDirection().Dot(D2) >= 0.0)
        {
            PlanAx3.YReverse();
        }
        Geom_Plane gpl = new(PlanAx3);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gpl, DStr));
        // About the orientation of the chamfer plane
        // Compute the normal to the face 1
        gp_Dir norpl = Pos1.XDirection().Crossed(Pos1.YDirection());
        gp_Dir norface1 = norpl;
        if (Of1 == TopAbs_REVERSED)
        {
            norface1.Reverse();
        }
        // Compute the orientation of the chamfer plane
        gp_Dir norplch = gpl.Pln().Position().XDirection().Crossed(gpl.Pln().Position().YDirection());
        gp_Dir DirCh12 = new(new gp_Vec(P1, P2));
        bool toreverse = (norplch.Dot(norface1) <= 0.0);
        if (VecTransl1.Dot(DirCh12) > 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }
        // Loading of the FaceInterferences with pcurves & 3d curves.
        // case face 1
        gp_Lin linPln = new(P1, xdir);
        Geom_Line GLinPln1 = new(linPln);
        double u, v;
        ElSLib.PlaneParameters(Pos1, P1, out u, out v);
        gp_Pnt2d p2dPln = new(u, v);
        gp_Dir2d dir2dPln = new(xdir.Dot(Pos1.XDirection()), xdir.Dot(Pos1.YDirection()));
        gp_Lin2d lin2dPln = new(p2dPln, dir2dPln);
        Geom2d_Line GLin2dPln1 = new(lin2dPln);
        ElSLib.PlaneParameters(PlanAx3, P1, out u, out v);
        p2dPln.SetCoord(u, v);
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(gp.DX2d());
        Geom2d_Line GLin2dPlnCh1 = new(lin2dPln);
        TopAbs_Orientation trans;
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (VecTransl1.Dot(DirCh12) > 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            trans = TopAbs_FORWARD;
        }
        else
        {
            trans = TopAbs_REVERSED;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln1, DStr), trans, GLin2dPln1,
                                                      GLin2dPlnCh1);
        // case face 2
        linPln.SetLocation(P2);
        Geom_Line GLinPln2 = new(linPln);
        ElSLib.PlaneParameters(Pos2, P2, out u, out v);
        p2dPln.SetCoord(u, v);
        dir2dPln.SetCoord(xdir.Dot(Pos2.XDirection()), xdir.Dot(Pos2.YDirection()));
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(dir2dPln);
        Geom2d_Line GLin2dPln2 = new(lin2dPln);
        ElSLib.PlaneParameters(PlanAx3, P2, out u, out v);
        p2dPln.SetCoord(u, v);
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(gp.DX2d());
        Geom2d_Line GLin2dPlnCh2 = new(lin2dPln);
        norpl = Pos2.XDirection().Crossed(Pos2.YDirection());
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (VecTransl2.Dot(DirCh12) < 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            trans = TopAbs_REVERSED;
        }
        else
        {
            trans = TopAbs_FORWARD;
        }
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln2, DStr), trans, GLin2dPln2,
                                                      GLin2dPlnCh2);
        return true;
    }
}
