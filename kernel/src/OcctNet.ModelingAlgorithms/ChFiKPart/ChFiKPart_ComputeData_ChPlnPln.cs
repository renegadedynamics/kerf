// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChPlnPln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The chamfer between two planes: a plane.</summary>
internal static class ChFiKPart_ComputeData_ChPlnPln
{
    //=======================================================================
    // function : MakeChamfer
    // Purpose  : Compute the chamfer in the particular case plane/plane.
    //           Compute the SurfData <Data> of the chamfer on the <Spine>
    //           between the plane <Pl1> and the plane <Pl2>, with distances
    //           <Dis1> on <Pl1> and <Dis2> on <Pl2>.
    //           <First> is the parameter of the start point on the <Spine>
    //           <Or1> and <Or2> are the orientations of the plane <Pl1> and
    //           <Pl2>, and <Of1> the orientation of the face build on the
    //           plane <Pl1>.
    // Out      : True if the chamfer has been computed
    //           False else
    //=======================================================================

    /// <summary>The chamfer plane between <paramref name="Pl1"/> and <paramref name="Pl2"/>, at the distances
    /// <paramref name="theDis1"/> and <paramref name="theDis2"/> (or the throat, by <paramref name="theMode"/>).
    /// </summary>
    internal static bool ChFiKPart_MakeChamfer(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data,
                                               ChFiDS_ChamfMode theMode, in Plane3D Pl1, in Plane3D Pl2,
                                               Orientation Or1, Orientation Or2, double theDis1,
                                               double theDis2, in Line3D Spine, double First,
                                               Orientation Of1)
    {
        // Creation of the plane which carry the chamfer
        // compute the normals to the planes Pl1 and Pl2
        CoordinateSystem Pos1 = Pl1.Position();
        Direction3D D1 = Pos1.XDirection().Crossed(Pos1.YDirection());
        if (Or1 == Reversed)
        {
            D1.Reverse();
        }
        CoordinateSystem Pos2 = Pl2.Position();
        Direction3D D2 = Pos2.XDirection().Crossed(Pos2.YDirection());
        if (Or2 == Reversed)
        {
            D2.Reverse();
        }
        // compute the intersection line of Pl1 and Pl2
        QuadQuadGeo LInt = new(Pl1, Pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D P;
        double Fint;
        if (LInt.IsDone)
        {
            Fint = global::Kerf.Engine.Geometry.ElCLib.Parameter(LInt.Line(1), global::Kerf.Engine.Geometry.ElCLib.Value(First, Spine));
            P = global::Kerf.Engine.Geometry.ElCLib.Value(Fint, LInt.Line(1));
        }
        else
        {
            return false;
        }
        Direction3D LinAx1 = Spine.Direction();
        Direction3D VecTransl1 = LinAx1.Crossed(D1);
        if (VecTransl1.Dot(D2) <= 0.0)
        {
            VecTransl1.Reverse();
        }
        Direction3D VecTransl2 = LinAx1.Crossed(D2);
        if (VecTransl2.Dot(D1) <= 0.0)
        {
            VecTransl2.Reverse();
        }
        double Dis1 = theDis1, Dis2 = theDis2;
        double Alpha = VecTransl1.Angle(VecTransl2);
        double CosHalfAlpha = Math.Cos(Alpha / 2);
        if (theMode == ChFiDS_ConstThroatChamfer)
        {
            Dis1 = Dis2 = theDis1 / CosHalfAlpha;
        }
        else if (theMode == ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            double aDis1 = Math.Min(theDis1, theDis2);
            double aDis2 = Math.Max(theDis1, theDis2);
            double dis1dis1 = aDis1 * aDis1, dis2dis2 = aDis2 * aDis2;
            double SinAlpha = Math.Sin(Alpha);
            double CosAlpha = Math.Cos(Alpha);
            double CotanAlpha = CosAlpha / SinAlpha;
            Dis1 = Math.Sqrt(dis2dis2 - dis1dis1) - aDis1 * CotanAlpha;
            double CosBeta = Math.Sqrt(1 - dis1dis1 / dis2dis2) * CosAlpha + aDis1 / aDis2 * SinAlpha;
            double FullDist1 = aDis2 / CosBeta;
            Dis2 = FullDist1 - aDis1 / SinAlpha;
        }
        // Compute a point on the plane Pl1 and on the chamfer
        Point3D P1 = new(P.X() + Dis1 * VecTransl1.X(), P.Y() + Dis1 * VecTransl1.Y(), P.Z() + Dis1 * VecTransl1.Z());
        // Point on the plane Pl2 and on the chamfer
        Point3D P2 = new(P.X() + Dis2 * VecTransl2.X(), P.Y() + Dis2 * VecTransl2.Y(), P.Z() + Dis2 * VecTransl2.Z());
        // the middle point of P1 P2 is the origin of the chamfer
        Point3D Po = new((P1.X() + P2.X()) / 2.0, (P1.Y() + P2.Y()) / 2.0, (P1.Z() + P2.Z()) / 2.0);
        // compute a second point on the plane Pl2
        Point3D Pp = global::Kerf.Engine.Geometry.ElCLib.Value(Fint + 10.0, LInt.Line(1));
        Point3D P22 = new(Pp.X() + Dis2 * VecTransl2.X(), Pp.Y() + Dis2 * VecTransl2.Y(),
                         Pp.Z() + Dis2 * VecTransl2.Z());
        // Compute the normal vector <AxisPlan> to the chamfer's plane
        Direction3D V1 = new(P2.X() - P1.X(), P2.Y() - P1.Y(), P2.Z() - P1.Z());
        Direction3D V2 = new(P22.X() - P1.X(), P22.Y() - P1.Y(), P22.Z() - P1.Z());
        Direction3D AxisPlan = V1.Crossed(V2);
        Direction3D xdir = LinAx1; // u axis
        CoordinateSystem PlanAx3 = new(Po, AxisPlan, xdir);
        if (PlanAx3.YDirection().Dot(D2) >= 0.0)
        {
            PlanAx3.YReverse();
        }
        PlaneSurface gpl = new(PlanAx3);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gpl, DStr));
        // About the orientation of the chamfer plane
        // Compute the normal to the face 1
        Direction3D norpl = Pos1.XDirection().Crossed(Pos1.YDirection());
        Direction3D norface1 = norpl;
        if (Of1 == Reversed)
        {
            norface1.Reverse();
        }
        // Compute the orientation of the chamfer plane
        Direction3D norplch = gpl.Pln().Position().XDirection().Crossed(gpl.Pln().Position().YDirection());
        Direction3D DirCh12 = new(new Vector3D(P1, P2));
        bool toreverse = (norplch.Dot(norface1) <= 0.0);
        if (VecTransl1.Dot(DirCh12) > 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            Data.ChangeOrientation() = Reversed;
        }
        else
        {
            Data.ChangeOrientation() = Forward;
        }
        // Loading of the FaceInterferences with pcurves & 3d curves.
        // case face 1
        Line3D linPln = new(P1, xdir);
        LineCurve3D GLinPln1 = new(linPln);
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(Pos1, P1, out u, out v);
        Point2D p2dPln = new(u, v);
        Direction2D dir2dPln = new(xdir.Dot(Pos1.XDirection()), xdir.Dot(Pos1.YDirection()));
        Line2D lin2dPln = new(p2dPln, dir2dPln);
        LineCurve2D GLin2dPln1 = new(lin2dPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PlanAx3, P1, out u, out v);
        p2dPln.SetCoord(u, v);
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(Constants.DX2d());
        LineCurve2D GLin2dPlnCh1 = new(lin2dPln);
        Orientation trans;
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (VecTransl1.Dot(DirCh12) > 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            trans = Forward;
        }
        else
        {
            trans = Reversed;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln1, DStr), trans, GLin2dPln1,
                                                      GLin2dPlnCh1);
        // case face 2
        linPln.SetLocation(P2);
        LineCurve3D GLinPln2 = new(linPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(Pos2, P2, out u, out v);
        p2dPln.SetCoord(u, v);
        dir2dPln.SetCoord(xdir.Dot(Pos2.XDirection()), xdir.Dot(Pos2.YDirection()));
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(dir2dPln);
        LineCurve2D GLin2dPln2 = new(lin2dPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(PlanAx3, P2, out u, out v);
        p2dPln.SetCoord(u, v);
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(Constants.DX2d());
        LineCurve2D GLin2dPlnCh2 = new(lin2dPln);
        norpl = Pos2.XDirection().Crossed(Pos2.YDirection());
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (VecTransl2.Dot(DirCh12) < 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            trans = Reversed;
        }
        else
        {
            trans = Forward;
        }
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln2, DStr), trans, GLin2dPln2,
                                                      GLin2dPlnCh2);
        return true;
    }
}
