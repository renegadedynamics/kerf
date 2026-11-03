// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_ChAsymPlnPln.cxx
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

/// <summary>The chamfer by a distance and an angle between two planes: a plane.</summary>
internal static class ComputeDataChAsymPlnPln
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

    /// <summary>The chamfer plane between <paramref name="pl1"/> and <paramref name="pl2"/> at the distance
    /// <paramref name="dis"/> on the first (or the second, by <paramref name="disOnP1"/>) and the angle
    /// <paramref name="angle"/>.</summary>
    internal static bool ChFiKPart_MakeChAsym(DsDataStructure dStr, SurfData data, in Plane3D pl1,
                                              in Plane3D pl2, Orientation or1, Orientation or2,
                                              double dis, double angle, in Line3D spine, double first,
                                              Orientation of1, bool disOnP1)
    {
        // Creation of the plane which carry the chamfer
        // compute the normals to the planes Pl1 and Pl2
        CoordinateSystem pos1 = pl1.Position();
        Direction3D d1 = pos1.XDirection().Crossed(pos1.YDirection());
        if (or1 == Reversed)
        {
            d1 = d1.Reversed();
        }
        CoordinateSystem pos2 = pl2.Position();
        Direction3D d2 = pos2.XDirection().Crossed(pos2.YDirection());
        if (or2 == Reversed)
        {
            d2 = d2.Reversed();
        }
        // compute the intersection line of Pl1 and Pl2
        QuadQuadGeo lInt = new(pl1, pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D p;
        double fint;
        if (lInt.IsDone)
        {
            fint = global::Kerf.Engine.Geometry.ElCLib.Parameter(lInt.Line(1), global::Kerf.Engine.Geometry.ElCLib.Value(first, spine));
            p = global::Kerf.Engine.Geometry.ElCLib.Value(fint, lInt.Line(1));
        }
        else
        {
            return false;
        }
        Direction3D linAx1 = spine.Direction();
        Direction3D vecTransl1 = linAx1.Crossed(d1);
        if (vecTransl1.Dot(d2) < 0.0)
        {
            vecTransl1 = vecTransl1.Reversed();
        }
        Direction3D vecTransl2 = linAx1.Crossed(d2);
        if (vecTransl2.Dot(d1) < 0.0)
        {
            vecTransl2 = vecTransl2.Reversed();
        }
        double cosP, sinP, dis1, dis2;
        cosP = vecTransl1.Dot(vecTransl2);
        sinP = Math.Sqrt(1.0 - cosP * cosP);
        if (disOnP1)
        {
            dis1 = dis;
            dis2 = dis / (cosP + sinP / Math.Tan(angle));
        }
        else
        {
            dis1 = dis / (cosP + sinP / Math.Tan(angle));
            dis2 = dis;
        }
        // Compute a point on the plane Pl1 and on the chamfer
        Point3D p1 = new(p.X() + dis1 * vecTransl1.X(), p.Y() + dis1 * vecTransl1.Y(), p.Z() + dis1 * vecTransl1.Z());
        // Point on the plane Pl2 and on the chamfer
        Point3D p2 = new(p.X() + dis2 * vecTransl2.X(), p.Y() + dis2 * vecTransl2.Y(), p.Z() + dis2 * vecTransl2.Z());
        // the middle point of P1 P2 is the origin of the chamfer
        Point3D po = new((p1.X() + p2.X()) / 2.0, (p1.Y() + p2.Y()) / 2.0, (p1.Z() + p2.Z()) / 2.0);
        // compute a second point on the plane Pl2
        Point3D pp = global::Kerf.Engine.Geometry.ElCLib.Value(fint + 10.0, lInt.Line(1));
        Point3D p22 = new(pp.X() + dis2 * vecTransl2.X(), pp.Y() + dis2 * vecTransl2.Y(),
                         pp.Z() + dis2 * vecTransl2.Z());
        // Compute the normal vector <AxisPlan> to the chamfer's plane
        Direction3D v1 = new(p2.X() - p1.X(), p2.Y() - p1.Y(), p2.Z() - p1.Z());
        Direction3D v2 = new(p22.X() - p1.X(), p22.Y() - p1.Y(), p22.Z() - p1.Z());
        Direction3D axisPlan = v1.Crossed(v2);
        Direction3D xdir = linAx1; // u axis
        CoordinateSystem planAx3 = new(po, axisPlan, xdir);
        if (planAx3.YDirection().Dot(d2) >= 0.0)
        {
            planAx3 = planAx3.YReversed();
        }
        PlaneSurface gpl = new(planAx3);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gpl, dStr));
        // About the orientation of the chamfer plane
        // Compute the normal to the face 1
        Direction3D norpl = pos1.XDirection().Crossed(pos1.YDirection());
        Direction3D norface1 = norpl;
        if (of1 == Reversed)
        {
            norface1 = norface1.Reversed();
        }
        // Compute the orientation of the chamfer plane
        Direction3D norplch = gpl.Pln().Position().XDirection().Crossed(gpl.Pln().Position().YDirection());
        Direction3D dirCh12 = new(new Vector3D(p1, p2));
        bool toreverse = (norplch.Dot(norface1) <= 0.0);
        if (vecTransl1.Dot(dirCh12) > 0)
        {
            toreverse = !toreverse;
        }
        if (toreverse)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }
        // Loading of the FaceInterferences with pcurves & 3d curves.
        // case face 1
        Line3D linPln = new(p1, xdir);
        LineCurve3D gLinPln1 = new(linPln);
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(pos1, p1, out u, out v);
        Point2D p2dPln = new(u, v);
        Direction2D dir2dPln = new(xdir.Dot(pos1.XDirection()), xdir.Dot(pos1.YDirection()));
        Line2D lin2dPln = new(p2dPln, dir2dPln);
        LineCurve2D gLin2dPln1 = new(lin2dPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(planAx3, p1, out u, out v);
        p2dPln = p2dPln.WithCoord(u, v);
        lin2dPln = lin2dPln.WithLocation(p2dPln);
        lin2dPln = lin2dPln.WithDirection(Constants.DX2d());
        LineCurve2D gLin2dPlnCh1 = new(lin2dPln);
        Orientation trans;
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (vecTransl1.Dot(dirCh12) > 0)
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
        data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gLinPln1, dStr), trans, gLin2dPln1,
                                                      gLin2dPlnCh1);
        // case face 2
        linPln = linPln.WithLocation(p2);
        LineCurve3D gLinPln2 = new(linPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(pos2, p2, out u, out v);
        p2dPln = p2dPln.WithCoord(u, v);
        dir2dPln = dir2dPln.WithCoord(xdir.Dot(pos2.XDirection()), xdir.Dot(pos2.YDirection()));
        lin2dPln = lin2dPln.WithLocation(p2dPln);
        lin2dPln = lin2dPln.WithDirection(dir2dPln);
        LineCurve2D gLin2dPln2 = new(lin2dPln);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(planAx3, p2, out u, out v);
        p2dPln = p2dPln.WithCoord(u, v);
        lin2dPln = lin2dPln.WithLocation(p2dPln);
        lin2dPln = lin2dPln.WithDirection(Constants.DX2d());
        LineCurve2D gLin2dPlnCh2 = new(lin2dPln);
        norpl = pos2.XDirection().Crossed(pos2.YDirection());
        toreverse = (norplch.Dot(norpl) <= 0.0);
        if (vecTransl2.Dot(dirCh12) < 0)
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
        data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gLinPln2, dStr), trans, gLin2dPln2,
                                                      gLin2dPlnCh2);
        return true;
    }
}
