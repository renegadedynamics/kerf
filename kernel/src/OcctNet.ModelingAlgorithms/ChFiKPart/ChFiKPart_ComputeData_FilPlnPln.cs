// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>The constant radius fillet between two planes: a cylinder.</summary>
internal static class ChFiKPart_ComputeData_FilPlnPln
{
    //=================================================================================================

    /// <summary>Fills <paramref name="Data"/> with the cylinder of radius <paramref name="Radius"/> between the
    /// planes, its origin at <paramref name="First"/> on the guide line.</summary>
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in Plane3D Pl1,
                                              in Plane3D Pl2, Orientation Or1, Orientation Or2,
                                              double Radius, in Line3D Spine, double First, Orientation Of1)
    {
        // calcul du cylindre
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
        QuadQuadGeo LInt = new(Pl1, Pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D Pv;
        if (LInt.IsDone)
        {
            // On met l origine du cylindre au point de depart fourni sur la
            // ligne guide.
            Pv = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(LInt.Line(1), global::Kerf.Engine.Geometry.ElCLib.Value(First, Spine)), LInt.Line(1));
        }
        else
        {
            return false;
        }
        Direction3D AxisCylinder = Spine.Direction();
        double Ang = D1.Angle(D2);
        Vector3D V = new Vector3D(D1) + new Vector3D(D2);
        Direction3D S = new(V);
        Point3D C = new();
        double Fac = Radius / Math.Cos(Ang / 2.0);
        C.SetCoord(Pv.X() + Fac * S.X(), Pv.Y() + Fac * S.Y(), Pv.Z() + Fac * S.Z());
        Direction3D xdir = D1.Reversed();
        CoordinateSystem CylAx3 = new(C, AxisCylinder, xdir);
        if (CylAx3.YDirection().Dot(D2) >= 0.0)
        {
            CylAx3.YReverse();
        }
        CylindricalSurface gcyl = new(CylAx3, Radius);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcyl, DStr));

        // On regarde si l orientation du cylindre est la meme que celle
        // des faces.
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(0.0, 0.0, CylAx3, Radius, out Point3D P, out Vector3D deru, out Vector3D derv);
        Direction3D norcyl = new(deru.Crossed(derv));
        Direction3D norpl = Pos1.XDirection().Crossed(Pos1.YDirection());
        Direction3D norface = norpl;
        if (Of1 == Reversed)
        {
            norface.Reverse();
        }
        bool toreverse = (norcyl.Dot(norface) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = Reversed;
        }
        else
        {
            Data.ChangeOrientation() = Forward;
        }

        // On charge les FaceInterferences avec les pcurves et courbes 3d.

        double u, v;
        // La face 1.
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(Pos1, P, out u, out v);
        Point2D p2dPln = new(u, v);
        Direction2D dir2dPln = new(AxisCylinder.Dot(Pos1.XDirection()), AxisCylinder.Dot(Pos1.YDirection()));
        Line2D lin2dPln = new(p2dPln, dir2dPln);
        LineCurve2D GLin2dPln1 = new(lin2dPln);
        Line3D linPln = new(P, AxisCylinder);
        LineCurve3D GLinPln1 = new(linPln);
        Line2D lin2dCyl = new(new Point2D(0.0, 0.0), Constants.DY2d());
        LineCurve2D GLin2dCyl1 = new(lin2dCyl);
        Orientation trans;
        toreverse = (norcyl.Dot(norpl) <= 0.0);
        if (toreverse)
        {
            trans = Reversed;
        }
        else
        {
            trans = Forward;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln1, DStr), trans, GLin2dPln1,
                                                      GLin2dCyl1);

        // La face 2.
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(Ang, 0.0, CylAx3, Radius, out P, out deru, out derv);
        norcyl = new Direction3D(deru.Crossed(derv));
        norpl = Pos2.XDirection().Crossed(Pos2.YDirection());
        toreverse = (norcyl.Dot(norpl) <= 0.0);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(Pos2, P, out u, out v);
        p2dPln.SetCoord(u, v);
        dir2dPln.SetCoord(AxisCylinder.Dot(Pos2.XDirection()), AxisCylinder.Dot(Pos2.YDirection()));
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(dir2dPln);
        LineCurve2D GLin2dPln2 = new(lin2dPln);
        linPln.SetLocation(P);
        linPln.SetDirection(AxisCylinder);
        LineCurve3D GLinPln2 = new(linPln);
        lin2dCyl.SetLocation(new Point2D(Ang, 0.0));
        LineCurve2D GLin2dCyl2 = new(lin2dCyl);
        if (toreverse)
        {
            trans = Forward;
        }
        else
        {
            trans = Reversed;
        }
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln2, DStr), trans, GLin2dPln2,
                                                      GLin2dCyl2);
        return true;
    }
}
