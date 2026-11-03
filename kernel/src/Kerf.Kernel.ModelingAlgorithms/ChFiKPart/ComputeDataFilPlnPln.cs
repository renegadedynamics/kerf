// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.cxx
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

/// <summary>The constant radius fillet between two planes: a cylinder.</summary>
internal static class ComputeDataFilPlnPln
{
    //=================================================================================================

    /// <summary>Fills <paramref name="data"/> with the cylinder of radius <paramref name="radius"/> between the
    /// planes, its origin at <paramref name="first"/> on the guide line.</summary>
    internal static bool ChFiKPart_MakeFillet(DsDataStructure dStr, SurfData data, in Plane3D pl1,
                                              in Plane3D pl2, Orientation or1, Orientation or2,
                                              double radius, in Line3D spine, double first, Orientation of1)
    {
        // calcul du cylindre
        CoordinateSystem pos1 = pl1.Position;
        Direction3D d1 = pos1.XDirection.Crossed(pos1.YDirection);
        if (or1 == Reversed)
        {
            d1 = d1.Reversed();
        }
        CoordinateSystem pos2 = pl2.Position;
        Direction3D d2 = pos2.XDirection.Crossed(pos2.YDirection);
        if (or2 == Reversed)
        {
            d2 = d2.Reversed();
        }
        QuadQuadGeo lInt = new(pl1, pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D pv;
        if (lInt.IsDone)
        {
            // On met l origine du cylindre au point de depart fourni sur la
            // ligne guide.
            pv = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(lInt.Line(1), global::Kerf.Engine.Geometry.ElCLib.Value(first, spine)), lInt.Line(1));
        }
        else
        {
            return false;
        }
        Direction3D axisCylinder = spine.Direction;
        double ang = d1.AngleTo(d2);
        Vector3D V = new Vector3D(d1) + new Vector3D(d2);
        Direction3D s = new(V);
        Point3D c = new();
        double fac = radius / Math.Cos(ang / 2.0);
        c = c.WithCoord(pv.X + fac * s.X, pv.Y + fac * s.Y, pv.Z + fac * s.Z);
        Direction3D xdir = d1.Reversed();
        CoordinateSystem cylAx3 = new(c, axisCylinder, xdir);
        if (cylAx3.YDirection.Dot(d2) >= 0.0)
        {
            cylAx3 = cylAx3.YReversed();
        }
        CylindricalSurface gcyl = new(cylAx3, radius);
        data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcyl, dStr));

        // On regarde si l orientation du cylindre est la meme que celle
        // des faces.
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(0.0, 0.0, cylAx3, radius, out Point3D p, out Vector3D deru, out Vector3D derv);
        Direction3D norcyl = new(deru.Crossed(derv));
        Direction3D norpl = pos1.XDirection.Crossed(pos1.YDirection);
        Direction3D norface = norpl;
        if (of1 == Reversed)
        {
            norface = norface.Reversed();
        }
        bool toreverse = (norcyl.Dot(norface) <= 0.0);
        if (toreverse)
        {
            data.ChangeOrientation() = Reversed;
        }
        else
        {
            data.ChangeOrientation() = Forward;
        }

        // On charge les FaceInterferences avec les pcurves et courbes 3d.

        double u, v;
        // La face 1.
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(pos1, p, out u, out v);
        Point2D p2dPln = new(u, v);
        Direction2D dir2dPln = new(axisCylinder.Dot(pos1.XDirection), axisCylinder.Dot(pos1.YDirection));
        Line2D lin2dPln = new(p2dPln, dir2dPln);
        LineCurve2D gLin2dPln1 = new(lin2dPln);
        Line3D linPln = new(p, axisCylinder);
        LineCurve3D gLinPln1 = new(linPln);
        Line2D lin2dCyl = new(new Point2D(0.0, 0.0), Constants.DY2d());
        LineCurve2D gLin2dCyl1 = new(lin2dCyl);
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
        data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(gLinPln1, dStr), trans, gLin2dPln1,
                                                      gLin2dCyl1);

        // La face 2.
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(ang, 0.0, cylAx3, radius, out p, out deru, out derv);
        norcyl = new Direction3D(deru.Crossed(derv));
        norpl = pos2.XDirection.Crossed(pos2.YDirection);
        toreverse = (norcyl.Dot(norpl) <= 0.0);
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(pos2, p, out u, out v);
        p2dPln = p2dPln.WithCoord(u, v);
        dir2dPln = dir2dPln.WithCoord(axisCylinder.Dot(pos2.XDirection), axisCylinder.Dot(pos2.YDirection));
        lin2dPln = lin2dPln.WithLocation(p2dPln);
        lin2dPln = lin2dPln.WithDirection(dir2dPln);
        LineCurve2D gLin2dPln2 = new(lin2dPln);
        linPln = linPln.WithLocation(p);
        linPln = linPln.WithDirection(axisCylinder);
        LineCurve3D gLinPln2 = new(linPln);
        lin2dCyl = lin2dCyl.WithLocation(new Point2D(ang, 0.0));
        LineCurve2D gLin2dCyl2 = new(lin2dCyl);
        if (toreverse)
        {
            trans = Forward;
        }
        else
        {
            trans = Reversed;
        }
        data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(gLinPln2, dStr), trans, gLin2dPln2,
                                                      gLin2dCyl2);
        return true;
    }
}
