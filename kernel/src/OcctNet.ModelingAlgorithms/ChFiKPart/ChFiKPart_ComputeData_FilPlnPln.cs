// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_FilPlnPln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>The constant radius fillet between two planes: a cylinder.</summary>
internal static class ChFiKPart_ComputeData_FilPlnPln
{
    //=================================================================================================

    /// <summary>Fills <paramref name="Data"/> with the cylinder of radius <paramref name="Radius"/> between the
    /// planes, its origin at <paramref name="First"/> on the guide line.</summary>
    internal static bool ChFiKPart_MakeFillet(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Data, in gp_Pln Pl1,
                                              in gp_Pln Pl2, TopAbs_Orientation Or1, TopAbs_Orientation Or2,
                                              double Radius, in gp_Lin Spine, double First, TopAbs_Orientation Of1)
    {
        // calcul du cylindre
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
        IntAna_QuadQuadGeo LInt = new(Pl1, Pl2, Precision.Angular(), Precision.Confusion());
        gp_Pnt Pv;
        if (LInt.IsDone())
        {
            // On met l origine du cylindre au point de depart fourni sur la
            // ligne guide.
            Pv = ElCLib.Value(ElCLib.Parameter(LInt.Line(1), ElCLib.Value(First, Spine)), LInt.Line(1));
        }
        else
        {
            return false;
        }
        gp_Dir AxisCylinder = Spine.Direction();
        double Ang = D1.Angle(D2);
        gp_Vec V = new gp_Vec(D1) + new gp_Vec(D2);
        gp_Dir S = new(V);
        gp_Pnt C = new();
        double Fac = Radius / Math.Cos(Ang / 2.0);
        C.SetCoord(Pv.X() + Fac * S.X(), Pv.Y() + Fac * S.Y(), Pv.Z() + Fac * S.Z());
        gp_Dir xdir = D1.Reversed();
        gp_Ax3 CylAx3 = new(C, AxisCylinder, xdir);
        if (CylAx3.YDirection().Dot(D2) >= 0.0)
        {
            CylAx3.YReverse();
        }
        Geom_CylindricalSurface gcyl = new(CylAx3, Radius);
        Data.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gcyl, DStr));

        // On regarde si l orientation du cylindre est la meme que celle
        // des faces.
        ElSLib.CylinderD1(0.0, 0.0, CylAx3, Radius, out gp_Pnt P, out gp_Vec deru, out gp_Vec derv);
        gp_Dir norcyl = new(deru.Crossed(derv));
        gp_Dir norpl = Pos1.XDirection().Crossed(Pos1.YDirection());
        gp_Dir norface = norpl;
        if (Of1 == TopAbs_REVERSED)
        {
            norface.Reverse();
        }
        bool toreverse = (norcyl.Dot(norface) <= 0.0);
        if (toreverse)
        {
            Data.ChangeOrientation() = TopAbs_REVERSED;
        }
        else
        {
            Data.ChangeOrientation() = TopAbs_FORWARD;
        }

        // On charge les FaceInterferences avec les pcurves et courbes 3d.

        double u, v;
        // La face 1.
        ElSLib.PlaneParameters(Pos1, P, out u, out v);
        gp_Pnt2d p2dPln = new(u, v);
        gp_Dir2d dir2dPln = new(AxisCylinder.Dot(Pos1.XDirection()), AxisCylinder.Dot(Pos1.YDirection()));
        gp_Lin2d lin2dPln = new(p2dPln, dir2dPln);
        Geom2d_Line GLin2dPln1 = new(lin2dPln);
        gp_Lin linPln = new(P, AxisCylinder);
        Geom_Line GLinPln1 = new(linPln);
        gp_Lin2d lin2dCyl = new(new gp_Pnt2d(0.0, 0.0), gp.DY2d());
        Geom2d_Line GLin2dCyl1 = new(lin2dCyl);
        TopAbs_Orientation trans;
        toreverse = (norcyl.Dot(norpl) <= 0.0);
        if (toreverse)
        {
            trans = TopAbs_REVERSED;
        }
        else
        {
            trans = TopAbs_FORWARD;
        }
        Data.ChangeInterferenceOnS1().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln1, DStr), trans, GLin2dPln1,
                                                      GLin2dCyl1);

        // La face 2.
        ElSLib.CylinderD1(Ang, 0.0, CylAx3, Radius, out P, out deru, out derv);
        norcyl = new gp_Dir(deru.Crossed(derv));
        norpl = Pos2.XDirection().Crossed(Pos2.YDirection());
        toreverse = (norcyl.Dot(norpl) <= 0.0);
        ElSLib.PlaneParameters(Pos2, P, out u, out v);
        p2dPln.SetCoord(u, v);
        dir2dPln.SetCoord(AxisCylinder.Dot(Pos2.XDirection()), AxisCylinder.Dot(Pos2.YDirection()));
        lin2dPln.SetLocation(p2dPln);
        lin2dPln.SetDirection(dir2dPln);
        Geom2d_Line GLin2dPln2 = new(lin2dPln);
        linPln.SetLocation(P);
        linPln.SetDirection(AxisCylinder);
        Geom_Line GLinPln2 = new(linPln);
        lin2dCyl.SetLocation(new gp_Pnt2d(Ang, 0.0));
        Geom2d_Line GLin2dCyl2 = new(lin2dCyl);
        if (toreverse)
        {
            trans = TopAbs_FORWARD;
        }
        else
        {
            trans = TopAbs_REVERSED;
        }
        Data.ChangeInterferenceOnS2().SetInterference(ChFiKPart_IndexCurveInDS(GLinPln2, DStr), trans, GLin2dPln2,
                                                      GLin2dCyl2);
        return true;
    }
}
