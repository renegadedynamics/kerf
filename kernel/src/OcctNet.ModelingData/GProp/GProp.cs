// Port of: src/ModelingData/TKGeomBase/GProp/GProp.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Global properties (length, area, volume, centre of mass, inertia) of sets of points, curves, surfaces, solids
/// and compound systems.
/// </summary>
public static class GProp
{
    /// <summary>
    /// The "Huyghens operator" of a system of mass <paramref name="Mass"/> and centre of mass <paramref name="G"/>
    /// at the point <paramref name="Q"/>: Inertia/Q = Inertia/G + HOperator(G, Q, Mass).
    /// </summary>
    public static void HOperator(in gp_Pnt G, in gp_Pnt Q, double Mass, out gp_Mat Operator)
    {
        gp_XYZ QG = G.XYZ() - Q.XYZ();
        double Ixx = QG.Y() * QG.Y() + QG.Z() * QG.Z();
        double Iyy = QG.X() * QG.X() + QG.Z() * QG.Z();
        double Izz = QG.Y() * QG.Y() + QG.X() * QG.X();
        double Ixy = -QG.X() * QG.Y();
        double Iyz = -QG.Y() * QG.Z();
        double Ixz = -QG.X() * QG.Z();
        Operator = new gp_Mat();
        Operator.SetCols(new gp_XYZ(Ixx, Ixy, Ixz), new gp_XYZ(Ixy, Iyy, Iyz), new gp_XYZ(Ixz, Iyz, Izz));
        Operator.Multiply(Mass);
    }
}
