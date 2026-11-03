// Port of: src/ModelingData/TKGeomBase/GProp/GProp.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>
/// Global properties (length, area, volume, centre of mass, inertia) of sets of points, curves, surfaces, solids
/// and compound systems.
/// </summary>
public static class GProp
{
    /// <summary>
    /// The "Huyghens operator" of a system of mass <paramref name="mass"/> and centre of mass <paramref name="g"/>
    /// at the point <paramref name="q"/>: Inertia/Q = Inertia/G + HOperator(G, Q, Mass).
    /// </summary>
    public static void HOperator(in Point3D g, in Point3D q, double mass, out Matrix3x3 Operator)
    {
        Xyz qg = g.XYZ() - q.XYZ();
        double ixx = qg.Y() * qg.Y() + qg.Z() * qg.Z();
        double iyy = qg.X() * qg.X() + qg.Z() * qg.Z();
        double izz = qg.Y() * qg.Y() + qg.X() * qg.X();
        double ixy = -qg.X() * qg.Y();
        double iyz = -qg.Y() * qg.Z();
        double ixz = -qg.X() * qg.Z();
        Operator = new Matrix3x3();
        Operator.SetCols(new Xyz(ixx, ixy, ixz), new Xyz(ixy, iyy, iyz), new Xyz(ixz, iyz, izz));
        Operator.Multiply(mass);
    }
}
