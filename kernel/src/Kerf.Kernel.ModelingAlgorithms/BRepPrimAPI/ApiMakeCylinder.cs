// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCylinder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds cylinders or portions of cylinders about the Z axis of the global coordinate system or of a local one,
/// limited by V in 0..H and, for a portion, U in 0..Angle (radians). The result has a lateral cylindrical face, two
/// planar faces at z = 0 and z = H and, for a portion, two planar faces at u = 0 and u = Angle.
/// </summary>
internal class ApiMakeCylinder : MakeOneAxis
{
    private readonly PrimitiveCylinder _cylinder;

    /// <summary>A cylinder of radius R and height H.</summary>
    public ApiMakeCylinder(double r, double h)
    {
        _cylinder = new PrimitiveCylinder(Constants.XOY(), r, h);
    }

    /// <summary>A part cylinder of radius R, height H and angle Angle.</summary>
    public ApiMakeCylinder(double r, double h, double angle)
    {
        _cylinder = new PrimitiveCylinder(r, h);
        _cylinder.Angle(angle);
    }

    /// <summary>A cylinder of radius R and length H on the axes.</summary>
    public ApiMakeCylinder(in Frame3D axes, double r, double h)
    {
        _cylinder = new PrimitiveCylinder(axes, r, h);
    }

    /// <summary>A part cylinder of radius R, length H and angle Angle on the axes.</summary>
    public ApiMakeCylinder(in Frame3D axes, double r, double h, double angle)
    {
        _cylinder = new PrimitiveCylinder(axes, r, h);
        _cylinder.Angle(angle);
    }

    internal override OneAxis OneAxis()
    {
        return _cylinder;
    }

    /// <summary>The algorithm.</summary>
    internal PrimitiveCylinder Cylinder()
    {
        return _cylinder;
    }
}
