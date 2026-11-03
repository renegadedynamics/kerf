// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCone.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds cones or portions of cones of height H, radius R1 at z = 0 and R2 at z = H (either may be 0), about the Z
/// axis of the global coordinate system or of a local one; a portion is limited by U in 0..angle.
/// </summary>
/// <remarks>
/// Throws Standard_DomainError if H is below Precision::Confusion(), if R1 and R2 are equal, or if a radius is
/// negative or not null and below Precision::Confusion().
/// </remarks>
public class ApiMakeCone : MakeOneAxis
{
    private readonly PrimitiveCone _cone;

    /// <summary>A cone of radius R1 at z = 0, R2 at z = H.</summary>
    public ApiMakeCone(double r1, double r2, double h)
    {
        _cone = new PrimitiveCone(Constants.XOY(), r1, r2, h);
    }

    /// <summary>A part cone of radius R1 at z = 0, R2 at z = H and angle.</summary>
    public ApiMakeCone(double r1, double r2, double h, double angle)
    {
        _cone = new PrimitiveCone(r1, r2, h);
        _cone.Angle(angle);
    }

    /// <summary>A cone on the axes.</summary>
    public ApiMakeCone(in Frame3D axes, double r1, double r2, double h)
    {
        _cone = new PrimitiveCone(axes, r1, r2, h);
    }

    /// <summary>A part cone on the axes.</summary>
    public ApiMakeCone(in Frame3D axes, double r1, double r2, double h, double angle)
    {
        _cone = new PrimitiveCone(axes, r1, r2, h);
        _cone.Angle(angle);
    }

    internal override OneAxis OneAxis()
    {
        return _cone;
    }

    /// <summary>The algorithm.</summary>
    internal PrimitiveCone Cone()
    {
        return _cone;
    }
}
