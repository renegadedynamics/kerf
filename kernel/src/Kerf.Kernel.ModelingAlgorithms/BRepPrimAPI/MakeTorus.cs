// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeTorus.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeTorus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds tori or portions of tori: R1 is the distance from the center of the pipe to the center of the torus, R2 the
/// radius of the pipe; angle1, angle2 cut the pipe's circle (a ring segment) and angle the revolution (a pipe segment).
/// </summary>
public class MakeTorus : MakeOneAxis
{
    private readonly PrimitiveTorus _torus;

    /// <summary>A torus.</summary>
    public MakeTorus(double r1, double r2)
    {
        _torus = new PrimitiveTorus(r1, r2);
    }

    /// <summary>A torus pipe segment.</summary>
    public MakeTorus(double r1, double r2, double angle)
    {
        _torus = new PrimitiveTorus(r1, r2);
        _torus.Angle(angle);
    }

    /// <summary>A torus ring segment.</summary>
    public MakeTorus(double r1, double r2, double angle1, double angle2)
    {
        _torus = new PrimitiveTorus(r1, r2);
        _torus.VMin(angle1);
        _torus.VMax(angle2);
    }

    /// <summary>A torus ring and pipe segment.</summary>
    public MakeTorus(double r1, double r2, double angle1, double angle2, double angle)
    {
        _torus = new PrimitiveTorus(r1, r2);
        _torus.VMin(angle1);
        _torus.VMax(angle2);
        _torus.Angle(angle);
    }

    /// <summary>A torus on the axes.</summary>
    public MakeTorus(in Frame3D axes, double r1, double r2)
    {
        _torus = new PrimitiveTorus(axes, r1, r2);
    }

    /// <summary>A torus pipe segment on the axes.</summary>
    public MakeTorus(in Frame3D axes, double r1, double r2, double angle)
    {
        _torus = new PrimitiveTorus(axes, r1, r2);
        _torus.Angle(angle);
    }

    /// <summary>A torus ring segment on the axes.</summary>
    public MakeTorus(in Frame3D axes, double r1, double r2, double angle1, double angle2)
    {
        _torus = new PrimitiveTorus(axes, r1, r2);
        _torus.VMin(angle1);
        _torus.VMax(angle2);
    }

    /// <summary>A torus ring and pipe segment on the axes.</summary>
    public MakeTorus(in Frame3D axes, double r1, double r2, double angle1, double angle2, double angle)
    {
        _torus = new PrimitiveTorus(axes, r1, r2);
        _torus.VMin(angle1);
        _torus.VMax(angle2);
        _torus.Angle(angle);
    }

    internal override OneAxis OneAxis()
    {
        return _torus;
    }

    /// <summary>The algorithm.</summary>
    internal PrimitiveTorus Torus()
    {
        return _torus;
    }
}
