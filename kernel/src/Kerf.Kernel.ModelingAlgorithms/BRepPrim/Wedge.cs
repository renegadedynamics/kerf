// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Wedge.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Wedge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>Provides the wedge constructors without Builders.</summary>
internal sealed class Wedge : GWedge
{
    /// <summary>Default constructor.</summary>
    public Wedge()
    {
    }

    /// <summary>A box of dimensions dx, dy, dz in the axes.</summary>
    public Wedge(in Frame3D axes, double dx, double dy, double dz)
        : base(new PrimitiveBuilder(), axes, dx, dy, dz)
    {
    }

    /// <summary>A STEP right angular wedge: X2Min = 0, X2Max = ltx.</summary>
    public Wedge(in Frame3D axes, double dx, double dy, double dz, double ltx)
        : base(new PrimitiveBuilder(), axes, dx, dy, dz, ltx)
    {
    }

    /// <summary>A wedge with all the fields given.</summary>
    public Wedge(in Frame3D axes,
                          double xmin,
                          double ymin,
                          double zmin,
                          double z2min,
                          double x2min,
                          double xmax,
                          double ymax,
                          double zmax,
                          double z2max,
                          double x2max)
        : base(new PrimitiveBuilder(), axes, xmin, ymin, zmin, z2min, x2min, xmax, ymax, zmax, z2max, x2max)
    {
    }
}
