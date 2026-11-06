// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeWedge.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeWedge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Foundation;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds wedges: boxes with inclined faces.</summary>
/// <remarks>The shell and solid it returns are frozen (design 4.2); the internal forms aren't, for ported code.</remarks>
internal class MakeWedge : MakeShape
{
    private readonly Wedge _wedge;

    /// <summary>A STEP right angular wedge (ltx >= 0).</summary>
    public MakeWedge(double dx, double dy, double dz, double ltx)
    {
        _wedge = new Wedge(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), dx, dy, dz, ltx);
    }

    /// <summary>A STEP right angular wedge (ltx >= 0) in the axes.</summary>
    public MakeWedge(in Frame3D axes, double dx, double dy, double dz, double ltx)
    {
        _wedge = new Wedge(axes, dx, dy, dz, ltx);
    }

    /// <summary>A wedge whose face at dy is xmin,zmin xmax,zmax.</summary>
    public MakeWedge(double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax)
    {
        _wedge = new Wedge(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     0,
                                     0,
                                     0,
                                     zmin,
                                     xmin,
                                     dx,
                                     dy,
                                     dz,
                                     zmax,
                                     xmax);
    }

    /// <summary>A wedge whose face at dy is xmin,zmin xmax,zmax, in the axes.</summary>
    public MakeWedge(in Frame3D axes, double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax)
    {
        _wedge = new Wedge(axes, 0, 0, 0, zmin, xmin, dx, dy, dz, zmax, xmax);
    }

    /// <summary>The internal algorithm.</summary>
    internal Wedge Wedge()
    {
        return _wedge;
    }

    /// <summary>The wedge as a shell (frozen).</summary>
    public Shell Shell()
    {
        Shell shell = InternalShell();
        shell.TShape()?.FreezeGraph();
        return shell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        Build();
        return _wedge.Shell();
    }

    /// <summary>Stores the solid in the result (a new solid holding the shell each time, as OCCT).</summary>
    public override void Build(ProgressRange range = default)
    {
        var b = new BrepBuilder();
        // Kerf: a new solid in a new variable (OCCT remakes the one in myShape, decision 414).
        Solid solid = new();
        b.MakeSolid(ref solid);
        b.Add(solid, _wedge.Shell());
        _shape = solid;
        Done();
    }

    /// <summary>The wedge as a solid (frozen).</summary>
    public Solid Solid()
    {
        Solid solid = InternalSolid();
        solid.TShape()?.FreezeGraph();
        return solid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        Build();
        return ShapeCasts.Solid(_shape);
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(MakeWedge maker) => maker.Shell();

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(MakeWedge maker) => maker.Solid();
}
