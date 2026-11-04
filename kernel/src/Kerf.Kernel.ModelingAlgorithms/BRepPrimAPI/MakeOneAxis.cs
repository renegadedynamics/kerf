// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeOneAxis.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeOneAxis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>The root of the makers of primitives with one axis of revolution (cylinder, cone, sphere, torus, revolution).</summary>
/// <remarks>
/// The face, shell and solid it returns are frozen (design 4.2); the internal forms aren't, for ported code. Each of
/// them builds the primitive first, so a face is handed out complete.
/// </remarks>
internal abstract class MakeOneAxis : MakeShape
{
    private protected MakeOneAxis()
    {
    }

    /// <summary>The algorithm (OCCT's <c>void* OneAxis()</c>).</summary>
    internal abstract OneAxis OneAxis();

    /// <summary>Stores the solid in the result (a new solid holding the shell each time, as OCCT).</summary>
    public override void Build(ProgressRange range = default)
    {
        var b = new BrepBuilder();
        // OCCT.NET: a new solid in a new variable (OCCT remakes the one in myShape, decision 414).
        Solid solid = new();
        b.MakeSolid(ref solid);
        b.Add(solid, OneAxis().Shell());
        _shape = solid;
        Done();
    }

    /// <summary>The lateral face of the rotational primitive (frozen).</summary>
    public Face Face()
    {
        Face face = InternalFace();
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace()
    {
        Build();
        return OneAxis().LateralFace();
    }

    /// <summary>The rotational primitive as a shell (frozen).</summary>
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
        return OneAxis().Shell();
    }

    /// <summary>The rotational primitive as a solid (frozen).</summary>
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

    /// <summary>The lateral face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Face(MakeOneAxis maker) => maker.Face();

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(MakeOneAxis maker) => maker.Shell();

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(MakeOneAxis maker) => maker.Solid();
}
