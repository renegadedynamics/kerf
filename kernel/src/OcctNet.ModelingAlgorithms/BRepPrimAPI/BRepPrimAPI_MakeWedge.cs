// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeWedge.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeWedge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>Builds wedges: boxes with inclined faces.</summary>
/// <remarks>The shell and solid it returns are frozen (design 4.2); the internal forms aren't, for ported code.</remarks>
public class BRepPrimAPI_MakeWedge : BRepBuilderAPI_MakeShape
{
    private readonly BRepPrim_Wedge myWedge;

    /// <summary>A STEP right angular wedge (ltx >= 0).</summary>
    public BRepPrimAPI_MakeWedge(double dx, double dy, double dz, double ltx)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), dx, dy, dz, ltx);
    }

    /// <summary>A STEP right angular wedge (ltx >= 0) in the axes.</summary>
    public BRepPrimAPI_MakeWedge(in Frame3D Axes, double dx, double dy, double dz, double ltx)
    {
        myWedge = new BRepPrim_Wedge(Axes, dx, dy, dz, ltx);
    }

    /// <summary>A wedge whose face at dy is xmin,zmin xmax,zmax.</summary>
    public BRepPrimAPI_MakeWedge(double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
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
    public BRepPrimAPI_MakeWedge(in Frame3D Axes, double dx, double dy, double dz, double xmin, double zmin, double xmax, double zmax)
    {
        myWedge = new BRepPrim_Wedge(Axes, 0, 0, 0, zmin, xmin, dx, dy, dz, zmax, xmax);
    }

    /// <summary>The internal algorithm.</summary>
    internal BRepPrim_Wedge Wedge()
    {
        return myWedge;
    }

    /// <summary>The wedge as a shell (frozen).</summary>
    public TopoDS_Shell Shell()
    {
        TopoDS_Shell aShell = InternalShell();
        aShell.TShape()?.FreezeGraph();
        return aShell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal TopoDS_Shell InternalShell()
    {
        Build();
        return myWedge.Shell();
    }

    /// <summary>Stores the solid in the result (a new solid holding the shell each time, as OCCT).</summary>
    public override void Build(ProgressRange theRange = default)
    {
        var B = new BRep_Builder();
        // OCCT.NET: a new solid in a new variable (OCCT remakes the one in myShape, decision 414).
        TopoDS_Solid aSolid = new();
        B.MakeSolid(ref aSolid);
        B.Add(aSolid, myWedge.Shell());
        myShape = aSolid;
        Done();
    }

    /// <summary>The wedge as a solid (frozen).</summary>
    public TopoDS_Solid Solid()
    {
        TopoDS_Solid aSolid = InternalSolid();
        aSolid.TShape()?.FreezeGraph();
        return aSolid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal TopoDS_Solid InternalSolid()
    {
        Build();
        return TopoDS.Solid(myShape);
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Shell(BRepPrimAPI_MakeWedge theMaker) => theMaker.Shell();

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Solid(BRepPrimAPI_MakeWedge theMaker) => theMaker.Solid();
}
