// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeBox.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Primitives.PrimitiveDirection;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds parallelepiped boxes, with sides parallel to the axes of the global coordinate system or of a local one: with
/// a corner at (0, 0, 0) and of size (dx, dy, dz), with a corner at a point and of a size, or with corners at two
/// points. A negative size extends the box from the corner the other way (not with axes).
/// </summary>
/// <remarks>
/// <para>The shell, solid and faces it returns are frozen (design 4.2); the internal forms aren't, for ported code.
/// A face is handed out only once the whole shell is built (decision 467): in OCCT the face is completed later, in
/// place, by building the shell.</para>
/// <para>A flat box (a size not over Precision::Confusion()) throws Standard_DomainError when it is built.</para>
/// </remarks>
internal class MakeBox : MakeShape
{
    private protected Wedge _wedge;

    private static Point3D Pmin(in Point3D p, double dx, double dy, double dz)
    {
        Point3D P = p;
        if (dx < 0)
        {
            P = P.WithX(P.X + dx);
        }
        if (dy < 0)
        {
            P = P.WithY(P.Y + dy);
        }
        if (dz < 0)
        {
            P = P.WithZ(P.Z + dz);
        }
        return P;
    }

    private static Point3D Pmin(in Point3D p1, in Point3D p2)
    {
        return new Point3D(Min(p1.X, p2.X), Min(p1.Y, p2.Y), Min(p1.Z, p2.Z));
    }

    /// <summary>Default constructor.</summary>
    public MakeBox()
    {
        _wedge = new Wedge();
    }

    /// <summary>A box with a corner at 0,0,0 and the other at dx,dy,dz.</summary>
    public MakeBox(double dx, double dy, double dz)
    {
        _wedge = new Wedge(new Frame3D(Pmin(new Point3D(0, 0, 0), dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>A box with a corner at P and size dx, dy, dz.</summary>
    public MakeBox(in Point3D p, double dx, double dy, double dz)
    {
        _wedge = new Wedge(new Frame3D(Pmin(p, dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>A box with corners P1, P2.</summary>
    public MakeBox(in Point3D p1, in Point3D p2)
    {
        _wedge = new Wedge(new Frame3D(Pmin(p1, p2), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(p2.X - p1.X),
                                     Math.Abs(p2.Y - p1.Y),
                                     Math.Abs(p2.Z - p1.Z));
    }

    /// <summary>A box with Ax2 (the left corner and the axes) and size dx, dy, dz.</summary>
    public MakeBox(in Frame3D axes, double dx, double dy, double dz)
    {
        _wedge = new Wedge(axes, dx, dy, dz);
    }

    /// <summary>Inits a box with a corner at 0,0,0 and the other at theDX, theDY, theDZ.</summary>
    public void Init(double dx, double dy, double dz)
    {
        _wedge = new Wedge(new Frame3D(Pmin(new Point3D(0, 0, 0), dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>Inits a box with a corner at thePnt and size theDX, theDY, theDZ.</summary>
    public void Init(in Point3D pnt, double dx, double dy, double dz)
    {
        _wedge = new Wedge(new Frame3D(Pmin(pnt, dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>Inits a box with corners thePnt1, thePnt2.</summary>
    public void Init(in Point3D pnt1, in Point3D pnt2)
    {
        _wedge = new Wedge(new Frame3D(Pmin(pnt1, pnt2), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(pnt2.X - pnt1.X),
                                     Math.Abs(pnt2.Y - pnt1.Y),
                                     Math.Abs(pnt2.Z - pnt1.Z));
    }

    /// <summary>Inits a box with Ax2 (the left corner and the axes) and size theDX, theDY, theDZ.</summary>
    public void Init(in Frame3D axes, double dx, double dy, double dz)
    {
        _wedge = new Wedge(axes, dx, dy, dz);
    }

    /// <summary>The internal algorithm.</summary>
    internal Wedge Wedge()
    {
        return _wedge;
    }

    /// <summary>The box as a shell (frozen). Stores it as the result.</summary>
    public Shell Shell()
    {
        Shell shell = InternalShell();
        shell.TShape()?.FreezeGraph();
        return shell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        _shape = _wedge.Shell();
        Done();
        return ShapeCasts.Shell(_shape);
    }

    /// <summary>Stores the solid in the result.</summary>
    public override void Build(ProgressRange range = default)
    {
        InternalSolid();
    }

    /// <summary>The box as a solid (frozen). Makes a new solid holding the shell each time, as OCCT.</summary>
    public Solid Solid()
    {
        Solid solid = InternalSolid();
        solid.TShape()?.FreezeGraph();
        return solid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        var b = new BrepBuilder();
        // Kerf: a new solid in a new variable (OCCT remakes the one in myShape, decision 414).
        Solid solid = new();
        b.MakeSolid(ref solid);
        b.Add(solid, _wedge.Shell());
        _shape = solid;
        Done();
        return solid;
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(MakeBox maker) => maker.Shell();

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(MakeBox maker) => maker.Solid();

    /// <summary>The ZMin face (frozen).</summary>
    public Face BottomFace() => PublishedFace(PrimitiveDirection.BRepPrim_ZMin);

    /// <summary>The XMin face (frozen).</summary>
    public Face BackFace() => PublishedFace(PrimitiveDirection.BRepPrim_XMin);

    /// <summary>The XMax face (frozen).</summary>
    public Face FrontFace() => PublishedFace(PrimitiveDirection.BRepPrim_XMax);

    /// <summary>The YMin face (frozen).</summary>
    public Face LeftFace() => PublishedFace(PrimitiveDirection.BRepPrim_YMin);

    /// <summary>The YMax face (frozen).</summary>
    public Face RightFace() => PublishedFace(PrimitiveDirection.BRepPrim_YMax);

    /// <summary>The ZMax face (frozen).</summary>
    public Face TopFace() => PublishedFace(PrimitiveDirection.BRepPrim_ZMax);

    // Kerf: the face is published once every face of the box is built (decision 467): the other faces add pcurves
    // to its edges, which OCCT does in place after handing the face out. A degenerated box has no shell (Shell()
    // throws), so all six of its faces are built first, the six the accessors give (a face that fails to build is
    // left for its own accessor to throw).
    private Face PublishedFace(PrimitiveDirection direction)
    {
        if (!_wedge.IsDegeneratedShape)
        {
            _wedge.Shell();
        }
        else
        {
            foreach (PrimitiveDirection d in (ReadOnlySpan<PrimitiveDirection>)
                     [PrimitiveDirection.BRepPrim_XMin, PrimitiveDirection.BRepPrim_XMax, PrimitiveDirection.BRepPrim_YMin, PrimitiveDirection.BRepPrim_YMax, PrimitiveDirection.BRepPrim_ZMin, PrimitiveDirection.BRepPrim_ZMax])
            {
                try
                {
                    _wedge.Face(d);
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                }
            }
        }
        Face face = _wedge.Face(direction);
        face.TShape()?.FreezeGraph();
        return face;
    }
}
