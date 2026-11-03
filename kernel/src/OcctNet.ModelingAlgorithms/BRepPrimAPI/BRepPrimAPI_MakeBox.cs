// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeBox.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepPrim_Direction;

namespace Occt;

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
public class BRepPrimAPI_MakeBox : BRepBuilderAPI_MakeShape
{
    private protected BRepPrim_Wedge myWedge;

    private static Point3D pmin(in Point3D p, double dx, double dy, double dz)
    {
        Point3D P = p;
        if (dx < 0)
        {
            P.SetX(P.X() + dx);
        }
        if (dy < 0)
        {
            P.SetY(P.Y() + dy);
        }
        if (dz < 0)
        {
            P.SetZ(P.Z() + dz);
        }
        return P;
    }

    private static Point3D pmin(in Point3D p1, in Point3D p2)
    {
        return new Point3D(Min(p1.X(), p2.X()), Min(p1.Y(), p2.Y()), Min(p1.Z(), p2.Z()));
    }

    /// <summary>Default constructor.</summary>
    public BRepPrimAPI_MakeBox()
    {
        myWedge = new BRepPrim_Wedge();
    }

    /// <summary>A box with a corner at 0,0,0 and the other at dx,dy,dz.</summary>
    public BRepPrimAPI_MakeBox(double dx, double dy, double dz)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(new Point3D(0, 0, 0), dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>A box with a corner at P and size dx, dy, dz.</summary>
    public BRepPrimAPI_MakeBox(in Point3D P, double dx, double dy, double dz)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(P, dx, dy, dz), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(dx),
                                     Math.Abs(dy),
                                     Math.Abs(dz));
    }

    /// <summary>A box with corners P1, P2.</summary>
    public BRepPrimAPI_MakeBox(in Point3D P1, in Point3D P2)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(P1, P2), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(P2.X() - P1.X()),
                                     Math.Abs(P2.Y() - P1.Y()),
                                     Math.Abs(P2.Z() - P1.Z()));
    }

    /// <summary>A box with Ax2 (the left corner and the axes) and size dx, dy, dz.</summary>
    public BRepPrimAPI_MakeBox(in Frame3D Axes, double dx, double dy, double dz)
    {
        myWedge = new BRepPrim_Wedge(Axes, dx, dy, dz);
    }

    /// <summary>Inits a box with a corner at 0,0,0 and the other at theDX, theDY, theDZ.</summary>
    public void Init(double theDX, double theDY, double theDZ)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(new Point3D(0, 0, 0), theDX, theDY, theDZ), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(theDX),
                                     Math.Abs(theDY),
                                     Math.Abs(theDZ));
    }

    /// <summary>Inits a box with a corner at thePnt and size theDX, theDY, theDZ.</summary>
    public void Init(in Point3D thePnt, double theDX, double theDY, double theDZ)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(thePnt, theDX, theDY, theDZ), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(theDX),
                                     Math.Abs(theDY),
                                     Math.Abs(theDZ));
    }

    /// <summary>Inits a box with corners thePnt1, thePnt2.</summary>
    public void Init(in Point3D thePnt1, in Point3D thePnt2)
    {
        myWedge = new BRepPrim_Wedge(new Frame3D(pmin(thePnt1, thePnt2), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                     Math.Abs(thePnt2.X() - thePnt1.X()),
                                     Math.Abs(thePnt2.Y() - thePnt1.Y()),
                                     Math.Abs(thePnt2.Z() - thePnt1.Z()));
    }

    /// <summary>Inits a box with Ax2 (the left corner and the axes) and size theDX, theDY, theDZ.</summary>
    public void Init(in Frame3D theAxes, double theDX, double theDY, double theDZ)
    {
        myWedge = new BRepPrim_Wedge(theAxes, theDX, theDY, theDZ);
    }

    /// <summary>The internal algorithm.</summary>
    internal BRepPrim_Wedge Wedge()
    {
        return myWedge;
    }

    /// <summary>The box as a shell (frozen). Stores it as the result.</summary>
    public Shell Shell()
    {
        Shell aShell = InternalShell();
        aShell.TShape()?.FreezeGraph();
        return aShell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        myShape = myWedge.Shell();
        Done();
        return ShapeCasts.Shell(myShape);
    }

    /// <summary>Stores the solid in the result.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        InternalSolid();
    }

    /// <summary>The box as a solid (frozen). Makes a new solid holding the shell each time, as OCCT.</summary>
    public Solid Solid()
    {
        Solid aSolid = InternalSolid();
        aSolid.TShape()?.FreezeGraph();
        return aSolid;
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        var B = new BrepBuilder();
        // OCCT.NET: a new solid in a new variable (OCCT remakes the one in myShape, decision 414).
        Solid aSolid = new();
        B.MakeSolid(ref aSolid);
        B.Add(aSolid, myWedge.Shell());
        myShape = aSolid;
        Done();
        return aSolid;
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(BRepPrimAPI_MakeBox theMaker) => theMaker.Shell();

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(BRepPrimAPI_MakeBox theMaker) => theMaker.Solid();

    /// <summary>The ZMin face (frozen).</summary>
    public Face BottomFace() => PublishedFace(BRepPrim_ZMin);

    /// <summary>The XMin face (frozen).</summary>
    public Face BackFace() => PublishedFace(BRepPrim_XMin);

    /// <summary>The XMax face (frozen).</summary>
    public Face FrontFace() => PublishedFace(BRepPrim_XMax);

    /// <summary>The YMin face (frozen).</summary>
    public Face LeftFace() => PublishedFace(BRepPrim_YMin);

    /// <summary>The YMax face (frozen).</summary>
    public Face RightFace() => PublishedFace(BRepPrim_YMax);

    /// <summary>The ZMax face (frozen).</summary>
    public Face TopFace() => PublishedFace(BRepPrim_ZMax);

    // OCCT.NET: the face is published once every face of the box is built (decision 467): the other faces add pcurves
    // to its edges, which OCCT does in place after handing the face out. A degenerated box has no shell (Shell()
    // throws), so all six of its faces are built first, the six the accessors give (a face that fails to build is
    // left for its own accessor to throw).
    private Face PublishedFace(BRepPrim_Direction theDirection)
    {
        if (!myWedge.IsDegeneratedShape())
        {
            myWedge.Shell();
        }
        else
        {
            foreach (BRepPrim_Direction aD in (ReadOnlySpan<BRepPrim_Direction>)
                     [BRepPrim_XMin, BRepPrim_XMax, BRepPrim_YMin, BRepPrim_YMax, BRepPrim_ZMin, BRepPrim_ZMax])
            {
                try
                {
                    myWedge.Face(aD);
                }
                catch (FailureException)
                {
                }
            }
        }
        Face aFace = myWedge.Face(theDirection);
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }
}
