// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeSolid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Library.LibraryShapeModification;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes a solid from shells, from a solid and shells, or from a compsolid (one shell of the faces that aren't
/// shared by two of its shells).
/// </summary>
/// <remarks>
/// The shells are taken as given: no check of closure or orientation. <see cref="Solid"/> freezes the solid
/// (design 4.2); ported code reads <see cref="InternalSolid"/>. Adding after a public read goes on with a new solid
/// holding the same shells.
/// </remarks>
internal class LibraryMakeSolid : LibraryMakeShape
{
    private readonly NCollection_List<Shape> _deletedFaces = new();

    /// <summary>Makes an empty solid.</summary>
    public LibraryMakeSolid()
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        Done();
    }

    /// <summary>Makes a solid from a compsolid: one shell of the faces that are in only one of its shells.</summary>
    public LibraryMakeSolid(in CompSolid s)
    {
        var b = new BrepBuilder();
        MakeSolid(b);

        Explorer ex1 = new(), ex2 = new();
        var mapOfFaces = new HashSet<Shape>(ShapeHasher.Instance);
        for (ex1.Init(s, ShapeType.Shell); ex1.More(); ex1.Next())
        {
            for (ex2.Init(ex1.Current(), ShapeType.Face); ex2.More(); ex2.Next())
            {
                if (!mapOfFaces.Add(ex2.Current()))
                {
                    _deletedFaces.Append(ex2.Current());
                    mapOfFaces.Remove(ex2.Current());
                }
            }
        }

        // OCCT iterates its hash map, in an order of addresses; the port's map iterates in its own order (decision 18),
        // so the faces of the shell may be in another order than native OCCT's.
        Shell shell = new();
        b.MakeShell(ref shell);
        foreach (Shape face in mapOfFaces)
        {
            b.Add(shell, face);
        }
        shell.Closed(Tool.IsClosed(shell));
        b.Add(_shape, shell);
        Done();
    }

    /// <summary>Makes a solid from a shell.</summary>
    public LibraryMakeSolid(in Shell s)
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        b.Add(_shape, s);
        Done();
    }

    /// <summary>Makes a solid from two shells.</summary>
    public LibraryMakeSolid(in Shell s1, in Shell s2)
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        b.Add(_shape, s1);
        b.Add(_shape, s2);
        Done();
    }

    /// <summary>Makes a solid from three shells.</summary>
    public LibraryMakeSolid(in Shell s1, in Shell s2, in Shell s3)
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        b.Add(_shape, s1);
        b.Add(_shape, s2);
        b.Add(_shape, s3);
        Done();
    }

    /// <summary>Makes a solid with the shells of a solid.</summary>
    public LibraryMakeSolid(in Solid so)
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        var ex = new Explorer();
        for (ex.Init(so, ShapeType.Shell); ex.More(); ex.Next())
        {
            b.Add(_shape, ex.Current());
        }
        Done();
    }

    /// <summary>Makes a solid with the shells of a solid and a shell.</summary>
    public LibraryMakeSolid(in Solid so, in Shell s)
    {
        var b = new BrepBuilder();
        MakeSolid(b);
        var ex = new Explorer();
        for (ex.Init(so, ShapeType.Shell); ex.More(); ex.Next())
        {
            b.Add(_shape, ex.Current());
        }
        b.Add(_shape, s);
        Done();
    }

    // B.MakeSolid(TopoDS::Solid(myShape))
    private void MakeSolid(BrepBuilder b)
    {
        Solid solid = new();
        b.MakeSolid(ref solid);
        _shape = solid;
    }

    /// <summary>Adds a shell to the solid.</summary>
    public void Add(in Shell s)
    {
        WorkingOverlay? overlay = BeginResultChange(); // Kerf: a published solid changes into a new one
        var b = new BrepBuilder();
        b.Add(_shape, s);
        EndResultChange(overlay);
    }

    /// <summary>The solid (frozen).</summary>
    public Solid Solid()
    {
        return ShapeCasts.Solid(Shape());
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal Solid InternalSolid()
    {
        return ShapeCasts.Solid(InternalShape());
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Solid(LibraryMakeSolid maker) => maker.Solid();

    /// <summary>Deleted for a face shared by two shells of the compsolid, otherwise preserved.</summary>
    public override LibraryShapeModification FaceStatus(in Face f)
    {
        LibraryShapeModification myStatus = LibraryShapeModification.Preserved;
        for (NCollection_List<Shape>.Iterator iter = new(_deletedFaces); iter.More(); iter.Next())
        {
            if (f.IsSame(iter.Value()))
            {
                myStatus = LibraryShapeModification.Deleted;
                break;
            }
        }
        return myStatus;
    }
}
