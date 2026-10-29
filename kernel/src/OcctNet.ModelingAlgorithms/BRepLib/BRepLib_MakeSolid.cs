// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeSolid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_ShapeModification;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Makes a solid from shells, from a solid and shells, or from a compsolid (one shell of the faces that aren't
/// shared by two of its shells).
/// </summary>
/// <remarks>
/// The shells are taken as given: no check of closure or orientation. <see cref="Solid"/> freezes the solid
/// (design 4.2); ported code reads <see cref="InternalSolid"/>. Adding after a public read goes on with a new solid
/// holding the same shells.
/// </remarks>
public class BRepLib_MakeSolid : BRepLib_MakeShape
{
    private readonly NCollection_List<TopoDS_Shape> myDeletedFaces = new();

    /// <summary>Makes an empty solid.</summary>
    public BRepLib_MakeSolid()
    {
        var B = new BRep_Builder();
        makeSolid(B);
        Done();
    }

    /// <summary>Makes a solid from a compsolid: one shell of the faces that are in only one of its shells.</summary>
    public BRepLib_MakeSolid(in TopoDS_CompSolid S)
    {
        var B = new BRep_Builder();
        makeSolid(B);

        TopExp_Explorer ex1 = new(), ex2 = new();
        var aMapOfFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (ex1.Init(S, TopAbs_SHELL); ex1.More(); ex1.Next())
        {
            for (ex2.Init(ex1.Current(), TopAbs_FACE); ex2.More(); ex2.Next())
            {
                if (!aMapOfFaces.Add(ex2.Current()))
                {
                    myDeletedFaces.Append(ex2.Current());
                    aMapOfFaces.Remove(ex2.Current());
                }
            }
        }

        // OCCT iterates its hash map, in an order of addresses; the port's map iterates in its own order (decision 18),
        // so the faces of the shell may be in another order than native OCCT's.
        TopoDS_Shell aShell = new();
        B.MakeShell(ref aShell);
        foreach (TopoDS_Shape aFace in aMapOfFaces)
        {
            B.Add(aShell, aFace);
        }
        aShell.Closed(BRep_Tool.IsClosed(aShell));
        B.Add(myShape, aShell);
        Done();
    }

    /// <summary>Makes a solid from a shell.</summary>
    public BRepLib_MakeSolid(in TopoDS_Shell S)
    {
        var B = new BRep_Builder();
        makeSolid(B);
        B.Add(myShape, S);
        Done();
    }

    /// <summary>Makes a solid from two shells.</summary>
    public BRepLib_MakeSolid(in TopoDS_Shell S1, in TopoDS_Shell S2)
    {
        var B = new BRep_Builder();
        makeSolid(B);
        B.Add(myShape, S1);
        B.Add(myShape, S2);
        Done();
    }

    /// <summary>Makes a solid from three shells.</summary>
    public BRepLib_MakeSolid(in TopoDS_Shell S1, in TopoDS_Shell S2, in TopoDS_Shell S3)
    {
        var B = new BRep_Builder();
        makeSolid(B);
        B.Add(myShape, S1);
        B.Add(myShape, S2);
        B.Add(myShape, S3);
        Done();
    }

    /// <summary>Makes a solid with the shells of a solid.</summary>
    public BRepLib_MakeSolid(in TopoDS_Solid So)
    {
        var B = new BRep_Builder();
        makeSolid(B);
        var ex = new TopExp_Explorer();
        for (ex.Init(So, TopAbs_SHELL); ex.More(); ex.Next())
        {
            B.Add(myShape, ex.Current());
        }
        Done();
    }

    /// <summary>Makes a solid with the shells of a solid and a shell.</summary>
    public BRepLib_MakeSolid(in TopoDS_Solid So, in TopoDS_Shell S)
    {
        var B = new BRep_Builder();
        makeSolid(B);
        var ex = new TopExp_Explorer();
        for (ex.Init(So, TopAbs_SHELL); ex.More(); ex.Next())
        {
            B.Add(myShape, ex.Current());
        }
        B.Add(myShape, S);
        Done();
    }

    // B.MakeSolid(TopoDS::Solid(myShape))
    private void makeSolid(BRep_Builder B)
    {
        TopoDS_Solid aSolid = new();
        B.MakeSolid(ref aSolid);
        myShape = aSolid;
    }

    /// <summary>Adds a shell to the solid.</summary>
    public void Add(in TopoDS_Shell S)
    {
        UnfreezeResult(); // OCCT.NET: frozen
        var B = new BRep_Builder();
        B.Add(myShape, S);
    }

    /// <summary>The solid (frozen).</summary>
    public TopoDS_Solid Solid()
    {
        return TopoDS.Solid(Shape());
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal TopoDS_Solid InternalSolid()
    {
        return TopoDS.Solid(InternalShape());
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Solid(BRepLib_MakeSolid theMaker) => theMaker.Solid();

    /// <summary>Deleted for a face shared by two shells of the compsolid, otherwise preserved.</summary>
    public override BRepLib_ShapeModification FaceStatus(in TopoDS_Face F)
    {
        BRepLib_ShapeModification myStatus = BRepLib_Preserved;
        for (NCollection_List<TopoDS_Shape>.Iterator anIter = new(myDeletedFaces); anIter.More(); anIter.Next())
        {
            if (F.IsSame(anIter.Value()))
            {
                myStatus = BRepLib_Deleted;
                break;
            }
        }
        return myStatus;
    }
}
