// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeSolid.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeSolid.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_ShapeModification;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds a solid from shells (the outer skin and the cavities), from a compsolid, or from a solid and shells to add.</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeSolid"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakeSolid : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeSolid myMakeSolid;

    // OCCT's "if (myMakeSolid.IsDone()) { Done(); myShape = myMakeSolid.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeSolid.IsDone())
        {
            Done();
            myShape = myMakeSolid.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeSolid()
    {
        myMakeSolid = new BRepLib_MakeSolid();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_CompSolid S)
    {
        myMakeSolid = new BRepLib_MakeSolid(S);
        TakeResult();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_Shell S)
    {
        myMakeSolid = new BRepLib_MakeSolid(S);
        TakeResult();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_Shell S1, in TopoDS_Shell S2)
    {
        myMakeSolid = new BRepLib_MakeSolid(S1, S2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_Shell S1, in TopoDS_Shell S2, in TopoDS_Shell S3)
    {
        myMakeSolid = new BRepLib_MakeSolid(S1, S2, S3);
        TakeResult();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_Solid So)
    {
        myMakeSolid = new BRepLib_MakeSolid(So);
        TakeResult();
    }

    public BRepBuilderAPI_MakeSolid(in TopoDS_Solid So, in TopoDS_Shell S)
    {
        myMakeSolid = new BRepLib_MakeSolid(So, S);
        TakeResult();
    }

    public void Add(in TopoDS_Shell S)
    {
        myMakeSolid.Add(S);
        TakeResult();
    }


    public override bool IsDone()
    {
        return myMakeSolid.IsDone();
    }

    /// <summary>The solid (frozen).</summary>
    public TopoDS_Solid Solid()
    {
        return myMakeSolid.Solid();
    }

    /// <summary><see cref="Solid"/> without freezing, for ported code.</summary>
    internal TopoDS_Solid InternalSolid()
    {
        return myMakeSolid.InternalSolid();
    }

    /// <summary>The solid (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Solid(BRepBuilderAPI_MakeSolid theMaker) => theMaker.Solid();

    /// <summary>True if the face <paramref name="S"/> was deleted (BRepLib's face status).</summary>
    public override bool IsDeleted(in TopoDS_Shape S)
    {
        if (S.ShapeType() == TopAbs_FACE)
        {
            BRepLib_ShapeModification aStatus = myMakeSolid.FaceStatus(TopoDS.Face(S));

            if (aStatus == BRepLib_Deleted)
            {
                return true;
            }
        }

        return false;
    }
}
