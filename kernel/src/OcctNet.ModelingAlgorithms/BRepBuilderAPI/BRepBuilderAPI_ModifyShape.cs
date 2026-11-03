// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ModifyShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ModifyShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The constant topology modifications done with a <see cref="BRepTools_Modifier"/>: <see cref="BRepBuilderAPI_Copy"/>,
/// <see cref="BRepBuilderAPI_Transform"/>, <see cref="BRepBuilderAPI_GTransform"/>, <see cref="BRepBuilderAPI_NurbsConvert"/>.
/// </summary>
/// <remarks>
/// The input is only read: the modifier builds new shapes and reuses the sub-shapes it leaves alone. The result is
/// read through the modifier's <c>InternalModifiedShape</c> and frozen by <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and <see cref="ModifiedShape"/> (design 4.2), so a derived algorithm can go on updating it first (NurbsConvert's
/// vertex tolerances).
/// </remarks>
public abstract class BRepBuilderAPI_ModifyShape : BRepBuilderAPI_MakeShape
{
    protected BRepTools_Modifier myModifier;
    protected TopoDS_Shape myInitialShape = new();
    protected BRepTools_Modification? myModification;

    /// <summary>Empty constructor.</summary>
    protected BRepBuilderAPI_ModifyShape()
    {
        myModifier = new BRepTools_Modifier();
    }

    /// <summary>Initializes the modifier with the shape <paramref name="S"/>.</summary>
    protected BRepBuilderAPI_ModifyShape(in TopoDS_Shape S)
    {
        myModifier = new BRepTools_Modifier(S);
        myInitialShape = S;
    }

    /// <summary>Sets the modification.</summary>
    protected BRepBuilderAPI_ModifyShape(BRepTools_Modification M)
    {
        myModifier = new BRepTools_Modifier();
        myModification = M;
    }

    /// <summary>Initializes the modifier with the shape <paramref name="S"/> and performs the modification <paramref name="M"/>.</summary>
    protected BRepBuilderAPI_ModifyShape(in TopoDS_Shape S, BRepTools_Modification M)
    {
        myModifier = new BRepTools_Modifier(S);
        myInitialShape = S;
        myModification = M;
        DoModif();
    }

    private void DoModif()
    {
        if (myInitialShape.IsNull() || myModification is null)
        {
            throw new NullObjectException();
        }
        myModifier.Perform(myModification);
        if (myModifier.IsDone())
        {
            Done();
            myShape = myModifier.InternalModifiedShape(myInitialShape);
        }
        else
        {
            NotDone();
        }
    }

    /// <summary>Performs the previously given modification on the shape <paramref name="S"/>.</summary>
    protected void DoModif(in TopoDS_Shape S)
    {
        if (!S.IsEqual(myInitialShape) || !IsDone())
        {
            myInitialShape = S;
            myModifier.Init(S);
            DoModif();
        }
    }

    /// <summary>Performs the modification <paramref name="M"/> on the previously given shape.</summary>
    protected void DoModif(BRepTools_Modification M)
    {
        myModification = M;
        DoModif();
    }

    /// <summary>Performs the modification <paramref name="M"/> on the shape <paramref name="S"/>.</summary>
    protected void DoModif(in TopoDS_Shape S, BRepTools_Modification M)
    {
        myInitialShape = S;
        myModifier.Init(S);
        myModification = M;
        DoModif();
    }

    /// <summary>
    /// The modified shape corresponding to <paramref name="S"/>, the initial shape or one of its sub-shapes (frozen).
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="S"/> isn't the initial shape or one of its sub-shapes.</exception>
    public TopoDS_Shape ModifiedShape(in TopoDS_Shape S)
    {
        TopoDS_Shape aShape = InternalModifiedShape(S);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="ModifiedShape"/> without freezing (OCCT's virtual), for ported code.</summary>
    internal virtual TopoDS_Shape InternalModifiedShape(in TopoDS_Shape S)
    {
        return myModifier.InternalModifiedShape(S);
    }

    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape F)
    {
        myGenerated.Clear();
        myGenerated.Append(myModifier.InternalModifiedShape(F));
        return myGenerated;
    }
}
