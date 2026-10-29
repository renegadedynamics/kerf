// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;

namespace Occt;

/// <summary>The root of the BRepBuilderAPI algorithms that build a shape.</summary>
/// <remarks>
/// <para><see cref="Shape"/>, <see cref="Generated"/> and <see cref="Modified"/> are publication points
/// (docs/phase3-design.md, section 4.2): they freeze what they return. OCCT's virtuals are the internal forms
/// <see cref="InternalShape"/>, <see cref="InternalGenerated"/> and <see cref="InternalModified"/>, which don't freeze;
/// derived algorithms override those, and ported code that goes on building on a result reads them.</para>
/// <para>The algorithm is a <see cref="BRepTools_History.IAlgorithm"/>, so its history can be collected with
/// <c>new BRepTools_History(arguments, algorithm)</c> or merged with <see cref="BRepTools_History.Merge(NCollection_List{TopoDS_Shape}, BRepTools_History.IAlgorithm)"/>.</para>
/// </remarks>
public abstract class BRepBuilderAPI_MakeShape : BRepBuilderAPI_Command, BRepTools_History.IAlgorithm
{
    protected TopoDS_Shape myShape = new();
    protected NCollection_List<TopoDS_Shape> myGenerated = new();

    protected BRepBuilderAPI_MakeShape()
    {
    }

    /// <summary>This is called by <see cref="Shape"/>. It does nothing but may be redefined.</summary>
    public virtual void Build(Message_ProgressRange theRange = default)
    {
    }

    /// <summary>
    /// The shape built by the algorithm (frozen). Builds it if the command isn't done.
    /// </summary>
    /// <exception cref="StdFail_NotDone">The shape was not built.</exception>
    public TopoDS_Shape Shape()
    {
        TopoDS_Shape aShape = InternalShape();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="Shape"/> without freezing (OCCT's virtual), for ported code and derived algorithms.</summary>
    internal virtual TopoDS_Shape InternalShape()
    {
        if (!IsDone())
        {
            Build();
            Check();
        }
        return myShape;
    }

    /// <summary>The result (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Shape(BRepBuilderAPI_MakeShape theMaker) => theMaker.Shape();

    /// <summary>The shapes generated from the shape <paramref name="S"/> (frozen).</summary>
    public IBentoList<TopoDS_Shape> Generated(in TopoDS_Shape S) => PublishedList(InternalGenerated(S));

    /// <summary>The algorithm's own list behind <see cref="Generated"/>, unfrozen (OCCT's virtual).</summary>
    internal virtual NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        return myGenerated;
    }

    /// <summary>The shapes modified from the shape <paramref name="S"/> (frozen).</summary>
    public IBentoList<TopoDS_Shape> Modified(in TopoDS_Shape S) => PublishedList(InternalModified(S));

    /// <summary>The algorithm's own list behind <see cref="Modified"/>, unfrozen (OCCT's virtual).</summary>
    internal virtual NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape S)
    {
        myGenerated.Clear();
        return myGenerated;
    }

    /// <summary>True if the shape <paramref name="S"/> has been deleted.</summary>
    public virtual bool IsDeleted(in TopoDS_Shape S)
    {
        return false;
    }

    bool BRepTools_History.IAlgorithm.IsDeleted(in TopoDS_Shape theS) => IsDeleted(theS);

    IEnumerable<TopoDS_Shape> BRepTools_History.IAlgorithm.Modified(in TopoDS_Shape theS) => Modified(theS);

    IEnumerable<TopoDS_Shape> BRepTools_History.IAlgorithm.Generated(in TopoDS_Shape theS) => Generated(theS);

    // A frozen copy of a list of shapes, each shape's graph frozen.
    private protected static IBentoList<TopoDS_Shape> PublishedList(NCollection_List<TopoDS_Shape> theList)
    {
        if (theList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[theList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }
}
