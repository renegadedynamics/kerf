// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.lxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Creates topological data structures: the Make methods create shapes, Add puts a shape into another and Remove takes
/// it out. The root of the builders; BRep_Builder adds the shapes with geometry.
/// </summary>
/// <remarks>
/// <para>
/// A builder changes shapes in place. Callers build compounds with <see cref="MakeCompound(TopoDS_Shape[])"/>, which
/// returns a frozen compound, and other shapes through the public algorithms. The Make methods, Add and Remove are public
/// for the modelling layer (Kerf unit 10B; they become engine, internal, in unit 10L): a shape they build stays open until
/// the caller freezes it with <see cref="TopoDS_TShape.FreezeGraph"/>, before anything else reads it.
/// </para>
/// <para>
/// Add checks OCCT's rules: any shape goes into a COMPOUND; only a SOLID into a COMPSOLID; only a SHELL, an EDGE or a
/// VERTEX into a SOLID; only a FACE into a SHELL; only a WIRE or a VERTEX into a FACE; only an EDGE into a WIRE; only a
/// VERTEX into an EDGE; nothing into a VERTEX.
/// </para>
/// <para>
/// The Make methods take the shape by <c>ref</c> (C++ <c>T&amp;</c>; some of BRep_Builder's read it before replacing
/// it), so ported code writes <c>TopoDS_Wire W = new(); B.MakeWire(ref W);</c>.
/// </para>
/// </remarks>
public class TopoDS_Builder
{
    public TopoDS_Builder()
    {
    }

    /// <summary>
    /// A frozen compound of the given shapes (OCCT.NET addition, the public way to group shapes). The shapes' Free
    /// flags don't change (docs/phase3-design.md, section 4.4).
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    public static TopoDS_Compound MakeCompound(params TopoDS_Shape[] theShapes)
        => MakeCompound((IEnumerable<TopoDS_Shape>)theShapes);

    /// <summary>
    /// A frozen compound of the given shapes (OCCT.NET addition, the public way to group shapes). The shapes' Free
    /// flags don't change (docs/phase3-design.md, section 4.4).
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    public static TopoDS_Compound MakeCompound(IEnumerable<TopoDS_Shape> theShapes)
    {
        var aBuilder = new TopoDS_Builder();
        TopoDS_Compound aCompound = new TopoDS_Compound();
        aBuilder.MakeCompound(ref aCompound);
        foreach (TopoDS_Shape aShape in theShapes)
        {
            Standard_NullObject_Raise_if(aShape.IsNull(), "TopoDS_Builder::MakeCompound: a shape is null");
            aBuilder.Add(aCompound, aShape);
        }

        aCompound.TShape()!.FreezeGraph();
        return aCompound;
    }

    /// <summary>
    /// A frozen wire of the given edges, as OCCT's <c>MakeWire</c> and <c>Add</c> make it (OCCT.NET addition, decision
    /// 10404): no flag is set and no geometry is checked.
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    /// <exception cref="TopoDS_UnCompatibleShapes">A shape isn't an edge.</exception>
    public static TopoDS_Wire MakeWire(IEnumerable<TopoDS_Shape> theEdges)
    {
        TopoDS_Wire aWire = new TopoDS_Wire();
        new TopoDS_Builder().MakeWire(ref aWire);
        Fill(aWire, theEdges, "MakeWire");
        return aWire;
    }

    /// <summary>
    /// A frozen shell of the given faces, as OCCT's <c>MakeShell</c> and <c>Add</c> make it (OCCT.NET addition,
    /// decision 1104): no flag is set (it isn't marked closed) and no geometry is checked.
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    /// <exception cref="TopoDS_UnCompatibleShapes">A shape isn't a face.</exception>
    public static TopoDS_Shell MakeShell(IEnumerable<TopoDS_Shape> theFaces)
    {
        TopoDS_Shell aShell = new TopoDS_Shell();
        new TopoDS_Builder().MakeShell(ref aShell);
        Fill(aShell, theFaces, "MakeShell");
        return aShell;
    }

    /// <summary>
    /// A frozen solid of the given shells (or edges and vertices), as OCCT's <c>MakeSolid</c> and <c>Add</c> make it
    /// (OCCT.NET addition, decision 1104): the shells are taken as they are oriented, nothing is checked.
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    /// <exception cref="TopoDS_UnCompatibleShapes">A shape isn't a shell, an edge or a vertex.</exception>
    public static TopoDS_Solid MakeSolid(IEnumerable<TopoDS_Shape> theShells)
    {
        TopoDS_Solid aSolid = new TopoDS_Solid();
        new TopoDS_Builder().MakeSolid(ref aSolid);
        Fill(aSolid, theShells, "MakeSolid");
        return aSolid;
    }

    /// <summary>
    /// A frozen composite solid of the given solids, as OCCT's <c>MakeCompSolid</c> and <c>Add</c> make it (OCCT.NET
    /// addition, decision 1104).
    /// </summary>
    /// <exception cref="Standard_NullObject">A shape is null.</exception>
    /// <exception cref="TopoDS_UnCompatibleShapes">A shape isn't a solid.</exception>
    public static TopoDS_CompSolid MakeCompSolid(IEnumerable<TopoDS_Shape> theSolids)
    {
        TopoDS_CompSolid aCompSolid = new TopoDS_CompSolid();
        new TopoDS_Builder().MakeCompSolid(ref aCompSolid);
        Fill(aCompSolid, theSolids, "MakeCompSolid");
        return aCompSolid;
    }

    // Adds the shapes to the new shape and freezes it.
    private static void Fill(in TopoDS_Shape theShape, IEnumerable<TopoDS_Shape> theShapes, string theWhat)
    {
        var aBuilder = new TopoDS_Builder();
        foreach (TopoDS_Shape aComponent in theShapes)
        {
            Standard_NullObject_Raise_if(aComponent.IsNull(), $"TopoDS_Builder::{theWhat}: a shape is null");
            aBuilder.Add(theShape, aComponent);
        }

        theShape.TShape()!.FreezeGraph();
    }

    /// <summary>Makes an empty wire.</summary>
    public void MakeWire(ref TopoDS_Wire W)
    {
        TopoDS_TWire TW = new TopoDS_TWire();
        MakeShape(ref TopoDS.AsShapeRef(ref W), TW);
    }

    /// <summary>Makes an empty shell.</summary>
    public void MakeShell(ref TopoDS_Shell S)
    {
        TopoDS_TShell TS = new TopoDS_TShell();
        MakeShape(ref TopoDS.AsShapeRef(ref S), TS);
    }

    /// <summary>Makes a solid covering the whole 3D space.</summary>
    public void MakeSolid(ref TopoDS_Solid S)
    {
        TopoDS_TSolid TS = new TopoDS_TSolid();
        MakeShape(ref TopoDS.AsShapeRef(ref S), TS);
    }

    /// <summary>Makes an empty composite solid.</summary>
    public void MakeCompSolid(ref TopoDS_CompSolid C)
    {
        TopoDS_TCompSolid TC = new TopoDS_TCompSolid();
        MakeShape(ref TopoDS.AsShapeRef(ref C), TC);
    }

    /// <summary>Makes an empty compound.</summary>
    public void MakeCompound(ref TopoDS_Compound C)
    {
        TopoDS_TCompound TC = new TopoDS_TCompound();
        MakeShape(ref TopoDS.AsShapeRef(ref C), TC);
    }

    /// <summary>
    /// Adds the shape <paramref name="aComponent"/> to <paramref name="aShape"/>. Only the TShape of aShape changes,
    /// so it is taken by <c>in</c>.
    /// </summary>
    /// <exception cref="TopoDS_FrozenShape">aShape isn't free.</exception>
    /// <exception cref="TopoDS_UnCompatibleShapes">aShape can't contain aComponent.</exception>
    /// <exception cref="Standard_ImmutableObject">aShape is frozen.</exception>
    public void Add(in TopoDS_Shape aShape, in TopoDS_Shape aComponent)
    {
        // From now the Component cannot be edited
        // OCCT.NET: frozen: a frozen component keeps the Free flag it was frozen with, so its BRep text never
        // changes (docs/phase3-design.md, section 4.4; OCCT clears it).
        TopoDS_TShape aComponentTShape = aComponent.TShape()
                                         ?? throw new Standard_NullObject("TopoDS_Builder::Add: the component is null");
        if (!aComponentTShape.IsFrozen())
        {
            aComponentTShape.Free(false);
        }

        // Note that freezing aComponent before testing if aShape is free
        // prevents from self-insertion
        // but aShape will be frozen when the Exception is raised
        if (aShape.Free())
        {
            uint iC = (uint)aComponent.ShapeType();
            uint iS = (uint)aShape.ShapeType();
            if ((aTb[iC] & (1u << (int)iS)) != 0)
            {
                TopoDS_Shape aChild = aComponent;
                // compute the relative Orientation
                if (aShape.Orientation() == TopAbs_REVERSED)
                {
                    aChild.Reverse();
                }

                // and the Relative Location
                TopLoc_Location aLoc = aShape.Location();
                if (!aLoc.IsIdentity())
                {
                    aChild.Move(aLoc.Inverted(), false);
                }

                // Add to the subshapes list
                TopoDS_TShape aTShape = aShape.TShape()!;
                aTShape.CheckMutable(); // OCCT.NET: frozen
                aTShape.myShapes.Append(aChild);
                aTShape.Modified(true);
            }
            else
            {
                throw new TopoDS_UnCompatibleShapes("TopoDS_Builder::Add");
            }
        }
        else
        {
            throw new TopoDS_FrozenShape("TopoDS_Builder::Add");
        }
    }

    // The types each shape type can be added to, as bit masks of TopAbs_ShapeEnum
    private static readonly uint[] aTb =
    [
        // COMPOUND to:
        1u << (int)TopAbs_COMPOUND,
        // COMPSOLID to:
        1u << (int)TopAbs_COMPOUND,
        // SOLID to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_COMPSOLID),
        // SHELL to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_SOLID),
        // FACE to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_SHELL),
        // WIRE to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_FACE),
        // EDGE to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_SOLID) | (1u << (int)TopAbs_WIRE),
        // VERTEX to:
        (1u << (int)TopAbs_COMPOUND) | (1u << (int)TopAbs_SOLID) | (1u << (int)TopAbs_FACE) | (1u << (int)TopAbs_EDGE),
        // SHAPE to:
        0,
    ];

    /// <summary>
    /// Removes the shape <paramref name="aComponent"/> from <paramref name="aShape"/>. Only the TShape of aShape
    /// changes, so it is taken by <c>in</c>.
    /// </summary>
    /// <exception cref="TopoDS_FrozenShape">aShape isn't free.</exception>
    /// <exception cref="Standard_ImmutableObject">aShape is frozen.</exception>
    public void Remove(in TopoDS_Shape aShape, in TopoDS_Shape aComponent)
    {
        // check if aShape  is not Frozen
        TopoDS_Raise.TopoDS_FrozenShape_Raise_if(!aShape.Free(), "TopoDS_Builder::Remove");

        // compute the relative Orientation and Location of aComponent
        TopoDS_Shape S = aComponent;
        if (aShape.Orientation() == TopAbs_REVERSED)
        {
            S.Reverse();
        }

        S.Location(S.Location().Predivided(aShape.Location()), false);

        TopoDS_TShape aTShape = aShape.TShape()!;
        NCollection_List<TopoDS_Shape> aList = aTShape.myShapes;
        var anIter = new NCollection_List<TopoDS_Shape>.Iterator(aList);
        while (anIter.More())
        {
            if (anIter.Value() == S)
            {
                aTShape.CheckMutable(); // OCCT.NET: frozen
                aList.Remove(anIter);
                aTShape.Modified(true);
                return;
            }

            anIter.Next();
        }
    }

    /// <summary>The basic method to make a shape, used by all the Make methods.</summary>
    private protected static void MakeShape(ref TopoDS_Shape S, TopoDS_TShape T)
    {
        S.TShape(T);
        S.Location(new TopLoc_Location());
        S.Orientation(TopAbs_FORWARD);
    }
}
