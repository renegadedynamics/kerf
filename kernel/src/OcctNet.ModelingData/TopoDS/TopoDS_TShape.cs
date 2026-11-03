// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

/// <summary>
/// The shared part of a shape: its sub-shapes (children, each with a location and an orientation) and its flags.
/// <see cref="TopoDS_Shape"/> values refer to a TShape and add a location and an orientation of their own.
/// </summary>
/// <remarks>
/// <para>
/// The flags are Free, Modified, Checked, Orientable, Closed, Infinite, Convex and Locked. Their getters are public;
/// their setters and the child list are internal to OCCT.NET.
/// </para>
/// <para>
/// A TShape freezes when the shape that holds it is handed to code outside OCCT.NET: when a public algorithm returns
/// it, or when it is read from a file (docs/phase3-design.md, section 4.2). A frozen TShape, its children and its
/// geometry never change again, and it is <see cref="Locked()"/>: OCCT's own guards then refuse to modify it.
/// </para>
/// </remarks>
public abstract class TopoDS_TShape : Standard_Transient
{
    /// <summary>
    /// Bit layout of the compact state: bits 0-3 store the TopAbs_ShapeEnum value, bits 4-11 the flags, bits 12-15
    /// are reserved.
    /// </summary>
    public enum BitLayout : ushort
    {
        Bits_ShapeType_Mask = 0x000F,
        Bits_ShapeType_Shift = 0,
        Bit_Free = 0x0010,
        Bit_Modified = 0x0020,
        Bit_Checked = 0x0040,
        Bit_Orientable = 0x0080,
        Bit_Closed = 0x0100,
        Bit_Infinite = 0x0200,
        Bit_Convex = 0x0400,
        Bit_Locked = 0x0800,
        Bits_Reserved = 0xF000,
    }

    /// <summary>The child shapes (OCCT: private, read by its friends TopoDS_Iterator and TopoDS_Builder).</summary>
    internal readonly NCollection_List<TopoDS_Shape> myShapes = new();

    private ushort myState;

    /// <summary>A TShape of the given type: Free, Modified and Orientable.</summary>
    protected TopoDS_TShape(TopAbs_ShapeEnum theType)
    {
        myState = (ushort)((ushort)theType | (ushort)BitLayout.Bit_Free | (ushort)BitLayout.Bit_Modified
                           | (ushort)BitLayout.Bit_Orientable);
    }

    /// <summary>The free flag: false once the TShape is a child of another one (its child list can't change).</summary>
    public bool Free() => (myState & (ushort)BitLayout.Bit_Free) != 0;

    internal void Free(bool theIsFree)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Free, theIsFree);
    }

    /// <summary>The locked flag. A frozen TShape is always locked (docs/phase3-design.md, section 4.3).</summary>
    public bool Locked() => (myState & (ushort)BitLayout.Bit_Locked) != 0 || IsFrozen();

    /// <summary>
    /// The stored locked bit, without the frozen state: what the shape sets write to files, so files carry OCCT's
    /// flags (OCCT.NET addition).
    /// </summary>
    internal bool LockedFlag() => (myState & (ushort)BitLayout.Bit_Locked) != 0;

    internal void Locked(bool theIsLocked)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Locked, theIsLocked);
    }

    /// <summary>The modification flag.</summary>
    public bool Modified() => (myState & (ushort)BitLayout.Bit_Modified) != 0;

    internal void Modified(bool theIsModified)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Modified, theIsModified);
        if (theIsModified)
        {
            // when a TShape is modified it is also unchecked
            setBit(BitLayout.Bit_Checked, false);
        }
    }

    /// <summary>The checked flag.</summary>
    public bool Checked() => (myState & (ushort)BitLayout.Bit_Checked) != 0;

    internal void Checked(bool theIsChecked)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Checked, theIsChecked);
    }

    /// <summary>The orientability flag.</summary>
    public bool Orientable() => (myState & (ushort)BitLayout.Bit_Orientable) != 0;

    internal void Orientable(bool theIsOrientable)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Orientable, theIsOrientable);
    }

    /// <summary>The closedness flag.</summary>
    public bool Closed() => (myState & (ushort)BitLayout.Bit_Closed) != 0;

    internal void Closed(bool theIsClosed)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Closed, theIsClosed);
    }

    /// <summary>The infinity flag.</summary>
    public bool Infinite() => (myState & (ushort)BitLayout.Bit_Infinite) != 0;

    internal void Infinite(bool theIsInfinite)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Infinite, theIsInfinite);
    }

    /// <summary>The convexness flag.</summary>
    public bool Convex() => (myState & (ushort)BitLayout.Bit_Convex) != 0;

    internal void Convex(bool theIsConvex)
    {
        CheckMutable();
        setBit(BitLayout.Bit_Convex, theIsConvex);
    }

    /// <summary>The type: VERTEX, EDGE, WIRE, FACE, SHELL, SOLID, COMPSOLID or COMPOUND.</summary>
    public TopAbs_ShapeEnum ShapeType() => (TopAbs_ShapeEnum)(myState & (ushort)BitLayout.Bits_ShapeType_Mask);

    /// <summary>A new, unfrozen TShape of the same type and geometry, with no sub-shapes.</summary>
    public abstract TopoDS_TShape EmptyCopy();

    /// <summary>The number of direct sub-shapes (children).</summary>
    public int NbChildren() => myShapes.Extent();

    private void setBit(BitLayout theBit, bool theIsOn)
    {
        if (theIsOn)
        {
            myState |= (ushort)theBit;
        }
        else
        {
            myState &= (ushort)~(ushort)theBit;
        }
    }

    // Freezing (OCCT.NET, docs/phase3-design.md, section 4.2).

    /// <summary>
    /// Freezes this TShape and every TShape reachable from it, with their representations, in post-order: the
    /// children first, then the representations, then the TShape itself. Stops at frozen TShapes, so a frozen
    /// TShape's whole sub-graph is always frozen, and the cost is proportional to the TShapes not yet frozen. Public for
    /// the modelling layer, which freezes what it builds with <see cref="TopoDS_Builder"/> (Kerf unit 10B; engine,
    /// internal, in unit 10L).
    /// </summary>
    public void FreezeGraph()
    {
        if (IsFrozen())
        {
            return;
        }

        // Iterative, so deep nesting can't overflow the stack. The set guards against cycles, which OCCT's Free
        // flag prevents but Free(true) could still create.
        var aStack = new List<FreezeFrame>();
        var anOnStack = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        aStack.Add(new FreezeFrame(this));
        anOnStack.Add(this);
        while (aStack.Count > 0)
        {
            ref FreezeFrame aTop = ref CollectionsMarshal.AsSpan(aStack)[^1];
            if (aTop.Children.MoveNext())
            {
                TopoDS_TShape? aChild = aTop.Children.Current.TShape();
                if (aChild is not null && !aChild.IsFrozen() && anOnStack.Add(aChild))
                {
                    aStack.Add(new FreezeFrame(aChild));
                }
            }
            else
            {
                TopoDS_TShape aTShape = aTop.TShape;
                aTShape.FreezeRepresentations();
                aTShape.Freeze();
                aStack.RemoveAt(aStack.Count - 1);
            }
        }
    }

    /// <summary>
    /// Freezes the TShape's own representations and the geometry they hold. BRep_TVertex, BRep_TEdge and BRep_TFace
    /// override it; the TShapes of TopoDS hold none.
    /// </summary>
    internal virtual void FreezeRepresentations()
    {
    }

    private struct FreezeFrame(TopoDS_TShape theTShape)
    {
        public readonly TopoDS_TShape TShape = theTShape;
        public List<TopoDS_Shape>.Enumerator Children = theTShape.myShapes.GetEnumerator();
    }
}
