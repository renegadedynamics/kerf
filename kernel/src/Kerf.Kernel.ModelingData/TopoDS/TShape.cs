// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShape.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_TShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// The shared part of a shape: its sub-shapes (children, each with a location and an orientation) and its flags.
/// <see cref="Shape"/> values refer to a TShape and add a location and an orientation of their own.
/// </summary>
/// <remarks>
/// <para>
/// The flags are Free, Modified, Checked, Orientable, Closed, Infinite, Convex and Locked. Their getters are public;
/// their setters and the child list are internal to Kerf.
/// </para>
/// <para>
/// A TShape freezes when the shape that holds it is handed to code outside Kerf: when a public algorithm returns
/// it, or when it is read from a file (docs/phase3-design.md, section 4.2). A frozen TShape, its children and its
/// geometry never change again, and it is <see cref="Locked()"/>: OCCT's own guards then refuse to modify it.
/// </para>
/// </remarks>
internal abstract class TShape : Transient
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
    internal NCollection_List<Shape> _shapes = new();

    private ushort _state;

    /// <summary>
    /// The identity hash of shape maps (unit 10J; performance.md's proposal 7): drawn once at creation, in place of
    /// <see cref="System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(object)"/>, which took 2 to 5% of the
    /// Booleans' time. The maps iterate in insertion order whatever the hash, so no result depends on its value.
    /// </summary>
    internal readonly int _hash = Random.Shared.Next();

    /// <summary>A TShape of the given type: Free, Modified and Orientable.</summary>
    protected TShape(ShapeType type)
    {
        _state = (ushort)((ushort)type | (ushort)BitLayout.Bit_Free | (ushort)BitLayout.Bit_Modified
                           | (ushort)BitLayout.Bit_Orientable);
    }

    /// <summary>The free flag: false once the TShape is a child of another one (its child list can't change).</summary>
    public bool Free() => (State & (ushort)BitLayout.Bit_Free) != 0;

    internal void Free(bool isFree)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Free(isFree);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Free, isFree);
    }

    /// <summary>The locked flag. A frozen TShape is always locked (docs/phase3-design.md, section 4.3).</summary>
    public bool Locked() => (State & (ushort)BitLayout.Bit_Locked) != 0 || IsFrozen;

    /// <summary>
    /// The stored locked bit, without the frozen state: what the shape sets write to files, so files carry OCCT's
    /// flags (Kerf addition).
    /// </summary>
    internal bool LockedFlag() => (State & (ushort)BitLayout.Bit_Locked) != 0;

    internal void Locked(bool isLocked)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Locked(isLocked);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Locked, isLocked);
    }

    /// <summary>The modification flag.</summary>
    public bool Modified() => (State & (ushort)BitLayout.Bit_Modified) != 0;

    internal void Modified(bool isModified)
    {
        if (Modified() == isModified && WorkingOverlay.TakesWrite(this))
        {
            return; // Kerf: the flag as it is (unit 10H3)
        }
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Modified(isModified);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Modified, isModified);
        if (isModified)
        {
            // when a TShape is modified it is also unchecked
            SetBit(BitLayout.Bit_Checked, false);
        }
    }

    /// <summary>The checked flag.</summary>
    public bool Checked() => (State & (ushort)BitLayout.Bit_Checked) != 0;

    internal void Checked(bool isChecked)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Checked(isChecked);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Checked, isChecked);
    }

    /// <summary>The orientability flag.</summary>
    public bool Orientable() => (State & (ushort)BitLayout.Bit_Orientable) != 0;

    internal void Orientable(bool isOrientable)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Orientable(isOrientable);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Orientable, isOrientable);
    }

    /// <summary>The closedness flag.</summary>
    public bool Closed() => (State & (ushort)BitLayout.Bit_Closed) != 0;

    internal void Closed(bool isClosed)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Closed(isClosed);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Closed, isClosed);
    }

    /// <summary>The infinity flag.</summary>
    public bool Infinite() => (State & (ushort)BitLayout.Bit_Infinite) != 0;

    internal void Infinite(bool isInfinite)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Infinite(isInfinite);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Infinite, isInfinite);
    }

    /// <summary>The convexness flag.</summary>
    public bool Convex() => (State & (ushort)BitLayout.Bit_Convex) != 0;

    internal void Convex(bool isConvex)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            working.Convex(isConvex);
            return;
        }

        CheckMutable();
        SetBit(BitLayout.Bit_Convex, isConvex);
    }

    /// <summary>The type: VERTEX, EDGE, WIRE, FACE, SHELL, SOLID, COMPSOLID or COMPOUND.</summary>
    public ShapeType ShapeType() => (ShapeType)(_state & (ushort)BitLayout.Bits_ShapeType_Mask);

    /// <summary>A new, unfrozen TShape of the same type and geometry, with no sub-shapes.</summary>
    public abstract TShape EmptyCopy();

    /// <summary>The number of direct sub-shapes (children).</summary>
    public int ChildCount => WorkingOverlay.View(this)._shapes.Extent();

    // The flags as the current working overlay sees them (WorkingOverlay; the shape type never changes).
    private ushort State => WorkingOverlay.TryView(this, out TShape? view) ? view._state : _state;

    private void SetBit(BitLayout bit, bool isOn)
    {
        if (isOn)
        {
            _state |= (ushort)bit;
        }
        else
        {
            _state &= (ushort)~(ushort)bit;
        }
    }

    // Freezing (Kerf, docs/phase3-design.md, section 4.2).

    /// <summary>
    /// Freezes this TShape and every TShape reachable from it, with their representations, in post-order: the
    /// children first, then the representations, then the TShape itself. Stops at frozen TShapes, so a frozen
    /// TShape's whole sub-graph is always frozen, and the cost is proportional to the TShapes not yet frozen. Public for
    /// the modelling layer, which freezes what it builds with <see cref="Builder"/> (Kerf unit 10B; engine,
    /// internal, in unit 10L).
    /// </summary>
    public void FreezeGraph()
    {
        if (IsFrozen)
        {
            return;
        }

        // Nearly always every child is already frozen: a new wire of frozen edges, an edge of frozen vertices. The walk
        // below would visit only this one, so it isn't set up
        bool isWalkNeeded = false;
        foreach (Shape aChild in _shapes)
        {
            TShape? aChildT = aChild.TShape();
            if (aChildT is not null && !aChildT.IsFrozen)
            {
                isWalkNeeded = true;
                break;
            }
        }
        if (!isWalkNeeded)
        {
            FreezeOne();
            return;
        }

        // Iterative, so deep nesting can't overflow the stack. The set guards against cycles, which OCCT's Free
        // flag prevents but Free(true) could still create.
        var stack = new List<FreezeFrame>();
        var onStack = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        stack.Add(new FreezeFrame(this));
        onStack.Add(this);
        while (stack.Count > 0)
        {
            ref FreezeFrame top = ref CollectionsMarshal.AsSpan(stack)[^1];
            if (top.Children.MoveNext())
            {
                TShape? child = top.Children.Current.TShape();
                if (child is not null && !child.IsFrozen && onStack.Add(child))
                {
                    stack.Add(new FreezeFrame(child));
                }
            }
            else
            {
                top.TShape.FreezeOne();
                stack.RemoveAt(stack.Count - 1);
            }
        }
    }

    // Its lists won't grow now, and a kept shape holds them for hours, so they let go of the room they had for growing
    private void FreezeOne()
    {
        _shapes = _shapes.Finished();
        FreezeRepresentations();
        Freeze();
    }

    /// <summary>
    /// Freezes the TShape's own representations and the geometry they hold. BRep_TVertex, BRep_TEdge and BRep_TFace
    /// override it; the TShapes of TopoDS hold none.
    /// </summary>
    internal virtual void FreezeRepresentations()
    {
    }

    private struct FreezeFrame(TShape theTShape)
    {
        public readonly TShape TShape = theTShape;
        public NCollection_List<Shape>.Enumerator Children = theTShape._shapes.GetEnumerator();
    }
}
