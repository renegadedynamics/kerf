// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.hxx
// Port of: src/ModelingData/TKBRep/TopExp/TopExp_Explorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Visits a shape's structure depth first, finding the sub-shapes of a type (<c>ToFind</c>) that aren't inside a
/// sub-shape of another type (<c>ToAvoid</c>, SHAPE for no restriction).
/// </summary>
/// <remarks>
/// <code>
/// for (var Ex = new TopExp_Explorer(S, TopAbs_FACE); Ex.More(); Ex.Next())
/// {
///     ProcessFace(TopoDS.Face(Ex.Current()));
/// }
/// </code>
/// A type to avoid that is the same as, or simpler than, the type to find has no effect. A class (a working object),
/// where OCCT's is used by value; its stack of iterators is reused across <see cref="Init"/>. OCCT's range-for support
/// is left out.
/// </remarks>
public sealed class TopExp_Explorer
{
    // The DFS stack: iterators are kept when popped and reused when pushed again
    private readonly List<TopoDS_Iterator> myStack = [];
    private int myStackTop = -1;
    private TopoDS_Shape myShape;
    private TopAbs_ShapeEnum toFind;
    private TopAbs_ShapeEnum toAvoid;
    private bool hasMore;

    /// <summary>An empty explorer, useful after <see cref="Init"/>.</summary>
    public TopExp_Explorer()
    {
        myShape = new TopoDS_Shape();
        toFind = TopAbs_SHAPE;
        toAvoid = TopAbs_SHAPE;
        hasMore = false;
    }

    /// <summary>An explorer of <paramref name="S"/> for the sub-shapes of type ToFind not inside a ToAvoid.</summary>
    /// <param name="S">The shape to explore.</param>
    /// <param name="ToFind">The type of the shapes to find: VERTEX, EDGE, ...</param>
    /// <param name="ToAvoid">The type of the shapes to skip; no effect if SHAPE, or if the same as or simpler than ToFind.</param>
    public TopExp_Explorer(in TopoDS_Shape S, TopAbs_ShapeEnum ToFind, TopAbs_ShapeEnum ToAvoid = TopAbs_SHAPE)
    {
        myShape = new TopoDS_Shape();
        toFind = ToFind;
        toAvoid = ToAvoid;
        hasMore = false;
        Init(S, ToFind, ToAvoid);
    }

    /// <summary>Restarts the exploration on <paramref name="S"/>.</summary>
    /// <param name="S">The shape to explore.</param>
    /// <param name="ToFind">The type of the shapes to find: VERTEX, EDGE, ...</param>
    /// <param name="ToAvoid">The type of the shapes to skip; no effect if SHAPE, or if the same as or simpler than ToFind.</param>
    public void Init(in TopoDS_Shape S, TopAbs_ShapeEnum ToFind, TopAbs_ShapeEnum ToAvoid = TopAbs_SHAPE)
    {
        Clear();

        myShape = S;
        toFind = ToFind;
        toAvoid = ToAvoid;

        if (S.IsNull())
        {
            hasMore = false;
            return;
        }

        if (toFind == TopAbs_SHAPE)
        {
            hasMore = false;
        }
        else
        {
            TopAbs_ShapeEnum ty = S.ShapeType();

            if (ty > toFind)
            {
                hasMore = false;
            }
            else if (!isSameType(ty, toFind))
            {
                hasMore = true;
                Next();
            }
            else
            {
                hasMore = true;
            }
        }
    }

    /// <summary>True if there is a current shape.</summary>
    public bool More() => hasMore;

    /// <summary>Moves to the next shape.</summary>
    public void Next()
    {
        if (myStackTop < 0)
        {
            TopAbs_ShapeEnum ty = myShape.ShapeType();
            if (isSameType(ty, toFind))
            {
                hasMore = false;
                return;
            }
            else if (shouldAvoid(ty, toAvoid))
            {
                hasMore = false;
                return;
            }
            else
            {
                pushIterator(myShape);
            }
        }
        else
        {
            myStack[myStackTop].Next();
        }

        for (;;)
        {
            TopoDS_Iterator aTopIter = myStack[myStackTop];
            if (aTopIter.More())
            {
                TopoDS_Shape aShapTop = aTopIter.Value();
                TopAbs_ShapeEnum ty = aShapTop.ShapeType();
                if (isSameType(ty, toFind))
                {
                    hasMore = true;
                    return;
                }
                else if (isMoreComplex(ty, toFind) && !shouldAvoid(ty, toAvoid))
                {
                    pushIterator(aShapTop);
                }
                else
                {
                    aTopIter.Next();
                }
            }
            else
            {
                popIterator();
                if (myStackTop < 0)
                {
                    break;
                }

                myStack[myStackTop].Next();
            }
        }

        hasMore = false;
    }

    /// <summary>The current shape.</summary>
    public TopoDS_Shape Value() => Current();

    /// <summary>The current shape.</summary>
    public TopoDS_Shape Current() => myStackTop < 0 ? myShape : myStack[myStackTop].Value();

    /// <summary>Restarts the exploration with the same arguments.</summary>
    public void ReInit() => Init(myShape, toFind, toAvoid);

    /// <summary>The explored shape.</summary>
    public TopoDS_Shape ExploredShape() => myShape;

    /// <summary>The current depth of the exploration: 0 is the explored shape itself.</summary>
    public int Depth() => myStackTop + 1;

    /// <summary>Clears the explorer.</summary>
    public void Clear()
    {
        // Reset live iterators to release shape references, keeping them for reuse
        for (int i = myStackTop; i >= 0; --i)
        {
            myStack[i].Initialize(new TopoDS_Shape());
        }

        myStackTop = -1;
        hasMore = false;
    }

    // Pushes an iterator on the shape onto the stack, reusing a popped one.
    private void pushIterator(in TopoDS_Shape theShape)
    {
        ++myStackTop;
        if (myStackTop >= myStack.Count)
        {
            myStack.Add(new TopoDS_Iterator(theShape));
        }
        else
        {
            myStack[myStackTop].Initialize(theShape);
        }
    }

    // Pops the top iterator, resetting it to release its shape references.
    private void popIterator()
    {
        myStack[myStackTop].Initialize(new TopoDS_Shape());
        --myStackTop;
    }

    // Returns true if the given type matches the type to find.
    private static bool isSameType(TopAbs_ShapeEnum theType, TopAbs_ShapeEnum toFind) => toFind == theType;

    // Returns true if the given type should be avoided.
    private static bool shouldAvoid(TopAbs_ShapeEnum theType, TopAbs_ShapeEnum toAvoid)
        => toAvoid != TopAbs_SHAPE && toAvoid == theType;

    // Returns true if the given type is more complex than the type to find.
    private static bool isMoreComplex(TopAbs_ShapeEnum theType, TopAbs_ShapeEnum toFind) => toFind > theType;
}
