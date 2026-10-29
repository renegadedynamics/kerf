// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriPtr.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentTriPtr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A node of a circular, doubly linked list of triangles (<see cref="Poly_CoherentTriangle"/>), and the list itself:
/// the triangles around a <see cref="Poly_CoherentNode"/>.
/// </summary>
/// <remarks>OCCT's allocator arguments are dropped; removing a node unlinks it and leaves it to the garbage collector.</remarks>
public class Poly_CoherentTriPtr
{
    /// <summary>
    /// Iterates a list from any of its members, forward, until it comes back to that member.
    /// </summary>
    public sealed class Iterator
    {
        private readonly Poly_CoherentTriPtr? myFirst;
        private Poly_CoherentTriPtr? myCurrent;

        /// <summary>An empty iterator.</summary>
        public Iterator()
        {
            myFirst = null;
            myCurrent = null;
        }

        /// <summary>
        /// An iterator from <paramref name="thePtr"/>; null gives an empty iteration (OCCT dereferences the null list
        /// of a free node with the same effect).
        /// </summary>
        public Iterator(Poly_CoherentTriPtr? thePtr)
        {
            myFirst = thePtr;
            myCurrent = thePtr;
        }

        /// <summary>The triangle that started the iteration, or null.</summary>
        public Poly_CoherentTriangle? First() => myFirst?.GetTriangle();

        /// <summary>True if there is a current triangle.</summary>
        public bool More() => myCurrent is not null;

        /// <summary>Moves to the next triangle.</summary>
        public void Next()
        {
            if (myCurrent is not null)
            {
                myCurrent = myCurrent.myNext;
                if (myCurrent == myFirst)
                {
                    myCurrent = null;
                }
            }
        }

        /// <summary>The current triangle.</summary>
        public Poly_CoherentTriangle Value() => myCurrent!.GetTriangle();

        /// <summary>The current triangle, for writing.</summary>
        public Poly_CoherentTriangle ChangeValue() => myCurrent!.GetTriangle();

        /// <summary>The current list node.</summary>
        public Poly_CoherentTriPtr PtrValue() => myCurrent!;
    }

    private Poly_CoherentTriangle mypTriangle;
    private Poly_CoherentTriPtr myNext;
    private Poly_CoherentTriPtr myPrevious;

    /// <summary>A list of one triangle.</summary>
    public Poly_CoherentTriPtr(Poly_CoherentTriangle theTri)
    {
        mypTriangle = theTri;
        myNext = this;
        myPrevious = this;
    }

    /// <summary>The triangle.</summary>
    public Poly_CoherentTriangle GetTriangle() => mypTriangle;

    /// <summary>Sets the triangle.</summary>
    public void SetTriangle(Poly_CoherentTriangle pTri) => mypTriangle = pTri;

    /// <summary>The next node of the list.</summary>
    public Poly_CoherentTriPtr Next() => myNext;

    /// <summary>The previous node of the list.</summary>
    public Poly_CoherentTriPtr Previous() => myPrevious;

    /// <summary>Inserts a triangle after this node.</summary>
    public void Append(Poly_CoherentTriangle pTri)
    {
        var aNewPtr = new Poly_CoherentTriPtr(pTri);
        aNewPtr.myNext = myNext;
        myNext.myPrevious = aNewPtr;
        aNewPtr.myPrevious = this;
        myNext = aNewPtr;
    }

    /// <summary>Inserts a triangle before this node.</summary>
    public void Prepend(Poly_CoherentTriangle pTri)
    {
        var aNewPtr = new Poly_CoherentTriPtr(pTri);
        aNewPtr.myPrevious = myPrevious;
        myPrevious.myNext = aNewPtr;
        aNewPtr.myNext = this;
        myPrevious = aNewPtr;
    }

    /// <summary>Removes a node from its list.</summary>
    public static void Remove(Poly_CoherentTriPtr thePtr)
    {
        thePtr.myPrevious.myNext = thePtr.myNext;
        thePtr.myNext.myPrevious = thePtr.myPrevious;
        thePtr.myNext = thePtr;
        thePtr.myPrevious = thePtr;
        // OCCT frees the node here
    }

    /// <summary>Releases a whole list (OCCT frees its nodes; here the garbage collector does).</summary>
    public static void RemoveList(Poly_CoherentTriPtr? thePtr)
    {
        _ = thePtr;
    }
}
