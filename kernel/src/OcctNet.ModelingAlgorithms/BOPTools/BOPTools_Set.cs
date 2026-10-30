// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Set.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Set.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The set of the sub-shapes of a given type of a shape (degenerated edges left out, INTERNAL ones taken both
/// FORWARD and REVERSED), compared as a set of shapes: BOPAlgo keys maps by it to find shapes made of the same
/// sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// A working object filled by <see cref="Add"/> and then used as a map key (decision 6432), so a sealed class
/// with OCCT's copy constructor and <see cref="Assign"/> written out where OCCT copies by value.
/// </para>
/// <para>
/// Equality is OCCT's <see cref="IsEqual"/>: the same number of shapes, and every shape of the other set
/// <see cref="TopoDS_Shape.IsSame"/> as one of this set. The hash is OCCT's <see cref="GetSum"/>, a sum of the shapes'
/// normalised <see cref="TopTools_ShapeMapHasher"/> hashes. The hash values differ from native's (native hashes
/// addresses), which changes no result: the port's maps iterate in insertion order (decision 18).
/// </para>
/// </remarks>
public sealed class BOPTools_Set : IEquatable<BOPTools_Set>
{
    private NCollection_DynamicArray<TopoDS_Shape> myShapes;
    private TopoDS_Shape myShape;
    private int myNbShapes;
    private ulong mySum;
    private int myUpper;

    /// <summary>An empty set.</summary>
    public BOPTools_Set()
    {
        myShapes = new NCollection_DynamicArray<TopoDS_Shape>(256);
        myNbShapes = 0;
        mySum = 0;
        myUpper = 432123;
    }

    /// <summary>A copy of the set (OCCT's copy constructor).</summary>
    public BOPTools_Set(BOPTools_Set theOther)
    {
        myShape = theOther.myShape;
        myNbShapes = theOther.myNbShapes;
        mySum = theOther.mySum;
        myUpper = theOther.myUpper;
        myShapes = new NCollection_DynamicArray<TopoDS_Shape>(theOther.myShapes);
    }

    /// <summary>Makes this set a copy of the other (OCCT's assignment).</summary>
    public BOPTools_Set Assign(BOPTools_Set theOther)
    {
        myShape = theOther.myShape;
        myNbShapes = theOther.myNbShapes;
        mySum = theOther.mySum;
        myUpper = theOther.myUpper;
        myShapes = new NCollection_DynamicArray<TopoDS_Shape>(theOther.myShapes);
        return this;
    }

    /// <summary>The shape the set was made from.</summary>
    public TopoDS_Shape Shape() => myShape;

    /// <summary>
    /// Fills the set with the sub-shapes of type <paramref name="theType"/> of <paramref name="theS"/>, replacing its
    /// content. Don't call it on a set that is a key of a map.
    /// </summary>
    public void Add(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        ulong aId, aIdN;
        TopAbs_Orientation aOr;
        TopExp_Explorer aExp = new();
        //
        myShape = theS;
        myShapes.Clear();
        myNbShapes = 0;
        mySum = 0;
        //
        aExp.Init(theS, theType);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aSx = aExp.Current();
            if (theType == TopAbs_EDGE)
            {
                TopoDS_Edge aEx = TopoDS.Edge(aSx);
                if (BRep_Tool.Degenerated(aEx))
                {
                    continue;
                }
            }
            //
            aOr = aSx.Orientation();
            if (aOr == TopAbs_INTERNAL)
            {
                TopoDS_Shape aSy;
                //
                aSy = aSx;
                //
                aSy.Orientation(TopAbs_FORWARD);
                myShapes.Append(aSy);
                //
                aSy.Orientation(TopAbs_REVERSED);
                myShapes.Append(aSy);
            }
            else
            {
                myShapes.Append(aSx);
            }
        }
        //
        myNbShapes = myShapes.Length();
        if (myNbShapes == 0)
        {
            return;
        }
        //
        for (int i = 0; i < myShapes.Length(); ++i)
        {
            TopoDS_Shape aSx = myShapes.Value(i);
            // OCCT's size_t hash; the port's is an int, taken as unsigned
            aId = unchecked((ulong)(uint)TopTools_ShapeMapHasher.Instance.GetHashCode(aSx)) % (ulong)myUpper + 1;
            aIdN = NormalizedIds(aId, myNbShapes);
            mySum = unchecked(mySum + aIdN);
        }
    }

    /// <summary>The number of shapes in the set (INTERNAL ones count twice).</summary>
    public int NbShapes() => myNbShapes;

    /// <summary>True if the sets have the same number of shapes and every shape of the other is in this one (by IsSame).</summary>
    public bool IsEqual(BOPTools_Set aOther)
    {
        if (aOther.myNbShapes != myNbShapes)
        {
            return false;
        }
        //
        NCollection_Map<TopoDS_Shape> aM1 = new(TopTools_ShapeMapHasher.Instance);
        //
        for (int i = 0; i < myShapes.Length(); ++i)
        {
            aM1.Add(myShapes.Value(i));
        }
        //
        for (int i = 0; i < aOther.myShapes.Length(); ++i)
        {
            if (!aM1.Contains(aOther.myShapes.Value(i)))
            {
                return false;
            }
        }
        //
        return true;
    }

    /// <summary>The hash of the set: the sum of its shapes' normalised hashes.</summary>
    public ulong GetSum() => mySum;

    /// <summary>Empties the set.</summary>
    private void Clear()
    {
        myNbShapes = 0;
        mySum = 0;
        myShapes.Clear();
    }

    private static ulong NormalizedIds(ulong aId, int aDiv)
    {
        ulong aMax, aTresh, aIdRet;
        //
        aIdRet = aId;
        aMax = ulong.MaxValue;
        aTresh = aMax / (ulong)aDiv;
        if (aId > aTresh)
        {
            aIdRet = aId % aTresh;
        }
        return aIdRet;
    }

    /// <inheritdoc/>
    public bool Equals(BOPTools_Set? theOther) => theOther is not null && IsEqual(theOther);

    /// <inheritdoc/>
    public override bool Equals(object? theObj) => theObj is BOPTools_Set aSet && IsEqual(aSet);

    /// <summary>OCCT's <c>std::hash</c>: <see cref="GetSum"/>, folded.</summary>
    public override int GetHashCode() => mySum.GetHashCode();
}
