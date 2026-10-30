// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_PairSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Selects the pairs of elements of two BVH trees whose boxes meet.</summary>
/// <remarks>
/// OCCT's <c>RejectNode</c> builds a <c>BVH_Box&lt;double, 3&gt;</c> whatever the dimension (only the 3D selector is
/// instantiated in OCCT); the port uses the selector's own dimension, the same thing in 3D.
/// </remarks>
public class BOPTools_PairSelector<TVec> : BVH_PairTraverse<double, TVec, BVH_BoxSet<double, TVec, int>, double>
    where TVec : unmanaged, INCollection_Vec<double, TVec>
{
    /// <summary>A pair of element indices.</summary>
    public struct PairIDs : IComparable<PairIDs>, IEquatable<PairIDs>
    {
        public int ID1;
        public int ID2;

        public PairIDs()
            : this(-1, -1)
        {
        }

        public PairIDs(int theId1, int theId2 = -1)
        {
            ID1 = theId1;
            ID2 = theId2;
        }

        /// <summary>OCCT's <c>operator&lt;</c>: by the first index, then the second.</summary>
        public static bool operator <(PairIDs theLeft, PairIDs theOther) =>
            theLeft.ID1 < theOther.ID1 || (theLeft.ID1 == theOther.ID1 && theLeft.ID2 < theOther.ID2);

        public static bool operator >(PairIDs theLeft, PairIDs theOther) => theOther < theLeft;

        public readonly int CompareTo(PairIDs theOther) => this < theOther ? -1 : (theOther < this ? 1 : 0);

        public readonly bool Equals(PairIDs theOther) => ID1 == theOther.ID1 && ID2 == theOther.ID2;

        public override readonly bool Equals(object? theObj) => theObj is PairIDs anOther && Equals(anOther);

        public override readonly int GetHashCode() => HashCode.Combine(ID1, ID2);

        public static bool operator ==(PairIDs theLeft, PairIDs theRight) => theLeft.Equals(theRight);

        public static bool operator !=(PairIDs theLeft, PairIDs theRight) => !theLeft.Equals(theRight);
    }

    protected NCollection_LinearVector<PairIDs> myPairs = new();    //!< Selected pairs of indices
    protected bool mySameBVHs;                                       //!< Selection is performed from the same BVH trees

    public BOPTools_PairSelector()
    {
        mySameBVHs = false;
    }

    /// <summary>Clears the indices.</summary>
    public void Clear() => myPairs.Clear();

    /// <summary>
    /// Sorts the pairs (OCCT's <c>std::sort</c>). The pairs are distinct, so the order doesn't depend on the sort
    /// algorithm: decision 491's tie order doesn't arise.
    /// </summary>
    public void Sort()
    {
        int aNb = myPairs.Size();
        if (aNb > 1)
        {
            PairIDs[] aPairs = new PairIDs[aNb];
            for (int i = 0; i < aNb; ++i)
            {
                aPairs[i] = myPairs.Value(i);
            }
            Array.Sort(aPairs);
            for (int i = 0; i < aNb; ++i)
            {
                myPairs[i] = aPairs[i];
            }
        }
    }

    /// <summary>
    /// Tells the selector that the trees are the same: then only unique pairs are kept ((1, 2) but not (2, 1)) and no
    /// pair of an element with itself.
    /// </summary>
    public void SetSame(bool theIsSame) => mySameBVHs = theIsSame;

    /// <summary>The accepted pairs.</summary>
    public NCollection_LinearVector<PairIDs> Pairs() => myPairs;

    /// <summary>Rejects a pair of nodes by their boxes.</summary>
    public override bool RejectNode(in TVec theCMin1, in TVec theCMax1, in TVec theCMin2, in TVec theCMax2, out double theMetric)
    {
        theMetric = 0.0;
        return new BVH_Box<double, TVec>(theCMin1, theCMax1).IsOut(theCMin2, theCMax2);
    }

    /// <summary>Checks if the pair of elements should be rejected.</summary>
    public bool RejectElement(int theID1, int theID2)
    {
        return (mySameBVHs && theID1 >= theID2) || myBVHSet1!.Box(theID1).IsOut(myBVHSet2!.Box(theID2));
    }

    /// <summary>Checks and accepts the pair of elements.</summary>
    public override bool Accept(int theID1, int theID2)
    {
        if (!RejectElement(theID1, theID2))
        {
            myPairs.Append(new PairIDs(myBVHSet1!.Element(theID1), myBVHSet2!.Element(theID2)));
            return true;
        }
        return false;
    }
}
