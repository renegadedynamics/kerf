// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_PairSelector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>Selects the pairs of elements of two BVH trees whose boxes meet.</summary>
/// <remarks>
/// OCCT's <c>RejectNode</c> builds a <c>BVH_Box&lt;double, 3&gt;</c> whatever the dimension (only the 3D selector is
/// instantiated in OCCT); the port uses the selector's own dimension, the same thing in 3D.
/// </remarks>
internal class PairSelector<TVec> : PairTraverse<double, TVec, BoxSet<double, TVec, int>, double>
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

        public PairIDs(int id1, int id2 = -1)
        {
            ID1 = id1;
            ID2 = id2;
        }

        /// <summary>OCCT's <c>operator&lt;</c>: by the first index, then the second.</summary>
        public static bool operator <(PairIDs left, PairIDs other) =>
            left.ID1 < other.ID1 || (left.ID1 == other.ID1 && left.ID2 < other.ID2);

        public static bool operator >(PairIDs left, PairIDs other) => other < left;

        public readonly int CompareTo(PairIDs other) => this < other ? -1 : (other < this ? 1 : 0);

        public readonly bool Equals(PairIDs other) => ID1 == other.ID1 && ID2 == other.ID2;

        public override readonly bool Equals(object? obj) => obj is PairIDs other && Equals(other);

        public override readonly int GetHashCode() => HashCode.Combine(ID1, ID2);

        public static bool operator ==(PairIDs left, PairIDs right) => left.Equals(right);

        public static bool operator !=(PairIDs left, PairIDs right) => !left.Equals(right);
    }

    protected NCollection_LinearVector<PairIDs> _pairs = new();    //!< Selected pairs of indices
    protected bool _sameBVHs;                                       //!< Selection is performed from the same BVH trees

    public PairSelector()
    {
        _sameBVHs = false;
    }

    /// <summary>Clears the indices.</summary>
    public void Clear() => _pairs.Clear();

    /// <summary>
    /// Sorts the pairs (OCCT's <c>std::sort</c>). The pairs are distinct, so the order doesn't depend on the sort
    /// algorithm: decision 491's tie order doesn't arise.
    /// </summary>
    public void Sort()
    {
        int nb = _pairs.Size();
        if (nb > 1)
        {
            PairIDs[] pairs = new PairIDs[nb];
            for (int i = 0; i < nb; ++i)
            {
                pairs[i] = _pairs.Value(i);
            }
            Array.Sort(pairs);
            for (int i = 0; i < nb; ++i)
            {
                _pairs[i] = pairs[i];
            }
        }
    }

    /// <summary>
    /// Tells the selector that the trees are the same: then only unique pairs are kept ((1, 2) but not (2, 1)) and no
    /// pair of an element with itself.
    /// </summary>
    public void SetSame(bool isSame) => _sameBVHs = isSame;

    /// <summary>The accepted pairs.</summary>
    public NCollection_LinearVector<PairIDs> Pairs() => _pairs;

    /// <summary>Rejects a pair of nodes by their boxes.</summary>
    public override bool RejectNode(in TVec cMin1, in TVec cMax1, in TVec cMin2, in TVec cMax2, out double metric)
    {
        metric = 0.0;
        return new Box<double, TVec>(cMin1, cMax1).IsOut(cMin2, cMax2);
    }

    /// <summary>Checks if the pair of elements should be rejected.</summary>
    public bool RejectElement(int id1, int id2)
    {
        return (_sameBVHs && id1 >= id2) || _bvhSet1!.Box(id1).IsOut(_bvhSet2!.Box(id2));
    }

    /// <summary>Checks and accepts the pair of elements.</summary>
    public override bool Accept(int id1, int id2)
    {
        if (!RejectElement(id1, id2))
        {
            _pairs.Append(new PairIDs(_bvhSet1!.Element(id1), _bvhSet2!.Element(id2)));
            return true;
        }
        return false;
    }
}
