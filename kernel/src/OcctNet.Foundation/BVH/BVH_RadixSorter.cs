// Port of: src/FoundationClasses/TKMath/BVH/BVH_RadixSorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A pair of Morton code and primitive ID (OCCT's <c>std::pair&lt;unsigned int, int&gt;</c>).</summary>
public struct BVH_EncodedLink : IEquatable<BVH_EncodedLink>
{
    public uint first;
    public int second;

    public BVH_EncodedLink(uint theFirst, int theSecond)
    {
        first = theFirst;
        second = theSecond;
    }

    public readonly bool Equals(BVH_EncodedLink other) => first == other.first && second == other.second;

    public override readonly bool Equals(object? obj) => obj is BVH_EncodedLink other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(first, second);

    public static bool operator ==(BVH_EncodedLink theLeft, BVH_EncodedLink theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BVH_EncodedLink theLeft, BVH_EncodedLink theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"({first}, {second})";
}

public static partial class BVH
{
    /// <summary>Lookup table expanding an 8-bit value to a 24-bit Morton code component (every bit to every third position).</summary>
    public static ReadOnlySpan<uint> THE_MORTON_LUT =>
    [
        0x000000, 0x000001, 0x000008, 0x000009, 0x000040, 0x000041, 0x000048, 0x000049, 0x000200,
        0x000201, 0x000208, 0x000209, 0x000240, 0x000241, 0x000248, 0x000249, 0x001000, 0x001001,
        0x001008, 0x001009, 0x001040, 0x001041, 0x001048, 0x001049, 0x001200, 0x001201, 0x001208,
        0x001209, 0x001240, 0x001241, 0x001248, 0x001249, 0x008000, 0x008001, 0x008008, 0x008009,
        0x008040, 0x008041, 0x008048, 0x008049, 0x008200, 0x008201, 0x008208, 0x008209, 0x008240,
        0x008241, 0x008248, 0x008249, 0x009000, 0x009001, 0x009008, 0x009009, 0x009040, 0x009041,
        0x009048, 0x009049, 0x009200, 0x009201, 0x009208, 0x009209, 0x009240, 0x009241, 0x009248,
        0x009249, 0x040000, 0x040001, 0x040008, 0x040009, 0x040040, 0x040041, 0x040048, 0x040049,
        0x040200, 0x040201, 0x040208, 0x040209, 0x040240, 0x040241, 0x040248, 0x040249, 0x041000,
        0x041001, 0x041008, 0x041009, 0x041040, 0x041041, 0x041048, 0x041049, 0x041200, 0x041201,
        0x041208, 0x041209, 0x041240, 0x041241, 0x041248, 0x041249, 0x048000, 0x048001, 0x048008,
        0x048009, 0x048040, 0x048041, 0x048048, 0x048049, 0x048200, 0x048201, 0x048208, 0x048209,
        0x048240, 0x048241, 0x048248, 0x048249, 0x049000, 0x049001, 0x049008, 0x049009, 0x049040,
        0x049041, 0x049048, 0x049049, 0x049200, 0x049201, 0x049208, 0x049209, 0x049240, 0x049241,
        0x049248, 0x049249, 0x200000, 0x200001, 0x200008, 0x200009, 0x200040, 0x200041, 0x200048,
        0x200049, 0x200200, 0x200201, 0x200208, 0x200209, 0x200240, 0x200241, 0x200248, 0x200249,
        0x201000, 0x201001, 0x201008, 0x201009, 0x201040, 0x201041, 0x201048, 0x201049, 0x201200,
        0x201201, 0x201208, 0x201209, 0x201240, 0x201241, 0x201248, 0x201249, 0x208000, 0x208001,
        0x208008, 0x208009, 0x208040, 0x208041, 0x208048, 0x208049, 0x208200, 0x208201, 0x208208,
        0x208209, 0x208240, 0x208241, 0x208248, 0x208249, 0x209000, 0x209001, 0x209008, 0x209009,
        0x209040, 0x209041, 0x209048, 0x209049, 0x209200, 0x209201, 0x209208, 0x209209, 0x209240,
        0x209241, 0x209248, 0x209249, 0x240000, 0x240001, 0x240008, 0x240009, 0x240040, 0x240041,
        0x240048, 0x240049, 0x240200, 0x240201, 0x240208, 0x240209, 0x240240, 0x240241, 0x240248,
        0x240249, 0x241000, 0x241001, 0x241008, 0x241009, 0x241040, 0x241041, 0x241048, 0x241049,
        0x241200, 0x241201, 0x241208, 0x241209, 0x241240, 0x241241, 0x241248, 0x241249, 0x248000,
        0x248001, 0x248008, 0x248009, 0x248040, 0x248041, 0x248048, 0x248049, 0x248200, 0x248201,
        0x248208, 0x248209, 0x248240, 0x248241, 0x248248, 0x248249, 0x249000, 0x249001, 0x249008,
        0x249009, 0x249040, 0x249041, 0x249048, 0x249049, 0x249200, 0x249201, 0x249208, 0x249209,
        0x249240, 0x249241, 0x249248, 0x249249,
    ];

    /// <summary>Encodes 10-bit voxel coordinates (0-1023) into a 30-bit Morton code with interleaved bits.</summary>
    public static uint EncodeMortonCode(uint theVoxelX, uint theVoxelY, uint theVoxelZ)
    {
        // Split each 10-bit coordinate into two 8-bit lookups (upper 2 bits + lower 8 bits)
        ReadOnlySpan<uint> aLut = THE_MORTON_LUT;
        return (aLut[(int)(theVoxelX & 0xFF)] | (aLut[(int)((theVoxelX >> 8) & 0x03)] << 24))
               | ((aLut[(int)(theVoxelY & 0xFF)] | (aLut[(int)((theVoxelY >> 8) & 0x03)] << 24)) << 1)
               | ((aLut[(int)(theVoxelZ & 0xFF)] | (aLut[(int)((theVoxelZ >> 8) & 0x03)] << 24)) << 2);
    }

    /// <summary>
    /// <c>std::partition</c> of a range of links by the given bit (0-bit to the left side), as Microsoft's standard
    /// library does it (swapping from both ends), so the order of the links matches OCCT built with MSVC.
    /// </summary>
    internal static int PartitionLinks(BVH_EncodedLink[] theLinks, int theFirst, int theLast, int theDigit)
    {
        uint aBit = 1U << theDigit;
        for (;;)
        {
            // skip in-place elements at beginning
            for (;;)
            {
                if (theFirst == theLast)
                {
                    return theFirst;
                }

                if ((theLinks[theFirst].first & aBit) != 0)
                {
                    break;
                }

                ++theFirst;
            }

            // skip in-place elements at end
            do
            {
                --theLast;
                if (theFirst == theLast)
                {
                    return theFirst;
                }
            } while ((theLinks[theLast].first & aBit) != 0);

            (theLinks[theFirst], theLinks[theLast]) = (theLinks[theLast], theLinks[theFirst]); // out of place, swap and loop
            ++theFirst;
        }
    }

    /// <summary>
    /// Sorts the links of the (exclusive) range by Morton code with an MSD radix sort (<c>BVH::RadixSorter::Sort</c>):
    /// the top digits split in parallel when allowed, the rest sequentially.
    /// </summary>
    internal static void RadixSort(BVH_EncodedLink[] theLinks, int theStart, int theFinal, int theDigit, bool isParallel)
    {
        if (theDigit < 24)
        {
            RadixSortPerform(theLinks, theStart, theFinal, theDigit);
        }
        else
        {
            int anOffset = PartitionLinks(theLinks, theStart, theFinal, theDigit);
            (int Start, int Final, int Digit)[] aSplits = [(theStart, anOffset, theDigit - 1),
                                                           (anOffset, theFinal, theDigit - 1)];
            // OSD_Parallel::For(0, 2, Functor, !isParallel)
            OSD_Parallel.For(0, 2, theIndex => RadixSort(theLinks, aSplits[theIndex].Start, aSplits[theIndex].Final,
                                                         aSplits[theIndex].Digit, isParallel), !isParallel);
        }
    }

    // Performs MSD (most significant digit) radix sort.
    private static void RadixSortPerform(BVH_EncodedLink[] theLinks, int theStart, int theFinal, int theDigit = 29)
    {
        while (theStart != theFinal && theDigit >= 0)
        {
            int anOffset = PartitionLinks(theLinks, theStart, theFinal, theDigit--);
            RadixSortPerform(theLinks, theStart, anOffset, theDigit);
            theStart = anOffset;
        }
    }
}

/// <summary>Radix sort of a BVH primitive set by 10-bit Morton codes (a 1024 x 1024 x 1024 grid).</summary>
public class BVH_RadixSorter<T, TVec> : BVH_Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Axis-aligned bounding box (AABB) to perform sorting.</summary>
    protected BVH_Box<T, TVec> myBox;

    /// <summary>Morton codes assigned to BVH primitives.</summary>
    protected NCollection_Array1<BVH_EncodedLink>? myEncodedLinks;

    /// <summary>Creates a new radix sorter for the given AABB.</summary>
    public BVH_RadixSorter(in BVH_Box<T, TVec> theBox)
    {
        myBox = theBox;
    }

    /// <summary>Sorts the set.</summary>
    public override void Perform(BVH_Set<T, TVec> theSet) => Perform(theSet, 0, theSet.Size() - 1);

    /// <summary>The Morton codes assigned to the primitives (by the last sort).</summary>
    public NCollection_Array1<BVH_EncodedLink> EncodedLinks() => myEncodedLinks!;

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public override void Perform(BVH_Set<T, TVec> theSet, int theStart, int theFinal)
    {
        const int aDimension = 1024;
        int aNbEffComp = TVec.Length() == 2 ? 2 : 3; // 4th component is ignored

        TVec aSceneMin = myBox.CornerMin();
        TVec aSceneMax = myBox.CornerMax();

        TVec aNodeMinSizeVecT = TVec.Create(T.CreateTruncating(BVH.THE_NODE_MIN_SIZE));
        BVH.CwiseMax<T, TVec>(ref aNodeMinSizeVecT, aSceneMax - aSceneMin);

        TVec aReverseSize = TVec.Create(T.CreateTruncating(aDimension)) / aNodeMinSizeVecT;

        myEncodedLinks = new NCollection_Array1<BVH_EncodedLink>(theStart, theFinal);

        // Step 1 -- Assign Morton code to each primitive using LUT for faster encoding
        for (int aPrimIdx = theStart; aPrimIdx <= theFinal; ++aPrimIdx)
        {
            TVec aCenter = theSet.Box(aPrimIdx).Center();
            TVec aVoxelF = (aCenter - aSceneMin) * aReverseSize;

            // Compute voxel coordinates clamped to valid range
            int aVoxelX = Math.Clamp(BVH.IntFloor(BVH.VecComp<T, TVec>(aVoxelF, 0)), 0, aDimension - 1);
            int aVoxelY = Math.Clamp(BVH.IntFloor(BVH.VecComp<T, TVec>(aVoxelF, 1)), 0, aDimension - 1);
            int aVoxelZ = (aNbEffComp > 2)
                              ? Math.Clamp(BVH.IntFloor(BVH.VecComp<T, TVec>(aVoxelF, 2)), 0, aDimension - 1)
                              : 0;

            // Use LUT-based Morton code encoding for better performance
            uint aMortonCode = BVH.EncodeMortonCode((uint)aVoxelX, (uint)aVoxelY, (uint)aVoxelZ);

            myEncodedLinks[aPrimIdx] = new BVH_EncodedLink(aMortonCode, aPrimIdx);
        }

        // Step 2 -- Sort primitives by their Morton codes using radix sort
        // (on a copy of the links, which the parallel branches can share; copied back after)
        BVH_EncodedLink[] aLinks = myEncodedLinks.AsSpan().ToArray();
        BVH.RadixSort(aLinks, 0, aLinks.Length, 29, IsParallel());
        aLinks.CopyTo(myEncodedLinks.AsSpan());

        NCollection_Array1<int> aLinkMap = new(theStart, theFinal);
        for (int aLinkIdx = theStart; aLinkIdx <= theFinal; ++aLinkIdx)
        {
            aLinkMap[myEncodedLinks.Value(aLinkIdx).second] = aLinkIdx;
        }

        // Step 3 -- Rearranging primitive list according to Morton codes (in place)
        int aPrimIdx2 = theStart;
        while (aPrimIdx2 <= theFinal)
        {
            int aSortIdx = aLinkMap[aPrimIdx2];
            if (aPrimIdx2 != aSortIdx)
            {
                theSet.Swap(aPrimIdx2, aSortIdx);
                (aLinkMap[aPrimIdx2], aLinkMap[aSortIdx]) = (aLinkMap[aSortIdx], aLinkMap[aPrimIdx2]);
            }
            else
            {
                ++aPrimIdx2;
            }
        }
    }
}
