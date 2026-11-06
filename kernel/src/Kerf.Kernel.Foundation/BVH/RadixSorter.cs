// Port of: src/FoundationClasses/TKMath/BVH/BVH_RadixSorter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A pair of Morton code and primitive ID (OCCT's <c>std::pair&lt;unsigned int, int&gt;</c>).</summary>
internal struct EncodedLink : IEquatable<EncodedLink>
{
    public uint First;
    public int Second;

    public EncodedLink(uint theFirst, int theSecond)
    {
        First = theFirst;
        Second = theSecond;
    }

    public readonly bool Equals(EncodedLink other) => First == other.First && Second == other.Second;

    public override readonly bool Equals(object? obj) => obj is EncodedLink other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(First, Second);

    public static bool operator ==(EncodedLink left, EncodedLink right) => left.Equals(right);

    public static bool operator !=(EncodedLink left, EncodedLink right) => !left.Equals(right);

    public override readonly string ToString() => $"({First}, {Second})";
}

internal static partial class BvhTools
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
    public static uint EncodeMortonCode(uint voxelX, uint voxelY, uint voxelZ)
    {
        // Split each 10-bit coordinate into two 8-bit lookups (upper 2 bits + lower 8 bits)
        ReadOnlySpan<uint> lut = THE_MORTON_LUT;
        return (lut[(int)(voxelX & 0xFF)] | (lut[(int)((voxelX >> 8) & 0x03)] << 24))
               | ((lut[(int)(voxelY & 0xFF)] | (lut[(int)((voxelY >> 8) & 0x03)] << 24)) << 1)
               | ((lut[(int)(voxelZ & 0xFF)] | (lut[(int)((voxelZ >> 8) & 0x03)] << 24)) << 2);
    }

    /// <summary>
    /// <c>std::partition</c> of a range of links by the given bit (0-bit to the left side), as Microsoft's standard
    /// library does it (swapping from both ends), so the order of the links matches OCCT built with MSVC.
    /// </summary>
    internal static int PartitionLinks(EncodedLink[] links, int first, int last, int digit)
    {
        uint bit = 1U << digit;
        for (;;)
        {
            // skip in-place elements at beginning
            for (;;)
            {
                if (first == last)
                {
                    return first;
                }

                if ((links[first].First & bit) != 0)
                {
                    break;
                }

                ++first;
            }

            // skip in-place elements at end
            do
            {
                --last;
                if (first == last)
                {
                    return first;
                }
            } while ((links[last].First & bit) != 0);

            (links[first], links[last]) = (links[last], links[first]); // out of place, swap and loop
            ++first;
        }
    }

    /// <summary>
    /// Sorts the links of the (exclusive) range by Morton code with an MSD radix sort (<c>BVH::RadixSorter::Sort</c>):
    /// the top digits split in parallel when allowed, the rest sequentially.
    /// </summary>
    internal static void RadixSort(EncodedLink[] links, int start, int final, int digit, bool isParallel)
    {
        if (digit < 24)
        {
            RadixSortPerform(links, start, final, digit);
        }
        else
        {
            int offset = PartitionLinks(links, start, final, digit);
            (int Start, int Final, int Digit)[] splits = [(start, offset, digit - 1),
                                                           (offset, final, digit - 1)];
            // OSD_Parallel::For(0, 2, Functor, !isParallel)
            OsdParallel.For(0, 2, index => RadixSort(links, splits[index].Start, splits[index].Final,
                                                         splits[index].Digit, isParallel), !isParallel);
        }
    }

    // Performs MSD (most significant digit) radix sort.
    private static void RadixSortPerform(EncodedLink[] links, int start, int final, int digit = 29)
    {
        while (start != final && digit >= 0)
        {
            int offset = PartitionLinks(links, start, final, digit--);
            RadixSortPerform(links, start, offset, digit);
            start = offset;
        }
    }
}

/// <summary>Radix sort of a BVH primitive set by 10-bit Morton codes (a 1024 x 1024 x 1024 grid).</summary>
internal class RadixSorter<T, TVec> : Sorter<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Axis-aligned bounding box (AABB) to perform sorting.</summary>
    protected Box<T, TVec> _box;

    /// <summary>Morton codes assigned to BVH primitives.</summary>
    protected NCollection_Array1<EncodedLink>? _encodedLinks;

    /// <summary>Creates a new radix sorter for the given AABB.</summary>
    public RadixSorter(in Box<T, TVec> box)
    {
        _box = box;
    }

    /// <summary>Sorts the set.</summary>
    public override void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set) => Perform(set, 0, set.Size() - 1);

    /// <summary>The Morton codes assigned to the primitives (by the last sort).</summary>
    public NCollection_Array1<EncodedLink> EncodedLinks() => _encodedLinks!;

    /// <summary>Sorts the given (inclusive) range of the set.</summary>
    public override void Perform(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, int start, int final)
    {
        const int dimension = 1024;
        int nbEffComp = TVec.Length() == 2 ? 2 : 3; // 4th component is ignored

        TVec sceneMin = _box.CornerMin();
        TVec sceneMax = _box.CornerMax();

        TVec nodeMinSizeVecT = TVec.Create(T.CreateTruncating(BvhTools.THE_NODE_MIN_SIZE));
        BvhTools.CwiseMax<T, TVec>(ref nodeMinSizeVecT, sceneMax - sceneMin);

        TVec reverseSize = TVec.Create(T.CreateTruncating(dimension)) / nodeMinSizeVecT;

        _encodedLinks = new NCollection_Array1<EncodedLink>(start, final);

        // Step 1 -- Assign Morton code to each primitive using LUT for faster encoding
        for (int primIdx = start; primIdx <= final; ++primIdx)
        {
            TVec center = set.Box(primIdx).Center();
            TVec voxelF = (center - sceneMin) * reverseSize;

            // Compute voxel coordinates clamped to valid range
            int voxelX = Math.Clamp(BvhTools.IntFloor(BvhTools.VecComp<T, TVec>(voxelF, 0)), 0, dimension - 1);
            int voxelY = Math.Clamp(BvhTools.IntFloor(BvhTools.VecComp<T, TVec>(voxelF, 1)), 0, dimension - 1);
            int voxelZ = (nbEffComp > 2)
                              ? Math.Clamp(BvhTools.IntFloor(BvhTools.VecComp<T, TVec>(voxelF, 2)), 0, dimension - 1)
                              : 0;

            // Use LUT-based Morton code encoding for better performance
            uint mortonCode = BvhTools.EncodeMortonCode((uint)voxelX, (uint)voxelY, (uint)voxelZ);

            _encodedLinks[primIdx] = new EncodedLink(mortonCode, primIdx);
        }

        // Step 2 -- Sort primitives by their Morton codes using radix sort
        // (on a copy of the links, which the parallel branches can share; copied back after)
        EncodedLink[] links = _encodedLinks.AsSpan().ToArray();
        BvhTools.RadixSort(links, 0, links.Length, 29, IsParallel);
        links.CopyTo(_encodedLinks.AsSpan());

        NCollection_Array1<int> linkMap = new(start, final);
        for (int linkIdx = start; linkIdx <= final; ++linkIdx)
        {
            linkMap[_encodedLinks.Value(linkIdx).Second] = linkIdx;
        }

        // Step 3 -- Rearranging primitive list according to Morton codes (in place)
        int primIdx2 = start;
        while (primIdx2 <= final)
        {
            int sortIdx = linkMap[primIdx2];
            if (primIdx2 != sortIdx)
            {
                set.Swap(primIdx2, sortIdx);
                (linkMap[primIdx2], linkMap[sortIdx]) = (linkMap[sortIdx], linkMap[primIdx2]);
            }
            else
            {
                ++primIdx2;
            }
        }
    }
}
