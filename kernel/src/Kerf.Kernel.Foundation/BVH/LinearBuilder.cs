// Port of: src/FoundationClasses/TKMath/BVH/BVH_LinearBuilder.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Numerics;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Geometry.Bvh.Constants;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>
/// Fast BVH construction with the LBVH approach: spatial Morton codes reduce the construction to a radix sort
/// (O(N)). The trees are of lower quality than SAH builders give, but are built over an order of magnitude faster.
/// See C. Lauterbach, M. Garland, S. Sengupta, D. Luebke, and D. Manocha, Fast BVH construction on GPUs,
/// Eurographics, 2009.
/// </summary>
public class LinearBuilder<T, TVec> : Builder<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Creates a binned LBVH builder.</summary>
    public LinearBuilder(int leafNodeSize = BVH_Constants_LeafNodeSizeDefault,
                             int maxTreeDepth = BVH_Constants_MaxTreeDepth)
        : base(leafNodeSize, maxTreeDepth)
    {
    }

    /// <summary>The index of the first element (from <paramref name="start"/>) with the given bit set.</summary>
    protected int LowerBound(NCollection_Array1<EncodedLink> encodedLinks, int start, int final, int digit)
    {
        int nbPrims = final - start;
        uint bit = 1U << digit;
        while (nbPrims > 0)
        {
            int step = nbPrims / 2;
            if ((encodedLinks.Value(start + step).first & bit) != 0)
            {
                nbPrims = step;
            }
            else
            {
                start += step + 1;
                nbPrims -= step + 1;
            }
        }

        return start;
    }

    /// <summary>Emits the hierarchy from the sorted Morton codes; returns the index of the node made.</summary>
    protected int EmitHierachy(Tree<T, TVec> bvh,
                               NCollection_Array1<EncodedLink> encodedLinks,
                               int digit,
                               int shift,
                               int start,
                               int final)
    {
        if (final - start > _leafNodeSize)
        {
            int position = digit < 0 ? (start + final) / 2
                                         : LowerBound(encodedLinks, start, final, digit);
            if (position == start || position == final)
            {
                return EmitHierachy(bvh, encodedLinks, digit - 1, shift, start, final);
            }

            // Build inner node
            int node = bvh.AddInnerNode(0, 0);
            int rghNode = shift + position - start;

            int lftChild = EmitHierachy(bvh, encodedLinks, digit - 1, shift, start, position);
            int rghChild = EmitHierachy(bvh, encodedLinks, digit - 1, rghNode, position, final);

            bvh.NodeInfoBuffer()[node][1] = lftChild;
            bvh.NodeInfoBuffer()[node][2] = rghChild;

            return node;
        }
        else
        {
            // Build leaf node
            return bvh.AddLeafNode(shift, shift + final - start - 1);
        }
    }

    /// <summary>Builds the BVH.</summary>
    public override void Build(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> bvh, in Box<T, TVec> box)
    {
        int setSize = set.Size();
        if (bvh is null || setSize == 0)
        {
            return;
        }

        bvh.Clear();

        // Step 0 -- Initialize parameter of virtual grid
        RadixSorter<T, TVec> radixSorter = new(box);
        radixSorter.SetParallel(IsParallel);

        // Step 1 - Perform radix sorting of primitive set
        radixSorter.Perform(set);

        // Step 2 -- Emitting BVH hierarchy from sorted Morton codes
        EmitHierachy(bvh, radixSorter.EncodedLinks(), 29, 0, 0, set.Size());

        // Step 3 -- Compute bounding boxes of BVH nodes
        bvh.MinPointBuffer().Resize(bvh.NodeInfoBuffer().Size());
        bvh.MaxPointBuffer().Resize(bvh.NodeInfoBuffer().Size());

        StrongBox<int> height = new(0);

        BvhTools.BoundData<T, TVec> boundData = new(set, bvh, 0, 0, height);
        BvhTools.UpdateBoundTask<T, TVec> boundTask = new(IsParallel);

        boundTask.Invoke(boundData);

        UpdateDepth(bvh, height.Value);
    }
}

public static partial class BvhTools
{
    /// <summary>Calculates the bounding boxes of the given tree's nodes from the node down; returns the height.</summary>
    public static int UpdateBounds<T, TVec>(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> set, Tree<T, TVec> tree, int node = 0)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        NCollection_Vec4<int> data = tree.NodeInfoBuffer()[node];
        if (data.X() == 0)
        {
            int lftChild = tree.NodeInfoBuffer()[node].Y();
            int rghChild = tree.NodeInfoBuffer()[node].Z();

            int lftDepth = UpdateBounds(set, tree, lftChild);
            int rghDepth = UpdateBounds(set, tree, rghChild);

            TVec lftMinPoint = tree.MinPointBuffer()[lftChild];
            TVec lftMaxPoint = tree.MaxPointBuffer()[lftChild];
            TVec rghMinPoint = tree.MinPointBuffer()[rghChild];
            TVec rghMaxPoint = tree.MaxPointBuffer()[rghChild];

            CwiseMin<T, TVec>(ref lftMinPoint, rghMinPoint);
            CwiseMax<T, TVec>(ref lftMaxPoint, rghMaxPoint);

            tree.MinPointBuffer()[node] = lftMinPoint;
            tree.MaxPointBuffer()[node] = lftMaxPoint;

            return Math.Max(lftDepth, rghDepth) + 1;
        }
        else
        {
            ref TVec minPoint = ref tree.MinPointBuffer()[node];
            ref TVec maxPoint = ref tree.MaxPointBuffer()[node];

            for (int primIdx = data.Y(); primIdx <= data.Z(); ++primIdx)
            {
                Box<T, TVec> box = set.Box(primIdx);

                if (primIdx == data.Y())
                {
                    minPoint = box.CornerMin();
                    maxPoint = box.CornerMax();
                }
                else
                {
                    CwiseMin<T, TVec>(ref minPoint, box.CornerMin());
                    CwiseMax<T, TVec>(ref maxPoint, box.CornerMax());
                }
            }
        }

        return 0;
    }

    /// <summary>The data of a bounds update task (OCCT's <c>int* myHeight</c> is a shared box).</summary>
    public readonly struct BoundData<T, TVec>(global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> theSet, Tree<T, TVec> theBVH, int theNode, int theLevel, StrongBox<int> theHeight)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        public readonly global::Kerf.Engine.Geometry.Bvh.ISet<T, TVec> mySet = theSet;        //!< Set of geometric objects
        public readonly Tree<T, TVec> myBVH = theBVH;       //!< BVH tree built over the set
        public readonly int Node = theNode;                   //!< BVH node to update bounding box
        public readonly int myLevel = theLevel;                 //!< Level of the processed BVH node
        public readonly StrongBox<int> myHeight = theHeight;    //!< Height of the processed BVH node
    }

    /// <summary>A task for updating bounds, in parallel for the top levels when allowed.</summary>
    public sealed class UpdateBoundTask<T, TVec>(bool isParallel)
        where T : unmanaged, INumber<T>, IMinMaxValue<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        private readonly bool _isParallel = isParallel;

        /// <summary>Executes the task (OCCT's <c>operator()</c>).</summary>
        public void Invoke(BoundData<T, TVec> data)
        {
            if (data.myBVH.IsOuter(data.Node) || data.myLevel > 2)
            {
                data.myHeight.Value = UpdateBounds(data.mySet, data.myBVH, data.Node);
            }
            else
            {
                StrongBox<int> lftHeight = new(0);
                StrongBox<int> rghHeight = new(0);

                int lftChild = data.myBVH.NodeInfoBuffer()[data.Node].Y();
                int rghChild = data.myBVH.NodeInfoBuffer()[data.Node].Z();

                List<BoundData<T, TVec>> list = new(2);

                if (!data.myBVH.IsOuter(lftChild))
                {
                    list.Add(new BoundData<T, TVec>(data.mySet, data.myBVH, lftChild, data.myLevel + 1, lftHeight));
                }
                else
                {
                    lftHeight.Value = UpdateBounds(data.mySet, data.myBVH, lftChild);
                }

                if (!data.myBVH.IsOuter(rghChild))
                {
                    list.Add(new BoundData<T, TVec>(data.mySet, data.myBVH, rghChild, data.myLevel + 1, rghHeight));
                }
                else
                {
                    rghHeight.Value = UpdateBounds(data.mySet, data.myBVH, rghChild);
                }

                if (list.Count > 0)
                {
                    // OSD_Parallel::ForEach(..., !myIsParallel)
                    UpdateBoundTask<T, TVec> task = new(_isParallel);
                    OsdParallel.ForEach(list, task.Invoke, !_isParallel);
                }

                TVec lftMinPoint = data.myBVH.MinPointBuffer()[lftChild];
                TVec lftMaxPoint = data.myBVH.MaxPointBuffer()[lftChild];
                TVec rghMinPoint = data.myBVH.MinPointBuffer()[rghChild];
                TVec rghMaxPoint = data.myBVH.MaxPointBuffer()[rghChild];

                CwiseMin<T, TVec>(ref lftMinPoint, rghMinPoint);
                CwiseMax<T, TVec>(ref lftMaxPoint, rghMaxPoint);

                data.myBVH.MinPointBuffer()[data.Node] = lftMinPoint;
                data.myBVH.MaxPointBuffer()[data.Node] = lftMaxPoint;

                data.myHeight.Value = Math.Max(lftHeight.Value, rghHeight.Value) + 1;
            }
        }
    }
}
