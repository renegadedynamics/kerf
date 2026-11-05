// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_CompositeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_CompositeClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Classifies composite loops: loops that are either a shape or a block of elements of a
/// <see cref="BlockBuilder"/>.
/// </summary>
/// <remarks>OCCT holds the block builder as a <c>void*</c>; the port holds a reference to it.</remarks>
internal abstract class CompositeClassifier : LoopClassifier
{
    protected BlockBuilder _blockBuilder;

    /// <summary>A classifier of the loops whose blocks are in <paramref name="bb"/>.</summary>
    protected CompositeClassifier(BlockBuilder bb)
    {
        _blockBuilder = bb;
    }

    /// <inheritdoc/>
    public override Containment Compare(BuildLoop l1, BuildLoop l2)
    {
        Containment state = Unknown;

        bool isshape1 = l1.IsShape;
        bool isshape2 = l2.IsShape;

        if (isshape2 && isshape1)
        { // L1 is Shape , L2 is Shape
            Shape s1 = l1.Shape();
            Shape s2 = l2.Shape();
            state = CompareShapes(s1, s2);
        }
        else if (isshape2 && !isshape1)
        { // L1 is Block , L2 is Shape
            BlockIterator bit1 = l1.BlockIterator();
            bit1.Initialize();
            bool yena1 = bit1.More();
            while (yena1)
            {
                Shape s1 = _blockBuilder.Element(bit1);
                Shape s2 = l2.Shape();
                state = CompareElementToShape(s1, s2);
                yena1 = false;
                if (state == Unknown)
                {
                    if (bit1.More())
                    {
                        bit1.Next();
                    }
                    yena1 = bit1.More();
                }
            }
        }
        else if (!isshape2 && isshape1)
        { // L1 is Shape , L2 is Block
            Shape s1 = l1.Shape();
            ResetShape(s1);
            BlockIterator bit2 = l2.BlockIterator();
            for (bit2.Initialize(); bit2.More(); bit2.Next())
            {
                Shape s2 = _blockBuilder.Element(bit2);
                if (!CompareElement(s2))
                {
                    break;
                }
            }
            state = State();
        }
        else if (!isshape2 && !isshape1)
        { // L1 is Block , L2 is Block
            BlockIterator bit1 = l1.BlockIterator();
            bit1.Initialize();
            bool yena1 = bit1.More();
            while (yena1)
            {
                Shape s1 = _blockBuilder.Element(bit1);
                ResetElement(s1);
                BlockIterator bit2 = l2.BlockIterator();
                for (bit2.Initialize(); bit2.More(); bit2.Next())
                {
                    Shape s2 = _blockBuilder.Element(bit2);
                    CompareElement(s2);
                }
                state = State();
                yena1 = false;
                if (state == Unknown)
                {
                    if (bit1.More())
                    {
                        bit1.Next();
                    }
                    yena1 = bit1.More();
                }
            }
        }
        return state;
    }

    /// <summary>Classifies the shape <paramref name="b1"/> with the shape <paramref name="b2"/>.</summary>
    public abstract Containment CompareShapes(in Shape b1, in Shape b2);

    /// <summary>Classifies the element <paramref name="e"/> with the shape <paramref name="b"/>.</summary>
    public abstract Containment CompareElementToShape(in Shape e, in Shape b);

    /// <summary>Prepares a classification involving the shape <paramref name="b"/> (ResetElement on its first element).</summary>
    public abstract void ResetShape(in Shape b);

    /// <summary>Prepares a classification involving the element <paramref name="e"/>.</summary>
    public abstract void ResetElement(in Shape e);

    /// <summary>
    /// Adds the element <paramref name="e"/> to the elements used in the classification; false if it was already
    /// there.
    /// </summary>
    public abstract bool CompareElement(in Shape e);

    /// <summary>The state of the point set by ResetElement against the elements added by CompareElement.</summary>
    public abstract Containment State();
}
