// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_CompositeClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_CompositeClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Classifies composite loops: loops that are either a shape or a block of elements of a
/// <see cref="TopOpeBRepBuild_BlockBuilder"/>.
/// </summary>
/// <remarks>OCCT holds the block builder as a <c>void*</c>; the port holds a reference to it.</remarks>
public abstract class TopOpeBRepBuild_CompositeClassifier : TopOpeBRepBuild_LoopClassifier
{
    protected TopOpeBRepBuild_BlockBuilder myBlockBuilder;

    /// <summary>A classifier of the loops whose blocks are in <paramref name="BB"/>.</summary>
    protected TopOpeBRepBuild_CompositeClassifier(TopOpeBRepBuild_BlockBuilder BB)
    {
        myBlockBuilder = BB;
    }

    /// <inheritdoc/>
    public override TopAbs_State Compare(TopOpeBRepBuild_Loop L1, TopOpeBRepBuild_Loop L2)
    {
        TopAbs_State state = TopAbs_UNKNOWN;

        bool isshape1 = L1.IsShape();
        bool isshape2 = L2.IsShape();

        if (isshape2 && isshape1)
        { // L1 is Shape , L2 is Shape
            TopoDS_Shape s1 = L1.Shape();
            TopoDS_Shape s2 = L2.Shape();
            state = CompareShapes(s1, s2);
        }
        else if (isshape2 && !isshape1)
        { // L1 is Block , L2 is Shape
            TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
            Bit1.Initialize();
            bool yena1 = Bit1.More();
            while (yena1)
            {
                TopoDS_Shape s1 = myBlockBuilder.Element(Bit1);
                TopoDS_Shape s2 = L2.Shape();
                state = CompareElementToShape(s1, s2);
                yena1 = false;
                if (state == TopAbs_UNKNOWN)
                {
                    if (Bit1.More())
                    {
                        Bit1.Next();
                    }
                    yena1 = Bit1.More();
                }
            }
        }
        else if (!isshape2 && isshape1)
        { // L1 is Shape , L2 is Block
            TopoDS_Shape s1 = L1.Shape();
            ResetShape(s1);
            TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
            for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
            {
                TopoDS_Shape s2 = myBlockBuilder.Element(Bit2);
                if (!CompareElement(s2))
                {
                    break;
                }
            }
            state = State();
        }
        else if (!isshape2 && !isshape1)
        { // L1 is Block , L2 is Block
            TopOpeBRepBuild_BlockIterator Bit1 = L1.BlockIterator();
            Bit1.Initialize();
            bool yena1 = Bit1.More();
            while (yena1)
            {
                TopoDS_Shape s1 = myBlockBuilder.Element(Bit1);
                ResetElement(s1);
                TopOpeBRepBuild_BlockIterator Bit2 = L2.BlockIterator();
                for (Bit2.Initialize(); Bit2.More(); Bit2.Next())
                {
                    TopoDS_Shape s2 = myBlockBuilder.Element(Bit2);
                    CompareElement(s2);
                }
                state = State();
                yena1 = false;
                if (state == TopAbs_UNKNOWN)
                {
                    if (Bit1.More())
                    {
                        Bit1.Next();
                    }
                    yena1 = Bit1.More();
                }
            }
        }
        return state;
    }

    /// <summary>Classifies the shape <paramref name="B1"/> with the shape <paramref name="B2"/>.</summary>
    public abstract TopAbs_State CompareShapes(in TopoDS_Shape B1, in TopoDS_Shape B2);

    /// <summary>Classifies the element <paramref name="E"/> with the shape <paramref name="B"/>.</summary>
    public abstract TopAbs_State CompareElementToShape(in TopoDS_Shape E, in TopoDS_Shape B);

    /// <summary>Prepares a classification involving the shape <paramref name="B"/> (ResetElement on its first element).</summary>
    public abstract void ResetShape(in TopoDS_Shape B);

    /// <summary>Prepares a classification involving the element <paramref name="E"/>.</summary>
    public abstract void ResetElement(in TopoDS_Shape E);

    /// <summary>
    /// Adds the element <paramref name="E"/> to the elements used in the classification; false if it was already
    /// there.
    /// </summary>
    public abstract bool CompareElement(in TopoDS_Shape E);

    /// <summary>The state of the point set by ResetElement against the elements added by CompareElement.</summary>
    public abstract TopAbs_State State();
}
