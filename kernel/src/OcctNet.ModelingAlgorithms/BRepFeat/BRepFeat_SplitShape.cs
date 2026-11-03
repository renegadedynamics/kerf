// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Splits the faces of a shape by wires or edges lying on them: the split shape, the faces left of the wires and the
/// faces right of them.
/// </summary>
/// <remarks>
/// The splitting (LocOpe_WiresOnShape with LocOpe_Spliter) copies the frozen shapes it is given and ends its working
/// shapes at the end of <see cref="Build"/> (decision 1013): the result shares the caller's unchanged sub-shapes, and
/// <see cref="BRepBuilderAPI_MakeShape.Modified"/> answers for the caller's faces and edges. A wires-on-shape serves
/// one splitting: <see cref="Init"/> resets it.
/// </remarks>
public sealed class BRepFeat_SplitShape : BRepBuilderAPI_MakeShape
{
    private readonly LocOpe_Spliter mySShape;
    private LocOpe_WiresOnShape? myWOnShape;
    private readonly NCollection_List<Shape> myRight = new();

    /// <summary>Empty constructor.</summary>
    public BRepFeat_SplitShape()
    {
        mySShape = new LocOpe_Spliter();
    }

    /// <summary>Creates the process with the shape <paramref name="S"/>.</summary>
    public BRepFeat_SplitShape(in Shape S)
    {
        mySShape = new LocOpe_Spliter(S);
        myWOnShape = new LocOpe_WiresOnShape(S);
    }

    /// <summary>
    /// Adds splitting edges or wires for the whole initial shape, without saying which face or edge each lies on: puts
    /// each edge on the corresponding faces of the initial shape.
    /// </summary>
    public bool Add(NCollection_Sequence<Shape> theEdges)
    {
        return myWOnShape!.Add(theEdges);
    }

    /// <summary>Initializes the process on the shape <paramref name="S"/>.</summary>
    public void Init(in Shape S)
    {
        mySShape.Init(S);
        if (myWOnShape is null)
        {
            myWOnShape = new LocOpe_WiresOnShape(S);
        }
        else
        {
            myWOnShape.Init(S);
        }
    }

    /// <summary>Set the flag of check internal intersections (true by default).</summary>
    public void SetCheckInterior(bool ToCheckInterior)
    {
        myWOnShape!.SetCheckInterior(ToCheckInterior);
    }

    /// <summary>Adds the wire <paramref name="W"/> on the face <paramref name="F"/>.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="F"/> does not belong to the original shape.</exception>
    public void Add(in Wire W, in Face F)
    {
        myWOnShape!.Bind(W, F);
    }

    /// <summary>Adds the edge <paramref name="E"/> on the face <paramref name="F"/>.</summary>
    public void Add(in Edge E, in Face F)
    {
        myWOnShape!.Bind(E, F);
    }

    /// <summary>
    /// Adds the compound <paramref name="Comp"/> of edges lying on the face <paramref name="F"/>; edges connected
    /// geometrically must share their vertices.
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="F"/> does not belong to the original shape.</exception>
    public void Add(in Compound Comp, in Face F)
    {
        myWOnShape!.Bind(Comp, F);
    }

    /// <summary>Adds the edge <paramref name="E"/> on the existing edge <paramref name="EOn"/>.</summary>
    public void Add(in Edge E, in Edge EOn)
    {
        myWOnShape!.Bind(E, EOn);
    }

    /// <summary>Returns the faces which are the left of the projected wires (frozen).</summary>
    public ISchematiqList<Shape> DirectLeft()
    {
        BeforePublishingLists();
        return PublishedList(mySShape.InternalDirectLeft());
    }

    /// <summary>
    /// Returns the faces of the "left" part on the shape, built from <see cref="DirectLeft"/> with the faces connected
    /// to this set, and so on (frozen).
    /// </summary>
    /// <exception cref="NotDoneException">The splitting isn't done.</exception>
    public ISchematiqList<Shape> Left()
    {
        BeforePublishingLists();
        return PublishedList(mySShape.InternalLeft());
    }

    /// <summary><see cref="Left"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLeft() => mySShape.InternalLeft();

    /// <summary>Returns the faces of the "right" part on the shape (frozen).</summary>
    public ISchematiqList<Shape> Right()
    {
        BeforePublishingLists();
        return PublishedList(InternalRight());
    }

    /// <summary><see cref="Right"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalRight()
    {
        if (myRight.IsEmpty())
        {
            NCollection_Map<Shape> aMapOfLeft = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator anIterator = new();
            for (anIterator.Initialize(mySShape.InternalLeft()); anIterator.More(); anIterator.Next())
            {
                aMapOfLeft.Add(anIterator.Value());
            }
            Explorer anExplorer = new();
            for (anExplorer.Init(myShape, ShapeType.Face); anExplorer.More(); anExplorer.Next())
            {
                Shape aFace = anExplorer.Current();
                if (!aMapOfLeft.Contains(aFace))
                {
                    myRight.Append(aFace);
                }
            }
        }
        return myRight;
    }

    /// <summary>Builds the cut and the resulting faces and edges as well.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        mySShape.Perform(myWOnShape!);
        if (mySShape.IsDone())
        {
            Done();
            myShape = mySShape.InternalResultingShape();
            myRight.Clear();
        }
    }

    /// <summary>Returns true if the shape has been deleted.</summary>
    public override bool IsDeleted(in Shape F)
    {
        NCollection_List<Shape>.Iterator itl = new(mySShape.InternalDescendantShapes(F));
        return !itl.More(); // a priori impossible
    }

    /// <summary>The shapes descending from <paramref name="F"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape F)
    {
        return mySShape.InternalDescendantShapes(F);
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        myShape.TShape()?.FreezeGraph();
    }
}
