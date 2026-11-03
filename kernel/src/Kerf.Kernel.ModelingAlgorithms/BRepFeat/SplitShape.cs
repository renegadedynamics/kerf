// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_SplitShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Splits the faces of a shape by wires or edges lying on them: the split shape, the faces left of the wires and the
/// faces right of them.
/// </summary>
/// <remarks>
/// The splitting (LocOpe_WiresOnShape with LocOpe_Spliter) copies the frozen shapes it is given and ends its working
/// shapes at the end of <see cref="Build"/> (decision 1013): the result shares the caller's unchanged sub-shapes, and
/// <see cref="MakeShape.Modified"/> answers for the caller's faces and edges. A wires-on-shape serves
/// one splitting: <see cref="Init"/> resets it.
/// </remarks>
public sealed class SplitShape : MakeShape
{
    private readonly Spliter _sShape;
    private WiresOnShape? _wOnShape;
    private readonly NCollection_List<Shape> _right = new();

    /// <summary>Empty constructor.</summary>
    public SplitShape()
    {
        _sShape = new Spliter();
    }

    /// <summary>Creates the process with the shape <paramref name="s"/>.</summary>
    public SplitShape(in Shape s)
    {
        _sShape = new Spliter(s);
        _wOnShape = new WiresOnShape(s);
    }

    /// <summary>
    /// Adds splitting edges or wires for the whole initial shape, without saying which face or edge each lies on: puts
    /// each edge on the corresponding faces of the initial shape.
    /// </summary>
    public bool Add(NCollection_Sequence<Shape> edges)
    {
        return _wOnShape!.Add(edges);
    }

    /// <summary>Initializes the process on the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        _sShape.Init(s);
        if (_wOnShape is null)
        {
            _wOnShape = new WiresOnShape(s);
        }
        else
        {
            _wOnShape.Init(s);
        }
    }

    /// <summary>Set the flag of check internal intersections (true by default).</summary>
    public void SetCheckInterior(bool toCheckInterior)
    {
        _wOnShape!.SetCheckInterior(toCheckInterior);
    }

    /// <summary>Adds the wire <paramref name="w"/> on the face <paramref name="f"/>.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="f"/> does not belong to the original shape.</exception>
    public void Add(in Wire w, in Face f)
    {
        _wOnShape!.Bind(w, f);
    }

    /// <summary>Adds the edge <paramref name="e"/> on the face <paramref name="f"/>.</summary>
    public void Add(in Edge e, in Face f)
    {
        _wOnShape!.Bind(e, f);
    }

    /// <summary>
    /// Adds the compound <paramref name="comp"/> of edges lying on the face <paramref name="f"/>; edges connected
    /// geometrically must share their vertices.
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="f"/> does not belong to the original shape.</exception>
    public void Add(in Compound comp, in Face f)
    {
        _wOnShape!.Bind(comp, f);
    }

    /// <summary>Adds the edge <paramref name="e"/> on the existing edge <paramref name="eOn"/>.</summary>
    public void Add(in Edge e, in Edge eOn)
    {
        _wOnShape!.Bind(e, eOn);
    }

    /// <summary>Returns the faces which are the left of the projected wires (frozen).</summary>
    public ISchematiqList<Shape> DirectLeft()
    {
        BeforePublishingLists();
        return PublishedList(_sShape.InternalDirectLeft());
    }

    /// <summary>
    /// Returns the faces of the "left" part on the shape, built from <see cref="DirectLeft"/> with the faces connected
    /// to this set, and so on (frozen).
    /// </summary>
    /// <exception cref="NotDoneException">The splitting isn't done.</exception>
    public ISchematiqList<Shape> Left()
    {
        BeforePublishingLists();
        return PublishedList(_sShape.InternalLeft());
    }

    /// <summary><see cref="Left"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLeft() => _sShape.InternalLeft();

    /// <summary>Returns the faces of the "right" part on the shape (frozen).</summary>
    public ISchematiqList<Shape> Right()
    {
        BeforePublishingLists();
        return PublishedList(InternalRight());
    }

    /// <summary><see cref="Right"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalRight()
    {
        if (_right.IsEmpty())
        {
            NCollection_Map<Shape> mapOfLeft = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator iterator = new();
            for (iterator.Initialize(_sShape.InternalLeft()); iterator.More(); iterator.Next())
            {
                mapOfLeft.Add(iterator.Value());
            }
            Explorer explorer = new();
            for (explorer.Init(_shape, ShapeType.Face); explorer.More(); explorer.Next())
            {
                Shape face = explorer.Current();
                if (!mapOfLeft.Contains(face))
                {
                    _right.Append(face);
                }
            }
        }
        return _right;
    }

    /// <summary>Builds the cut and the resulting faces and edges as well.</summary>
    public override void Build(ProgressRange range = default)
    {
        _sShape.Perform(_wOnShape!);
        if (_sShape.IsDone)
        {
            Done();
            _shape = _sShape.InternalResultingShape();
            _right.Clear();
        }
    }

    /// <summary>Returns true if the shape has been deleted.</summary>
    public override bool IsDeleted(in Shape f)
    {
        NCollection_List<Shape>.Iterator itl = new(_sShape.InternalDescendantShapes(f));
        return !itl.More(); // a priori impossible
    }

    /// <summary>The shapes descending from <paramref name="f"/> (OCCT's virtual; unfrozen).</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        return _sShape.InternalDescendantShapes(f);
    }

    // The history's lists hold sub-shapes of the result: publishing them publishes the result (decision 414).
    private protected override void BeforePublishingLists()
    {
        _shape.TShape()?.FreezeGraph();
    }
}
