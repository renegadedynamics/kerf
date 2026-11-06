// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Iterator.hxx
// Port of: src/ModelingData/TKBRep/TopoDS/TopoDS_Iterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1990-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Kerf.Engine.Topology;

/// <summary>
/// Iterates on the direct sub-shapes (children) of a shape. Each child is returned with its orientation composed with
/// the shape's and its location multiplied by the shape's, unless asked otherwise.
/// </summary>
/// <remarks>A class (a working object), where OCCT's is used by value. OCCT's range-for support is left out.</remarks>
internal sealed class Iterator
{
    private static readonly NCollection_List<Shape> EmptyList = new();

    private Shape _shape;
    // The list and the place in it, not a list iterator: an object more for each of these (unit 11B)
    private NCollection_List<Shape> _children = EmptyList;
    private int _position;
    private Orientation _orientation;
    private Placement _location;

    /// <summary>An empty iterator.</summary>
    public Iterator()
    {
        _shape = new Shape();
        _orientation = Orientation.Forward;
        _location = new Placement();
    }

    /// <summary>An iterator on the sub-shapes of <paramref name="s"/>.</summary>
    /// <param name="s">The shape whose children to visit.</param>
    /// <param name="cumOri">Compose the children's orientations with the orientation of S.</param>
    /// <param name="cumLoc">Multiply the children's locations by the location of S.</param>
    public Iterator(in Shape s, bool cumOri = true, bool cumLoc = true)
    {
        _shape = new Shape();
        _orientation = Orientation.Forward;
        _location = new Placement();
        Initialize(s, cumOri, cumLoc);
    }

    /// <summary>Starts iterating on the sub-shapes of <paramref name="s"/>.</summary>
    /// <param name="s">The shape whose children to visit.</param>
    /// <param name="cumOri">Compose the children's orientations with the orientation of S.</param>
    /// <param name="cumLoc">Multiply the children's locations by the location of S.</param>
    public void Initialize(in Shape s, bool cumOri = true, bool cumLoc = true)
    {
        if (cumLoc)
        {
            _location = s.Placement;
        }
        else
        {
            _location = new Placement();
        }

        if (cumOri)
        {
            _orientation = s.Orientation;
        }
        else
        {
            _orientation = Orientation.Forward;
        }

        _children = s.IsNull ? EmptyList : WorkingOverlay.View(s.TShape()!)._shapes;
        _position = 0;

        if (More())
        {
            UpdateCurrentShape();
        }
    }

    /// <summary>True if there is a current sub-shape.</summary>
    public bool More() => _position < _children.Extent();

    /// <summary>Moves to the next sub-shape.</summary>
    public void Next()
    {
        _position++;
        if (More())
        {
            UpdateCurrentShape();
        }
    }

    /// <summary>The current sub-shape.</summary>
    /// <exception cref="NoSuchObjectException">There is no current sub-shape.</exception>
    public Shape Value()
    {
        Standard_NoSuchObject_Raise_if(!More(), "TopoDS_Iterator::Value");
        return _shape;
    }

    // Updates myShape from the current iterator position.
    private void UpdateCurrentShape()
    {
        _shape = _children.ItemAt(_position);
        _shape.Orient(global::Kerf.Engine.Topology.TopAbs.Compose(_orientation, _shape.Orientation));
        if (!_location.IsIdentity)
        {
            _shape.Move(_location, false);
        }
    }
}
