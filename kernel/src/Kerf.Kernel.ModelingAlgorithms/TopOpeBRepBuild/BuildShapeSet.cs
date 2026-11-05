// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_ShapeSet.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_ShapeSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// The shapes to rebuild from: a list of shapes (wires or shells, kept as they are), a list of start elements (edges
/// or faces) and, for each sub-shape (vertex or edge) of the elements, the elements incident to it.
/// </summary>
/// <remarks>
/// <para>
/// A working object. The maps of shapes already added compare with IsEqual (OCCT's default hasher), the map of
/// sub-shapes with IsSame (TopTools_ShapeMapHasher). The checking of added shapes (<see cref="CheckShape(in Shape, bool)"/>)
/// is always off, as in OCCT (whose constructor turns it off, "temporary NYI").
/// </para>
/// <para>
/// The debugging members (<c>DumpName</c>, <c>DumpCheck</c>, <c>DumpSS</c>, <c>DumpBB</c>, <c>DEBName</c>,
/// <c>DEBNumber</c>, <c>SName</c>, <c>SNameori</c>) are left out (decision 493).
/// </para>
/// </remarks>
internal class BuildShapeSet
{
    protected ShapeType _shapeType;
    protected ShapeType _subShapeType;
    protected readonly ToolShapeExplorer _subShapeExplorer = new();
    protected readonly NCollection_List<Shape> _startShapes = new();
    protected readonly NCollection_List<Shape>.Iterator _startShapesIter = new();
    protected readonly IndexedMap<Shape, NCollection_List<Shape>> _subShapeMap =
        new(ShapeHasher.Instance);
    protected readonly NCollection_List<Shape>.Iterator _incidentShapesIter = new();
    protected readonly NCollection_List<Shape> _shapes = new();
    protected readonly NCollection_List<Shape>.Iterator _shapesIter = new();
    protected Shape _currentShape = new();
    protected readonly NCollection_List<Shape> _currentShapeNeighbours = new();
    protected readonly IndexedSet<Shape> _omss = new(EqualityComparer<Shape>.Default);
    protected readonly IndexedSet<Shape> _omes = new(EqualityComparer<Shape>.Default);
    protected readonly IndexedSet<Shape> _omsh = new(EqualityComparer<Shape>.Default);
    protected bool _checkShape;

    /// <summary>
    /// A shape set to build shapes connected by sub-shapes of type <paramref name="subShapeType"/> (EDGE: faces into
    /// shells; VERTEX: edges into wires).
    /// </summary>
    /// <exception cref="ProgramErrorException">Another type.</exception>
    public BuildShapeSet(ShapeType subShapeType, bool checkshape = true)
    {
        _subShapeType = subShapeType;
        _checkShape = checkshape;
        if (subShapeType == ShapeType.Edge)
        {
            _shapeType = ShapeType.Face;
        }
        else if (subShapeType == ShapeType.Vertex)
        {
            _shapeType = ShapeType.Edge;
        }
        else
        {
            throw new ProgramErrorException("ShapeSet : bad ShapeType");
        }

        _checkShape = false; // temporary NYI
    }

    /// <summary>Adds <paramref name="s"/> (a wire or a shell) to the shapes.</summary>
    public virtual void AddShape(in Shape s)
    {
        bool chk = CheckShape(s);
        if (!chk)
        {
            return;
        }
        ProcessAddShape(s);
    }

    /// <summary>Adds <paramref name="s"/> (an edge or a face) to the start elements, and to the elements.</summary>
    public virtual void AddStartElement(in Shape s)
    {
        bool chk = CheckShape(s);
        if (!chk)
        {
            return;
        }
        ProcessAddStartElement(s);
    }

    /// <summary>Adds <paramref name="s"/> to the elements incident to each of its sub-shapes.</summary>
    public virtual void AddElement(in Shape s)
    {
        bool chk = CheckShape(s);
        if (!chk)
        {
            return;
        }
        ProcessAddElement(s);
    }

    protected void ProcessAddShape(in Shape s)
    {
        if (!_omsh.Contains(s))
        {
            _omsh.Add(s);
            _shapes.Append(s);
        }
    }

    protected void ProcessAddStartElement(in Shape s)
    {
        if (!_omss.Contains(s))
        {
            _omss.Add(s);
            _startShapes.Append(s);
            ProcessAddElement(s);
        }
    }

    protected void ProcessAddElement(in Shape s)
    {
        if (!_omes.Contains(s))
        {
            _omes.Add(s);
            ToolShapeExplorer ex = new(s, _subShapeType);
            for (; ex.More(); ex.Next())
            {
                Shape subshape = ex.Current();
                bool b = !_subShapeMap.ContainsKey(subshape);
                if (b)
                {
                    // OCCT copies an empty list into the map
                    _subShapeMap.Add(subshape, new NCollection_List<Shape>());
                }
                _subShapeMap.GetValue(subshape).Append(s);
            }
        }
    }

    /// <summary>The start elements.</summary>
    public NCollection_List<Shape> StartElements() => _startShapes;

    /// <summary>Starts the iteration on the shapes.</summary>
    public void InitShapes()
    {
        _shapesIter.Initialize(_shapes);
    }

    /// <summary>Whether there is a current shape.</summary>
    public bool MoreShapes()
    {
        bool b = _shapesIter.More();
        return b;
    }

    /// <summary>Moves to the next shape.</summary>
    public void NextShape()
    {
        _shapesIter.Next();
    }

    /// <summary>The current shape.</summary>
    public Shape Shape()
    {
        Shape s = _shapesIter.Value();
        return s;
    }

    /// <summary>Starts the iteration on the start elements.</summary>
    public void InitStartElements()
    {
        _startShapesIter.Initialize(_startShapes);
    }

    /// <summary>Whether there is a current start element.</summary>
    public bool MoreStartElements()
    {
        bool b = _startShapesIter.More();
        return b;
    }

    /// <summary>Moves to the next start element.</summary>
    public void NextStartElement()
    {
        _startShapesIter.Next();
    }

    /// <summary>The current start element.</summary>
    public Shape StartElement()
    {
        Shape s = _startShapesIter.Value();
        return s;
    }

    /// <summary>Starts the iteration on the neighbours of the element <paramref name="s"/>.</summary>
    public virtual void InitNeighbours(in Shape s)
    {
        _subShapeExplorer.Init(s, _subShapeType);
        _currentShape = s;
        FindNeighbours();
    }

    /// <summary>Whether there is a current neighbour.</summary>
    public bool MoreNeighbours()
    {
        bool b = _incidentShapesIter.More();
        return b;
    }

    /// <summary>Moves to the next neighbour.</summary>
    public void NextNeighbour()
    {
        _incidentShapesIter.Next();
        bool noisimore = !_incidentShapesIter.More();
        if (noisimore)
        {
            bool ssemore = _subShapeExplorer.More();
            if (ssemore)
            {
                _subShapeExplorer.Next();
                FindNeighbours();
            }
        }
    }

    /// <summary>The current neighbour.</summary>
    public Shape Neighbour()
    {
        Shape s = _incidentShapesIter.Value();
        return s;
    }

    /// <summary>The start elements, to change.</summary>
    public NCollection_List<Shape> ChangeStartShapes() => _startShapes;

    /// <summary>
    /// Finds the next sub-shape of the current element that has neighbours, and starts the iteration on them.
    /// </summary>
    public virtual void FindNeighbours()
    {
        while (_subShapeExplorer.More())
        {
            // l = list of edges neighbour of edge myCurrentShape through
            // the vertex mySubShapeExplorer.Current(), which is a vertex of the
            // edge myCurrentShape.
            Shape v = _subShapeExplorer.Current();
            NCollection_List<Shape> l = MakeNeighboursList(_currentShape, v);

            // myIncidentShapesIter iterates on the neighbour edges of the edge
            // given as InitNeighbours() argument (this edge has been stored
            // in the field myCurrentShape).

            _incidentShapesIter.Initialize(l);
            if (_incidentShapesIter.More())
            {
                break;
            }
            else
            {
                _subShapeExplorer.Next();
            }
        }
    }

    /// <summary>The elements incident to the sub-shape <paramref name="v"/> of <paramref name="e"/>.</summary>
    public virtual NCollection_List<Shape> MakeNeighboursList(in Shape e, in Shape v)
    {
        NCollection_List<Shape> l = _subShapeMap.GetValue(v);
        return l;
    }

    /// <summary>The largest number of elements incident to one sub-shape of <paramref name="shape"/>.</summary>
    public int MaxNumberSubShape(in Shape shape)
    {
        int i, m = 0;
        ToolShapeExplorer se = new(shape, _subShapeType);
        NCollection_List<Shape>.Iterator li = new();
        while (se.More())
        {
            Shape subShape = se.Current();
            if (!_subShapeMap.ContainsKey(subShape))
            {
                se.Next();
                continue;
            }
            NCollection_List<Shape> l = _subShapeMap.GetValue(subShape);
            li.Initialize(l);
            for (i = 0; li.More(); li.Next(), i++)
            {
            }
            m = Max(m, i);
            se.Next();
        }
        return m;
    }

    /// <summary>Turns the checking of added shapes on or off.</summary>
    public void CheckShape(bool checkshape)
    {
        _checkShape = checkshape;
    }

    /// <summary>Whether added shapes are checked.</summary>
    public bool CheckShape() => _checkShape;

    /// <summary>Whether <paramref name="s"/> is valid (BRepCheck), or true when checking is off.</summary>
    public bool CheckShape(in Shape s, bool checkgeom = false)
    {
        if (!_checkShape)
        {
            return true;
        }

        Analyzer ana = new(s, checkgeom);
        bool val = ana.IsValid();
        return val;
    }
}
