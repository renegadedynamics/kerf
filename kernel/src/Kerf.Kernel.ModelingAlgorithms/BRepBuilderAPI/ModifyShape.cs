// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ModifyShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_ModifyShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// The constant topology modifications done with a <see cref="Modifier"/>: <see cref="Copy"/>,
/// <see cref="Transform"/>, <see cref="GTransform"/>, <see cref="NurbsConvert"/>.
/// </summary>
/// <remarks>
/// The input is only read: the modifier builds new shapes and reuses the sub-shapes it leaves alone. The result is
/// read through the modifier's <c>InternalModifiedShape</c> and frozen by <see cref="MakeShape.Shape"/>
/// and <see cref="ModifiedShape"/> (design 4.2), so a derived algorithm can go on updating it first (NurbsConvert's
/// vertex tolerances).
/// </remarks>
internal abstract class ModifyShape : MakeShape
{
    protected Modifier _modifier;
    protected Shape _initialShape = new();
    protected Modification? _modification;

    /// <summary>Empty constructor.</summary>
    protected ModifyShape()
    {
        _modifier = new Modifier();
    }

    /// <summary>Initializes the modifier with the shape <paramref name="s"/>.</summary>
    protected ModifyShape(in Shape s)
    {
        _modifier = new Modifier(s);
        _initialShape = s;
    }

    /// <summary>Sets the modification.</summary>
    protected ModifyShape(Modification m)
    {
        _modifier = new Modifier();
        _modification = m;
    }

    /// <summary>Initializes the modifier with the shape <paramref name="s"/> and performs the modification <paramref name="m"/>.</summary>
    protected ModifyShape(in Shape s, Modification m)
    {
        _modifier = new Modifier(s);
        _initialShape = s;
        _modification = m;
        DoModif();
    }

    private void DoModif()
    {
        if (_initialShape.IsNull || _modification is null)
        {
            throw new NullObjectException();
        }
        _modifier.Perform(_modification);
        if (_modifier.IsDone)
        {
            Done();
            _shape = _modifier.InternalModifiedShape(_initialShape);
        }
        else
        {
            NotDone();
        }
    }

    /// <summary>Performs the previously given modification on the shape <paramref name="s"/>.</summary>
    protected void DoModif(in Shape s)
    {
        if (!s.IsEqual(_initialShape) || !IsDone)
        {
            _initialShape = s;
            _modifier.Init(s);
            DoModif();
        }
    }

    /// <summary>Performs the modification <paramref name="m"/> on the previously given shape.</summary>
    protected void DoModif(Modification m)
    {
        _modification = m;
        DoModif();
    }

    /// <summary>Performs the modification <paramref name="m"/> on the shape <paramref name="s"/>.</summary>
    protected void DoModif(in Shape s, Modification m)
    {
        _initialShape = s;
        _modifier.Init(s);
        _modification = m;
        DoModif();
    }

    /// <summary>
    /// The modified shape corresponding to <paramref name="s"/>, the initial shape or one of its sub-shapes (frozen).
    /// </summary>
    /// <exception cref="NoSuchObjectException"><paramref name="s"/> isn't the initial shape or one of its sub-shapes.</exception>
    public Shape ModifiedShape(in Shape s)
    {
        Shape shape = InternalModifiedShape(s);
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary><see cref="ModifiedShape"/> without freezing (OCCT's virtual), for ported code.</summary>
    internal virtual Shape InternalModifiedShape(in Shape s)
    {
        return _modifier.InternalModifiedShape(s);
    }

    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        _generated.Clear();
        _generated.Append(_modifier.InternalModifiedShape(f));
        return _generated;
    }
}
