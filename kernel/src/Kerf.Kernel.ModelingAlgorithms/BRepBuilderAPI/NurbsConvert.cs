// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_NurbsConvert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_NurbsConvert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Conversion of the whole geometry of a shape into NURBS geometry (planes excepted): the curves of its edges become
/// B-spline curves and the surfaces of its faces B-spline surfaces.
/// </summary>
/// <remarks>
/// After the modifier (<see cref="NurbsConvertModification"/>), the vertices of the edges whose tolerance
/// the conversion raised get that tolerance: new vertices of the result are raised in place, and the input's own
/// vertices are replaced by new, more tolerant ones (the input isn't changed). The result is read unfrozen until then.
/// </remarks>
public class NurbsConvert : ModifyShape
{
    private readonly NCollection_DataMap<Shape, Shape> _vtxToReplace =
        new(ShapeHasher.Instance);
    private readonly ReShape _subs = new();

    /// <summary>A framework for converting a shape; give the shape with <see cref="Perform"/>.</summary>
    public NurbsConvert()
    {
        _modification = new NurbsConvertModification();
    }

    /// <summary>Converts the geometry of <paramref name="s"/> into NURBS geometry.</summary>
    /// <param name="s">The shape.</param>
    /// <param name="copy">Unused (as in OCCT).</param>
    public NurbsConvert(in Shape s, bool copy = false)
    {
        _modification = new NurbsConvertModification();
        Perform(s, copy);
    }

    /// <summary>Converts the geometry of <paramref name="s"/> into NURBS geometry.</summary>
    /// <param name="s">The shape.</param>
    /// <param name="copy">Unused (as in OCCT).</param>
    public void Perform(in Shape s, bool copy = false)
    {
        DoModif(s, _modification!);
        CorrectVertexTol();
    }

    private void CorrectVertexTol()
    {
        var initVertices = new HashSet<Shape>(ShapeHasher.Instance);
        var exp = new Explorer(_initialShape, ShapeType.Vertex);
        for (; exp.More(); exp.Next())
        {
            initVertices.Add(exp.Current());
        }

        var modif = (NurbsConvertModification)_modification!;
        var bb = new BrepBuilder();
        _vtxToReplace.Clear();
        var eIter = new NCollection_List<Shape>.Iterator(modif.GetUpdatedEdges());
        for (; eIter.More(); eIter.Next())
        {
            Shape e = eIter.Value();

            double eTol = Tool.Tolerance(ShapeCasts.Edge(e));
            var iter = new Iterator(e);
            for (; iter.More(); iter.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(iter.Value());
                if (initVertices.Contains(vtx))
                {
                    if (_vtxToReplace.IsBound(vtx))
                    {
                        bb.UpdateVertex(ShapeCasts.Vertex(_vtxToReplace[vtx]), eTol + Epsilon(eTol));
                    }
                    else
                    {
                        double vTol = Tool.Tolerance(vtx);
                        if (vTol < eTol)
                        {
                            Vertex newVtx = new();
                            Point3D vPnt = Tool.Pnt(vtx);
                            bb.MakeVertex(ref newVtx, vPnt, eTol + Epsilon(eTol));
                            newVtx.Orientation(vtx.Orientation());
                            _vtxToReplace.Bind(vtx, newVtx);
                        }
                    }
                }
                else
                {
                    // a vertex the modifier made: unfrozen until Shape()
                    bb.UpdateVertex(vtx, eTol + Epsilon(eTol));
                }
            }
        }

        if (_vtxToReplace.IsEmpty())
        {
            return;
        }

        _subs.Clear();
        foreach (KeyValuePair<Shape, Shape> iter2 in _vtxToReplace)
        {
            _subs.Replace(iter2.Key, iter2.Value);
        }
        _subs.InternalApply(_shape);
        _shape = _subs.InternalValue(_shape);
    }

    internal override Shape InternalModifiedShape(in Shape s)
    {
        if (s.ShapeType() == ShapeType.Vertex)
        {
            if (_vtxToReplace.IsBound(s))
            {
                return _vtxToReplace[s];
            }
        }
        if (_vtxToReplace.IsEmpty())
        {
            return _modifier.InternalModifiedShape(s);
        }
        else
        {
            Shape ns = _modifier.InternalModifiedShape(s);
            return _subs.InternalValue(ns);
        }
    }

    internal override NCollection_List<Shape> InternalModified(in Shape f)
    {
        _generated.Clear();
        if (f.ShapeType() == ShapeType.Vertex)
        {
            if (_vtxToReplace.IsBound(f))
            {
                _generated.Append(_vtxToReplace[f]);
            }
            else
            {
                _generated.Append(_modifier.InternalModifiedShape(f));
            }
        }
        else
        {
            if (_vtxToReplace.IsEmpty())
            {
                _generated.Append(_modifier.InternalModifiedShape(f));
            }
            else
            {
                Shape ns = _modifier.InternalModifiedShape(f);
                _generated.Append(_subs.InternalValue(ns));
            }
        }
        return _generated;
    }
}
