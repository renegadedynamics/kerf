// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_RemoveInternalWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Removes the internal wires (holes) of faces whose area is less than the minimal area, and, in the remove-face mode
/// (the default), the faces whose outer wire is made only of edges of removed wires.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's modifiable references (<see cref="MinArea"/>, <see cref="RemoveFaceMode"/>)
/// return <c>ref</c>. It only records removals in its context and rebuilds the shape with it, so the shape given is
/// never modified (decision 999). <see cref="GetResult"/> freezes what it returns; ported callers read
/// <see cref="InternalGetResult"/>.
/// </remarks>
public class RemoveInternalWires : UpgradeTool
{
    /// <summary>The status.</summary>
    protected int _status;

    private Shape _shape;
    private Shape _result;
    private double _minArea;
    private bool _removeFacesMode;
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _edgeFaces =
        new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _removeEdges =
        new(ShapeHasher.Instance);
    private readonly NCollection_Sequence<Shape> _removedFaces = new();
    private readonly NCollection_Sequence<Shape> _removeWires = new();

    /// <summary>Empty constructor.</summary>
    public RemoveInternalWires()
    {
        _shape = new Shape();
        _result = new Shape();
        _minArea = 0.0;
        _removeFacesMode = true;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var context = new ReShapeBuilder();
        SetContext(context);
    }

    /// <summary>Initializes by a shape.</summary>
    public RemoveInternalWires(in Shape shape)
    {
        _shape = new Shape();
        _result = new Shape();
        var context = new ReShapeBuilder();
        SetContext(context);
        Init(shape);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in Shape shape)
    {
        _shape = shape;
        Context()!.InternalApply(shape);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, _edgeFaces);
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _minArea = 0.0;
        _removeFacesMode = true;
    }

    /// <summary>Removes all the internal wires with an area less than the minimal area; true if something was done.</summary>
    public bool Perform()
    {
        Clear();
        if (_shape.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }
        var expF = new Explorer(_shape, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            RemoveSmallWire(f, new Wire());
        }
        if (_removeFacesMode)
        {
            RemoveSmallFaces();
        }

        _result = Context()!.InternalApply(_shape);
        return Status(ExtendStatus.Done);
    }

    /// <summary>
    /// Removes the internal wires given (wires), or those of the faces given (faces), with an area less than the
    /// minimal area; true if something was done.
    /// </summary>
    public bool Perform(NCollection_Sequence<Shape> seqShapes)
    {
        if (_shape.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }
        Clear();
        var wireFaces = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        int i = 1, nb = seqShapes.Length();
        for (; i <= nb; i++)
        {
            Shape s = seqShapes.Value(i);
            if (s.ShapeType() == ShapeType.Face)
            {
                RemoveSmallWire(s, new Wire());
            }
            else if (s.ShapeType() == ShapeType.Wire)
            {
                if (wireFaces.Extent() == 0)
                {
                    Exploration.MapShapesAndAncestors(_shape, ShapeType.Wire, ShapeType.Face, wireFaces);
                }
                if (wireFaces.Contains(s))
                {
                    NCollection_List<Shape> alfaces = wireFaces.FindFromKey(s);
                    foreach (Shape face in alfaces)
                    {
                        RemoveSmallWire(face, s);
                    }
                }
            }
        }
        if (_removeFacesMode)
        {
            RemoveSmallFaces();
        }
        _result = Context()!.InternalApply(_shape);
        return Status(ExtendStatus.Done);
    }

    /// <summary>Gets the resulting shape, frozen.</summary>
    public Shape GetResult()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="GetResult"/> without freezing it, for ported callers.</summary>
    internal Shape InternalGetResult() => _result;

    /// <summary>The minimal area of the holes kept (all holes of smaller area are removed).</summary>
    public ref double MinArea() => ref _minArea;

    /// <summary>
    /// The mode for removing the faces whose outer wires consist only of edges of removed internal wires (true by
    /// default).
    /// </summary>
    public ref bool RemoveFaceMode() => ref _removeFacesMode;

    /// <summary>Returns the removed faces (the tool's own sequence).</summary>
    public NCollection_Sequence<Shape> RemovedFaces() => _removedFaces;

    /// <summary>Returns the removed wires (the tool's own sequence).</summary>
    public NCollection_Sequence<Shape> RemovedWires() => _removeWires;

    /// <summary>
    /// Queries the status of the last Perform: OK (nothing done), DONE1 (internal wires removed), DONE2 (small faces
    /// removed), FAIL1 (no initial shape), FAIL2 (a sub-shape given isn't in the initial shape).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Clears the sequences and the temporary map.</summary>
    protected void Clear()
    {
        _removeEdges.Clear();
        _removedFaces.Clear();
        _removeWires.Clear();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    // Removes the internal wires with a contour area less than the minimal area.
    private void RemoveSmallWire(in Shape face, in Shape wire)
    {
        Face f = ShapeCasts.Face(face);
        Wire outW = ShapeAnalysisAnalysis.OuterWire(f);
        var it = new Iterator(f);
        for (; it.More(); it.Next())
        {
            if (it.Value().ShapeType() != ShapeType.Wire || it.Value().IsSame(outW))
            {
                continue;
            }
            // occ::handle<ShapeExtend_WireData> asewd = new  ShapeExtend_WireData();
            Wire w = ShapeCasts.Wire(it.Value());
            if (!wire.IsNull && !wire.IsSame(w))
            {
                continue;
            }
            double area = ShapeAnalysisAnalysis.ContourArea(w);
            if (area < _minArea - Kerf.Engine.Foundation.Precision.Confusion())
            {
                Context()!.Remove(w);
                _removeWires.Append(w);
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                if (!_removeFacesMode)
                {
                    continue;
                }

                var ite = new Iterator(w, false);
                for (; ite.More(); ite.Next())
                {
                    Shape e = ite.Value();
                    if (_removeEdges.ContainsKey(e))
                    {
                        _removeEdges.GetValue(e).Append(f);
                    }
                    else
                    {
                        var alfaces = new NCollection_List<Shape>();
                        alfaces.Append(f);
                        _removeEdges[e] = alfaces;
                    }
                }
            }
        }
    }

    // Removes the faces whose outer wire consists of edges of removed (small internal) wires.
    private void RemoveSmallFaces()
    {
        int i = 1;
        for (; i <= _removeWires.Length(); i++)
        {
            Shape wire = _removeWires.Value(i);
            var ite = new Iterator(wire, false);
            var faceCandidates = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            // collecting all faces containing edges from removed wire
            for (; ite.More(); ite.Next())
            {
                Shape edge = ite.Value();
                if (!_edgeFaces.Contains(edge))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                    continue;
                }
                NCollection_List<Shape> lface1 = _edgeFaces.FindFromKey(edge);
                NCollection_List<Shape> lface2 = _removeEdges.GetValue(edge);
                // OCCT's second iterator is not reset for each face of the first list
                var aliter2 = new NCollection_List<Shape>.Iterator(lface2);
                foreach (Shape face1 in lface1)
                {
                    Shape f = Context()!.InternalApply(face1);
                    if (f.IsNull)
                    {
                        continue;
                    }
                    bool isFind = false;
                    for (; aliter2.More() && !isFind; aliter2.Next())
                    {
                        Shape f2 = Context()!.InternalApply(aliter2.Value());
                        isFind = f.IsSame(f2);
                    }

                    if (!isFind)
                    {
                        Wire wout = ShapeAnalysisAnalysis.OuterWire(ShapeCasts.Face(f));
                        bool isOuter = false;
                        var iter = new Iterator(wout, false);
                        for (; iter.More() && !isOuter; iter.Next())
                        {
                            isOuter = edge.IsSame(iter.Value());
                        }
                        if (isOuter)
                        {
                            faceCandidates.Add(f);
                        }
                    }
                }
            }

            // remove faces which have outer wire consist of only
            // edges from removed wires and
            // seam edges for faces based on conic surface or
            // in the case of a few faces based on the same conic surface
            // the edges belogining these faces.
            int k = 1;
            for (; k <= faceCandidates.Extent(); k++)
            {
                Shape f = faceCandidates.FindKey(k);
                Wire outW = ShapeAnalysisAnalysis.OuterWire(ShapeCasts.Face(f));
                var asewd = new ExtendWireData(outW);
                int n = 1, nbE = asewd.EdgeCount;
                int nbNotRemoved = 0;
                for (; n <= nbE; n++)
                {
                    if (asewd.IsSeam(n))
                    {
                        continue;
                    }
                    Edge e = asewd.Edge(n);
                    if (!_removeEdges.ContainsKey(e))
                    {
                        NCollection_List<Shape> lface3 = _edgeFaces.FindFromKey(e);
                        foreach (Shape face3 in lface3)
                        {
                            Shape f2 = Context()!.InternalApply(face3);
                            if (f2.IsNull)
                            {
                                continue;
                            }
                            if (!f.IsSame(f2) && !faceCandidates.Contains(f2))
                            {
                                nbNotRemoved++;
                            }
                        }
                    }
                }

                if (nbNotRemoved == 0)
                {
                    Context()!.Remove(f);
                    _removedFaces.Append(f);
                }
            }
        }

        if (_removedFaces.Length() != 0)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
    }
}
