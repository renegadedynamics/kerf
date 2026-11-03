// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitCommonVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Splits the vertices two wires of a face have in common: valid in the BRep model, not in STEP, so before writing
/// STEP each wire must get its own vertex.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Init"/>, <see cref="Perform"/>, then <see cref="Shape"/>. It writes into
/// no shape it is given: the edges holding a common vertex are copied with a new vertex
/// (<see cref="BuildEdgeBuilder"/>'s copy) and replaced through the context, whose apply builds new shapes. So frozen
/// shapes need no copy at the entry, and the context answers for the caller's shapes as in OCCT. <see cref="Shape"/>
/// freezes the result.
/// </remarks>
public class SplitCommonVertexFixer : RootFixer
{
    // OCCT declares its own private myShape, hiding ShapeFix_Root's (which the messages are attached to, and which
    // stays null here).
    private new Shape _shape;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private Shape _result;
    private int          _status;
#pragma warning restore CS0414

    /// <summary>Empty constructor.</summary>
    public SplitCommonVertexFixer()
    {
        _shape  = new Shape();
        _result = new Shape();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (and creates a context if there is none).</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        _result = _shape;
        Context()!.InternalApply(_shape);
    }

    /// <summary>Splits the common vertices of the wires of each face.</summary>
    public void Perform()
    {
        ShapeType st = _shape.ShapeType();
        if (st > ShapeType.Face)
        {
            return;
        }
        for (var itf = new Explorer(_shape, ShapeType.Face); itf.More(); itf.Next())
        {
            Shape tmpFace = Context()!.InternalApply(itf.Current());
            Face  f       = ShapeCasts.Face(tmpFace);
            if (f.IsNull)
            {
                continue;
            }
            // analys face and split if necessary
            var wires = new NCollection_Sequence<Shape>();
            for (var itw = new Iterator(f, false); itw.More(); itw.Next())
            {
                if (itw.Value().ShapeType() != ShapeType.Wire)
                {
                    continue;
                }
                wires.Append(itw.Value());
            }
            if (wires.Length() < 2)
            {
                continue;
            }
            var mapVV = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
            mapVV.Clear();
            for (int nw1 = 1; nw1 < wires.Length(); nw1++)
            {
                Wire w1 = ShapeCasts.Wire(wires.Value(nw1));
                for (int nw2 = nw1 + 1; nw2 <= wires.Length(); nw2++)
                {
                    Wire w2    = ShapeCasts.Wire(wires.Value(nw2));
                    var         sewd2 = new ExtendWireData(w2);

                    for (var expv1 = new Explorer(w1, ShapeType.Vertex); expv1.More(); expv1.Next())
                    {
                        Vertex v1 = ShapeCasts.Vertex(expv1.Current());
                        for (var expv2 = new Explorer(w2, ShapeType.Vertex); expv2.More(); expv2.Next())
                        {
                            Vertex v2 = ShapeCasts.Vertex(expv2.Current());
                            if (v1 == v2)
                            {
                                // common vertex exists
                                Vertex vnew = new();
                                if (mapVV.IsBound(v2))
                                {
                                    vnew = ShapeCasts.Vertex(mapVV.Find(v2));
                                }
                                else
                                {
                                    Point3D p   = Tool.Pnt(v2);
                                    double tol = Tool.Tolerance(v2);
                                    var    b   = new BrepBuilder();
                                    b.MakeVertex(ref vnew, p, tol);
                                    mapVV.Bind(v2, vnew);
                                }
                                var sbe = new BuildEdgeBuilder();
                                var sae = new EdgeAnalysis();
                                for (int ne2 = 1; ne2 <= sewd2.EdgeCount; ne2++)
                                {
                                    Edge   e       = sewd2.Edge(ne2);
                                    Vertex fv      = sae.FirstVertex(e);
                                    Vertex lv      = sae.LastVertex(e);
                                    bool          isCoinc = false;
                                    if (fv == v2)
                                    {
                                        fv      = vnew;
                                        isCoinc = true;
                                    }
                                    if (lv == v2)
                                    {
                                        lv      = vnew;
                                        isCoinc = true;
                                    }
                                    if (isCoinc)
                                    {
                                        Edge newE = sbe.InternalCopyReplaceVertices(e, fv, lv);
                                        Context()!.Replace(e, newE);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (!mapVV.IsEmpty())
            {
                SendWarning(new Msg("Fix.SplitCommonVertex.MSG0"));
            }
        }

        _shape = Context()!.InternalApply(_shape);
    }

    /// <summary>Returns the resulting shape, frozen.</summary>
    public Shape Shape()
    {
        Shape shape = _shape;
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>The resulting shape, unfrozen, for ported code that goes on building it.</summary>
    internal Shape InternalShape() => _shape;
}
