// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepWire.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Engine.Exchange.Step.Export.MakeWireError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// Maps a TopoDS_Wire (with its relative orientation in the face) to a STEP loop: a poly_loop in a faceted context,
/// otherwise an edge_loop (or a vertex_loop for a wire made of seam edges only).
/// </summary>
public sealed class MakeStepWire : ExportRoot
{
    private StepTopologicalRepresentationItem _result;
    private MakeWireError _error;

    public MakeStepWire()
    {
        _error = MakeWireError.WireOther;
        _done = false;
    }

    public MakeStepWire(in Wire w, ExportTool t, FinderProcess fp, in Factors localFactors)
    {
        _done = false;
        Init(w, t, fp, localFactors);
    }

    public MakeStepWire(in Wire w, ExportTool t, FinderProcess fp)
        : this(w, t, fp, new Factors())
    {
    }

    public void Init(in Wire aWire, ExportTool aTool, FinderProcess fp, in Factors localFactors)
    {
        // ----------------------------------------------------------------
        // The Wire is given in its relative orientation (i.e. in the face)
        // ----------------------------------------------------------------
        aTool.SetCurrentWire(aWire);

        if (aTool.IsBound(aWire))
        {
            _error = MakeWireError.WireDone;
            _done = true;
            _result = aTool.Find(aWire);
            return;
        }

        if (aWire.Orientation == Internal || aWire.Orientation == External)
        {
            var errShape = new ShapeMapper(aWire);
            fp.AddWarning(errShape, " Wire(internal/external) from Non Manifold Topology");
            _error = MakeWireError.NonManifoldWire;
            _done = false;
            return;
        }

        var mySeq = new NCollection_Sequence<Transient>();

        // --------
        // Polyloop
        // --------

        if (aTool.Faceted())
        {
            StepVertexPoint vertexPoint;
            StepPoint point;
            StepTopologicalRepresentationItem gpms;
            Vertex topoDSVertex1, topoDSVertex2;

            var mkVertex = new MakeStepVertex();

            for (var itW = new WireExplorer(aWire, aTool.CurrentFace()); itW.More(); itW.Next())
            {
                Edge currentEdge = itW.Current();
                if (currentEdge.Orientation == Forward)
                {
                    Exploration.Vertices(currentEdge, out topoDSVertex1, out topoDSVertex2);
                }
                else
                {
                    Exploration.Vertices(currentEdge, out topoDSVertex2, out topoDSVertex1);
                }

                mkVertex.Init(topoDSVertex1, aTool, fp, localFactors);
                if (mkVertex.IsDone)
                {
                    vertexPoint = mkVertex.Value() as StepVertexPoint;
                    point = vertexPoint.VertexGeometry();
                    mySeq.Append(point);
                }
                else
                {
                    var errShape = new ShapeMapper(aWire);
                    fp.AddWarning(errShape, " a Vertex Point not mapped");
                    _error = MakeWireError.WireOther;
                    _done = false;
                    return;
                }
            }

            int nbPoints = mySeq.Length();
            if (nbPoints >= 3)
            {
                var polygon = new NCollection_HArray1<StepCartesianPoint>(1, nbPoints);
                for (int i = 1; i <= nbPoints; i++)
                {
                    polygon.SetValue(i, mySeq.Value(i) as StepCartesianPoint);
                }

                var pl = new StepPolyLoop();
                var aName = new TCollection_HAsciiString("");
                pl.Init(aName, polygon);

                aTool.Bind(aWire, pl);
                _error = MakeWireError.WireDone;
                _done = true;
                _result = pl;
                return;
            }
            else
            {
                var errShape = new ShapeMapper(aWire);
                fp.AddWarning(errShape, " PolyLoop: Wire has less than 3 points");
                _error = MakeWireError.WireOther;
                _done = false;
                return;
            }
        }

        // --------
        // EdgeLoop
        // --------

        else
        {
            StepTopologicalRepresentationItem gpms;
            StepEdge epms;
            StepOrientedEdge orientedEdge;

            var mkEdge = new MakeStepEdge();

            Wire forwardWire = ShapeCasts.Wire(aWire.Oriented(Forward));
            var stw = new WireFixer(forwardWire, aTool.CurrentFace(), global::Kerf.Engine.Foundation.Precision.Confusion());
            // for toroidal like surfaces we need to use both (3d and 2d) mode to correctly reorder the
            // edges
            stw.FixReorder(true);
            // OCCT.NET: the fix works on copies of frozen edges; a result gives the wire data back the caller's own
            // edges, which the tool keys the edges' entities by (decision 1250)
            stw.Wire();
            ExtendWireData extWire = stw.WireData();

            //: abv 04.05.00: CAX-IF TRJ4: writing complete sphere with single vertex_loop
            // check that whole wire is one seam (perhaps made of several seam edges)
            // pdn remove degenerated pcurves

            // collect not degenerated edges
            var extWire2 = new ExtendWireData();
            for (int ie = 1; ie <= extWire.EdgeCount; ie++)
            {
                Edge edge = extWire.Edge(ie);
                if (!Tool.Degenerated(edge))
                {
                    extWire2.Add(edge);
                }
            }

            // check for seam edges
            int nb = extWire2.EdgeCount;
            if (nb % 2 == 0)
            {
                int ie;
                // check if two adjacent edges are the same
                for (ie = 1; ie < nb; ie++)
                {
                    if (extWire2.Edge(ie).IsSame(extWire2.Edge(ie + 1)))
                    {
                        break;
                    }
                }

                // if found seam edges
                if (ie < nb)
                {
                    // make the first edge from pair last
                    extWire2.SetLast(ie);
                    for (ie = nb / 2 + 1; ie <= nb; ie++)
                    {
                        if (!extWire2.Edge(ie).IsSame(extWire2.Edge(nb - ie + 1)))
                        {
                            break;
                        }
                    }

                    if (ie > nb)
                    {
                        // make vertex_loop
                        var sae = new EdgeAnalysis();
                        Vertex v = sae.FirstVertex(extWire2.Edge(1));
                        var mkV = new MakeStepVertex(v, aTool, fp, localFactors);
                        var vloop = new StepVertexLoop();
                        var name = new TCollection_HAsciiString("");
                        vloop.Init(name, mkV.Value() as StepVertex);
                        aTool.Bind(aWire, vloop);
                        _error = MakeWireError.WireDone;
                        _done = true;
                        _result = vloop;
                        return;
                    }
                }
            }

            for (int nEdge = 1; nEdge <= extWire.EdgeCount; nEdge++)
            {
                Edge edge = extWire.Edge(nEdge);

                // ---------------------------------
                // --- Is the edge Degenerated ? ---
                // ---------------------------------

                Curve2D theC2d = Tool.CurveOnSurface(edge, aTool.CurrentFace(), out double cf, out double cl);
                if (Tool.Degenerated(edge))
                {
                    var errShape = new ShapeMapper(aWire);
                    fp.AddWarning(errShape, " EdgeLoop: Degenerated Pcurve not mapped");
                    continue;
                }
                else
                {
                    mkEdge.Init(edge, aTool, fp, localFactors);
                    if (mkEdge.IsDone)
                    {
                        orientedEdge = new StepOrientedEdge();
                        epms = mkEdge.Value() as StepEdge;
                        var aName = new TCollection_HAsciiString("");
                        orientedEdge.Init(aName, epms, (edge.Orientation == Forward));
                        mySeq.Append(orientedEdge);
                    }
                    else
                    {
                        var errShape = new ShapeMapper(aWire);
                        fp.AddWarning(errShape, " EdgeLoop: an Edge not mapped");
                        _error = MakeWireError.WireOther;
                        _done = false;
                        return;
                    }
                }
            }

            int nbEdges = mySeq.Length();
            if (nbEdges > 0)
            {
                var list = new NCollection_HArray1<StepOrientedEdge>(1, nbEdges);
                for (int i = 1; i <= nbEdges; i++)
                {
                    list.SetValue(i, mySeq.Value(i) as StepOrientedEdge);
                }

                var epmsl = new StepEdgeLoop();
                var aName = new TCollection_HAsciiString("");
                epmsl.Init(aName, list);
                aTool.Bind(aWire, epmsl);
                _done = true;
                _result = epmsl;
                return;
            }
            else
            {
                var errShape = new ShapeMapper(aWire);
                fp.AddWarning(errShape, " No Edges of this Wire were mapped");
                _error = MakeWireError.WireOther;
                _done = false;
                return;
            }
        }
    }

    public void Init(in Wire aWire, ExportTool aTool, FinderProcess fp)
        => Init(aWire, aTool, fp, new Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepTopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeStepWire::Value() - no result");
        return _result;
    }

    public MakeWireError Error() => _error;
}
