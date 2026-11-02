// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepWire.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using static Occt.TopAbs_Orientation;
using static Occt.TopoDSToStep_MakeWireError;

namespace Occt;

/// <summary>
/// Maps a TopoDS_Wire (with its relative orientation in the face) to a STEP loop: a poly_loop in a faceted context,
/// otherwise an edge_loop (or a vertex_loop for a wire made of seam edges only).
/// </summary>
public sealed class TopoDSToStep_MakeStepWire : TopoDSToStep_Root
{
    private StepShape_TopologicalRepresentationItem myResult;
    private TopoDSToStep_MakeWireError myError;

    public TopoDSToStep_MakeStepWire()
    {
        myError = TopoDSToStep_WireOther;
        done = false;
    }

    public TopoDSToStep_MakeStepWire(in TopoDS_Wire W, TopoDSToStep_Tool T, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        done = false;
        Init(W, T, FP, theLocalFactors);
    }

    public TopoDSToStep_MakeStepWire(in TopoDS_Wire W, TopoDSToStep_Tool T, Transfer_FinderProcess FP)
        : this(W, T, FP, new StepData_Factors())
    {
    }

    public void Init(in TopoDS_Wire aWire, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        // ----------------------------------------------------------------
        // The Wire is given in its relative orientation (i.e. in the face)
        // ----------------------------------------------------------------
        aTool.SetCurrentWire(aWire);

        if (aTool.IsBound(aWire))
        {
            myError = TopoDSToStep_WireDone;
            done = true;
            myResult = aTool.Find(aWire);
            return;
        }

        if (aWire.Orientation() == TopAbs_INTERNAL || aWire.Orientation() == TopAbs_EXTERNAL)
        {
            var errShape = new TransferBRep_ShapeMapper(aWire);
            FP.AddWarning(errShape, " Wire(internal/external) from Non Manifold Topology");
            myError = TopoDSToStep_NonManifoldWire;
            done = false;
            return;
        }

        var mySeq = new NCollection_Sequence<Standard_Transient>();

        // --------
        // Polyloop
        // --------

        if (aTool.Faceted())
        {
            StepShape_VertexPoint VertexPoint;
            StepGeom_Point Point;
            StepShape_TopologicalRepresentationItem Gpms;
            TopoDS_Vertex TopoDSVertex1, TopoDSVertex2;

            var MkVertex = new TopoDSToStep_MakeStepVertex();

            for (var ItW = new BRepTools_WireExplorer(aWire, aTool.CurrentFace()); ItW.More(); ItW.Next())
            {
                TopoDS_Edge CurrentEdge = ItW.Current();
                if (CurrentEdge.Orientation() == TopAbs_FORWARD)
                {
                    TopExp.Vertices(CurrentEdge, out TopoDSVertex1, out TopoDSVertex2);
                }
                else
                {
                    TopExp.Vertices(CurrentEdge, out TopoDSVertex2, out TopoDSVertex1);
                }

                MkVertex.Init(TopoDSVertex1, aTool, FP, theLocalFactors);
                if (MkVertex.IsDone())
                {
                    VertexPoint = MkVertex.Value() as StepShape_VertexPoint;
                    Point = VertexPoint.VertexGeometry();
                    mySeq.Append(Point);
                }
                else
                {
                    var errShape = new TransferBRep_ShapeMapper(aWire);
                    FP.AddWarning(errShape, " a Vertex Point not mapped");
                    myError = TopoDSToStep_WireOther;
                    done = false;
                    return;
                }
            }

            int nbPoints = mySeq.Length();
            if (nbPoints >= 3)
            {
                var aPolygon = new NCollection_HArray1<StepGeom_CartesianPoint>(1, nbPoints);
                for (int i = 1; i <= nbPoints; i++)
                {
                    aPolygon.SetValue(i, mySeq.Value(i) as StepGeom_CartesianPoint);
                }

                var PL = new StepShape_PolyLoop();
                var aName = new TCollection_HAsciiString("");
                PL.Init(aName, aPolygon);

                aTool.Bind(aWire, PL);
                myError = TopoDSToStep_WireDone;
                done = true;
                myResult = PL;
                return;
            }
            else
            {
                var errShape = new TransferBRep_ShapeMapper(aWire);
                FP.AddWarning(errShape, " PolyLoop: Wire has less than 3 points");
                myError = TopoDSToStep_WireOther;
                done = false;
                return;
            }
        }

        // --------
        // EdgeLoop
        // --------

        else
        {
            StepShape_TopologicalRepresentationItem Gpms;
            StepShape_Edge Epms;
            StepShape_OrientedEdge OrientedEdge;

            var MkEdge = new TopoDSToStep_MakeStepEdge();

            TopoDS_Wire ForwardWire = TopoDS.Wire(aWire.Oriented(TopAbs_FORWARD));
            var STW = new ShapeFix_Wire(ForwardWire, aTool.CurrentFace(), Precision.Confusion());
            // for toroidal like surfaces we need to use both (3d and 2d) mode to correctly reorder the
            // edges
            STW.FixReorder(true);
            // OCCT.NET: the fix works on copies of frozen edges; a result gives the wire data back the caller's own
            // edges, which the tool keys the edges' entities by (decision 13007)
            STW.Wire();
            ShapeExtend_WireData anExtWire = STW.WireData();

            //: abv 04.05.00: CAX-IF TRJ4: writing complete sphere with single vertex_loop
            // check that whole wire is one seam (perhaps made of several seam edges)
            // pdn remove degenerated pcurves

            // collect not degenerated edges
            var anExtWire2 = new ShapeExtend_WireData();
            for (int ie = 1; ie <= anExtWire.NbEdges(); ie++)
            {
                TopoDS_Edge anEdge = anExtWire.Edge(ie);
                if (!BRep_Tool.Degenerated(anEdge))
                {
                    anExtWire2.Add(anEdge);
                }
            }

            // check for seam edges
            int nb = anExtWire2.NbEdges();
            if (nb % 2 == 0)
            {
                int ie;
                // check if two adjacent edges are the same
                for (ie = 1; ie < nb; ie++)
                {
                    if (anExtWire2.Edge(ie).IsSame(anExtWire2.Edge(ie + 1)))
                    {
                        break;
                    }
                }

                // if found seam edges
                if (ie < nb)
                {
                    // make the first edge from pair last
                    anExtWire2.SetLast(ie);
                    for (ie = nb / 2 + 1; ie <= nb; ie++)
                    {
                        if (!anExtWire2.Edge(ie).IsSame(anExtWire2.Edge(nb - ie + 1)))
                        {
                            break;
                        }
                    }

                    if (ie > nb)
                    {
                        // make vertex_loop
                        var sae = new ShapeAnalysis_Edge();
                        TopoDS_Vertex V = sae.FirstVertex(anExtWire2.Edge(1));
                        var mkV = new TopoDSToStep_MakeStepVertex(V, aTool, FP, theLocalFactors);
                        var vloop = new StepShape_VertexLoop();
                        var name = new TCollection_HAsciiString("");
                        vloop.Init(name, mkV.Value() as StepShape_Vertex);
                        aTool.Bind(aWire, vloop);
                        myError = TopoDSToStep_WireDone;
                        done = true;
                        myResult = vloop;
                        return;
                    }
                }
            }

            for (int nEdge = 1; nEdge <= anExtWire.NbEdges(); nEdge++)
            {
                TopoDS_Edge anEdge = anExtWire.Edge(nEdge);

                // ---------------------------------
                // --- Is the edge Degenerated ? ---
                // ---------------------------------

                Geom2d_Curve theC2d = BRep_Tool.CurveOnSurface(anEdge, aTool.CurrentFace(), out double cf, out double cl);
                if (BRep_Tool.Degenerated(anEdge))
                {
                    var errShape = new TransferBRep_ShapeMapper(aWire);
                    FP.AddWarning(errShape, " EdgeLoop: Degenerated Pcurve not mapped");
                    continue;
                }
                else
                {
                    MkEdge.Init(anEdge, aTool, FP, theLocalFactors);
                    if (MkEdge.IsDone())
                    {
                        OrientedEdge = new StepShape_OrientedEdge();
                        Epms = MkEdge.Value() as StepShape_Edge;
                        var aName = new TCollection_HAsciiString("");
                        OrientedEdge.Init(aName, Epms, (anEdge.Orientation() == TopAbs_FORWARD));
                        mySeq.Append(OrientedEdge);
                    }
                    else
                    {
                        var errShape = new TransferBRep_ShapeMapper(aWire);
                        FP.AddWarning(errShape, " EdgeLoop: an Edge not mapped");
                        myError = TopoDSToStep_WireOther;
                        done = false;
                        return;
                    }
                }
            }

            int nbEdges = mySeq.Length();
            if (nbEdges > 0)
            {
                var aList = new NCollection_HArray1<StepShape_OrientedEdge>(1, nbEdges);
                for (int i = 1; i <= nbEdges; i++)
                {
                    aList.SetValue(i, mySeq.Value(i) as StepShape_OrientedEdge);
                }

                var Epmsl = new StepShape_EdgeLoop();
                var aName = new TCollection_HAsciiString("");
                Epmsl.Init(aName, aList);
                aTool.Bind(aWire, Epmsl);
                done = true;
                myResult = Epmsl;
                return;
            }
            else
            {
                var errShape = new TransferBRep_ShapeMapper(aWire);
                FP.AddWarning(errShape, " No Edges of this Wire were mapped");
                myError = TopoDSToStep_WireOther;
                done = false;
                return;
            }
        }
    }

    public void Init(in TopoDS_Wire aWire, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP)
        => Init(aWire, aTool, FP, new StepData_Factors());

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepShape_TopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeStepWire::Value() - no result");
        return myResult;
    }

    public TopoDSToStep_MakeWireError Error() => myError;
}
