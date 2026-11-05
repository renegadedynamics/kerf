// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_OutLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Contours;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>
/// A shape and its outlined shape for a projector: each face rebuilt with its edges split at the outline vertices,
/// its internal outlines (silhouettes) and its isolines as internal wires, with the data that maps them back.
/// </summary>
/// <remarks>
/// <para>A working object (a transient), as OCCT's.</para>
/// <para>
/// OCCT's outlining writes into the original shape: the vertices of the edges it splits get point representations on
/// the new edges (HLRTopoBRep_DSFiller's <c>ProcessEdges</c>, HLRTopoBRep_FaceIsoLiner's <c>MakeIsoLine</c>). So
/// <see cref="Fill"/> outlines the caller's shape in a working overlay (unit 10H6, <see cref="WorkingOverlay"/>; in
/// place inside an operation's overlay that takes writes, as HLRBRep_ShapeToHLR's <c>Load</c> opens one around the
/// outlining and its reads): the vertices' new representations go to working copies, and the data and the maps of
/// adaptors refer to the caller's own sub-shapes, as OCCT's. At the end the outlined shape holds new TShapes for the
/// vertices that took a representation (and the caller's edges holding them); the hidden-line results are new edges. <see cref="OutLinedShape()"/> is a publication point (frozen);
/// <see cref="InternalOutLinedShape"/> doesn't freeze.
/// </para>
/// </remarks>
internal class OutLiner : Transient
{
    private Shape _originalShape = new();
    private Shape _outLinedShape = new();
    private readonly TopologyData _ds = new();

    public OutLiner()
    {
    }

    public OutLiner(in Shape oriSh)
    {
        _originalShape = oriSh;
    }

    public OutLiner(in Shape oriS, in Shape outS)
    {
        _originalShape = oriS;
        _outLinedShape = outS;
    }

    public void OriginalShape(in Shape oriS) => _originalShape = oriS;

    /// <summary>The caller's original shape.</summary>
    public Shape OriginalShape() => _originalShape;

    public void OutLinedShape(in Shape outS) => _outLinedShape = outS;

    /// <summary>The outlined shape (frozen).</summary>
    public Shape OutLinedShape()
    {
        _outLinedShape.TShape()?.FreezeGraph();
        return _outLinedShape;
    }

    /// <summary><see cref="OutLinedShape()"/> without freezing, for ported code.</summary>
    internal Shape InternalOutLinedShape() => _outLinedShape;

    public TopologyData DataStructure() => _ds;

    /// <summary>
    /// Builds the outlined shape for the projector P (when the original shape is set and the outlined shape is not),
    /// with nbIso isolines per face direction; MST caches the faces' adaptors.
    /// </summary>
    public void Fill(in AlgorithmProjector p, Dictionary<Shape, AdaptorTool> mst, int nbIso)
    {
        if (!_originalShape.IsNull)
        {
            if (_outLinedShape.IsNull)
            {
                // Kerf: outlined in a working overlay (unit 10H6), unless one that takes writes is open here
                using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_originalShape);

                Vector3D vecz = new(0.0, 0.0, 1.0);
                Transform3D tr = p.Transformation();
                tr = tr.Inverted();
                vecz = vecz.Transformed(tr);
                Contour fo = new();
                if (p.Perspective())
                {
                    Point3D eye = new Point3D().WithXYZ(p.Focus() * vecz.XYZ());
                    fo.Init(eye);
                }
                else
                {
                    Direction3D dirZ = new(vecz);
                    fo.Init(new Vector3D(dirZ));
                }
                DSFiller.Insert(_originalShape, fo, _ds, mst, nbIso);
                BuildShape(mst);
                if (overlay is not null)
                {
                    _outLinedShape = overlay.Finish(_outLinedShape);
                }
            }
        }
    }

    /// <summary>Builds the outlined face of F in S.</summary>
    private void ProcessFace(in Face f, ref Shape s,
                             Dictionary<Shape, AdaptorTool> mst)
    {
        BrepBuilder b = new();
        Explorer exE = new(), exW = new();

        IndexedMap<Shape, NCollection_List<Shape>> veMap =
            new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(f, ShapeType.Vertex, ShapeType.Edge, veMap);

        Shape nf; // = F;
        nf = f.EmptyCopied();

        for (exW.Init(f, ShapeType.Wire); exW.More(); exW.Next())
        {
            Wire w = new();
            b.MakeWire(ref w);

            for (exE.Init(exW.Current(), ShapeType.Edge); exE.More(); exE.Next())
            {
                Edge E = ShapeCasts.Edge(exE.Current());
                if (_ds.EdgeHasSplE(E))
                {
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(_ds.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        Edge newE = ShapeCasts.Edge(itS.Value());
                        newE.Orient(E.Orientation);
                        _ds.AddOldS(newE, E);
                        b.Add(w, newE);
                    }
                }
                else
                {
                    b.Add(w, E);
                }
            }
            b.Add(nf, w); // add the new wire in the new face.
        }
        _ds.AddIntL(f);
        NCollection_List<Shape> outL = _ds.AddOutL(f);

        if (_ds.FaceHasIntL(f))
        { // get the InternalOutLines on face F
            Wire w = new();
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(_ds.FaceIntL(f)); itE.More(); itE.Next())
            {
                Edge E = ShapeCasts.Edge(itE.Value());
                E.Orient(Internal);

                // Check, if outline edge coincides with face boundary
                EdgeCurveAdaptor C = new(E);
                double par = 0.34 * C.FirstParameter() + 0.66 * C.LastParameter();
                Point3D p = C.Value(par);
                Vertex V1, V2, v1, v2;
                Exploration.Vertices(E, out V1, out V2);
                bool sameEdge = false;
                if (!V1.IsNull && veMap.ContainsKey(V1))
                {
                    NCollection_List<Shape> eList = veMap.GetValue(V1);
                    NCollection_List<Shape>.Iterator it = new(eList);
                    for (; it.More(); it.Next())
                    {
                        Edge aE = ShapeCasts.Edge(it.Value());
                        Exploration.Vertices(aE, out v1, out v2);

                        if ((V1.IsSame(v1) && V2.IsSame(v2)) || (V1.IsSame(v2) && V2.IsSame(v1)))
                        {
                            EdgeCurveAdaptor c = new(aE);
                            if ((C.GetType() == Line) && (c.GetType() == Line))
                            {
                                sameEdge = true;
                                break;
                            }
                            else
                            {
                                CurvePointExtrema ext = new(p, c);
                                if (ext.IsDone)
                                {
                                    int ne = ext.ExtCount;
                                    if (ne > 0)
                                    {
                                        double dist = RealLast();
                                        int ec;
                                        for (ec = 1; ec <= ne; ++ec)
                                        {
                                            dist = Min(dist, ext.SquareDistance(ec));
                                        }

                                        if (dist <= 1.0e-14)
                                        {
                                            sameEdge = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (sameEdge)
                {
                    outL.Append(E);
                    continue;
                }

                if (_ds.EdgeHasSplE(E))
                {
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(_ds.EdgeSplE(E)); itS.More(); itS.Next())
                    {
                        Shape newE = itS.Value();
                        newE.Orient(Internal);
                        if (w.IsNull)
                        {
                            b.MakeWire(ref w);
                        }
                        _ds.AddOldS(newE, f);
                        b.Add(w, newE);
                    }
                }
                else
                {
                    if (w.IsNull)
                    {
                        b.MakeWire(ref w);
                    }
                    _ds.AddOldS(E, f);
                    b.Add(w, E);
                }
            }
            if (!w.IsNull)
            {
                b.Add(nf, w); // add the new wire in the new face.
            }
        }

        if (_ds.FaceHasIsoL(f))
        { // get the IsoLines on face F
            Wire w = new();
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(_ds.FaceIsoL(f)); itE.More(); itE.Next())
            {
                Edge e = ShapeCasts.Edge(itE.Value());
                e.Orient(Internal);
                if (_ds.EdgeHasSplE(e))
                { // normally IsoLines are never split.
                    NCollection_List<Shape>.Iterator itS = new();
                    for (itS.Initialize(_ds.EdgeSplE(e)); itS.More(); itS.Next())
                    {
                        Shape newE = itS.Value();
                        newE.Orient(Internal);
                        if (w.IsNull)
                        {
                            b.MakeWire(ref w);
                        }
                        _ds.AddOldS(newE, f);
                        b.Add(w, newE);
                    }
                }
                else
                {
                    if (w.IsNull)
                    {
                        b.MakeWire(ref w);
                    }
                    _ds.AddOldS(e, f);
                    b.Add(w, e);
                }
            }
            if (!w.IsNull)
            {
                b.Add(nf, w); // add the new wire in the new face.
            }
        }
        _ds.AddOldS(nf, f);
        // OCCT copies the adaptors' tool into the new face's entry; the port's entries share it (decision 1096)
        mst[nf] = mst.GetValue(f);
        b.Add(s, nf); // add the face in the shell.
    }

    /// <summary>Builds the outlined shape: the shells and the free faces outlined, the free edges as they are.</summary>
    private void BuildShape(Dictionary<Shape, AdaptorTool> mst)
    {
        Explorer exshell = new(), exface = new(), exedge = new();
        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        _outLinedShape = compound;
        HashSet<Shape> shapeMap = new(ShapeHasher.Instance);

        for (exshell.Init(_originalShape, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell (open or close)

            Shell theShell = new();
            b.MakeShell(ref theShell);
            theShell.Closed(exshell.Current().ClosedFlag);

            for (exface.Init(exshell.Current(), ShapeType.Face); exface.More(); exface.Next())
            {
                if (shapeMap.Add(exface.Current()))
                {
                    Shape shell = theShell;
                    ProcessFace(ShapeCasts.Face(exface.Current()), ref shell, mst);
                }
            }
            b.Add(_outLinedShape, theShell);
        }

        for (exface.Init(_originalShape, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (shapeMap.Add(exface.Current()))
            {
                ProcessFace(ShapeCasts.Face(exface.Current()), ref _outLinedShape, mst);
            }
        }

        for (exedge.Init(_originalShape, ShapeType.Edge, ShapeType.Face); exedge.More(); exedge.Next())
        { // edges not in a face
            b.Add(_outLinedShape, exedge.Current());
        }
    }
}
