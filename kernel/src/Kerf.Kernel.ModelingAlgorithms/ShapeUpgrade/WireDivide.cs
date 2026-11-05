// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_WireDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_WireDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Divides the edges of a wire lying on a face (or of a free wire, or a free edge) with a criterion: splits the 3D
/// curve and the pcurve on the face of each edge, and copies the edge's other pcurves; if the 3D curve is split the
/// pcurve is split as well, and vice versa.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The wire given is not modified: every edge split is replaced by new edges on new
/// vertices (the end vertices are empty copies of the edge's, so SameParameter can't raise the input's tolerances),
/// and the replacements are recorded in the context (<see cref="ReShapeBuilder"/>), which must be set, so
/// sharing is kept. The audit of its write sites (plan 2.2, decision 999): every BRep_Builder call writes into an
/// edge made by ShapeBuild_Edge.CopyReplaceVertices, a vertex made here (MakeVertex, EmptyCopied) or the new wires;
/// so a frozen wire needs no copy.
/// </para>
/// <para>
/// <see cref="Wire"/> freezes what it returns; ported callers read <see cref="InternalWire"/>.
/// </para>
/// </remarks>
internal class WireDivide : UpgradeTool
{
    /// <summary>The supporting face (null for free wires and edges).</summary>
    protected Face _face;
    /// <summary>The wire.</summary>
    protected Wire _wire;
    /// <summary>The status.</summary>
    protected int _status;
    /// <summary>The mode for splitting 3D curves (0: of free edges only, 1: of shared edges only, 2: all).</summary>
    protected int _edgeMode;

    private SplitCurve3d? _splitCurve3dTool;
    private SplitCurve2d? _splitCurve2dTool;
    private EdgeDivide? _edgeDivide;
    private TransferParametersAnalysis? _transferParamTool;
    private FixSmallCurves? _fixSmallCurveTool;

    /// <summary>Empty constructor.</summary>
    public WireDivide()
    {
        _face = new Face();
        _wire = new Wire();
        _status = 0;
        _splitCurve3dTool = new SplitCurve3d();
        _splitCurve2dTool = new SplitCurve2d();
        _transferParamTool = new TransferParametersProjAnalysis();
        _edgeMode = 2;
        _fixSmallCurveTool = new FixSmallCurves();
        _edgeDivide = new EdgeDivide();
    }

    /// <summary>Initializes by wire and face.</summary>
    public void Init(in Wire w, in Face f)
    {
        _wire = w;
        _face = f;
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Initializes by wire and surface.</summary>
    public void Init(in Wire w, Surface s)
    {
        _wire = w;
        var mkf = new LibraryMakeFace(s, Kerf.Engine.Foundation.Precision.Confusion());
        _face = mkf.InternalFace();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Loads the working wire.</summary>
    public void Load(in Wire w)
    {
        _wire = w;
    }

    /// <summary>Makes a wire of the edge and loads it.</summary>
    public void Load(in Edge e)
    {
        var makeWire = new LibraryMakeWire(e);
        if (makeWire.IsDone)
        {
            Load(makeWire.InternalWire());
        }
    }

    /// <summary>Sets the supporting surface by a face.</summary>
    public void SetFace(in Face f)
    {
        _face = f;
    }

    /// <summary>Sets the supporting surface.</summary>
    public void SetSurface(Surface s)
    {
        var mkf = new LibraryMakeFace(s, Kerf.Engine.Foundation.Precision.Confusion());
        _face = mkf.InternalFace();
    }

    /// <summary>Sets the supporting surface with a location.</summary>
    public void SetSurface(Surface s, in Placement l)
    {
        var b = new BrepBuilder();
        b.MakeFace(ref _face, s, l, Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OCCT's file-static CorrectSplitValues: makes the split values transferred between the 3D curve and the pcurve
    // agree with the original ones and increase.
    private static void CorrectSplitValues(NCollection_HSequence<double> orig3d,
                                           NCollection_HSequence<double> orig2d,
                                           NCollection_HSequence<double> new2d,
                                           NCollection_HSequence<double> new3d)
    {
        double preci = Kerf.Engine.Foundation.Precision.PConfusion();
        int len3d = orig3d.Length();
        int len2d = orig2d.Length();
        var fixNew2d = new NCollection_Array1<bool>(1, len3d);
        fixNew2d.Init(false);
        var fixNew3d = new NCollection_Array1<bool>(1, len2d);
        fixNew3d.Init(false);
        double last3d = orig3d.Value(len3d);
        double last2d = orig2d.Value(len2d);

        int i;
        for (i = 1; i <= len3d; i++)
        {
            double par = new2d.Value(i);
            int index = 0;
            for (int j = 1; j <= len2d && index == 0; j++)
            {
                if (Math.Abs(par - orig2d.Value(j)) < preci)
                {
                    index = j;
                }
            }
            if (index != 0 && !fixNew3d[index])
            {
                double newPar = orig2d.Value(index);
                new2d.SetValue(i, newPar);
                fixNew2d[i] = true;
                double newPar3d = orig3d.Value(i);
                new3d.SetValue(index, newPar3d);
                fixNew3d[index] = true;
            }
        }

        for (i = 1; i <= len2d; i++)
        {
            double par = new3d.Value(i);
            int index = 0;
            for (int j = 1; j <= len3d && index == 0; j++)
            {
                if (Math.Abs(par - orig3d.Value(j)) < preci)
                {
                    index = j;
                }
            }
            if (index != 0 && !fixNew2d[index])
            {
                double newPar = orig3d.Value(index);
                new3d.SetValue(i, newPar);
                fixNew3d[i] = true;
                double newPar2d = orig2d.Value(i);
                new2d.SetValue(index, newPar2d);
                fixNew2d[index] = true;
            }
        }

        double dpreci = 2 * preci;
        for (i = 1; i < len3d; i++)
        {
            double dist = new2d.Value(i + 1) - new2d.Value(i);
            if (dist < preci)
            {
                if (fixNew2d[i + 1])
                {
                    // changing
                    double tmp = new2d.Value(i + 1);
                    new2d.SetValue(i + 1, new2d.Value(i) + dpreci);
                    new2d.SetValue(i, tmp);
                    fixNew2d[i] = true;
                    fixNew2d[i + 1] = false;
                }
                else
                {
                    new2d.SetValue(i + 1, new2d.Value(i) + dpreci);
                }
            }
        }
        if (new2d.Value(len3d) > last3d)
        {
            int ind;
            for (ind = len3d; ind > 1 && !fixNew2d[ind]; ind--)
            {
                ;
            }
            double lastFix = new2d.Value(ind);
            for (i = len3d; i >= ind; i--)
            {
                new2d.SetValue(i, lastFix);
                lastFix -= dpreci;
            }
        }

        for (i = 1; i < len2d; i++)
        {
            double dist = new3d.Value(i + 1) - new3d.Value(i);
            if (dist < preci)
            {
                if (fixNew3d[i + 1])
                {
                    // changing
                    double tmp = new3d.Value(i + 1);
                    new3d.SetValue(i + 1, new3d.Value(i) + dpreci);
                    new3d.SetValue(i, tmp);
                    fixNew3d[i] = true;
                    fixNew3d[i + 1] = false;
                }
                else
                {
                    new3d.SetValue(i + 1, new3d.Value(i) + dpreci);
                }
            }
        }
        if (new3d.Value(len2d) > last2d)
        {
            int ind;
            for (ind = len2d; ind > 1 && !fixNew3d[ind]; ind--)
            {
                ;
            }
            double lastFix = new3d.Value(ind);
            for (i = len2d; i >= ind; i--)
            {
                new3d.SetValue(i, lastFix);
                lastFix -= dpreci;
            }
        }
    }

    /// <summary>
    /// Computes the resulting wire by splitting all its edges with the splitting criteria, recording the changes in the
    /// context. Without a supporting face only the 3D criteria apply.
    /// </summary>
    public virtual void Perform()
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        var b = new BrepBuilder();
        var sae = new EdgeAnalysis();

        var newWire = new Wire();
        b.MakeWire(ref newWire);
        Placement loc = new();
        Surface? surf = null;
        if (!_face.IsNull)
        {
            surf = Tool.Surface(_face, out loc);
        }

        bool isSplit3d = true;
        switch (_edgeMode)
        {
            case 0:
                if (!_face.IsNull)
                {
                    isSplit3d = false;
                }
                break;
            case 1:
                if (_face.IsNull)
                {
                    isSplit3d = false;
                }
                break;
            default:
                break;
        }
        _edgeDivide!.SetFace(_face);
        if (isSplit3d)
        {
            _edgeDivide.SetSplitCurve3dTool(GetSplitCurve3dTool());
        }
        _edgeDivide.SetSplitCurve2dTool(GetSplitCurve2dTool());
        for (var itW = new Iterator(_wire, false); itW.More(); itW.Next())
        {
            // for each Edge:
            Shape sh = Context()!.InternalApply(itW.Value(), ShapeType.Shape);
            for (var exp = new Explorer(sh, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                // skip degenerated edges (and also INTERNAL/EXTERNAL, to avoid failures)
                if (e.Orientation == Internal || e.Orientation == External)
                {
                    b.Add(newWire, e);
                    continue;
                }

                if (!_edgeDivide.Compute(e))
                {
                    b.Add(newWire, e);
                    continue;
                }
                // first iteration: getting split knots
                // on 3D curve: preliminary

                TransferParametersAnalysis theTransferParamTool = GetTransferParamTool()!;
                theTransferParamTool.SetMaxTolerance(MaxTolerance());
                theTransferParamTool.Init(e, _face);
                bool wasSR = theTransferParamTool.IsSameRange;

                // on pcurve(s): all knots
                // assume that if seam-edge, its pcurve1 and pcurve2 has the same split knots !!!
                NCollection_HSequence<double>? theKnots3d = _edgeDivide.Knots3d();
                NCollection_HSequence<double>? theKnots2d = _edgeDivide.Knots2d();

                // second iteration: transfer parameters and build segments
                NCollection_HSequence<double>? splitValues2d = null;
                NCollection_HSequence<double>? splitValues3d = null;
                if (_edgeDivide.HasCurve2d && _edgeDivide.HasCurve3d)
                {
                    splitValues2d = theTransferParamTool.Perform(theKnots3d!, true);
                    splitValues3d = theTransferParamTool.Perform(theKnots2d!, false);
                    CorrectSplitValues(theKnots3d!, theKnots2d!, splitValues2d, splitValues3d);
                }

                SplitCurve3d theSplit3dTool = _edgeDivide.GetSplitCurve3dTool()!;
                SplitCurve2d theSplit2dTool = _edgeDivide.GetSplitCurve2dTool()!;

                if (_edgeDivide.HasCurve2d)
                {
                    if (theKnots3d is not null)
                    {
                        splitValues2d!.Remove(1);
                        splitValues2d.Remove(splitValues2d.Length());
                        theSplit2dTool.SetSplitValues(splitValues2d);
                    }
                    theSplit2dTool.Build(true);
                }
                if (_edgeDivide.HasCurve3d)
                {
                    if (theKnots2d is not null)
                    {
                        splitValues3d!.Remove(1);
                        splitValues3d.Remove(splitValues3d.Length());
                        theSplit3dTool.SetSplitValues(splitValues3d);
                    }
                    theSplit3dTool.Build(true);
                }

                // get 2d and 3d split values which should be the same
                if (_edgeDivide.HasCurve2d)
                {
                    theKnots2d = theSplit2dTool.SplitValues();
                }
                if (_edgeDivide.HasCurve3d)
                {
                    theKnots3d = theSplit3dTool.SplitValues();
                }

                bool isSeam = false;
                if (!_face.IsNull)
                {
                    isSeam = Tool.IsClosed(e, _face);
                }
                NCollection_HArray1<Curve2D>? theSegments2d = null;
                if (_edgeDivide.HasCurve2d)
                {
                    theSegments2d = theSplit2dTool.GetCurves();
                }
                NCollection_HArray1<Curve2D>? theSegments2dR = null;
                if (isSeam)
                {
                    Shape tmpE = e.Reversed();
                    Edge erev = ShapeCasts.Edge(tmpE);
                    if (sae.PCurve(erev, _face, out Curve2D? c2, out double f2, out double l2, false))
                    {
                        theSplit2dTool.Init(c2!, f2, l2);
                        if (theKnots2d is not null)
                        {
                            theSplit2dTool.SetSplitValues(theKnots2d);
                        }
                        theSplit2dTool.Perform(true);
                        NCollection_HSequence<double> revKnots2d = theSplit2dTool.SplitValues()!;
                        if (revKnots2d.Length() != theKnots2d!.Length())
                        {
                            isSeam = false;
                            // OCCT_DEBUG output left out: seam has different splitting values on pcurves
                        }
                        else
                        {
                            theSegments2dR = theSplit2dTool.GetCurves();
                        }
                    }
                    else
                    {
                        isSeam = false;
                    }
                }

                // Exploring theEdge
                Vertex v1o = Exploration.FirstVertex(e, false);
                Vertex v2o = Exploration.LastVertex(e, false);
                bool isForward = (e.Orientation == Forward);
                double tolEdge = Tool.Tolerance(e);
                bool isDeg = Tool.Degenerated(e);

                // Copy vertices to protect original shape against SameParamseter
                Shape emptyCopiedV1 = v1o.EmptyCopied();
                Vertex v1 = ShapeCasts.Vertex(emptyCopiedV1);
                Context()!.Replace(v1o, v1);
                Vertex v2;
                if (v1o.IsSame(v2o))
                {
                    Shape tmpV = v1.Oriented(v2o.Orientation);
                    v2 = ShapeCasts.Vertex(tmpV);
                }
                else
                {
                    Shape emptyCopied = v2o.EmptyCopied();
                    v2 = ShapeCasts.Vertex(emptyCopied);
                    Context()!.Replace(v2o, v2);
                }

                // collect NM vertices
                double af = 0.0, al = 0.0;
                Curve3D? c3d = null;
                var adCS = new CurveOnSurface();
                if (_edgeDivide.HasCurve3d)
                {
                    sae.Curve3d(e, out c3d, out af, out al, false);
                }
                else if (_edgeDivide.HasCurve2d && surf is not null)
                {
                    sae.PCurve(e, _face, out Curve2D? c2d, out af, out al, false);
                    SurfaceAdaptorBase adS = new SurfaceAdaptor(surf);
                    Curve2DAdaptorBase ac2d = new CurveAdaptor2D(c2d!, af, al);
                    adCS.Load(ac2d);
                    adCS.Load(adS);
                }
                var seqNMVertices = new NCollection_Sequence<Shape>();
                var seqParNM = new NCollection_Sequence<double>();
                var itv = new Iterator(e, false);
                var sac = new CurveAnalysis();
                for (; itv.More(); itv.Next())
                {
                    if (itv.Value().Orientation == Internal
                        || itv.Value().Orientation == External)
                    {
                        Vertex vold = ShapeCasts.Vertex(itv.Value());
                        seqNMVertices.Append(vold);
                        Point3D aP = Tool.Pnt(ShapeCasts.Vertex(vold));
                        double ppar;
                        if (c3d is not null)
                        {
                            sac.Project(c3d, aP, Precision(), out _, out ppar, af, al, false);
                        }
                        else
                        {
                            sac.Project(adCS, aP, Precision(), out _, out ppar);
                        }
                        seqParNM.Append(ppar);
                    }
                }

                // creating new edge(s)
                NCollection_HArray1<Curve3D>? theSegments3d = null;
                if (_edgeDivide.HasCurve3d)
                {
                    theSegments3d = theSplit3dTool.GetCurves();
                }

                int nbc = 0;
                if (theSegments3d is not null)
                {
                    nbc = theSegments3d.Length();
                    if (theSegments2d is not null)
                    {
                        int nbc2d = theSegments2d.Length();
                        if (nbc != nbc2d)
                        {
                            // OCCT_DEBUG output left out: number of intervals not equal for 2d 3d
                            nbc = Math.Min(nbc, nbc2d);
                        }
                    }
                }
                else if (theSegments2d is not null)
                { // if theSegments have different length ???
                    nbc = theSegments2d.Length();
                }

                if (nbc <= 1 && !theSplit3dTool.Status(ExtendStatus.Done)
                    && !theSplit2dTool.Status(ExtendStatus.Done))
                {
                    b.Add(newWire, e);
                    continue;
                }

                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);

                var resWire = new Wire();
                b.MakeWire(ref resWire);
                int numE = 0;
                Point3D pntV1 = Tool.Pnt(v1);
                // gp_Pnt pntV2 = BRep_Tool::Pnt(V2); // pntV2 not used - see below (skl)
                // double V2Tol = LimitTolerance( BRep_Tool::Tolerance(V2) ); // V2Tol not used - see below (skl)

                FixSmallCurves fixSmallCurveTool = GetFixSmallCurveTool()!; // gka Precision
                fixSmallCurveTool.SetMinTolerance(MinTolerance());
                fixSmallCurveTool.Init(e, _face);
                fixSmallCurveTool.SetSplitCurve3dTool(theSplit3dTool);
                fixSmallCurveTool.SetSplitCurve2dTool(theSplit2dTool);
                fixSmallCurveTool.SetPrecision(MinTolerance());
                int savnum = 0;
                double savParf = 0;
                int small = 0;
                for (int icurv = 1; icurv <= nbc; icurv++)
                {
                    Curve3D? theNewCurve3d = null;
                    if (theSegments3d is not null)
                    {
                        theNewCurve3d = theSegments3d.Value(icurv);
                    }

                    Curve2D? theNewPCurve1 = null;
                    if (theSegments2d is not null)
                    {
                        theNewPCurve1 = theSegments2d.Value(icurv);
                    }
                    Curve2D? revPCurve = null;
                    if (isSeam)
                    {
                        revPCurve = theSegments2dR!.Value(icurv);
                    }
                    // construction of the intermediate Vertex
                    var v = new Vertex();
                    if (icurv <= nbc && nbc != 1 && !isDeg)
                    {
                        double par, parf /*,SavParl*/;
                        Point3D p, p1, pm;
                        // if edge has 3d curve, take point from it
                        if (theNewCurve3d is not null)
                        {
                            if (theNewCurve3d is BoundedCurve3D)
                            {
                                par = theNewCurve3d.LastParameter;
                                parf = theNewCurve3d.FirstParameter;
                            }
                            else
                            {
                                par = theKnots3d!.Value(icurv + 1);
                                parf = theKnots3d.Value(icurv);
                            }
                            p = theNewCurve3d.Value(par);
                            p1 = theNewCurve3d.Value(parf);
                            pm = theNewCurve3d.Value((parf + par) / 2);
                        }
                        // else use pcurve and surface (suppose that both exist)
                        else
                        {
                            if (surf is null)
                            {
                                surf = Tool.Surface(_face, out loc);
                            }
                            if (theNewPCurve1 is BoundedCurve2D)
                            {
                                par = theNewPCurve1.LastParameter;
                                parf = theNewPCurve1.FirstParameter;
                            }
                            else
                            {
                                par = theKnots2d!.Value(icurv + 1);
                                parf = theKnots2d.Value(icurv);
                            }
                            Point2D p2d = theNewPCurve1!.Value(par);
                            Point2D p2df = theNewPCurve1.Value(parf);
                            Point2D p2dM = theNewPCurve1.Value((parf + par) / 2);
                            p = surf!.Value(p2d.X, p2d.Y);
                            p1 = surf.Value(p2df.X, p2df.Y);
                            pm = surf.Value(p2dM.X, p2dM.Y);
                            p = p.Transformed(loc.Transformation);
                            p1 = p1.Transformed(loc.Transformation);
                            pm = pm.Transformed(loc.Transformation);
                        }

                        if (p.DistanceTo(pntV1) < MinTolerance() && p.DistanceTo(pm) < MinTolerance()
                            && !_face.IsNull)
                        {
                            if (small == 0)
                            {
                                savParf = parf;
                                savnum = icurv;
                            }
                            // SavParl = par;
                            small++;
                            if (icurv == nbc)
                            {
                                Vertex vvv = v1;
                                vvv.Orient(v2.Orientation);
                                Context()!.Replace(v2, vvv);
                            }
                            continue;
                        }
                        if (small != 0)
                        {
                            if (p.DistanceTo(p1) > MinTolerance() || p.DistanceTo(pm) > MinTolerance())
                            {
                                // FixSmallCurveTool->Perform(prevEdge,theNewCurve3d,theNewPCurve1,revPCurve,SavParf,SavParl);
                                var pmid = new Point3D(0.5 * (pntV1.XYZ() + p1.XYZ()));
                                b.UpdateVertex(v1, pmid, 0);
                            }
                            else
                            {
                                Curve3D? atmpCurve = null;
                                Curve2D? atmpCurve2d1 = null, atmprepcurve = null;
                                if (fixSmallCurveTool.Approx(ref atmpCurve, ref atmpCurve2d1, ref atmprepcurve, ref savParf, ref par))
                                { // BRepTools
                                    theNewCurve3d = atmpCurve;
                                    theNewPCurve1 = atmpCurve2d1;
                                    revPCurve = atmprepcurve;
                                }
                                else
                                {
                                    var pmid = new Point3D(0.5 * (pntV1.XYZ() + p1.XYZ()));
                                    b.UpdateVertex(v1, pmid, 0);
                                }
                            }
                            small = 0;
                        }
                        // pdn
                        // (OCCT's commented-out reuse of the end vertices left out)
                        if (icurv != nbc)
                        {
                            b.MakeVertex(ref v, p, tolEdge); // tolerance of the edge
                            pntV1 = p;
                        }
                        else
                        {
                            v = v2;
                        }
                        // else  V2;
                        // }
                    }
                    // else V = V2;

                    Edge newEdge;
                    var sbe = new BuildEdgeBuilder();
                    if (isForward)
                    {
                        v1.Orient(Forward);
                        v.Orient(Reversed);
                        newEdge = sbe.InternalCopyReplaceVertices(e, v1, v);
                    }
                    else
                    {
                        v1.Orient(Reversed);
                        v.Orient(Forward);
                        newEdge = sbe.InternalCopyReplaceVertices(e, v, v1);
                    }
                    sbe.CopyPCurves(newEdge, e);
                    if (theNewCurve3d is not null)
                    {
                        b.UpdateEdge(newEdge, theNewCurve3d, 0.0);
                    }
                    else if (isDeg)
                    {
                        b.Degenerated(newEdge, true);
                    }
                    // (OCCT's commented-out pcurve updates left out)

                    double f3d = 0.0, l3d = 0.0;
                    if (savnum == 0)
                    {
                        savnum = icurv;
                    }
                    bool srNew;
                    if (theNewCurve3d is not null)
                    {
                        if (theNewCurve3d is BoundedCurve3D)
                        {
                            f3d = theNewCurve3d.FirstParameter;
                            l3d = theNewCurve3d.LastParameter;
                            srNew = ((f3d == theKnots3d!.Value(savnum)) && (l3d == theKnots3d.Value(icurv + 1)));
                        }
                        else
                        {
                            f3d = theKnots3d!.Value(savnum);
                            l3d = theKnots3d.Value(icurv + 1);
                            srNew = true;
                        }
                    }
                    else
                    {
                        srNew = true;
                    }

                    double f2d = 0, l2d = 0;
                    if (theNewPCurve1 is not null)
                    {
                        if (theNewPCurve1 is BoundedCurve2D)
                        {
                            f2d = theNewPCurve1.FirstParameter;
                            l2d = theNewPCurve1.LastParameter;
                            srNew &= ((f2d == theKnots2d!.Value(savnum)) && (l2d == theKnots2d.Value(icurv + 1)));
                        }
                        else
                        {
                            f2d = theKnots2d!.Value(savnum);
                            l2d = theKnots2d.Value(icurv + 1);
                        }
                    }
                    // if(!Savnum) Savnum = icurv;
                    if (theNewCurve3d is not null)
                    {
                        theTransferParamTool.TransferRange(newEdge,
                                                           theKnots3d!.Value(savnum),
                                                           theKnots3d.Value(icurv + 1),
                                                           false);
                    }
                    else
                    {
                        theTransferParamTool.TransferRange(newEdge,
                                                           theKnots2d!.Value(savnum),
                                                           theKnots2d.Value(icurv + 1),
                                                           true);
                    }
                    // (OCCT's commented-out CopyRanges left out)
                    savnum = 0;
                    if (!_face.IsNull && sae.PCurve(newEdge, _face, out _, out double setF, out double setL, false))
                    {
                        srNew &= ((setF == f2d) && (setL == l2d));
                    }

                    if (isSeam)
                    {
                        if (newEdge.Orientation == Forward)
                        {
                            b.UpdateEdge(newEdge, theNewPCurve1, revPCurve, _face, 0.0);
                        }
                        else
                        {
                            b.UpdateEdge(newEdge, revPCurve, theNewPCurve1, _face, 0.0);
                        }
                    }
                    else if (!_face.IsNull)
                    {
                        b.UpdateEdge(newEdge, theNewPCurve1, _face, 0.0);
                    }

                    if (theNewCurve3d is not null)
                    {
                        sbe.SetRange3d(newEdge, f3d, l3d);
                    }
                    if (theNewPCurve1 is not null)
                    {
                        b.Range(newEdge, _face, f2d, l2d);
                    }
                    if ((!wasSR || !srNew) && !Tool.Degenerated(newEdge))
                    {
                        b.SameRange(newEdge, false);
                    }

                    // addition NM vertices to new edges
                    double afpar = (_edgeDivide.HasCurve3d ? f3d : f2d);
                    double alpar = (_edgeDivide.HasCurve3d ? l3d : l2d);
                    for (int n = 1; n <= seqParNM.Length(); ++n)
                    {
                        double apar = seqParNM.Value(n);
                        Vertex vold = ShapeCasts.Vertex(seqNMVertices.Value(n));
                        Vertex nmVer = TransferParametersProjAnalysis.InternalCopyNMVertex(vold, newEdge, e);
                        Context()!.Replace(vold, nmVer);
                        if (Math.Abs(apar - afpar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            Context()!.Replace(nmVer, v1);
                        }
                        else if (Math.Abs(apar - alpar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            Context()!.Replace(nmVer, v);
                        }
                        else if (apar > afpar && apar < alpar)
                        {
                            b.Add(newEdge, nmVer);
                        }
                        else
                        {
                            continue;
                        }
                        seqNMVertices.Remove(n);
                        seqParNM.Remove(n);
                        n--;
                    }

                    b.Add(resWire, newEdge);
                    b.Add(newWire, newEdge);
                    numE++;
                    v1 = v;
                }
                if (numE != 0)
                {
                    resWire.Closed(Tool.IsClosed(resWire));
                    Context()!.Replace(e, resWire);
                }
                else
                {
                    Context()!.Remove(e);
                }
            }
        }
        if (Status(ExtendStatus.Done))
        {
            newWire.Closed(Tool.IsClosed(newWire));
            Shape tmpW = Context()!.InternalApply(newWire).Oriented(_wire.Orientation);
            _wire = ShapeCasts.Wire(tmpW);
        }
    }

    /// <summary>
    /// Returns the resulting wire (the initial one if not done, null if not loaded), frozen.
    /// </summary>
    public Wire Wire()
    {
        _wire.TShape()?.FreezeGraph();
        return _wire;
    }

    /// <summary><see cref="Wire"/> without freezing it, for ported callers.</summary>
    internal Wire InternalWire() => _wire;

    /// <summary>
    /// Queries the status of the last <see cref="Perform"/>: OK (no edge split), DONE1 (some edges split), FAIL1 (some
    /// edges have no 3D curve, skipped), FAIL2 (some edges have no pcurve, skipped).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(SplitCurve3d? splitCurve3dTool)
    {
        _splitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(SplitCurve2d? splitCurve2dTool)
    {
        _splitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    protected virtual SplitCurve3d? GetSplitCurve3dTool() => _splitCurve3dTool;

    /// <summary>Returns the tool for splitting pcurves.</summary>
    protected virtual SplitCurve2d? GetSplitCurve2dTool() => _splitCurve2dTool;

    /// <summary>Sets the tool for splitting edges.</summary>
    public void SetEdgeDivideTool(EdgeDivide? edgeDivideTool)
    {
        _edgeDivide = edgeDivideTool;
    }

    /// <summary>Returns the tool for splitting edges.</summary>
    public virtual EdgeDivide? GetEdgeDivideTool() => _edgeDivide;

    /// <summary>Sets the tool for transferring parameters between curves and pcurves.</summary>
    public void SetTransferParamTool(TransferParametersAnalysis? transferParam)
    {
        _transferParamTool = transferParam;
    }

    /// <summary>Returns the tool for transferring parameters.</summary>
    public virtual TransferParametersAnalysis? GetTransferParamTool() => _transferParamTool;

    /// <summary>Sets the mode for splitting 3D curves of edges: 0 free edges only, 1 shared edges only, 2 all.</summary>
    public void SetEdgeMode(int edgeMode)
    {
        _edgeMode = edgeMode;
    }

    /// <summary>Sets the tool for fixing small curves.</summary>
    public void SetFixSmallCurveTool(FixSmallCurves? fixSmallCurvesTool)
    {
        _fixSmallCurveTool = fixSmallCurvesTool;
    }

    /// <summary>Returns the tool for fixing small curves.</summary>
    public FixSmallCurves? GetFixSmallCurveTool() => _fixSmallCurveTool;
}
