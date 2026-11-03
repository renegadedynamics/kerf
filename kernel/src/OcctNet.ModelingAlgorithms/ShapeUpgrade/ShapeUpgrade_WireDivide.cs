// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_WireDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_WireDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Divides the edges of a wire lying on a face (or of a free wire, or a free edge) with a criterion: splits the 3D
/// curve and the pcurve on the face of each edge, and copies the edge's other pcurves; if the 3D curve is split the
/// pcurve is split as well, and vice versa.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The wire given is not modified: every edge split is replaced by new edges on new
/// vertices (the end vertices are empty copies of the edge's, so SameParameter can't raise the input's tolerances),
/// and the replacements are recorded in the context (<see cref="ShapeBuild_ReShape"/>), which must be set, so
/// sharing is kept. The audit of its write sites (plan 2.2, decision 999): every BRep_Builder call writes into an
/// edge made by ShapeBuild_Edge.CopyReplaceVertices, a vertex made here (MakeVertex, EmptyCopied) or the new wires;
/// so a frozen wire needs no copy.
/// </para>
/// <para>
/// <see cref="Wire"/> freezes what it returns; ported callers read <see cref="InternalWire"/>.
/// </para>
/// </remarks>
public class ShapeUpgrade_WireDivide : ShapeUpgrade_Tool
{
    /// <summary>The supporting face (null for free wires and edges).</summary>
    protected Face myFace;
    /// <summary>The wire.</summary>
    protected Wire myWire;
    /// <summary>The status.</summary>
    protected int myStatus;
    /// <summary>The mode for splitting 3D curves (0: of free edges only, 1: of shared edges only, 2: all).</summary>
    protected int myEdgeMode;

    private ShapeUpgrade_SplitCurve3d? mySplitCurve3dTool;
    private ShapeUpgrade_SplitCurve2d? mySplitCurve2dTool;
    private ShapeUpgrade_EdgeDivide? myEdgeDivide;
    private ShapeAnalysis_TransferParameters? myTransferParamTool;
    private ShapeUpgrade_FixSmallCurves? myFixSmallCurveTool;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_WireDivide()
    {
        myFace = new Face();
        myWire = new Wire();
        myStatus = 0;
        mySplitCurve3dTool = new ShapeUpgrade_SplitCurve3d();
        mySplitCurve2dTool = new ShapeUpgrade_SplitCurve2d();
        myTransferParamTool = new ShapeAnalysis_TransferParametersProj();
        myEdgeMode = 2;
        myFixSmallCurveTool = new ShapeUpgrade_FixSmallCurves();
        myEdgeDivide = new ShapeUpgrade_EdgeDivide();
    }

    /// <summary>Initializes by wire and face.</summary>
    public void Init(in Wire W, in Face F)
    {
        myWire = W;
        myFace = F;
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Initializes by wire and surface.</summary>
    public void Init(in Wire W, Surface S)
    {
        myWire = W;
        var mkf = new BRepLib_MakeFace(S, Kerf.Engine.Foundation.Precision.Confusion());
        myFace = mkf.InternalFace();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Loads the working wire.</summary>
    public void Load(in Wire W)
    {
        myWire = W;
    }

    /// <summary>Makes a wire of the edge and loads it.</summary>
    public void Load(in Edge E)
    {
        var MakeWire = new BRepLib_MakeWire(E);
        if (MakeWire.IsDone())
        {
            Load(MakeWire.InternalWire());
        }
    }

    /// <summary>Sets the supporting surface by a face.</summary>
    public void SetFace(in Face F)
    {
        myFace = F;
    }

    /// <summary>Sets the supporting surface.</summary>
    public void SetSurface(Surface S)
    {
        var mkf = new BRepLib_MakeFace(S, Kerf.Engine.Foundation.Precision.Confusion());
        myFace = mkf.InternalFace();
    }

    /// <summary>Sets the supporting surface with a location.</summary>
    public void SetSurface(Surface S, in Placement L)
    {
        var B = new BrepBuilder();
        B.MakeFace(ref myFace, S, L, Kerf.Engine.Foundation.Precision.Confusion());
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
        double Last3d = orig3d.Value(len3d);
        double Last2d = orig2d.Value(len2d);

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
        if (new2d.Value(len3d) > Last3d)
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
        if (new3d.Value(len2d) > Last2d)
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
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        var B = new BrepBuilder();
        var sae = new ShapeAnalysis_Edge();

        var newWire = new Wire();
        B.MakeWire(ref newWire);
        Placement Loc = new();
        Surface? Surf = null;
        if (!myFace.IsNull)
        {
            Surf = Tool.Surface(myFace, out Loc);
        }

        bool isSplit3d = true;
        switch (myEdgeMode)
        {
            case 0:
                if (!myFace.IsNull)
                {
                    isSplit3d = false;
                }
                break;
            case 1:
                if (myFace.IsNull)
                {
                    isSplit3d = false;
                }
                break;
            default:
                break;
        }
        myEdgeDivide!.SetFace(myFace);
        if (isSplit3d)
        {
            myEdgeDivide.SetSplitCurve3dTool(GetSplitCurve3dTool());
        }
        myEdgeDivide.SetSplitCurve2dTool(GetSplitCurve2dTool());
        for (var ItW = new Iterator(myWire, false); ItW.More(); ItW.Next())
        {
            // for each Edge:
            Shape sh = Context()!.InternalApply(ItW.Value(), ShapeType.Shape);
            for (var exp = new Explorer(sh, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge E = ShapeCasts.Edge(exp.Current());
                // skip degenerated edges (and also INTERNAL/EXTERNAL, to avoid failures)
                if (E.Orientation() == Internal || E.Orientation() == External)
                {
                    B.Add(newWire, E);
                    continue;
                }

                if (!myEdgeDivide.Compute(E))
                {
                    B.Add(newWire, E);
                    continue;
                }
                // first iteration: getting split knots
                // on 3D curve: preliminary

                ShapeAnalysis_TransferParameters theTransferParamTool = GetTransferParamTool()!;
                theTransferParamTool.SetMaxTolerance(MaxTolerance());
                theTransferParamTool.Init(E, myFace);
                bool wasSR = theTransferParamTool.IsSameRange();

                // on pcurve(s): all knots
                // assume that if seam-edge, its pcurve1 and pcurve2 has the same split knots !!!
                NCollection_HSequence<double>? theKnots3d = myEdgeDivide.Knots3d();
                NCollection_HSequence<double>? theKnots2d = myEdgeDivide.Knots2d();

                // second iteration: transfer parameters and build segments
                NCollection_HSequence<double>? SplitValues2d = null;
                NCollection_HSequence<double>? SplitValues3d = null;
                if (myEdgeDivide.HasCurve2d() && myEdgeDivide.HasCurve3d())
                {
                    SplitValues2d = theTransferParamTool.Perform(theKnots3d!, true);
                    SplitValues3d = theTransferParamTool.Perform(theKnots2d!, false);
                    CorrectSplitValues(theKnots3d!, theKnots2d!, SplitValues2d, SplitValues3d);
                }

                ShapeUpgrade_SplitCurve3d theSplit3dTool = myEdgeDivide.GetSplitCurve3dTool()!;
                ShapeUpgrade_SplitCurve2d theSplit2dTool = myEdgeDivide.GetSplitCurve2dTool()!;

                if (myEdgeDivide.HasCurve2d())
                {
                    if (theKnots3d is not null)
                    {
                        SplitValues2d!.Remove(1);
                        SplitValues2d.Remove(SplitValues2d.Length());
                        theSplit2dTool.SetSplitValues(SplitValues2d);
                    }
                    theSplit2dTool.Build(true);
                }
                if (myEdgeDivide.HasCurve3d())
                {
                    if (theKnots2d is not null)
                    {
                        SplitValues3d!.Remove(1);
                        SplitValues3d.Remove(SplitValues3d.Length());
                        theSplit3dTool.SetSplitValues(SplitValues3d);
                    }
                    theSplit3dTool.Build(true);
                }

                // get 2d and 3d split values which should be the same
                if (myEdgeDivide.HasCurve2d())
                {
                    theKnots2d = theSplit2dTool.SplitValues();
                }
                if (myEdgeDivide.HasCurve3d())
                {
                    theKnots3d = theSplit3dTool.SplitValues();
                }

                bool isSeam = false;
                if (!myFace.IsNull)
                {
                    isSeam = Tool.IsClosed(E, myFace);
                }
                NCollection_HArray1<Curve2D>? theSegments2d = null;
                if (myEdgeDivide.HasCurve2d())
                {
                    theSegments2d = theSplit2dTool.GetCurves();
                }
                NCollection_HArray1<Curve2D>? theSegments2dR = null;
                if (isSeam)
                {
                    Shape tmpE = E.Reversed();
                    Edge erev = ShapeCasts.Edge(tmpE);
                    if (sae.PCurve(erev, myFace, out Curve2D? c2, out double f2, out double l2, false))
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
                Vertex V1o = Exploration.FirstVertex(E, false);
                Vertex V2o = Exploration.LastVertex(E, false);
                bool isForward = (E.Orientation() == Forward);
                double TolEdge = Tool.Tolerance(E);
                bool isDeg = Tool.Degenerated(E);

                // Copy vertices to protect original shape against SameParamseter
                Shape emptyCopiedV1 = V1o.EmptyCopied();
                Vertex V1 = ShapeCasts.Vertex(emptyCopiedV1);
                Context()!.Replace(V1o, V1);
                Vertex V2;
                if (V1o.IsSame(V2o))
                {
                    Shape tmpV = V1.Oriented(V2o.Orientation());
                    V2 = ShapeCasts.Vertex(tmpV);
                }
                else
                {
                    Shape emptyCopied = V2o.EmptyCopied();
                    V2 = ShapeCasts.Vertex(emptyCopied);
                    Context()!.Replace(V2o, V2);
                }

                // collect NM vertices
                double af = 0.0, al = 0.0;
                Curve3D? c3d = null;
                var AdCS = new CurveOnSurface();
                if (myEdgeDivide.HasCurve3d())
                {
                    sae.Curve3d(E, out c3d, out af, out al, false);
                }
                else if (myEdgeDivide.HasCurve2d() && Surf is not null)
                {
                    sae.PCurve(E, myFace, out Curve2D? c2d, out af, out al, false);
                    SurfaceAdaptorBase AdS = new SurfaceAdaptor(Surf);
                    Curve2DAdaptorBase AC2d = new CurveAdaptor2D(c2d!, af, al);
                    AdCS.Load(AC2d);
                    AdCS.Load(AdS);
                }
                var aSeqNMVertices = new NCollection_Sequence<Shape>();
                var aSeqParNM = new NCollection_Sequence<double>();
                var aItv = new Iterator(E, false);
                var sac = new ShapeAnalysis_Curve();
                for (; aItv.More(); aItv.Next())
                {
                    if (aItv.Value().Orientation() == Internal
                        || aItv.Value().Orientation() == External)
                    {
                        Vertex aVold = ShapeCasts.Vertex(aItv.Value());
                        aSeqNMVertices.Append(aVold);
                        Point3D aP = Tool.Pnt(ShapeCasts.Vertex(aVold));
                        double ppar;
                        if (c3d is not null)
                        {
                            sac.Project(c3d, aP, Precision(), out _, out ppar, af, al, false);
                        }
                        else
                        {
                            sac.Project(AdCS, aP, Precision(), out _, out ppar);
                        }
                        aSeqParNM.Append(ppar);
                    }
                }

                // creating new edge(s)
                NCollection_HArray1<Curve3D>? theSegments3d = null;
                if (myEdgeDivide.HasCurve3d())
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

                if (nbc <= 1 && !theSplit3dTool.Status(ShapeExtend_DONE)
                    && !theSplit2dTool.Status(ShapeExtend_DONE))
                {
                    B.Add(newWire, E);
                    continue;
                }

                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);

                var resWire = new Wire();
                B.MakeWire(ref resWire);
                int numE = 0;
                Point3D pntV1 = Tool.Pnt(V1);
                // gp_Pnt pntV2 = BRep_Tool::Pnt(V2); // pntV2 not used - see below (skl)
                // double V2Tol = LimitTolerance( BRep_Tool::Tolerance(V2) ); // V2Tol not used - see below (skl)

                ShapeUpgrade_FixSmallCurves FixSmallCurveTool = GetFixSmallCurveTool()!; // gka Precision
                FixSmallCurveTool.SetMinTolerance(MinTolerance());
                FixSmallCurveTool.Init(E, myFace);
                FixSmallCurveTool.SetSplitCurve3dTool(theSplit3dTool);
                FixSmallCurveTool.SetSplitCurve2dTool(theSplit2dTool);
                FixSmallCurveTool.SetPrecision(MinTolerance());
                int Savnum = 0;
                double SavParf = 0;
                int Small = 0;
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
                    var V = new Vertex();
                    if (icurv <= nbc && nbc != 1 && !isDeg)
                    {
                        double par, parf /*,SavParl*/;
                        Point3D P, P1, PM;
                        // if edge has 3d curve, take point from it
                        if (theNewCurve3d is not null)
                        {
                            if (theNewCurve3d is BoundedCurve3D)
                            {
                                par = theNewCurve3d.LastParameter();
                                parf = theNewCurve3d.FirstParameter();
                            }
                            else
                            {
                                par = theKnots3d!.Value(icurv + 1);
                                parf = theKnots3d.Value(icurv);
                            }
                            P = theNewCurve3d.Value(par);
                            P1 = theNewCurve3d.Value(parf);
                            PM = theNewCurve3d.Value((parf + par) / 2);
                        }
                        // else use pcurve and surface (suppose that both exist)
                        else
                        {
                            if (Surf is null)
                            {
                                Surf = Tool.Surface(myFace, out Loc);
                            }
                            if (theNewPCurve1 is BoundedCurve2D)
                            {
                                par = theNewPCurve1.LastParameter();
                                parf = theNewPCurve1.FirstParameter();
                            }
                            else
                            {
                                par = theKnots2d!.Value(icurv + 1);
                                parf = theKnots2d.Value(icurv);
                            }
                            Point2D p2d = theNewPCurve1!.Value(par);
                            Point2D p2df = theNewPCurve1.Value(parf);
                            Point2D p2dM = theNewPCurve1.Value((parf + par) / 2);
                            P = Surf!.Value(p2d.X(), p2d.Y());
                            P1 = Surf.Value(p2df.X(), p2df.Y());
                            PM = Surf.Value(p2dM.X(), p2dM.Y());
                            P.Transform(Loc.Transformation());
                            P1.Transform(Loc.Transformation());
                            PM.Transform(Loc.Transformation());
                        }

                        if (P.Distance(pntV1) < MinTolerance() && P.Distance(PM) < MinTolerance()
                            && !myFace.IsNull)
                        {
                            if (Small == 0)
                            {
                                SavParf = parf;
                                Savnum = icurv;
                            }
                            // SavParl = par;
                            Small++;
                            if (icurv == nbc)
                            {
                                Vertex VVV = V1;
                                VVV.Orientation(V2.Orientation());
                                Context()!.Replace(V2, VVV);
                            }
                            continue;
                        }
                        if (Small != 0)
                        {
                            if (P.Distance(P1) > MinTolerance() || P.Distance(PM) > MinTolerance())
                            {
                                // FixSmallCurveTool->Perform(prevEdge,theNewCurve3d,theNewPCurve1,revPCurve,SavParf,SavParl);
                                var pmid = new Point3D(0.5 * (pntV1.XYZ() + P1.XYZ()));
                                B.UpdateVertex(V1, pmid, 0);
                            }
                            else
                            {
                                Curve3D? atmpCurve = null;
                                Curve2D? atmpCurve2d1 = null, atmprepcurve = null;
                                if (FixSmallCurveTool.Approx(ref atmpCurve, ref atmpCurve2d1, ref atmprepcurve, ref SavParf, ref par))
                                { // BRepTools
                                    theNewCurve3d = atmpCurve;
                                    theNewPCurve1 = atmpCurve2d1;
                                    revPCurve = atmprepcurve;
                                }
                                else
                                {
                                    var pmid = new Point3D(0.5 * (pntV1.XYZ() + P1.XYZ()));
                                    B.UpdateVertex(V1, pmid, 0);
                                }
                            }
                            Small = 0;
                        }
                        // pdn
                        // (OCCT's commented-out reuse of the end vertices left out)
                        if (icurv != nbc)
                        {
                            B.MakeVertex(ref V, P, TolEdge); // tolerance of the edge
                            pntV1 = P;
                        }
                        else
                        {
                            V = V2;
                        }
                        // else  V2;
                        // }
                    }
                    // else V = V2;

                    Edge newEdge;
                    var sbe = new ShapeBuild_Edge();
                    if (isForward)
                    {
                        V1.Orientation(Forward);
                        V.Orientation(Reversed);
                        newEdge = sbe.InternalCopyReplaceVertices(E, V1, V);
                    }
                    else
                    {
                        V1.Orientation(Reversed);
                        V.Orientation(Forward);
                        newEdge = sbe.InternalCopyReplaceVertices(E, V, V1);
                    }
                    sbe.CopyPCurves(newEdge, E);
                    if (theNewCurve3d is not null)
                    {
                        B.UpdateEdge(newEdge, theNewCurve3d, 0.0);
                    }
                    else if (isDeg)
                    {
                        B.Degenerated(newEdge, true);
                    }
                    // (OCCT's commented-out pcurve updates left out)

                    double f3d = 0.0, l3d = 0.0;
                    if (Savnum == 0)
                    {
                        Savnum = icurv;
                    }
                    bool srNew;
                    if (theNewCurve3d is not null)
                    {
                        if (theNewCurve3d is BoundedCurve3D)
                        {
                            f3d = theNewCurve3d.FirstParameter();
                            l3d = theNewCurve3d.LastParameter();
                            srNew = ((f3d == theKnots3d!.Value(Savnum)) && (l3d == theKnots3d.Value(icurv + 1)));
                        }
                        else
                        {
                            f3d = theKnots3d!.Value(Savnum);
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
                            f2d = theNewPCurve1.FirstParameter();
                            l2d = theNewPCurve1.LastParameter();
                            srNew &= ((f2d == theKnots2d!.Value(Savnum)) && (l2d == theKnots2d.Value(icurv + 1)));
                        }
                        else
                        {
                            f2d = theKnots2d!.Value(Savnum);
                            l2d = theKnots2d.Value(icurv + 1);
                        }
                    }
                    // if(!Savnum) Savnum = icurv;
                    if (theNewCurve3d is not null)
                    {
                        theTransferParamTool.TransferRange(newEdge,
                                                           theKnots3d!.Value(Savnum),
                                                           theKnots3d.Value(icurv + 1),
                                                           false);
                    }
                    else
                    {
                        theTransferParamTool.TransferRange(newEdge,
                                                           theKnots2d!.Value(Savnum),
                                                           theKnots2d.Value(icurv + 1),
                                                           true);
                    }
                    // (OCCT's commented-out CopyRanges left out)
                    Savnum = 0;
                    if (!myFace.IsNull && sae.PCurve(newEdge, myFace, out _, out double setF, out double setL, false))
                    {
                        srNew &= ((setF == f2d) && (setL == l2d));
                    }

                    if (isSeam)
                    {
                        if (newEdge.Orientation() == Forward)
                        {
                            B.UpdateEdge(newEdge, theNewPCurve1, revPCurve, myFace, 0.0);
                        }
                        else
                        {
                            B.UpdateEdge(newEdge, revPCurve, theNewPCurve1, myFace, 0.0);
                        }
                    }
                    else if (!myFace.IsNull)
                    {
                        B.UpdateEdge(newEdge, theNewPCurve1, myFace, 0.0);
                    }

                    if (theNewCurve3d is not null)
                    {
                        sbe.SetRange3d(newEdge, f3d, l3d);
                    }
                    if (theNewPCurve1 is not null)
                    {
                        B.Range(newEdge, myFace, f2d, l2d);
                    }
                    if ((!wasSR || !srNew) && !Tool.Degenerated(newEdge))
                    {
                        B.SameRange(newEdge, false);
                    }

                    // addition NM vertices to new edges
                    double afpar = (myEdgeDivide.HasCurve3d() ? f3d : f2d);
                    double alpar = (myEdgeDivide.HasCurve3d() ? l3d : l2d);
                    for (int n = 1; n <= aSeqParNM.Length(); ++n)
                    {
                        double apar = aSeqParNM.Value(n);
                        Vertex aVold = ShapeCasts.Vertex(aSeqNMVertices.Value(n));
                        Vertex aNMVer = ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aVold, newEdge, E);
                        Context()!.Replace(aVold, aNMVer);
                        if (Math.Abs(apar - afpar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            Context()!.Replace(aNMVer, V1);
                        }
                        else if (Math.Abs(apar - alpar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            Context()!.Replace(aNMVer, V);
                        }
                        else if (apar > afpar && apar < alpar)
                        {
                            B.Add(newEdge, aNMVer);
                        }
                        else
                        {
                            continue;
                        }
                        aSeqNMVertices.Remove(n);
                        aSeqParNM.Remove(n);
                        n--;
                    }

                    B.Add(resWire, newEdge);
                    B.Add(newWire, newEdge);
                    numE++;
                    V1 = V;
                }
                if (numE != 0)
                {
                    resWire.Closed(Tool.IsClosed(resWire));
                    Context()!.Replace(E, resWire);
                }
                else
                {
                    Context()!.Remove(E);
                }
            }
        }
        if (Status(ShapeExtend_DONE))
        {
            newWire.Closed(Tool.IsClosed(newWire));
            Shape tmpW = Context()!.InternalApply(newWire).Oriented(myWire.Orientation());
            myWire = ShapeCasts.Wire(tmpW);
        }
    }

    /// <summary>
    /// Returns the resulting wire (the initial one if not done, null if not loaded), frozen.
    /// </summary>
    public Wire Wire()
    {
        myWire.TShape()?.FreezeGraph();
        return myWire;
    }

    /// <summary><see cref="Wire"/> without freezing it, for ported callers.</summary>
    internal Wire InternalWire() => myWire;

    /// <summary>
    /// Queries the status of the last <see cref="Perform"/>: OK (no edge split), DONE1 (some edges split), FAIL1 (some
    /// edges have no 3D curve, skipped), FAIL2 (some edges have no pcurve, skipped).
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(ShapeUpgrade_SplitCurve3d? splitCurve3dTool)
    {
        mySplitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(ShapeUpgrade_SplitCurve2d? splitCurve2dTool)
    {
        mySplitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    protected virtual ShapeUpgrade_SplitCurve3d? GetSplitCurve3dTool() => mySplitCurve3dTool;

    /// <summary>Returns the tool for splitting pcurves.</summary>
    protected virtual ShapeUpgrade_SplitCurve2d? GetSplitCurve2dTool() => mySplitCurve2dTool;

    /// <summary>Sets the tool for splitting edges.</summary>
    public void SetEdgeDivideTool(ShapeUpgrade_EdgeDivide? edgeDivideTool)
    {
        myEdgeDivide = edgeDivideTool;
    }

    /// <summary>Returns the tool for splitting edges.</summary>
    public virtual ShapeUpgrade_EdgeDivide? GetEdgeDivideTool() => myEdgeDivide;

    /// <summary>Sets the tool for transferring parameters between curves and pcurves.</summary>
    public void SetTransferParamTool(ShapeAnalysis_TransferParameters? TransferParam)
    {
        myTransferParamTool = TransferParam;
    }

    /// <summary>Returns the tool for transferring parameters.</summary>
    public virtual ShapeAnalysis_TransferParameters? GetTransferParamTool() => myTransferParamTool;

    /// <summary>Sets the mode for splitting 3D curves of edges: 0 free edges only, 1 shared edges only, 2 all.</summary>
    public void SetEdgeMode(int EdgeMode)
    {
        myEdgeMode = EdgeMode;
    }

    /// <summary>Sets the tool for fixing small curves.</summary>
    public void SetFixSmallCurveTool(ShapeUpgrade_FixSmallCurves? FixSmallCurvesTool)
    {
        myFixSmallCurveTool = FixSmallCurvesTool;
    }

    /// <summary>Returns the tool for fixing small curves.</summary>
    public ShapeUpgrade_FixSmallCurves? GetFixSmallCurveTool() => myFixSmallCurveTool;
}
