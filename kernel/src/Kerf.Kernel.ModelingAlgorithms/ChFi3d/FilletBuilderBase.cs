// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Topology.Containment;

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Fillets;

/// <summary>
/// Root class of the fillet and chamfer builders: computes the fillets of a shape's contours (the spines), their
/// surfaces and the corners where they meet, writes them into the legacy data structure, and hands that to the
/// topological reconstruction.
/// </summary>
/// <remarks>
/// <para>
/// A working object of the fillet algorithms (decision 21), ported over OCCT's files: this file (the header and
/// <c>ChFi3d_Builder.cxx</c>), <c>_1</c> (construction, queries, spines), <c>_2</c> (stripes), <c>_6</c> (walking
/// and approximation), <c>_C1</c>, <c>_C2</c>, <c>_CnCrn</c> (corners), <c>_SpKP</c> (splitting the analytic cases)
/// and <c>_NotImp</c>. Each file's static functions are in a static class named after it
/// (<c>ChFi3d_Builder_2</c>, ...), as <see cref="Builder_0"/>'s.
/// </para>
/// <para>
/// <b>The working overlay</b> (decisions 1087, 1419): the builder works on the caller's shape, as OCCT's. The
/// vertices whose tolerance OCCT raises (<c>ChFi3d_IndexPointInDS</c>, <c>Compute</c>'s <c>UpdateVertex</c>) and the
/// edges the elementary spines give a 3D curve are written in a working overlay that <see cref="Compute"/> and
/// <c>Simulate</c> enter (<see cref="EnterOverlay()"/>), so the caller's frozen TShapes are never written. Edges,
/// vertices and faces given by the caller and handed back are the caller's own.
/// </para>
/// <para>
/// <b>The seam</b> (decision 992): OCCT's <c>myCoup</c>, a <c>TopOpeBRepBuild_HBuilder</c>, is an
/// <see cref="Reconstruction"/>, unit 7R's <see cref="BOPReconstruction"/> on the Boolean engine. At the
/// end of <see cref="Compute"/> the result holds new TShapes for the caller's sub-shapes the computation changed, the
/// caller's own for the rest (decision 1136); <see cref="Shape"/>, <see cref="BadShape"/> and <see cref="Generated"/> freeze what they hand out.
/// </para>
/// </remarks>
internal abstract partial class FilletBuilderBase
{
    //=================================================================================================
    // Fields (OCCT's protected and private members)

    internal double _tolappangle;
    internal double _tolesp;
    internal double _tol2d;
    internal double _tolapp3d;

    // Kerf's own (unit 15U4): how near its edge a guide that is smoothed to C2 is kept (OCCT's 1.000001e-3, whatever the
    // shape's size), and how many guides were smoothed
    internal double _guideTolerance = Builder_0.GuideTolerance;

    internal int _guidesSmoothed;
    internal double _tolapp2d;
    internal double _fleche;
    internal Continuity _conti;
    internal Map _efMap = new();
    internal Map _eSoMap = new();
    internal Map _eShMap = new();
    internal Map _vfMap = new();
    internal Map _veMap = new();
    internal DsHDataStructure _ds;
    /// <summary>OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R).</summary>
    internal Reconstruction _coup;
    internal NCollection_List<Stripe> _listStripe = new();
    internal StripeMap _vDataMap = new();
    internal NCollection_List<Regul> _regul = new();
    internal NCollection_List<Stripe> _badstripes = new();
    internal NCollection_List<Shape> _badvertices = new();
    internal Dictionary<Shape, NCollection_List<int>> _eviMap = new(ShapeHasher.Instance);
    internal Dictionary<Shape, Shape> _edgeFirstFace = new(ShapeHasher.Instance);
    internal bool _done;
    internal bool _hasresult;

    private Shape _shape;
    private double _angular;
    private NCollection_List<Shape> _generated = new();
    private Shape _shapeResult = new();
    private Shape _badShape = new();

    /// <summary>
    /// The builder's working overlay (Kerf, unit 10H5): detached, entered by each call that may write into the shape
    /// (<see cref="Compute"/>, <c>Simulate</c>), finished by the first <see cref="Compute"/> that gives a result. A
    /// later call starts a new one, on the caller's shape as it was given.
    /// </summary>
    private WorkingOverlay? _overlay;

    /// <summary>The shape the builder works on: the caller's (Kerf, for tests and unit 7R).</summary>
    internal Shape WorkingShape() => _shape;

    /// <summary>
    /// Enters the builder's working overlay for a call that may write into the shape (Kerf, unit 10H5): OCCT writes
    /// the vertex tolerances where the fillets end, and 3D curves into the spine's edges that have none. Returns the
    /// overlay, or null when an operation's overlay that takes writes is open here: the builder then writes into that
    /// one, as OCCT's into the shape of the operation that runs it.
    /// </summary>
    internal WorkingOverlay? EnterOverlay(out WorkingOverlay.Scope scope)
    {
        if (WorkingOverlay.Writing is not null)
        {
            scope = default;
            return null;
        }
        if (_overlay is null || _overlay.IsFinished)
        {
            _overlay = WorkingOverlay.Detached(_shape);
        }
        scope = _overlay.Enter();
        return _overlay;
    }

    /// <summary>Enters the builder's working overlay (see <see cref="EnterOverlay(out WorkingOverlay.Scope)"/>).</summary>
    internal WorkingOverlay.Scope EnterOverlay()
    {
        EnterOverlay(out WorkingOverlay.Scope scope);
        return scope;
    }

    //=================================================================================================
    // The pure virtual members (ChFi3d_FilBuilder and ChFi3d_ChBuilder implement them)

    internal abstract void SimulKPart(SurfData sd);

    internal abstract bool SimulSurf(SurfData data, ElSpine guide, Spine spine, int choix,
                                     FaceSurfaceAdaptor s1, TopolTool i1, FaceSurfaceAdaptor s2,
                                     TopolTool i2, double tolGuide, ref double first, ref double last,
                                     bool inside, bool appro, bool forward, bool recOnS1, bool recOnS2,
                                     math_Vector soldep, ref int intf, ref int intl);

    internal abstract void SetRegul();

    internal abstract bool PerformSurf(NCollection_Sequence<SurfData> data, ElSpine guide,
                                       Spine spine, int choix, FaceSurfaceAdaptor s1, TopolTool i1,
                                       FaceSurfaceAdaptor s2, TopolTool i2, double maxStep, double fleche,
                                       double tolGuide, ref double first, ref double last, bool inside, bool appro,
                                       bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf,
                                       ref int intl);

    /// <summary>
    /// Kerf (unit 15AK3): called once a stripe's surfaces are all walked and before its ends are made, for what a
    /// builder has to add between them.
    /// </summary>
    internal virtual void CompleteSetOfSurf(Stripe stripe)
    {
    }

    internal abstract void PerformTwoCorner(int index);

    internal abstract void PerformThreeCorner(int index);

    internal abstract void ExtentOneCorner(in Vertex v, Stripe s);

    internal abstract void ExtentTwoCorner(in Vertex v, NCollection_List<Stripe> ls);

    internal abstract void ExtentThreeCorner(in Vertex v, NCollection_List<Stripe> ls);

    internal abstract bool PerformFirstSection(Spine s, ElSpine hGuide, int choix,
                                               FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2,
                                               TopolTool i1, TopolTool i2, double par,
                                               math_Vector solDep, ref Containment pos1, ref Containment pos2);

    //=================================================================================================
    // ChFi3d_Builder.cxx

    private static void CompleteDS(DsDataStructure dStr, in Shape s)
    {
        Map mapEW = new(), mapFS = new();
        mapEW.Fill(s, ShapeType.Edge, ShapeType.Wire);
        mapFS.Fill(s, ShapeType.Face, ShapeType.Shell);

        Explorer expE = new();
        for (expE.Init(s, ShapeType.Edge); expE.More(); expE.Next())
        {
            Edge e = ShapeCasts.Edge(expE.Current());
            bool hasgeom = dStr.HasGeometry(e);
            if (hasgeom)
            {
                NCollection_List<Shape> wireListAnc = mapEW[e];
                NCollection_List<Shape>.Iterator itaW = new(wireListAnc);
                while (itaW.More())
                {
                    Shape wireAnc = itaW.Value();
                    dStr.AddShape(wireAnc);
                    itaW.Next();
                }
            }
        }

        Explorer expF = new();
        for (expF.Init(s, ShapeType.Face); expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            bool hasgeom = dStr.HasGeometry(f);
            if (hasgeom)
            {
                NCollection_List<Shape> shellListAnc = mapFS[f];
                NCollection_List<Shape>.Iterator itaS = new(shellListAnc);
                while (itaS.More())
                {
                    Shape shellAnc = itaS.Value();
                    dStr.AddShape(shellAnc);
                    itaS.Next();
                }
            }
        }

        // set the range on the DS Curves
        for (int ic = 1; ic <= dStr.NbCurves(); ic++)
        {
            double parmin = RealLast(), parmax = RealFirst();
            NCollection_List<DsInterference> li = 
                dStr.CurveInterferences(ic);
            for (DsPointIterator it = new(li); it.More(); it.Next())
            {
                double par = it.Parameter();
                parmin = Min(parmin, par);
                parmax = Max(parmax, par);
            }
            dStr.ChangeCurve(ic).SetRange(parmin, parmax);
        }
    }



    //=================================================================================================

    //=================================================================================================


    //=================================================================================================

    private void ExtentAnalyse()
    {
        int nbedges = 0, nbs = 0;
        for (int iv = 1; iv <= _vDataMap.Extent(); iv++)
        {
            nbs = _vDataMap[iv].Extent();
            Vertex vtx = _vDataMap.FindKey(iv);
            // nbedges = ChFi3d_NumberOfEdges(Vtx, myVEMap);
            nbedges = ChFi3d_NumberOfSharpEdges(vtx, _veMap, _efMap);
            switch (nbs)
            {
                case 1:
                    ExtentOneCorner(vtx, _vDataMap.FindFromIndex(iv).First());
                    break;
                case 2:
                    if (nbedges <= 3)
                    {
                        ExtentTwoCorner(vtx, _vDataMap.FindFromIndex(iv));
                    }
                    break;
                case 3:
                    if (nbedges <= 3)
                    {
                        ExtentThreeCorner(vtx, _vDataMap.FindFromIndex(iv));
                    }
                    break;
                default:
                    break;
            }
        }
    }

    //=================================================================================================

    public void Compute()
    {
        WorkingOverlay? overlay = EnterOverlay(out WorkingOverlay.Scope scope);
        using (scope)
        {
            ComputeCore();

            // Kerf (decisions 1119, 1136; unit 10H5): the result (or the partial result) holds new TShapes for the
            // caller's sub-shapes the computation changed, the caller's own for the rest
            if (overlay is not null)
            {
                if (_done && !_shapeResult.IsNull)
                {
                    _shapeResult = overlay.Finish(_shapeResult);
                }
                else if (_hasresult && !_badShape.IsNull)
                {
                    _badShape = overlay.Finish(_badShape);
                }
            }
        }
    }

    private void ComputeCore()
    {

        UpdateTolesp();

        if (_listStripe.IsEmpty())
        {
            throw new FailureException("There are no suitable edges for chamfer or fillet");
        }

        Reset();
        _ds = new DsHDataStructure();
        DsDataStructure dStr = _ds.ChangeDS();
        _done = true;
        _hasresult = false;

        // filling of myVDatatMap
        NCollection_List<Stripe>.Iterator itel = new();

        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            if ((itel.Value().Spine().FirstStatus() <= State.BreakPoint))
            {
                _vDataMap.Add(itel.Value().Spine().FirstVertex(), itel.Value());
            }
            else if (itel.Value().Spine().FirstStatus() == State.FreeBoundary)
            {
                ExtentOneCorner(itel.Value().Spine().FirstVertex(), itel.Value());
            }
            if ((itel.Value().Spine().LastStatus() <= State.BreakPoint))
            {
                _vDataMap.Add(itel.Value().Spine().LastVertex(), itel.Value());
            }
            else if (itel.Value().Spine().LastStatus() == State.FreeBoundary)
            {
                ExtentOneCorner(itel.Value().Spine().LastVertex(), itel.Value());
            }
        }
        // preanalysis to evaluate the extensions.
        ExtentAnalyse();


        // Construction of the stripe of fillet on each stripe.
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            Cancellation.ThrowIfRequested(); // Kerf: OCCT's fillets ignore their range (decision 1855)
            itel.Value().Spine().SetErrorStatus(ErrorStatus.Ok);
            try
            {
                PerformSetOfSurf(itel.ChangeValue());
            }
            catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
            {
                _badstripes.Append(itel.Value());
                _done = true;
                if (itel.Value().Spine().ErrorStatus() == ErrorStatus.Ok)
                {
                    itel.Value().Spine().SetErrorStatus(ErrorStatus.Error);
                }
            }
            if (!_done)
            {
                _badstripes.Append(itel.Value());
            }
            _done = true;
        }
        _done = (_badstripes.IsEmpty());


        // construct fillets on each vertex + feed the Ds
        if (_done)
        {
            int j = 0;
            for (j = 1; j <= _vDataMap.Extent(); j++)
            {
                Cancellation.ThrowIfRequested();
                try
                {
                    PerformFilletOnVertex(j);
                }
                catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
                {
                    _badvertices.Append(_vDataMap.FindKey(j));
                    _hasresult = false;
                    _done = true;
                }
                if (!_done)
                {
                    _badvertices.Append(_vDataMap.FindKey(j));
                }
                _done = true;
            }
            if (!_hasresult)
            {
                _done = _badvertices.IsEmpty();
            }
        }


        NCollection_IntMap mapIndSo = new(); // NCollection_Map<int>: iterated in OCCT's order
        Explorer expso = new(_shape, ShapeType.Solid);
        for (; expso.More(); expso.Next())
        {
            Shape cursol = expso.Current();
            int indcursol = dStr.AddShape(cursol);
            mapIndSo.Add(indcursol);
        }
        Explorer expsh = new(_shape, ShapeType.Shell, ShapeType.Solid);
        for (; expsh.More(); expsh.Next())
        {
            Shape cursh = expsh.Current();
            int indcursh = dStr.AddShape(cursh);
            mapIndSo.Add(indcursh);
        }
        if (_done)
        {
            int i1 = 0;
            for (itel.Initialize(_listStripe), i1 = 0; itel.More(); itel.Next(), i1++)
            {
                Cancellation.ThrowIfRequested();
                Stripe? st = itel.Value();
                // 05/02/02 akm vvv : (OCC119) First we'll check ain't there
                //                    intersections between fillets
                NCollection_List<Stripe>.Iterator itel1 = new();
                int i2 = 0;
                for (itel1.Initialize(_listStripe), i2 = 0; itel1.More(); itel1.Next(), i2++)
                {
                    if (i2 <= i1)
                    {
                        // Do not twice intersect the stripes
                        continue;
                    }
                    Stripe? checkStripe = itel1.Value();
                    try
                    {
                        ChFi3d_StripeEdgeInter(st, checkStripe, dStr, _tol2d);
                    }
                    catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
                    {
                        _badstripes.Append(itel.Value());
                        _hasresult = false;
                        _done = false;
                        break;
                    }
                }
                // 05/02/02 akm ^^^
                int solidindex = st.SolidIndex();
                ChFi3d_FilDS(solidindex, st, dStr, _regul, _tolapp3d, _tol2d);
                if (!_done)
                {
                    break;
                }
            }


            if (_done)
            {
                BrepBuilder b1 = new();
                CompleteDS(dStr, _shape);
                // Update tolerances on vertex to max adjacent edges or
                // Update tolerances on degenerated edge to max of adjacent vertexes.
                DsCurveExplorer cex = new(dStr);
                for (; cex.More(); cex.Next())
                {
                    // OCCT casts the constness of cex.Curve() away: the data structure's curve itself
                    ref DsCurve c = ref dStr.ChangeCurve(cex.Index());
                    double tolc = 0.0;
                    bool degen = c.Curve() is null;
                    if (!degen)
                    {
                        tolc = c.Tolerance();
                    }
                    int ic = cex.Index();
                    DsPointIterator itP = _ds.CurvePoints(ic); // It in OCCT
                    for (; itP.More(); itP.Next())
                    {
                        DsCurvePointInterference? ii = null;
                        ii = (itP.Value() as DsCurvePointInterference);
                        if (ii is null)
                        {
                            continue;
                        }
                        DsKind gk = ii.GeometryType();
                        int gi = ii.Geometry();
                        if (gk == DsKind.Vertex)
                        {
                            Vertex v = ShapeCasts.Vertex(_ds.Shape(gi));
                            double tolv = Tool.Tolerance(v);
                            if (tolv > 0.0001)
                            {
                                tolv += 0.0003;
                                if (tolc < tolv)
                                {
                                    tolc = tolv + 0.00001;
                                }
                            }
                            if (degen && tolc < tolv)
                            {
                                tolc = tolv;
                            }
                            else if (tolc > tolv)
                            {
                                b1.UpdateVertex(v, tolc);
                            }
                        }
                        else if (gk == DsKind.Point)
                        {
                            ref DsPoint p = ref dStr.ChangePoint(gi);
                            double tolp = p.Tolerance();
                            if (degen && tolc < tolp)
                            {
                                tolc = tolp;
                            }
                            else if (tolc > tolp)
                            {
                                p.Tolerance(tolc);
                            }
                        }
                    }
                    if (degen)
                    {
                        c.Tolerance(tolc);
                    }
                }
                // The seam (decision 992): unit 7R's reconstruction; until then Perform throws
                // NotImplementedException("7R") here, with the data structure complete.
                _coup.Perform(_ds);
                NCollection_IntMap.Iterator It = new(mapIndSo);
                for (; It.More(); It.Next())
                {
                    int indsol = It.Key();
                    Shape curshape = dStr.Shape(indsol);
                    _coup.MergeSolid(curshape, In);
                }

                int i = 1, n = dStr.NbShapes();
                for (; i <= n; i++)
                {
                    Shape s = dStr.Shape(i);
                    if (s.Type != ShapeType.Edge)
                    {
                        continue;
                    }
                    bool issplitIN = _coup.IsSplit(s, In);
                    if (!issplitIN)
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator it = new(_coup.Splits(s, In));
                    for (; it.More(); it.Next())
                    {
                        Edge newE = ShapeCasts.Edge(it.Value());
                        double tole = Tool.Tolerance(newE);
                        Explorer exv = new(newE, ShapeType.Vertex);
                        for (; exv.More(); exv.Next())
                        {
                            Vertex v = ShapeCasts.Vertex(exv.Current());
                            double tolv = Tool.Tolerance(v);
                            if (tole > tolv)
                            {
                                b1.UpdateVertex(v, tole);
                            }
                        }
                    }
                }
                if (!_hasresult && !_coup.LostFaces)
                {
                    Compound result = new();
                    b1.MakeCompound(ref result);
                    _shapeResult = result;
                    for (It = new NCollection_IntMap.Iterator(mapIndSo); It.More(); It.Next())
                    {
                        int indsol = It.Key();
                        Shape curshape = dStr.Shape(indsol);
                        NCollection_List<Shape>.Iterator its = new(_coup.Merged(curshape, In));
                        if (!its.More())
                        {
                            b1.Add(_shapeResult, curshape);
                        }
                        else
                        {
                            // If the old type of Shape is Shell, Shell is placed instead of Solid,
                            // However there is a problem for compound of open Shell.
                            while (its.More())
                            {
                                ShapeType letype = curshape.Type;
                                if (letype == ShapeType.Shell)
                                {
                                    Explorer expsh2 = new(its.Value(), ShapeType.Shell);
                                    Shape cursh = expsh2.Current();
                                    b1.Add(_shapeResult, cursh);
                                    its.Next();
                                }
                                else
                                {
                                    b1.Add(_shapeResult, its.Value());
                                    its.Next();
                                }
                            }
                        }
                    }
                }
                else
                {
                    _done = false;
                    Compound bad = new();
                    b1.MakeCompound(ref bad);
                    _badShape = bad;
                    for (It = new NCollection_IntMap.Iterator(mapIndSo); It.More(); It.Next())
                    {
                        int indsol = It.Key();
                        Shape curshape = dStr.Shape(indsol);
                        NCollection_List<Shape>.Iterator its = new(_coup.Merged(curshape, In));
                        if (!its.More())
                        {
                            b1.Add(_badShape, curshape);
                        }
                        else
                        {
                            while (its.More())
                            {
                                b1.Add(_badShape, its.Value());
                                its.Next();
                            }
                        }
                    }
                }

                // Regularities are coded after cutting.
                SetRegul();

            }
        }

        // display of time for perfs

        //
        // Inspect the new faces to provide sameparameter
        // if it is necessary
        if (IsDone)
        {
            double sameParTol = global::Kerf.Engine.Foundation.Precision.Confusion();
            int nbSurfaces = 0, iF = 0;
            NCollection_List<Shape>.Iterator aIt = new();
            //
            nbSurfaces = _ds.SurfaceCount;

            for (iF = 1; iF <= nbSurfaces; ++iF)
            {
                NCollection_List<Shape> lf = _coup.NewFaces(iF);
                aIt.Initialize(lf);
                for (; aIt.More(); aIt.Next())
                {
                    Shape f = aIt.Value();
                    BrepLibrary.SameParameter(f, sameParTol, true);
                    ShapeFixFixer.SameParameter(f, false, sameParTol);
                }
            }
        }
    }

    //=======================================================================
    // function : PerformSingularCorner
    // purpose  : Load vertex and degenerated edges.
    //=======================================================================

    private void PerformSingularCorner(int index)
    {
        NCollection_List<Stripe>.Iterator it = new();
        Stripe? stripe = null;
        DsDataStructure dStr = _ds.ChangeDS();
        Vertex vtx = _vDataMap.FindKey(index);

        SurfData? fd = null;
        int i = 0, icurv = 0;
        int ivtx = 0;
        for (it.Initialize(_vDataMap[index]), i = 0; it.More(); it.Next(), i++)
        {
            stripe = it.Value();
            // SurfData concerned and its CommonPoints,
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
            bool isfirst = (sens == 1);
            fd = stripe.SetOfSurfData().Value(num);
            CommonPoint cv1 = fd.Vertex(isfirst, 1);
            CommonPoint cv2 = fd.Vertex(isfirst, 2);
            // Is it always degenerated ?
            if (cv1.Point().IsEqual(cv2.Point(), 0))
            {
                // if yes the vertex is stored in the stripe
                // and the edge at end is created
                if (i == 0)
                {
                    ivtx = ChFi3d_IndexPointInDS(cv1, dStr);
                }
                double tolreached = 0.0;
                double pardeb = 0.0, parfin = 0.0;
                Point2D vOnS1 = new(), vOnS2 = new();
                Curve3D? c3d = null;
                Curve2D? pCurv = null;
                DsCurve crv = new();
                if (isfirst)
                {
                    vOnS1 =
                        fd.InterferenceOnS1().PCurveOnSurf().Value(fd.InterferenceOnS1().FirstParameter());
                    vOnS2 =
                        fd.InterferenceOnS2().PCurveOnSurf().Value(fd.InterferenceOnS2().FirstParameter());
                }
                else
                {
                    vOnS1 =
                        fd.InterferenceOnS1().PCurveOnSurf().Value(fd.InterferenceOnS1().LastParameter());
                    vOnS2 =
                        fd.InterferenceOnS2().PCurveOnSurf().Value(fd.InterferenceOnS2().LastParameter());
                }

                ChFi3d_ComputeArete(cv1, vOnS1, cv2, vOnS2, dStr.Surface(fd.Surf()).Surface(), ref c3d, ref pCurv, ref pardeb,
                                                        ref parfin, _tolapp3d, _tolapp2d, out tolreached, 0);
                crv = new DsCurve(c3d, tolreached);
                icurv = dStr.AddCurve(crv);

                stripe.SetCurve(icurv, isfirst);
                stripe.SetParameters(isfirst, pardeb, parfin);
                stripe.ChangePCurve(isfirst) = pCurv;
                stripe.SetIndexPoint(ivtx, isfirst, 1);
                stripe.SetIndexPoint(ivtx, isfirst, 2);
            }
        }
    }

    //=================================================================================================

    private void PerformFilletOnVertex(int index)
    {

        NCollection_List<Stripe>.Iterator it = new();
        Stripe? stripe = null;
        Spine? sp = null;
        Vertex vtx = _vDataMap.FindKey(index);

        SurfData? fd = null;
        int i = 0;
        bool nondegenere = true;
        bool toujoursdegenere = true;
        bool isfirst = false;
        for (it.Initialize(_vDataMap[index]), i = 0; it.More(); it.Next(), i++)
        {
            stripe = it.Value();
            sp = stripe.Spine();
            // SurfData and its CommonPoints,
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
            isfirst = (sens == 1);
            fd = stripe.SetOfSurfData().Value(num);
            CommonPoint cv1 = fd.Vertex(isfirst, 1);
            CommonPoint cv2 = fd.Vertex(isfirst, 2);
            // Is it always degenerated ?
            if (cv1.Point().IsEqual(cv2.Point(), 0))
            {
                nondegenere = false;
            }
            else
            {
                toujoursdegenere = false;
            }
        }

        // calcul du nombre de faces = nombre d'aretes
        /* NCollection_List<TopoDS_Shape>.Iterator ItF,JtF,ItE;
            int nbf = 0, jf = 0;
            for (ItF.Initialize(myVFMap[Vtx]); ItF.More(); ItF.Next()){
                jf++;
                int kf = 1;
                TopoDS_Shape cur = ItF.Value();
                for (JtF.Initialize(myVFMap[Vtx]); JtF.More() && (kf < jf); JtF.Next(), kf++){
                    if(cur.IsSame(JtF.Value())) break;
                }
                if(kf == jf) nbf++;
            }
            int nba = myVEMap[Vtx].Extent();
            for (ItE.Initialize(myVEMap[Vtx]); ItE.More(); ItE.Next()){
                TopoDS_Edge cur = TopoDS.Edge(ItE.Value());
                if (BRep_Tool.Degenerated(cur)) nba--;
            }
            nba=nba/2;*/
        int nba = ChFi3d_NumberOfSharpEdges(vtx, _veMap, _efMap);

        if (nondegenere)
        { // Normal processing
            switch (i)
            {
                case 1: {
                    if (sp.Status(isfirst) == State.FreeBoundary)
                    {
                        return;
                    }
                    if (nba > 3)
                    {
                        PerformIntersectionAtEnd(index);
                    }
                    else
                    {
                        if (MoreSurfdata(index))
                        {
                            PerformMoreSurfdata(index);
                        }
                        else
                        {
                            PerformOneCorner(index);
                        }
                    }
                }
                break;
                case 2: {
                    if (nba > 3)
                    {
                        PerformMoreThreeCorner(index, i);
                    }
                    else
                    {
                        PerformTwoCorner(index);
                    }
                }
                break;
                case 3: {
                    if (nba > 3)
                    {
                        PerformMoreThreeCorner(index, i);
                    }
                    else
                    {
                        PerformThreeCorner(index);
                    }
                }
                break;
                default: {
                    PerformMoreThreeCorner(index, i);
                    break;
                }
            }
        }
        else
        { // Single case processing
            if (toujoursdegenere)
            {
                PerformSingularCorner(index);
            }
            else
            {
                PerformMoreThreeCorner(index, i); // Last chance...
            }
        }
    }

    //=================================================================================================

    public void Reset()
    {
        _done = false;
        _vDataMap.Clear();
        _regul.Clear();
        _eviMap.Clear();
        _badstripes.Clear();
        _badvertices.Clear();

        NCollection_List<Stripe>.Iterator itel = new();
        for (itel.Initialize(_listStripe); itel.More();)
        {
            if (itel.Value().Spine() is not null)
            {
                itel.Value().Reset();
                itel.Next();
            }
            else
            {
                _listStripe.Remove(itel);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The faces generated by the edge or vertex <paramref name="eouV"/> of the shape (frozen, with the result).
    /// </summary>
    public IReadOnlyList<Shape> Generated(in Shape eouV)
    {
        if (_done)
        {
            _shapeResult.TShape()?.FreezeGraph();
        }
        NCollection_List<Shape> list = InternalGenerated(eouV);
        if (list.IsEmpty())
        {
            return Array.Empty<Shape>();
        }
        Shape[] shapes = new Shape[list.Extent()];
        int i = 0;
        foreach (Shape s in list)
        {
            s.TShape()?.FreezeGraph();
            shapes[i++] = s;
        }
        return ImmutableArrays.Own(shapes);
    }

    /// <summary>OCCT's <c>Generated</c>: the list itself, unfrozen, for ported callers (Kerf).</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape eouV)
    {
        _generated.Clear();
        if (eouV.IsNull)
        {
            return _generated;
        }
        if (eouV.Type != ShapeType.Edge && eouV.Type != ShapeType.Vertex)
        {
            return _generated;
        }
        Shape work = eouV;
        if (_eviMap.ContainsKey(work))
        {
            NCollection_List<int> l = _eviMap.GetValue(work);
            NCollection_List<int>.Iterator il = new();
            for (il.Initialize(l); il.More(); il.Next())
            {
                int i = il.Value();
                NCollection_List<Shape> ls = _coup.NewFaces(i);
                NCollection_List<Shape>.Iterator ils = new();
                for (ils.Initialize(ls); ils.More(); ils.Next())
                {
                    _generated.Append(ils.Value());
                }
            }
        }
        return _generated;
    }
}
