// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepDS_Kind;

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;

namespace Occt;

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
/// (<c>ChFi3d_Builder_2</c>, ...), as <see cref="ChFi3d_Builder_0"/>'s.
/// </para>
/// <para>
/// <b>The working copy</b> (decisions 1005, 1087): the constructor copies a shape holding frozen TShapes
/// (<see cref="CopyAtEntry"/>) and builds its maps from the copy, so that the vertices whose tolerance OCCT
/// raises (<c>ChFi3d_IndexPointInDS</c>, <c>Compute</c>'s <c>UpdateVertex</c>) and the edges the elementary spines
/// give a 3D curve are the copy's. Edges and vertices given by the caller are mapped onto the copy at the entry of
/// each public member, and those handed back are mapped to the caller's.
/// </para>
/// <para>
/// <b>The seam</b> (decision 992): OCCT's <c>myCoup</c>, a <c>TopOpeBRepBuild_HBuilder</c>, is an
/// <see cref="ChFi3d_Reconstruction"/>, unit 7R's <see cref="ChFi3d_BOPReconstruction"/> on the Boolean engine. At the
/// end of <see cref="Compute"/> the working copy's sub-shapes left unchanged are replaced by the caller's in the result
/// (decision 1136); <see cref="Shape"/>, <see cref="BadShape"/> and <see cref="Generated"/> freeze what they hand out.
/// </para>
/// </remarks>
public abstract partial class ChFi3d_Builder
{
    //=================================================================================================
    // Fields (OCCT's protected and private members)

    internal double tolappangle;
    internal double tolesp;
    internal double tol2d;
    internal double tolapp3d;
    internal double tolapp2d;
    internal double fleche;
    internal Continuity myConti;
    internal ChFiDS_Map myEFMap = new();
    internal ChFiDS_Map myESoMap = new();
    internal ChFiDS_Map myEShMap = new();
    internal ChFiDS_Map myVFMap = new();
    internal ChFiDS_Map myVEMap = new();
    internal TopOpeBRepDS_HDataStructure myDS;
    /// <summary>OCCT's <c>TopOpeBRepBuild_HBuilder</c>: the reconstruction (unit 7R).</summary>
    internal ChFi3d_Reconstruction myCoup;
    internal NCollection_List<ChFiDS_Stripe> myListStripe = new();
    internal ChFiDS_StripeMap myVDataMap = new();
    internal NCollection_List<ChFiDS_Regul> myRegul = new();
    internal NCollection_List<ChFiDS_Stripe> badstripes = new();
    internal NCollection_List<Shape> badvertices = new();
    internal NCollection_DataMap<Shape, NCollection_List<int>> myEVIMap = new(ShapeHasher.Instance);
    internal NCollection_DataMap<Shape, Shape> myEdgeFirstFace = new(ShapeHasher.Instance);
    internal bool done;
    internal bool hasresult;

    private Shape myShape;
    private double angular;
    private NCollection_List<Shape> myGenerated = new();
    private Shape myShapeResult = new();
    private Shape badShape = new();

    /// <summary>The copy at the entry (OCCT.NET): <c>myShape</c> is its <see cref="CopyAtEntry.Shape"/>.</summary>
    private CopyAtEntry myCopyAtEntry;

    /// <summary>The working shape: the copy of the shape given (OCCT.NET, for tests and unit 7R).</summary>
    internal Shape WorkingShape() => myShape;

    /// <summary>The caller's sub-shape mapped onto the working copy (OCCT.NET).</summary>
    internal Shape ToWork(in Shape S) => myCopyAtEntry.Copied(S);

    /// <summary>A sub-shape of the working copy mapped back to the caller's (OCCT.NET).</summary>
    internal Shape FromWork(in Shape S) => myCopyAtEntry.Original(S);

    //=================================================================================================
    // The pure virtual members (ChFi3d_FilBuilder and ChFi3d_ChBuilder implement them)

    internal abstract void SimulKPart(ChFiDS_SurfData SD);

    internal abstract bool SimulSurf(ChFiDS_SurfData Data, ChFiDS_ElSpine Guide, ChFiDS_Spine Spine, int Choix,
                                     FaceSurfaceAdaptor S1, TopolTool I1, FaceSurfaceAdaptor S2,
                                     TopolTool I2, double TolGuide, ref double First, ref double Last,
                                     bool Inside, bool Appro, bool Forward, bool RecOnS1, bool RecOnS2,
                                     math_Vector Soldep, ref int Intf, ref int Intl);

    internal abstract void SetRegul();

    internal abstract bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> Data, ChFiDS_ElSpine Guide,
                                       ChFiDS_Spine Spine, int Choix, FaceSurfaceAdaptor S1, TopolTool I1,
                                       FaceSurfaceAdaptor S2, TopolTool I2, double MaxStep, double Fleche,
                                       double TolGuide, ref double First, ref double Last, bool Inside, bool Appro,
                                       bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int Intf,
                                       ref int Intl);

    internal abstract void PerformTwoCorner(int Index);

    internal abstract void PerformThreeCorner(int Index);

    internal abstract void ExtentOneCorner(in Vertex V, ChFiDS_Stripe S);

    internal abstract void ExtentTwoCorner(in Vertex V, NCollection_List<ChFiDS_Stripe> LS);

    internal abstract void ExtentThreeCorner(in Vertex V, NCollection_List<ChFiDS_Stripe> LS);

    internal abstract bool PerformFirstSection(ChFiDS_Spine S, ChFiDS_ElSpine HGuide, int Choix,
                                               FaceSurfaceAdaptor S1, FaceSurfaceAdaptor S2,
                                               TopolTool I1, TopolTool I2, double Par,
                                               math_Vector SolDep, ref Containment Pos1, ref Containment Pos2);

    //=================================================================================================
    // ChFi3d_Builder.cxx

    private static void CompleteDS(TopOpeBRepDS_DataStructure DStr, in Shape S)
    {
        ChFiDS_Map MapEW = new(), MapFS = new();
        MapEW.Fill(S, ShapeType.Edge, ShapeType.Wire);
        MapFS.Fill(S, ShapeType.Face, ShapeType.Shell);

        Explorer ExpE = new();
        for (ExpE.Init(S, ShapeType.Edge); ExpE.More(); ExpE.Next())
        {
            Edge E = ShapeCasts.Edge(ExpE.Current());
            bool hasgeom = DStr.HasGeometry(E);
            if (hasgeom)
            {
                NCollection_List<Shape> WireListAnc = MapEW[E];
                NCollection_List<Shape>.Iterator itaW = new(WireListAnc);
                while (itaW.More())
                {
                    Shape WireAnc = itaW.Value();
                    DStr.AddShape(WireAnc);
                    itaW.Next();
                }
            }
        }

        Explorer ExpF = new();
        for (ExpF.Init(S, ShapeType.Face); ExpF.More(); ExpF.Next())
        {
            Face F = ShapeCasts.Face(ExpF.Current());
            bool hasgeom = DStr.HasGeometry(F);
            if (hasgeom)
            {
                NCollection_List<Shape> ShellListAnc = MapFS[F];
                NCollection_List<Shape>.Iterator itaS = new(ShellListAnc);
                while (itaS.More())
                {
                    Shape ShellAnc = itaS.Value();
                    DStr.AddShape(ShellAnc);
                    itaS.Next();
                }
            }
        }

        // set the range on the DS Curves
        for (int ic = 1; ic <= DStr.NbCurves(); ic++)
        {
            double parmin = RealLast(), parmax = RealFirst();
            NCollection_List<TopOpeBRepDS_Interference> LI = 
                DStr.CurveInterferences(ic);
            for (TopOpeBRepDS_PointIterator it = new(LI); it.More(); it.Next())
            {
                double par = it.Parameter();
                parmin = Min(parmin, par);
                parmax = Max(parmax, par);
            }
            DStr.ChangeCurve(ic).SetRange(parmin, parmax);
        }
    }



    //=================================================================================================

    //=================================================================================================


    //=================================================================================================

    private void ExtentAnalyse()
    {
        int nbedges = 0, nbs = 0;
        for (int iv = 1; iv <= myVDataMap.Extent(); iv++)
        {
            nbs = myVDataMap[iv].Extent();
            Vertex Vtx = myVDataMap.FindKey(iv);
            // nbedges = ChFi3d_NumberOfEdges(Vtx, myVEMap);
            nbedges = ChFi3d_NumberOfSharpEdges(Vtx, myVEMap, myEFMap);
            switch (nbs)
            {
                case 1:
                    ExtentOneCorner(Vtx, myVDataMap.FindFromIndex(iv).First());
                    break;
                case 2:
                    if (nbedges <= 3)
                    {
                        ExtentTwoCorner(Vtx, myVDataMap.FindFromIndex(iv));
                    }
                    break;
                case 3:
                    if (nbedges <= 3)
                    {
                        ExtentThreeCorner(Vtx, myVDataMap.FindFromIndex(iv));
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

        UpdateTolesp();

        if (myListStripe.IsEmpty())
        {
            throw new FailureException("There are no suitable edges for chamfer or fillet");
        }

        Reset();
        myDS = new TopOpeBRepDS_HDataStructure();
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        done = true;
        hasresult = false;

        // filling of myVDatatMap
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();

        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            if ((itel.Value().Spine().FirstStatus() <= ChFiDS_BreakPoint))
            {
                myVDataMap.Add(itel.Value().Spine().FirstVertex(), itel.Value());
            }
            else if (itel.Value().Spine().FirstStatus() == ChFiDS_FreeBoundary)
            {
                ExtentOneCorner(itel.Value().Spine().FirstVertex(), itel.Value());
            }
            if ((itel.Value().Spine().LastStatus() <= ChFiDS_BreakPoint))
            {
                myVDataMap.Add(itel.Value().Spine().LastVertex(), itel.Value());
            }
            else if (itel.Value().Spine().LastStatus() == ChFiDS_FreeBoundary)
            {
                ExtentOneCorner(itel.Value().Spine().LastVertex(), itel.Value());
            }
        }
        // preanalysis to evaluate the extensions.
        ExtentAnalyse();


        // Construction of the stripe of fillet on each stripe.
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            itel.Value().Spine().SetErrorStatus(ChFiDS_Ok);
            try
            {
                PerformSetOfSurf(itel.ChangeValue());
            }
            catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
            {
                badstripes.Append(itel.Value());
                done = true;
                if (itel.Value().Spine().ErrorStatus() == ChFiDS_Ok)
                {
                    itel.Value().Spine().SetErrorStatus(ChFiDS_Error);
                }
            }
            if (!done)
            {
                badstripes.Append(itel.Value());
            }
            done = true;
        }
        done = (badstripes.IsEmpty());


        // construct fillets on each vertex + feed the Ds
        if (done)
        {
            int j = 0;
            for (j = 1; j <= myVDataMap.Extent(); j++)
            {
                try
                {
                    PerformFilletOnVertex(j);
                }
                catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
                {
                    badvertices.Append(myVDataMap.FindKey(j));
                    hasresult = false;
                    done = true;
                }
                if (!done)
                {
                    badvertices.Append(myVDataMap.FindKey(j));
                }
                done = true;
            }
            if (!hasresult)
            {
                done = badvertices.IsEmpty();
            }
        }


        NCollection_IntMap MapIndSo = new(); // NCollection_Map<int>: iterated in OCCT's order
        Explorer expso = new(myShape, ShapeType.Solid);
        for (; expso.More(); expso.Next())
        {
            Shape cursol = expso.Current();
            int indcursol = DStr.AddShape(cursol);
            MapIndSo.Add(indcursol);
        }
        Explorer expsh = new(myShape, ShapeType.Shell, ShapeType.Solid);
        for (; expsh.More(); expsh.Next())
        {
            Shape cursh = expsh.Current();
            int indcursh = DStr.AddShape(cursh);
            MapIndSo.Add(indcursh);
        }
        if (done)
        {
            int i1 = 0;
            for (itel.Initialize(myListStripe), i1 = 0; itel.More(); itel.Next(), i1++)
            {
                ChFiDS_Stripe? st = itel.Value();
                // 05/02/02 akm vvv : (OCC119) First we'll check ain't there
                //                    intersections between fillets
                NCollection_List<ChFiDS_Stripe>.Iterator itel1 = new();
                int i2 = 0;
                for (itel1.Initialize(myListStripe), i2 = 0; itel1.More(); itel1.Next(), i2++)
                {
                    if (i2 <= i1)
                    {
                        // Do not twice intersect the stripes
                        continue;
                    }
                    ChFiDS_Stripe? aCheckStripe = itel1.Value();
                    try
                    {
                        ChFi3d_StripeEdgeInter(st, aCheckStripe, DStr, tol2d);
                    }
                    catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
                    {
                        badstripes.Append(itel.Value());
                        hasresult = false;
                        done = false;
                        break;
                    }
                }
                // 05/02/02 akm ^^^
                int solidindex = st.SolidIndex();
                ChFi3d_FilDS(solidindex, st, DStr, myRegul, tolapp3d, tol2d);
                if (!done)
                {
                    break;
                }
            }


            if (done)
            {
                BrepBuilder B1 = new();
                CompleteDS(DStr, myShape);
                // Update tolerances on vertex to max adjacent edges or
                // Update tolerances on degenerated edge to max of adjacent vertexes.
                TopOpeBRepDS_CurveExplorer cex = new(DStr);
                for (; cex.More(); cex.Next())
                {
                    // OCCT casts the constness of cex.Curve() away: the data structure's curve itself
                    ref TopOpeBRepDS_Curve c = ref DStr.ChangeCurve(cex.Index());
                    double tolc = 0.0;
                    bool degen = c.Curve() is null;
                    if (!degen)
                    {
                        tolc = c.Tolerance();
                    }
                    int ic = cex.Index();
                    TopOpeBRepDS_PointIterator ItP = myDS.CurvePoints(ic); // It in OCCT
                    for (; ItP.More(); ItP.Next())
                    {
                        TopOpeBRepDS_CurvePointInterference? II = null;
                        II = (ItP.Value() as TopOpeBRepDS_CurvePointInterference);
                        if (II is null)
                        {
                            continue;
                        }
                        TopOpeBRepDS_Kind gk = II.GeometryType();
                        int gi = II.Geometry();
                        if (gk == TopOpeBRepDS_VERTEX)
                        {
                            Vertex v = ShapeCasts.Vertex(myDS.Shape(gi));
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
                                B1.UpdateVertex(v, tolc);
                            }
                        }
                        else if (gk == TopOpeBRepDS_POINT)
                        {
                            ref TopOpeBRepDS_Point p = ref DStr.ChangePoint(gi);
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
                myCoup.Perform(myDS);
                NCollection_IntMap.Iterator It = new(MapIndSo);
                for (; It.More(); It.Next())
                {
                    int indsol = It.Key();
                    Shape curshape = DStr.Shape(indsol);
                    myCoup.MergeSolid(curshape, In);
                }

                int i = 1, n = DStr.NbShapes();
                for (; i <= n; i++)
                {
                    Shape S = DStr.Shape(i);
                    if (S.ShapeType() != ShapeType.Edge)
                    {
                        continue;
                    }
                    bool issplitIN = myCoup.IsSplit(S, In);
                    if (!issplitIN)
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator it = new(myCoup.Splits(S, In));
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
                                B1.UpdateVertex(v, tole);
                            }
                        }
                    }
                }
                if (!hasresult)
                {
                    Compound aResult = new();
                    B1.MakeCompound(ref aResult);
                    myShapeResult = aResult;
                    for (It = new NCollection_IntMap.Iterator(MapIndSo); It.More(); It.Next())
                    {
                        int indsol = It.Key();
                        Shape curshape = DStr.Shape(indsol);
                        NCollection_List<Shape>.Iterator its = new(myCoup.Merged(curshape, In));
                        if (!its.More())
                        {
                            B1.Add(myShapeResult, curshape);
                        }
                        else
                        {
                            // If the old type of Shape is Shell, Shell is placed instead of Solid,
                            // However there is a problem for compound of open Shell.
                            while (its.More())
                            {
                                ShapeType letype = curshape.ShapeType();
                                if (letype == ShapeType.Shell)
                                {
                                    Explorer expsh2 = new(its.Value(), ShapeType.Shell);
                                    Shape cursh = expsh2.Current();
                                    B1.Add(myShapeResult, cursh);
                                    its.Next();
                                }
                                else
                                {
                                    B1.Add(myShapeResult, its.Value());
                                    its.Next();
                                }
                            }
                        }
                    }
                }
                else
                {
                    done = false;
                    Compound aBad = new();
                    B1.MakeCompound(ref aBad);
                    badShape = aBad;
                    for (It = new NCollection_IntMap.Iterator(MapIndSo); It.More(); It.Next())
                    {
                        int indsol = It.Key();
                        Shape curshape = DStr.Shape(indsol);
                        NCollection_List<Shape>.Iterator its = new(myCoup.Merged(curshape, In));
                        if (!its.More())
                        {
                            B1.Add(badShape, curshape);
                        }
                        else
                        {
                            while (its.More())
                            {
                                B1.Add(badShape, its.Value());
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
        if (IsDone())
        {
            double SameParTol = global::Kerf.Engine.Foundation.Precision.Confusion();
            int aNbSurfaces = 0, iF = 0;
            NCollection_List<Shape>.Iterator aIt = new();
            //
            aNbSurfaces = myDS.NbSurfaces();

            for (iF = 1; iF <= aNbSurfaces; ++iF)
            {
                NCollection_List<Shape> aLF = myCoup.NewFaces(iF);
                aIt.Initialize(aLF);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aF = aIt.Value();
                    BRepLib.SameParameter(aF, SameParTol, true);
                    ShapeFix.SameParameter(aF, false, SameParTol);
                }
            }
        }

        // OCCT.NET (decisions 1119, 1136): the working copy's sub-shapes the computation left unchanged become the
        // caller's in the result (or the partial result); a second Compute after a Reset keeps the copy's.
        if (!myCopyAtEntry.IsFinished)
        {
            if (done && !myShapeResult.IsNull)
            {
                myShapeResult = myCopyAtEntry.Finish(myShapeResult);
            }
            else if (hasresult && !badShape.IsNull)
            {
                badShape = myCopyAtEntry.Finish(badShape);
            }
        }
    }

    //=======================================================================
    // function : PerformSingularCorner
    // purpose  : Load vertex and degenerated edges.
    //=======================================================================

    private void PerformSingularCorner(int Index)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        ChFiDS_Stripe? stripe = null;
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        Vertex Vtx = myVDataMap.FindKey(Index);

        ChFiDS_SurfData? Fd = null;
        int i = 0, Icurv = 0;
        int Ivtx = 0;
        for (It.Initialize(myVDataMap[Index]), i = 0; It.More(); It.Next(), i++)
        {
            stripe = It.Value();
            // SurfData concerned and its CommonPoints,
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
            bool isfirst = (sens == 1);
            Fd = stripe.SetOfSurfData().Value(num);
            ChFiDS_CommonPoint CV1 = Fd.Vertex(isfirst, 1);
            ChFiDS_CommonPoint CV2 = Fd.Vertex(isfirst, 2);
            // Is it always degenerated ?
            if (CV1.Point().IsEqual(CV2.Point(), 0))
            {
                // if yes the vertex is stored in the stripe
                // and the edge at end is created
                if (i == 0)
                {
                    Ivtx = ChFi3d_IndexPointInDS(CV1, DStr);
                }
                double tolreached = 0.0;
                double Pardeb = 0.0, Parfin = 0.0;
                Point2D VOnS1 = new(), VOnS2 = new();
                Curve3D? C3d = null;
                Curve2D? PCurv = null;
                TopOpeBRepDS_Curve Crv = new();
                if (isfirst)
                {
                    VOnS1 =
                        Fd.InterferenceOnS1().PCurveOnSurf().Value(Fd.InterferenceOnS1().FirstParameter());
                    VOnS2 =
                        Fd.InterferenceOnS2().PCurveOnSurf().Value(Fd.InterferenceOnS2().FirstParameter());
                }
                else
                {
                    VOnS1 =
                        Fd.InterferenceOnS1().PCurveOnSurf().Value(Fd.InterferenceOnS1().LastParameter());
                    VOnS2 =
                        Fd.InterferenceOnS2().PCurveOnSurf().Value(Fd.InterferenceOnS2().LastParameter());
                }

                ChFi3d_ComputeArete(CV1, VOnS1, CV2, VOnS2, DStr.Surface(Fd.Surf()).Surface(), ref C3d, ref PCurv, ref Pardeb,
                                                        ref Parfin, tolapp3d, tolapp2d, out tolreached, 0);
                Crv = new TopOpeBRepDS_Curve(C3d, tolreached);
                Icurv = DStr.AddCurve(Crv);

                stripe.SetCurve(Icurv, isfirst);
                stripe.SetParameters(isfirst, Pardeb, Parfin);
                stripe.ChangePCurve(isfirst) = PCurv;
                stripe.SetIndexPoint(Ivtx, isfirst, 1);
                stripe.SetIndexPoint(Ivtx, isfirst, 2);
            }
        }
    }

    //=================================================================================================

    private void PerformFilletOnVertex(int Index)
    {

        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        ChFiDS_Stripe? stripe = null;
        ChFiDS_Spine? sp = null;
        Vertex Vtx = myVDataMap.FindKey(Index);

        ChFiDS_SurfData? Fd = null;
        int i = 0;
        bool nondegenere = true;
        bool toujoursdegenere = true;
        bool isfirst = false;
        for (It.Initialize(myVDataMap[Index]), i = 0; It.More(); It.Next(), i++)
        {
            stripe = It.Value();
            sp = stripe.Spine();
            // SurfData and its CommonPoints,
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
            isfirst = (sens == 1);
            Fd = stripe.SetOfSurfData().Value(num);
            ChFiDS_CommonPoint CV1 = Fd.Vertex(isfirst, 1);
            ChFiDS_CommonPoint CV2 = Fd.Vertex(isfirst, 2);
            // Is it always degenerated ?
            if (CV1.Point().IsEqual(CV2.Point(), 0))
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
        int nba = ChFi3d_NumberOfSharpEdges(Vtx, myVEMap, myEFMap);

        if (nondegenere)
        { // Normal processing
            switch (i)
            {
                case 1: {
                    if (sp.Status(isfirst) == ChFiDS_FreeBoundary)
                    {
                        return;
                    }
                    if (nba > 3)
                    {
                        PerformIntersectionAtEnd(Index);
                    }
                    else
                    {
                        if (MoreSurfdata(Index))
                        {
                            PerformMoreSurfdata(Index);
                        }
                        else
                        {
                            PerformOneCorner(Index);
                        }
                    }
                }
                break;
                case 2: {
                    if (nba > 3)
                    {
                        PerformMoreThreeCorner(Index, i);
                    }
                    else
                    {
                        PerformTwoCorner(Index);
                    }
                }
                break;
                case 3: {
                    if (nba > 3)
                    {
                        PerformMoreThreeCorner(Index, i);
                    }
                    else
                    {
                        PerformThreeCorner(Index);
                    }
                }
                break;
                default: {
                    PerformMoreThreeCorner(Index, i);
                    break;
                }
            }
        }
        else
        { // Single case processing
            if (toujoursdegenere)
            {
                PerformSingularCorner(Index);
            }
            else
            {
                PerformMoreThreeCorner(Index, i); // Last chance...
            }
        }
    }

    //=================================================================================================

    public void Reset()
    {
        done = false;
        myVDataMap.Clear();
        myRegul.Clear();
        myEVIMap.Clear();
        badstripes.Clear();
        badvertices.Clear();

        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        for (itel.Initialize(myListStripe); itel.More();)
        {
            if (itel.Value().Spine() is not null)
            {
                itel.Value().Reset();
                itel.Next();
            }
            else
            {
                myListStripe.Remove(itel);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The faces generated by the edge or vertex <paramref name="EouV"/> of the shape (frozen, with the result).
    /// </summary>
    public ISchematiqList<Shape> Generated(in Shape EouV)
    {
        if (done)
        {
            myShapeResult.TShape()?.FreezeGraph();
        }
        NCollection_List<Shape> aList = InternalGenerated(EouV);
        if (aList.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        Shape[] aShapes = new Shape[aList.Extent()];
        int i = 0;
        foreach (Shape aS in aList)
        {
            aS.TShape()?.FreezeGraph();
            aShapes[i++] = aS;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>OCCT's <c>Generated</c>: the list itself, unfrozen, for ported callers (OCCT.NET).</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape EouV)
    {
        myGenerated.Clear();
        if (EouV.IsNull)
        {
            return myGenerated;
        }
        if (EouV.ShapeType() != ShapeType.Edge && EouV.ShapeType() != ShapeType.Vertex)
        {
            return myGenerated;
        }
        Shape aWork = ToWork(EouV); // the working copy's shape (OCCT.NET)
        if (myEVIMap.IsBound(aWork))
        {
            NCollection_List<int> L = myEVIMap.Find(aWork);
            NCollection_List<int>.Iterator IL = new();
            for (IL.Initialize(L); IL.More(); IL.Next())
            {
                int I = IL.Value();
                NCollection_List<Shape> LS = myCoup.NewFaces(I);
                NCollection_List<Shape>.Iterator ILS = new();
                for (ILS.Initialize(LS); ILS.More(); ILS.Next())
                {
                    myGenerated.Append(ILS.Value());
                }
            }
        }
        return myGenerated;
    }
}
