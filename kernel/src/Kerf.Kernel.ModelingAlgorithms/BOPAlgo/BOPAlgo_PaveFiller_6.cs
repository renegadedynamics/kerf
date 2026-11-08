// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.InteropServices;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The intersection of a pair of faces run by the pave filler's parallel loop (<c>BOPAlgo_PaveFiller_6.cxx</c>'s
/// file-local class).
/// </summary>
/// <remarks>
/// OCCT derives it from IntTools_FaceFace and BOPAlgo_ParallelAlgo; C# has one base, so the parallel algorithm's part
/// (the progress range, the report, the parallel flag) is a <see cref="Options"/> member with OCCT's
/// members forwarding to it. OCCT runs the vector without a context (<c>BOPTools_Parallel::Perform(myRunParallel,
/// aVFaceFace)</c>), so each face/face intersection makes its own context, as IntTools_FaceFace does natively: the
/// class is a plain solver, not a context solver.
/// </remarks>
internal sealed class FaceFaceJob : FaceFace, BooleanParallel.ISolver
{
    private readonly Options _options = new();
    private ProgressRange _progressRange;

    private int _if1;
    private int _if2;
    private double _tolFF;
    private Face _f1 = new();
    private Face _f2 = new();
    private Box _box1 = new();
    private Box _box2 = new();
    private Transform3D _trsf = new();

    public FaceFaceJob()
    {
        _if1 = -1;
        _if2 = -1;
        _tolFF = 1.0e-7;
    }

    //
    public void SetIndices(int nF1, int nF2)
    {
        _if1 = nF1;
        _if2 = nF2;
    }

    //
    public void Indices(out int nF1, out int nF2)
    {
        nF1 = _if1;
        nF2 = _if2;
    }

    //
    public void SetFaces(in Face aF1, in Face aF2)
    {
        _f1 = aF1;
        _f2 = aF2;
    }

    //
    public void SetBoxes(in Box box1, in Box box2)
    {
        _box1 = box1;
        _box2 = box2;
    }

    //
    public new Face Face1() => _f1;

    //
    public new Face Face2() => _f2;

    //
    public void SetTolFF(double aTolFF) => _tolFF = aTolFF;

    //
    public double TolFF() => _tolFF;

    //
    public Transform3D Trsf() => _trsf;

    // BOPAlgo_ParallelAlgo's and BOPAlgo_Options' members used by the filler.
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    public void SetRunParallel(bool flag) => _options.SetRunParallel(flag);

    public bool HasErrors => _options.HasErrors;

    //
    public void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (_options.UserBreak(ps))
        {
            return;
        }
        try
        {
            Transform3D trsf = new();
            Face f1 = _f1, f2 = _f2;
            if (BuilderTools.TrsfToPoint(_box1, _box2, ref trsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                Placement loc = new(trsf);
                f1.Move(loc);
                f2.Move(loc);

                // The starting point is initialized only with the UV parameters
                // on the faces - 3D point is not set (see GetEFPnts method),
                // so no need to transform anything.

                _trsf = trsf.Inverted();
            }

            Perform(f1, f2, _options.RunParallel());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            _options.AddError(new AlertIntersectionFailed());
        }
    }

    //
    public void ApplyTrsf()
    {
        if (IsDone)
        {
            // Update curves
            for (int i = 1; i <= _seqOfCurve.Length(); ++i)
            {
                ref IntersectionCurve ic = ref _seqOfCurve.ChangeValue(i);
                ic.Curve()!.Transform(_trsf);
            }
            // Update points
            for (int i = 1; i <= _pnts.Length(); ++i)
            {
                ref PntOn2Faces p2F = ref _pnts.ChangeValue(i);
                PntOnFace pOnF1 = p2F.P1(), pOnF2 = p2F.P2();
                pOnF1.SetPnt(pOnF1.Pnt().Transformed(_trsf));
                pOnF2.SetPnt(pOnF2.Pnt().Transformed(_trsf));
                p2F.SetP1(pOnF1);
                p2F.SetP2(pOnF2);
            }
        }
    }
}

internal partial class PaveFiller
{
    private static bool IsPlaneFF(Surface surface)
    {
        if (surface is PlaneSurface)
        {
            return true;
        }
        else if (surface is global::Kerf.Geometry.OffsetSurface offsetSurface)
        {
            return offsetSurface.BasisSurface is PlaneSurface;
        }
        else if (surface is TrimmedSurface trimmedSurface)
        {
            return trimmedSurface.BasisSurface is PlaneSurface;
        }

        return false;
    }

    //=================================================================================================

    private static bool IsClosedFF(in Edge edge,
                                   Surface? surface,
                                   Triangulation? triangulation,
                                   in Placement location,
                                   bool isPlane)
    {
        if (!isPlane)
        {
            // Check surface
            Placement aLocation = location.Predivided(edge.Placement);

            // find the representation
            BrepTEdge tEdge = (BrepTEdge)edge.TShape()!;
            for (NCollection_List<CurveRepresentation>.Iterator iter = new(tEdge.Curves());
                 iter.More();
                 iter.Next())
            {
                CurveRepresentation curveRepresentation = iter.Value();
                if (curveRepresentation.IsCurveOnSurface(surface, aLocation)
                    && curveRepresentation.IsCurveOnClosedSurface)
                {
                    return true;
                }
            }
        }

        // Check triangulation
        return Tool.IsClosed(edge, triangulation, location);
    }

    //=================================================================================================

    /// <summary>The face/face intersections: the section curves and points of each pair of faces.</summary>
    protected virtual void PerformFF(ProgressRange range)
    {
        // Update face info for all Face/Face intersection pairs
        // and also for the rest of the faces with FaceInfo already initialized,
        // i.e. anyhow touched faces.
        _iterator!.Initialize(ShapeType.Face, ShapeType.Face);
        int iSize = _iterator.ExpectedLength();

        // Collect faces from intersection pairs
        NCollection_IntMap miFence = new();
        int nF1, nF2;
        for (; _iterator.More(); _iterator.Next())
        {
            _iterator.Value(out nF1, out nF2);
            miFence.Add(nF1);
            miFence.Add(nF2);
        }
        // Collect the rest of the touched faces
        for (int i = 0; i < _ds!.SourceShapeCount; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() == ShapeType.Face && si.HasReference)
            {
                miFence.Add(i);
            }
        }
        // Update face info
        _ds.UpdateFaceInfoOn(miFence);
        _ds.UpdateFaceInfoIn(miFence);

        if (iSize == 0)
        {
            // no intersection pairs found
            return;
        }

        using var psOuter = new ProgressScope(range, null, 1);

        List<InterfFF> fFs = _ds.InterfFF();
        //
        // Options for the intersection algorithm
        bool bApprox = _sectionAttribute.Approximation(), bCompC2D1 = _sectionAttribute.PCurveOnS1(),
             bCompC2D2 = _sectionAttribute.PCurveOnS2();
        double approxTol = 1.0e-7;
        // Post-processing options
        bool bSplitCurve = false;
        //
        // Collect all pairs of Edge/Edge interferences to check if
        // some faces have to be moved to obtain more precise intersection
        Dictionary<Pair, NCollection_List<int>> eeMap = new();
        List<InterfEE> veEs = _ds.InterfEE();
        for (int iEE = 0; iEE < veEs.Count; ++iEE)
        {
            Interf ee = veEs[iEE];
            if (!ee.HasIndexNew())
            {
                continue;
            }
            ee.Indices(out int nE1, out int nE2);

            int nVN = ee.IndexNew();

            Pair pair = new(nE1, nE2);
            if (eeMap.TryGetValue(pair, out NCollection_List<int>? pPoints))
            {
                pPoints.Append(nVN);
            }
            else
            {
                pPoints = eeMap.SetValueRef(new Pair(nE1, nE2), new NCollection_List<int>());
                pPoints.Append(nVN);
            }
        }

        // Prepare the pairs of faces for intersection
        List<FaceFaceJob> vFaceFace = new();
        _iterator.Initialize(ShapeType.Face, ShapeType.Face);
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            _iterator.Value(out nF1, out nF2);

            if (_glue == global::Kerf.Booleans.GlueMode.Off)
            {
                Face f1 = ShapeCasts.Face(_ds.Shape(nF1));
                Face f2 = ShapeCasts.Face(_ds.Shape(nF2));
                //
                FaceSurfaceAdaptor bas1 = _context!.SurfaceAdaptor(f1);
                FaceSurfaceAdaptor bas2 = _context.SurfaceAdaptor(f2);
                if (bas1.GetType() == Plane && bas2.GetType() == Plane)
                {
                    // Check if the planes are really interfering
                    bool bToIntersect = CheckPlanes(nF1, nF2);
                    if (!bToIntersect)
                    {
                        InterfFF ff0 =  new InterfFF();
                        fFs.Add(ff0);
                        ff0.SetIndices(nF1, nF2);
                        ff0.Init(0, 0);
                        continue;
                    }
                }

                // Check if there is an intersection between edges of the faces.
                // If there is an intersection, check if there is a shift between the edges
                // (intersection point is on some distance from the edges), and move one of
                // the faces to the point of exact edges intersection. This should allow
                // obtaining more precise intersection curves between the faces
                // (at least the curves should reach the boundary).
                // Note, that currently this check considers only closed edges (seam edges).
                Face fShifted1 = f1, fShifted2 = f2;
                // Keep shift value to use it as the tolerance for intersection curves
                double shiftValue = 0.0;

                if (bas1.GetType() != Plane || bas2.GetType() != Plane)
                {
                    Surface surface1 = Tool.Surface(f1, out Placement location1)!;
                    Triangulation? triangulation1 = Tool.Triangulation(f1, out location1);
                    bool isPlane1 = IsPlaneFF(surface1);

                    Surface surface2 = Tool.Surface(f2, out Placement location2)!;
                    Triangulation? triangulation2 = Tool.Triangulation(f2, out location2);
                    bool isPlane2 = IsPlaneFF(surface2);

                    bool isFound = false;
                    for (Iterator itW1 = new(f1); !isFound && itW1.More(); itW1.Next())
                    {
                        for (Iterator itE1 = new(itW1.Value()); !isFound && itE1.More(); itE1.Next())
                        {
                            Edge edge1 = ShapeCasts.Edge(itE1.Value());
                            int edgeIndex1 = _ds.Index(edge1);

                            for (Iterator itW2 = new(f2); !isFound && itW2.More(); itW2.Next())
                            {
                                for (Iterator itE2 = new(itW2.Value()); !isFound && itE2.More(); itE2.Next())
                                {
                                    Edge edge2 = ShapeCasts.Edge(itE2.Value());
                                    int edgeIndex2 = _ds.Index(edge2);

                                    bool isClosed1 =
                                        IsClosedFF(edge1, surface1, triangulation1, location1, isPlane1);
                                    bool isClosed2 =
                                        IsClosedFF(edge2, surface2, triangulation2, location2, isPlane2);
                                    if (!isClosed1 && !isClosed2)
                                    {
                                        continue;
                                    }

                                    if (!eeMap.TryGetValue(new Pair(edgeIndex1, edgeIndex2), out NCollection_List<int>? vertexIndices))
                                    {
                                        continue;
                                    }

                                    for (NCollection_List<int>.Iterator itEEP = new(vertexIndices); itEEP.More();
                                         itEEP.Next())
                                    {
                                        int vertexIndex = itEEP.Value();
                                        Vertex vertex = ShapeCasts.Vertex(_ds.Shape(vertexIndex));
                                        Point3D vertexPoint = Tool.Pnt(vertex);

                                        // Compute points exactly on the edges
                                        ProjectPointOnCurve projPC1 = _context.ProjPC(edge1);
                                        ProjectPointOnCurve projPC2 = _context.ProjPC(edge2);
                                        projPC1.Perform(vertexPoint);
                                        projPC2.Perform(vertexPoint);
                                        if (projPC1.NbPoints() == 0 && projPC2.NbPoints() == 0)
                                        {
                                            continue;
                                        }
                                        Point3D p1 =
                                            projPC1.NbPoints() > 0 ? projPC1.NearestPoint() : vertexPoint;
                                        Point3D p2 =
                                            projPC2.NbPoints() > 0 ? projPC2.NearestPoint() : vertexPoint;

                                        double shiftDist = p1.DistanceTo(p2);
                                        if (shiftDist > Tool.Tolerance(vertex))
                                        {
                                            // Move one of the faces to the point of exact intersection of edges
                                            Transform3D trsf = new Transform3D().WithTranslation(isClosed1 ? new Vector3D(p1, p2) : new Vector3D(p2, p1));
                                            Placement loc = new(trsf);
                                            if (isClosed1)
                                            {
                                                fShifted1.Move(loc);
                                            }
                                            else
                                            {
                                                fShifted2.Move(loc);
                                            }
                                            shiftValue = shiftDist;
                                            isFound = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                //
                FaceFaceJob faceFace =  new FaceFaceJob();
                vFaceFace.Add(faceFace);
                //
                faceFace.SetRunParallel(_runParallel);
                faceFace.SetIndices(nF1, nF2);
                faceFace.SetFaces(fShifted1, fShifted2);
                faceFace.SetBoxes(_ds.ShapeInfo(nF1).Box(), _ds.ShapeInfo(nF2).Box());
                // Note: in case of faces with closed edges it should not be less than value of the shift
                double tolFF = Max(shiftValue, ToleranceFF(bas1, bas2));
                faceFace.SetTolFF(tolFF);
                //
                NCollection_List<PntOn2S> listOfPnts = new();
                GetEFPnts(nF1, nF2, listOfPnts);
                int nbLP = listOfPnts.Extent();
                if (nbLP != 0)
                {
                    faceFace.SetList(listOfPnts);
                }
                //
                faceFace.SetParameters(bApprox, bCompC2D1, bCompC2D2, approxTol);
                faceFace.SetFuzzyValue(_fuzzyValue);
            }
            else
            {
                // for the Glue mode just add all interferences of that type
                InterfFF ff0 =  new InterfFF();
                fFs.Add(ff0);
                ff0.SetIndices(nF1, nF2);
                ff0.SetTangentFaces(false);
                ff0.Init(0, 0);
            }
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        int k, nbFaceFace = vFaceFace.Count;
        using var ps = new ProgressScope(psOuter.Next(), "Performing Face-Face intersection", nbFaceFace);
        for (k = 0; k < nbFaceFace; k++)
        {
            FaceFaceJob faceFace = vFaceFace[k];
            faceFace.SetProgressRange(ps.Next());
        }
        //======================================================
        // Perform intersection
        BooleanParallel.Perform(_runParallel, vFaceFace);
        if (UserBreak(psOuter))
        {
            return;
        }
        //======================================================
        // Treatment of the results

        for (k = 0; k < nbFaceFace; ++k)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            FaceFaceJob faceFace = vFaceFace[k];
            faceFace.Indices(out nF1, out nF2);
            if (!faceFace.IsDone || faceFace.HasErrors)
            {
                InterfFF ff0 =  new InterfFF();
                fFs.Add(ff0);
                ff0.SetIndices(nF1, nF2);
                ff0.Init(0, 0);
                // Warn about failed intersection of faces
                AddIntersectionFailedWarning(faceFace.Face1(), faceFace.Face2());
                continue;
            }
            //
            bool bTangentFaces = faceFace.TangentFaces();
            double tolFF = faceFace.TolFF();
            //
            faceFace.PrepareLines3D(bSplitCurve);
            //
            faceFace.ApplyTrsf();
            //
            // The curves are kept unfrozen in the data structure: InternalLines() (Lines() copies and freezes).
            NCollection_Sequence<IntersectionCurve> cvsX = faceFace.InternalLines();
            NCollection_Sequence<PntOn2Faces> pntsX = faceFace.Points();
            //
            int nbCurves = cvsX.Length();
            int nbPoints = pntsX.Length();
            //
            if (nbCurves != 0 || nbPoints != 0)
            {
                _ds.AddInterf(nF1, nF2);
            }
            //
            InterfFF ff =  new InterfFF();
            fFs.Add(ff);
            ff.SetIndices(nF1, nF2);
            ff.SetTangentFaces(bTangentFaces);
            ff.Init(nbCurves, nbPoints);
            //
            // Curves
            // Fix bounding box expanding coefficient.
            double boxExpandValue = tolFF;
            if (nbCurves > 0)
            {
                // Modify geometric expanding coefficient by topology value,
                // since this bounding box used in sharing (vertex or edge).
                double maxVertexTol = Max(Tool.MaxTolerance(faceFace.Face1(), ShapeType.Vertex),
                                           Tool.MaxTolerance(faceFace.Face2(), ShapeType.Vertex));
                boxExpandValue += maxVertexTol;
            }
            //
            List<SectionCurve> vnc = ff.ChangeCurves();
            for (int i = 1; i <= nbCurves; ++i)
            {
                if (UserBreak(psOuter))
                {
                    return;
                }
                Box box = new();
                IntersectionCurve ic = cvsX[i];
                bool bIsValid = IntersectionTools.CheckCurve(ic, ref box);
                if (bIsValid)
                {
                    SectionCurve nc =  new SectionCurve();
                    vnc.Add(nc);
                    nc.SetCurve(ic);
                    // make sure that the bounding box has the maximal gap
                    box.Enlarge(boxExpandValue);
                    nc.SetBox(box);
                    nc.SetTolerance(Max(ic.Tolerance(), tolFF));
                }
            }
            //
            // Points
            List<DataPoint> vnp = ff.ChangePoints();
            for (int i = 1; i <= nbPoints; ++i)
            {
                PntOn2Faces pi = pntsX[i];
                Point3D p = pi.P1().Pnt();
                //
                vnp.Add(new DataPoint());
                ref DataPoint np = ref CollectionsMarshal.AsSpan(vnp)[^1];
                np.SetPnt(p);
            }
        }
    }

    //=======================================================================
    // function : UpdateSavedTolerance
    // purpose  : Updates the saved tolerance of the vertices of the edge
    //           with new tolerance of edge
    //=======================================================================
    private static void UpdateSavedTolerance(DS ds,
                                             int ne,
                                             double tolNew,
                                             NCollection_IntDataMap<double> mvTol)
    {
        NCollection_List<int> subShapes = ds.ShapeInfo(ne).SubShapes();
        NCollection_List<int>.Iterator itSS = new(subShapes);
        for (; itSS.More(); itSS.Next())
        {
            int nV = itSS.Value();
            if (mvTol.IsBound(nV))
            {
                ref double pTolSaved = ref mvTol.ChangeFind(nV);
                if (pTolSaved < tolNew)
                {
                    pTolSaved = tolNew;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Builds the section edges from the face/face interferences, with their post-treatment.</summary>
    protected void MakeBlocks(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 4);
        if (_glue != global::Kerf.Booleans.GlueMode.Off)
        {
            return;
        }
        //
        List<InterfFF> fFs = _ds!.InterfFF();
        int nbFF = fFs.Count;
        using var ps = new ProgressScope(psOuter.Next(), "Building section edges", nbFF);
        if (nbFF == 0)
        {
            return;
        }
        //
        bool bExist, bValid2D;
        int i, nF1, nF2, nbC, nbP, j;
        int nV1, nV2;
        double t1, t2;
        NCollection_List<PaveBlock>.Iterator itLPB = new();
        Edge es;
        PaveBlock? pbOut = null;
        //
        //-----------------------------------------------------scope f
        //
        // Per-iteration collections (OCCT: a temporary allocator, cleared and reset each iteration).
        NCollection_List<int> lse = new();
        NCollection_List<int> lbv = new();
        NCollection_IntMap mvOnIn = new(100);
        NCollection_IntMap mvCommon = new(100);
        NCollection_IntMap mvStick = new(100);
        NCollection_IntMap mvef = new(100);
        NCollection_IntMap mvBounds = new(100);
        NCollection_IntMap mi = new(100);
        IndexedSet<PaveBlock> mpbOnIn = new();
        HashSet<PaveBlock> mpbCommon = new();
        NCollection_IntDataMap<NCollection_List<int>> dmbv = new(100);
        NCollection_IntDataMap<double> mvTol = new(100);
        //
        // Cross-iteration collections (persist through entire loop)
        HashSet<PaveBlock> mpbAdd = new();
        NCollection_List<PaveBlock> lpb = new();
        IndexedMap<Shape, CoupleOfPaveBlocks> mscpb = new(ShapeHasher.Instance);
        Dictionary<Shape, int> mvi = new(ShapeHasher.Instance);
        Dictionary<PaveBlock, NCollection_List<PaveBlock>> dmExEdges = new();
        NCollection_IntDataMap<int> dmNewSD = new(100);
        NCollection_IntDataMap<NCollection_List<int>> dmvlv = new(100);
        IndexedSet<PaveBlock> microPB = new();
        IndexedSet<Shape> vertsOnRejectedPB = new(ShapeHasher.Instance);
        // Map of PaveBlocks with the faces to which it has to be added
        Dictionary<PaveBlock, NCollection_List<int>> pbFacesMap = new();
        //
        // The vector aFFToRecheck contains indices of potentially problematic Face-Face intersections
        List<int> ffToRecheck = new();
        // aNbFF may be increased while processing this loop, because it is necessary to recheck
        // some of Face-Face intersections to avoid missing section edges
        // aNbFF will be increased to the number of potentially problematic Face-Face intersections
        int nbFFPrev = nbFF;
        for (i = 0; i < nbFF; ++i, ps.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            // after passing through all of Face-Face intersections it is necessary to return
            // to potentially problematic Face-Face intersections and process them one more time
            int curInd = i < nbFFPrev ? i : ffToRecheck[i - nbFFPrev];
            //
            InterfFF ff = fFs[curInd];
            ff.Indices(out nF1, out nF2);
            //
            List<DataPoint> vp = ff.ChangePoints();
            nbP = vp.Count;
            List<SectionCurve> vc = ff.ChangeCurves();
            nbC = vc.Count;
            if (nbP == 0 && nbC == 0)
            {
                continue;
            }
            //
            Face f1 = ShapeCasts.Face(_ds.Shape(nF1));
            Face f2 = ShapeCasts.Face(_ds.Shape(nF2));
            //
            double tolFF = Max(Tool.Tolerance(f1), Tool.Tolerance(f2));
            //
            FaceInfo fi1 = _ds.ChangeFaceInfo(nF1);
            FaceInfo fi2 = _ds.ChangeFaceInfo(nF2);
            //
            // Clear per-iteration collections
            mvOnIn.Clear();
            mvCommon.Clear();
            mpbOnIn.Clear();
            mpbCommon.Clear();
            dmbv.Clear();
            mvTol.Clear();
            lse.Clear();
            lbv.Clear();
            mvStick.Clear();
            mvef.Clear();
            mvBounds.Clear();
            mi.Clear();
            //
            _ds.SubShapesOnIn(nF1, nF2, mvOnIn, mvCommon, mpbOnIn, mpbCommon);
            _ds.SharedEdges(nF1, nF2, lse);
            //
            // 1. Treat Points
            for (j = 0; j < nbP; ++j)
            {
                Vertex v;
                CoupleOfPaveBlocks cpb = new();
                //
                ref DataPoint np = ref CollectionsMarshal.AsSpan(vp)[j];
                Point3D p = np.Pnt();
                //
                bExist = IsExistingVertex(p, tolFF, mvOnIn);
                if (!bExist)
                {
                    AlgoTools.MakeNewVertex(p, tolFF, out v);
                    //
                    cpb.SetIndexInterf(i);
                    cpb.SetIndex(j);
                    mscpb.Add(v, cpb);
                }
            }
            //
            // 2. Treat Curves
            GetStickVertices(nF1, nF2, mvStick, mvef, mi);
            //
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vc[j];
                // DEBt
                nc.InitPaveBlock1();
                //
                // In order to avoid problems connected with
                // extending tolerance of vertex while putting
                // (e.g. see "bugs modalg_6 bug26789_1" test case),
                // all not-common vertices will be checked by
                // BndBoxes before putting. For common-vertices,
                // filtering by BndBoxes is not necessary.
                PutPavesOnCurve(mvOnIn, mvCommon, nc, mi, mvef, mvTol, dmvlv);
            }

            // if some E-F vertex was put on a curve due to large E-F intersection range,
            // and it also was put on another curve correctly then remove this vertex from
            // the first curve. Detect such case if the distance to curve exceeds aTolR3D.
            FilterPavesOnCurves(vc, mvTol);

            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vc[j];
                IntersectionCurve ic = nc.Curve();
                //
                PutStickPavesOnCurve(f1, f2, mi, vc, j, mvStick, mvTol, dmvlv);
                // 904/F7
                if (nbC == 1)
                {
                    PutEFPavesOnCurve(vc, j, mi, mvef, mvTol, dmvlv);
                }
                //
                if (ic.HasBounds())
                {
                    lbv.Clear();
                    //
                    PutBoundPaveOnCurve(f1, f2, nc, lbv);
                    //
                    if (!lbv.IsEmpty())
                    {
                        // OCCT binds a copy of the list (aLBV is cleared and reused).
                        dmbv.Bind(j, new NCollection_List<int>(lbv));
                        NCollection_List<int>.Iterator itI = new(lbv);
                        for (; itI.More(); itI.Next())
                        {
                            mvBounds.Add(itI.Value());
                        }
                    }
                }
            } // for (j=0; j<aNbC; ++j) {

            JoinCurveEnds(vc);

            // Put closing pave if needed
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vc[j];
                PutClosingPaveOnCurve(nc);
            }
            //

            BoxTree pbTree = new();
            {
                // Fill the tree with boxes of pave blocks ON/IN
                // Tree will be build on first selection from the tree.
                int nbPB = mpbOnIn.Count;
                pbTree.SetSize(nbPB);
                for (int iPB = 1; iPB <= nbPB; ++iPB)
                {
                    PaveBlock pb = mpbOnIn[iPB - 1];
                    if (!pb.HasEdge())
                    {
                        continue;
                    }

                    if (_ds.ShapeInfo(pb.OriginalEdge()).HasFlag())
                    {
                        continue;
                    }

                    pbTree.Add(iPB, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(_ds.ShapeInfo(pb.Edge()).Box()));
                }
            }

            // Added additional check of Face-Face intersection to avoid missing section edges
            // because of sequence of Face-Face interference processing
            bool isToRecheck = nbC > 0 && i < nbFFPrev;
            //
            // 3. Make section edges
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vc[j];
                IntersectionCurve ic = nc.Curve();
                double tolR3D = Max(nc.Tolerance(), nc.TangentialTolerance());
                //
                NCollection_List<PaveBlock> lpbc = nc.ChangePaveBlocks();
                PaveBlock pb1 = nc.ChangePaveBlock1();
                //
                lpb.Clear();
                pb1.Update(lpb, false);
                //
                if (lpb.Extent() != 0)
                {
                    isToRecheck = false;
                }

                itLPB.Initialize(lpb);
                for (; itLPB.More(); itLPB.Next())
                {
                    PaveBlock pb = itLPB.Value();
                    pb.Indices(out nV1, out nV2);
                    pb.Range(out t1, out t2);
                    //
                    if (Math.Abs(t1 - t2) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }

                    // Check validity of the block for the faces:
                    // classify bounding and middle points on the curve
                    // relatively both faces
                    bValid2D = _context!.IsValidBlockForFaces(t1, t2, ic, f1, f2, tolR3D);
                    if (!bValid2D)
                    {
                        continue;
                    }
                    //
                    int nEOut = 0; // OCCT: uninitialised, read only when IsExistingPaveBlock writes it
                    double tolNew = -1.0;
                    bExist = IsExistingPaveBlock(pb, nc, lse, ref nEOut, ref tolNew);
                    if (bExist)
                    {
                        // Update edge with new tolerance
                        UpdateEdgeTolerance(nEOut, tolNew);
                        // Update aMVTol map with new tolerances of vertices
                        UpdateSavedTolerance(_ds, nEOut, tolNew, mvTol);
                        continue;
                    }
                    //
                    Vertex v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                    Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                    //
                    // check if the pave block has a valid range
                    double first = 0.0, last = 0.0;
                    if (!BrepLibrary.FindValidRange(new CurveAdaptor(ic.Curve()!),
                                                tolR3D,
                                                t1,
                                                Tool.Pnt(v1),
                                                Max(tolR3D, Tool.Tolerance(v1)),
                                                t2,
                                                Tool.Pnt(v2),
                                                Max(tolR3D, Tool.Tolerance(v2)),
                                                ref first,
                                                ref last))
                    {
                        // If the pave block does not have valid range, i.e. it is completely
                        // covered by the tolerance spheres of its vertices, it will be
                        // passed into post treatment process to fuse its vertices.
                        // The pave block itself will not be kept.
                        if (!mvBounds.Contains(nV1) && !mvBounds.Contains(nV2))
                        {
                            microPB.Add(pb);
                            // keep vertices for post treatment
                            mvi[v1] = nV1;
                            mvi[v2] = nV2;
                        }
                        continue;
                    }
                    //
                    bExist =
                        IsExistingPaveBlock(pb, nc, tolR3D, mpbOnIn, pbTree, mpbCommon, ref pbOut, ref tolNew);
                    if (bExist)
                    {
                        bool bInF1 =
                            (fi1.PaveBlocksOn().Contains(pbOut!) || fi1.PaveBlocksIn().Contains(pbOut!));
                        bool bInF2 =
                            (fi2.PaveBlocksOn().Contains(pbOut!) || fi2.PaveBlocksIn().Contains(pbOut!));
                        if (!bInF1 || !bInF2)
                        {
                            // Update edge to touch both faces
                            int nE = pbOut!.Edge();
                            Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                            double tolE = Tool.Tolerance(e);
                            if (tolNew < nc.Tolerance())
                            {
                                tolNew = nc.Tolerance(); // use real tolerance of intersection
                            }
                            if (tolNew > tolE)
                            {
                                UpdateEdgeTolerance(nE, tolNew);
                                // Update aMVTol map with new tolerances of vertices
                                UpdateSavedTolerance(_ds, nE, tolNew, mvTol);
                            }

                            // Face without pave block
                            int nF = bInF1 ? nF2 : nF1;
                            if (!pbFacesMap.TryGetValue(pbOut, out NCollection_List<int>? pFaces))
                            {
                                pFaces = pbFacesMap.SetValueRef(pbOut, new NCollection_List<int>());
                            }
                            // List is expected to be short, so we allow the check here
                            if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                            {
                                pFaces.Append(nF);
                            }

                            // Try fusing the vertices of the existing pave block
                            // with the vertices put on the real section curve (except
                            // for technological vertices, which will be removed)
                            pbOut.Indices(out int nVOut1, out int nVOut2);
                            if (nV1 != nVOut1 && nV1 != nVOut2 && !mvBounds.Contains(nV1))
                            {
                                vertsOnRejectedPB.Add(v1);
                            }
                            if (nV2 != nVOut1 && nV2 != nVOut2 && !mvBounds.Contains(nV2))
                            {
                                vertsOnRejectedPB.Add(v2);
                            }

                            if (mpbAdd.Add(pbOut))
                            {
                                // Add edge for processing as the section edge
                                PreparePostTreatFF(curInd, j, pbOut, mscpb, mvi, lpbc);
                            }
                        }
                        continue;
                    }
                    //
                    // Make Edge
                    AlgoTools.MakeEdge(ic, v1, t1, v2, t2, tolR3D, out es);
                    // Make p-curves
                    AlgoTools.MakePCurve(es,
                                                  f1,
                                                  f2,
                                                  ic,
                                                  _sectionAttribute.PCurveOnS1(),
                                                  _sectionAttribute.PCurveOnS2(),
                                                  _context);
                    //
                    // Append the Pave Block to the Curve j
                    lpbc.Append(pb);
                    //
                    // Keep info for post treatment
                    CoupleOfPaveBlocks cpb = new();
                    cpb.SetIndexInterf(curInd);
                    cpb.SetIndex(j);
                    cpb.SetPaveBlock1(pb);
                    //
                    mscpb.Add(es, cpb);
                    mvi[v1] = nV1;
                    mvi[v2] = nV2;
                    //
                    mvTol.UnBind(nV1);
                    mvTol.UnBind(nV2);

                    // Add existing pave blocks for post treatment
                    ProcessExistingPaveBlocks(curInd,
                                              j,
                                              nF1,
                                              nF2,
                                              es,
                                              mpbOnIn,
                                              pbTree,
                                              mscpb,
                                              mvi,
                                              lpbc,
                                              pbFacesMap,
                                              mpbAdd);
                }
                //
                lpbc.RemoveFirst();
            } // for (j=0; j<aNbC; ++j) {
            if (isToRecheck)
            {
                ffToRecheck.Add(curInd);
                ++nbFF;
            }
            //
            // back to previous tolerance values for unused vertices
            // and forget about SD groups of such vertices
            foreach (KeyValuePair<int, double> itMV in mvTol)
            {
                nV1 = itMV.Key;
                double tol = itMV.Value;
                //
                Vertex v = ShapeCasts.Vertex(_ds.Shape(nV1));
                BrepTVertex tv = (BrepTVertex)v.TShape()!;
                // Kerf: frozen. OCCT writes the saved tolerance back also when it is the vertex's own (a vertex
                // only used as an existing pave keeps its tolerance): a no-op, which on a frozen argument vertex would
                // throw, so it is skipped. A different value is written as OCCT writes it (decision 875).
                if (tv.Tolerance() != tol)
                {
                    tv.Tolerance(tol);
                }
                // reset bnd box
                ShapeInfo sids = _ds.ChangeShapeInfo(nV1);
                ref Box boxDS = ref sids.ChangeBox();
                boxDS = new Box();
                Bounds.Add(v, ref boxDS);
                boxDS.SetGap(boxDS.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
                //
                if (dmvlv.IsBound(nV1))
                {
                    dmvlv.UnBind(nV1);
                }
            }
            //
            ProcessExistingPaveBlocks(curInd,
                                      nF1,
                                      nF2,
                                      mpbOnIn,
                                      pbTree,
                                      dmbv,
                                      mscpb,
                                      mvi,
                                      pbFacesMap,
                                      mpbAdd);
        } // for (i=0; i<aNbFF; ++i) {

        // Remove "micro" section edges
        RemoveMicroSectionEdges(mscpb, microPB);

        // post treatment
        MakeSDVerticesFF(dmvlv, dmNewSD);
        PostTreatFF(mscpb,
                    dmExEdges,
                    dmNewSD,
                    microPB,
                    vertsOnRejectedPB,
                    psOuter.Next(2));
        if (HasErrors)
        {
            return;
        }
        // reduce tolerances of section edges where it is appropriate
        CorrectToleranceOfSE();
        //
        // update face info
        UpdateFaceInfo(dmExEdges, dmNewSD, pbFacesMap);
        // Update all pave blocks
        UpdatePaveBlocks(dmNewSD);
        //
        // Treat possible common zones by trying to put each section edge
        // into all faces, not participated in creation of that edge, as IN edge

        PutSEInOtherFaces(psOuter.Next());
    }

    //=================================================================================================

    /// <summary>Makes a same-domain vertex for each group of coinciding vertices.</summary>
    protected void MakeSDVerticesFF(NCollection_IntDataMap<NCollection_List<int>> dmvlv,
                                    NCollection_IntDataMap<int> dmNewSD)
    {
        // Create a new SD vertex for each group of coinciding vertices
        // and put new substitutions to theDMNewSD.
        // The groups are visited in the map's order: the new vertices' indices depend on it (decision 784).
        foreach (KeyValuePair<int, NCollection_List<int>> itG in dmvlv)
        {
            NCollection_List<int> list = itG.Value;
            // make SD vertices w/o creation of interfs
            int nSD = MakeSDVertices(list, false);
            // update theDMNewSD
            NCollection_List<int>.Iterator itL = new(list);
            for (; itL.More(); itL.Next())
            {
                int nV = itL.Value();
                dmNewSD.Bind(nV, nSD);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The post-treatment of the section edges and vertices: they are intersected among themselves (and with the
    /// existing edges and vertices involved) by a nested pave filler, and the results are put into the data structure.
    /// </summary>
    protected void PostTreatFF(IndexedMap<Shape, CoupleOfPaveBlocks> mscpb,
                               Dictionary<PaveBlock, NCollection_List<PaveBlock>> aDMExEdges,
                               NCollection_IntDataMap<int> aDMNewSD,
                               IndexedSet<PaveBlock> microPB,
                               IndexedSet<Shape> vertsOnRejectedPB,
                               ProgressRange range)
    {
        int nbS = mscpb.Count;
        if (nbS == 0)
        {
            return;
        }
        //
        bool bHasPaveBlocks, bOld;
        int nSx, nVSD = 0, iX, iP, iC, j, nV, iV = 0, iE, k;
        int nbLPBx;
        ShapeType type;
        Shape v, e;
        NCollection_List<Shape>.Iterator itLS = new();
        NCollection_List<PaveBlock>.Iterator itLPB = new();
        DS pds;
        PaveBlock pb1;
        Span<Pave> pave = stackalloc Pave[2];
        ShapeInfo si = new();
        //
        NCollection_List<Shape> ls = new();
        PaveFiller pf = new();
        pf.SetIsPrimary(false);
        pf.SetNonDestructive(_nonDestructive);
        //
        List<InterfFF> fFs = _ds!.InterfFF();
        int nbFF = fFs.Count;
        //

        // Find unused vertices
        IndexedSet<Shape> vertsUnused = new(ShapeHasher.Instance);
        NCollection_IntMap indMap = new();
        for (int i = 0; i < nbFF; i++)
        {
            InterfFF ff = fFs[i];
            ff.Indices(out int nF1, out int nF2);

            NCollection_IntMap mv = new(), mvef = new(), mi = new();
            GetStickVertices(nF1, nF2, mv, mvef, mi);
            List<SectionCurve> vc = ff.ChangeCurves();
            RemoveUsedVertices(vc, mv);

            // The vertices are visited in the map's order, which orders VertsUnused and so the nested filler's
            // arguments (decision 784).
            foreach (int indV in mv)
            {
                Shape vertex = _ds.Shape(indV);
                if (indMap.Add(indV))
                {
                    vertsUnused.Add(vertex);
                }
                else
                {
                    vertsUnused.Remove(vertex);
                }
            }
        }
        /////////////////////

        int nbME = microPB.Count;
        int nbVOnRPB = vertsOnRejectedPB.Count;
        // 0
        if (nbS == 1 && (nbME == 0) && (nbVOnRPB == 0) && vertsUnused.Count == 0)
        {
            Shape s = mscpb.KeyAt(0);
            CoupleOfPaveBlocks cpb = mscpb.ValueAt(0);
            //
            type = s.Type;
            if (type == ShapeType.Vertex)
            {
                si.SetShapeType(type);
                si.SetShape(s);
                iV = _ds.Append(si);
                //
                iX = cpb.IndexInterf();
                iP = cpb.Index();
                InterfFF ff = fFs[iX];
                List<DataPoint> vnp = ff.ChangePoints();
                ref DataPoint np = ref CollectionsMarshal.AsSpan(vnp)[iP];
                np.SetIndex(iV);
            }
            else if (type == ShapeType.Edge)
            {
                pb1 = cpb.PaveBlock1()!;
                //
                if (pb1.HasEdge())
                {
                    NCollection_List<PaveBlock> lpBx = new();
                    lpBx.Append(pb1);
                    aDMExEdges[pb1] = lpBx;
                }
                else
                {
                    si.SetShapeType(type);
                    si.SetShape(s);
                    iE = _ds.Append(si);
                    //
                    pb1.SetEdge(iE);
                }
            }
            return;
        }
        //
        // 1 prepare arguments
        // Avoid intersection of existing edges among themselves
        Compound existingEdges = new();
        new BrepBuilder().MakeCompound(ref existingEdges);
        HashSet<Shape> addedSD = new(ShapeHasher.Instance);
        for (k = nbS; k > 0; --k)
        {
            Shape s = mscpb.KeyAt(k - 1);
            PaveBlock? pb = mscpb.ValueAt(k - 1).PaveBlock1();
            if (pb is not null && pb.HasEdge())
            {
                new BrepBuilder().Add(existingEdges, s);
            }
            else
            {
                ls.Append(s);
            }
            // add vertices-candidates for SD from the map aDMNewSD,
            // so that they took part in fuse operation.
            Iterator itV = new(s);
            for (; itV.More(); itV.Next())
            {
                Shape ver = itV.Value();
                int iVer = _ds.Index(ver);
                if (aDMNewSD.Find(iVer, out int pSD))
                {
                    Shape vsd = _ds.Shape(pSD);
                    if (addedSD.Add(vsd))
                    {
                        ls.Append(vsd);
                    }
                }
            }
        }
        if (existingEdges.ChildCount > 0)
        {
            ls.Append(existingEdges);
        }
        //
        // The section edges considered as a micro should be
        // specially treated - their vertices should be united and
        // the edge itself should be removed. Thus, we add only
        // its vertices into operation.
        //
        BrepBuilder bb = new();
        Span<int> nVerts = stackalloc int[2];
        Vertex[] verts = new Vertex[2];
        for (k = 1; k <= nbME; ++k)
        {
            microPB[k - 1].Indices(out nVerts[0], out nVerts[1]);
            for (int i = 0; i < 2; ++i)
            {
                bool isSD = aDMNewSD.Find(nVerts[i], out int pSD);
                verts[i] = ShapeCasts.Vertex(_ds.Shape(isSD ? pSD : nVerts[i]));
                if (addedSD.Add(verts[i]))
                {
                    ls.Append(verts[i]);
                }
            }
            //
            if (verts[0].IsSame(verts[1]))
            {
                continue;
            }
            //
            // make sure these vertices will be united
            Point3D p1 = Tool.Pnt(verts[0]);
            Point3D p2 = Tool.Pnt(verts[1]);
            //
            double dist = p1.DistanceTo(p2);
            double tolV1 = Tool.Tolerance(verts[0]);
            double tolV2 = Tool.Tolerance(verts[1]);
            //
            dist -= (tolV1 + tolV2);
            if (dist > 0.0)
            {
                dist /= 2.0;
                bb.UpdateVertex(verts[0], tolV1 + dist);
                bb.UpdateVertex(verts[1], tolV2 + dist);
            }
        }

        // Add vertices put on the real section curves to unify them with the
        // vertices of the edges, by which these sections curves have been rejected
        // and with unused vertices
        IndexedSet<Shape>[] verMap = [vertsOnRejectedPB, vertsUnused];
        for (int imap = 0; imap < 2; imap++)
        {
            int nbVer = verMap[imap].Count;
            for (int i = 1; i <= nbVer; ++i)
            {
                Shape ver = verMap[imap][i - 1];
                int iVer = _ds.Index(ver);
                if (aDMNewSD.Find(iVer, out int pSD))
                {
                    ver = _ds.Shape(pSD);
                }

                if (addedSD.Add(ver))
                {
                    ls.Append(ver);
                }
            }
        }
        //
        using var ps = new ProgressScope(range, "Intersection of section edges", 1);

        // 2 Fuse shapes
        pf.SetRunParallel(_runParallel);
        pf.SetArguments(ls);
        pf.Perform(ps.Next());
        if (pf.HasErrors)
        {
            AddError(new AlertPostTreatFF());
            return;
        }
        pds = pf.PDS()!;
        //
        // Map to store the real tolerance of the common block
        // and avoid repeated computation of it
        Dictionary<CommonBlock, double> mcbTol = new();
        // Map to avoid creation of different pave blocks for
        // the same intersection edge
        NCollection_IntDataMap<PaveBlock> mepb = new();
        //
        itLS.Initialize(ls);
        for (; itLS.More(); itLS.Next())
        {
            Shape sx = itLS.Value();
            if (sx.Type == ShapeType.Compound)
            {
                for (Iterator itC = new(sx); itC.More(); itC.Next())
                {
                    ls.Append(itC.Value());
                }
                continue;
            }
            nSx = pds.Index(sx);
            ShapeInfo sIx = pds.ShapeInfo(nSx);
            //
            type = sIx.ShapeType();
            //
            if (type == ShapeType.Vertex)
            {
                bool bIntersectionPoint = mscpb.ContainsKey(sx);
                //
                if (pds.HasShapeSD(nSx, ref nVSD))
                {
                    v = pds.Shape(nVSD);
                }
                else
                {
                    v = sx;
                }
                // index of new vertex in theDS -> iV
                iV = _ds.Index(v);
                if (iV < 0)
                {
                    si.SetShapeType(type);
                    si.SetShape(v);
                    iV = _ds.Append(si);
                }
                //
                if (!bIntersectionPoint)
                {
                    // save SD connection
                    nSx = _ds.Index(sx);
                    if (nSx != iV)
                    {
                        aDMNewSD.Bind(nSx, iV);
                        _ds.AddShapeSD(nSx, iV);
                    }
                }
                else
                {
                    // update FF interference
                    CoupleOfPaveBlocks cpb = mscpb.GetValue(sx);
                    iX = cpb.IndexInterf();
                    iP = cpb.Index();
                    InterfFF ff = fFs[iX];
                    List<DataPoint> vnp = ff.ChangePoints();
                    ref DataPoint np = ref CollectionsMarshal.AsSpan(vnp)[iP];
                    np.SetIndex(iV);
                }
            } // if (aType==TopAbs_VERTEX) {
            //
            else if (type == ShapeType.Edge)
            {
                bHasPaveBlocks = pds.HasPaveBlocks(nSx);
                CoupleOfPaveBlocks cpb = mscpb.GetValue(sx);
                iX = cpb.IndexInterf();
                iC = cpb.Index();
                pb1 = cpb.PaveBlock1()!;
                //
                bOld = pb1.HasEdge();
                if (bOld)
                {
                    NCollection_List<PaveBlock> lpBx = new();
                    aDMExEdges[pb1] = lpBx;
                }
                //
                if (!bHasPaveBlocks)
                {
                    if (bOld)
                    {
                        aDMExEdges.GetValue(pb1).Append(pb1);
                    }
                    else
                    {
                        si.SetShapeType(type);
                        si.SetShape(sx);
                        iE = _ds.Append(si);
                        //
                        pb1.SetEdge(iE);
                    }
                }
                else
                {
                    InterfFF ff = fFs[iX];
                    List<SectionCurve> vnc = ff.ChangeCurves();
                    SectionCurve nc = vnc[iC];
                    NCollection_List<PaveBlock> lpbc = nc.ChangePaveBlocks();
                    //
                    // check if edge occurred to be micro edge;
                    // note we check not the edge aSx itself, but its image in aPDS
                    NCollection_List<PaveBlock> lpBx = pds.PaveBlocks(nSx);
                    nbLPBx = lpBx.Extent();
                    if (nbLPBx == 0 || (nbLPBx == 1 && !lpBx.First().HasShrunkData))
                    {
                        NCollection_List<PaveBlock>.Iterator it = new(lpbc);
                        for (; it.More(); it.Next())
                        {
                            if (it.Value() == pb1)
                            {
                                lpbc.Remove(it);
                                break;
                            }
                        }

                        // The edge became micro edge, check vertices for SD
                        Iterator itV = new(sx);
                        for (; itV.More(); itV.Next())
                        {
                            ls.Append(itV.Value());
                        }

                        continue;
                    }
                    //
                    if (bOld && nbLPBx == 0)
                    {
                        aDMExEdges.GetValue(pb1).Append(pb1);
                        continue;
                    }
                    //
                    if (!bOld)
                    {
                        itLPB.Initialize(lpbc);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            PaveBlock pbc = itLPB.Value();
                            if (pbc == pb1)
                            {
                                lpbc.Remove(itLPB);
                                break;
                            }
                        }
                    }
                    //
                    if (nbLPBx != 0)
                    {
                        itLPB.Initialize(lpBx);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            PaveBlock pBx = itLPB.Value();
                            PaveBlock pbRx = pds.RealPaveBlock(pBx);
                            //
                            // update vertices of paves
                            pave[0] = pBx.Pave1();
                            pave[1] = pBx.Pave2();
                            for (j = 0; j < 2; ++j)
                            {
                                nV = pave[j].Index();
                                v = pds.Shape(nV);
                                // index of new vertex in myDS -> iV
                                iV = _ds.Index(v);
                                if (iV < 0)
                                {
                                    si.SetShapeType(ShapeType.Vertex);
                                    si.SetShape(v);
                                    iV = _ds.Append(si);
                                }
                                Pave p1 = j == 0 ? pb1.Pave1() : pb1.Pave2();
                                if (p1.Index() != iV)
                                {
                                    if (p1.Parameter() == pave[j].Parameter())
                                    {
                                        aDMNewSD.Bind(p1.Index(), iV);
                                        _ds.AddShapeSD(p1.Index(), iV);
                                    }
                                    else
                                    {
                                        // check aPDS to have the SD connection between these vertices
                                        Shape vPave = _ds.Shape(p1.Index());
                                        int nVnewSD = 0, nVnew = pds.Index(vPave);
                                        if (pds.HasShapeSD(nVnew, ref nVnewSD))
                                        {
                                            if (nVnewSD == nV)
                                            {
                                                aDMNewSD.Bind(p1.Index(), iV);
                                                _ds.AddShapeSD(p1.Index(), iV);
                                            }
                                        }
                                    }
                                }
                                //
                                pave[j].SetIndex(iV);
                            }
                            //
                            // add edge
                            e = pds.Shape(pbRx.Edge());
                            iE = _ds.Index(e);
                            if (iE < 0)
                            {
                                si.SetShapeType(type);
                                si.SetShape(e);
                                iE = _ds.Append(si);
                            }
                            //
                            // update real edge tolerance according to distances in common block if any
                            if (pds.IsCommonBlock(pbRx))
                            {
                                CommonBlock cb = pds.CommonBlock(pbRx)!;
                                if (!mcbTol.TryGetValue(cb, out double pTol))
                                {
                                    double tol = BuilderTools.ComputeToleranceOfCB(cb, pds, pf.Context());
                                    pTol = mcbTol.SetValueRef(cb, tol);
                                }
                                //
                                if (nc.Tolerance() < pTol)
                                {
                                    nc.SetTolerance(pTol);
                                }
                            }
                            // append new PaveBlock to aLPBC
                            if (!mepb.Find(iE, out PaveBlock pPBC))
                            {
                                pPBC = mepb.Bound(iE, new PaveBlock());
                                Pave paveR1, paveR2;
                                paveR1 = pbRx.Pave1();
                                paveR2 = pbRx.Pave2();
                                paveR1.SetIndex(_ds.Index(pds.Shape(paveR1.Index())));
                                paveR2.SetIndex(_ds.Index(pds.Shape(paveR2.Index())));
                                //
                                pPBC.SetPave1(paveR1);
                                pPBC.SetPave2(paveR2);
                                pPBC.SetEdge(iE);
                            }
                            //
                            if (bOld)
                            {
                                pPBC.SetOriginalEdge(pb1.OriginalEdge());
                                aDMExEdges.GetValue(pb1).Append(pPBC);
                            }
                            else
                            {
                                lpbc.Append(pPBC);
                            }
                        }
                    }
                }
            } // else if (aType==TopAbs_EDGE)
        } // for (; aItLS.More(); aItLS.Next()) {

        // Update SD for vertices that did not participate in operation
        // OCCT assigns through the map's iterator; the port visits a snapshot of the keys in the map's order and
        // assigns through the map (the same values in the same order; the order matters for chains, decision 784).
        NCollection_List<int> keys = new();
        foreach (KeyValuePair<int, int> kv in aDMNewSD)
        {
            keys.Append(kv.Key);
        }
        for (NCollection_List<int>.Iterator itDM = new(keys); itDM.More(); itDM.Next())
        {
            int key = itDM.Value();
            if (aDMNewSD.Find(aDMNewSD.Find(key), out int pSD))
            {
                aDMNewSD.ChangeFind(key) = pSD;
                _ds.AddShapeSD(key, pSD);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Updates the face information with the section edges and vertices, and with the pave blocks and vertices
    /// that the post-treatment replaced.
    /// </summary>
    protected void UpdateFaceInfo(Dictionary<PaveBlock, NCollection_List<PaveBlock>> dme,
                                  NCollection_IntDataMap<int> dmv,
                                  Dictionary<PaveBlock, NCollection_List<int>> pbFacesMap)
    {
        int i, j, nV1, nF1, nF2, nbFF, nbC, nbP;
        NCollection_List<PaveBlock>.Iterator aItLPB = new();
        NCollection_IntMap mf = new();

        // Unify pave blocks of the existing edges united on the post-treat stage
        NCollection_IntDataMap<NCollection_List<PaveBlock>> edgeLPB = new();

        List<InterfFF> fFs = _ds!.InterfFF();
        nbFF = fFs.Count;
        // 1. Sections (curves, points);
        for (i = 0; i < nbFF; ++i)
        {
            InterfFF ff = fFs[i];
            ff.Indices(out nF1, out nF2);
            //
            FaceInfo fi1 = _ds.ChangeFaceInfo(nF1);
            FaceInfo fi2 = _ds.ChangeFaceInfo(nF2);
            //
            // 1.1. Section edges
            List<SectionCurve> vnc = ff.ChangeCurves();
            nbC = vnc.Count;
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vnc[j];
                NCollection_List<PaveBlock> lpbc = nc.ChangePaveBlocks();
                //
                // Add section edges to face info
                aItLPB.Initialize(lpbc);
                for (; aItLPB.More();)
                {
                    PaveBlock pb = aItLPB.Value();
                    //
                    // Treat existing pave blocks
                    if (dme.ContainsKey(pb))
                    {
                        NCollection_List<PaveBlock> lpb = dme.GetValue(pb);
                        UpdateExistingPaveBlocks(pb, lpb, pbFacesMap);

                        NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            int nE1 = itLPB.Value().Edge();
                            if (!edgeLPB.Find(nE1, out NCollection_List<PaveBlock> pLPBOnE1))
                            {
                                pLPBOnE1 = edgeLPB.Bound(nE1, new NCollection_List<PaveBlock>());
                            }
                            pLPBOnE1.Append(itLPB.Value());
                        }

                        lpbc.Remove(aItLPB);
                        continue;
                    }
                    //
                    fi1.ChangePaveBlocksSc().Add(pb);
                    fi2.ChangePaveBlocksSc().Add(pb);
                    // Add edge-PB connection
                    int nE = pb.Edge();
                    if (!edgeLPB.Find(nE, out NCollection_List<PaveBlock> pLPBOnE))
                    {
                        pLPBOnE = edgeLPB.Bound(nE, new NCollection_List<PaveBlock>());
                    }
                    pLPBOnE.Append(pb);

                    aItLPB.Next();
                }
            }
            //
            // 1.2. Section vertices
            List<DataPoint> vnp = ff.Points();
            nbP = vnp.Count;
            for (j = 0; j < nbP; ++j)
            {
                DataPoint np = vnp[j];
                nV1 = np.Index();
                if (nV1 < 0)
                {
                    continue;
                }
                fi1.ChangeVerticesSc().Add(nV1);
                fi2.ChangeVerticesSc().Add(nV1);
            }
            //
            mf.Add(nF1);
            mf.Add(nF2);
        }

        bool bNewCB = false;
        {
            // Unify pave blocks of the existing edges united on the post-treat stage
            // by making new Common blocks from them
            // (the edges are visited in the map's order, which orders the new common blocks, decision 784)
            foreach (KeyValuePair<int, NCollection_List<PaveBlock>> itDM in edgeLPB)
            {
                NCollection_List<PaveBlock> lpb = itDM.Value;
                if (lpb.Extent() == 1)
                {
                    continue;
                }

                bNewCB = true;

                // Find or create common block
                CommonBlock? cb = null;
                // Collect all faces attached to common blocks
                NCollection_IntMap mFaces = new();
                // Collect all pave blocks attached to common blocks
                IndexedSet<PaveBlock> mPaveBlocks = new();

                NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
                for (; itLPB.More(); itLPB.Next())
                {
                    PaveBlock pb = itLPB.Value();
                    mPaveBlocks.Add(pb);

                    if (_ds.IsCommonBlock(pb))
                    {
                        CommonBlock pbcb = _ds.CommonBlock(pb)!;
                        // Get pave blocks
                        NCollection_List<PaveBlock> lpbOnCB = pbcb.PaveBlocks();
                        for (NCollection_List<PaveBlock>.Iterator it = new(lpbOnCB); it.More();
                             it.Next())
                        {
                            mPaveBlocks.Add(it.Value());
                        }

                        // Get faces
                        NCollection_List<int> lFacesOnCB = pbcb.Faces();
                        for (NCollection_List<int>.Iterator it = new(lFacesOnCB); it.More(); it.Next())
                        {
                            mFaces.Add(it.Value());
                        }

                        cb ??= pbcb;
                    }
                }

                if (cb is null)
                {
                    // None of the pave blocks in the list is a common block,
                    // so create the new one.
                    cb = new CommonBlock();
                    cb.SetPaveBlocks(lpb);
                    itLPB.Initialize(lpb);
                    for (; itLPB.More(); itLPB.Next())
                    {
                        PaveBlock pb = itLPB.Value();
                        _ds.SetCommonBlock(pb, cb);
                    }
                }
                else
                {
                    // Update common block with new pave blocks
                    NCollection_List<PaveBlock> lpbNew = new();
                    {
                        int nbPB = mPaveBlocks.Count;
                        for (int iPB = 0; iPB < nbPB; ++iPB)
                        {
                            PaveBlock pb = mPaveBlocks[iPB];
                            _ds.SetCommonBlock(pb, cb);
                            lpbNew.Append(pb);
                        }
                    }
                    cb.SetPaveBlocks(lpbNew);

                    // Update faces of the common block
                    // (in the map's order, decision 784)
                    NCollection_List<int> lFaces = new();
                    foreach (int nF in mFaces)
                    {
                        lFaces.Append(nF);
                    }
                    cb.SetFaces(lFaces);
                }
            }
        }

        bool bVerts = dmv.Extent() > 0;
        bool bEdges = dme.Count > 0 || bNewCB;
        //
        if (!bVerts && !bEdges)
        {
            return;
        }
        //
        // 2. Update Face Info information with new vertices and new
        //    pave blocks created in PostTreatFF from existing ones
        int nV2;
        //
        foreach (int nF in mf)
        {
            nF1 = nF;
            //
            FaceInfo fi = _ds.ChangeFaceInfo(nF1);
            //
            // 2.1. Update information about vertices
            if (bVerts)
            {
                NCollection_IntMap mvOn = fi.ChangeVerticesOn();
                NCollection_IntMap mvIn = fi.ChangeVerticesIn();
                //
                foreach (KeyValuePair<int, int> itMV in dmv)
                {
                    nV1 = itMV.Key;
                    nV2 = itMV.Value;
                    //
                    if (mvOn.Remove(nV1))
                    {
                        mvOn.Add(nV2);
                    }
                    //
                    if (mvIn.Remove(nV1))
                    {
                        mvIn.Add(nV2);
                    }
                } // for (; aItMV.More(); aItMV.Next()) {
            } // if (bVerts) {
            //
            // 2.2. Update information about pave blocks
            if (bEdges)
            {
                HashSet<PaveBlock> mpbFence = new();
                IndexedSet<PaveBlock>[] pMPB = [fi.ChangePaveBlocksOn(),
                                                                  fi.ChangePaveBlocksIn(),
                                                                  fi.ChangePaveBlocksSc()];
                for (i = 0; i < 3; ++i)
                {
                    IndexedSet<PaveBlock> mpbCopy = new(pMPB[i]);
                    pMPB[i].Clear();
                    int nbPB = mpbCopy.Count;
                    for (j = 1; j <= nbPB; ++j)
                    {
                        PaveBlock pb = mpbCopy[j - 1];
                        if (dme.TryGetValue(pb, out NCollection_List<PaveBlock>? pLPB) && !pLPB.IsEmpty())
                        {
                            aItLPB.Initialize(pLPB);
                            for (; aItLPB.More(); aItLPB.Next())
                            {
                                PaveBlock pb1 = aItLPB.Value();
                                PaveBlock pbr = _ds.RealPaveBlock(pb1);
                                if (mpbFence.Add(pbr))
                                {
                                    pMPB[i].Add(pbr);
                                }
                            }
                        }
                        else
                        {
                            PaveBlock pbr = _ds.RealPaveBlock(pb);
                            if (mpbFence.Add(pbr))
                            {
                                pMPB[i].Add(pbr);
                            }
                        }
                    } // for (j = 1; j <= aNbPB; ++j) {
                } // for (i = 0; i < 2; ++i) {
            } // if (bEdges) {
        }
    }

    //=================================================================================================

    /// <summary>Whether the point coincides with one of the vertices <paramref name="aMVOnIn"/>.</summary>
    protected bool IsExistingVertex(in Point3D aP, double tolR3D, NCollection_IntMap aMVOnIn)
    {
        bool bRet;
        int nV, iFlag;
        double tolCheck;
        Box boxP = new();
        //
        tolCheck = tolR3D + _fuzzyValue;
        bRet = true;
        //
        boxP.Add(aP);
        boxP.Enlarge(tolR3D);
        //
        foreach (int it in aMVOnIn)
        {
            nV = it;
            ShapeInfo siv = _ds!.ShapeInfo(nV);
            Vertex v = ShapeCasts.Vertex(siv.Shape());
            Box boxV = siv.Box();
            //
            if (!boxP.IsOut(boxV))
            {
                iFlag = AlgoTools.ComputeVV(v, aP, tolCheck);
                if (iFlag == 0)
                {
                    return bRet;
                }
            }
        }
        return !bRet;
    }

    //=================================================================================================

    /// <summary>Whether the pave block of the section curve coincides with one of the shared edges <paramref name="lse"/>.</summary>
    protected bool IsExistingPaveBlock(PaveBlock pb,
                                       SectionCurve nc,
                                       NCollection_List<int> lse,
                                       ref int neOut,
                                       ref double tolNew)
    {
        if (lse.IsEmpty())
        {
            return false;
        }

        double t1, t2, tm, tx = 0.0, tolE, tolCheck, tol, dist = 0.0;
        int nE, iFlag, nV1, nV2;
        Point3D pm = new();
        Box boxPm = new();
        NCollection_List<int>.Iterator itLI = new();
        //
        pb.Range(out t1, out t2);
        pb.Indices(out nV1, out nV2);
        Vertex v1 = ShapeCasts.Vertex(_ds!.Shape(nV1)),
                      v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
        double tolV1 = Tool.Tolerance(v1), tolV2 = Tool.Tolerance(v2);

        tol = Max(tolV1, tolV2);

        tm = IntersectionTools.IntermediatePoint(t1, t2);
        nc.Curve().D0(tm, ref pm);
        boxPm.Add(pm);
        boxPm.Enlarge(tol);
        //
        itLI.Initialize(lse);
        for (; itLI.More(); itLI.Next())
        {
            nE = itLI.Value();
            if (nE < 0)
            {
                continue;
            }
            ShapeInfo sie = _ds.ChangeShapeInfo(nE);
            Box boxE = sie.Box();
            if (!boxE.IsOut(boxPm))
            {
                Edge e = ShapeCasts.Edge(sie.Shape());
                tolE = Tool.Tolerance(e);
                tolCheck = Max(tolE, tol) + _fuzzyValue;
                iFlag = _context!.ComputePE(pm, tolCheck, e, ref tx, ref dist);
                if (iFlag == 0)
                {
                    neOut = nE;
                    tolNew = dist;
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the existing pave block (ON/IN the faces) closest to the pave block of the section curve; returns
    /// it in <paramref name="aPBOut"/> with the new tolerance.
    /// </summary>
    protected bool IsExistingPaveBlock(PaveBlock pb,
                                       SectionCurve nc,
                                       double tolR3D,
                                       IndexedSet<PaveBlock> mpbOnIn,
                                       BoxTree pbTree,
                                       HashSet<PaveBlock> mpbCommon,
                                       ref PaveBlock? aPBOut,
                                       ref double tolNew)
    {
        IntersectionCurve ic = nc.Curve();

        pb.Range(out double t1, out double t2);

        pb.Indices(out int nV11, out int nV12);

        // first point
        Box boxP1 = new();
        Point3D p1 = new();
        ic.D0(t1, ref p1);
        boxP1.Add(p1);
        double tolV11 = Tool.Tolerance(ShapeCasts.Vertex(_ds!.Shape(nV11)));
        boxP1.Enlarge(tolV11);

        // Find edges intersecting by AABB with the first point
        BoxTreeSelector selector = new();
        selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxP1));
        selector.SetBVHSet(pbTree);
        if (selector.Select() == 0)
        {
            return false;
        }

        // intermediate point
        Box boxPm = new();
        double tm = IntersectionTools.IntermediatePoint(t1, t2);
        Curve3D c3d = ic.Curve()!;
        c3d.D1(tm, out Point3D pm, out Vector3D vTgt1);
        boxPm.Add(pm);
        bool isVtgt1Valid = vTgt1.SquareMagnitude > Constants.Resolution();
        if (isVtgt1Valid)
        {
            vTgt1 = vTgt1.Normalized();
        }

        // last point
        Box boxP2 = new();
        Point3D p2 = new();
        ic.D0(t2, ref p2);
        boxP2.Add(p2);
        double tolV12 = Tool.Tolerance(ShapeCasts.Vertex(_ds.Shape(nV12)));
        boxP2.Enlarge(tolV12);

        double tolV1 = Max(tolV11, tolV12) + _fuzzyValue;

        double tolCheck = tolR3D + _fuzzyValue;

        // Some limit values to define "thin" face when iflag1=iflag2=2 and
        // edge has no common block with any face
        double maxTolAdd = 0.001; // Maximal tolerance of edge allowed
        const double coeffTolAdd = 10.0; // Coeff to define max. tolerance with help of aTolCheck
        maxTolAdd = Min(maxTolAdd, coeffTolAdd * tolCheck);

        // Look for the existing pave block closest to the section curve
        bool bFound = false;
        tolNew = RealLast();

        for (NCollection_List<int>.Iterator it = new(selector.Indices()); it.More(); it.Next())
        {
            PaveBlock aPB = mpbOnIn[it.Value() - 1];

            aPB.Indices(out int nV21, out int nV22);

            double tolV21 = Tool.Tolerance(ShapeCasts.Vertex(_ds.Shape(nV21)));
            double tolV22 = Tool.Tolerance(ShapeCasts.Vertex(_ds.Shape(nV22)));
            double tolV2 = Max(tolV21, tolV22) + _fuzzyValue;

            ShapeInfo siSp = _ds.ChangeShapeInfo(aPB.Edge());
            Edge sp = ShapeCasts.Edge(siSp.Shape());
            Box boxSp = siSp.Box();

            int iFlag1 = (nV11 == nV21 || nV11 == nV22) ? 2 : 1;
            int iFlag2 = (nV12 == nV21 || nV12 == nV22) ? 2 : (!boxSp.IsOut(boxP2) ? 1 : 0);
            if (iFlag2 == 0)
            {
                continue;
            }

            double dist = 0.0;
            double coeff = 1.0; // Coeff for taking in account deflections between edge and theNC
            // when aPB is not common block
            double distm1m2 = 0.0;
            int peStatus = 1;

            double realTol = tolCheck;
            if (_ds.IsCommonBlock(aPB))
            {
                realTol = Max(realTol, Max(tolV1, tolV2));
                if (mpbCommon.Contains(aPB))
                {
                    // for an edge, which is a common block with a face,
                    // increase the chance to coincide with section curve
                    realTol *= 2.0;
                }
            }
            else if (iFlag1 == 2 && iFlag2 == 2)
            {
                // Check, if edge could be common block with section curve
                //  and increase the chance to coincide with section curve
                // skip processing if one edge is closed, but other is not closed
                // such configurations can give iFlag1 == 2 && iFlag2 == 2
                bool bSkipProcessing =
                    ((nV11 == nV12) && (nV21 != nV22)) || ((nV11 != nV12) && (nV21 == nV22));

                if (!bSkipProcessing)
                {

                    if (isVtgt1Valid)
                    {
                        EdgeCurveAdaptor bac2 = new(sp);
                        if (ic.Type() != Line || bac2.GetType() != Line)
                        {
                            double tldp = 0.0; // OCCT: uninitialised, read only when ComputePE succeeds
                            double tolAdd =
                                2.0 * Min(maxTolAdd, Max(realTol, Max(tolV1, tolV2)));
                            peStatus = _context!.ComputePE(pm, tolAdd, sp, ref tldp, ref distm1m2);

                            if (peStatus == 0)
                            {
                                bac2.D1(tldp, out Point3D pm2, out Vector3D vTgt2);
                                if (vTgt2.SquareMagnitude > Constants.Resolution())
                                {
                                    // The angle should be close to zero
                                    double cos = vTgt1.Dot(vTgt2.Normalized());
                                    if (Math.Abs(cos) >= 0.9063)
                                    {
                                        realTol = tolAdd;
                                        coeff = 2.0;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            Box boxTmp = boxPm;
            boxTmp.Enlarge(realTol);

            double distToSp = 0.0;
            double tx = 0.0; // OCCT: uninitialised, written by ComputePE and never read
            if (boxSp.IsOut(boxTmp) || peStatus < 0)
            {
                continue;
            }
            else if (peStatus == 0) // aPEStatus == 0 for case iflag1 == iflag2 == 2
            {
                distToSp = distm1m2;
            }
            else if (peStatus == 1) // Projection has not been done yet
            {
                peStatus = _context!.ComputePE(pm, realTol, sp, ref tx, ref distToSp);
                if (peStatus < 0)
                {
                    continue;
                }
            }
            //
            if (iFlag1 == 1)
            {
                iFlag1 = _context!.ComputePE(p1, realTol, sp, ref tx, ref dist) == 0 ? 1 : 0;
                if (iFlag1 != 0 && distToSp < dist)
                {
                    distToSp = dist;
                }
            }
            //
            if (iFlag2 == 1)
            {
                iFlag2 = _context!.ComputePE(p2, realTol, sp, ref tx, ref dist) == 0 ? 1 : 0;
                if (iFlag2 != 0 && distToSp < dist)
                {
                    distToSp = dist;
                }
            }
            //
            if (iFlag1 != 0 && iFlag2 != 0)
            {
                if (distToSp < tolNew)
                {
                    aPBOut = aPB;
                    tolNew = coeff * distToSp;
                    bFound = true;
                }
            }
        }
        return bFound;
    }

    //=================================================================================================

    private static void GetBoundPaves(DS ds, SectionCurve nc, Span<int> nv)
    {
        nv[0] = nv[1] = -1;

        // get extreme paves
        PaveBlock pb = nc.PaveBlocks().First();
        NCollection_List<Pave> lp = pb.ExtPaves();
        int nbEP = lp.Extent();
        if (nbEP == 0)
        {
            return;
        }
        double tmin = RealLast();
        double tmax = -tmin;
        for (NCollection_List<Pave>.Iterator itLP = new(lp); itLP.More(); itLP.Next())
        {
            Pave pv = itLP.Value();
            int nV;
            double tv;
            pv.Contents(out nV, out tv);
            if (tv < tmin)
            {
                nv[0] = pv.Index();
                tmin = tv;
            }
            if (tv > tmax)
            {
                nv[1] = pv.Index();
                tmax = tv;
            }
        }

        // compare extreme vertices with ends of the curve
        IntersectionCurve ic = nc.Curve();
        double t0 = 0.0, t1 = 0.0;
        Point3D[] p = [new Point3D(), new Point3D()];
        ic.Bounds(ref t0, ref t1, ref p[0], ref p[1]);
        double tol = Max(nc.Tolerance(), nc.TangentialTolerance());
        tol += global::Kerf.Engine.Foundation.Precision.Confusion();
        for (int j = 0; j < 2; ++j)
        {
            ShapeInfo siv = ds.ShapeInfo(nv[j]);
            Vertex v = ShapeCasts.Vertex(siv.Shape());
            int iFlag = AlgoTools.ComputeVV(v, p[j], tol);
            if (iFlag != 0)
            {
                nv[j] = -1;
            }
        }
    }

    //=================================================================================================

    protected void PutBoundPaveOnCurve(in Face aF1, in Face aF2, SectionCurve aNC, NCollection_List<int> aLVB)
    {
        IntersectionCurve ic = aNC.Curve();
        Span<double> t = stackalloc double[2];
        Point3D[] p = [new Point3D(), new Point3D()];
        ic.Bounds(ref t[0], ref t[1], ref p[0], ref p[1]);
        double tolR3D = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        PaveBlock pb = aNC.ChangePaveBlock1();
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> bndNV = stackalloc int[2];
        GetBoundPaves(_ds!, aNC, bndNV);
        //
        double tolVnew = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool isClosed = p[1].IsEqual(p[0], tolVnew);
        if (isClosed && (bndNV[0] > 0 || bndNV[1] > 0))
        {
            return;
        }

        for (int j = 0; j < 2; ++j)
        {
            if (bndNV[j] < 0)
            {
                // no vertex on this end
                if (j != 0 && isClosed)
                {
                    // if curve is closed, process only one bound
                    continue;
                }
                bool bVF = _context!.IsValidPointForFaces(p[j], aF1, aF2, tolR3D);
                if (!bVF)
                {
                    continue;
                }
                AlgoTools.MakeNewVertex(p[j], tolR3D, out Vertex vn);
                AlgoTools.UpdateVertex(ic, t[j], vn);
                tolVnew = Tool.Tolerance(vn);

                ShapeInfo siVn = new();
                siVn.SetShapeType(ShapeType.Vertex);
                siVn.SetShape(vn);

                ref Box box = ref siVn.ChangeBox();
                Bounds.Add(vn, ref box);
                box.SetGap(box.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());

                int nVn = _ds!.Append(siVn);

                Pave pn = new();
                pn.SetIndex(nVn);
                pn.SetParameter(t[j]);
                pb.AppendExtPave(pn);

                aLVB.Append(nVn);
            }
        }
    }

    //=================================================================================================

    // A walk that stops short at a near-tangency leaves two curves of one pair of faces a little apart, the end of one with a vertex,
    // and the end of the other with none, as it's a little off a face. That curve then has no pave block and is lost, and the section
    // stays open: OCCT #1659, 1e-3 apart with a tolerance of 3e-4, which makes a Cut remove nothing. Not OCCT's: the end with none
    // takes the near vertex, whose tolerance grows to reach it (decision 1935).
    private const double EndsJoinedByTolerance = 5.0;

    private void JoinCurveEnds(List<SectionCurve> curves)
    {
        if (curves.Count < 2)
        {
            return;
        }

        Span<int> ends = stackalloc int[2];
        Span<int> otherEnds = stackalloc int[2];
        Span<double> t = stackalloc double[2];
        Span<double> otherT = stackalloc double[2];
        Point3D[] p = [new Point3D(), new Point3D()];
        Point3D[] otherP = [new Point3D(), new Point3D()];
        for (int i = 0; i < curves.Count; i++)
        {
            SectionCurve curve = curves[i];
            IntersectionCurve ic = curve.Curve();
            if (!ic.HasBounds())
            {
                continue;
            }

            GetBoundPaves(_ds!, curve, ends);
            if (ends[0] >= 0 && ends[1] >= 0)
            {
                continue;
            }

            ic.Bounds(ref t[0], ref t[1], ref p[0], ref p[1]);
            if (p[0].IsEqual(p[1], global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                continue;
            }

            for (int end = 0; end < 2; end++)
            {
                if (ends[end] >= 0)
                {
                    continue;
                }

                for (int k = 0; k < curves.Count; k++)
                {
                    if (k == i || !curves[k].Curve().HasBounds())
                    {
                        continue;
                    }

                    curves[k].Curve().Bounds(ref otherT[0], ref otherT[1], ref otherP[0], ref otherP[1]);
                    GetBoundPaves(_ds!, curves[k], otherEnds);
                    int nearest = otherP[0].DistanceTo(p[end]) <= otherP[1].DistanceTo(p[end]) ? 0 : 1;
                    double reach = EndsJoinedByTolerance * Max(Max(curve.Tolerance(), curve.TangentialTolerance()), Max(curves[k].Tolerance(), curves[k].TangentialTolerance()));
                    if (otherEnds[nearest] < 0 || otherP[nearest].DistanceTo(p[end]) > reach)
                    {
                        continue;
                    }

                    int nV = otherEnds[nearest];
                    Vertex vertex = ShapeCasts.Vertex(_ds!.Shape(nV));
                    AlgoTools.UpdateVertex(ic, t[end], vertex);
                    ref Box box = ref _ds.ChangeShapeInfo(nV).ChangeBox();
                    Bounds.Add(vertex, ref box);

                    Pave pave = new();
                    pave.SetIndex(nV);
                    pave.SetParameter(t[end]);
                    curve.ChangePaveBlock1().AppendExtPave(pave);
                    break;
                }
            }
        }
    }

    //=================================================================================================

    protected void PutPavesOnCurve(NCollection_IntMap mvOnIn,
                                   NCollection_IntMap mvCommon,
                                   SectionCurve nc,
                                   NCollection_IntMap mi,
                                   NCollection_IntMap mvef,
                                   NCollection_IntDataMap<double> mvTol,
                                   NCollection_IntDataMap<NCollection_List<int>> dmvlv)
    {
        //
        Box boxC = nc.Box();
        double tolR3D = Max(nc.Tolerance(), nc.TangentialTolerance());
        //
        // Put EF vertices first
        // (decision 784: the order of the map decides the order of the extra paves and of the lists of theDMVLV)
        foreach (int nV in mvef)
        {
            PutPaveOnCurve(nV, tolR3D, nc, mi, mvTol, dmvlv, 2);
        }

        // Put all other vertices
        foreach (int nV in mvOnIn)
        {
            if (mvef.Contains(nV))
            {
                continue;
            }

            if (!mvCommon.Contains(nV))
            {
                ShapeInfo siv = _ds!.ShapeInfo(nV);
                Box boxV = siv.Box();
                //
                if (boxC.IsOut(boxV))
                {
                    continue;
                }
                if (!_ds.IsNewShape(nV))
                {
                    continue;
                }
            }
            //
            PutPaveOnCurve(nV, tolR3D, nc, mi, mvTol, dmvlv, 1);
        }
    }

    //=================================================================================================

    private struct PaveBlockDist
    {
        public PaveBlock PB;
        public double SquareDist; // square distance from vertex to the paveblock
        public double SinAngle;   // sinus of angle between projection vector
        // and tangent at projection point
        public double Tolerance; // tolerance of the section curve
    }

    protected void FilterPavesOnCurves(List<SectionCurve> vnc, NCollection_IntDataMap<double> mvTol)
    {
        // For each vertex found in ExtPaves of pave blocks of section curves
        // collect list of pave blocks with distance to the curve
        IndexedMap<int, NCollection_List<PaveBlockDist>> idmVertPBs = new();
        int i;
        double eps = Constants.Resolution();
        for (i = 0; i < vnc.Count; ++i)
        {
            SectionCurve nc = vnc[i];
            IntersectionCurve ic = nc.Curve();
            double tolR3D = Max(nc.Tolerance(), nc.TangentialTolerance());
            CurveAdaptor gac = new(ic.Curve()!);
            PaveBlock pb = nc.PaveBlocks().First();
            NCollection_List<Pave> paves = pb.ExtPaves();
            NCollection_List<Pave>.Iterator itPaves = new(paves);
            for (; itPaves.More(); itPaves.Next())
            {
                Pave pave = itPaves.Value();
                int nV = pave.Index();
                Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
                // compute distance from vertex to the point on curve with vertex parameter
                Point3D pv = Tool.Pnt(v);
                double par = pave.Parameter();
                gac.D1(par, out Point3D ponC, out Vector3D d1);
                Vector3D projVec = new(pv, ponC);
                double sqDist = projVec.SquareMagnitude;
                double sqD1Mod = d1.SquareMagnitude;
                double sin = projVec.CrossSquareMagnitude(d1);
                if (sqDist > eps && sqD1Mod > eps)
                {
                    sin = Math.Sqrt(sin / sqDist / sqD1Mod);
                }
                int index = idmVertPBs.IndexOf(nV) + 1;
                NCollection_List<PaveBlockDist> pList = index > 0
                    ? idmVertPBs.ValueAt(index - 1)
                    : idmVertPBs.ValueAt(idmVertPBs.Add(nV, new NCollection_List<PaveBlockDist>()));
                PaveBlockDist pbd = new() { PB = pb, SquareDist = sqDist, SinAngle = sin, Tolerance = tolR3D };
                pList.Append(pbd);
            }
        }

        // Process each vertex
        const double sinAngleMin = 0.5; // angle below which projection is suspicious
        for (i = 1; i <= idmVertPBs.Count; i++)
        {
            int nV = idmVertPBs.KeyAt(i - 1);
            NCollection_List<PaveBlockDist> list = idmVertPBs.ValueAt(i - 1);
            // Find a pave with minimal distance
            double minDist = RealLast();
            PaveBlock? pbMinDist = null;
            NCollection_List<PaveBlockDist>.Iterator itL = new(list);
            for (; itL.More(); itL.Next())
            {
                PaveBlockDist pbd = itL.Value();
                if (pbd.SquareDist < minDist)
                {
                    minDist = pbd.SquareDist;
                    pbMinDist = pbd.PB;
                }
            }
            _ = pbMinDist; // OCCT finds it and never reads it
            // Remove a vertex from a pave block if the distance is greater than the tolerance
            // and there are other pave blocks for which the distance is less than the current.
            // Do not remove a vertex if it is projected on the curve with quite large angle
            // (see test bugs modalg_6 bug27761).

            // Reduce tolerance for the vertex to the value of maximal distance to
            // to section curve on which it will be kept.
            double maxDistKept = -1;
            bool isRemoved = false;
            for (itL.Initialize(list); itL.More(); itL.Next())
            {
                PaveBlockDist pbd = itL.Value();
                double checkDist = 100.0 * Max(pbd.Tolerance * pbd.Tolerance, minDist);
                if (pbd.SquareDist > checkDist && pbd.SinAngle < sinAngleMin)
                {
                    pbd.PB.RemoveExtPave(nV);
                    isRemoved = true;
                }
                else if (pbd.SquareDist > maxDistKept)
                {
                    maxDistKept = pbd.SquareDist;
                }
            }

            if (isRemoved && maxDistKept > 0)
            {
                if (mvTol.Find(nV, out double pTol))
                {
                    Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
                    double realTol = Max(pTol, Math.Sqrt(maxDistKept) + global::Kerf.Engine.Foundation.Precision.Confusion());
                    // OCCT writes the tolerance into the vertex's TShape directly (a frozen vertex throws).
                    // Kerf: an identical write is skipped, as in MakeBlocks (unit 6G).
                    if (((BrepTVertex)v.TShape()!).Tolerance() != realTol)
                    {
                        ((BrepTVertex)v.TShape()!).Tolerance(realTol);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected bool ExtendedTolerance(int nV, NCollection_IntMap aMI, ref double aTolVExt, int aType = 0)
    {
        bool bFound = false;
        if (!(_ds!.IsNewShape(nV)))
        {
            return bFound;
        }
        //
        int i, k, nbLines, nbInt;
        double t11, t12, d1, d2, d;
        Vertex v;
        Point3D pv, p11, p12;
        //
        k = 0;
        nbInt = 2;
        if (aType == 1)
        {
            nbInt = 1;
        }
        else if (aType == 2)
        {
            k = 1;
        }
        //
        v = ShapeCasts.Vertex(_ds.Shape(nV));
        pv = Tool.Pnt(v);
        //
        List<InterfEE> eEs = _ds.InterfEE();
        List<InterfEF> eFs = _ds.InterfEF();
        //
        for (; k < nbInt; ++k)
        {
            nbLines = k == 0 ? eEs.Count : eFs.Count;
            for (i = 0; i < nbLines; ++i)
            {
                Interf aInt = k == 0 ? eEs[i] : eFs[i];
                if (aInt.IndexNew() == nV)
                {
                    if (aMI.Contains(aInt.Index1()) && aMI.Contains(aInt.Index2()))
                    {
                        CommonPrt comPrt = k == 0 ? eEs[i].CommonPart() : eFs[i].CommonPart();
                        //
                        Edge e1 = comPrt.Edge1();
                        comPrt.Range1(out t11, out t12);
                        AlgoTools.PointOnEdge(e1, t11, out p11);
                        AlgoTools.PointOnEdge(e1, t12, out p12);
                        d1 = pv.DistanceTo(p11);
                        d2 = pv.DistanceTo(p12);
                        d = (d1 > d2) ? d1 : d2;
                        if (d > aTolVExt)
                        {
                            aTolVExt = d;
                        }
                        return !bFound;
                    } // if (aMI.Contains(aEF.Index1()) && aMI.Contains(aEF.Index2())) {
                } // if (aInt->IndexNew() == nV) {
            } // for (i = 0; i < aNbLines; ++i) {
        } // for (k=0; k<2; ++k) {
        return bFound;
    }

    //=================================================================================================

    protected void GetEFPnts(int nF1, int nF2, NCollection_List<PntOn2S> aListOfPnts)
    {
        int nE, nF, nFOpposite, nbEFs, i;
        double u1, u2, v1, v2, f, l;
        NCollection_IntMap mi = new();
        //
        // collect indexes of all shapes from nF1 and nF2.
        GetFullShapeMap(nF1, mi);
        GetFullShapeMap(nF2, mi);
        //
        List<InterfEF> eFs = _ds!.InterfEF();
        nbEFs = eFs.Count;
        //
        for (i = 0; i < nbEFs; ++i)
        {
            InterfEF ef = eFs[i];
            if (ef.HasIndexNew())
            {
                ef.Indices(out nE, out nFOpposite);
                if (mi.Contains(nE) && mi.Contains(nFOpposite))
                {
                    CommonPrt cp = ef.CommonPart();
                    double par = cp.VertexParameter1();
                    Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                    Face fOpposite = ShapeCasts.Face(_ds.Shape(nFOpposite));
                    //
                    Curve3D curve = Tool.Curve(e, out f, out l)!;
                    //
                    nF = (nFOpposite == nF1) ? nF2 : nF1;
                    Face aF = ShapeCasts.Face(_ds.Shape(nF));
                    Curve2D? pCurve = Tool.CurveOnSurface(e, aF, out f, out l);
                    //
                    ProjectPointOnSurface proj = _context!.ProjPS(fOpposite);
                    //
                    curve.D0(par, out Point3D point);
                    PntOn2S pnt = new();
                    if (pCurve is not null)
                    {
                        Point2D p2d = pCurve.Value(par);
                        proj.Perform(point);
                        if (proj.IsDone)
                        {
                            proj.LowerDistanceParameters(out u1, out v1);
                            if (nF == nF1)
                            {
                                pnt.SetValue(p2d.X, p2d.Y, u1, v1);
                            }
                            else
                            {
                                pnt.SetValue(u1, v1, p2d.X, p2d.Y);
                            }
                            aListOfPnts.Append(pnt);
                        }
                    }
                    else
                    {
                        ProjectPointOnSurface proj1 = _context.ProjPS(aF);
                        proj1.Perform(point);
                        proj.Perform(point);
                        if (proj1.IsDone && proj.IsDone)
                        {
                            proj1.LowerDistanceParameters(out u1, out v1);
                            proj.LowerDistanceParameters(out u2, out v2);
                            if (nF == nF1)
                            {
                                pnt.SetValue(u1, v1, u2, v2);
                            }
                            else
                            {
                                pnt.SetValue(u2, v2, u1, v1);
                            }
                            aListOfPnts.Append(pnt);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void PutEFPavesOnCurve(List<SectionCurve> vc,
                                     int index,
                                     NCollection_IntMap aMI,
                                     NCollection_IntMap aMVEF,
                                     NCollection_IntDataMap<double> aMVTol,
                                     NCollection_IntDataMap<NCollection_List<int>> aDMVLV)
    {
        if (aMVEF.Extent() == 0)
        {
            return;
        }
        //
        SectionCurve nc = vc[index];
        IntersectionCurve ic = nc.Curve();
        CurveKind typeC;
        typeC = ic.Type();
        if (typeC != BezierCurve && typeC != BSplineCurve)
        {
            return;
        }
        //
        NCollection_IntMap mv = new(aMVEF);
        //
        RemoveUsedVertices(vc, mv);
        if (mv.Extent() == 0)
        {
            return;
        }
        //
        double dist;
        //
        Curve3D c3D = ic.Curve()!;
        ProjectPointOnCurve projPT = _context!.ProjPT(c3D);
        //
        // (decision 784: the order of the map decides the order of the extra paves)
        foreach (int nV in mv)
        {
            Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
            Point3D pv = Tool.Pnt(v);
            projPT.Perform(pv);
            int nbPoints = projPT.NbPoints();
            if (nbPoints != 0)
            {
                dist = projPT.LowerDistance();
                PutPaveOnCurve(nV, dist, nc, aMI, aMVTol, aDMVLV);
            }
        }
    }

    //=================================================================================================

    protected void PutStickPavesOnCurve(in Face aF1,
                                        in Face aF2,
                                        NCollection_IntMap aMI,
                                        List<SectionCurve> vc,
                                        int index,
                                        NCollection_IntMap aMVStick,
                                        NCollection_IntDataMap<double> aMVTol,
                                        NCollection_IntDataMap<NCollection_List<int>> aDMVLV)
    {
        SectionCurve nc = vc[index];
        // Get numbers of vertices assigned to the ends of the curve
        Span<int> bndNV = stackalloc int[2];
        GetBoundPaves(_ds!, nc, bndNV);
        if (bndNV[0] >= 0 && bndNV[1] >= 0)
        {
            // both curve ends already have assigned vertices
            return;
        }
        NCollection_IntMap mv = new(aMVStick);
        RemoveUsedVertices(vc, mv);
        //
        if (mv.Extent() == 0)
        {
            return;
        }
        //
        Surface s1 = Tool.Surface(aF1)!;
        Surface s2 = Tool.Surface(aF2)!;
        //
        IntersectionCurve ic = nc.Curve();
        Curve2D?[] c2D = new Curve2D?[2];
        //
        c2D[0] = ic.FirstCurve2d();
        c2D[1] = ic.SecondCurve2d();
        if (c2D[0] is not null && c2D[1] is not null)
        {
            int m, n;
            double d, d2, u, v, dt2, scPr, dScPr;
            Span<double> tc = stackalloc double[2];
            Point3D[] pc = [new Point3D(), new Point3D()];
            Point3D pv;
            Direction3D[] dn = [new Direction3D(), new Direction3D()];
            Point2D p2D;
            //
            dt2 = 2e-7;  // the rich criteria
            dScPr = 5.0e-9; // the creasing criteria
            ic.Bounds(ref tc[0], ref tc[1], ref pc[0], ref pc[1]);
            //
            // (decision 784: the order of the map decides which vertex is put first)
            foreach (int nV in mv)
            {
                Vertex aV = ShapeCasts.Vertex(_ds!.Shape(nV));
                pv = Tool.Pnt(aV);
                //
                for (m = 0; m < 2; ++m)
                {
                    if (bndNV[m] >= 0)
                    {
                        continue;
                    }
                    d2 = pc[m].SquareDistanceTo(pv);
                    if (d2 > dt2)
                    { // no rich
                        continue;
                    }
                    //
                    for (n = 0; n < 2; ++n)
                    {
                        Surface s = (n == 0) ? s1 : s2;
                        c2D[n]!.D0(tc[m], out p2D);
                        p2D.Coord(out u, out v);
                        AlgoTools3D.GetNormalToSurface(s, u, v, ref dn[n]);
                    }
                    //
                    scPr = dn[0] * dn[1];
                    if (scPr < 0.0)
                    {
                        scPr = -scPr;
                    }
                    scPr = 1.0 - scPr;
                    //
                    if (scPr > dScPr)
                    {
                        continue;
                    }
                    //
                    // The intersection curve aIC is vanishing curve (the crease)
                    d = Math.Sqrt(d2);
                    //
                    PutPaveOnCurve(nV, d, nc, aMI, aMVTol, aDMVLV);
                }
            } // for (jVU=1; jVU=aNbVU; ++jVU) {
        }
    }

    //=================================================================================================

    private const int StickTypeShift = 28;
    private readonly Dictionary<int, List<int>> _stickIndex = [];
    private readonly List<int> _stickCandidates = [];
    private readonly int[] _stickIndexed = new int[5];
    private DS? _stickIndexOf;

    // The interference of a kind (VV, VE, EE, VF, EF) and position
    private Interf StickInterf(int typeInt, int i) => typeInt switch
    {
        0 => _ds!.InterfVV()[i],
        1 => _ds!.InterfVE()[i],
        2 => _ds!.InterfEE()[i],
        3 => _ds!.InterfVF()[i],
        _ => _ds!.InterfEF()[i],
    };

    // Brings the index of interferences by their first shape up to date: the lists only grow during a run, and a shape's
    // indices don't change. Whether one has a new vertex can, so that's read each time
    private void UpdateStickIndex()
    {
        // A run starts with a new data structure
        if (!ReferenceEquals(_stickIndexOf, _ds))
        {
            _stickIndexOf = _ds;
            _stickIndex.Clear();
            Array.Clear(_stickIndexed);
        }

        Span<int> counts = [_ds!.InterfVV().Count, _ds.InterfVE().Count, _ds.InterfEE().Count, _ds.InterfVF().Count, _ds.InterfEF().Count];

        for (int typeInt = 0; typeInt < counts.Length; typeInt++)
        {
            for (int i = _stickIndexed[typeInt]; i < counts[typeInt]; i++)
            {
                StickInterf(typeInt, i).Indices(out int nS1, out _);
                if (!_stickIndex.TryGetValue(nS1, out List<int>? starting))
                {
                    starting = [];
                    _stickIndex.Add(nS1, starting);
                }

                starting.Add((typeInt << StickTypeShift) | i);
            }

            _stickIndexed[typeInt] = counts[typeInt];
        }
    }

    protected void GetStickVertices(int nF1, int nF2, NCollection_IntMap aMVStick, NCollection_IntMap aMVEF, NCollection_IntMap aMI)
    {
        // collect indices of all shapes from nF1 and nF2.
        aMI.Clear();
        GetFullShapeMap(nF1, aMI);
        GetFullShapeMap(nF2, aMI);
        //
        // Only the interferences that start from one of those shapes can pass, and with a model's thousands of interferences and
        // pairs of faces it's worth finding them directly. They're taken in the order of a scan of the five lists, which orders the maps
        UpdateStickIndex();
        _stickCandidates.Clear();
        foreach (int nS in aMI)
        {
            if (_stickIndex.TryGetValue(nS, out List<int>? starting))
            {
                _stickCandidates.AddRange(starting);
            }
        }

        _stickCandidates.Sort();
        foreach (int candidate in _stickCandidates)
        {
            int typeInt = candidate >> StickTypeShift;
            Interf aInt = StickInterf(typeInt, candidate & ((1 << StickTypeShift) - 1));
            if (aInt.HasIndexNew())
            {
                aInt.Indices(out int nS1, out int nS2);
                if (aMI.Contains(nS1) && aMI.Contains(nS2))
                {
                    int nVNew = aInt.IndexNew();
                    _ds!.HasShapeSD(nVNew, ref nVNew);
                    aMVStick.Add(nVNew);
                    if (typeInt == 4)
                    {
                        // collect EF interferences
                        aMVEF.Add(nVNew);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void GetFullShapeMap(int nF, NCollection_IntMap aMI)
    {
        NCollection_List<int>.Iterator it = new();
        int nS;
        //
        ShapeInfo si = _ds!.ShapeInfo(nF);
        NCollection_List<int> li = si.SubShapes();
        //
        aMI.Add(nF);
        it.Initialize(li);
        for (; it.More(); it.Next())
        {
            nS = it.Value();
            aMI.Add(nS);
        }
    }

    //=================================================================================================

    protected void RemoveUsedVertices(List<SectionCurve> aVC, NCollection_IntMap aMV)
    {
        if (aMV.IsEmpty())
        {
            return;
        }

        for (int i = 0; i < aVC.Count; ++i)
        {
            SectionCurve nc = aVC[i];
            NCollection_List<PaveBlock> lpbc = nc.PaveBlocks();
            NCollection_List<PaveBlock>.Iterator itPB = new(lpbc);
            for (; itPB.More(); itPB.Next())
            {
                PaveBlock pb = itPB.Value();
                NCollection_List<Pave> lp = pb.ExtPaves();
                NCollection_List<Pave>.Iterator itLP = new(lp);
                for (; itLP.More(); itLP.Next())
                {
                    aMV.Remove(itLP.Value().Index());
                }

                aMV.Remove(pb.Pave1().Index());
                aMV.Remove(pb.Pave2().Index());
            }
        }
    }

    //=================================================================================================

    protected void PutPaveOnCurve(int nV,
                                  double aTolR3D,
                                  SectionCurve aNC,
                                  NCollection_IntMap aMI,
                                  NCollection_IntDataMap<double> aMVTol,
                                  NCollection_IntDataMap<NCollection_List<int>> aDMVLV,
                                  int iCheckExtend = 0)
    {
        bool bIsVertexOnLine;
        double t = 0.0; // OCCT: uninitialised, written by IsVertexOnLine when it returns true
        //
        Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
        PaveBlock pb = aNC.PaveBlocks().First();
        IntersectionCurve ic = aNC.Curve();
        //
        double tolV = (aMVTol.IsBound(nV) ? aMVTol.Find(nV) : Tool.Tolerance(v));

        bIsVertexOnLine = _context!.IsVertexOnLine(v, tolV, ic, aTolR3D + _fuzzyValue, ref t);
        if (!bIsVertexOnLine && iCheckExtend != 0 && !_vertsToAvoidExtension.Contains(nV))
        {
            double extraTol = tolV;
            if (ExtendedTolerance(nV, aMI, ref extraTol, iCheckExtend))
            {
                bIsVertexOnLine = _context.IsVertexOnLine(v, extraTol, ic, aTolR3D + _fuzzyValue, ref t);
                if (bIsVertexOnLine)
                {
                    Point3D pOnC = new();
                    ic.D0(t, ref pOnC);
                    tolV = pOnC.DistanceTo(Tool.Pnt(v));
                }
            }
        }
        //
        if (bIsVertexOnLine)
        {
            // check if aPB contains the parameter aT
            bool bExist;
            int nVUsed = 0; // OCCT: uninitialised, written by ContainsParameter when it returns true
            double pTol, dTol;
            //
            dTol = AlgoTools.DTolerance();
            //
            CurveAdaptor gac = new(ic.Curve()!);
            pTol = gac.Resolution(Max(aTolR3D, tolV));
            //
            bExist = pb.ContainsParameter(t, pTol, ref nVUsed);
            if (bExist)
            {
                // use existing pave
                if (!aDMVLV.Find(nVUsed, out NCollection_List<int> pList))
                {
                    pList = aDMVLV.Bound(nVUsed, new NCollection_List<int>());
                    pList.Append(nVUsed);
                    if (!aMVTol.IsBound(nVUsed))
                    {
                        Vertex vUsed = ShapeCasts.Vertex(_ds.Shape(nVUsed));
                        tolV = Tool.Tolerance(vUsed);
                        aMVTol.Bind(nVUsed, tolV);
                    }
                }
                // avoid repeated elements in the list
                NCollection_List<int>.Iterator itLI = new(pList);
                for (; itLI.More(); itLI.Next())
                {
                    if (itLI.Value() == nV)
                    {
                        break;
                    }
                }
                if (!itLI.More())
                {
                    pList.Append(nV);
                }
                // save initial tolerance for the vertex
                if (!aMVTol.IsBound(nV))
                {
                    tolV = Tool.Tolerance(v);
                    aMVTol.Bind(nV, tolV);
                }
            }
            else
            {
                // add new pave
                Pave pave = new();
                pave.SetIndex(nV);
                pave.SetParameter(t);
                pb.AppendExtPave(pave);
                //
                Point3D p1 = gac.Value(t);
                tolV = Tool.Tolerance(v);
                Point3D p2 = Tool.Pnt(v);
                double dist = p1.DistanceTo(p2);
                if (tolV < dist + dTol)
                {
                    // OCCT writes the vertex in place, also an argument vertex shared by both faces (theMVCommon of
                    // PutPavesOnCurve) in non-destructive mode: a frozen vertex throws here.
                    new BrepBuilder().UpdateVertex(v, dist + dTol);
                    //
                    if (!aMVTol.IsBound(nV))
                    {
                        aMVTol.Bind(nV, tolV);
                    }
                    //
                    ShapeInfo sids = _ds.ChangeShapeInfo(nV);
                    ref Box boxDS = ref sids.ChangeBox();
                    Bounds.Add(v, ref boxDS);
                    boxDS.SetGap(boxDS.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int cur,
                                             int nF1,
                                             int nF2,
                                             in Edge es,
                                             IndexedSet<PaveBlock> mpbOnIn,
                                             BoxTree pbTree,
                                             IndexedMap<Shape, CoupleOfPaveBlocks> mscpb,
                                             Dictionary<Shape, int> mvi,
                                             NCollection_List<PaveBlock> lpbc,
                                             Dictionary<PaveBlock, NCollection_List<int>> pbFacesMap,
                                             HashSet<PaveBlock> mpb)
    {
        Box boxES = new();
        Bounds.Add(es, ref boxES, false);

        BoxTreeSelector selector = new();
        selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxES));
        selector.SetBVHSet(pbTree);
        if (selector.Select() == 0)
        {
            return;
        }

        double tolES = Tool.Tolerance(es);

        FaceInfo fi1 = _ds!.FaceInfo(nF1);
        FaceInfo fi2 = _ds.FaceInfo(nF2);

        for (NCollection_List<int>.Iterator itPB = new(selector.Indices()); itPB.More(); itPB.Next())
        {
            PaveBlock pbf = mpbOnIn[itPB.Value() - 1];
            if (mpb.Contains(pbf))
            {
                continue;
            }

            bool bInF1 = (fi1.PaveBlocksOn().Contains(pbf) || fi1.PaveBlocksIn().Contains(pbf));
            bool bInF2 = (fi2.PaveBlocksOn().Contains(pbf) || fi2.PaveBlocksIn().Contains(pbf));
            if (bInF1 && bInF2)
            {
                // Add all common edges for post treatment
                mpb.Add(pbf);
                PreparePostTreatFF(theInt, cur, pbf, mscpb, mvi, lpbc);
                continue;
            }

            int nF = bInF1 ? nF2 : nF1;
            if (!_distances.TryGetValue(new Pair(pbf.OriginalEdge(), nF), out NCollection_List<EdgeRangeDistance>? pList))
            {
                continue;
            }

            pbf.Range(out double t1, out double t2);

            double dist = RealLast();
            for (NCollection_List<EdgeRangeDistance>.Iterator itR = new(pList); itR.More(); itR.Next())
            {
                EdgeRangeDistance rangeDist = itR.Value();
                if ((t1 <= rangeDist.First && rangeDist.First <= t2)
                    || (t1 <= rangeDist.Last && rangeDist.Last <= t2)
                    || (rangeDist.First <= t1 && t1 <= rangeDist.Last)
                    || (rangeDist.First <= t2 && t2 <= rangeDist.Last))
                {
                    dist = rangeDist.Distance;
                    break;
                }
            }
            if (dist < RealLast())
            {
                Edge ef = ShapeCasts.Edge(_ds.Shape(pbf.Edge()));
                double tolSum = tolES + Tool.Tolerance(ef);

                if (dist <= tolSum)
                {
                    mpb.Add(pbf);
                    PreparePostTreatFF(theInt, cur, pbf, mscpb, mvi, lpbc);

                    if (!pbFacesMap.TryGetValue(pbf, out NCollection_List<int>? pFaces))
                    {
                        pFaces = pbFacesMap.SetValueRef(pbf, new NCollection_List<int>());
                    }
                    if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                    {
                        pFaces.Append(nF);
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void ProcessExistingPaveBlocks(int theInt,
                                             int nF1,
                                             int nF2,
                                             IndexedSet<PaveBlock> aMPBOnIn,
                                             BoxTree pbTree,
                                             NCollection_IntDataMap<NCollection_List<int>> aDMBV,
                                             IndexedMap<Shape, CoupleOfPaveBlocks> aMSCPB,
                                             Dictionary<Shape, int> aMVI,
                                             Dictionary<PaveBlock, NCollection_List<int>> pbFacesMap,
                                             HashSet<PaveBlock> aMPB)
    {
        if (aDMBV.IsEmpty())
        {
            return;
        }
        //
        double t = 0.0, dummy = 0.0;
        int nV, nE, iC, iFlag;
        NCollection_List<int>.Iterator itLI = new();
        //
        List<InterfFF> fFs = _ds!.InterfFF();
        InterfFF ff = fFs[theInt];
        List<SectionCurve> vc = ff.ChangeCurves();
        //
        FaceInfo fi1 = _ds.FaceInfo(nF1);
        FaceInfo fi2 = _ds.FaceInfo(nF2);
        //
        // (decision 784: OCCT iterates the DataMap<int, ...> in bucket order; the order of the curves decides the
        // order of the post-treated pave blocks)
        foreach (KeyValuePair<int, NCollection_List<int>> itBV in aDMBV)
        {
            iC = itBV.Key;
            NCollection_List<int> lbv = itBV.Value;
            //
            SectionCurve nc = vc[iC];
            NCollection_List<PaveBlock> lpbc = nc.ChangePaveBlocks();
            //
            itLI.Initialize(lbv);
            for (; itLI.More(); itLI.Next())
            {
                nV = itLI.Value();
                ShapeInfo siv = _ds.ShapeInfo(nV);
                Box boxV = siv.Box();
                Vertex v = ShapeCasts.Vertex(siv.Shape());
                if (!aMVI.ContainsKey(v))
                {
                    continue;
                }
                //
                BoxTreeSelector selector = new();
                selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxV));
                selector.SetBVHSet(pbTree);
                if (selector.Select() == 0)
                {
                    continue;
                }

                for (NCollection_List<int>.Iterator it = new(selector.Indices()); it.More(); it.Next())
                {
                    PaveBlock pb = aMPBOnIn[it.Value() - 1];
                    if (pb.Pave1().Index() == nV || pb.Pave2().Index() == nV)
                    {
                        continue;
                    }
                    //
                    if (aMPB.Contains(pb))
                    {
                        continue;
                    }
                    nE = pb.Edge();
                    ShapeInfo sie = _ds.ShapeInfo(nE);
                    Edge e = ShapeCasts.Edge(sie.Shape());
                    //
                    iFlag = _context!.ComputeVE(v, e, ref t, ref dummy, _fuzzyValue);
                    if (iFlag == 0)
                    {
                        aMPB.Add(pb);
                        PreparePostTreatFF(theInt, iC, pb, aMSCPB, aMVI, lpbc);

                        // Add faces to PB
                        bool bInF1 = (fi1.PaveBlocksOn().Contains(pb) || fi1.PaveBlocksIn().Contains(pb));
                        bool bInF2 = (fi2.PaveBlocksOn().Contains(pb) || fi2.PaveBlocksIn().Contains(pb));
                        if (!bInF1 || !bInF2)
                        {
                            // Face without pave block
                            int nF = bInF1 ? nF2 : nF1;
                            if (!pbFacesMap.TryGetValue(pb, out NCollection_List<int>? pFaces))
                            {
                                pFaces = pbFacesMap.SetValueRef(pb, new NCollection_List<int>());
                            }
                            // List is expected to be short, so we allow the check here
                            if (pFaces.IsEmpty() || !pFaces.Contains(nF))
                            {
                                pFaces.Append(nF);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    protected void UpdateExistingPaveBlocks(PaveBlock aPBf,
                                            NCollection_List<PaveBlock> aLPB,
                                            Dictionary<PaveBlock, NCollection_List<int>> pbFacesMap)
    {
        if (aLPB.Extent() == 0)
        {
            return;
        }
        //
        int nE;
        bool bCB;
        PaveBlock pb, pb1, pb2, pb2n;
        CommonBlock? cb;
        NCollection_List<PaveBlock>.Iterator it = new(), it1 = new(), it2 = new();
        //
        // 1. Remove old pave blocks
        CommonBlock? cb1 = _ds!.CommonBlock(aPBf);
        bCB = cb1 is not null;
        NCollection_List<PaveBlock> lpb1 = new();
        //
        if (bCB)
        {
            lpb1.Assign(cb1!.PaveBlocks());
        }
        else
        {
            lpb1.Append(aPBf);
        }
        it1.Initialize(lpb1);
        for (; it1.More(); it1.Next())
        {
            pb1 = it1.Value();
            nE = pb1.OriginalEdge();
            //
            NCollection_List<PaveBlock> lpb2 = _ds.ChangePaveBlocks(nE);
            it2.Initialize(lpb2);
            for (; it2.More(); it2.Next())
            {
                pb2 = it2.Value();
                if (pb1 == pb2)
                {
                    lpb2.Remove(it2);
                    break;
                }
            }
        }
        //
        // 2. Update pave blocks
        if (bCB)
        {
            // Create new common blocks
            NCollection_List<PaveBlock> lpbNew = new();
            NCollection_List<int> faces = cb1!.Faces();
            it.Initialize(aLPB);
            for (; it.More(); it.Next())
            {
                PaveBlock pbValue = it.Value();
                Pave pbValuePave0 = pbValue.Pave1(), pbValuePave1 = pbValue.Pave2();
                //
                cb = new CommonBlock();
                it1.Initialize(lpb1);
                for (; it1.More(); it1.Next())
                {
                    pb2 = it1.Value();
                    nE = pb2.OriginalEdge();
                    //
                    // Create new pave block
                    pb2n = new PaveBlock();
                    if (pbValue.OriginalEdge() == nE)
                    {
                        pb2n.SetPave1(pbValuePave0);
                        pb2n.SetPave2(pbValuePave1);
                    }
                    else
                    {
                        // For the different original edge compute the parameters of paves
                        Pave[] pave = [new Pave(), new Pave()];

                        if (pbValuePave0.Index() == pbValuePave1.Index()
                            && pb2.Pave1().Index() == pb2.Pave2().Index())
                        {
                            // still closed
                            pave[0].SetIndex(pbValuePave0.Index());
                            pave[0].SetParameter(pb2.Pave1().Parameter());
                            pave[1].SetIndex(pbValuePave1.Index());
                            pave[1].SetParameter(pb2.Pave2().Parameter());
                        }
                        else
                        {
                            for (int i = 0; i < 2; ++i)
                            {
                                int nV = (i == 0 ? pbValuePave0 : pbValuePave1).Index();
                                pave[i].SetIndex(nV);
                                if (nV == pb2.Pave1().Index())
                                {
                                    pave[i].SetParameter(pb2.Pave1().Parameter());
                                }
                                else if (nV == pb2.Pave2().Index())
                                {
                                    pave[i].SetParameter(pb2.Pave2().Parameter());
                                }
                                else
                                {
                                    // Compute the parameter by projecting the point
                                    Vertex v = ShapeCasts.Vertex(_ds.Shape(nV));
                                    Edge eOr = ShapeCasts.Edge(_ds.Shape(nE));
                                    double tOut = 0.0, dist = 0.0;
                                    int iErr = _context!.ComputeVE(v, eOr, ref tOut, ref dist, _fuzzyValue);
                                    if (iErr == 0)
                                    {
                                        pave[i].SetParameter(tOut);
                                    }
                                    else
                                    {
                                        // Unable to project - set the parameter of the closest boundary
                                        Vertex v1 = ShapeCasts.Vertex(_ds.Shape(pb2.Pave1().Index()));
                                        Vertex v2 = ShapeCasts.Vertex(_ds.Shape(pb2.Pave2().Index()));
                                        //
                                        Point3D p = Tool.Pnt(v);
                                        Point3D p1 = Tool.Pnt(v1);
                                        Point3D p2 = Tool.Pnt(v2);
                                        //
                                        double dist1 = p.SquareDistanceTo(p1);
                                        double dist2 = p.SquareDistanceTo(p2);
                                        //
                                        pave[i].SetParameter(dist1 < dist2 ? pb2.Pave1().Parameter()
                                                                              : pb2.Pave2().Parameter());
                                    }
                                }
                            }
                        }
                        //
                        if (pave[1].Parameter() < pave[0].Parameter())
                        {
                            Pave paveTmp = pave[0];
                            pave[0] = pave[1];
                            pave[1] = paveTmp;
                        }
                        //
                        pb2n.SetPave1(pave[0]);
                        pb2n.SetPave2(pave[1]);
                    }
                    //
                    pb2n.SetEdge(pbValue.Edge());
                    pb2n.SetOriginalEdge(nE);
                    cb.AddPaveBlock(pb2n);
                    _ds.SetCommonBlock(pb2n, cb);
                    _ds.ChangePaveBlocks(nE).Append(pb2n);
                }
                cb.SetFaces(faces);
                //
                PaveBlock pbNew = cb.PaveBlocks().First();
                lpbNew.Append(pbNew);
            }
            //
            aLPB.Assign(lpbNew);
        }
        else
        {
            nE = aPBf.OriginalEdge();
            NCollection_List<PaveBlock> lpbe = _ds.ChangePaveBlocks(nE);
            it.Initialize(aLPB);
            for (; it.More(); it.Next())
            {
                pb = it.Value();
                lpbe.Append(pb);
            }
        }

        // Try to project the edge on the faces
        if (!pbFacesMap.TryGetValue(aPBf, out NCollection_List<int>? pLFaces))
        {
            return;
        }
        NCollection_List<int>.Iterator itLF = new(pLFaces);
        for (; itLF.More(); itLF.Next())
        {
            int nF = itLF.Value();
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            Face f = ShapeCasts.Face(_ds.Shape(nF));

            it.Initialize(aLPB);
            for (; it.More(); it.Next())
            {
                pb = it.Value();
                if (fi.PaveBlocksOn().Contains(pb) || fi.PaveBlocksIn().Contains(pb))
                {
                    continue;
                }

                Edge e = ShapeCasts.Edge(_ds.Shape(pb.Edge()));
                //
                EdgeFace ef = new();
                ef.SetEdge(e);
                ef.SetFace(f);
                ef.SetFuzzyValue(_fuzzyValue);
                ef.SetRange(pb.Pave1().Parameter(), pb.Pave2().Parameter());
                ef.SetContext(_context);
                ef.Perform();
                //
                NCollection_Sequence<CommonPrt> cPrts = ef.CommonParts();
                bool bCoincide = (cPrts.Length() == 1 && cPrts.Value(1).Type() == ShapeType.Edge);
                if (bCoincide)
                {
                    cb = _ds.CommonBlock(pb);
                    if (cb is null)
                    {
                        cb = new CommonBlock();
                        cb.AddPaveBlock(pb);
                        _ds.SetCommonBlock(pb, cb);
                    }
                    cb.AddFace(nF);
                    fi.ChangePaveBlocksIn().Add(pb);
                }
            }
        }
    }

    //=================================================================================================

    protected void PutClosingPaveOnCurve(SectionCurve aNC)
    {
        IntersectionCurve ic = aNC.Curve();
        Curve3D? c3D = ic.Curve();
        // check 3d curve
        if (c3D is null)
        {
            return;
        }

        // check bounds
        if (!ic.HasBounds())
        {
            return;
        }

        // check closeness
        Span<double> t = stackalloc double[2];
        Point3D[] p = [new Point3D(), new Point3D()];
        ic.Bounds(ref t[0], ref t[1], ref p[0], ref p[1]);

        // Find the pave which has been put at one of the ends
        Pave pave = new();
        // Index of the vertex put at one of the ends
        int nV = -1;
        // Keep the opposite parameter
        double tOp = 0.0;
        // Keep the opposite bounding point
        Point3D pOp = new();

        PaveBlock pb = aNC.ChangePaveBlock1();
        NCollection_List<Pave> lp = pb.ChangeExtPaves();
        NCollection_List<Pave>.Iterator itLP = new(lp);
        for (; itLP.More() && (nV < 0); itLP.Next())
        {
            pave = itLP.Value();
            double tc = pave.Parameter();
            for (int j = 0; j < 2; ++j)
            {
                if (Math.Abs(tc - t[j]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    nV = pave.Index();
                    tOp = (j == 0) ? t[1] : t[0];
                    pOp = (j == 0) ? p[1] : p[0];
                    break;
                }
            }
        }

        if (nV < 0)
        {
            // No paves on the bounds of the curve
            return;
        }

        // Check if the curve is closed using the tolerance
        // of found vertex
        Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
        double tolV = Tool.Tolerance(v);
        Point3D pv = Tool.Pnt(v);
        // Tolerance for the point on the curve
        double tolP = Max(aNC.Tolerance(), aNC.TangentialTolerance());
        tolP += global::Kerf.Engine.Foundation.Precision.Confusion();

        double distVP = pv.DistanceTo(pOp);
        if (distVP > tolV + tolP)
        {
            // Curve is not closed
            return;
        }

        // Check if there will be valid range on the curve
        double first = 0.0, last = 0.0;
        double newTolV = Max(tolV, distVP + AlgoTools.DTolerance());
        if (!BrepLibrary.FindValidRange(new CurveAdaptor(c3D),
                                    ic.Tolerance(),
                                    t[0],
                                    p[0],
                                    newTolV,
                                    t[1],
                                    p[1],
                                    newTolV,
                                    ref first,
                                    ref last))
        {
            // No valid range
            return;
        }

        if (newTolV > tolV)
        {
            int nVn = UpdateVertex(nV, newTolV);
            if (nVn != nV)
            {
                pave.SetIndex(nVn);
                nV = nVn;
            }
            tolV = Tool.Tolerance(ShapeCasts.Vertex(_ds.Shape(nV)));
        }

        // Add closing pave to the curve
        Pave newPave = new();
        newPave.SetIndex(nV);
        newPave.SetParameter(tOp);
        lp.Append(newPave);
    }

    //=================================================================================================

    protected void PreparePostTreatFF(int aInt,
                                      int aCur,
                                      PaveBlock aPB,
                                      IndexedMap<Shape, CoupleOfPaveBlocks> aMSCPB,
                                      Dictionary<Shape, int> aMVI,
                                      NCollection_List<PaveBlock> aLPBC)
    {
        int nV1, nV2;
        //
        aLPBC.Append(aPB);
        //
        aPB.Indices(out nV1, out nV2);
        Vertex v1 = ShapeCasts.Vertex(_ds!.Shape(nV1));
        Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
        Edge e = ShapeCasts.Edge(_ds.Shape(aPB.Edge()));
        // Keep info for post treatment
        CoupleOfPaveBlocks cpb = new();
        cpb.SetIndexInterf(aInt);
        cpb.SetIndex(aCur);
        cpb.SetPaveBlock1(aPB);
        //
        aMSCPB.Add(e, cpb);
        aMVI[v1] = nV1;
        aMVI[v2] = nV2;
    }

    //=================================================================================================

    protected bool CheckPlanes(int nF1, int nF2)
    {
        bool bToIntersect;
        int i, iCnt;
        //
        bToIntersect = false;
        //
        FaceInfo fi1 = _ds!.ChangeFaceInfo(nF1);
        FaceInfo fi2 = _ds.ChangeFaceInfo(nF2);
        //
        NCollection_IntMap mvIn1 = fi1.VerticesIn();
        NCollection_IntMap mvOn1 = fi1.VerticesOn();
        //
        iCnt = 0;
        for (i = 0; (i < 2 && !bToIntersect); ++i)
        {
            NCollection_IntMap mv2 = (i == 0) ? fi2.VerticesIn() : fi2.VerticesOn();
            //
            foreach (int nV2 in mv2)
            {
                if (mvIn1.Contains(nV2) || mvOn1.Contains(nV2))
                {
                    ++iCnt;
                    if (iCnt > 1)
                    {
                        bToIntersect = !bToIntersect;
                        break;
                    }
                }
            }
        }
        //
        return bToIntersect;
    }

    //=================================================================================================

    protected void UpdatePaveBlocks(NCollection_IntDataMap<int> aDMNewSD)
    {
        if (aDMNewSD.IsEmpty())
        {
            return;
        }
        //
        int nSp, nbPBP, i, j;
        Span<int> nV = stackalloc int[2];
        Span<double> t = stackalloc double[2];
        bool bCB, bRebuild;
        NCollection_List<PaveBlock>.Iterator itPB = new();
        HashSet<PaveBlock> mpb = new();
        NCollection_IntMap microEdges = new();
        //
        NCollection_List<PaveBlock> allPBs = new();

        // Get pave blocks of section edges
        List<InterfFF> fFs = _ds!.InterfFF();
        int nbFF = fFs.Count;
        for (i = 0; i < nbFF; ++i)
        {
            InterfFF ff = fFs[i];
            List<SectionCurve> vnc = ff.Curves();
            int nbC = vnc.Count;
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vnc[j];
                NCollection_List<PaveBlock> lpbc = nc.PaveBlocks();
                itPB.Initialize(lpbc);
                for (; itPB.More(); itPB.Next())
                {
                    allPBs.Append(itPB.Value());
                }
            }
        }

        // Get pave blocks from the pool
        List<NCollection_List<PaveBlock>> pbp = _ds.ChangePaveBlocksPool();
        nbPBP = pbp.Count;
        for (i = 0; i < nbPBP; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                allPBs.Append(itPB.Value());
            }
        }

        // Process all pave blocks
        itPB.Initialize(allPBs);
        for (; itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            CommonBlock? cb = _ds.CommonBlock(pb);
            bCB = cb is not null;
            if (bCB)
            {
                pb = cb!.PaveBlock1();
            }
            //
            if (mpb.Add(pb))
            {
                bRebuild = false;
                pb.Indices(out nV[0], out nV[1]);
                pb.Range(out t[0], out t[1]);
                // remember the fact if the edge had different vertices before substitution
                bool wasRegularEdge = (nV[0] != nV[1]);
                //
                for (j = 0; j < 2; ++j)
                {
                    if (aDMNewSD.IsBound(nV[j]))
                    {
                        Pave pave = new();
                        //
                        nV[j] = aDMNewSD.Find(nV[j]);
                        pave.SetIndex(nV[j]);
                        pave.SetParameter(t[j]);
                        //
                        bRebuild = true;
                        if (j == 0)
                        {
                            pb.SetPave1(pave);
                        }
                        else
                        {
                            pb.SetPave2(pave);
                        }
                    }
                }
                //
                if (bRebuild)
                {
                    int nE = pb.Edge();
                    // Check if the Pave Block has the edge set
                    if (nE < 0)
                    {
                        // untouched edge
                        nE = pb.OriginalEdge();
                    }
                    bool isDegEdge = _ds.ShapeInfo(nE).HasFlag();
                    if (wasRegularEdge && !isDegEdge && nV[0] == nV[1])
                    {
                        // now edge has the same vertex on both ends;
                        // check if it is not a regular closed curve.
                        FillShrunkData(pb);
                        if (!pb.HasShrunkData)
                        {
                            // micro edge, so mark it for removal
                            microEdges.Add(nE);
                            continue;
                        }
                    }
                    nSp = SplitEdge(nE, nV[0], t[0], nV[1], t[1]);
                    if (bCB)
                    {
                        cb!.SetEdge(nSp);
                    }
                    else
                    {
                        pb.SetEdge(nSp);
                    }
                } // if (bRebuild) {
            } // if (aMPB.Add(aPB)) {
        } // for (; aItPB.More(); aItPB.Next()) {
        mpb.Clear();

        if (microEdges.Extent() != 0)
        {
            RemovePaveBlocks(microEdges);
        }
    }

    //=================================================================================================

    protected void RemovePaveBlocks(NCollection_IntMap edges)
    {
        // Remove all pave blocks referring to input edges:
        //
        // 1. from the Pave Blocks Pool
        List<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        int nbPBP = pbp.Count, i;
        for (i = 0; i < nbPBP; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            //
            NCollection_List<PaveBlock>.Iterator itPB = new(lpb);
            while (itPB.More())
            {
                PaveBlock pb = itPB.Value();
                if (edges.Contains(pb.Edge()))
                {
                    lpb.Remove(itPB);
                }
                else
                {
                    itPB.Next();
                }
            }
        }

        // 2. from section curves
        List<InterfFF> fFs = _ds.InterfFF();
        int nbFF = fFs.Count, j;
        for (i = 0; i < nbFF; ++i)
        {
            InterfFF ff = fFs[i];
            // remove from Section pave blocks
            List<SectionCurve> vnc = ff.ChangeCurves();
            int nbC = vnc.Count;
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vnc[j];
                NCollection_List<PaveBlock> lpb = nc.ChangePaveBlocks();
                NCollection_List<PaveBlock>.Iterator itPB = new(lpb);
                while (itPB.More())
                {
                    PaveBlock pb = itPB.Value();
                    if (edges.Contains(pb.Edge()))
                    {
                        lpb.Remove(itPB);
                    }
                    else
                    {
                        itPB.Next();
                    }
                }
            }
        }

        // 3. From Face Info
        for (i = 0; i < _ds.SourceShapeCount; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            if (!si.HasReference)
            {
                continue;
            }

            FaceInfo fi = _ds.ChangeFaceInfo(i);
            IndexedSet<PaveBlock>[] impb = [fi.ChangePaveBlocksIn(),
                                                               fi.ChangePaveBlocksOn(),
                                                               fi.ChangePaveBlocksSc()];
            for (int k = 0; k < 3; k++)
            {
                int nbPB = impb[k].Count, m;
                for (m = 1; m <= nbPB; ++m)
                {
                    PaveBlock pb = impb[k][m - 1];
                    if (edges.Contains(pb.Edge()))
                    {
                        break;
                    }
                }
                if (m <= nbPB)
                {
                    IndexedSet<PaveBlock> mpbCopy = new(impb[k]);
                    impb[k].Clear();
                    for (m = 1; m <= nbPB; ++m)
                    {
                        PaveBlock pb = mpbCopy[m - 1];
                        if (!edges.Contains(pb.Edge()))
                        {
                            impb[k].Add(pb);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : ToleranceFF
    // purpose  : Computes the TolFF according to the tolerance value and
    //           types of the faces.
    //=======================================================================
    private static double ToleranceFF(FaceSurfaceAdaptor aBAS1, FaceSurfaceAdaptor aBAS2)
    {
        double tol1 = aBAS1.Tolerance();
        double tol2 = aBAS2.Tolerance();
        double tolFF = Max(tol1, tol2);
        //
        bool isAna1, isAna2;
        isAna1 = (aBAS1.GetType() == Plane || aBAS1.GetType() == SurfaceKind.Cylinder
                  || aBAS1.GetType() == SurfaceKind.Cone || aBAS1.GetType() == SurfaceKind.Sphere
                  || aBAS1.GetType() == SurfaceKind.Torus);
        //
        isAna2 = (aBAS2.GetType() == Plane || aBAS2.GetType() == SurfaceKind.Cylinder
                  || aBAS2.GetType() == SurfaceKind.Cone || aBAS2.GetType() == SurfaceKind.Sphere
                  || aBAS2.GetType() == SurfaceKind.Torus);
        //
        if (!isAna1 || !isAna2)
        {
            tolFF = Max(tolFF, 5.0e-6);
        }
        return tolFF;
    }

    //=================================================================================================

    protected void UpdateBlocksWithSharedVertices()
    {
        if (!_nonDestructive)
        {
            return;
        }
        //
        int nbFF;
        //
        List<InterfFF> fFs = _ds!.InterfFF();
        nbFF = fFs.Count;
        if (nbFF == 0)
        {
            return;
        }
        //
        bool bOnCurve, bHasShapeSD;
        int i, nF1, nF2, nbC, j, nVSD = 0;
        double tolV;
        NCollection_IntMap mf = new();
        //
        for (i = 0; i < nbFF; ++i)
        {
            InterfFF ff = fFs[i];
            //
            List<SectionCurve> vc = ff.ChangeCurves();
            nbC = vc.Count;
            if (nbC == 0)
            {
                continue;
            }
            //
            ff.Indices(out nF1, out nF2);
            //
            if (mf.Add(nF1))
            {
                _ds.UpdateFaceInfoOn(nF1);
            }
            if (mf.Add(nF2))
            {
                _ds.UpdateFaceInfoOn(nF2);
            }
            //
            // Collect old vertices that are shared for nF1, nF2 ->aMI;
            NCollection_IntMap mi = new();
            //
            FaceInfo fi1 = _ds.ChangeFaceInfo(nF1);
            FaceInfo fi2 = _ds.ChangeFaceInfo(nF2);
            //
            NCollection_IntMap mvOn1 = fi1.VerticesOn();
            NCollection_IntMap mvIn1 = fi1.VerticesIn();
            NCollection_IntMap mvOn2 = fi2.VerticesOn();
            NCollection_IntMap mvIn2 = fi2.VerticesIn();
            //
            for (j = 0; j < 2; ++j)
            {
                NCollection_IntMap mv1 = (j == 0) ? mvOn1 : mvIn1;
                foreach (int nV in mv1)
                {
                    if (_ds.IsNewShape(nV))
                    {
                        continue;
                    }
                    if (mvOn2.Contains(nV) || mvIn2.Contains(nV))
                    {
                        mi.Add(nV);
                    }
                }
            }
            //
            // Try to put vertices aMI on curves
            for (j = 0; j < nbC; ++j)
            {
                SectionCurve nc = vc[j];
                double tolR3D = Max(nc.Tolerance(), nc.TangentialTolerance());
                //
                // (decision 784: the order of aMI decides the order of the new vertices UpdateVertex appends)
                foreach (int nV in mi)
                {
                    //
                    bHasShapeSD = _ds.HasShapeSD(nV, ref nVSD);
                    if (bHasShapeSD)
                    {
                        continue;
                    }
                    //
                    bOnCurve = EstimatePaveOnCurve(nV, nc, tolR3D);
                    if (!bOnCurve)
                    {
                        continue;
                    }
                    //
                    Vertex v = ShapeCasts.Vertex(_ds.Shape(nV));
                    tolV = Tool.Tolerance(v);
                    //
                    UpdateVertex(nV, tolV);
                    _ds.InitPaveBlocksForVertex(nV);
                }
            } // for (j=0; j<aNbC; ++j) {
        } // for (i=0; i<aNbFF; ++i) {
        //
        UpdateCommonBlocksWithSDVertices();
    }

    //=================================================================================================

    protected bool EstimatePaveOnCurve(int nV, SectionCurve aNC, double aTolR3D)
    {
        bool bIsVertexOnLine;
        double t = 0.0;
        //
        Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
        IntersectionCurve ic = aNC.Curve();
        //
        bIsVertexOnLine = _context!.IsVertexOnLine(v, ic, aTolR3D, ref t);
        return bIsVertexOnLine;
    }

    //=================================================================================================

    protected void CorrectToleranceOfSE()
    {
        List<InterfFF> fFs = _ds!.InterfFF();
        IndexedMap<int, NCollection_List<PaveBlock>> mvipBs = new();
        NCollection_IntMap mviToReduce = new();
        // Fence map to avoid repeated checking of the same edge
        HashSet<PaveBlock> mpb = new();
        //
        // 1. iterate on all sections F-F
        int nb = fFs.Count, i;
        for (i = 0; i < nb; ++i)
        {
            InterfFF ff = fFs[i];
            //
            List<SectionCurve> vnc = ff.ChangeCurves();
            int nbC = vnc.Count, k;
            for (k = 0; k < nbC; ++k)
            {
                SectionCurve nc = vnc[k];
                NCollection_List<PaveBlock> lpb = nc.ChangePaveBlocks();
                NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
                for (; itLPB.More();)
                {
                    PaveBlock pb = itLPB.Value();
                    if (!pb.HasEdge(out int nE))
                    {
                        lpb.Remove(itLPB);
                        continue;
                    }
                    //
                    if (!mpb.Add(pb))
                    {
                        itLPB.Next();
                        continue;
                    }
                    //
                    bool bIsReduced = false;
                    if (pb.OriginalEdge() < 0)
                    {
                        // It is possible that due to small angle between faces the
                        // common zone between faces can be large and the tangential
                        // tolerance of the curve will be large as well.
                        // Here we're trying to reduce the tolerance of the section
                        // edge using the valid tolerance of the edge.
                        // Note, that if the pave block has created common block with
                        // other edges its valid tolerance could have been changed to
                        // cover all edges in common block (see PostTreatFF() method).
                        double tolC = nc.Tolerance();
                        double tolTang = nc.TangentialTolerance();
                        if (tolC < tolTang)
                        {
                            Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                            double tolE = Tool.Tolerance(e);
                            if (tolC < tolE)
                            {
                                // reduce edge tolerance (a new section edge: never an argument's)
                                ((BrepTEdge)e.TShape()!).Tolerance(tolC);
                                bIsReduced = true;
                            }
                        }
                    }
                    //
                    // fill in the map vertex index - pave blocks
                    for (int j = 0; j < 2; j++)
                    {
                        int nV = (j == 0 ? pb.Pave1().Index() : pb.Pave2().Index());
                        _ds.HasShapeSD(nV, ref nV);
                        int index = mvipBs.IndexOf(nV) + 1;
                        NCollection_List<PaveBlock> pPBList = index > 0
                            ? mvipBs.ValueAt(index - 1)
                            : mvipBs.ValueAt(mvipBs.Add(nV, new NCollection_List<PaveBlock>()));
                        pPBList.Append(pb);
                        if (bIsReduced)
                        {
                            mviToReduce.Add(nV);
                        }
                    }
                    itLPB.Next();
                }
            }
        }
        //
        if (mviToReduce.IsEmpty())
        {
            return;
        }
        //
        // 2. try to reduce tolerances of connected vertices
        // 2.1 find all other edges containing these connected vertices to avoid
        //     reducing the tolerance to the value less than the tolerances of edges,
        //     i.e. minimal tolerance for the vertex is the max tolerance of the
        //     edges containing this vertex
        NCollection_IntDataMap<double> mviTol = new();
        List<NCollection_List<PaveBlock>> pbp = _ds.ChangePaveBlocksPool();
        nb = pbp.Count;
        Span<int> nVs = stackalloc int[2];
        for (i = 0; i < nb; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                if (!pb.HasEdge(out int nE))
                {
                    continue;
                }
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                double tolE = Tool.Tolerance(e);
                //
                pb.Indices(out nVs[0], out nVs[1]);
                //
                for (int j = 0; j < 2; j++)
                {
                    if (mviToReduce.Contains(nVs[j]))
                    {
                        if (!mviTol.IsBound(nVs[j]))
                        {
                            mviTol.Bind(nVs[j], tolE);
                        }
                        else if (tolE > mviTol.Find(nVs[j]))
                        {
                            mviTol.ChangeFind(nVs[j]) = tolE;
                        }
                        NCollection_List<PaveBlock> pbList = mvipBs.GetValue(nVs[j]);
                        pbList.Append(pb);
                    }
                }
            }
        }
        //
        // 2.2 reduce tolerances if possible
        nb = mvipBs.Count;
        for (i = 1; i <= nb; ++i)
        {
            int nV = mvipBs.KeyAt(i - 1);
            if (!mviToReduce.Contains(nV))
            {
                continue;
            }
            //
            Vertex v = ShapeCasts.Vertex(_ds.Shape(nV));
            double tolV = Tool.Tolerance(v);
            double maxTol = mviTol.Find(nV, out double pMaxTol) ? pMaxTol : 0.0;
            // it makes no sense to compute the real tolerance if it is
            // impossible to reduce the tolerance at least 0.1% of the current value
            if (tolV - maxTol < 0.001 * tolV)
            {
                continue;
            }
            //
            // compute the maximal distance from the vertex to the adjacent edges
            Point3D p = Tool.Pnt(v);
            //
            // Avoid repeated checks
            HashSet<PaveBlock> mpbFence = new();
            //
            NCollection_List<PaveBlock> lpb = mvipBs.ValueAt(i - 1);
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                if (!mpbFence.Add(pb))
                {
                    continue;
                }
                int nE = pb.Edge();
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                EdgeCurveAdaptor c = new(e);
                for (int iPave = 0; iPave < 2; ++iPave)
                {
                    Pave pave = iPave == 0 ? pb.Pave1() : pb.Pave2();
                    int nVSD = pave.Index();
                    _ds.HasShapeSD(nVSD, ref nVSD);
                    if (nVSD != nV)
                    {
                        continue;
                    }
                    //
                    Point3D ponE = c.Value(pave.Parameter());
                    double dist = p.DistanceTo(ponE);
                    dist += Tool.Tolerance(e);
                    if (dist > maxTol)
                    {
                        maxTol = dist;
                    }
                }
            }
            //
            if (maxTol < tolV)
            {
                // OCCT reduces the vertex's tolerance in place, even for an argument vertex in non-destructive mode
                // (a vertex of a section edge that is an untouched argument vertex): a frozen vertex throws here.
                ((BrepTVertex)v.TShape()!).Tolerance(maxTol);
            }
        }
    }

    //=================================================================================================

    protected void PutSEInOtherFaces(ProgressRange range)
    {
        // Try to intersect each section edge with the faces
        // not participated in its creation

        // Get all section edges
        IndexedSet<PaveBlock> mpbScAll = new();

        List<InterfFF> fFs = _ds!.InterfFF();
        int nbFF = fFs.Count;
        using var ps = new ProgressScope(range, null, 1);
        for (int i = 0; i < nbFF; ++i)
        {
            List<SectionCurve> vnc = fFs[i].Curves();
            int nbC = vnc.Count;
            for (int j = 0; j < nbC; ++j)
            {
                NCollection_List<PaveBlock> lpbc = vnc[j].PaveBlocks();
                NCollection_List<PaveBlock>.Iterator itPB = new(lpbc);
                for (; itPB.More(); itPB.Next())
                {
                    mpbScAll.Add(itPB.Value());
                }
            }
        }
        // Perform intersection of collected pave blocks
        ForceInterfEF(mpbScAll, ps.Next(), false);
    }

    //=================================================================================================

    protected void RemoveMicroSectionEdges(IndexedMap<Shape, CoupleOfPaveBlocks> mscpb,
                                           IndexedSet<PaveBlock> microPB)
    {
        if (mscpb.Count == 0)
        {
            // no section edges
            return;
        }

        // Get all F/F interferences
        List<InterfFF> fFs = _ds!.InterfFF();

        // Build the new map of section edges avoiding the micro edges
        IndexedMap<Shape, CoupleOfPaveBlocks> sepbMap = new(ShapeHasher.Instance);
        // Analyze all section edges
        int nbCPB = mscpb.Count;
        for (int i = 0; i < nbCPB; ++i)
        {
            Shape si = mscpb.KeyAt(i);
            CoupleOfPaveBlocks cpb = mscpb.ValueAt(i);

            if (si.Type != ShapeType.Edge)
            {
                // Not an edge
                sepbMap.Add(si, cpb);
                continue;
            }

            // Get pave block for analysis
            PaveBlock pb = cpb.PaveBlock1()!;
            if (pb.HasEdge())
            {
                // Not a real section edge
                sepbMap.Add(si, cpb);
                continue;
            }

            if (!AlgoTools.IsMicroEdge(ShapeCasts.Edge(si), _context, false))
            {
                // Normal edge
                sepbMap.Add(si, cpb);
                continue;
            }

            // Micro edge is found, avoid it in the <theMSCPB> map
            // and remove from the F/F Intersection info structure

            // Get F/F interference which created this micro edge
            InterfFF ff = fFs[cpb.IndexInterf()];
            // Get curve from which this edge has been created
            SectionCurve curve = ff.ChangeCurves()[cpb.Index()];
            // Get all section pave blocks created from this curve
            NCollection_List<PaveBlock> lpbc = curve.ChangePaveBlocks();
            // Remove pave block from the list
            for (NCollection_List<PaveBlock>.Iterator it = new(lpbc); it.More(); it.Next())
            {
                if (it.Value() == pb)
                {
                    lpbc.Remove(it);
                    break;
                }
            }

            // Add the pave block of "micro" edge into outgoing map for
            // unification of its vertices in the PostTreatFF method
            microPB.Add(pb);
        }

        // Overwrite the old map if necessary (OCCT's assignment: the caller's map takes the new contents)
        if (sepbMap.Count != mscpb.Count)
        {
            mscpb.Clear();
            for (int i = 0; i < sepbMap.Count; ++i)
            {
                mscpb.Add(sepbMap.KeyAt(i), sepbMap.ValueAt(i));
            }
        }
    }

    //=================================================================================================

    protected void RemoveMicroEdges()
    {
        // Fence map
        HashSet<PaveBlock> mpbFence = new();
        // Resulting map of micro edges
        NCollection_IntMap microEdges = new();
        // Check all pave blocks from the pool to find the micro edges
        List<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        int nbPBP = pbp.Count;
        for (int i = 0; i < nbPBP; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            if (lpb.Extent() < 2)
            {
                // No splits
                continue;
            }

            if (_ds.ShapeInfo(lpb.First().OriginalEdge()).HasFlag())
            {
                continue;
            }

            NCollection_List<PaveBlock>.Iterator it = new(lpb);
            for (; it.More(); it.Next())
            {
                PaveBlock pb = it.Value();
                PaveBlock pbr = _ds.RealPaveBlock(pb);

                if (mpbFence.Add(pbr))
                {
                    pbr.Indices(out int nV1, out int nV2);
                    if (nV1 == nV2)
                    {
                        // Check if it has the valid range
                        FillShrunkData(pbr);
                        if (!pbr.HasShrunkData)
                        {
                            microEdges.Add(pbr.Edge());
                        }
                    }
                }
            }
        }
        RemovePaveBlocks(microEdges);
    }
}
