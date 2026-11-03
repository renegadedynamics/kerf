// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>Tools used in the intersection part of the Boolean operations.</summary>
/// <remarks>
/// Unit 6E ported the members the pave filler calls, unit 6F the rest. OCCT's templates <c>MakeBlocks</c> and
/// <c>FillMap</c> are generic over the key; the allocator arguments go. <see cref="WiresToFaces"/> copies a frozen input at
/// its entry, since OCCT writes p-curves and tolerances into the input's edges.
/// </remarks>
public static partial class BuilderTools
{
    /// <summary>Makes the chains of the connected elements from the given connexity map.</summary>
    /// <remarks>OCCT's fence map takes the map's hasher from its type: pass it as <paramref name="hasher"/> (null: the default).</remarks>
    public static void MakeBlocks<TKey>(NCollection_IndexedDataMap<TKey, NCollection_List<TKey>> mili,
                                        NCollection_List<NCollection_List<TKey>> mBlocks,
                                        IEqualityComparer<TKey>? hasher = null)
        where TKey : notnull
    {
        NCollection_Map<TKey> mFence = new(hasher);
        int i, nb = mili.Extent();
        for (i = 1; i <= nb; ++i)
        {
            TKey n = mili.FindKey(i);
            if (!mFence.Add(n))
                continue;
            //
            // Start the chain
            NCollection_List<TKey> chain = mBlocks.Append(new NCollection_List<TKey>());
            chain.Append(n);
            // Look for connected elements
            NCollection_List<TKey>.Iterator itLChain = new(chain);
            for (; itLChain.More(); itLChain.Next())
            {
                TKey n1 = itLChain.Value();
                NCollection_List<TKey> li = mili.FindFromKey(n1);
                // Add connected elements into the chain
                NCollection_List<TKey>.Iterator itLI = new(li);
                for (; itLI.More(); itLI.Next())
                {
                    TKey n2 = itLI.Value();
                    if (mFence.Add(n2))
                    {
                        chain.Append(n2);
                    }
                }
            }
        }
    }

    /// <summary>Fills the map with the connected entities.</summary>
    public static void FillMap<TKey>(TKey n1, TKey n2, NCollection_IndexedDataMap<TKey, NCollection_List<TKey>> mili)
        where TKey : notnull
    {
        int index1 = mili.FindIndex(n1);
        NCollection_List<TKey> pList1 = index1 > 0 ? mili[index1] : mili[mili.Add(n1, new NCollection_List<TKey>())];
        pList1.Append(n2);
        //
        int index2 = mili.FindIndex(n2);
        NCollection_List<TKey> pList2 = index2 > 0 ? mili[index2] : mili[mili.Add(n2, new NCollection_List<TKey>())];
        pList2.Append(n1);
    }

    /// <summary>Adds the face to the pave block's list of faces.</summary>
    public static void FillMap(PaveBlock aPB, int nF, NCollection_IndexedDataMap<PaveBlock, NCollection_List<int>> aMPBLI)
    {
        int index = aMPBLI.FindIndex(aPB);
        NCollection_List<int> pLI = index > 0 ? aMPBLI[index] : aMPBLI[aMPBLI.Add(aPB, new NCollection_List<int>())];
        pLI.Append(nF);
    }

    /// <summary>Creates the common blocks from the groups of connected pave blocks of <paramref name="aMPBLPB"/>.</summary>
    public static void PerformCommonBlocks(NCollection_IndexedDataMap<PaveBlock, NCollection_List<PaveBlock>> aMPBLPB,
                                           DS pDS, IntersectionContext? context = null)
    {
        int nbCB;
        //
        nbCB = aMPBLPB.Extent();
        if (nbCB == 0)
        {
            return;
        }
        // Make Blocks of the pave blocks
        NCollection_List<NCollection_List<PaveBlock>> mBlocks = new();
        MakeBlocks(aMPBLPB, mBlocks);

        // Use temporary allocator for the local fence map
        NCollection_IntMap mFaces = new();

        NCollection_List<NCollection_List<PaveBlock>>.Iterator itB = new(mBlocks);
        for (; itB.More(); itB.Next())
        {
            NCollection_List<PaveBlock> lpb = itB.Value();
            int nbPB = lpb.Extent();
            if (nbPB < 2)
            {
                continue;
            }

            mFaces.Clear();
            // New common block
            CommonBlock? cb = null;
            // Faces of the common block
            NCollection_List<int> lFaces = new();

            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                if (pDS.IsCommonBlock(pb))
                {
                    CommonBlock cBx = pDS.CommonBlock(pb)!;
                    // Move all faces to the new common block
                    NCollection_List<int>.Iterator itLF = new(cBx.Faces());
                    for (; itLF.More(); itLF.Next())
                    {
                        int nF = itLF.Value();
                        // Append to common list avoiding duplicates
                        if (mFaces.Add(nF))
                        {
                            lFaces.Append(nF);
                        }
                    }
                    cb ??= cBx;
                }
            }

            cb ??= new CommonBlock();

            cb.SetPaveBlocks(lpb);
            cb.SetFaces(lFaces);
            for (itLPB.Initialize(lpb); itLPB.More(); itLPB.Next())
            {
                pDS.SetCommonBlock(itLPB.Value(), cb);
            }

            // Compute tolerance for Common Block
            double tolCB = ComputeToleranceOfCB(cb, pDS, context);
            cb.SetTolerance(tolCB);
        }
    }

    /// <summary>Creates the common blocks on faces from the pave block → faces map <paramref name="aMPBLI"/>.</summary>
    public static void PerformCommonBlocks(NCollection_IndexedDataMap<PaveBlock, NCollection_List<int>> aMPBLI,
                                           DS pDS, IntersectionContext? context = null)
    {
        int nF, i, nb;
        NCollection_List<int>.Iterator itLI = new();
        PaveBlock pb;
        CommonBlock cb;
        //
        nb = aMPBLI.Extent();
        for (i = 1; i <= nb; ++i)
        {
            pb = aMPBLI.FindKey(i);
            if (pDS.IsCommonBlock(pb))
            {
                cb = pDS.CommonBlock(pb)!;
            }
            else
            {
                cb = new CommonBlock();
                cb.AddPaveBlock(pb);
            }
            //
            NCollection_List<int> li = aMPBLI.FindFromKey(pb);
            NCollection_List<int> newFaces = new();
            NCollection_List<int> oldFaces = cb.Faces();
            itLI.Initialize(li);
            for (; itLI.More(); itLI.Next())
            {
                nF = itLI.Value();
                // the both lists aLI and anOldFaces are expected to be short,
                // so we can allow to run nested loop here
                NCollection_List<int>.Iterator it = new(oldFaces);
                for (; it.More(); it.Next())
                {
                    if (it.Value() == nF)
                    {
                        break;
                    }
                }
                if (!it.More())
                {
                    newFaces.Append(nF);
                }
            }
            cb.AppendFaces(newFaces);
            pDS.SetCommonBlock(pb, cb);
            // Compute tolerance for Common Block
            double tolCB = ComputeToleranceOfCB(cb, pDS, context);
            cb.SetTolerance(tolCB);
        }
    }

    /// <summary>
    /// The tolerance of a common block: the largest distance from 11 points of its real pave block's edge to the other
    /// edges and the faces of the block, plus their tolerances (at least the edge's tolerance).
    /// </summary>
    public static double ComputeToleranceOfCB(CommonBlock? cb, DS ds, IntersectionContext? context)
    {
        double tolMax = 0.0;
        if (cb is null)
        {
            return tolMax;
        }
        //
        PaveBlock pbr = cb.PaveBlock1();
        int nE = pbr.OriginalEdge();
        Edge eOr = ShapeCasts.Edge(ds.Shape(nE));
        tolMax = Tool.Tolerance(eOr);
        //
        NCollection_List<PaveBlock> lpb = cb.PaveBlocks();
        NCollection_List<int> lfi = cb.Faces();
        //
        if ((lpb.Extent() < 2) && lfi.IsEmpty())
        {
            return tolMax;
        }
        //
        const int nbPnt = 11;
        double tol, t, t1, t2, dt;
        Point3D p;
        //
        Curve3D c3D = Tool.Curve(eOr, out t1, out t2)!;
        //
        pbr.Range(out t1, out t2);
        dt = (t2 - t1) / (nbPnt + 1);
        //
        IntersectionContext ctx = context ?? new IntersectionContext();

        // compute max tolerance for common blocks on edges
        if (lpb.Extent() > 1)
        {
            // compute max distance between edges
            NCollection_List<PaveBlock>.Iterator itPB = new();
            //
            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                PaveBlock pb = itPB.Value();
                if (pb == pbr)
                {
                    continue;
                }
                //
                nE = pb.OriginalEdge();
                Edge e = ShapeCasts.Edge(ds.Shape(nE));
                tol = Tool.Tolerance(e);
                //
                // OCCT copies the context's projector (aProjPC = aCtx->ProjPC(aE)) and projects with the copy: the
                // context's projector, used here directly, is left with the last projection, which nothing reads.
                ProjectPointOnCurve projPC = ctx.ProjPC(e);
                //
                t = t1;
                for (int i = 1; i <= nbPnt; i++)
                {
                    t += dt;
                    c3D.D0(t, out p);
                    projPC.Perform(p);
                    if (projPC.NbPoints() != 0)
                    {
                        double tolNew = tol + projPC.LowerDistance();
                        if (tolNew > tolMax)
                        {
                            tolMax = tolNew;
                        }
                    }
                }
            }
        }
        //
        // compute max tolerance for common blocks on faces
        if (lfi.Extent() != 0)
        {
            for (NCollection_List<int>.Iterator itLI = new(lfi); itLI.More(); itLI.Next())
            {
                int nF = itLI.Value();
                Face f = ShapeCasts.Face(ds.Shape(nF));
                tol = Tool.Tolerance(f);
                //
                ProjectPointOnSurface projPS = ctx.ProjPS(f);
                //
                t = t1;
                for (int i = 1; i <= nbPnt; i++)
                {
                    t += dt;
                    c3D.D0(t, out p);
                    projPS.Perform(p);
                    if (projPS.NbPoints() != 0)
                    {
                        double tolNew = tol + projPS.LowerDistance();
                        if (tolNew > tolMax)
                        {
                            tolMax = tolNew;
                        }
                    }
                }
            }
        }
        //
        return tolMax;
    }

    /// <summary>
    /// Makes wires from the edges of <paramref name="edges"/>, grouping them by the planes they lie in. The edges are
    /// intersected first with the General Fuse unless <paramref name="shared"/> says they already share their
    /// vertices. Returns 0 on success, 1 when there is no edge to process, 2 when the edges can't be intersected (then
    /// <paramref name="wires"/> is left unchanged).
    /// </summary>
    public static int EdgesToWires(in Shape edges, ref Shape wires, bool shared = false, double angTol = 1.0e-8)
    {
        int iErr = 0;
        //
        // 1. Check the input edges
        //
        // List of edges to process
        NCollection_List<Shape> le = new();
        //
        Explorer exp = new(edges, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(e) && Tool.IsGeometric(e))
            {
                le.Append(exp.Current());
            }
        }
        //
        if (le.IsEmpty())
        {
            // no edges to process
            iErr = 1;
            return iErr;
        }
        //
        BrepBuilder bb = new();
        Compound rWires = new();
        bb.MakeCompound(ref rWires);
        //
        if (le.Extent() == 1)
        {
            Wire wire = new();
            bb.MakeWire(ref wire);
            bb.Add(wire, le.First());
            bb.Add(rWires, wire);
            wires = rWires;
            return iErr;
        }
        //
        // 2. Make compound of shared edges
        Shape sEdges;
        //
        if (!shared)
        {
            // intersect the edges if necessary
            BooleanBuilder gf = new();
            gf.SetArguments(le);
            gf.Perform();
            if (gf.HasErrors)
            {
                // unable to share the edges
                iErr = 2;
                return iErr;
            }
            //
            sEdges = gf.InternalShape();
        }
        else
        {
            Compound sEdgesC = new();
            bb.MakeCompound(ref sEdgesC);
            sEdges = sEdgesC;
            NCollection_List<Shape>.Iterator itLE = new(le);
            for (; itLE.More(); itLE.Next())
            {
                bb.Add(sEdges, itLE.Value());
            }
        }
        //
        // 3. Find edges located in the same planes and make wires from them.
        // If the plane cannot be found for a single edge, then it is necessary
        // to find all pairs of connected edges with the same cross product.

        // Try to compute the plane in which the edge is located
        NCollection_IndexedDataMap<Shape, Plane3D> dmEdgePln = new(ShapeHasher.Instance);
        // Compute the tangent direction for the edges for which the plane is not defined
        NCollection_IndexedDataMap<Shape, Direction3D> dmEdgeTgt = new(ShapeHasher.Instance);
        //
        // edges for which the plane is not found
        NCollection_Map<Shape> mEdgesNoUniquePlane = new(ShapeHasher.Instance);
        //
        // edges for which the plane cannot be found on a single edge
        Compound lEdges = new();
        bb.MakeCompound(ref lEdges);
        //
        exp.Init(sEdges, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            EdgeCurveAdaptor bac = new(e);
            //
            Plane3D pln = new();
            if (FindPlane(bac, ref pln))
            {
                dmEdgePln.Add(e, pln);
            }
            else
            {
                Vector3D vt = new();
                if (FindEdgeTangent(bac, ref vt))
                {
                    dmEdgeTgt.Add(e, new Direction3D(vt));
                    bb.Add(lEdges, e);
                    mEdgesNoUniquePlane.Add(e);
                }
            }
        }
        //
        // to avoid processing of the same edges in the same plane store
        // the processed planes into a list and use it as a fence map
        NCollection_List<Direction3D> lpFence = new();
        //
        // used edges
        NCollection_Map<Shape> meFence = new(ShapeHasher.Instance);
        //
        // look for a planes on the single edges
        int i, j, nbPlanes = dmEdgePln.Extent(), nbEdges = dmEdgeTgt.Extent();
        for (i = 1; i <= nbPlanes; ++i)
        {
            Shape ei = dmEdgePln.FindKey(i);
            if (!meFence.Add(ei))
            {
                continue;
            }
            //
            Plane3D plnI = dmEdgePln[i];
            Direction3D di = plnI.Position().Direction();
            //
            lpFence.Append(di);
            //
            NCollection_IndexedMap<Shape> mePln = new(ShapeHasher.Instance);
            mePln.Add(ei);
            //
            NCollection_IndexedMap<Shape> mv = new(ShapeHasher.Instance);
            Exploration.MapShapes(ei, ShapeType.Vertex, mv);
            //
            // look for other edges with the plane parallel to current one
            for (j = i + 1; j <= nbPlanes; ++j)
            {
                Direction3D dj = dmEdgePln[j].Position().Direction();
                if (di.IsParallel(dj, angTol))
                {
                    Shape ej = dmEdgePln.FindKey(j);
                    mePln.Add(ej);
                    meFence.Add(ej);
                    Exploration.MapShapes(ej, ShapeType.Vertex, mv);
                }
            }
            //
            // look for all other edges located in the plane parallel to current one
            Compound cePln = new();
            bb.MakeCompound(ref cePln);
            //
            for (j = 1; j <= nbEdges; ++j)
            {
                Direction3D dj = dmEdgeTgt[j];
                if (di.IsNormal(dj, angTol))
                {
                    bb.Add(cePln, dmEdgeTgt.FindKey(j));
                }
            }
            //
            // make blocks of these edges and check blocks to be connected
            // to any of the already added edges or forming a wire themselves
            NCollection_List<Shape> lcbe = new();
            AlgoTools.MakeConnexityBlocks(cePln, ShapeType.Vertex, ShapeType.Edge, lcbe);
            //
            // make wire from each block
            NCollection_List<Shape>.Iterator itLCB = new(lcbe);
            for (; itLCB.More(); itLCB.Next())
            {
                Shape cbe = itLCB.Value();
                //
                // check connectivity
                Explorer expV = new(cbe, ShapeType.Vertex);
                for (; expV.More(); expV.Next())
                {
                    if (mv.Contains(expV.Current()))
                    {
                        break;
                    }
                }
                //
                bool bAddBlock = expV.More();
                if (!bAddBlock)
                {
                    // check if the edges are forming a wire
                    Plane3D pln = new();
                    bAddBlock = FindPlane(cbe, ref pln, dmEdgeTgt, mEdgesNoUniquePlane);
                }
                //
                if (bAddBlock)
                {
                    // add edges
                    for (Iterator itE = new(cbe); itE.More(); itE.Next())
                    {
                        mePln.Add(itE.Value());
                    }
                }
            }
            //
            MakeWires(mePln, rWires, false, dmEdgeTgt, mEdgesNoUniquePlane);
        }
        //
        // make connection map from vertices to edges to find the connected pairs
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> dmve = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(lEdges, ShapeType.Vertex, ShapeType.Edge, dmve);
        //
        // find planes for connected edges
        int nbV = dmve.Extent();
        for (i = 1; i <= nbV; ++i)
        {
            NCollection_List<Shape> lei = dmve[i];
            if (lei.Extent() < 2)
            {
                continue;
            }
            //
            NCollection_List<Shape>.Iterator itLEI1 = new(lei);
            for (; itLEI1.More(); itLEI1.Next())
            {
                Shape ei1 = itLEI1.Value();
                Direction3D di1 = dmEdgeTgt.FindFromKey(ei1);
                //
                NCollection_List<Shape>.Iterator itLEI2 = new(lei);
                for (; itLEI2.More(); itLEI2.Next())
                {
                    Shape ei2 = itLEI2.Value();
                    if (ei2.IsSame(ei1))
                    {
                        continue;
                    }
                    //
                    Direction3D di2 = dmEdgeTgt.FindFromKey(ei2);
                    //
                    if (di1.IsParallel(di2, angTol))
                    {
                        continue;
                    }
                    //
                    Direction3D dni = di1 ^ di2;
                    //
                    // check if this normal direction has not been checked yet
                    NCollection_List<Direction3D>.Iterator itLPln = new(lpFence);
                    for (; itLPln.More(); itLPln.Next())
                    {
                        if (dni.IsParallel(itLPln.Value(), angTol))
                        {
                            break;
                        }
                    }
                    if (itLPln.More())
                    {
                        continue;
                    }
                    //
                    lpFence.Append(dni);
                    //
                    // find all other edges in the plane parallel to current one
                    NCollection_IndexedMap<Shape> mePln = new(ShapeHasher.Instance);
                    mePln.Add(ei1);
                    mePln.Add(ei2);
                    //
                    // iterate on all other edges to find all edges lying in the plane parallel to current one
                    for (j = 1; j <= nbEdges; ++j)
                    {
                        Direction3D dj = dmEdgeTgt[j];
                        if (dni.IsNormal(dj, angTol))
                        {
                            mePln.Add(dmEdgeTgt.FindKey(j));
                        }
                    }
                    //
                    MakeWires(mePln, rWires, true, dmEdgeTgt, mEdgesNoUniquePlane);
                } // for (; aItLEI2.More(); aItLEI2.Next()) {
            } // for (; aItLEI1.More(); aItLEI1.Next()) {
        } // for (i = 1; i < aNb; ++i) {
        //
        // 4. Find unused edges and make wires from them
        NCollection_IndexedMap<Shape> meAlone = new(ShapeHasher.Instance), meUsed = new(ShapeHasher.Instance);
        Exploration.MapShapes(rWires, ShapeType.Edge, meUsed);
        //
        for (i = 1; i <= nbEdges; ++i)
        {
            Shape e = dmEdgeTgt.FindKey(i);
            if (!meUsed.Contains(e))
            {
                meAlone.Add(e);
            }
        }
        //
        MakeWires(meAlone, rWires, false, dmEdgeTgt, mEdgesNoUniquePlane);
        //
        wires = rWires;
        //
        return iErr;
    }

    /// <summary>
    /// Makes planar faces from the wires of <paramref name="wires"/>, grouping the wires lying in one plane. Returns
    /// true if any face was built.
    /// </summary>
    /// <remarks>
    /// OCCT adds p-curves on the planes to the input's edges and corrects their tolerances in place. It runs on the
    /// input in a working overlay: the edges it writes into are new in the result, everything else is the caller's.
    /// </remarks>
    public static bool WiresToFaces(in Shape wires, ref Shape faces, double angTol = 1.0e-8)
    {
        // Kerf: OCCT writes into the input's edges (p-curves, tolerances): the overlay takes the writes
        using var overlay = WorkingOverlay.Begin(wires);
        Shape aWires = wires;
        //
        BrepBuilder bb = new();
        NCollection_Map<Shape> mFence = new(ShapeHasher.Instance);
        Compound rFaces = new();
        bb.MakeCompound(ref rFaces);
        //
        const double max = 1.0e+8;
        //
        // map to store the tangent vectors for the edges
        NCollection_IndexedDataMap<Shape, Direction3D> dmEdgeTgt = new(ShapeHasher.Instance);
        // maps to store the planes found for the wires
        NCollection_IndexedDataMap<Shape, Plane3D> dmWirePln = new(ShapeHasher.Instance);
        // map to store the tolerance for the wire
        NCollection_DataMap<Shape, double> dmWireTol = new(ShapeHasher.Instance);
        // edges for which the plane is not found
        NCollection_Map<Shape> mEdgesNoUniquePlane = new(ShapeHasher.Instance);
        //
        // Find planes for the wires
        Explorer expW = new(aWires, ShapeType.Wire);
        for (; expW.More(); expW.Next())
        {
            Wire wire = ShapeCasts.Wire(expW.Current());
            Plane3D plane = new();
            if (FindPlane(wire, ref plane, dmEdgeTgt, mEdgesNoUniquePlane))
            {
                dmWirePln.Add(wire, plane);
                // find tolerance for the wire - max tolerance of its edges
                dmWireTol.Bind(wire, Tool.MaxTolerance(wire, ShapeType.Edge));
            }
        }
        //
        int i, j, nb = dmWirePln.Extent();
        for (i = 1; i <= nb; ++i)
        {
            Shape wireI = dmWirePln.FindKey(i);
            if (mFence.Contains(wireI))
            {
                continue;
            }
            //
            Plane3D plnI = dmWirePln[i];
            //
            NCollection_List<Shape> lw = new();
            lw.Append(wireI);
            mFence.Add(wireI);
            //
            double tolI = dmWireTol.Find(wireI);
            //
            // Find other wires in the same plane
            for (j = i + 1; j <= nb; ++j)
            {
                Shape wireJ = dmWirePln.FindKey(j);
                if (mFence.Contains(wireJ))
                {
                    continue;
                }
                //
                // check if the planes are the same
                Plane3D plnJ = dmWirePln[j];
                // check direction of the planes
                if (!plnI.Position().Direction().IsParallel(plnJ.Position().Direction(), angTol))
                {
                    continue;
                }
                // check distance between the planes
                double dist = plnI.Distance(plnJ.Location());
                double tolJ = dmWireTol.Find(wireJ);
                if (dist > (tolI + tolJ))
                {
                    continue;
                }
                //
                lw.Append(wireJ);
                mFence.Add(wireJ);
            }
            //
            // Take the edges to build the face
            NCollection_List<Shape> le = new();
            NCollection_List<Shape>.Iterator itLW = new(lw);
            for (; itLW.More(); itLW.Next())
            {
                Iterator itE = new(itLW.Value());
                for (; itE.More(); itE.Next())
                {
                    le.Append(itE.Value().Oriented(Forward));
                    le.Append(itE.Value().Oriented(Reversed));
                }
            }
            //
            // build planar face
            Face ff = new MakeFace(plnI, -max, max, -max, max).InternalFace();
            ff.Orientation(Forward);
            //
            try
            {
                //
                // build pcurves for edges on this face
                BrepLibrary.BuildPCurveForEdgesOnPlane(le, ff);
                //
                // split the face with the edges
                BuilderFace bf = new();
                bf.SetShapes(le);
                bf.SetFace(ff);
                bf.Perform();
                if (bf.HasErrors)
                {
                    continue;
                }
                //
                NCollection_List<Shape> lfSp = bf.Areas();
                NCollection_List<Shape>.Iterator itLF = new(lfSp);
                for (; itLF.More(); itLF.Next())
                {
                    Shape fSp = itLF.Value();
                    bb.Add(rFaces, fSp);
                }
            }
            catch (FailureException)
            {
                continue;
            }
        }
        //
        // fix tolerances of the resulting faces
        NCollection_IndexedMap<Shape> mEmpty = new(ShapeHasher.Instance);
        AlgoTools.CorrectTolerances(rFaces, mEmpty, 0.05, false);
        AlgoTools.CorrectShapeTolerances(rFaces, mEmpty, false);
        //
        faces = overlay.Finish(rFaces);
        return faces.ChildCount > 0;
    }

    // Makes wires from the separate blocks of the given edges (OCCT's file-static MakeWires).
    private static void MakeWires(NCollection_IndexedMap<Shape> edges,
                                  Compound wires,
                                  bool checkUniquePlane,
                                  NCollection_IndexedDataMap<Shape, Direction3D> dmEdgeTgt,
                                  NCollection_Map<Shape> mEdgesNoUniquePlane)
    {
        Compound ce = new();
        new BrepBuilder().MakeCompound(ref ce);
        int i, nbE = edges.Extent();
        for (i = 1; i <= nbE; ++i)
        {
            new BrepBuilder().Add(ce, edges[i]);
        }
        //
        NCollection_List<Shape> lcbe = new();
        AlgoTools.MakeConnexityBlocks(ce, ShapeType.Vertex, ShapeType.Edge, lcbe);
        //
        // make wire from each block
        NCollection_List<Shape>.Iterator itLCB = new(lcbe);
        for (; itLCB.More(); itLCB.Next())
        {
            Shape cbe = itLCB.Value();
            //
            if (checkUniquePlane)
            {
                Plane3D pln = new();
                if (!FindPlane(cbe, ref pln, dmEdgeTgt, mEdgesNoUniquePlane))
                {
                    continue;
                }
            }
            //
            Wire wire = new();
            new BrepBuilder().MakeWire(ref wire);
            for (Iterator itE = new(cbe); itE.More(); itE.Next())
            {
                new BrepBuilder().Add(wire, itE.Value());
            }
            //
            new BrepBuilder().Add(wires, wire);
        }
    }

    // Finds the tangent for the edge using the map (OCCT's file-static FindEdgeTangent); theTgt is written on success.
    private static bool FindEdgeTangent(in Edge edge,
                                        NCollection_IndexedDataMap<Shape, Direction3D> dmEdgeTgt,
                                        ref Direction3D tgt)
    {
        int index = dmEdgeTgt.FindIndex(edge);
        if (index == 0)
        {
            Vector3D vte = new();
            EdgeCurveAdaptor bac = new(edge);
            if (!FindEdgeTangent(bac, ref vte))
            {
                return false;
            }
            index = dmEdgeTgt.Add(edge, new Direction3D(vte));
        }
        tgt = dmEdgeTgt[index];
        return true;
    }

    // Finds the tangent for the edge (OCCT's file-static FindEdgeTangent).
    private static bool FindEdgeTangent(EdgeCurveAdaptor curve, ref Vector3D tangent)
    {
        if (!curve.Is3DCurve())
        {
            return false;
        }
        // for the line the tangent is defined by the direction
        if (curve.GetType() == CurveKind.Line)
        {
            tangent = new Vector3D(curve.Line().Position().Direction());
            return true;
        }
        //
        // for other curves take D1 and check for its length
        double t, t1 = curve.FirstParameter(), t2 = curve.LastParameter();
        const int nbP = 11;
        double dt = (t2 - t1) / nbP;
        //
        for (t = t1 + dt; t <= t2; t += dt)
        {
            curve.D1(t, out Point3D _, out tangent);
            if (tangent.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return true;
            }
        }
        //
        return false;
    }

    // Finds the plane in which the edge is located (OCCT's file-static FindPlane); thePlane is written on success.
    private static bool FindPlane(EdgeCurveAdaptor curve, ref Plane3D plane)
    {
        if (!curve.Is3DCurve())
        {
            return false;
        }
        //
        bool bFound = true;
        Vector3D vn = new();
        switch (curve.GetType())
        {
            case CurveKind.Line:
                return false;
            case CurveKind.Circle:
                vn = new Vector3D(curve.Circle().Position().Direction());
                break;
            case CurveKind.Ellipse:
                vn = new Vector3D(curve.Ellipse().Position().Direction());
                break;
            case CurveKind.Hyperbola:
                vn = new Vector3D(curve.Hyperbola().Position().Direction());
                break;
            case CurveKind.Parabola:
                vn = new Vector3D(curve.Parabola().Position().Direction());
                break;
            default:
            {
                // for all other types of curve compute two tangent vectors
                // on the curve and cross them
                bFound = false;
                double t, t1 = curve.FirstParameter(), t2 = curve.LastParameter();
                const int nbP = 11;
                double dt = (t2 - t1) / nbP;
                //
                t = t1;
                curve.D1(t, out Point3D _, out Vector3D v1);
                //
                for (t = t1 + dt; t <= t2; t += dt)
                {
                    curve.D1(t, out Point3D _, out Vector3D v2);
                    //
                    vn = v1 ^ v2;
                    if (vn.Magnitude() > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        bFound = true;
                        break;
                    }
                }
                break;
            }
        }
        //
        if (bFound)
        {
            plane = new Plane3D(curve.Value(curve.FirstParameter()), new Direction3D(vn));
        }
        return bFound;
    }

    // Finds the plane in which the wire is located (OCCT's file-static FindPlane); thePlane is written on success.
    private static bool FindPlane(in Shape wire,
                                  ref Plane3D plane,
                                  NCollection_IndexedDataMap<Shape, Direction3D> dmEdgeTgt,
                                  NCollection_Map<Shape> mEdgesNoUniquePlane)
    {
        Explorer expE1 = new(wire, ShapeType.Edge);
        if (!expE1.More())
        {
            return false;
        }
        //
        // try to find two not parallel edges in wire to get normal of the plane
        for (; expE1.More(); expE1.Next())
        {
            // get the first edge in the wire
            Edge e1 = ShapeCasts.Edge(expE1.Current());
            //
            // find tangent for the first edge
            Direction3D dte1 = new();
            if (!FindEdgeTangent(e1, dmEdgeTgt, ref dte1))
            {
                continue;
            }
            //
            // find the other edge not parallel to the first one
            Explorer expE2 = new(wire, ShapeType.Edge);
            for (; expE2.More(); expE2.Next())
            {
                Edge e2 = ShapeCasts.Edge(expE2.Current());
                if (e1.IsSame(e2))
                {
                    continue;
                }
                //
                // find tangent for the second edge
                Direction3D dte2 = new();
                if (!FindEdgeTangent(e2, dmEdgeTgt, ref dte2))
                {
                    continue;
                }
                //
                if (dte1.IsParallel(dte2, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    continue;
                }
                //
                Direction3D dn = dte1 ^ dte2;
                //
                Iterator itV = new(e1);
                plane = new Plane3D(Tool.Pnt(ShapeCasts.Vertex(itV.Value())), dn);
                return true;
            }
        }
        //
        // try to compute normal on the single edge
        expE1.Init(wire, ShapeType.Edge);
        for (; expE1.More(); expE1.Next())
        {
            Edge e = ShapeCasts.Edge(expE1.Current());
            if (mEdgesNoUniquePlane.Contains(e))
            {
                continue;
            }
            EdgeCurveAdaptor bac = new(e);
            if (FindPlane(bac, ref plane))
            {
                return true;
            }
            mEdgesNoUniquePlane.Add(e);
        }
        return false;
    }

    /// <summary>Selects the pairs of vertices whose tolerance spheres (with the fuzzy value) meet.</summary>
    private sealed class BOPAlgo_PairVerticesSelector : PairSelector<NCollection_Vec3<double>>
    {
        private NCollection_IndexedDataMap<Shape, double>? _vertices;
        private double _fuzzyValue;

        public BOPAlgo_PairVerticesSelector()
        {
            _vertices = null;
            _fuzzyValue = global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        //! Sets the map of vertices with tolerances
        public void SetMapOfVerticesTolerances(NCollection_IndexedDataMap<Shape, double> vertices) => _vertices = vertices;

        //! Sets the fuzzy value
        public void SetFuzzyValue(double fuzzyValue) => _fuzzyValue = fuzzyValue;

        //! Checks and accepts the pair of elements.
        public override bool Accept(int id1, int id2)
        {
            if (!RejectElement(id1, id2))
            {
                int anID1 = _bvhSet1!.Element(id1);
                Vertex v1 = ShapeCasts.Vertex(_vertices!.FindKey(anID1));
                double tolV1 = Tool.Tolerance(v1);
                if (tolV1 < _vertices.FindFromIndex(anID1))
                {
                    tolV1 = _vertices.FindFromIndex(anID1);
                }
                Point3D p1 = Tool.Pnt(v1);

                int anID2 = _bvhSet1.Element(id2);
                Vertex v2 = ShapeCasts.Vertex(_vertices.FindKey(anID2));
                double tolV2 = Tool.Tolerance(v2);
                if (tolV2 < _vertices.FindFromIndex(anID2))
                {
                    tolV2 = _vertices.FindFromIndex(anID2);
                }
                Point3D p2 = Tool.Pnt(v2);

                double tolSum2 = tolV1 + tolV2 + _fuzzyValue;
                tolSum2 *= tolSum2;

                double d2 = p1.SquareDistance(p2);
                if (d2 < tolSum2)
                {
                    _pairs.Append(new PairIDs(anID1, anID2));
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>
    /// Finds the chains of intersecting vertices: the vertices of <paramref name="vertices"/> (with tolerances at
    /// least their values) connected by meeting tolerance spheres (with half the fuzzy value each).
    /// </summary>
    public static void IntersectVertices(NCollection_IndexedDataMap<Shape, double> vertices,
                                         double fuzzyValue,
                                         NCollection_List<NCollection_List<Shape>> chains)
    {
        int nbV = vertices.Extent();
        if (nbV <= 1)
        {
            if (nbV == 1)
            {
                chains.Append(new NCollection_List<Shape>()).Append(vertices.FindKey(1));
            }
            return;
        }

        // Additional tolerance for intersection
        double tolAdd = fuzzyValue / 2.0;

        // Use BVH Tree for sorting the vertices
        BoxTree bbTree = new();
        bbTree.SetSize(nbV);

        for (int i = 1; i <= nbV; ++i)
        {
            Vertex v = ShapeCasts.Vertex(vertices.FindKey(i));
            double tol = Tool.Tolerance(v);
            if (tol < vertices[i])
            {
                tol = vertices[i];
            }

            // Build bnd box for vertex
            Box box = new();
            box.Add(Tool.Pnt(v));
            box.SetGap(tol + tolAdd);
            bbTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
        }

        bbTree.Build();

        // Perform selection of the interfering vertices
        BOPAlgo_PairVerticesSelector pairSelector = new();
        pairSelector.SetBVHSets(bbTree, bbTree);
        pairSelector.SetSame(true);
        pairSelector.SetMapOfVerticesTolerances(vertices);
        pairSelector.SetFuzzyValue(fuzzyValue);
        pairSelector.Select();

        // Treat the selected pairs
        NCollection_LinearVector<PairSelector<NCollection_Vec3<double>>.PairIDs> pairs = pairSelector.Pairs();
        int nbPairs = pairs.Size();

        // Collect interfering pairs
        NCollection_IndexedDataMap<int, NCollection_List<int>> mili = new();

        for (int iPair = 0; iPair < nbPairs; ++iPair)
        {
            PairSelector<NCollection_Vec3<double>>.PairIDs pair = pairs.Value(iPair);
            FillMap(pair.ID1, pair.ID2, mili);
        }

        NCollection_List<NCollection_List<int>> blocks = new();
        MakeBlocks(mili, blocks);

        NCollection_List<NCollection_List<int>>.Iterator itLI = new(blocks);
        for (; itLI.More(); itLI.Next())
        {
            NCollection_List<int> li = itLI.Value();
            NCollection_List<Shape> chain = chains.Append(new NCollection_List<Shape>());

            for (NCollection_List<int>.Iterator itI = new(li); itI.More(); itI.Next())
            {
                chain.Append(vertices.FindKey(itI.Value()));
            }
        }

        // Add not interfered vertices as a chain of 1 element
        for (int i = 1; i <= nbV; ++i)
        {
            if (!mili.Contains(i))
            {
                chains.Append(new NCollection_List<Shape>()).Append(vertices.FindKey(i));
            }
        }
    }

    //=======================================================================
    // Classification of the faces relatively solids
    //=======================================================================

    /// <summary>A shape and its bounding box (OCCT's file-local BOPAlgo_ShapeBox).</summary>
    private sealed class BOPAlgo_ShapeBox
    {
        private Shape _shape = new();
        private Box _box = new();

        //! Sets the shape
        public void SetShape(in Shape s) => _shape = s;

        //! Returns the shape
        public Shape Shape() => _shape;

        //! Sets the bounding box
        public void SetBox(in Box box) => _box = box;

        //! Returns the bounding box
        public Box Box() => _box;
    }

    /// <summary>
    /// The classification of the selected faces relatively one solid, run in parallel (OCCT's file-local
    /// BOPAlgo_FillIn3DParts).
    /// </summary>
    private sealed class BOPAlgo_FillIn3DParts : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
    {
        private Solid _solid = new();                                   //! Solid
        private Box _boxS = new();                                         // Bounding box of the solid
        private NCollection_List<Shape> _ownIF = new();                 //! Own INTERNAL faces of the solid
        private readonly NCollection_List<Shape> _inFaces = new();      //! Faces classified as IN

        private BoxTree? _bbTree;                                     //! BVH tree of bounding boxes
        private NCollection_DynamicArray<BOPAlgo_ShapeBox>? _vShapeBox;        //! ShapeBoxMap

        private readonly Iterator _itF = new();                         //! Iterators
        private readonly Iterator _itW = new();

        private IntersectionContext? _context;                                    //! Context

        public BOPAlgo_FillIn3DParts()
        {
            _bbTree = null;
            _vShapeBox = null;
        }

        //! Sets the solid
        public void SetSolid(in Solid solid) => _solid = solid;

        //! Returns the solid
        public Solid Solid() => _solid;

        //! Sets the box for the solid
        public void SetBoxS(in Box box) => _boxS = box;

        //! Returns the solid's box
        public Box BoxS() => _boxS;

        //! Sets own INTERNAL faces of the solid
        public void SetOwnIF(NCollection_List<Shape> lif) => _ownIF = new NCollection_List<Shape>(lif);

        //! Returns own INTERNAL faces of the solid
        public NCollection_List<Shape> OwnIF() => _ownIF;

        //! Sets the Bounding Box tree
        public void SetBBTree(BoxTree bbTree) => _bbTree = bbTree;

        //! Sets the ShapeBox structure
        public void SetShapeBoxVector(NCollection_DynamicArray<BOPAlgo_ShapeBox> shapeBox) => _vShapeBox = shapeBox;

        //! Sets the context
        public void SetContext(IntersectionContext context) => _context = context;

        //! Returns the context
        public IntersectionContext? Context() => _context;

        //! Returns the faces classified as IN for solid
        public NCollection_List<Shape> InFaces() => _inFaces;

        //! Performs the classification
        public override void Perform()
        {
            using var psOuter = new ProgressScope(_progressRange, null, 2);
            if (UserBreak(psOuter))
            {
                return;
            }

            _inFaces.Clear();

            // 1. Select boxes of faces that are not out of aBoxS
            BoxTreeSelector selector = new();
            selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(_boxS));
            selector.SetBVHSet(_bbTree!);
            //
            if (selector.Select() == 0)
            {
                return;
            }

            NCollection_List<int> lifp = selector.Indices();

            // 2. Fill maps of edges and faces of the solid

            NCollection_DynamicArray<BOPAlgo_ShapeBox> vShapeBox = _vShapeBox!;

            NCollection_IndexedMap<Shape> mse = new(ShapeHasher.Instance), msf = new(ShapeHasher.Instance);
            Exploration.MapShapes(_solid, ShapeType.Edge, mse);
            Exploration.MapShapes(_solid, ShapeType.Face, msf);

            // Check if the Solid contains any faces
            bool bIsEmpty = msf.IsEmpty();

            // Add own internal faces of the solid into aMSF
            NCollection_List<Shape>.Iterator itLS = new(_ownIF);
            for (; itLS.More(); itLS.Next())
            {
                msf.Add(itLS.Value());
            }

            // 3. aIVec - faces to process.
            //    Filter the selected faces with faces of the solid.
            List<int> iVec = new(256);

            NCollection_List<int>.Iterator itLI = new(lifp);
            for (; itLI.More(); itLI.Next())
            {
                int nFP = itLI.Value();
                Shape fp0 = vShapeBox[nFP].Shape();
                if (!msf.Contains(fp0))
                {
                    iVec.Add(nFP);
                }
            }

            // 4. Classify faces relatively solid.
            //    Store faces that are IN mySolid into <myInFaces>

            int k, nbFP = iVec.Count;
            // Sort indices if necessary (distinct integers: any sort gives std::sort's order)
            if (nbFP > 1)
            {
                iVec.Sort();
            }

            if (bIsEmpty)
            {
                // The solid is empty as it does not contain any faces.
                // It could happen when the input solid consists of INTERNAL faces only.
                // Classification of any point relatively empty solid would always give IN status.
                // Thus, we consider all selected faces as IN without real classification.
                for (k = 0; k < nbFP; ++k)
                {
                    _inFaces.Append(vShapeBox[iVec[k]].Shape());
                }

                return;
            }

            // Prepare EF map of faces to process for building connexity blocks
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mefp = new(ShapeHasher.Instance);
            if (nbFP > 1)
            {
                for (k = 0; k < nbFP; ++k)
                {
                    MapEdgesAndFaces(vShapeBox[iVec[k]].Shape(), mefp);
                }
            }

            psOuter.Next();

            // Map of Edge-Face connection, necessary for solid classification.
            // It will be filled when first classification is performed.
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mefds = new(ShapeHasher.Instance);

            // Fence map to avoid processing of the same faces twice
            NCollection_Map<Shape> mfDone = new(ShapeHasher.Instance);

            // Per-iteration list for connexity block faces
            NCollection_List<Shape> lcbf = new();

            using var psLoop = new ProgressScope(psOuter.Next(), null, nbFP);
            for (k = 0; k < nbFP; ++k, psLoop.Next())
            {
                if (UserBreak(psLoop))
                {
                    return;
                }
                lcbf.Clear();

                int nFP = iVec[k];
                Face fp = ShapeCasts.Face(vShapeBox[nFP].Shape());
                if (!mfDone.Add(fp))
                {
                    continue;
                }

                // Make connexity blocks of faces, avoiding passing through the
                // borders of the solid. It helps to reduce significantly the
                // number of classified faces.
                // The most appropriate face for classification
                Face faceToClassify = new();
                MakeConnexityBlock(fp, mse, mefp, mfDone, lcbf, ref faceToClassify);

                if (!_boxS.IsWhole)
                {
                    // First, try fast classification of the whole block by additional
                    // check on bounding boxes - check that bounding boxes of all vertices
                    // of the block interfere with the box of the solid.
                    // If not, the faces are out.
                    bool bOut = false;
                    itLS.Initialize(lcbf);
                    for (; itLS.More() && !bOut; itLS.Next())
                    {
                        Explorer expV = new(itLS.Value(), ShapeType.Vertex);
                        for (; expV.More() && !bOut; expV.Next())
                        {
                            Vertex v = ShapeCasts.Vertex(expV.Current());
                            Box bbv = new();
                            bbv.Add(Tool.Pnt(v));
                            bbv.SetGap(Tool.Tolerance(v));
                            bOut = _boxS.IsOut(bbv);
                        }
                    }
                    if (bOut)
                    {
                        continue;
                    }
                }

                if (faceToClassify.IsNull)
                {
                    faceToClassify = fp;
                }

                if (mefds.IsEmpty())
                {
                    // Fill EF map for Solid
                    Exploration.MapShapesAndAncestors(_solid, ShapeType.Edge, ShapeType.Face, mefds);
                }

                // All vertices are interfere with the solids box, run classification.
                bool bIsIN = AlgoTools.IsInternalFace(faceToClassify,
                                                               _solid,
                                                               mefds,
                                                               global::Kerf.Engine.Foundation.Precision.Confusion(),
                                                               _context!);
                if (bIsIN)
                {
                    itLS.Initialize(lcbf);
                    for (; itLS.More(); itLS.Next())
                    {
                        _inFaces.Append(itLS.Value());
                    }
                }
            }
        }

        //! Prepares Edge-Face connection map of the given shape
        private void MapEdgesAndFaces(in Shape f,
                                      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap)
        {
            _itF.Initialize(f);
            for (; _itF.More(); _itF.Next())
            {
                Shape w = _itF.Value();
                if (w.ShapeType() != ShapeType.Wire)
                {
                    continue;
                }

                _itW.Initialize(w);
                for (; _itW.More(); _itW.Next())
                {
                    Shape e = _itW.Value();

                    int index = efMap.FindIndex(e);
                    NCollection_List<Shape> pLF = index > 0
                                                         ? efMap[index]
                                                         : efMap[efMap.Add(e, new NCollection_List<Shape>())];
                    pLF.Append(f);
                }
            }
        }

        //! Makes the connexity block of faces using the connection map
        private void MakeConnexityBlock(in Face fStart,
                                        NCollection_IndexedMap<Shape> meAvoid,
                                        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap,
                                        NCollection_Map<Shape> mfDone,
                                        NCollection_List<Shape> lcb,
                                        ref Face faceToClassify)
        {
            // Add start element
            lcb.Append(fStart);
            if (efMap.IsEmpty())
            {
                return;
            }

            NCollection_List<Shape>.Iterator itCB = new(lcb);
            for (; itCB.More(); itCB.Next())
            {
                Shape f = itCB.Value();
                _itF.Initialize(f);
                for (; _itF.More(); _itF.Next())
                {
                    Shape w = _itF.Value();
                    if (w.ShapeType() != ShapeType.Wire)
                    {
                        continue;
                    }

                    _itW.Initialize(w);
                    for (; _itW.More(); _itW.Next())
                    {
                        Edge e = ShapeCasts.Edge(_itW.Value());
                        if (meAvoid.Contains(e) || Tool.Degenerated(e))
                        {
                            if (faceToClassify.IsNull)
                            {
                                faceToClassify = ShapeCasts.Face(f);
                            }
                            continue;
                        }

                        if (!efMap.FindFromKey(e, out NCollection_List<Shape> pLF))
                        {
                            continue;
                        }
                        NCollection_List<Shape>.Iterator itLF = new(pLF);
                        for (; itLF.More(); itLF.Next())
                        {
                            Shape fToAdd = itLF.Value();
                            if (mfDone.Add(fToAdd))
                            {
                                lcb.Append(fToAdd);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Classifies the faces relatively the solids: fills <paramref name="inParts"/> with each solid and the faces
    /// IN it. The boxes of <paramref name="shapeBoxMap"/> are used where given (null: none), and the faces of
    /// <paramref name="solidsIF"/> (each solid's own internal faces; null: none) are not classified relatively
    /// their solid. Runs in parallel if <paramref name="runParallel"/>.
    /// </summary>
    public static void ClassifyFaces(NCollection_List<Shape> faces,
                                     NCollection_List<Shape> solids,
                                     bool runParallel,
                                     IntersectionContext context,
                                     NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> inParts,
                                     NCollection_DataMap<Shape, Box>? shapeBoxMap = null,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>>? solidsIF = null,
                                     ProgressRange range = default)
    {
        shapeBoxMap ??= new NCollection_DataMap<Shape, Box>(ShapeHasher.Instance);
        solidsIF ??= new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

        using var psOuter = new ProgressScope(range, null, 10);

        // Fill the vector of shape box with faces and its bounding boxes
        NCollection_DynamicArray<BOPAlgo_ShapeBox> vsb = new(256);

        NCollection_List<Shape>.Iterator itLF = new(faces);
        for (; itLF.More(); itLF.Next())
        {
            if (!psOuter.More())
            {
                return;
            }

            Shape f = itLF.Value();
            // Append face to the vector of shape box
            BOPAlgo_ShapeBox sb = vsb.Append(new BOPAlgo_ShapeBox());
            sb.SetShape(f);

            // Get bounding box for the face
            if (shapeBoxMap.Find(f, out Box pBox))
            {
                sb.SetBox(pBox);
            }
            else
            {
                // Build the bounding box
                Box box = new();
                Bounds.Add(f, ref box);
                sb.SetBox(box);
            }
        }

        // Prepare BVH tree of bounding boxes of the faces to classify
        // taking the bounding boxes from the just prepared vector
        BoxTree bbTree = new();
        int nbF = vsb.Length();
        bbTree.SetSize(nbF);
        for (int i = 0; i < nbF; ++i)
        {
            bbTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(vsb[i].Box()));
        }

        // Shake tree filler
        bbTree.Build();

        // Prepare vector of solids to classify
        NCollection_DynamicArray<BOPAlgo_FillIn3DParts> vfip = new();

        NCollection_List<Shape>.Iterator itLS = new(solids);
        for (; itLS.More(); itLS.Next())
        {
            Solid solid = ShapeCasts.Solid(itLS.Value());
            // Append solid to the vector
            BOPAlgo_FillIn3DParts fip = vfip.Append(new BOPAlgo_FillIn3DParts());
            fip.SetSolid(solid);

            // Get bounding box for the solid
            if (shapeBoxMap.Find(solid, out Box pBox))
            {
                fip.SetBoxS(pBox);
            }
            else
            {
                // Build the bounding box
                Box box = new();
                Bounds.Add(solid, ref box);
                if (!box.IsWhole)
                {
                    if (AlgoTools.IsInvertedSolid(solid))
                    {
                        box.SetWhole();
                    }
                }
                fip.SetBoxS(box);
            }

            if (solidsIF.Find(solid, out NCollection_List<Shape> pLIF))
            {
                fip.SetOwnIF(pLIF);
            }

            fip.SetBBTree(bbTree);
            fip.SetShapeBoxVector(vsb);
        }

        // Close preparation task
        psOuter.Next();
        // Set progress range for each task to be run in parallel
        int nbS = vfip.Length();
        using var psParallel = new ProgressScope(psOuter.Next(9), "Classification of faces relatively solids", nbS);
        for (int iFS = 0; iFS < nbS; ++iFS)
        {
            vfip.ChangeValue(iFS).SetProgressRange(psParallel.Next());
        }
        // Perform classification
        //================================================================
        BooleanParallel.Perform(runParallel, vfip, context);
        //================================================================
        // Analyze the results and fill the resulting map
        for (int i = 0; i < nbS; ++i)
        {
            BOPAlgo_FillIn3DParts fip = vfip[i];
            Shape s = fip.Solid();
            NCollection_List<Shape> lfIn = fip.InFaces();
            inParts.Add(s, new NCollection_List<Shape>(lfIn));
        }
    }

    /// <summary>
    /// Classifies the parts (vertices, edges and faces, or containers of them; replaced by their images in
    /// <paramref name="images"/> where they have some) relatively the solids, and adds those IN a solid into it as
    /// INTERNAL parts (the faces as internal shells). The solids are changed in place: they must be the caller's own
    /// unfrozen solids.
    /// </summary>
    public static void FillInternals(NCollection_List<Shape> solids,
                                     NCollection_List<Shape> parts,
                                     NCollection_DataMap<Shape, NCollection_List<Shape>> images,
                                     IntersectionContext context)
    {
        if (solids.IsEmpty() || parts.IsEmpty())
        {
            return;
        }

        // Map the solids to avoid classification of the own shapes of the solids
        NCollection_IndexedMap<Shape> msSolids = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(solids);
        for (; itLS.More(); itLS.Next())
        {
            Shape solid = itLS.Value();
            if (solid.ShapeType() == ShapeType.Solid)
            {
                Exploration.MapShapes(solid, ShapeType.Vertex, msSolids);
                Exploration.MapShapes(solid, ShapeType.Edge, msSolids);
                Exploration.MapShapes(solid, ShapeType.Face, msSolids);
            }
        }

        // Extract BRep elements from the given parts and
        // check them for possible splits
        NCollection_List<Shape> lPartsInput = new(parts), lParts = new();
        NCollection_List<Shape>.Iterator itLP = new(lPartsInput);
        for (; itLP.More(); itLP.Next())
        {
            Shape part = itLP.Value();
            switch (part.ShapeType())
            {
                case ShapeType.Vertex:
                case ShapeType.Edge:
                case ShapeType.Face:
                {
                    if (images.Find(part, out NCollection_List<Shape> pIm))
                    {
                        NCollection_List<Shape>.Iterator itIm = new(pIm);
                        for (; itIm.More(); itIm.Next())
                        {
                            Shape partIm = itIm.Value();
                            if (!msSolids.Contains(partIm))
                            {
                                lParts.Append(partIm);
                            }
                        }
                    }
                    else if (!msSolids.Contains(part))
                    {
                        lParts.Append(part);
                    }

                    break;
                }
                default:
                {
                    for (Iterator it = new(part); it.More(); it.Next())
                    {
                        lPartsInput.Append(it.Value());
                    }
                    break;
                }
            }
        }

        // Classify the given parts relatively solids.
        // Add edges and vertices classified as IN into solids instantly,
        // and collect faces classified as IN into a list for further shell creation

        NCollection_DataMap<Shape, NCollection_List<Shape>> inFaces = new(ShapeHasher.Instance);
        itLS.Initialize(solids);
        for (; itLS.More(); itLS.Next())
        {
            Shape solid = itLS.Value();
            if (solid.ShapeType() != ShapeType.Solid)
            {
                continue;
            }

            Solid sd = ShapeCasts.Solid(solid);

            itLP.Initialize(lParts);
            for (; itLP.More();)
            {
                Shape part = itLP.Value();
                Containment state = AlgoTools.ComputeStateByOnePoint(part, sd, global::Kerf.Engine.Foundation.Precision.Confusion(), context);
                if (state == Containment.In)
                {
                    if (part.ShapeType() == ShapeType.Face)
                    {
                        if (!inFaces.Find(sd, out NCollection_List<Shape> pFaces))
                        {
                            pFaces = inFaces.Bound(sd, new NCollection_List<Shape>());
                        }
                        pFaces.Append(part);
                    }
                    else
                    {
                        part.Orientation(Internal);
                        new BrepBuilder().Add(sd, part);
                    }
                    lParts.Remove(itLP);
                }
                else
                {
                    itLP.Next();
                }
            }
        }

        // Make shells from faces and put them into solids
        // (native iterates this map in address order; each solid gets only its own shells, so the order isn't observable)
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itM in inFaces)
        {
            Solid sd = ShapeCasts.Solid(itM.Key);
            NCollection_List<Shape> faces = itM.Value;

            Compound cf = new();
            new BrepBuilder().MakeCompound(ref cf);

            NCollection_List<Shape>.Iterator itLF = new(faces);
            for (; itLF.More(); itLF.Next())
            {
                new BrepBuilder().Add(cf, itLF.Value());
            }

            // Build blocks from the faces
            NCollection_List<Shape> lcb = new();
            AlgoTools.MakeConnexityBlocks(cf, ShapeType.Edge, ShapeType.Face, lcb);

            // Build shell from each block
            NCollection_List<Shape>.Iterator itCB = new(lcb);
            for (; itCB.More(); itCB.Next())
            {
                Shape cb = itCB.Value();

                Shell shell = new();
                new BrepBuilder().MakeShell(ref shell);
                // Add faces of the block to the shell
                Explorer expF = new(cb, ShapeType.Face);
                for (; expF.More(); expF.Next())
                {
                    Face fInt = ShapeCasts.Face(expF.Current());
                    fInt.Orientation(Internal);
                    new BrepBuilder().Add(shell, fInt);
                }

                new BrepBuilder().Add(sd, shell);
            }
        }
    }

    /// <summary>
    /// Computes the translation moving the two boxes' union to <paramref name="point"/> (its minimal corner there),
    /// for more accurate computations. Returns true if the boxes are far from the point relative to their size
    /// (<paramref name="criteria"/>), false otherwise (the transformation is then left unchanged).
    /// </summary>
    public static bool TrsfToPoint(in Box box1, in Box box2, ref Transform3D trsf, in Point3D point,
                                   double criteria = 1.0e+5)
    {
        // Unify two boxes
        Box box = box1;
        box.Add(box2);

        Xyz bCenter = (box.CornerMin().XYZ() + box.CornerMax().XYZ()) / 2.0;
        double pbDist = (point.XYZ() - bCenter).Modulus();
        if (pbDist < criteria)
        {
            return false;
        }

        double bSize = Math.Sqrt(box.SquareExtent());
        if ((bSize / pbDist) > (1.0 / criteria))
        {
            return false;
        }

        trsf.SetTranslation(new Vector3D(box.CornerMin(), point));
        return true;
    }

    /// <summary><see cref="TrsfToPoint(in Box, in Box, ref Transform3D, in Point3D, double)"/> to the origin.</summary>
    public static bool TrsfToPoint(in Box box1, in Box box2, ref Transform3D trsf) =>
        TrsfToPoint(box1, box2, ref trsf, new Point3D(0.0, 0.0, 0.0));
}
