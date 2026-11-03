// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>The topological tools of the Boolean component.</summary>
/// <remarks>
/// <para>
/// A static partial class over four files, as OCCT's class is over four .cxx files: this one (BOPTools_AlgoTools.cxx),
/// BOPTools_AlgoTools_1.cs (the tolerance corrections) and BOPTools_AlgoTools_2.cs (vertices, edges, ranges,
/// dimensions). Decision 6440.
/// </para>
/// <para>
/// Context arguments are non-null where OCCT dereferences them unchecked, nullable where OCCT passes them on to
/// functions that accept a null context. Outputs OCCT leaves unwritten on some path are <c>ref</c>.
/// </para>
/// <para>
/// Some functions write into the shapes they are given, as in OCCT (plan 2.3): <see cref="MakeEdge"/> (the
/// tolerances of its two vertices), <see cref="MakePCurve"/> (pcurves, SameParameter on the edge) and the
/// <c>UpdateVertex</c> forms (the vertex's tolerance). Ported callers give them shapes they built or copied; a frozen
/// shape throws at the first write.
/// </para>
/// </remarks>
public static partial class AlgoTools
{
    /// <summary>The tolerance increment used when updating tolerances (1e-12).</summary>
    public static double DTolerance() => 1.0e-12;

    /// <summary>
    /// Makes the connexity blocks of the elements of <paramref name="s"/> connected through sub-shapes of
    /// <paramref name="connectionType"/>, each as a list, and fills the connection map.
    /// </summary>
    public static void MakeConnexityBlocks(in Shape s,
                                           ShapeType connectionType,
                                           ShapeType elementType,
                                           NCollection_List<NCollection_List<Shape>> lcb,
                                           NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> connectionMap)
    {
        // Map shapes to find connected elements
        Exploration.MapShapesAndAncestors(s, connectionType, elementType, connectionMap);
        // Fence map
        NCollection_Map<Shape> mFence = new(ShapeHasher.Instance);

        Explorer exp = new(s, elementType);
        for (; exp.More(); exp.Next())
        {
            Shape aS = exp.Current();
            if (!mFence.Add(aS))
            {
                continue;
            }
            // The block
            NCollection_List<Shape> lBlock = new();
            // Start the block
            lBlock.Append(aS);
            // Look for connected parts
            NCollection_List<Shape>.Iterator itB = new(lBlock);
            for (; itB.More(); itB.Next())
            {
                Shape s1 = itB.Value();
                Explorer expSS = new(s1, connectionType);
                for (; expSS.More(); expSS.Next())
                {
                    Shape subS = expSS.Current();
                    NCollection_List<Shape> ls = connectionMap.FindFromKey(subS);
                    NCollection_List<Shape>.Iterator itLS = new(ls);
                    for (; itLS.More(); itLS.Next())
                    {
                        Shape s2 = itLS.Value();
                        if (mFence.Add(s2))
                        {
                            lBlock.Append(s2);
                        }
                    }
                }
            }
            // Add the block into result
            lcb.Append(lBlock);
        }
    }

    /// <summary>
    /// Makes the connexity blocks of the elements of <paramref name="s"/> connected through sub-shapes of
    /// <paramref name="connectionType"/>, each as a compound.
    /// </summary>
    public static void MakeConnexityBlocks(in Shape s,
                                           ShapeType connectionType,
                                           ShapeType elementType,
                                           NCollection_List<Shape> lcb)
    {
        NCollection_List<NCollection_List<Shape>> lBlocks = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> cMap = new(ShapeHasher.Instance);
        MakeConnexityBlocks(s, connectionType, elementType, lBlocks, cMap);

        // Make compound from each block
        NCollection_List<NCollection_List<Shape>>.Iterator itB = new(lBlocks);
        for (; itB.More(); itB.Next())
        {
            NCollection_List<Shape> lb = itB.Value();

            Compound block = new();
            new BrepBuilder().MakeCompound(ref block);
            for (NCollection_List<Shape>.Iterator it = new(lb); it.More(); it.Next())
            {
                new BrepBuilder().Add(block, it.Value());
            }

            lcb.Append(block);
        }
    }

    /// <summary>
    /// Makes the connexity blocks of the shapes of <paramref name="ls"/> connected through sub-shapes of
    /// <paramref name="connectionType"/>, and tells whether each is regular (every connection shared by exactly two
    /// elements, no element given twice).
    /// </summary>
    public static void MakeConnexityBlocks(NCollection_List<Shape> ls,
                                           ShapeType connectionType,
                                           ShapeType elementType,
                                           NCollection_List<ConnexityBlock> lcb)
    {
        BrepBuilder bb = new();
        // Make connexity blocks from start elements
        Compound cStart = new();
        bb.MakeCompound(ref cStart);

        NCollection_Map<Shape> mFence = new(ShapeHasher.Instance), mnRegular = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itL = new(ls);
        for (; itL.More(); itL.Next())
        {
            Shape s = itL.Value();
            if (mFence.Add(s))
            {
                bb.Add(cStart, s);
            }
            else
            {
                mnRegular.Add(s);
            }
        }

        NCollection_List<NCollection_List<Shape>> aLCB = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> cMap = new(ShapeHasher.Instance);
        MakeConnexityBlocks(cStart, connectionType, elementType, aLCB, cMap);

        // Save the blocks and check their regularity
        NCollection_List<NCollection_List<Shape>>.Iterator itB = new(aLCB);
        for (; itB.More(); itB.Next())
        {
            NCollection_List<Shape> block = itB.Value();

            ConnexityBlock cb = new();
            NCollection_List<Shape> lcs = cb.ChangeShapes();

            bool bRegular = true;
            for (NCollection_List<Shape>.Iterator it = new(block); it.More(); it.Next())
            {
                Shape s = it.Value();
                if (mnRegular.Contains(s))
                {
                    bRegular = false;
                    s.Orientation(Forward);
                    lcs.Append(s);
                    s.Orientation(Reversed);
                    lcs.Append(s);
                }
                else
                {
                    lcs.Append(s);
                    if (bRegular)
                    {
                        // Check if there are no multi-connected shapes
                        for (Explorer ex = new(s, connectionType); ex.More() && bRegular; ex.Next())
                        {
                            bRegular = (cMap.FindFromKey(ex.Current()).Extent() == 2);
                        }
                    }
                }
            }

            cb.SetRegular(bRegular);
            // OCCT appends a copy of the local block; the local one is not used afterwards, so it is appended itself.
            lcb.Append(cb);
        }
    }

    /// <summary>Reorients the edges of the wire for a correct ordering (connected edges oriented head to tail).</summary>
    public static void OrientEdgesOnWire(ref Shape wire)
    {
        // make vertex-edges connexity map
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> veMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(wire, ShapeType.Vertex, ShapeType.Edge, veMap);
        //
        if (veMap.IsEmpty())
        {
            return;
        }
        //
        BrepBuilder bb = new();
        // new wire
        Wire aWire = new();
        bb.MakeWire(ref aWire);
        // fence map (OCCT's default hasher: orientation included, decision 6433)
        NCollection_Map<Shape> mFence = new(EqualityComparer<Shape>.Default);
        //
        Iterator it = new(wire);
        for (; it.More(); it.Next())
        {
            Edge ec = ShapeCasts.Edge(it.Value());
            if (!mFence.Add(ec))
            {
                continue;
            }
            //
            // add edge to a wire as it is
            bb.Add(aWire, ec);
            //
            Exploration.Vertices(ec, out Vertex v1, out Vertex v2, true);
            //
            if (v1.IsSame(v2))
            {
                // closed edge, go to the next edge
                continue;
            }
            //
            // orient the adjacent edges
            for (int i = 0; i < 2; ++i)
            {
                Shape vc = i == 0 ? v1 : v2;
                //
                for (; ; )
                {
                    NCollection_List<Shape> le = veMap.FindFromKey(vc);
                    if (le.Extent() != 2)
                    {
                        // free vertex or multi-connexity, go to the next edge
                        break;
                    }
                    //
                    bool bStop = true;
                    //
                    NCollection_List<Shape>.Iterator itLE = new(le);
                    for (; itLE.More(); itLE.Next())
                    {
                        Edge en = ShapeCasts.Edge(itLE.Value());
                        if (mFence.Contains(en))
                        {
                            continue;
                        }
                        //
                        Exploration.Vertices(en, out Vertex vn1, out Vertex vn2, true);
                        if (vn1.IsSame(vn2))
                        {
                            // closed edge, go to the next edge
                            break;
                        }
                        //
                        // change orientation if necessary and go to the next edges
                        if ((i == 0 && vc.IsSame(vn2)) || (i != 0 && vc.IsSame(vn1)))
                        {
                            bb.Add(aWire, en);
                        }
                        else
                        {
                            bb.Add(aWire, en.Reversed());
                        }
                        mFence.Add(en);
                        vc = vc.IsSame(vn1) ? vn2 : vn1;
                        bStop = false;
                        break;
                    }
                    //
                    if (bStop)
                    {
                        break;
                    }
                }
            }
        }
        //
        wire = aWire;
    }

    /// <summary>
    /// Makes a new shell of the faces of <paramref name="aShell"/>, reversing faces so that each edge shared by two
    /// faces has opposite orientations in them.
    /// </summary>
    public static void OrientFacesOnShell(ref Shape aShell)
    {
        bool bIsProcessed1, bIsProcessed2;
        int i, nbE, nbF, j;
        Orientation orE1, orE2;
        Face f1x, f2x;
        Shape shellNew = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> processedFaces = new(ShapeHasher.Instance);
        BrepBuilder bb = new();
        //
        MakeContainer(ShapeType.Shell, ref shellNew);
        //
        Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, efMap);
        nbE = efMap.Extent();
        //
        // One seam edge  in aEFMap contains  2 equivalent faces.
        for (i = 1; i <= nbE; ++i)
        {
            NCollection_List<Shape> lf = efMap.ChangeFromIndex(i);
            nbF = lf.Extent();
            if (nbF > 1)
            {
                NCollection_List<Shape> lfTmp = new();
                NCollection_IndexedMap<Shape> fm = new(ShapeHasher.Instance);
                //
                NCollection_List<Shape>.Iterator it = new(lf);
                for (; it.More(); it.Next())
                {
                    Shape f = it.Value();
                    if (!fm.Contains(f))
                    {
                        fm.Add(f);
                        lfTmp.Append(f);
                    }
                }
                lf.Clear();
                lf.Assign(lfTmp);
            }
        }
        //
        // Do
        for (i = 1; i <= nbE; ++i)
        {
            Edge e = ShapeCasts.Edge(efMap.FindKey(i));
            if (Tool.Degenerated(e))
            {
                continue;
            }
            //
            NCollection_List<Shape> lf = efMap.FindFromIndex(i);
            nbF = lf.Extent();
            if (nbF != 2)
            {
                continue;
            }
            //
            // OCCT holds references into the list and reverses its items in place.
            ref Shape f1 = ref lf.ChangeFirst();
            ref Shape f2 = ref lf.ChangeLast();
            //
            bIsProcessed1 = processedFaces.Contains(f1);
            bIsProcessed2 = processedFaces.Contains(f2);
            if (bIsProcessed1 && bIsProcessed2)
            {
                continue;
            }

            if (!bIsProcessed1 && !bIsProcessed2)
            {
                processedFaces.Add(f1);
                bb.Add(shellNew, f1);
                bIsProcessed1 = !bIsProcessed1;
            }
            //
            f1x = ShapeCasts.Face(f1);
            if (bIsProcessed1)
            {
                j = processedFaces.FindIndex(f1);
                f1x = ShapeCasts.Face(processedFaces.FindKey(j));
            }
            //
            f2x = ShapeCasts.Face(f2);
            if (bIsProcessed2)
            {
                j = processedFaces.FindIndex(f2);
                f2x = ShapeCasts.Face(processedFaces.FindKey(j));
            }
            //
            orE1 = Orientation(e, f1x);
            orE2 = Orientation(e, f2x);
            //
            if (bIsProcessed1 && !bIsProcessed2)
            {
                if (orE1 == orE2)
                {
                    if (!Tool.IsClosed(e, ShapeCasts.Face(f1)) && !Tool.IsClosed(e, ShapeCasts.Face(f2)))
                    {
                        f2.Reverse();
                    }
                }
                processedFaces.Add(f2);
                bb.Add(shellNew, f2);
            }
            else if (!bIsProcessed1 && bIsProcessed2)
            {
                if (orE1 == orE2)
                {
                    if (!Tool.IsClosed(e, ShapeCasts.Face(f1)) && !Tool.IsClosed(e, ShapeCasts.Face(f2)))
                    {
                        f1.Reverse();
                    }
                }
                processedFaces.Add(f1);
                bb.Add(shellNew, f1);
            }
        }
        //
        //
        for (i = 1; i <= nbE; ++i)
        {
            Edge e = ShapeCasts.Edge(efMap.FindKey(i));
            if (Tool.Degenerated(e))
            {
                continue;
            }
            //
            NCollection_List<Shape> lf = efMap.FindFromIndex(i);
            nbF = lf.Extent();
            if (nbF != 2)
            {
                NCollection_List<Shape>.Iterator it = new(lf);
                for (; it.More(); it.Next())
                {
                    Shape f = it.Value();
                    if (!processedFaces.Contains(f))
                    {
                        processedFaces.Add(f);
                        bb.Add(shellNew, f);
                    }
                }
            }
        }
        aShell = shellNew;
    }

    // The orientation of the edge in the face (the first same edge found), INTERNAL if the face hasn't it.
    private static Orientation Orientation(in Edge anE, in Face aF)
    {
        Orientation or = Internal;

        Explorer exp = new();
        exp.Init(aF, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape ef1 = exp.Current();
            if (ef1.IsSame(anE))
            {
                or = ef1.Orientation();
                break;
            }
        }
        return or;
    }

    /// <summary>
    /// Makes the connexity block of faces of <paramref name="lfIn"/> that contains its first face, through edges
    /// not in <paramref name="meAvoid"/>, and appends its faces to <paramref name="lcb"/>.
    /// </summary>
    public static void MakeConnexityBlock(NCollection_List<Shape> lfIn,
                                          NCollection_IndexedMap<Shape> meAvoid,
                                          NCollection_List<Shape> lcb)
    {
        int nbF, nbAdd1, nbAdd, i;
        Explorer exp = new();
        NCollection_List<Shape>.Iterator it = new();
        //
        NCollection_IndexedMap<Shape> mcb = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> mAdd = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> mAdd1 = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
        //
        // 1. aMEF
        it.Initialize(lfIn);
        for (; it.More(); it.Next())
        {
            Shape f = it.Value();
            Exploration.MapShapesAndAncestors(f, ShapeType.Edge, ShapeType.Face, mef);
        }
        //
        // 2. aMCB
        Shape f1 = lfIn.First();
        mAdd.Add(f1);
        //
        for (; ; )
        {
            mAdd1.Clear();
            nbAdd = mAdd.Extent();
            for (i = 1; i <= nbAdd; ++i)
            {
                Shape f = mAdd[i];
                //
                // aMAdd1.Clear();
                exp.Init(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    if (meAvoid.Contains(e))
                    {
                        continue;
                    }
                    //
                    NCollection_List<Shape> lf = mef.FindFromKey(e);
                    it.Initialize(lf);
                    for (; it.More(); it.Next())
                    {
                        Shape fx = it.Value();
                        if (fx.IsSame(f))
                        {
                            continue;
                        }
                        if (mcb.Contains(fx))
                        {
                            continue;
                        }
                        mAdd1.Add(fx);
                    }
                } // for (; aExp.More(); aExp.Next()){
                mcb.Add(f);
            } // for (i=1; i<=aNbAdd; ++i) {
            //
            nbAdd1 = mAdd1.Extent();
            if (nbAdd1 == 0)
            {
                break;
            }
            //
            mAdd.Clear();
            for (i = 1; i <= nbAdd1; ++i)
            {
                Shape fAdd = mAdd1[i];
                mAdd.Add(fAdd);
            }
            //
        } // while(1) {

        //
        nbF = mcb.Extent();
        for (i = 1; i <= nbF; ++i)
        {
            Shape f = mcb[i];
            lcb.Append(f);
        }
    }

    /// <summary>
    /// The state of the shape relative to the solid, from one point of it: a vertex's point, an edge's intermediate
    /// point, a face's point (see the face form of <see cref="ComputeState(in Face, in Solid, double,
    /// NCollection_IndexedMap{Shape}, IntersectionContext)"/>). UNKNOWN for other types.
    /// </summary>
    public static Containment ComputeStateByOnePoint(in Shape s, in Solid theRef, double tol,
                                                      IntersectionContext context)
    {
        Containment state = Unknown;
        ShapeType type = s.ShapeType();

        switch (type)
        {
            case ShapeType.Vertex:
                state = ComputeState(ShapeCasts.Vertex(s), theRef, tol, context);
                break;
            case ShapeType.Edge:
                state = ComputeState(ShapeCasts.Edge(s), theRef, tol, context);
                break;
            case ShapeType.Face:
                {
                    NCollection_IndexedMap<Shape> bounds = new(ShapeHasher.Instance);
                    Exploration.MapShapes(theRef, ShapeType.Edge, bounds);
                    state = ComputeState(ShapeCasts.Face(s), theRef, tol, bounds, context);
                    break;
                }
            default:
                {
                    Iterator it = new(s);
                    if (it.More())
                    {
                        // OCCT discards the state of the first sub-shape, so a container gives UNKNOWN.
                        ComputeStateByOnePoint(it.Value(), theRef, tol, context);
                    }
                    break;
                }
        }
        return state;
    }

    /// <summary>
    /// The state of the face relative to the solid: the state of an edge of the face that isn't in
    /// <paramref name="bounds"/> (the solid's edges), else of a point inside the face.
    /// </summary>
    public static Containment ComputeState(in Face f, in Solid theRef, double tol,
                                            NCollection_IndexedMap<Shape> bounds, IntersectionContext context)
    {
        Containment state = Unknown;

        // Try to find the edge on the face which does not
        // belong to the solid and classify the middle point of that
        // edge relatively solid.
        Explorer exp = new(f, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge se = ShapeCasts.Edge(exp.Current());
            if (Tool.Degenerated(se))
            {
                continue;
            }

            if (!bounds.Contains(se))
            {
                state = ComputeState(se, theRef, tol, context);
                return state;
            }
        }

        // All edges of the face are on the solid.
        // Get point inside the face and classify it relatively solid.
        Point3D p3D = new();
        Point2D p2D = new();
        int iErr = AlgoTools3D.PointInFace(f, ref p3D, ref p2D, context);
        if (iErr != 0)
        {
            // Hatcher fails to find the point -> get point near some edge
            exp.Init(f, ShapeType.Edge);
            for (; exp.More() && iErr != 0; exp.Next())
            {
                Edge se = ShapeCasts.Edge(exp.Current());
                if (Tool.Degenerated(se))
                {
                    continue;
                }

                iErr = AlgoTools3D.PointNearEdge(se, f, ref p2D, ref p3D, context);
            }
        }

        if (iErr == 0)
        {
            state = ComputeState(p3D, theRef, tol, context);
        }

        return state;
    }

    /// <summary>The state of the vertex's point relative to the solid.</summary>
    public static Containment ComputeState(in Vertex v, in Solid theRef, double tol, IntersectionContext context)
    {
        Containment state;
        Point3D p3D;
        //
        p3D = Tool.Pnt(v);
        state = ComputeState(p3D, theRef, tol, context);
        return state;
    }

    /// <summary>
    /// The state of the edge's intermediate point (or of its vertex, for an edge without a 3D curve) relative to the
    /// solid.
    /// </summary>
    public static Containment ComputeState(in Edge e, in Solid theRef, double tol, IntersectionContext context)
    {
        double t = 0.0;
        Containment state;
        Curve3D? c3D;
        Point3D p3D;
        //
        c3D = Tool.Curve(e, out double t1, out double t2);
        //
        if (c3D is null)
        {
            // it means that we are in degenerated edge
            Vertex v = Exploration.FirstVertex(e);
            if (v.IsNull)
            {
                return Unknown;
            }
            p3D = Tool.Pnt(v);
        }
        else
        { // usual case
            bool bF2Inf, bL2Inf;
            double dT = 10.0;
            //
            bF2Inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(t1);
            bL2Inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(t2);
            //
            if (bF2Inf && !bL2Inf)
            {
                t = t2 - dT;
            }
            else if (!bF2Inf && bL2Inf)
            {
                t = t1 + dT;
            }
            else if (bF2Inf && bL2Inf)
            {
                t = 0.0;
            }
            else
            {
                t = IntersectionTools.IntermediatePoint(t1, t2);
            }
            c3D.D0(t, out p3D);
        }
        //
        state = ComputeState(p3D, theRef, tol, context);
        //
        return state;
    }

    /// <summary>The state of the point relative to the solid, by the context's solid classifier.</summary>
    public static Containment ComputeState(in Point3D p, in Solid theRef, double tol, IntersectionContext context)
    {
        Containment state;
        //
        SolidClassifier sc = context.SolidClassifier(theRef);
        sc.Perform(p, tol);
        //
        state = sc.State();
        //
        return state;
    }

    /// <summary>
    /// True if the face is inside the solid: by the angles between the faces at an edge of the face found in
    /// <paramref name="mef"/> (the solid's edge/face map), else by classifying a point of the face.
    /// </summary>
    public static bool IsInternalFace(in Face face,
                                      in Solid solid,
                                      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef,
                                      double tol,
                                      IntersectionContext context)
    {
        bool bDegenerated;
        Orientation or;
        Edge e1 = new();
        Explorer exp = new();
        //
        // For all invoked functions: [::IsInternalFace(...)]
        // the returned value iRet means:
        // iRet=0;  - state is not IN
        // iRet=1;  - state is IN
        // iRet=2;  - state can not be found by the method of angles
        int iRet = 0;
        // 1 Try to find an edge from theFace in theMEF
        exp.Init(face, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (!mef.Contains(e))
            {
                continue;
            }
            //
            or = e.Orientation();
            if (or == Internal)
            {
                continue;
            }
            bDegenerated = Tool.Degenerated(e);
            if (bDegenerated)
            {
                continue;
            }
            // aE
            NCollection_List<Shape> lf = mef.ChangeFromKey(e);
            int nbF = lf.Extent();
            if (nbF == 1)
            {
                // aE is internal edge on aLF.First()
                Face f1 = ShapeCasts.Face(lf.First());
                GetEdgeOnFace(e, f1, ref e1);
                if (e1.Orientation() != Internal)
                {
                    continue;
                }
                //
                iRet = IsInternalFace(face, e, f1, f1, context);
                break;
            }
            //
            else if (nbF == 2)
            {
                Face f1 = ShapeCasts.Face(lf.First());
                Face f2 = ShapeCasts.Face(lf.Last());
                iRet = IsInternalFace(face, e, f1, f2, context);
                if (iRet != 2)
                {
                    break;
                }
            }
        } // for(; aExp.More(); aExp.Next()) {
        //
        if (exp.More() && iRet != 2)
        {
            return iRet == 1;
        }
        //
        //========================================
        // 2. Classify face using classifier
        //
        Containment state;
        NCollection_IndexedMap<Shape> bounds = new(ShapeHasher.Instance);
        //
        Exploration.MapShapes(solid, ShapeType.Edge, bounds);
        //
        state = ComputeState(face, solid, tol, bounds, context);
        return state == In;
    }

    /// <summary>
    /// Whether the face is inside the faces <paramref name="lf"/> sharing <paramref name="edge"/>: 1 if IN, 0 if
    /// not, 2 if the angles can't tell.
    /// </summary>
    public static int IsInternalFace(in Face face, in Edge edge, NCollection_List<Shape> lf,
                                     IntersectionContext context)
    {
        int nbF, iRet;
        //
        iRet = 0;
        //
        nbF = lf.Extent();
        if (nbF == 2)
        {
            Face f1 = ShapeCasts.Face(lf.First());
            Face f2 = ShapeCasts.Face(lf.Last());
            iRet = IsInternalFace(face, edge, f1, f2, context);
            return iRet;
        }
        //
        else
        {
            NCollection_List<CoupleOfShape> lcff = new();
            NCollection_List<CoupleOfShape>.Iterator it = new();
            //
            FindFacePairs(edge, lf, lcff, context);
            //
            it.Initialize(lcff);
            for (; it.More(); it.Next())
            {
                CoupleOfShape csff = it.Value();
                //
                Face f1 = ShapeCasts.Face(csff.Shape1());
                Face f2 = ShapeCasts.Face(csff.Shape2());
                iRet = IsInternalFace(face, edge, f1, f2, context);
                if (iRet != 0)
                {
                    return iRet;
                }
            }
        }
        return iRet;
    }

    /// <summary>
    /// Whether the face is between <paramref name="face1"/> and <paramref name="face2"/> at
    /// <paramref name="edge"/>: 1 if IN, 0 if not, 2 if the angles can't tell.
    /// </summary>
    public static int IsInternalFace(in Face face, in Edge edge, in Face face1, in Face face2,
                                     IntersectionContext context)
    {
        Edge e1 = new(), e2 = new();
        Face fOff = new();
        NCollection_List<CoupleOfShape> theLCSOff = new();
        CoupleOfShape cs1 = new(), cs2 = new();
        //
        GetEdgeOnFace(edge, face1, ref e1);
        if (e1.Orientation() == Internal)
        {
            e2 = e1;
            e1.Orientation(Forward);
            e2.Orientation(Reversed);
        }
        else if (face1 == face2)
        {
            e2 = e1;
            e1.Orientation(Forward);
            e2.Orientation(Reversed);
        }
        else
        {
            GetEdgeOnFace(edge, face2, ref e2);
        }
        //
        cs1.SetShape1(edge);
        cs1.SetShape2(face);
        theLCSOff.Append(cs1);
        //
        cs2.SetShape1(e2);
        cs2.SetShape2(face2);
        theLCSOff.Append(cs2);
        //
        int iRet = 0; // theFace is not internal
        bool isDone = GetFaceOff(e1, face1, theLCSOff, ref fOff, context);
        if (!isDone)
        {
            // error, unable to classify face by this edge
            iRet = 2;
        }
        else if (face.IsEqual(fOff))
        {
            // theFace is internal
            iRet = 1;
        }

        return iRet;
    }

    /// <summary>
    /// Among the edge/face couples <paramref name="lcsOff"/>, finds the face <paramref name="fOff"/> making the
    /// smallest angle with <paramref name="f1"/> around the edge <paramref name="e1"/>. False if the smallest
    /// angle can't be told apart (<paramref name="fOff"/> may still be set).
    /// </summary>
    public static bool GetFaceOff(in Edge e1, in Face f1, NCollection_List<CoupleOfShape> lcsOff,
                                  ref Face fOff, IntersectionContext context)
    {
        bool bRet, bIsComputed;
        double t, angle, twoPI, angleMin, dt3D;
        Point3D px;
        Direction3D dbf, dbf2, dtf;
        Vector3D vTgt = new();
        Orientation or;
        Curve3D c3D;
        PlaneSurface pl;
        NCollection_List<CoupleOfShape>.Iterator it = new();
        ProjectPointOnSurface projPL = new();
        //
        angleMin = 100.0;
        twoPI = M_PI + M_PI;
        c3D = Tool.Curve(e1, out double t1, out double t2)!;
        t = AlgoTools2D.IntermediatePoint(t1, t2);
        c3D.D0(t, out px);
        //
        AlgoTools2D.EdgeTangent(e1, t, ref vTgt);
        Direction3D dTgt = new(vTgt), dTgt2;
        or = e1.Orientation();
        //
        pl = new PlaneSurface(px, dTgt);
        pl.Bounds(out double umin, out double usup, out double vmin, out double vsup);
        projPL.Init(pl, umin, usup, vmin, vsup);
        //
        bool bSmallFaces = false;
        dt3D = MinStep3D(e1, f1, lcsOff, px, context, ref bSmallFaces);
        bIsComputed = GetFaceDir(e1, f1, px, t, dTgt, bSmallFaces, out Direction3D dn1, out dbf, context, projPL, dt3D);
        // OCCT_DEBUG only: a message when the bi-normal direction is incorrect.
        //
        dtf = dn1 ^ dbf;
        //
        // The difference between faces should be obvious enough
        // to guarantee the correctness of the classification
        double angleCriteria = global::Kerf.Engine.Foundation.Precision.Confusion();

        bRet = true;
        it.Initialize(lcsOff);
        for (; it.More(); it.Next())
        {
            CoupleOfShape cs = it.Value();
            Edge e2 = ShapeCasts.Edge(cs.Shape1());
            Face f2 = ShapeCasts.Face(cs.Shape2());
            //
            dTgt2 = (e2.Orientation() == or) ? dTgt : dTgt.Reversed();
            bIsComputed = GetFaceDir(e2, f2, px, t, dTgt2, bSmallFaces, out _, out dbf2, context, projPL, dt3D);
            // OCCT_DEBUG only: a message when the bi-normal direction is incorrect.
            // Angle
            angle = AngleWithRef(dbf, dbf2, dtf);
            //
            if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                if (f2 == f1)
                {
                    angle = M_PI;
                }
                else if (f2.IsSame(f1))
                {
                    angle = twoPI;
                }
                else if (!bIsComputed)
                {
                    // Near-zero angle between different faces where the bi-normal direction could not be
                    // reliably computed (near-degenerate geometry, e.g. very thin face producing
                    // nearly-coincident intersection curves). Treat as maximum angle (2*PI) to prevent
                    // incorrect selection of this unreliable face over a proper closing face.
                    angle = twoPI;
                }
            }
            //
            if (Math.Abs(angle) < angleCriteria || Math.Abs(angle - angleMin) < angleCriteria)
            {
                // the minimal angle can not be found
                bRet = false;
            }
            //
            if (angle < 0.0)
            {
                angle = twoPI + angle;
            }
            //
            if (angle < angleMin)
            {
                angleMin = angle;
                fOff = f2;
            }
        }
        return bRet;
    }

    /// <summary>Finds in the face the edge same as <paramref name="e1"/> with the opposite orientation.</summary>
    public static bool GetEdgeOff(in Edge e1, in Face f2, ref Edge e2)
    {
        bool bFound;
        Orientation or1, or1C, or2;
        Explorer exp = new();
        //
        bFound = false;
        or1 = e1.Orientation();
        or1C = global::Kerf.Engine.Topology.TopAbs.Reverse(or1);
        //
        exp.Init(f2, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge ef2 = ShapeCasts.Edge(exp.Current());
            if (ef2.IsSame(e1))
            {
                or2 = ef2.Orientation();
                if (or2 == or1C)
                {
                    e2 = ef2;
                    bFound = !bFound;
                    return bFound;
                }
            }
        }
        return bFound;
    }

    /// <summary>
    /// True if the faces coincide: a point inside the first face is valid for the second, within the faces' and their
    /// edges' tolerances and the fuzzy value.
    /// </summary>
    public static bool AreFacesSameDomain(in Face f1, in Face f2, IntersectionContext context) =>
        AreFacesSameDomain(f1, f2, context, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="AreFacesSameDomain(in Face, in Face, IntersectionContext)"/>
    public static bool AreFacesSameDomain(in Face f1, in Face f2, IntersectionContext context, double fuzz)
    {
        bool bFacesSD = false;

        // The idea is to find a point inside the first face
        // and check its validity for the second face.
        // If valid - the faces are same domain.

        Point3D p1 = new();
        Point2D p2D1 = new();
        // Find point inside the first face
        int iErr = AlgoTools3D.PointInFace(f1, ref p1, ref p2D1, context);

        if (iErr != 0)
        {
            // unable to find the point
            return bFacesSD;
        }

        // Check validity of the point for second face

        // Compute the tolerance to check the validity -
        // sum of tolerance of faces and fuzzy tolerance

        // Compute the tolerance of the faces, taking into account the deviation
        // of the edges from the surfaces
        double tolF1 = Tool.Tolerance(f1), tolF2 = Tool.Tolerance(f2);

        // Find maximal tolerance of edges.
        // The faces should have the same boundaries, thus
        // it does not matter which face to explore.
        {
            double tolEMax = -1.0;
            Explorer expE = new(f1, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Edge e = ShapeCasts.Edge(expE.Current());
                if (!Tool.Degenerated(e))
                {
                    double tolE = Tool.Tolerance(e);
                    if (tolE > tolEMax)
                    {
                        tolEMax = tolE;
                    }
                }
            }
            if (tolEMax > tolF1)
            {
                tolF1 = tolEMax;
            }
            if (tolEMax > tolF2)
            {
                tolF2 = tolEMax;
            }
        }

        // Checking criteria
        double tol = tolF1 + tolF2 + Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Project and classify the point on second face
        bFacesSD = context.IsValidPointForFace(p1, f2, tol);

        return bFacesSD;
    }

    /// <summary>
    /// The relative sense of two faces sharing an edge: 0 if they share no (non-degenerated, non-seam) edge, else
    /// <see cref="AlgoTools3D.SenseFlag"/> of their normals at it.
    /// </summary>
    public static int Sense(in Face f1, in Face f2, IntersectionContext context)
    {
        int iSense = 0;
        Direction3D dnf1, dnf2;
        Edge e1 = new(), e2 = new();
        Explorer exp = new();
        //
        exp.Init(f1, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            e1 = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(e1))
            {
                if (!Tool.IsClosed(e1, f1))
                {
                    break;
                }
            }
        }
        //
        exp.Init(f2, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            e2 = ShapeCasts.Edge(exp.Current());
            if (!Tool.Degenerated(e2))
            {
                if (!Tool.IsClosed(e2, f2))
                {
                    if (e2.IsSame(e1))
                    {
                        iSense = 1;
                        break;
                    }
                }
            }
        }
        //
        if (iSense == 0)
        {
            return iSense;
        }
        //
        AlgoTools3D.GetNormalToFaceOnEdge(e1, f1, out dnf1, context);
        AlgoTools3D.GetNormalToFaceOnEdge(e2, f2, out dnf2, context);
        //
        iSense = AlgoTools3D.SenseFlag(dnf1, dnf2);
        //
        return iSense;
    }

    /// <summary>
    /// True if the split (an edge or a face) must be reversed to have the orientation of the original shape it was
    /// split from. Other types give false.
    /// </summary>
    public static bool IsSplitToReverse(in Shape sp, in Shape sr, IntersectionContext? context) =>
        IsSplitToReverse(sp, sr, context, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Shape, in Shape, IntersectionContext?)"/>, with the error status
    /// (0 if fine, 100 for an unsupported type; the face and edge forms give their own codes). OCCT's
    /// <c>int* theError</c>.
    /// </summary>
    public static bool IsSplitToReverse(in Shape sp, in Shape sr, IntersectionContext? context, out int error)
    {
        bool bRet;
        ShapeType type;
        //
        bRet = false;
        error = 0; // the edge and face forms always set it
        //
        type = sp.ShapeType();
        switch (type)
        {
            case ShapeType.Edge:
                {
                    Edge eSp = ShapeCasts.Edge(sp);
                    Edge eSr = ShapeCasts.Edge(sr);
                    bRet = IsSplitToReverse(eSp, eSr, context, out error);
                }
                break;
            //
            case ShapeType.Face:
                {
                    Face fSp = ShapeCasts.Face(sp);
                    Face fSr = ShapeCasts.Face(sr);
                    bRet = IsSplitToReverse(fSp, fSr, context, out error);
                }
                break;
            //
            default:
                error = 100;
                break;
        }
        return bRet;
    }

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Shape, in Shape, IntersectionContext?)"/>; when the check fails,
    /// adds a <see cref="AlertUnableToOrientTheShape"/> warning (with both shapes) to the report.
    /// </summary>
    public static bool IsSplitToReverseWithWarn(in Shape split, in Shape shape, IntersectionContext? context,
                                                Report? report = null)
    {
        bool isToReverse = IsSplitToReverse(split, shape, context, out int err);
        if (err != 0 && report is not null)
        {
            // The error occurred during the check.
            // Add warning to the report, storing the shapes into the warning.
            Compound wc = new();
            new BrepBuilder().MakeCompound(ref wc);
            new BrepBuilder().Add(wc, split);
            new BrepBuilder().Add(wc, shape);
            report.AddAlert(Warning, new AlertUnableToOrientTheShape(wc));
        }
        return isToReverse;
    }

    /// <summary>
    /// True if the split face must be reversed: its normal at a point inside it points against the original face's
    /// normal at the projection of that point (orientations included).
    /// </summary>
    public static bool IsSplitToReverse(in Face fSp, in Face fSr, IntersectionContext? context) =>
        IsSplitToReverse(fSp, fSr, context, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Face, in Face, IntersectionContext?)"/>, with the error status: 0
    /// fine, 1 no point found in the split, 2 no normal on the split, 3 the projection failed, 4 no normal on the
    /// original.
    /// </summary>
    public static bool IsSplitToReverse(in Face fSp, in Face fSr, IntersectionContext? context, out int error)
    {
        // Set OK error status
        error = 0;

        // Compare surfaces
        Surface sfSp = Tool.Surface(fSp)!;
        Surface sfOr = Tool.Surface(fSr)!;
        if (ReferenceEquals(sfSp, sfOr))
        {
            return fSp.Orientation() != fSr.Orientation();
        }
        //
        bool bDone = false;
        // Find the point inside the split face
        Point3D pfSp = new();
        Point2D p2DFSp = new();
        //
        // Error status
        int iErr;
        // Use the hatcher to find the point in the middle of the face
        iErr = AlgoTools3D.PointInFace(fSp, ref pfSp, ref p2DFSp, context!);
        if (iErr != 0)
        {
            // Hatcher has failed to find a point.
            // Try to get the point near some not closed and
            // not degenerated edge on the split face.
            Explorer exp = new(fSp, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge eSp = ShapeCasts.Edge(exp.Current());
                if (!Tool.Degenerated(eSp) && !Tool.IsClosed(eSp, fSp))
                {
                    iErr = AlgoTools3D.PointNearEdge(eSp, fSp, ref p2DFSp, ref pfSp, context!);
                    if (iErr == 0)
                    {
                        break;
                    }
                }
            }
            //
            if (!exp.More())
            {
                error = 1;
                // The point has not been found.
                return bDone;
            }
        }
        //
        // Compute normal direction of the split face
        Direction3D dnfSp = new();
        bDone = AlgoTools3D.GetNormalToSurface(sfSp, p2DFSp.X(), p2DFSp.Y(), ref dnfSp);
        if (!bDone)
        {
            error = 2;
            return bDone;
        }
        //
        if (fSp.Orientation() == Reversed)
        {
            dnfSp = dnfSp.Reversed();
        }
        //
        // Project the point from the split face on the original face
        // to find its UV coordinates
        ProjectPointOnSurface projector = context!.ProjPS(fSr);
        projector.Perform(pfSp);
        bDone = (projector.NbPoints() > 0);
        if (!bDone)
        {
            error = 3;
            return bDone;
        }
        // UV coordinates of the point on the original face
        projector.LowerDistanceParameters(out double u, out double v);
        //
        // Compute normal direction for the original face in this point
        Direction3D dnfOr = new();
        bDone = AlgoTools3D.GetNormalToSurface(sfOr, u, v, ref dnfOr);
        if (!bDone)
        {
            error = 4;
            return bDone;
        }
        //
        if (fSr.Orientation() == Reversed)
        {
            dnfOr = dnfOr.Reversed();
        }
        //
        // compare the normals
        double cos = dnfSp * dnfOr;
        return (cos < 0.0);
    }

    /// <summary>
    /// True if the split edge must be reversed: its tangent at a point inside it points against the original edge's
    /// tangent at the projection of that point (orientations included).
    /// </summary>
    public static bool IsSplitToReverse(in Edge eSp, in Edge eOr, IntersectionContext? context) =>
        IsSplitToReverse(eSp, eOr, context, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Edge, in Edge, IntersectionContext?)"/>, with the error status: 0
    /// fine, 1 a degenerated edge, else the last failure over the samples (2 no tangent on the split, 3 the projection
    /// failed, 4 no tangent on the original).
    /// </summary>
    public static bool IsSplitToReverse(in Edge eSp, in Edge eOr, IntersectionContext? context, out int error)
    {
        // The idea is to compare the tangent vectors of two edges computed in
        // the same point. Thus, we need to take the point on split edge (since it is
        // shorter) and project it onto original edge to find corresponding parameter.

        if (Tool.Degenerated(eSp) || Tool.Degenerated(eOr))
        {
            error = 1;
            return false;
        }

        // Set OK error status
        error = 0;

        // Get the curves from the edges
        Curve3D? cSp = Tool.Curve(eSp, out double f, out double l);
        Curve3D? cOr = Tool.Curve(eOr, out f, out l);

        // If the curves are the same, compare orientations only
        if (ReferenceEquals(cSp, cOr))
        {
            return eSp.Orientation() != eOr.Orientation();
        }

        // Find valid range of the split edge, to ensure that the point for computing
        // tangent vectors will be inside both edges.
        if (!BrepLibrary.FindValidRange(eSp, ref f, ref l))
        {
            Tool.Range(eSp, out f, out l);
        }

        // Error code
        int err = 0;
        // Try a few sample points on the split edge until first valid found
        const int nbP = 11;
        double dt = (l - f) / nbP;
        for (int i = 1; i < nbP; ++i)
        {
            double tm = f + i * dt;
            // Compute tangent vector on split edge
            Vector3D vSpTgt = new();
            if (!AlgoTools2D.EdgeTangent(eSp, tm, ref vSpTgt))
            {
                // Unable to compute the tangent vector on the split edge
                // in this point -> take the next point
                err = 2;
                continue;
            }

            // Find corresponding parameter on the original edge
            double tmOr = 0.0;
            if (!context!.ProjectPointOnEdge(cSp!.Value(tm), eOr, ref tmOr))
            {
                // Unable to project the point inside the split edge
                // onto the original edge -> take the next point
                err = 3;
                continue;
            }

            // Compute tangent vector on original edge
            Vector3D vOrTgt = new();
            if (!AlgoTools2D.EdgeTangent(eOr, tmOr, ref vOrTgt))
            {
                // Unable to compute the tangent vector on the original edge
                // in this point -> take the next point
                err = 4;
                continue;
            }

            // Compute the Dot product
            double cos = vSpTgt.Dot(vOrTgt);
            return (cos < 0.0);
        }

        error = err;

        return false;
    }

    /// <summary>
    /// True if the wire is a hole in the face: the signed area its pcurves sweep (sampled) is positive.
    /// </summary>
    public static bool IsHole(in Shape aW, in Shape aFace)
    {
        bool bIsHole;
        int i, nbS;
        double s;
        double u1, u, dU;
        double x1, y1, x0, y0;
        Orientation or;

        Point2D p2D0, p2D1;
        Curve2D? c2D;
        Face f, ff;
        Iterator itW = new();
        //
        bIsHole = false;
        //
        f = ShapeCasts.Face(aFace);
        ff = f;
        ff.Orientation(Forward);
        //
        s = 0.0;
        itW.Initialize(aW);
        for (; itW.More(); itW.Next())
        {
            Edge e = ShapeCasts.Edge(itW.Value());
            or = e.Orientation();
            if (or != Forward && or != Reversed)
            {
                continue;
            }
            //
            c2D = Tool.CurveOnSurface(e, ff, out double t1, out double t2);
            if (c2D is null)
            {
                break; // xx
            }
            //
            EdgeCurve2DAdaptor bac2D = new(e, ff);
            nbS = Geom2dCurveTool.NbSamples(bac2D);
            if (nbS > 2)
            {
                nbS *= 4;
            }
            //
            dU = (t2 - t1) / (double)(nbS - 1);
            u = t1;
            u1 = t1;
            if (or == Reversed)
            {
                u = t2;
                u1 = t2;
                dU = -dU;
            }
            //
            bac2D.D0(u, out p2D0);
            for (i = 2; i <= nbS; i++)
            {
                u = u1 + (i - 1) * dU;
                bac2D.D0(u, out p2D1);
                p2D0.Coord(out x0, out y0);
                p2D1.Coord(out x1, out y1);
                //
                s = s + (y0 + y1) * (x1 - x0);
                //
                p2D0 = p2D1;
            }
        } // for (; aItW.More(); aItW.Next()) {
        bIsHole = (s > 0.0);
        return bIsHole;
    }

    /// <summary>
    /// Makes an empty container of the type (compound, compsolid, solid, shell or wire); other types leave
    /// <paramref name="c"/> unchanged.
    /// </summary>
    public static void MakeContainer(ShapeType type, ref Shape c)
    {
        BrepBuilder bb = new();
        //
        switch (type)
        {
            case ShapeType.Compound:
                {
                    Compound aC = new();
                    bb.MakeCompound(ref aC);
                    c = aC;
                }
                break;
            //
            case ShapeType.CompSolid:
                {
                    CompSolid cs = new();
                    bb.MakeCompSolid(ref cs);
                    c = cs;
                }
                break;
            //
            case ShapeType.Solid:
                {
                    Solid solid = new();
                    bb.MakeSolid(ref solid);
                    c = solid;
                }
                break;
            //
            //
            case ShapeType.Shell:
                {
                    Shell shell = new();
                    bb.MakeShell(ref shell);
                    c = shell;
                }
                break;
            //
            case ShapeType.Wire:
                {
                    Wire wire = new();
                    bb.MakeWire(ref wire);
                    c = wire;
                }
                break;
            //
            default:
                break;
        }
    }

    /// <summary>
    /// Adds to the edge its pcurves on the faces where asked: the intersection curve's own pcurve, or one built,
    /// adjusted to the face's periods; then makes the edge same-parameter. Writes into <paramref name="aE"/>.
    /// </summary>
    public static void MakePCurve(in Edge aE, in Face aF1, in Face aF2, in IntersectionCurve aIC, bool bPC1, bool bPC2,
                                  IntersectionContext? context = null)
    {
        int i;
        double tolE;
        Curve2D? c2D, c2DA, c2Dx1;
        Face ffwd;
        BrepBuilder bb = new();
        bool bPC;
        //
        tolE = Tool.Tolerance(aE);
        //
        Curve3D c3DE = Tool.Curve(aE, out double t1, out double t2)!;
        TrimmedCurve3D c3DETrim = new(c3DE, t1, t2);
        //
        for (i = 0; i < 2; ++i)
        {
            bPC = i == 0 ? bPC1 : bPC2;
            if (!bPC)
            {
                continue;
            }
            //
            if (i == 0)
            {
                ffwd = aF1;
                c2Dx1 = aIC.FirstCurve2d();
            }
            else
            {
                ffwd = aF2;
                c2Dx1 = aIC.SecondCurve2d();
            }
            //
            ffwd.Orientation(Forward);
            //
            c2D = c2Dx1;
            if (c2D is null)
            {
                AlgoTools2D.BuildPCurveForEdgeOnFace(aE, ffwd, context);
                AlgoTools2D.CurveOnSurface(aE, ffwd, out c2D, out _, out _, out _, context);
            }
            //
            if (c3DE.IsPeriodic)
            {
                AlgoTools2D.AdjustPCurveOnFace(ffwd, t1, t2, c2D!, out c2DA, context);
            }
            else
            {
                AlgoTools2D.AdjustPCurveOnFace(ffwd, c3DETrim, c2D!, out c2DA, context);
            }
            //
            bb.UpdateEdge(aE, c2DA, ffwd, tolE);
            // BRepLib::SameParameter(aE);
        }
        BrepLibrary.SameParameter(aE);
    }

    /// <summary>
    /// Makes the section edge of the intersection curve between the vertices (<see cref="MakeSectEdge"/>) with the
    /// tolerance <paramref name="tolR3D"/>, raising the vertices' tolerances to it first. Writes into the vertices.
    /// </summary>
    public static void MakeEdge(in IntersectionCurve ic, in Vertex v1, double t1, in Vertex v2, double t2,
                                double tolR3D, out Edge e)
    {
        BrepBuilder bb = new();
        double needTol = tolR3D + DTolerance();
        //
        bb.UpdateVertex(v1, needTol);
        bb.UpdateVertex(v2, needTol);
        //
        MakeSectEdge(ic, v1, t1, v2, t2, out e);
        //
        bb.UpdateEdge(e, tolR3D);
    }

    /// <summary>0 if the point is within the vertex's tolerance plus <paramref name="aTolP2"/> (and Confusion), else 1.</summary>
    public static int ComputeVV(in Vertex aV1, in Point3D aP2, double aTolP2)
    {
        double tolV1, tolSum, tolSum2, d2;
        Point3D p1;
        //
        tolV1 = Tool.Tolerance(aV1);

        tolSum = tolV1 + aTolP2 + global::Kerf.Engine.Foundation.Precision.Confusion();
        tolSum2 = tolSum * tolSum;
        //
        p1 = Tool.Pnt(aV1);
        //
        d2 = p1.SquareDistance(aP2);
        if (d2 > tolSum2)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>0 if the vertices are within the sum of their tolerances and the fuzzy value, else 1.</summary>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2) => ComputeVV(aV1, aV2, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVV(in Vertex, in Vertex)"/>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2, double aFuzz)
    {
        double tolV1, tolV2, tolSum, tolSum2, d2;
        Point3D p1, p2;
        double fuzz1 = (aFuzz > global::Kerf.Engine.Foundation.Precision.Confusion() ? aFuzz : global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        tolV1 = Tool.Tolerance(aV1);
        tolV2 = Tool.Tolerance(aV2);
        tolSum = tolV1 + tolV2 + fuzz1;
        tolSum2 = tolSum * tolSum;
        //
        p1 = Tool.Pnt(aV1);
        p2 = Tool.Pnt(aV2);
        //
        d2 = p1.SquareDistance(p2);
        if (d2 > tolSum2)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>
    /// A vertex covering the vertices of the list: the vertex itself for one, a new bounding vertex
    /// (<see cref="BrepLibrary.BoundingVertex"/>) for several; an empty list leaves <paramref name="aVnew"/> unchanged.
    /// </summary>
    public static void MakeVertex(NCollection_List<Shape> aLV, ref Vertex aVnew)
    {
        int nb = aLV.Extent();
        if (nb == 1)
        {
            aVnew = ShapeCasts.Vertex(aLV.First());
        }
        else if (nb > 1)
        {
            double nTol = 0.0;
            Point3D nc = new();
            BrepLibrary.BoundingVertex(aLV, ref nc, ref nTol);
            BrepBuilder bb = new();
            bb.MakeVertex(ref aVnew, nc, nTol);
        }
    }

    /// <summary>Finds in the face the edge same as <paramref name="e1"/> (any orientation), through its wires.</summary>
    public static bool GetEdgeOnFace(in Edge e1, in Face f2, ref Edge e2)
    {
        bool bFound;
        Iterator itF = new(), itW = new();
        //
        bFound = false;
        //
        itF.Initialize(f2);
        for (; itF.More(); itF.Next())
        {
            Shape w = itF.Value();
            itW.Initialize(w);
            for (; itW.More(); itW.Next())
            {
                Shape e = itW.Value();
                if (e.IsSame(e1))
                {
                    e2 = ShapeCasts.Edge(e);
                    bFound = !bFound;
                    return bFound;
                }
            }
        }
        return bFound;
    }

    // Pairs the faces of thLF sharing theE (each with the face off it); false if the edge isn't on one of them.
    private static bool FindFacePairs(in Edge e, NCollection_List<Shape> thLF,
                                      NCollection_List<CoupleOfShape> lcff, IntersectionContext context)
    {
        bool bFound;
        int i, nbCEF;
        Orientation or, orC = Forward;
        NCollection_Map<Shape> mfp = new(ShapeHasher.Instance);
        Face f1 = new(), f2 = new();
        Edge el = new(), e1 = new();
        NCollection_List<Shape>.Iterator itLF = new();
        CoupleOfShape cef = new(), cff = new();
        NCollection_List<CoupleOfShape> lcef = new(), lceFx = new();
        NCollection_List<CoupleOfShape>.Iterator it = new();
        //
        bFound = true;
        //
        // Preface aLCEF
        itLF.Initialize(thLF);
        for (; itLF.More(); itLF.Next())
        {
            Face fl = ShapeCasts.Face(itLF.Value());
            //
            bFound = GetEdgeOnFace(e, fl, ref el);
            if (!bFound)
            {
                return bFound; // it can not be so
            }
            //
            cef.SetShape1(el);
            cef.SetShape2(fl);
            lcef.Append(cef);
        }
        //
        nbCEF = lcef.Extent();
        while (nbCEF != 0)
        {
            //
            // aLCEFx
            lceFx.Clear();
            it.Initialize(lcef);
            for (i = 0; it.More(); it.Next(), ++i)
            {
                CoupleOfShape cSx = it.Value();
                Shape ex = cSx.Shape1();
                Shape fx = cSx.Shape2();
                //
                or = ex.Orientation();
                //
                if (i == 0)
                {
                    orC = global::Kerf.Engine.Topology.TopAbs.Reverse(or);
                    e1 = ShapeCasts.Edge(ex);
                    f1 = ShapeCasts.Face(fx);
                    mfp.Add(fx);
                    continue;
                }
                //
                if (or == orC)
                {
                    lceFx.Append(cSx);
                    mfp.Add(fx);
                }
            }
            //
            // F2
            GetFaceOff(e1, f1, lceFx, ref f2, context);
            //
            cff.SetShape1(f1);
            cff.SetShape2(f2);
            lcff.Append(cff);
            //
            mfp.Add(f1);
            mfp.Add(f2);
            //
            // refine aLCEF
            lceFx.Clear();
            lceFx.Assign(lcef);
            lcef.Clear();
            it.Initialize(lceFx);
            for (; it.More(); it.Next())
            {
                CoupleOfShape cSx = it.Value();
                Shape fx = cSx.Shape2();
                if (!mfp.Contains(fx))
                {
                    lcef.Append(cSx);
                }
            }
            //
            nbCEF = lcef.Extent();
        } // while(aNbCEF) {
        //
        return bFound;
    }

    // The angle from theD1 to theD2, signed by the reference direction (a pseudo-angle: pi/2 (1 - cos)).
    private static double AngleWithRef(in Direction3D d1, in Direction3D d2, in Direction3D dRef)
    {
        double cosinus, sinus, beta, halfPI, scPr;
        Xyz xyz;
        //
        halfPI = 0.5 * M_PI;
        //
        Xyz xyz1 = d1.XYZ();
        Xyz xyz2 = d2.XYZ();
        xyz = xyz1.Crossed(xyz2);
        sinus = xyz.Modulus();
        cosinus = d1 * d2;
        //
        beta = 0.0;
        if (sinus >= 0.0)
        {
            beta = halfPI * (1.0 - cosinus);
        }
        else
        {
            beta = 2.0 * M_PI - halfPI * (3.0 + cosinus);
        }
        //
        scPr = xyz.Dot(dRef.XYZ());
        if (scPr < 0.0)
        {
            beta = -beta;
        }
        return beta;
    }

    /// <summary>
    /// True if the part of the edge in the range (shrunk by 0.75% at each end) lies in or on the face: its two end
    /// points and its intermediate point project in or on the face, the intermediate one within the tolerances.
    /// </summary>
    public static bool IsBlockInOnFace(in ParameterRange aShrR, in Face aF, in Edge aE1, IntersectionContext aContext)
    {
        bool bFlag;
        double uld, vld;
        Point2D p2D = new();
        //
        aShrR.Range(out double f1, out double l1);
        double dt = 0.0075, k; // dt=0.001,  k;
        k = dt * (l1 - f1);
        f1 = f1 + k;
        l1 = l1 - k;
        //
        // Treatment P11
        PointOnEdge(aE1, f1, out Point3D p11);
        //
        ProjectPointOnSurface projector = aContext.ProjPS(aF);
        projector.Perform(p11);
        //
        bFlag = projector.IsDone;
        if (!bFlag)
        {
            return bFlag;
        }

        projector.LowerDistanceParameters(out uld, out vld);
        p2D = p2D.WithCoord(uld, vld);
        //
        bFlag = aContext.IsPointInOnFace(aF, p2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        //
        // Treatment P12
        PointOnEdge(aE1, l1, out Point3D p12);
        //
        projector.Perform(p12);
        //
        bFlag = projector.IsDone;
        if (!bFlag)
        {
            return bFlag;
        }

        projector.LowerDistanceParameters(out uld, out vld);
        p2D = p2D.WithCoord(uld, vld);
        //
        bFlag = aContext.IsPointInOnFace(aF, p2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        //

        // Treatment intermediate
        double m1, tolF, tolE, tol, dist;
        m1 = IntersectionTools.IntermediatePoint(f1, l1);
        PointOnEdge(aE1, m1, out p12);
        //
        projector.Perform(p12);
        //
        bFlag = projector.IsDone;
        if (!bFlag)
        {
            return bFlag;
        }
        //
        tolE = Tool.Tolerance(aE1);
        tolF = Tool.Tolerance(aF);
        tol = tolE + tolF;
        dist = projector.LowerDistance();
        if (dist > tol)
        {
            return false;
        }

        projector.LowerDistanceParameters(out uld, out vld);
        p2D = p2D.WithCoord(uld, vld);
        //
        bFlag = aContext.IsPointInOnFace(aF, p2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        return bFlag;
    }

    /// <summary>
    /// True if the edge is a micro edge: degenerated, without geometry, or too small for a shrunk range
    /// (<see cref="EdgeShrunkRange"/>), or, if <paramref name="bCheckSplittable"/>, not splittable.
    /// </summary>
    public static bool IsMicroEdge(in Edge aE, IntersectionContext? aCtx, bool bCheckSplittable = true)
    {
        bool bRet;
        double t1, t2, tmp;
        //
        bRet = (Tool.Degenerated(aE) || !Tool.IsGeometric(aE));
        if (bRet)
        {
            return bRet;
        }
        //
        _ = Tool.Curve(aE, out t1, out t2);
        Exploration.Vertices(aE, out Vertex v1, out Vertex v2);
        t1 = Tool.Parameter(v1, aE);
        t2 = Tool.Parameter(v2, aE);
        if (t2 < t1)
        {
            tmp = t1;
            t1 = t2;
            t2 = tmp;
        }
        //
        EdgeShrunkRange sr = new();
        sr.SetContext(aCtx);
        sr.SetData(aE, t1, t2, v1, v2);
        sr.Perform();
        bRet = !sr.IsDone;
        if (!bRet && bCheckSplittable)
        {
            bRet = !sr.IsSplittable;
        }
        //
        return bRet;
    }

    // The bi-normal direction aDB of the face at the point aP of the edge (and its normal aDN).
    private static bool GetFaceDir(in Edge aE,
                                   in Face aF,
                                   in Point3D aP,
                                   double aT,
                                   in Direction3D aDTgt,
                                   bool smallFaces,
                                   out Direction3D aDN,
                                   out Direction3D aDB,
                                   IntersectionContext context,
                                   ProjectPointOnSurface aProjPL,
                                   double aDt)
    {
        double tolE;
        Point3D px = new();
        //
        AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, aT, out aDN, context);
        if (aF.Orientation() == Reversed)
        {
            aDN = aDN.Reversed();
        }
        //
        tolE = Tool.Tolerance(aE);
        aDB = aDN ^ aDTgt;
        //
        // do not try to look for the point in the small face by intersecting
        // it with the circle because, most likely, the intersection point will
        // be out of the face
        bool bFound = !smallFaces && FindPointInFace(aF, aP, ref aDB, ref px, context, aProjPL, aDt, tolE);
        if (!bFound)
        {
            // if the first method did not succeed, try to use hatcher to find the point
            bFound = AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, aDt, ref px, ref aDN, context);
            aProjPL.Perform(px);
            // Standard_ASSERT_RETURN: returns false in release builds.
            if (!aProjPL.IsDone)
            {
                return false;
            }
            px = aProjPL.NearestPoint();
            Vector3D vec = new(aP, px);
            aDB = aDB.WithXYZ(vec.XYZ());
        }
        //
        return bFound;
    }

    // Finds a point in the face in the direction aDB: intersects the face with the circle of radius aDt at aP in the
    // plane of aProjPL, iterating projections.
    private static bool FindPointInFace(in Face aF,
                                        in Point3D aP,
                                        ref Direction3D aDB,
                                        ref Point3D aPOut,
                                        IntersectionContext context,
                                        ProjectPointOnSurface aProjPL,
                                        double aDt,
                                        double aTolE)
    {
        int nbItMax;
        double dist, dTol, pm, eps;
        bool bRet;
        Point3D p1 = new(), ps;
        //
        dTol = global::Kerf.Engine.Foundation.Precision.Angular();
        pm = aP.XYZ().Modulus();
        if (pm > 1000.0)
        {
            dTol = 5.0e-16 * pm;
        }
        bRet = false;
        nbItMax = 15;
        eps = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        //
        ProjectPointOnSurface proj = context.ProjPS(aF);
        //
        ps = aP;
        proj.Perform(ps);
        if (!proj.IsDone)
        {
            return bRet;
        }
        ps = proj.NearestPoint();
        aProjPL.Perform(ps);
        ps = aProjPL.NearestPoint();
        //
        ps = ps.WithXYZ(ps.XYZ() + 2.0 * aTolE * aDB.XYZ());
        proj.Perform(ps);
        if (!proj.IsDone)
        {
            return bRet;
        }
        ps = proj.NearestPoint();
        aProjPL.Perform(ps);
        ps = aProjPL.NearestPoint();
        //
        do
        {
            p1 = p1.WithXYZ(ps.XYZ() + aDt * aDB.XYZ());
            //
            proj.Perform(p1);
            if (!proj.IsDone)
            {
                return bRet;
            }
            aPOut = proj.NearestPoint();
            dist = proj.LowerDistance();
            //
            aProjPL.Perform(aPOut);
            aPOut = aProjPL.NearestPoint();
            //
            Vector3D v = new(ps, aPOut);
            if (v.SquareMagnitude() < eps)
            {
                return bRet;
            }
            aDB = aDB.WithXYZ(v.XYZ());
        } while (dist > dTol && --nbItMax != 0);
        //
        bRet = dist < dTol;
        return bRet;
    }

    // The 3D step for GetFaceDir, and whether some face is too small for it.
    private static double MinStep3D(in Edge e1,
                                    in Face f1,
                                    NCollection_List<CoupleOfShape> lcs,
                                    in Point3D aP,
                                    IntersectionContext context,
                                    ref bool smallFaces)
    {
        double dt, tolE, tolF, dtMax, dtMin;
        //
        // add the current pair of edge/face for checking as well
        CoupleOfShape cs1 = new();
        cs1.SetShape1(e1);
        cs1.SetShape2(f1);
        //
        NCollection_List<CoupleOfShape> aLCS = new(lcs);
        aLCS.Append(cs1);
        //
        tolE = Tool.Tolerance(e1);
        dtMax = -1.0;
        dtMin = 5.0e-6;
        //
        NCollection_List<CoupleOfShape>.Iterator it = new(aLCS);
        for (; it.More(); it.Next())
        {
            CoupleOfShape cs = it.Value();
            Face f = ShapeCasts.Face(cs.Shape2());
            //
            tolF = Tool.Tolerance(f);
            dt = 2 * (tolE + tolF);
            if (dt > dtMax)
            {
                dtMax = dt;
            }
            //
            // try to compute the minimal 3D step
            FaceSurfaceAdaptor bas = context.SurfaceAdaptor(f);
            double r = 0.0;
            SurfaceKind sType = bas.GetType();
            switch (sType)
            {
                case SurfaceKind.Cylinder:
                    {
                        r = bas.Cylinder().Radius();
                        break;
                    }
                case SurfaceKind.Cone:
                    {
                        Line3D l = new(bas.Cone().Axis());
                        r = l.Distance(aP);
                        break;
                    }
                case SurfaceKind.Sphere:
                    {
                        dtMin = Max(dtMin, 5.0e-4);
                        r = bas.Sphere().Radius();
                        break;
                    }
                case SurfaceKind.Torus:
                    {
                        r = bas.Torus().MajorRadius();
                        break;
                    }
                default:
                    dtMin = Max(dtMin, 5.0e-4);
                    break;
            }
            //
            if (r > 100.0)
            {
                double d = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
                dtMin = Max(dtMin, Math.Sqrt(d * d + 2 * d * r));
            }
        }
        //
        if (dtMax < dtMin)
        {
            dtMax = dtMin;
        }
        //
        // check if the computed 3D step is too big for any of the faces in the list
        it.Initialize(aLCS);
        for (; it.More(); it.Next())
        {
            CoupleOfShape cs = it.Value();
            Face f = ShapeCasts.Face(cs.Shape2());
            //
            FaceSurfaceAdaptor bas = context.SurfaceAdaptor(f);
            //
            context.UVBounds(f, out double uMin, out double uMax, out double vMin, out double vMax);
            //
            double du = uMax - uMin;
            if (du > 0.0)
            {
                double uRes = bas.UResolution(dtMax);
                if (2 * uRes > du)
                {
                    break;
                }
            }
            //
            double dv = vMax - vMin;
            if (dv > 0.0)
            {
                double vRes = bas.VResolution(dtMax);
                if (2 * vRes > dv)
                {
                    break;
                }
            }
        }
        //
        smallFaces = it.More();
        //
        return dtMax;
    }

    /// <summary>True if the shell has a free edge: a non-degenerated edge in only one forward or reversed face.</summary>
    public static bool IsOpenShell(in Shell aSh)
    {
        bool bRet;
        int i, nbE, nbF;
        Orientation orF;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mef = new(ShapeHasher.Instance);
        //
        bRet = false;
        //
        Exploration.MapShapesAndAncestors(aSh, ShapeType.Edge, ShapeType.Face, mef);
        //
        nbE = mef.Extent();
        for (i = 1; i <= nbE; ++i)
        {
            Edge e = ShapeCasts.Edge(mef.FindKey(i));
            if (Tool.Degenerated(e))
            {
                continue;
            }
            //
            nbF = 0;
            NCollection_List<Shape> lf = mef.FindFromIndex(i);
            for (NCollection_List<Shape>.Iterator itLS = new(lf); itLS.More(); itLS.Next())
            {
                Shape f = itLS.Value();
                orF = f.Orientation();
                if (orF == Internal || orF == External)
                {
                    continue;
                }
                ++nbF;
            }
            //
            if (nbF == 1)
            {
                bRet = !bRet; // True
                break;
            }
        }
        //
        return bRet;
    }

    /// <summary>True if the solid is inverted: the infinite point classifies inside it.</summary>
    public static bool IsInvertedSolid(in Solid aSolid)
    {
        double tolS;
        Containment state;
        SolidClassifier sc = new(aSolid);
        //
        tolS = 1.0e-7;
        sc.PerformInfinitePoint(tolS);
        state = sc.State();
        return state == In;
    }
}
