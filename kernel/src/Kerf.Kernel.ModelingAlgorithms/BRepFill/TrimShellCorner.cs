// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimShellCorner.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimShellCorner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2014 OPEN CASCADE SAS; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Filling.TransitionStyle;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Trims the two shells of faces meeting at a corner of a sweep's path along their intersection (right corner) or
/// cuts them before it (round corner), with the history of the faces and edges it rebuilt.
/// </summary>
/// <remarks>
/// A working object of <see cref="Sweep"/> (decision 21). It runs a pave filler on the sweep's own unfrozen
/// faces, so OCCT's destructive mode applies as it does natively (6P's notes), and edits the section edges the filler
/// made.
/// </remarks>
internal sealed class TrimShellCorner
{
    private readonly TransitionStyle _transition;
    private readonly Frame3D _axeOfBisPlane;
    private readonly Vector3D _intPointCrossDir;
    private Shape _shape1 = new();
    private Shape _shape2 = new();
    private NCollection_HArray2<Shape>? _bounds;
    private NCollection_HArray2<Shape>? _uEdges;
    private NCollection_HArray1<Shape>? _vEdges;
    private readonly NCollection_HArray2<Shape> _faces;
    private bool _done;
    private bool _hasSection;
    private readonly Dictionary<Shape, NCollection_List<Shape>> _histMap = new(ShapeHasher.Instance);

    //=================================================================================================

    /// <summary>
    /// The corner between the two columns of <paramref name="faces"/>, with the bisector plane's axis and the
    /// direction the cross product of the u-edges' tangents must follow at their intersection.
    /// </summary>
    public TrimShellCorner(NCollection_HArray2<Shape> faces,
                                    TransitionStyle transition,
                                    in Frame3D axeOfBisPlane,
                                    in Vector3D intPointCrossDir)
    {
        _transition = transition;
        _axeOfBisPlane = axeOfBisPlane;
        _intPointCrossDir = intPointCrossDir;
        _done = false;
        _hasSection = false;
        _faces = new NCollection_HArray2<Shape>(faces.LowerRow(),
                                                        faces.UpperRow(),
                                                        faces.LowerCol(),
                                                        faces.UpperCol());
        _faces.ChangeArray2().Assign(faces.Array2());
    }

    //=================================================================================================

    /// <summary>The section edges bounding the two shells at the corner.</summary>
    public void AddBounds(NCollection_HArray2<Shape> bounds)
    {
        _bounds = new NCollection_HArray2<Shape>(bounds.LowerRow(),
                                                         bounds.UpperRow(),
                                                         bounds.LowerCol(),
                                                         bounds.UpperCol());
        _bounds.ChangeArray2().Assign(bounds.Array2());
    }

    //=================================================================================================

    /// <summary>The u-edges of the two shells.</summary>
    public void AddUEdges(NCollection_HArray2<Shape> uEdges)
    {
        _uEdges = new NCollection_HArray2<Shape>(uEdges.LowerRow(),
                                                         uEdges.UpperRow(),
                                                         uEdges.LowerCol(),
                                                         uEdges.UpperCol());
        _uEdges.ChangeArray2().Assign(uEdges.Array2());
    }

    //=================================================================================================

    /// <summary>The column <paramref name="index"/> of the v-edges: the section edges at the corner.</summary>
    public void AddVEdges(NCollection_HArray2<Shape> vEdges, int index)
    {
        _vEdges = new NCollection_HArray1<Shape>(vEdges.LowerRow(), vEdges.UpperRow());

        for (int i = vEdges.LowerRow(); i <= vEdges.UpperRow(); i++)
        {
            _vEdges.SetValue(i, vEdges.Value(i, index));
        }
    }

    //=================================================================================================

    /// <summary>Trims the shells.</summary>
    public void Perform()
    {
        int index1, index2, nF1, nF2, i, j, nbP, nbC;
        bool bhassec;

        _done = false;
        _histMap.Clear();

        if (_faces.RowLength() != 2)
        {
            return;
        }
        int ii = 0, jj = 0;
        BrepBuilder bb = new();

        for (jj = _faces.LowerCol(); jj <= _faces.UpperCol(); jj++)
        {
            Shell shell = new();
            bb.MakeShell(ref shell);

            for (ii = _faces.LowerRow(); ii <= _faces.UpperRow(); ii++)
            {
                bb.Add(shell, _faces.Value(ii, jj));
            }
            shell.Closed(Tool.IsClosed(shell));

            if (jj == _faces.LowerCol())
            {
                _shape1 = shell;
            }
            else
            {
                _shape2 = shell;
            }
        }

        double maxTol = 0.0;
        Explorer exp = new(_shape1, ShapeType.Vertex);
        for (; exp.More(); exp.Next())
        {
            maxTol = Max(maxTol, Tool.Tolerance(ShapeCasts.Vertex(exp.Current())));
        }

        exp.Init(_shape2, ShapeType.Vertex);
        for (; exp.More(); exp.Next())
        {
            maxTol = Max(maxTol, Tool.Tolerance(ShapeCasts.Vertex(exp.Current())));
        }

        double fuzzy = 4.0 * global::Kerf.Engine.Foundation.Precision.Confusion();
        PaveFiller pf = new();
        NCollection_List<Shape> ls = new();
        ls.Append(_shape1);
        ls.Append(_shape2);
        pf.SetArguments(ls);
        if (maxTol < 1.005 * global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            fuzzy = Max(pf.FuzzyValue(), fuzzy);
            pf.SetFuzzyValue(fuzzy);
        }
        //
        pf.Perform();
        if (pf.HasErrors)
        {
            return;
        }
        //
        DS theDS = pf.PDS()!;
        //
        List<InterfFF> fFs = theDS.InterfFF();
        int nbFFs = fFs.Count;

        if (!SplitUEdges(_uEdges!, theDS, _intPointCrossDir, _histMap))
        {
            return;
        }

        for (ii = _faces.LowerRow(); ii <= _faces.UpperRow(); ii++)
        {
            Shape f1 = _faces.Value(ii, _faces.LowerCol());
            Shape f2 = _faces.Value(ii, _faces.UpperCol());
            //
            index1 = theDS.Index(f1);
            index2 = theDS.Index(f2);

            if ((index1 == -1) || (index2 == -1))
            {
                continue;
            }

            for (i = 0; i < nbFFs; ++i)
            {
                InterfFF fFi = fFs[i];
                fFi.Indices(out nF1, out nF2);
                //
                List<DataPoint> vp = fFi.ChangePoints();
                nbP = vp.Count;
                List<SectionCurve> vc = fFi.Curves();
                nbC = vc.Count;
                if (nbP == 0 && nbC == 0)
                {
                    if (!theDS.HasInterfSubShapes(nF1, nF2))
                    {
                        continue;
                    }
                }
                //
                if ((nF1 == index1) && (nF2 == index2))
                {
                    bhassec = false;
                    //
                    for (j = 0; j < nbC; ++j)
                    {
                        SectionCurve bCurve = vc[j];
                        NCollection_List<PaveBlock> sectEdges = bCurve.PaveBlocks();
                        //
                        if (sectEdges.Extent() != 0)
                        {
                            bhassec = true;
                            break;
                        }
                    }

                    if (!bhassec)
                    {
                        if (!MakeFacesNonSec(ii, theDS, index1, index2))
                        {
                            _histMap.Clear();
                            return;
                        }
                    }
                    else
                    {
                        if (!MakeFacesSec(ii, theDS, index1, index2, i))
                        {
                            _histMap.Clear();
                            return;
                        }
                    }
                    break;
                }
            }
        }
        _done = true;
    }

    //=================================================================================================

    /// <summary>True if the shells are trimmed.</summary>
    public bool IsDone => _done;

    //=================================================================================================

    /// <summary>True if the shells intersect (set nowhere: always false, as in OCCT).</summary>
    public bool HasSection => _hasSection; // OCCT: myHasSection is never set

    //=================================================================================================

    /// <summary>The shapes that replace <paramref name="shape"/> (a face, u-edge or v-edge), or none.</summary>
    public void Modified(in Shape shape, NCollection_List<Shape> modified)
    {
        modified.Clear();

        if (_histMap.ContainsKey(shape))
        {
            modified.Assign(_histMap.GetValue(shape));
        }
    }

    // ----------------------------------------------------------------------------------------------------
    // function: MakeFacesNonSec
    // purpose:         Updates <myHistMap> by new faces in the case when old faces do not intersect
    // ----------------------------------------------------------------------------------------------------
    private bool MakeFacesNonSec(int index, DS ds, int faceIndex1, int faceIndex2)
    {
        bool bHasNewEdge = false;
        Edge newEdge = new();

        BrepBuilder bb = new();
        Shape e1 = _bounds!.Value(index, 1);
        Shape e2 = _bounds.Value(index, 2);

        // search common vertex between bounds. begin
        int index1 = ds.Index(e1);
        int index2 = ds.Index(e2);

        bool bvertexfound = FindCommonVertex(ds,
                                             index1,
                                             index2,
                                             _intPointCrossDir,
                                             out Vertex commonVertex,
                                             out _,
                                             out _);
        // search common vertex between bounds. end

        ReShape substitutor = new();

        // search common vertices between uedges. begin
        NCollection_List<Shape> commonVertices = new();
        int acommonflag = 0; // 0 - no, 1 - first pair, 2 - second pair, 3 - both
        int ueit = 0, eindex = 0;

        for (ueit = 1, eindex = index; ueit <= 2; ueit++, eindex++)
        {
            Shape shape1 = _uEdges!.Value(eindex, _uEdges.LowerCol());
            Shape shape2 = _uEdges.Value(eindex, _uEdges.UpperCol());
            Edge ue1 = ShapeCasts.Edge(shape1);
            Edge ue2 = ShapeCasts.Edge(shape2);

            if (_histMap.ContainsKey(shape1))
            {
                NCollection_List<Shape> lst = _histMap.GetValue(shape1);

                if (!lst.IsEmpty())
                {
                    ue1 = ShapeCasts.Edge(lst.First());
                }
            }

            if (_histMap.ContainsKey(shape2))
            {
                NCollection_List<Shape> lst = _histMap.GetValue(shape2);

                if (!lst.IsEmpty())
                {
                    ue2 = ShapeCasts.Edge(lst.First());
                }
            }

            if (!shape1.IsSame(ue1))
            {
                substitutor.Replace(shape1.Oriented(Forward), ue1.Oriented(Forward));
            }

            if (!shape2.IsSame(ue2))
            {
                substitutor.Replace(shape2.Oriented(Forward), ue2.Oriented(Forward));
            }

            Vertex V1 = Exploration.LastVertex(ue1);
            Vertex V2 = Exploration.FirstVertex(ue2);

            if (V1.IsSame(V2))
            {
                acommonflag = (acommonflag == 0) ? ueit : 3;
                commonVertices.Append(V1);
            }
        }
        // search common vertices between uedges. end

        if (bvertexfound)
        {
            if (commonVertices.Extent() != 1)
            {
                return false;
            }

            if (acommonflag == 1)
            {
                newEdge = new LibraryMakeEdge(ShapeCasts.Vertex(commonVertices.First()), commonVertex).InternalEdge();
            }
            else
            {
                newEdge = new LibraryMakeEdge(commonVertex, ShapeCasts.Vertex(commonVertices.First())).InternalEdge();
            }

            bHasNewEdge = true;
        }

        if (commonVertices.Extent() == 2)
        {
            newEdge = new LibraryMakeEdge(ShapeCasts.Vertex(commonVertices.First()),
                                            ShapeCasts.Vertex(commonVertices.Last())).InternalEdge();
            bHasNewEdge = true;
        }
        int fit = 0;

        for (fit = 1; fit <= 2; fit++)
        {
            Compound comp = new();
            HashSet<Shape> mapV = new(ShapeHasher.Instance);
            bb.MakeCompound(ref comp);

            for (ueit = 1, eindex = index; ueit <= 2; ueit++, eindex++)
            {
                Shape shape = _uEdges!.Value(eindex, _uEdges.LowerCol() + fit - 1);
                Shape ue = shape;

                if (_histMap.ContainsKey(shape))
                {
                    NCollection_List<Shape> lst = _histMap.GetValue(shape);

                    if (!lst.IsEmpty())
                    {
                        ue = ShapeCasts.Edge(lst.First());
                    }
                }
                Shape v = (fit == 1) ? Exploration.FirstVertex(ShapeCasts.Edge(ue)) : Exploration.LastVertex(ShapeCasts.Edge(ue));
                mapV.Add(v);
                bb.Add(comp, ue);
            }

            if (bHasNewEdge)
            {
                bb.Add(comp, newEdge);
                StoreVedgeInHistMap(_vEdges!, index, newEdge, _histMap);
            }

            NCollection_List<Shape> alonevertices = new();
            FindFreeVertices(comp, mapV, alonevertices);

            if (!alonevertices.IsEmpty() && (alonevertices.Extent() != 2))
            {
                return false;
            }

            int faceIndex = (fit == 1) ? faceIndex1 : faceIndex2;
            Shape face = ds.Shape(faceIndex);
            Orientation faceOri = face.Orientation;
            face.Orient(Forward);

            Explorer expE = new(face, ShapeType.Edge);

            if (bHasNewEdge)
            {
                newEdge.Orient(Forward);

                // Refer to BrepFill_Sweep.cxx BuildEdge Construct an edge via an iso
                Point3D p1, p2;
                double p11, p12, p21, p22;
                p1 = Tool.Pnt(Exploration.FirstVertex(ShapeCasts.Edge(newEdge)));
                p2 = Tool.Pnt(Exploration.LastVertex(ShapeCasts.Edge(newEdge)));

                Edge eRef = ShapeCasts.Edge(fit == 1 ? e1 : e2);
                p11 = p1.DistanceTo(Tool.Pnt(Exploration.FirstVertex(eRef)));
                p22 = p2.DistanceTo(Tool.Pnt(Exploration.LastVertex(eRef)));
                p12 = p1.DistanceTo(Tool.Pnt(Exploration.LastVertex(eRef)));
                p21 = p2.DistanceTo(Tool.Pnt(Exploration.FirstVertex(eRef)));

                if (p11 > p12 && p22 > p21)
                {
                    newEdge.Reverse();
                }

                // for nonPlane surface, we should add pCurve
                EdgeFixer sfe = new();
                sfe.FixAddPCurve(newEdge, ShapeCasts.Face(face), false);
            }

            NCollection_List<Shape> orderedList = new();

            if (!alonevertices.IsEmpty())
            {
                int eIndex = (fit == 1) ? index1 : index2;
                bool bfound1 = false;
                bool bfound2 = false;
                double aparam1 = 0.0, aparam2 = 0.0;

                NCollection_List<Pave> lp = new();
                ds.Paves(eIndex, lp);
                NCollection_List<Pave>.Iterator it = new();
                it.Initialize(lp);
                for (; it.More(); it.Next())
                {
                    Pave pave = it.Value();
                    Shape v = ds.Shape(pave.Index());

                    if (v.IsSame(alonevertices.First()))
                    {
                        if (!bfound1)
                        {
                            aparam1 = pave.Parameter();
                            bfound1 = true;
                        }
                    }

                    if (v.IsSame(alonevertices.Last()))
                    {
                        if (!bfound2)
                        {
                            aparam2 = pave.Parameter();
                            bfound2 = true;
                        }
                    }
                }

                if (bfound1 && bfound2)
                {
                    Edge newBoundE;

                    if (fit == 1)
                    {
                        newBoundE = ShapeCasts.Edge(e1.EmptyCopied());
                    }
                    else
                    {
                        newBoundE = ShapeCasts.Edge(e2.EmptyCopied());
                    }
                    Vertex v1, v2;

                    if (aparam1 < aparam2)
                    {
                        v1 = ShapeCasts.Vertex(alonevertices.First());
                        v2 = ShapeCasts.Vertex(alonevertices.Last());
                    }
                    else
                    {
                        v1 = ShapeCasts.Vertex(alonevertices.Last());
                        v2 = ShapeCasts.Vertex(alonevertices.First());
                        double tmp = aparam1;
                        aparam1 = aparam2;
                        aparam2 = tmp;
                    }
                    v1.Orient(Forward);
                    v2.Orient(Reversed);
                    bb.Add(newBoundE, v1);
                    bb.Add(newBoundE, v2);
                    bb.Range(newBoundE, aparam1, aparam2);
                    newBoundE.Orient(Forward);

                    orderedList.Append(newBoundE);

                    if (bHasNewEdge)
                    {
                        Explorer expV = new(newEdge, ShapeType.Vertex);
                        bool bfoundv = false;

                        for (; !bfoundv && expV.More(); expV.Next())
                        {
                            if (v2.IsSame(expV.Current()))
                            {
                                bfoundv = true;
                            }
                        }

                        if (bfoundv)
                        {
                            orderedList.Append(newEdge);
                        }
                        else
                        {
                            orderedList.Prepend(newEdge);
                        }
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                if (bHasNewEdge)
                {
                    orderedList.Append(newEdge);
                }
            }

            if (!orderedList.IsEmpty())
            {
                Wire w = new();
                bb.MakeWire(ref w);
                NCollection_List<Shape>.Iterator itE = new(orderedList);

                for (; itE.More(); itE.Next())
                {
                    bb.Add(w, itE.Value());
                }
                if (fit == 1)
                {
                    substitutor.Replace(e1.Oriented(Forward), w);
                }
                else
                {
                    substitutor.Replace(e2.Oriented(Forward), w);
                }
            }

            substitutor.InternalApply(face);
            Shape newFace = substitutor.InternalValue(face);
            newFace.Orient(faceOri);
            NCollection_List<Shape> atmpList = new();
            atmpList.Append(newFace);
            _histMap[face] = atmpList;

            expE.Init(face, ShapeType.Edge);

            for (; expE.More(); expE.Next())
            {
                Shape newValue = substitutor.InternalValue(expE.Current());

                if (newValue.IsNull || newValue.IsSame(expE.Current()))
                {
                    continue;
                }

                if (_histMap.ContainsKey(expE.Current()))
                {
                    continue;
                }
                NCollection_List<Shape> listOfNewEdge = new();
                Explorer expE2 = new(newValue, ShapeType.Edge);

                for (; expE2.More(); expE2.Next())
                {
                    listOfNewEdge.Append(expE2.Current());
                }
                _histMap[expE.Current()] = listOfNewEdge;
            }
        }

        return true;
    }

    // ----------------------------------------------------------------------------------------------------
    // function: MakeFacesSec
    // purpose:  Updates <myHistMap> by new faces in the case when old faces intersect each other
    // ----------------------------------------------------------------------------------------------------
    private bool MakeFacesSec(int index, DS ds, int faceIndex1, int faceIndex2, int ssInterfIndex)
    {
        List<InterfFF> fFs = ds.InterfFF();
        InterfFF fFi = fFs[ssInterfIndex];
        List<SectionCurve> bCurves = fFi.Curves();

        Face aSecPlane = new();

        if (!FilterSectionEdges(bCurves, aSecPlane, ds, out Compound secEdges))
        {
            return false;
        }

        // Extract vertices on the intersection of correspondent U-edges
        Shape leftE1 = _uEdges!.Value(index, 1);
        Shape leftE2 = _uEdges.Value(index, 2);
        Shape rightE1 = _uEdges.Value(index + 1, 1);
        Shape rightE2 = _uEdges.Value(index + 1, 2);

        int indexOfLeftE1 = ds.Index(leftE1);
        int indexOfLeftE2 = ds.Index(leftE2);
        int indexOfRightE1 = ds.Index(rightE1);
        int indexOfRightE2 = ds.Index(rightE2);

        FindCommonVertex(ds,
                         indexOfLeftE1,
                         indexOfLeftE2,
                         _intPointCrossDir,
                         out Vertex firstVertex,
                         out _,
                         out _);
        FindCommonVertex(ds,
                         indexOfRightE1,
                         indexOfRightE2,
                         _intPointCrossDir,
                         out Vertex lastVertex,
                         out _,
                         out _);

        Shape secWire = new();
        Plane3D secPlane = new();
        bool wireFound = ChooseSection(secEdges, firstVertex, lastVertex, ref secWire, ref secPlane, out _);

        if (wireFound)
        {
            // aSecEdges = SecWire;
            Compound comp = new();
            BrepBuilder BB = new();
            BB.MakeCompound(ref comp);
            Explorer explo = new(secWire, ShapeType.Edge);

            for (; explo.More(); explo.Next())
            {
                BB.Add(comp, explo.Current());
            }
            secEdges = comp;

            StoreVedgeInHistMap(_vEdges!, index, secWire, _histMap);
        }

        int fit = 0;
        ReShape substitutor = new();

        for (fit = 0; fit < 2; fit++)
        {
            int faceIndex = (fit == 0) ? faceIndex1 : faceIndex2;
            Face face = ShapeCasts.Face(ds.Shape(faceIndex));
            Orientation faceOri = face.Orientation;
            Face faceF = face;
            faceF.Orient(Forward);
            Edge boundEdge = ShapeCasts.Edge(_bounds!.Value(index, _bounds.LowerCol() + fit));
            int boundEdgeIndex = ds.Index(boundEdge);

            if (!GetUEdges(index, fit, _uEdges, boundEdge, faceF, out Edge ue1, out Edge ue2))
            {
                return false;
            }

            Edge ue1old = ue1;
            Edge ue2old = ue2;

            if (_histMap.ContainsKey(ue1))
            {
                NCollection_List<Shape> lst = _histMap.GetValue(ue1);

                if (!lst.IsEmpty())
                {
                    Shape edge = lst.First().Oriented(ue1.Orientation);

                    if (!ue1.IsSame(edge))
                    {
                        substitutor.Replace(ue1.Oriented(Forward), edge.Oriented(Forward));
                    }
                    ue1 = ShapeCasts.Edge(edge);
                }
            }

            if (_histMap.ContainsKey(ue2))
            {
                NCollection_List<Shape> lst = _histMap.GetValue(ue2);

                if (!lst.IsEmpty())
                {
                    Shape edge = lst.First().Oriented(ue2.Orientation);

                    if (!ue2.IsSame(edge))
                    {
                        substitutor.Replace(ue2.Oriented(Forward), edge.Oriented(Forward));
                    }
                    ue2 = ShapeCasts.Edge(edge);
                }
            }
            Compound compOfSecEdges = secEdges;
            NCollection_List<Shape> listOfWireEdges = new();
            BrepBuilder bb = new();
            Pave pave1 = new();
            bool isPave1OnUEdge = true;

            if (FindFromUEdge(ue1old,
                              ue2old,
                              ue1,
                              ue2,
                              face,
                              secEdges,
                              fit,
                              boundEdge,
                              boundEdgeIndex,
                              ds,
                              _histMap,
                              ref compOfSecEdges,
                              listOfWireEdges,
                              ref pave1,
                              out isPave1OnUEdge))
            {
                NCollection_List<Shape> secondListOfEdges = new();

                if (!FindFromVEdge(pave1,
                                   isPave1OnUEdge,
                                   ue1old,
                                   ue2old,
                                   face,
                                   compOfSecEdges,
                                   fit,
                                   boundEdge,
                                   boundEdgeIndex,
                                   ds,
                                   _histMap,
                                   secondListOfEdges,
                                   out bool bisSectionFound))
                {
                    return false;
                }

                if (!bisSectionFound && listOfWireEdges.IsEmpty())
                {
                    return false;
                }
                listOfWireEdges.Append(secondListOfEdges);
            }
            else
            {
                return false;
            }

            if (!listOfWireEdges.IsEmpty())
            {
                Wire w = new();
                bb.MakeWire(ref w);
                NCollection_List<Shape>.Iterator eIt = new(listOfWireEdges);

                for (; eIt.More(); eIt.Next())
                {
                    if (!boundEdge.IsSame(eIt.Value()))
                    {
                        bb.Add(w, eIt.Value());
                    }
                }
                substitutor.Replace(boundEdge.Oriented(Forward), w);
            }

            substitutor.InternalApply(face);
            Shape newFace = substitutor.InternalValue(face);
            newFace.Orient(faceOri);
            NCollection_List<Shape> atmpList = new();
            atmpList.Append(newFace);
            _histMap[face] = atmpList;

            Explorer expE = new(face, ShapeType.Edge);

            for (; expE.More(); expE.Next())
            {
                Shape newValue = substitutor.InternalValue(expE.Current());

                if (newValue.IsNull || newValue.IsSame(expE.Current()))
                {
                    continue;
                }

                if (_histMap.ContainsKey(expE.Current()))
                {
                    continue;
                }
                NCollection_List<Shape> listOfNewEdge = new();
                Explorer expE2 = new(newValue, ShapeType.Edge);

                for (; expE2.More(); expE2.Next())
                {
                    listOfNewEdge.Append(expE2.Current());
                }
                _histMap[expE.Current()] = listOfNewEdge;
            }
        }
        return true;
    }

    //=================================================================================================

    private bool ChooseSection(in Shape comp,
                               in Vertex firstVertex,
                               in Vertex lastVertex,
                               ref Shape resWire,
                               ref Plane3D resPlane,
                               out bool isSingular)
    {
        isSingular = false;

        int ind, i, j;
        BrepBuilder bb = new();

        if (_transition == TransitionStyle.Right && !firstVertex.IsNull
            && !lastVertex.IsNull) // the case where section wire goes from
                                         // its known first vertex to its known last vertex
        {
            Wire newWire = new();
            bb.MakeWire(ref newWire);

            Compound oldComp = new();
            bb.MakeCompound(ref oldComp);
            Iterator iter = new(comp);
            for (; iter.More(); iter.Next())
            {
                bb.Add(oldComp, iter.Value());
            }

            Edge firstEdge = FindEdgeCloseToBisectorPlane(firstVertex, oldComp, _axeOfBisPlane.Axis);
            if (firstEdge.IsNull)
            {
                return false;
            }

            iter.Initialize(oldComp);
            if (!iter.More())
            {
                iter.Initialize(comp);
                bb.Add(oldComp, iter.Value());
            }
            Edge lastEdge = FindEdgeCloseToBisectorPlane(lastVertex, oldComp, _axeOfBisPlane.Axis);
            if (lastEdge.IsNull)
            {
                return false;
            }

            if (firstEdge.IsNull || lastEdge.IsNull)
            {
                return false;
            }

            bb.Add(newWire, firstEdge);

            if (!firstEdge.IsSame(lastEdge))
            {
                Vertex commonVertex = new();
                bool commonVertexExists = FindCommonVertex(firstEdge, lastEdge, firstVertex, lastVertex, ref commonVertex);
                if (commonVertexExists)
                {
                    bb.Add(newWire, lastEdge);
                }
                else
                {
                    Vertex vertex1, vertex2, v1, v2;
                    Exploration.Vertices(firstEdge, out v1, out v2);
                    vertex1 = (firstVertex.IsSame(v1)) ? v2 : v1;
                    Exploration.Vertices(lastEdge, out v1, out v2);
                    vertex2 = (lastVertex.IsSame(v1)) ? v2 : v1;

                    NCollection_List<Shape> middleEdges = new();
                    if (FindMiddleEdges(vertex1, vertex2, _axeOfBisPlane.Axis, oldComp, middleEdges))
                    {
                        NCollection_List<Shape>.Iterator itl = new(middleEdges);
                        for (; itl.More(); itl.Next())
                        {
                            bb.Add(newWire, itl.Value());
                        }
                        bb.Add(newWire, lastEdge);
                    }
                    else
                    {
                        // trim <FirstEdge> and <LastEdge> in the points of extrema
                        // these points become new vertex with centre between them
                        DistanceExtCC extrema = new(firstEdge, lastEdge);
                        if (extrema.IsDone && extrema.ExtCount > 0)
                        {
                            int imin = 1;
                            for (i = 2; i <= extrema.ExtCount; i++)
                            {
                                if (extrema.SquareDistance(i) < extrema.SquareDistance(imin))
                                {
                                    imin = i;
                                }
                            }

                            double minDist = Math.Sqrt(extrema.SquareDistance(imin));
                            double paramOnFirstEdge = extrema.ParameterOnE1(imin);
                            double paramOnLastEdge = extrema.ParameterOnE2(imin);
                            Point3D pointOnFirstEdge = extrema.PointOnE1(imin);
                            Point3D pointOnLastEdge = extrema.PointOnE2(imin);
                            Point3D midPnt = new((pointOnFirstEdge.XYZ() + pointOnLastEdge.XYZ()) / 2);
                            commonVertex = new LibraryMakeVertex(midPnt).InternalVertex();
                            bb.UpdateVertex(commonVertex, 1.001 * minDist / 2);

                            UpdateSectionEdge(firstEdge, firstVertex, ref commonVertex, paramOnFirstEdge);
                            UpdateSectionEdge(lastEdge, lastVertex, ref commonVertex, paramOnLastEdge);

                            bb.Add(newWire, lastEdge);
                        }
                    }
                }
            }

            resWire = newWire;
            resPlane = new Plane3D(new CoordinateSystem(_axeOfBisPlane));
            return true;
        }

        // General case: try to find continuous section closest to bisector plane
        Compound oldComp2 = new();
        BrepBuilder b = new();
        b.MakeCompound(ref oldComp2);
        Iterator iter2 = new(comp);
        for (; iter2.More(); iter2.Next())
        {
            b.Add(oldComp2, iter2.Value());
        }

        bool error = false;
        NCollection_Sequence<Shape> wseq = new();
        for (;;)
        {
            Explorer explo = new(oldComp2, ShapeType.Edge);
            if (!explo.More())
            {
                break;
            }
            Edge firstEdge = ShapeCasts.Edge(explo.Current());
            Wire newWire = new LibraryMakeWire(firstEdge).InternalWire();
            b.Remove(oldComp2, firstEdge);
            if (newWire.ClosedFlag)
            {
                wseq.Append(newWire);
                continue;
            }

            for (;;)
            {
                Vertex[] extremity = new Vertex[2];
                Exploration.Vertices(newWire, out extremity[0], out extremity[1]);
                if (extremity[0].IsNull || extremity[1].IsNull)
                {
                    error = true;
                    break;
                }
                IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(oldComp2, ShapeType.Vertex, ShapeType.Edge, vEmap);
                NCollection_List<Shape>[] vedges = [new(), new()];
                for (j = 0; j < 2; j++)
                {
                    if (vEmap.ContainsKey(extremity[j]))
                    {
                        vedges[j].Assign(vEmap.GetValue(extremity[j]));
                    }
                }
                if (vedges[0].IsEmpty() && vedges[1].IsEmpty())
                {
                    // no more edges in OldComp to continue NewWire
                    break;
                }
                bool modified = false;
                for (j = 0; j < 2; j++)
                {
                    if (vedges[j].Extent() == 1)
                    {
                        Edge edge = ShapeCasts.Edge(vedges[j].First());
                        newWire = new LibraryMakeWire(newWire, edge).InternalWire();
                        b.Remove(oldComp2, edge);
                        modified = true;
                    }
                }
                if (!modified) // only multiple connections
                {
                    ind = (vedges[0].IsEmpty()) ? 1 : 0;
                    NCollection_Sequence<Shape> edges = new();
                    NCollection_List<Shape>.Iterator itl = new(vedges[ind]);
                    for (; itl.More(); itl.Next())
                    {
                        edges.Append(itl.Value());
                    }
                    int theind = 0;
                    double minDeviation = RealLast();
                    for (j = 1; j <= edges.Length(); j++)
                    {
                        Wire wire = new LibraryMakeWire(newWire, ShapeCasts.Edge(edges[j])).InternalWire();
                        Plane3D plane = new();
                        double deviation = ComputeAveragePlaneAndMaxDeviation(wire, ref plane, out _);
                        if (deviation < minDeviation)
                        {
                            minDeviation = deviation;
                            theind = j;
                        }
                    }
                    newWire = new LibraryMakeWire(newWire, ShapeCasts.Edge(edges[theind])).InternalWire();
                    b.Remove(oldComp2, edges[theind]);
                }
                if (newWire.ClosedFlag)
                {
                    break;
                }
            }
            wseq.Append(newWire);
            if (error)
            {
                break;
            }
        }

        double minAngle = RealLast();
        Explorer Explo = new(oldComp2, ShapeType.Edge);
        if (!error && !Explo.More()) // wires are built successfully and compound <OldComp> is empty
        {
            if (wseq.Length() == 1) // only one wire => it becomes result
            {
                resWire = wseq.First();
                ComputeAveragePlaneAndMaxDeviation(resWire, ref resPlane, out isSingular);
                return true;
            }
            else // we must choose the wire which average plane is closest to bisector plane
            {    //(check angle between axes)
                for (i = 1; i <= wseq.Length(); i++)
                {
                    Wire wire = ShapeCasts.Wire(wseq[i]);
                    Plane3D pln = new();
                    ComputeAveragePlaneAndMaxDeviation(wire, ref pln, out bool issing);
                    if (issing)
                    {
                        continue;
                    }

                    double angle = pln.Axis.AngleTo(_axeOfBisPlane.Axis);
                    if (angle > M_PI / 2)
                    {
                        angle = M_PI - angle;
                    }

                    if (angle < minAngle)
                    {
                        minAngle = angle;
                        resWire = wire;
                        resPlane = pln;
                    }
                }
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private static bool SplitUEdges(NCollection_HArray2<Shape> uEdges,
                                    DS ds,
                                    in Vector3D crossDirection,
                                    Dictionary<Shape, NCollection_List<Shape>> histMap)
    {
        List<InterfVV> vVs = ds.InterfVV();

        BrepBuilder bb = new();
        int ueit = 0, upRow, lowCol, upCol;
        //
        upRow = uEdges.UpperRow();
        lowCol = uEdges.LowerCol();
        upCol = uEdges.UpperCol();
        //
        for (ueit = uEdges.LowerRow(); ueit <= upRow; ueit++)
        {
            Shape e1 = uEdges.Value(ueit, lowCol);
            Shape e2 = uEdges.Value(ueit, upCol);

            if (histMap.ContainsKey(e1) || histMap.ContainsKey(e2))
            {
                continue;
            }

            int eIndex1 = ds.Index(e1);
            int eIndex2 = ds.Index(e2);

            bool bvertexfound = FindCommonVertex(ds,
                                                 eIndex1,
                                                 eIndex2,
                                                 crossDirection,
                                                 out Vertex commonVertex,
                                                 out double apar1,
                                                 out double apar2);
            //
            if (!bvertexfound)
            {
                Vertex V1 = Exploration.LastVertex(ShapeCasts.Edge(e1));
                Vertex V2 = Exploration.FirstVertex(ShapeCasts.Edge(e2));
                int vindex1 = ds.Index(V1);
                int vindex2 = ds.Index(V2);
                int vvit = 0;
                int nbVVs = vVs.Count;

                for (vvit = 0; !bvertexfound && (vvit < nbVVs); vvit++)
                {
                    InterfVV vv = vVs[vvit];

                    if (((vindex1 == vv.Index1()) && (vindex2 == vv.Index2()))
                        || ((vindex1 == vv.Index2()) && (vindex2 == vv.Index1())))
                    {
                        if (!vv.HasIndexNew())
                        {
                            continue;
                        }
                        commonVertex = ShapeCasts.Vertex(ds.Shape(vv.IndexNew()));
                        bvertexfound = true;
                        apar1 = Tool.Parameter(V1, ShapeCasts.Edge(e1));
                        apar2 = Tool.Parameter(V2, ShapeCasts.Edge(e2));
                    }
                }
            }

            if (bvertexfound)
            {
                Vertex v1, v2;
                double f = 0.0, l = 0.0;
                //
                Edge newE1 = ShapeCasts.Edge(e1.EmptyCopied());
                Exploration.Vertices(ShapeCasts.Edge(e1), out v1, out v2);
                newE1.Orient(Forward);
                v1.Orient(Forward);
                bb.Add(newE1, v1);
                commonVertex.Orient(Reversed);
                bb.Add(newE1, commonVertex);
                Tool.Range(ShapeCasts.Edge(e1), out f, out l);
                bb.Range(newE1, f, apar1);

                //
                Edge newE2 = ShapeCasts.Edge(e2.EmptyCopied());
                Exploration.Vertices(ShapeCasts.Edge(e2), out v1, out v2);
                newE2.Orient(Forward);
                commonVertex.Orient(Forward);
                bb.Add(newE2, commonVertex);
                bb.Add(newE2, v2);
                Tool.Range(ShapeCasts.Edge(e2), out f, out l);
                bb.Range(newE2, apar2, l);

                NCollection_List<Shape> lst = new();
                lst.Append(newE1);
                histMap[e1] = lst;
                lst = new NCollection_List<Shape>();
                lst.Append(newE2);
                histMap[e2] = lst;
            }
        }
        return true;
    }

    //=================================================================================================

    private static void StoreVedgeInHistMap(NCollection_HArray1<Shape> vEdges,
                                            int index,
                                            in Shape newVshape,
                                            Dictionary<Shape, NCollection_List<Shape>> histMap)
    {
        // Replace default value in the map (v-iso edge of face)
        // by intersection of two consecutive faces
        Shape vEdge = vEdges.Value(index);

        histMap.SetValueRef(vEdge, new NCollection_List<Shape>()).Append(newVshape);
    }

    //=================================================================================================

    private static void FindFreeVertices(in Shape shape,
                                         HashSet<Shape> verticesToAvoid,
                                         NCollection_List<Shape> listOfVertex)
    {
        listOfVertex.Clear();
        IndexedMap<Shape, NCollection_List<Shape>> map = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Vertex, ShapeType.Edge, map);
        int i = 0;

        for (i = 1; i <= map.Count; i++)
        {
            Shape key = map.KeyAt(i - 1);

            if (verticesToAvoid.Contains(key))
            {
                continue;
            }
            NCollection_List<Shape> list = map.ValueAt(i - 1);

            if (list.Extent() < 2)
            {
                listOfVertex.Append(key);
            }
        }
    }

    //=================================================================================================

    private static bool FindCommonVertex(DS ds,
                                         int eIndex1,
                                         int eIndex2,
                                         in Vector3D crossDirection,
                                         out Vertex commonVertex,
                                         out double paramOnE1,
                                         out double paramOnE2)
    {
        // OCCT leaves the outputs unwritten when no vertex is found; its callers initialise them
        commonVertex = new Vertex();
        paramOnE1 = 0.0;
        paramOnE2 = 0.0;

        List<InterfEE> eEs = ds.InterfEE();

        bool bvertexfound = false;
        int eeit = 0;

        Edge theE1 = ShapeCasts.Edge(ds.Shape(eIndex1));
        Edge theE2 = ShapeCasts.Edge(ds.Shape(eIndex2));
        EdgeCurveAdaptor bc1 = new(theE1), bc2 = new(theE2);

        int nbEEs;
        nbEEs = eEs.Count;
        for (eeit = 0; eeit < nbEEs; ++eeit)
        {
            InterfEE ee = eEs[eeit];

            if ((eIndex1 == ee.Index1() && eIndex2 == ee.Index2())
                || (eIndex1 == ee.Index2() && eIndex2 == ee.Index1()))
            {
                if (!ee.HasIndexNew())
                {
                    continue;
                }

                CommonPrt cp = ee.CommonPart();
                if (cp.Type() == ShapeType.Vertex)
                {
                    commonVertex = ShapeCasts.Vertex(ds.Shape(ee.IndexNew()));

                    if (eIndex1 == ee.Index1())
                    {
                        IntersectionTools.VertexParameters(cp, out paramOnE1, out paramOnE2);
                    }
                    else
                    {
                        IntersectionTools.VertexParameters(cp, out paramOnE2, out paramOnE1);
                    }

                    Direction3D intersectPointCrossDir;

                    // intersect point aDirOnE1.cross(aDirOnE2) should same direction with path
                    // theCrossDirection
                    bc1.D1(paramOnE1, out _, out Vector3D dirOnE1);
                    bc2.D1(paramOnE2, out _, out Vector3D dirOnE2);
                    intersectPointCrossDir = new Direction3D(dirOnE1.Crossed(dirOnE2));

                    if (intersectPointCrossDir.Dot(new Direction3D(crossDirection)) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        bvertexfound = true;
                        break;
                    }
                }
            }
        }
        return bvertexfound;
    }

    //=================================================================================================

    private static bool GetUEdges(int index,
                                  int rank,
                                  NCollection_HArray2<Shape> uEdges,
                                  in Edge boundEdge,
                                  in Face face,
                                  out Edge firstUEdge,
                                  out Edge secondUEdge)
    {
        firstUEdge = new Edge();
        secondUEdge = new Edge();
        Shape ue1 = uEdges.Value(index, uEdges.LowerCol() + rank);
        Shape ue2 = uEdges.Value(index + 1, uEdges.LowerCol() + rank);

        Face aFace = face;
        aFace.Orient(Forward);
        Edge e1 = new(), e2 = new();
        Explorer exp = new(aFace, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            if (e1.IsNull && ue1.IsSame(exp.Current()))
            {
                e1 = ShapeCasts.Edge(exp.Current());
            }
            else if (e2.IsNull && ue2.IsSame(exp.Current()))
            {
                e2 = ShapeCasts.Edge(exp.Current());
            }
        }

        if (e1.IsNull || e2.IsNull)
        {
            return false;
        }

        Curve2D? c1 = Tool.CurveOnSurface(e1, aFace, out double f, out double l);

        if (c1 is null)
        {
            return false;
        }
        Point2D pu1 = (rank == 0) ? c1.Value(l) : c1.Value(f);
        Curve2D? c2 = Tool.CurveOnSurface(boundEdge, aFace, out f, out l);

        if (c2 is null)
        {
            return false;
        }
        Tool.Range(boundEdge, out f, out l);
        Point2D pf = c2.Value(f);
        Vertex v = (rank == 0) ? Exploration.LastVertex(e1) : Exploration.FirstVertex(e1);
        double tolerance = Tool.Tolerance(v);
        FaceSurfaceAdaptor bas = new(aFace, false);

        if (pf.DistanceTo(pu1) > bas.UResolution(tolerance))
        {
            Edge atmpE = e1;
            e1 = e2;
            e2 = atmpE;
        }
        firstUEdge = e1;
        secondUEdge = e2;
        return true;
    }

    //=================================================================================================

    private static bool FillGap(in Vertex firstVertex,
                                in Vertex lastVertex,
                                in Point2D firstPoint,
                                in Point2D lastPoint,
                                in Face face,
                                in Compound sectionEdges,
                                NCollection_List<Shape> orderedList)
    {
        IndexedMap<Shape, NCollection_List<Shape>> map = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(sectionEdges, ShapeType.Vertex, ShapeType.Edge, map);

        if (map.Count == 0)
        {
            return false;
        }

        if (!map.ContainsKey(firstVertex) || !map.ContainsKey(lastVertex))
        {
            return false;
        }
        NCollection_List<Shape> listOfEdge = new();
        HashSet<Shape> mapToAvoid = new(ShapeHasher.Instance);

        if (FindNextEdge(firstVertex, lastVertex, map, mapToAvoid, listOfEdge))
        {
            if (!listOfEdge.IsEmpty())
            {
                return CheckAndOrientEdges(listOfEdge, firstPoint, lastPoint, face, orderedList);
            }
        }
        return false;
    }

    //=================================================================================================

    private static bool FindNextEdge(in Vertex firstVertex,
                                     in Vertex lastVertex,
                                     IndexedMap<Shape, NCollection_List<Shape>> mapVE,
                                     HashSet<Shape> mapToAvoid,
                                     NCollection_List<Shape> orderedList)
    {
        Vertex curVertex = firstVertex;
        HashSet<Shape> aMapToAvoid = new(mapToAvoid,mapToAvoid.Comparer);
        NCollection_List<Shape> listOfEdge = new();
        int i = 0;

        for (i = 1; i <= mapVE.Count; i++)
        {
            if (!mapVE.ContainsKey(curVertex))
            {
                break;
            }
            NCollection_List<Shape> lste = mapVE.GetValue(curVertex);
            bool befound = false;

            NCollection_List<Shape>.Iterator it = new(lste);

            for (; it.More(); it.Next())
            {
                Shape edge = it.Value();
                Vertex saveCurVertex = curVertex;

                if (!aMapToAvoid.Contains(edge))
                {
                    Exploration.Vertices(ShapeCasts.Edge(edge), out Vertex v1, out Vertex v2);

                    if (!curVertex.IsSame(v1))
                    {
                        curVertex = v1;
                    }
                    else if (!curVertex.IsSame(v2))
                    {
                        curVertex = v2;
                    }
                    aMapToAvoid.Add(edge);
                    befound = true;
                    listOfEdge.Append(edge);

                    if (!curVertex.IsSame(lastVertex))
                    {
                        NCollection_List<Shape> listtmp = new();

                        if (!FindNextEdge(curVertex, lastVertex, mapVE, aMapToAvoid, listtmp))
                        {
                            listOfEdge.Clear();
                            curVertex = saveCurVertex;
                            continue;
                        }
                        else
                        {
                            listOfEdge.Append(listtmp);
                            orderedList.Append(listOfEdge);
                            return true;
                        }
                    }
                    break;
                }
            }

            if (curVertex.IsSame(lastVertex))
            {
                break;
            }

            if (!befound)
            {
                return false;
            }
        }

        if (curVertex.IsSame(lastVertex))
        {
            orderedList.Append(listOfEdge);
            return true;
        }
        return false;
    }

    //=================================================================================================

    private static bool CheckAndOrientEdges(NCollection_List<Shape> orderedList,
                                            in Point2D firstPoint,
                                            in Point2D lastPoint,
                                            in Face face,
                                            NCollection_List<Shape> orientedList)
    {
        NCollection_List<Shape>.Iterator it = new(orderedList);

        if (!it.More())
        {
            return true;
        }

        Edge ePrev = ShapeCasts.Edge(it.Value());
        it.Next();

        Curve2D curve = Tool.CurveOnSurface(ePrev, face, out double f, out double l)!;
        Exploration.Vertices(ePrev, out Vertex vf, out Vertex vl);
        FaceSurfaceAdaptor bas = new(face, false);

        double tolerance1 = (vf.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(vf);
        double tolerance2 = (vl.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(vl);
        double utol = bas.UResolution(tolerance1);
        double vtol = bas.VResolution(tolerance1);
        tolerance1 = (utol > vtol) ? utol : vtol;
        utol = bas.UResolution(tolerance2);
        vtol = bas.VResolution(tolerance2);
        tolerance2 = (utol > vtol) ? utol : vtol;

        Point2D ap = curve.Value(f);
        bool bFirstFound = false;
        bool bLastFound = false;

        if (ap.DistanceTo(firstPoint) < tolerance1)
        {
            if (orientedList.IsEmpty())
            {
                orientedList.Append(ePrev.Oriented(Forward));
            }
            bFirstFound = true;
        }
        else if (ap.DistanceTo(lastPoint) < tolerance1)
        {
            if (orientedList.IsEmpty())
            {
                orientedList.Append(ePrev.Oriented(Reversed));
            }
            bLastFound = true;
        }
        ap = curve.Value(l);

        if (ap.DistanceTo(lastPoint) < tolerance2)
        {
            if (orientedList.IsEmpty())
            {
                orientedList.Append(ePrev.Oriented(Forward));
            }
            bLastFound = true;
        }
        else if (ap.DistanceTo(firstPoint) < tolerance2)
        {
            if (orientedList.IsEmpty())
            {
                orientedList.Append(ePrev.Oriented(Reversed));
            }
            bFirstFound = true;
        }

        if (!orientedList.IsEmpty())
        {
            ePrev = ShapeCasts.Edge(orientedList.Last());
        }

        for (; it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            Exploration.Vertices(ePrev, out Vertex v11, out Vertex v12, true);
            Exploration.Vertices(e, out Vertex v21, out Vertex v22, false);

            Orientation ori = (v12.IsSame(v21) || v11.IsSame(v22)) ? Forward : Reversed;
            orientedList.Append(e.Oriented(ori));
            ePrev = ShapeCasts.Edge(orientedList.Last());

            tolerance1 = (v21.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(v21);
            tolerance2 = (v22.IsNull) ? global::Kerf.Engine.Foundation.Precision.Confusion() : Tool.Tolerance(v22);
            utol = bas.UResolution(tolerance1);
            vtol = bas.VResolution(tolerance1);
            tolerance1 = (utol > vtol) ? utol : vtol;
            utol = bas.UResolution(tolerance2);
            vtol = bas.VResolution(tolerance2);
            tolerance2 = (utol > vtol) ? utol : vtol;
            curve = Tool.CurveOnSurface(e, face, out f, out l)!;
            ap = curve.Value(f);

            if (ap.DistanceTo(firstPoint) < tolerance1)
            {
                bFirstFound = true;
            }
            else if (ap.DistanceTo(lastPoint) < tolerance1)
            {
                bLastFound = true;
            }
            ap = curve.Value(l);

            if (ap.DistanceTo(firstPoint) < tolerance2)
            {
                bFirstFound = true;
            }
            else if (ap.DistanceTo(lastPoint) < tolerance2)
            {
                bLastFound = true;
            }
        }

        return bFirstFound && bLastFound;
    }

    //=================================================================================================

    private static bool FindVertex(in Edge edge,
                                   int rank,
                                   DS ds,
                                   Dictionary<Shape, NCollection_List<Shape>> histMap,
                                   ref Vertex vertex,
                                   ref Pave pave)
    {
        if (!histMap.ContainsKey(edge))
        {
            return false;
        }

        NCollection_List<Shape> lst = histMap.GetValue(edge);

        if (lst.IsEmpty())
        {
            return false;
        }

        Edge newEdge = ShapeCasts.Edge(lst.First());
        Tool.Range(newEdge, out double f, out double l);

        if (rank == 0)
        {
            pave.SetParameter(l);
            vertex = Exploration.LastVertex(newEdge);
        }
        else
        {
            pave.SetParameter(f);
            vertex = Exploration.FirstVertex(newEdge);
        }
        int index = ds.Index(vertex);
        if (index == -1)
        {
            int i, i1, i2;
            i1 = ds.SourceShapeCount;
            i2 = ds.ShapeCount;
            for (i = i1; i < i2; ++i)
            {
                Shape sx = ds.Shape(i);
                if (sx.IsSame(vertex))
                {
                    index = i;
                    break;
                }
            }
        }

        pave.SetIndex(index);

        return true;
    }

    //=================================================================================================

    private static bool FindNextVertex(int edgeIndex,
                                       in Pave prevPave,
                                       DS ds,
                                       ref Vertex nextVertex,
                                       ref Pave pave)
    {
        bool bTakePave, bFound;
        Pave tmpPave;
        NCollection_List<Pave>.Iterator itP = new();
        //
        Pave anullpave = new();
        bFound = false;
        bTakePave = prevPave.IsEqual(anullpave);

        NCollection_List<Pave> lp = new();
        ds.Paves(edgeIndex, lp);
        itP.Initialize(lp);
        for (; itP.More(); itP.Next())
        {
            tmpPave = itP.Value();
            //
            if (bTakePave)
            {
                if (ds.IsNewShape(tmpPave.Index()))
                {
                    nextVertex = ShapeCasts.Vertex(ds.Shape(tmpPave.Index()));
                    pave = tmpPave;
                    bFound = true;
                    break;
                }
            }
            //
            else if (tmpPave.IsEqual(prevPave))
            {
                bTakePave = true;
            }
        }

        return bFound;
    }

    //=================================================================================================

    private static bool GetPave(int edgeIndex, bool isFirst, DS ds, ref Pave pave)
    {
        NCollection_List<Pave> lp = new();

        ds.Paves(edgeIndex, lp);
        if (lp.Extent() == 0)
        {
            return false;
        }
        //
        if (isFirst)
        {
            pave = lp.First();
        }
        else
        {
            pave = lp.Last();
        }

        return true;
    }

    //=================================================================================================

    private static bool FindFromUEdge(in Edge ue1Old,
                                      in Edge ue2Old,
                                      in Edge ue1New,
                                      in Edge ue2New,
                                      in Face face,
                                      in Compound secEdges,
                                      int rank,
                                      in Edge boundEdge,
                                      int boundEdgeIndex,
                                      DS ds,
                                      Dictionary<Shape, NCollection_List<Shape>> histMap,
                                      ref Compound secEdgesNew,
                                      NCollection_List<Shape> listOfWireEdges,
                                      ref Pave foundPave,
                                      out bool isOnUEdge)
    {
        foundPave.SetIndex(0);
        foundPave.SetParameter(0.0);
        isOnUEdge = true;

        Face faceF = face;
        faceF.Orient(Forward);
        Vertex prevVertex = new(), nextVertex = new();
        Compound compOfSecEdges = secEdges;
        NCollection_List<Shape> aListOfWireEdges = new();

        Pave pave1 = new(), pave2 = new();
        Point2D p1, p2;
        Vertex firstV, lastV;
        Pave atmpPave = new();

        if (!FindVertex(ue1Old, rank, ds, histMap, ref prevVertex, ref atmpPave))
        {
            return true;
        }

        if (prevVertex.IsNull)
        {
            return false;
        }

        firstV = prevVertex;
        bool bSecFound = false;
        Curve2D c1 = Tool.CurveOnSurface(ue1New, faceF, out double f, out double l)!;
        p1 = (rank == 0) ? c1.Value(l) : c1.Value(f);
        Pave afoundpave = new();
        NCollection_List<Pave> lp = new();
        ds.Paves(boundEdgeIndex, lp);
        int nbpave = lp.Extent();
        int pit = 0;

        while (FindNextVertex(boundEdgeIndex, pave1, ds, ref nextVertex, ref pave2) && (pit < nbpave))
        {
            lastV = nextVertex;
            Curve2D c2 = Tool.CurveOnSurface(boundEdge, faceF, out f, out l)!;
            p2 = c2.Value(pave2.Parameter());
            NCollection_List<Shape> orderedList = new();

            if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
            {
                // remove found edges...
                RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                compOfSecEdges = comp;
                aListOfWireEdges.Append(orderedList);
                afoundpave = pave2;
                isOnUEdge = false;
                bSecFound = true;
                break;
            }
            prevVertex = nextVertex;
            pave1 = pave2;
            pit++;
        }

        if (!bSecFound && FindVertex(ue2Old, rank, ds, histMap, ref nextVertex, ref pave2))
        {
            lastV = nextVertex;
            Curve2D c2 = Tool.CurveOnSurface(ue2New, faceF, out f, out l)!;
            p2 = c2.Value(pave2.Parameter());
            NCollection_List<Shape> orderedList = new();

            if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
            {
                // remove found edges...
                RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                compOfSecEdges = comp;
                aListOfWireEdges.Append(orderedList);
                afoundpave = pave2;
                bSecFound = true;
                isOnUEdge = true;
            }
        }

        if (bSecFound)
        {
            foundPave = afoundpave;
            listOfWireEdges.Assign(aListOfWireEdges);
            secEdgesNew = compOfSecEdges;
        }
        return true;
    }

    //=================================================================================================

    // Finds the split of the bound edge between aPave1 and aPave2 (the repeated "get split" blocks of
    // FindFromVEdge, Kerf)
    private static Edge BoundSplit(DS ds, int boundEdgeIndex, in Pave aPave1, in Pave aPave2)
    {
        Edge eSplit = new();
        NCollection_List<PaveBlock>.Iterator pbIt = new(ds.PaveBlocks(boundEdgeIndex));

        for (; pbIt.More(); pbIt.Next())
        {
            PaveBlock pb1 = pbIt.Value();
            if (pb1.OriginalEdge() == boundEdgeIndex && pb1.Pave1().IsEqual(aPave1) && pb1.Pave2().IsEqual(aPave2))
            {
                if (pb1.Edge() > 0)
                {
                    eSplit = ShapeCasts.Edge(ds.Shape(pb1.Edge()));
                    break;
                }
            }
        }
        return eSplit;
    }

    private static bool FindFromVEdge(in Pave prevPave,
                                      bool isOnUEdge,
                                      in Edge ue1Old,
                                      in Edge ue2Old,
                                      in Face face,
                                      in Compound secEdges,
                                      int rank,
                                      in Edge boundEdge,
                                      int boundEdgeIndex,
                                      DS ds,
                                      Dictionary<Shape, NCollection_List<Shape>> histMap,
                                      NCollection_List<Shape> listOfWireEdges,
                                      out bool isSectionFound)
    {
        listOfWireEdges.Clear();
        isSectionFound = false;
        //
        Face faceF = face;
        faceF.Orient(Forward);
        Vertex prevVertex, nextVertex = new();
        Compound compOfSecEdges = secEdges;
        NCollection_List<Shape> aListOfWireEdges = new();

        Pave pave1, pave2 = new();

        if (isOnUEdge)
        {
            Vertex atmpVertex = new();
            Pave paveOfE2 = new();

            if (FindVertex(ue2Old, rank, ds, histMap, ref atmpVertex, ref paveOfE2))
            {
                if (prevPave.IsEqual(paveOfE2))
                {
                    return true;
                }
            }
        }

        Point2D p1 = new(0.0, 0.0), p2 = new(0.0, 0.0);
        Vertex firstV, lastV;
        Curve2D c1 = Tool.CurveOnSurface(ue1Old, faceF, out double f, out double l)!;
        _ = c1;
        Curve2D c2 = Tool.CurveOnSurface(boundEdge, faceF, out f, out l)!;
        bool bSecFound = false;

        pave1 = prevPave;

        if (isOnUEdge)
        {
            Pave atmpPave = new();

            if (!GetPave(boundEdgeIndex, true, ds, ref atmpPave))
            {
                return false;
            }
            pave1 = atmpPave;
        }
        p1 = c2.Value(pave1.Parameter());
        prevVertex = ShapeCasts.Vertex(ds.Shape(pave1.Index()));

        NCollection_List<Pave> lp = new();
        ds.Paves(boundEdgeIndex, lp);
        int nbpave = lp.Extent();
        int pit = 0;
        NCollection_Array1<NCollection_List<Shape>> arrayOfListOfSec = new(1, nbpave);
        for (int iArr = 1; iArr <= nbpave; iArr++)
        {
            arrayOfListOfSec[iArr] = new NCollection_List<Shape>();
        }

        // by pairs non continuously. begin
        int k = 0;
        Pave firstPave = pave1;
        Vertex firstVertex = prevVertex;
        Point2D apfirst = p1;
        NCollection_List<Pave> firstPaves = new(), lastPaves = new();
        NCollection_List<int> listOfFlags = new();
        int apaircounter = 1;

        for (k = 0; k < nbpave; k++)
        {
            pave1 = firstPave;
            p1 = apfirst;
            prevVertex = firstVertex;
            bool bfound = false;
            pit = 0;

            while (FindNextVertex(boundEdgeIndex, pave1, ds, ref nextVertex, ref pave2) && (pit < nbpave))
            {
                firstV = prevVertex;
                lastV = nextVertex;
                p2 = c2.Value(pave2.Parameter());

                NCollection_List<Shape> orderedList = new();

                if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
                {
                    RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                    compOfSecEdges = comp;

                    arrayOfListOfSec[apaircounter++].Append(orderedList);
                    firstPaves.Append(firstPave);
                    lastPaves.Append(pave2);
                    listOfFlags.Append(1);
                    firstPave = pave2;
                    firstVertex = nextVertex;
                    apfirst = p2;
                    prevVertex = nextVertex;
                    bSecFound = true;
                    bfound = true;
                }
                pave1 = pave2;
                pit++;
            }

            if (FindVertex(ue2Old, rank, ds, histMap, ref nextVertex, ref pave2))
            {
                firstV = prevVertex;
                lastV = nextVertex;
                Curve2D c3 = Tool.CurveOnSurface(ue2Old, faceF, out f, out l)!;
                p2 = c3.Value(pave2.Parameter());

                NCollection_List<Shape> orderedList = new();

                if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
                {
                    RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                    compOfSecEdges = comp;
                    arrayOfListOfSec[apaircounter++].Append(orderedList);
                    firstPaves.Append(firstPave);
                    lastPaves.Append(pave2);
                    listOfFlags.Append(0);
                    bSecFound = true;
                    break;
                }
            }

            if (!bfound)
            {
                if (!FindNextVertex(boundEdgeIndex, firstPave, ds, ref nextVertex, ref pave2))
                {
                    break;
                }
                firstPave = pave2;
                apfirst = c2.Value(pave2.Parameter());
                firstVertex = nextVertex;
            }
        }
        // by pairs non continuously. end

        // by pairs continuously. begin
        pave1 = prevPave;

        if (isOnUEdge)
        {
            Pave atmpPave = new();

            if (!GetPave(boundEdgeIndex, true, ds, ref atmpPave))
            {
                return false;
            }
            pave1 = atmpPave;
        }
        p1 = c2.Value(pave1.Parameter());
        prevVertex = ShapeCasts.Vertex(ds.Shape(pave1.Index()));

        pit = 0;

        while (FindNextVertex(boundEdgeIndex, pave1, ds, ref nextVertex, ref pave2) && (pit < nbpave))
        {
            firstV = prevVertex;
            lastV = nextVertex;
            p2 = c2.Value(pave2.Parameter());

            bool bisinside = false;
            int apbindex = 0;
            int apbcounter = 1;
            NCollection_List<Pave>.Iterator pIt1 = new(), pIt2 = new();
            NCollection_List<int>.Iterator flagIt = new();

            for (pIt1.Initialize(firstPaves), pIt2.Initialize(lastPaves), flagIt.Initialize(listOfFlags);
                 pIt1.More() && pIt2.More() && flagIt.More();
                 pIt1.Next(), pIt2.Next(), flagIt.Next(), apbcounter++)
            {
                bool bfin = false;
                bool blin = false;

                if (pave1.IsEqual(pIt1.Value()))
                {
                    bfin = true;
                }
                else
                {
                    bfin = (pave1.Parameter() > pIt1.Value().Parameter());
                }

                if (flagIt.Value() != 0)
                {
                    if (pave2.IsEqual(pIt2.Value()))
                    {
                        blin = true;
                    }
                    else
                    {
                        blin = (pave2.Parameter() < pIt2.Value().Parameter());
                    }
                }
                else
                {
                    if ((pave2.Index() == pIt2.Value().Index()) && (pave2.Index() > 0))
                    {
                        Curve2D pc = Tool.CurveOnSurface(ue2Old, faceF, out f, out l)!;
                        Point2D p3 = pc.Value(pIt2.Value().Parameter());
                        Vertex v = ShapeCasts.Vertex(ds.Shape(pave2.Index()));
                        FaceSurfaceAdaptor bas = new(faceF, false);
                        double tolerance = Tool.Tolerance(v);
                        double utol = bas.UResolution(tolerance);
                        double vtol = bas.VResolution(tolerance);
                        tolerance = (utol > vtol) ? utol : vtol;

                        if (p2.DistanceTo(p3) < tolerance)
                        {
                            blin = true;
                        }
                    }
                }

                if (bfin && blin)
                {
                    apbindex = apbcounter;
                    bisinside = true;
                    break;
                }
            }

            if (!bisinside)
            {
                NCollection_List<Shape> orderedList = new();

                if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
                {
                    RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                    compOfSecEdges = comp;
                    aListOfWireEdges.Append(orderedList);

                    bSecFound = true;
                }
                else
                {
                    // get split
                    Edge eSplit = BoundSplit(ds, boundEdgeIndex, pave1, pave2);

                    if (!eSplit.IsNull)
                    {
                        aListOfWireEdges.Append(eSplit);
                    }
                }
            }
            else
            {
                if (apbindex > 0)
                {
                    NCollection_List<Shape> listOfSec = arrayOfListOfSec[apbindex];
                    aListOfWireEdges.Append(listOfSec);
                }
            }
            pave1 = pave2;
            prevVertex = nextVertex;
            p1 = p2;
            pit++;
        }

        if (FindVertex(ue2Old, rank, ds, histMap, ref nextVertex, ref pave2))
        {
            firstV = prevVertex;
            lastV = nextVertex;
            Curve2D c3 = Tool.CurveOnSurface(ue2Old, faceF, out f, out l)!;
            p2 = c3.Value(pave2.Parameter());

            bool bisinside = false;
            int apbindex = 0;
            int apbcounter = 1;
            NCollection_List<Pave>.Iterator pIt1 = new(), pIt2 = new();
            NCollection_List<int>.Iterator flagIt = new();

            for (pIt1.Initialize(firstPaves), pIt2.Initialize(lastPaves), flagIt.Initialize(listOfFlags);
                 pIt1.More() && pIt2.More() && flagIt.More();
                 pIt1.Next(), pIt2.Next(), flagIt.Next(), apbcounter++)
            {
                bool bfin = false;
                bool blin = false;

                if (pave1.IsEqual(pIt1.Value()))
                {
                    bfin = true;
                }
                else
                {
                    bfin = (pave1.Parameter() > pIt1.Value().Parameter());
                }

                if (flagIt.Value() != 0)
                {
                    if (pave2.IsEqual(pIt2.Value()))
                    {
                        blin = true;
                    }
                    else
                    {
                        blin = (pave2.Parameter() < pIt2.Value().Parameter());
                    }
                }
                else
                {
                    blin = true;
                }

                if (bfin && blin)
                {
                    apbindex = apbcounter;
                    bisinside = true;
                    break;
                }
            }

            if (!bisinside)
            {
                NCollection_List<Shape> orderedList = new();

                if (FillGap(firstV, lastV, p1, p2, faceF, compOfSecEdges, orderedList))
                {
                    RemoveEdges(compOfSecEdges, orderedList, out Compound comp);
                    compOfSecEdges = comp;
                    aListOfWireEdges.Append(orderedList);

                    bSecFound = true;
                }
                else
                {
                    // add split
                    // get split
                    if (!GetPave(boundEdgeIndex, false, ds, ref pave2))
                    {
                        return false;
                    }
                    //
                    Edge eSplit = BoundSplit(ds, boundEdgeIndex, pave1, pave2);

                    if (!eSplit.IsNull)
                    {
                        aListOfWireEdges.Append(eSplit);
                    }
                }
            }
            else
            {
                if (apbindex > 0)
                {
                    NCollection_List<Shape> listOfSec = arrayOfListOfSec[apbindex];
                    aListOfWireEdges.Append(listOfSec);
                }
            }
        }
        else
        {
            // add split
            // get split
            if (!GetPave(boundEdgeIndex, false, ds, ref pave2))
            {
                return false;
            }

            Edge eSplit = BoundSplit(ds, boundEdgeIndex, pave1, pave2);

            if (!eSplit.IsNull)
            {
                aListOfWireEdges.Append(eSplit);
            }
        }

        // by pairs continuously. end
        listOfWireEdges.Assign(aListOfWireEdges);
        isSectionFound = bSecFound;
        return true;
    }

    //=================================================================================================

    private static void RemoveEdges(in Compound sourceComp,
                                    NCollection_List<Shape> listToRemove,
                                    out Compound resultComp)
    {
        BrepBuilder bb = new();
        Compound comp = new();
        bb.MakeCompound(ref comp);
        Explorer exp = new(sourceComp, ShapeType.Edge);

        for (; exp.More(); exp.Next())
        {
            bool bfound = false;
            NCollection_List<Shape>.Iterator it = new(listToRemove);

            for (; !bfound && it.More(); it.Next())
            {
                bfound = exp.Current().IsSame(it.Value());
            }

            if (!bfound)
            {
                bb.Add(comp, exp.Current());
            }
        }
        resultComp = comp;
    }

    //=================================================================================================

    private static bool FilterSectionEdges(List<SectionCurve> bCurves,
                                           in Face secPlane,
                                           DS ds,
                                           out Compound result)
    {
        result = new Compound();
        result.Nullify();

        BrepBuilder bb = new();
        bb.MakeCompound(ref result);
        int nbCurves = bCurves.Count;
        int cit = 0;
        NCollection_List<PaveBlock>.Iterator pbIt = new();

        for (cit = 0; cit < nbCurves; ++cit)
        {
            SectionCurve bCurve = bCurves[cit];
            NCollection_List<PaveBlock> sectEdges = bCurve.PaveBlocks();

            pbIt.Initialize(sectEdges);
            for (; pbIt.More(); pbIt.Next())
            {
                PaveBlock pb = pbIt.Value();
                int nSect = pb.Edge();
                Shape s = ds.Shape(nSect);
                Edge edge = ShapeCasts.Edge(s);
                bool bAddEdge = true;

                if (!secPlane.IsNull)
                {
                    BeanFaceIntersector intersector = new(edge, secPlane);
                    Tool.Range(edge, out double f, out double l);
                    intersector.SetBeanParameters(f, l);
                    //
                    IntersectionContext context = new();
                    intersector.SetContext(context);
                    //
                    intersector.Perform();

                    if (intersector.IsDone)
                    {
                        bAddEdge = false;
                        int r = 0;

                        for (r = 1; r <= intersector.Result().Length(); r++)
                        {
                            ParameterRange range = intersector.Result().Value(r);

                            if (((range.First() - f) < global::Kerf.Engine.Foundation.Precision.PConfusion()) && ((l - range.Last()) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                bAddEdge = true;
                                break;
                            }
                        }
                    }
                }

                if (bAddEdge)
                {
                    bb.Add(result, s);
                }
            }
        }

        return true;
    }

    //=================================================================================================

    private static double ComputeAveragePlaneAndMaxDeviation(in Shape aWire, ref Plane3D plane, out bool isSingular)
    {
        int n = 40;
        int nedges = aWire.ChildCount;

        NCollection_Array1<Point3D> pnts = new(1, nedges * n);
        int ind = 1, i;
        for (Iterator iter = new(aWire); iter.More(); iter.Next())
        {
            Edge edge = ShapeCasts.Edge(iter.Value());
            EdgeCurveAdaptor curve = new(edge);
            UniformAbscissa distribution = new(curve, n + 1);
            for (i = 1; i <= n; i++)
            {
                double par = distribution.Parameter(i);
                pnts[ind++] = curve.Value(par);
            }
        }

        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(pnts, out Frame3D axe, out isSingular);
        if (isSingular)
        {
            return -1;
        }

        plane = new Plane3D(new CoordinateSystem(axe));
        double maxDeviation = 0;
        for (i = 1; i <= pnts.Length(); i++)
        {
            double dist = plane.DistanceTo(pnts[i]);
            if (dist > maxDeviation)
            {
                maxDeviation = dist;
            }
        }
        return maxDeviation;
    }

    private static void UpdateSectionEdge(in Edge edge,
                                          in Vertex constVertex,
                                          ref Vertex vertex,
                                          double param)
    {
        Edge F_Edge = edge;
        F_Edge.Orient(Forward);

        Orientation orOfVertex;
        Vertex anotherVertex;
        Exploration.Vertices(F_Edge, out Vertex v1, out Vertex v2);
        if (constVertex.IsSame(v1))
        {
            orOfVertex = Reversed;
            anotherVertex = v2;
        }
        else
        {
            orOfVertex = Forward;
            anotherVertex = v1;
        }

        BrepBuilder bb = new();
        Tool.Range(F_Edge, out double fpar, out double lpar);
        if (orOfVertex == Forward)
        {
            fpar = param;
        }
        else
        {
            lpar = param;
        }
        bb.Range(F_Edge, fpar, lpar);

        F_Edge.Free(true);
        bb.Remove(F_Edge, anotherVertex);
        vertex.Orient(orOfVertex);
        bb.Add(F_Edge, vertex);
    }

    // Finds the edge connected to <theVertex> in the compound <theComp>
    // that is closest to bisector plane angularly.
    // Removes found edge from <theComp>
    //<theAxis> is the axis of bisector plane
    private static Edge FindEdgeCloseToBisectorPlane(in Vertex vertex,
                                                            in Compound comp,
                                                            in Axis3D axis)
    {
        IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(comp, ShapeType.Vertex, ShapeType.Edge, vEmap);

        Edge minEdge = new();
        if (!vEmap.ContainsKey(vertex))
        {
            return minEdge;
        }

        BrepBuilder bb = new();

        NCollection_List<Shape> edges = vEmap.GetValue(vertex);
        if (edges.Extent() == 1)
        {
            minEdge = ShapeCasts.Edge(edges.First());
        }
        else
        {
            NCollection_List<Shape>.Iterator itl = new(edges);
            double minAngle = RealLast();
            for (; itl.More(); itl.Next())
            {
                Edge edge = ShapeCasts.Edge(itl.Value());
                Wire wire = new();
                bb.MakeWire(ref wire);
                bb.Add(wire, edge);
                Plane3D pln = new();
                ComputeAveragePlaneAndMaxDeviation(wire, ref pln, out bool issing);
                double angle;
                if (issing) // edge is a segment of line
                {
                    //<anAngle> is angle between <anEdge> and its projection on bisector plane
                    EdgeCurveAdaptor bAcurve = new(edge);
                    Point3D firstPnt = bAcurve.Value(bAcurve.FirstParameter());
                    Point3D lastPnt = bAcurve.Value(bAcurve.LastParameter());
                    Vector3D edgeVec = new(firstPnt, lastPnt);
                    Axis3D edgeAxis = new(firstPnt, new Direction3D(edgeVec));
                    angle = edgeAxis.Direction.AngleTo(axis.Direction);
                    if (angle > M_PI / 2)
                    {
                        angle = M_PI - angle;
                    }
                    angle = M_PI / 2 - angle;
                }
                else
                {
                    angle = pln.Axis.AngleTo(axis);
                    if (angle > M_PI / 2)
                    {
                        angle = M_PI - angle;
                    }
                }

                if (angle < minAngle)
                {
                    minAngle = angle;
                    minEdge = edge;
                }
            }
        } // else (more than one edge)

        bb.Remove(comp, minEdge);
        return minEdge;
    }

    private static bool FindMiddleEdges(in Vertex vertex1,
                                        in Vertex vertex2,
                                        in Axis3D axis,
                                        in Compound comp,
                                        NCollection_List<Shape> elist)
    {
        IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(comp, ShapeType.Vertex, ShapeType.Edge, vEmap);
        if (vEmap.Count == 0)
        {
            return false;
        }

        if (!vEmap.ContainsKey(vertex1) || !vEmap.ContainsKey(vertex2))
        {
            return false;
        }

        Vertex curVertex = vertex1;
        for (;;)
        {
            Edge curEdge;

            curEdge = FindEdgeCloseToBisectorPlane(curVertex, comp, axis);
            if (curEdge.IsNull)
            {
                return false;
            }

            Exploration.Vertices(curEdge, out Vertex v1, out Vertex v2);
            curVertex = (v1.IsSame(curVertex)) ? v2 : v1;

            elist.Append(curEdge);
            if (curVertex.IsSame(vertex2))
            {
                return true;
            }
        }
    }

    private static bool FindCommonVertex(in Edge firstEdge,
                                         in Edge lastEdge,
                                         in Vertex firstVertex,
                                         in Vertex lastVertex,
                                         ref Vertex commonVertex)
    {
        if (!firstVertex.IsSame(lastVertex))
        {
            bool commonVertexExists = Exploration.CommonVertex(firstEdge, lastEdge, ref commonVertex);
            return commonVertexExists;
        }

        Exploration.Vertices(firstEdge, out Vertex v1, out Vertex v2);
        Exploration.Vertices(lastEdge, out Vertex v3, out Vertex v4);

        if (v1.IsSame(firstVertex))
        {
            if (v2.IsSame(v3) || v2.IsSame(v4))
            {
                commonVertex = v2;
                return true;
            }
        }
        else
        {
            if (v1.IsSame(v3) || v1.IsSame(v4))
            {
                commonVertex = v1;
                return true;
            }
        }

        return false;
    }
}
