// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>
/// The Cells Builder: splits the arguments with the General Fuse algorithm into parts (cells) and lets the caller
/// choose the parts of the result by the arguments they belong to (<see cref="AddToResult"/>,
/// <see cref="RemoveFromResult"/>), with materials whose internal boundaries can be removed
/// (<see cref="RemoveInternalBoundaries"/>), and makes containers of them (<see cref="MakeContainers"/>).
/// </summary>
/// <remarks>
/// <para>
/// The result is built step by step: each Add/Remove call changes it. <see cref="BuilderShape.Shape"/>
/// publishes (freezes) the current result; OCCT then adds the next parts into that same compound, changing the shape
/// the caller holds. The port adds them in a working overlay instead, so a published result never changes and the
/// next one is OCCT's, a new compound holding the same parts and the new ones.
/// </para>
/// <para>
/// <see cref="GetAllParts"/> is the compound of all the parts, unfrozen until published like the result.
/// <see cref="RemoveInternalBoundaries"/> runs ShapeUpgrade_UnifySameDomain on containers of the parts, which hold
/// the arguments' own sub-shapes where they were not split: with frozen arguments it copies them at its entry
/// (decision 766), so the arguments are never written.
/// </para>
/// </remarks>
public class CellsBuilder : BooleanBuilder
{
    protected Shape _allParts = new(); //!< All split parts of the arguments
    protected NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _index = new(ShapeHasher.Instance); //!< Connection map from all splits parts to the argument shapes from which they were created
    protected NCollection_IntDataMap<NCollection_List<Shape>> _materials = new(100); //!< Map of assigned materials (material -> list of shape)
    protected NCollection_DataMap<Shape, int> _shapeMaterial = new(ShapeHasher.Instance); //!< Map of assigned materials (shape -> material)
    protected NCollection_DataMap<Shape, Shape> _mapModified = new(ShapeHasher.Instance); //!< Local modification map to track unification of the splits

    public CellsBuilder()
    {
    }

    /// <summary>Redefined method Clear - clears the contents.</summary>
    public override void Clear()
    {
        base.Clear();
        _index.Clear();
        _materials.Clear();
        _shapeMaterial.Clear();
        _mapModified.Clear();
    }

    /// <summary>All split parts of the arguments (frozen).</summary>
    public Shape GetAllParts()
    {
        _allParts.TShape()?.FreezeGraph();
        return _allParts;
    }

    /// <summary>All split parts of the arguments, unfrozen, for ported code.</summary>
    internal Shape InternalGetAllParts() => _allParts;

    protected override void PerformInternal1(PaveFiller filler, ProgressRange range)
    {
        // Avoid filling history after GF operation as later
        // in this method the result shape will be nullified
        bool isHistory = HasHistory;
        SetToFillHistory(false);
        // Perform splitting of the arguments
        using var ps = new ProgressScope(range, "Performing MakeCells operation", 1);
        base.PerformInternal1(filler, ps.Next());
        if (HasErrors)
        {
            return;
        }

        // index all the parts to its origins
        IndexParts();

        // and nullify <myShape> for building the result;
        RemoveAllFromResult();

        // Restore user's history settings
        SetToFillHistory(isHistory);
    }

    /// <summary>Indexes the parts for quick access to the arguments.</summary>
    protected void IndexParts()
    {
        BrepBuilder bb = new();
        // all split parts of the shapes
        Compound allParts = new();
        bb.MakeCompound(ref allParts);
        //
        NCollection_Map<Shape> mFence = new(ShapeHasher.Instance);
        NCollection_IntMap mDims = new();
        //
        NCollection_List<Shape>.Iterator it = new(_arguments);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();

            NCollection_List<Shape> lSubS = new();
            AlgoTools.TreatCompound(s, lSubS);
            for (NCollection_List<Shape>.Iterator itSub = new(lSubS); itSub.More(); itSub.Next())
            {
                Shape ss = itSub.Value();
                int iDim = AlgoTools.Dimension(ss);
                mDims.Add(iDim);
                ShapeType type = TypeToExplore(iDim);
                Explorer exp = new(ss, type);
                for (; exp.More(); exp.Next())
                {
                    Shape st = exp.Current();
                    if (!_images.Find(st, out NCollection_List<Shape> pLSIm))
                    {
                        if (!_index.FindFromKey(st, out NCollection_List<Shape> pLS))
                        {
                            pLS = new NCollection_List<Shape>();
                            _index.Add(st, pLS);
                        }
                        pLS.Append(s);
                        //
                        if (mFence.Add(st))
                        {
                            bb.Add(allParts, st);
                        }
                        //
                        continue;
                    }
                    //
                    NCollection_List<Shape>.Iterator itIm = new(pLSIm);
                    for (; itIm.More(); itIm.Next())
                    {
                        Shape stIm = itIm.Value();
                        //
                        if (!_index.FindFromKey(stIm, out NCollection_List<Shape> pLS))
                        {
                            pLS = new NCollection_List<Shape>();
                            _index.Add(stIm, pLS);
                        }
                        pLS.Append(s);
                        //
                        if (mFence.Add(stIm))
                        {
                            bb.Add(allParts, stIm);
                        }
                    } // for (; aItIm.More(); aItIm.Next()) {
                } // for (; aExp.More(); aExp.Next()) {
            } // for (; itSub.More(); itSub.Next())
        } // for (; aIt.More(); aIt.Next()) {
        //
        _allParts = allParts;
        //
        if (mDims.Extent() == 1)
        {
            return;
        }
        //
        // for the multi-dimensional case
        // add sub-shapes of the splits into the <myIndex> map
        //
        int i, nbS = _index.Extent();
        for (i = 1; i <= nbS; ++i)
        {
            Shape sp = _index.FindKey(i);
            NCollection_List<Shape> lsOr = _index[i];
            //
            int iType = AlgoTools.Dimension(sp);
            NCollection_IntMap.Iterator itM = new(mDims);
            for (; itM.More(); itM.Next())
            {
                int k = itM.Key();
                if (k >= iType)
                {
                    continue;
                }
                //
                Explorer exp = new(sp, TypeToExplore(k));
                for (; exp.More(); exp.Next())
                {
                    Shape ss = exp.Current();
                    if (!_index.FindFromKey(ss, out NCollection_List<Shape> pLSSOr))
                    {
                        // C++ copies the list into the map
                        _index.Add(ss, new NCollection_List<Shape>(lsOr));
                        continue;
                    }
                    // add ancestors of the shape to the ancestors of the sub-shape
                    NCollection_List<Shape>.Iterator itLS = new(lsOr);
                    for (; itLS.More(); itLS.Next())
                    {
                        Shape sOr = itLS.Value();
                        // provide uniqueness of the ancestors
                        NCollection_List<Shape>.Iterator itLSS = new(pLSSOr);
                        for (; itLSS.More(); itLSS.Next())
                        {
                            if (sOr.IsSame(itLSS.Value()))
                            {
                                break;
                            }
                        }
                        //
                        if (!itLSS.More())
                        {
                            pLSSOr.Append(sOr);
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Adds to the result the parts that belong to all the shapes of <paramref name="lsToTake"/> and to none of
    /// <paramref name="lsToAvoid"/>, with the material <paramref name="material"/> (0: none); with
    /// <paramref name="update"/> the internal boundaries between the parts of the same material are removed.
    /// </summary>
    public void AddToResult(NCollection_List<Shape> lsToTake,
                            NCollection_List<Shape> lsToAvoid,
                            int material = 0,
                            bool update = false)
    {
        // find parts
        NCollection_List<Shape> parts = new();
        FindParts(lsToTake, lsToAvoid, parts);
        if (parts.IsEmpty())
        {
            return;
        }
        //
        // collect result parts to avoid multiple adding of the same parts
        NCollection_Map<Shape> resParts = new(ShapeHasher.Instance);
        Iterator it = new(_shape);
        for (; it.More(); it.Next())
        {
            resParts.Add(it.Value());
        }
        //
        bool bChanged = false;
        // add parts to result
        // Kerf: OCCT adds the parts into myShape in place; once Shape() has published it, the working overlay takes
        // them, and the result is a new compound holding its parts and the new ones (a published result never changes)
        NCollection_List<Shape>.Iterator itLP = new(parts);
        using (var overlay = WorkingOverlay.Begin(_shape))
        {
            for (; itLP.More(); itLP.Next())
            {
                Shape part = itLP.Value();
                // provide uniqueness of the parts
                if (resParts.Add(part) && !_shapeMaterial.IsBound(part))
                {
                    new BrepBuilder().Add(_shape, part);
                    bChanged = true;
                }
            }
            _shape = overlay.Finish(_shape);
        }
        //
        // update the material
        if (material != 0)
        {
            NCollection_List<Shape> lsp = new();
            itLP.Initialize(parts);
            for (; itLP.More(); itLP.Next())
            {
                Shape part = itLP.Value();
                if (!_shapeMaterial.IsBound(part))
                {
                    _shapeMaterial.Bind(part, material);
                    lsp.Append(part);
                }
            } // for (; aIt.More(); aIt.Next()) {
            //
            if (lsp.Extent() != 0)
            {
                if (!_materials.Find(material, out NCollection_List<Shape> pLS))
                {
                    pLS = _materials.Bound(material, new NCollection_List<Shape>());
                }
                pLS.Append(lsp);
            } // if (aLSP.Extent()) {
        } // if (theMaterial != 0) {
        //
        if (!update)
        {
            if (bChanged)
            {
                PrepareHistory(new ProgressRange());
            }
        }
        else
        {
            RemoveInternalBoundaries();
        }
    }

    /// <summary>
    /// Adds all the parts to the result, with the material <paramref name="material"/> (0: none); with
    /// <paramref name="update"/> the internal boundaries between them are removed.
    /// </summary>
    public void AddAllToResult(int material = 0, bool update = false)
    {
        _shapeMaterial.Clear();
        _materials.Clear();
        _mapModified.Clear();
        //
        _shape = _allParts;
        //
        if (material != 0)
        {
            NCollection_List<Shape> pLSM = _materials.Bound(material, new NCollection_List<Shape>());
            //
            Iterator it = new(_allParts);
            for (; it.More(); it.Next())
            {
                Shape part = it.Value();
                _shapeMaterial.Bind(part, material);
                pLSM.Append(part);
            }
        }
        //
        if (!update)
        {
            PrepareHistory(new ProgressRange());
        }
        else
        {
            RemoveInternalBoundaries();
        }
    }

    /// <summary>
    /// Removes from the result the parts that belong to all the shapes of <paramref name="lsToTake"/> and to none
    /// of <paramref name="lsToAvoid"/>.
    /// </summary>
    public void RemoveFromResult(NCollection_List<Shape> lsToTake, NCollection_List<Shape> lsToAvoid)
    {
        // find parts
        NCollection_List<Shape> parts = new();
        FindParts(lsToTake, lsToAvoid, parts);
        if (parts.IsEmpty())
        {
            return;
        }
        //
        // collect parts into the map and remove parts from materials
        NCollection_Map<Shape> partsToRemove = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itP = new(parts);
        for (; itP.More(); itP.Next())
        {
            Shape part = itP.Value();
            partsToRemove.Add(part);
            //
            if (_shapeMaterial.Find(part, out int pMaterial))
            {
                if (_materials.Find(pMaterial, out NCollection_List<Shape> pLSM))
                {
                    NCollection_List<Shape>.Iterator itM = new(pLSM);
                    for (; itM.More(); itM.Next())
                    {
                        if (part.IsSame(itM.Value()))
                        {
                            pLSM.Remove(itM);
                            break;
                        }
                    }
                }
                _shapeMaterial.UnBind(part);
            }
        }
        //
        BrepBuilder bb = new();
        Compound result = new();
        bb.MakeCompound(ref result);
        bool bChanged = false;
        //
        Iterator it = new(_shape);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            ShapeType type = s.ShapeType();
            if (type != ShapeType.Wire && type != ShapeType.Shell && type != ShapeType.CompSolid)
            {
                // basic element
                if (partsToRemove.Contains(s))
                {
                    bChanged = true;
                    continue;
                }
                bb.Add(result, s);
            }
            else
            {
                // container
                Compound sc = new();
                bb.MakeCompound(ref sc);
                bool bSCNotEmpty = false;
                //
                Iterator itSC = new(s);
                for (; itSC.More(); itSC.Next())
                {
                    Shape ss = itSC.Value();
                    if (partsToRemove.Contains(ss))
                    {
                        bChanged = true;
                        continue;
                    }
                    //
                    bSCNotEmpty = true;
                    bb.Add(sc, ss);
                }
                //
                if (bSCNotEmpty)
                {
                    Shape res = result;
                    MakeTypedContainers(sc, ref res);
                }
            }
        }
        //
        if (bChanged)
        {
            _shape = result;
            //
            PrepareHistory(new ProgressRange());
        }
    }

    /// <summary>Removes all the parts from the result.</summary>
    public void RemoveAllFromResult()
    {
        // empty compound
        Compound c = new();
        new BrepBuilder().MakeCompound(ref c);
        _shape = c;
        //
        _materials.Clear();
        _shapeMaterial.Clear();
        _mapModified.Clear();
        //
        PrepareHistory(new ProgressRange());
    }

    /// <summary>Removes the internal boundaries between the parts of the same material.</summary>
    public void RemoveInternalBoundaries()
    {
        if (_materials.IsEmpty())
        {
            return;
        }
        //
        BrepBuilder bb = new();
        Compound result = new();
        bb.MakeCompound(ref result);
        //
        bool bChanged = false;
        // try to remove the internal boundaries between the
        // shapes of the same material
        NCollection_IntDataMap<NCollection_List<Shape>>.Iterator itM = new(_materials);
        NCollection_List<Shape>[] lsUnify = [new(), new()];
        NCollection_Map<Shape>[] keepMap = [new(ShapeHasher.Instance), new(ShapeHasher.Instance)];
        for (; itM.More(); itM.Next())
        {
            int iMaterial = itM.Key();
            NCollection_List<Shape> ls = itM.ChangeValue();
            //
            if (ls.IsEmpty())
            {
                continue;
            }
            //
            if (ls.Extent() == 1)
            {
                ShapeType type1 = ls.First().ShapeType();
                if (type1 != ShapeType.Wire && type1 != ShapeType.Shell && type1 != ShapeType.CompSolid)
                {
                    bb.Add(result, ls.First());
                    continue;
                }
            }
            //
            // check the shapes of the same material to be of the same type
            NCollection_List<Shape>.Iterator itLS = new(ls);
            ShapeType type = itLS.Value().ShapeType();
            for (itLS.Next(); itLS.More(); itLS.Next())
            {
                if (type != itLS.Value().ShapeType())
                {
                    break;
                }
            }

            if (itLS.More())
            {
                // add the warning
                Compound multiDimS = new();
                bb.MakeCompound(ref multiDimS);
                bb.Add(multiDimS, ls.First());
                bb.Add(multiDimS, itLS.Value());
                AddWarning(new AlertRemovalOfIBForMDimShapes(multiDimS));
            }
            else
            {
                if (type == ShapeType.Edge || type == ShapeType.Face)
                {
                    // for edges and faces, just collect shapes to unify them later after exiting the loop;
                    // collect boundaries of shapes of current material in the keep map
                    int iType = (type == ShapeType.Edge ? 0 : 1);
                    CollectMaterialBoundaries(ls, keepMap[iType]);
                    // save shapes to unify later
                    NCollection_List<Shape> copy = new(ls);
                    lsUnify[iType].Append(copy);
                    continue;
                }
                else
                {
                    // aType is Solid;
                    // remove internal faces between solids of the same material just now
                    NCollection_List<Shape> lsNew = new();
                    if (RemoveInternals(ls, lsNew))
                    {
                        bChanged = true;
                        // update materials maps
                        for (itLS.Initialize(lsNew); itLS.More(); itLS.Next())
                        {
                            Shape s = itLS.Value();
                            _shapeMaterial.Bind(s, iMaterial);
                        }
                        ls.Assign(lsNew);
                    }
                }
            }
            // add shapes to result (multidimensional and solids)
            for (itLS.Initialize(ls); itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                bb.Add(result, s);
            }
        }

        // remove internal boundaries for edges and faces
        for (int iType = 0; iType < 2; ++iType)
        {
            if (lsUnify[iType].IsEmpty())
            {
                continue;
            }
            NCollection_List<Shape> lsn = new();
            if (RemoveInternals(lsUnify[iType], lsn, keepMap[iType]))
            {
                bChanged = true;
            }
            // add shapes to result ([unified] edges or faces)
            for (NCollection_List<Shape>.Iterator itLS = new(lsn); itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                bb.Add(result, s);
            }
        }
        //
        if (bChanged)
        {
            // add shapes without material into result
            Iterator it = new(_shape);
            for (; it.More(); it.Next())
            {
                Shape s = it.Value();
                //
                if (_shapeMaterial.IsBound(s))
                {
                    continue;
                }
                //
                // check if it is not a collection
                ShapeType type = s.ShapeType();
                if (type != ShapeType.Wire && type != ShapeType.Shell && type != ShapeType.CompSolid)
                {
                    bb.Add(result, s);
                }
                else
                {
                    Compound sc = new();
                    bb.MakeCompound(ref sc);
                    bool bSCEmpty = true, bSCChanged = false;
                    //
                    Iterator itSC = new(s);
                    for (; itSC.More(); itSC.Next())
                    {
                        Shape ss = itSC.Value();
                        if (!_shapeMaterial.IsBound(ss))
                        {
                            bb.Add(sc, ss);
                            bSCEmpty = false;
                        }
                        else
                        {
                            bSCChanged = true;
                        }
                    }
                    //
                    if (bSCEmpty)
                    {
                        continue;
                    }
                    //
                    if (bSCChanged)
                    {
                        Shape res = result;
                        MakeTypedContainers(sc, ref res);
                    }
                    else
                    {
                        bb.Add(result, s);
                    }
                }
            }
            //
            _shape = result;
            //
            PrepareHistory(new ProgressRange());
        }
    }

    /// <summary>Finds the parts of the arguments to take (or remove).</summary>
    protected void FindParts(NCollection_List<Shape> lsToTake,
                             NCollection_List<Shape> lsToAvoid,
                             NCollection_List<Shape> parts)
    {
        if (lsToTake.IsEmpty())
        {
            return;
        }
        //
        // map shapes to avoid
        NCollection_Map<Shape> msToAvoid = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itArgs = new(lsToAvoid);
        for (; itArgs.More(); itArgs.Next())
        {
            Shape s = itArgs.Value();
            msToAvoid.Add(s);
        }
        //
        // map shapes to be taken
        NCollection_Map<Shape> msToTake = new(ShapeHasher.Instance);
        itArgs.Initialize(lsToTake);
        for (; itArgs.More(); itArgs.Next())
        {
            Shape s = itArgs.Value();
            msToTake.Add(s);
        }
        //
        int nbS = msToTake.Extent();
        //
        // among the shapes to be taken into result, find any one
        // of minimal dimension
        int iDimMin = 10;
        Shape sMin = new();
        //
        itArgs.Initialize(lsToTake);
        for (; itArgs.More(); itArgs.Next())
        {
            Shape s = itArgs.Value();
            int iDim = AlgoTools.Dimension(s);
            if (iDim < iDimMin)
            {
                iDimMin = iDim;
                sMin = s;
            }
        }
        //
        // among the split parts of the shape of minimal dimension
        // look for the parts to be taken into result
        ShapeType type = TypeToExplore(iDimMin);
        Explorer exp = new(sMin, type);
        for (; exp.More(); exp.Next())
        {
            Shape st = exp.Current();
            // get split parts of the shape
            NCollection_List<Shape> lstIm = new();
            if (!_images.IsBound(st))
            {
                lstIm.Append(st);
            }
            else
            {
                lstIm.Assign(_images.Find(st));
            }
            //
            NCollection_List<Shape>.Iterator itIm = new(lstIm);
            for (; itIm.More(); itIm.Next())
            {
                Shape part = itIm.Value();
                //
                if (!_index.Contains(part))
                {
                    continue;
                }
                //
                // get input shapes in which the split part is contained
                NCollection_List<Shape> ls = _index.FindFromKey(part);
                if (ls.Extent() < nbS)
                {
                    continue;
                }
                //
                // check that input shapes containing the part should not be avoided
                NCollection_Map<Shape> ms = new(ShapeHasher.Instance);
                itArgs.Initialize(ls);
                for (; itArgs.More(); itArgs.Next())
                {
                    Shape s = itArgs.Value();
                    ms.Add(s);
                    if (msToAvoid.Contains(s))
                    {
                        break;
                    }
                }
                //
                if (itArgs.More())
                {
                    continue;
                }
                //
                // check that all shapes which should be taken contain the part
                itArgs.Initialize(lsToTake);
                for (; itArgs.More(); itArgs.Next())
                {
                    if (!ms.Contains(itArgs.Value()))
                    {
                        break;
                    }
                }
                //
                if (!itArgs.More())
                {
                    parts.Append(part);
                }
            }
        }
    }

    /// <summary>
    /// Makes the containers of the parts in the result: wires of the edges, shells of the faces, compsolids of the
    /// solids, by connexity.
    /// </summary>
    public void MakeContainers()
    {
        BrepBuilder bb = new();
        Compound result = new();
        bb.MakeCompound(ref result);
        //
        // basic elements of type EDGE, FACE and SOLID added into result
        NCollection_List<Shape>[] ls = [new(), new(), new()];
        //
        Iterator it = new(_shape);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            //
            int iDim = AlgoTools.Dimension(s);
            if (iDim <= 0)
            {
                bb.Add(result, s);
                continue;
            }
            //
            ls[iDim - 1].Append(s);
        }
        //
        for (int i = 0; i < 3; ++i)
        {
            if (ls[i].IsEmpty())
            {
                continue;
            }
            //
            Compound c = new();
            bb.MakeCompound(ref c);
            NCollection_List<Shape>.Iterator itLS = new(ls[i]);
            for (; itLS.More(); itLS.Next())
            {
                bb.Add(c, itLS.Value());
            }
            //
            Shape res = result;
            MakeTypedContainers(c, ref res);
        }
        _shape = result;
    }

    /// <summary>Removes the internal boundaries between the shapes of the list (unifies them).</summary>
    protected bool RemoveInternals(NCollection_List<Shape> ls,
                                   NCollection_List<Shape> lsNew,
                                   NCollection_Map<Shape>? mapKeepBnd = null)
    {
        bool bRemoved = false;
        if (ls.Extent() < 2)
        {
            lsNew.Assign(ls);
            return bRemoved;
        }
        //
        ShapeType type = ls.First().ShapeType();
        //
        if (type == ShapeType.Edge || type == ShapeType.Face)
        {
            //
            // make container
            BrepBuilder bb = new();
            Shape shape = new();
            //
            AlgoTools.MakeContainer((type == ShapeType.Face) ? ShapeType.Shell : ShapeType.Wire, ref shape);
            //
            for (NCollection_List<Shape>.Iterator it = new(ls); it.More(); it.Next())
            {
                Shape s = it.Value();
                bb.Add(shape, s);
            }
            //
            // Unify same domain
            bool bFaces, bEdges;
            //
            bFaces = (type == ShapeType.Face);
            bEdges = (type == ShapeType.Edge);
            UnifySameDomain unify = new(shape, bEdges, bFaces);
            unify.KeepShapes(mapKeepBnd ?? new NCollection_Map<Shape>(ShapeHasher.Instance));
            unify.Build();
            Shape sNew = unify.InternalShape();
            //
            Explorer exp = new(sNew, type);
            for (; exp.More(); exp.Next())
            {
                Shape sn = exp.Current();
                lsNew.Append(sn);
            }
            //
            if (lsNew.IsEmpty())
            {
                // add the warning
                if (bFaces)
                {
                    AddWarning(new AlertRemovalOfIBForFacesFailed(shape));
                }
                else
                {
                    AddWarning(new AlertRemovalOfIBForEdgesFailed(shape));
                }
                //
                lsNew.Assign(ls);
                return bRemoved;
            }
            //
            // fill map of modified shapes
            NCollection_IndexedMap<Shape> mg = new(ShapeHasher.Instance);
            int i, nb;
            //
            Exploration.MapShapes(shape, ShapeType.Vertex, mg);
            Exploration.MapShapes(shape, ShapeType.Edge, mg);
            Exploration.MapShapes(shape, ShapeType.Face, mg);
            //
            History history = unify.InternalHistory()!;
            nb = mg.Extent();
            for (i = 1; i <= nb; ++i)
            {
                Shape ss = mg[i];
                bool hasMaterial = _shapeMaterial.Find(ss, out int pMaterial);
                foreach (Shape su in history.Modified(ss))
                {
                    _mapModified.Bind(ss, su);
                    bRemoved = true;
                    if (hasMaterial && !_shapeMaterial.IsBound(su))
                    {
                        _shapeMaterial.Bind(su, pMaterial);
                    }
                }
            }
        }
        else if (type == ShapeType.Solid)
        {
            BrepBuilder bb = new();
            Compound solids = new();
            bb.MakeCompound(ref solids);
            //
            NCollection_List<Shape>.Iterator itLS = new(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape sol = itLS.Value();
                bb.Add(solids, sol);
            }
            //
            // Make connexity blocks of solids to create from each isolated block one solid.
            // It will allow attaching internal entities of the solids to new solid.
            NCollection_List<Shape> lcb = new();
            AlgoTools.MakeConnexityBlocks(solids, ShapeType.Face, ShapeType.Solid, lcb);
            //
            // for each block remove internal faces
            NCollection_List<Shape>.Iterator itLCB = new(lcb);
            for (; itLCB.More(); itLCB.Next())
            {
                Shape cb = itLCB.Value();
                //
                // Map faces and solids to find boundary faces that can be removed
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> dmfs = new(ShapeHasher.Instance);
                // internal entities
                NCollection_List<Shape> lsInt = new();
                //
                Iterator itS = new(cb);
                for (; itS.More(); itS.Next())
                {
                    Shape sol = itS.Value();
                    //
                    Iterator itIS = new(sol);
                    for (; itIS.More(); itIS.Next())
                    {
                        Shape si = itIS.Value();
                        if (si.Orientation() == Internal)
                        {
                            lsInt.Append(si);
                        }
                        else
                        {
                            Iterator itF = new(si);
                            for (; itF.More(); itF.Next())
                            {
                                Shape f = itF.Value();
                                if (!dmfs.FindFromKey(f, out NCollection_List<Shape> pLSols))
                                {
                                    pLSols = new NCollection_List<Shape>();
                                    dmfs.Add(f, pLSols);
                                }
                                pLSols.Append(sol);
                            }
                        }
                    }
                }
                //
                // to build unified solid, select only faces attached to only one solid
                NCollection_List<Shape> lfUnique = new();
                int i, nb = dmfs.Extent();
                for (i = 1; i <= nb; ++i)
                {
                    if (dmfs[i].Extent() == 1)
                    {
                        lfUnique.Append(dmfs.FindKey(i));
                    }
                }
                //
                if (nb == lfUnique.Extent())
                {
                    // no faces to remove
                    itS.Initialize(cb);
                    for (; itS.More(); itS.Next())
                    {
                        lsNew.Append(itS.Value());
                    }
                    continue;
                }
                //
                // build new solid
                BuilderSolid bs = new();
                bs.SetShapes(lfUnique);
                bs.Perform();
                //
                if (bs.HasErrors || bs.Areas().Extent() != 1)
                {
                    // add the warning
                    {
                        Compound uniqeFaces = new();
                        bb.MakeCompound(ref uniqeFaces);
                        NCollection_List<Shape>.Iterator itLFUniqe = new(lfUnique);
                        for (; itLFUniqe.More(); itLFUniqe.Next())
                        {
                            bb.Add(uniqeFaces, itLFUniqe.Value());
                        }
                        //
                        AddWarning(new AlertRemovalOfIBForSolidsFailed(uniqeFaces));
                    }
                    //
                    itS.Initialize(cb);
                    for (; itS.More(); itS.Next())
                    {
                        lsNew.Append(itS.Value());
                    }
                    continue;
                }
                //
                _report.Merge(bs.GetReport());
                //
                Shape sNew = bs.Areas().First();
                //
                // put all internal parts into new solid
                sNew.Free(true);
                NCollection_List<Shape>.Iterator itLSI = new(lsInt);
                for (; itLSI.More(); itLSI.Next())
                {
                    bb.Add(sNew, itLSI.Value());
                }
                sNew.Free(false);
                //
                lsNew.Append(sNew);
                bRemoved = true;

                // Save information about the fuse of the solids into a history map
                itS.Initialize(cb);
                for (; itS.More(); itS.Next())
                {
                    _mapModified.Bind(itS.Value(), sNew);
                }
            }
        }
        return bRemoved;
    }

    /// <summary>The splits of the shape, with the local unification of the splits taken into account.</summary>
    protected override NCollection_List<Shape>? LocModified(in Shape s)
    {
        // Get shape's modification coming from GF operation
        NCollection_List<Shape>? pLSp = base.LocModified(s);
        if (_mapModified.IsEmpty())
        {
            // No local modifications
            return pLSp;
        }

        _histShapes.Clear();

        // Check if the shape (or its splits) has participated in unification
        if (pLSp is null)
        {
            // No splits from GF operation.
            // Check if the shape has been unified with other shapes
            if (!_mapModified.Find(s, out Shape pSU))
            {
                return null;
            }

            _histShapes.Append(pSU);
        }
        else
        {
            NCollection_Map<Shape> mFence = new(ShapeHasher.Instance);
            // Process all GF splits and check them for local unification with other shapes
            NCollection_List<Shape>.Iterator it = new(pLSp);
            for (; it.More(); it.Next())
            {
                Shape pSp = it.Value();
                if (_mapModified.Find(pSp, out Shape pSU))
                {
                    pSp = pSU;
                }
                if (mFence.Add(pSp))
                {
                    _histShapes.Append(pSp);
                }
            }
        }
        return _histShapes;
    }

    // Makes the typed containers (wires, shells, compsolids) of the connexity blocks of the parts and adds them to
    // theResult (OCCT's file-static MakeTypedContainers).
    private static void MakeTypedContainers(in Shape sc, ref Shape result)
    {
        ShapeType containerType, connexityType, partType;
        //
        partType = TypeToExplore(AlgoTools.Dimension(sc));
        switch (partType)
        {
            case ShapeType.Edge:
            {
                containerType = ShapeType.Wire;
                connexityType = ShapeType.Vertex;
                break;
            }
            case ShapeType.Face:
            {
                containerType = ShapeType.Shell;
                connexityType = ShapeType.Edge;
                break;
            }
            case ShapeType.Solid:
            {
                containerType = ShapeType.CompSolid;
                connexityType = ShapeType.Face;
                break;
            }
            default:
                return;
        }
        //
        NCollection_List<Shape> lcb = new();
        AlgoTools.MakeConnexityBlocks(sc, connexityType, partType, lcb);
        if (lcb.IsEmpty())
        {
            return;
        }
        //
        BrepBuilder bb = new();
        Explorer exp = new();
        NCollection_List<Shape>.Iterator itCB = new();
        //
        itCB.Initialize(lcb);
        for (; itCB.More(); itCB.Next())
        {
            Shape rcb = new();
            AlgoTools.MakeContainer(containerType, ref rcb);
            //
            Shape cb = itCB.Value();
            exp.Init(cb, partType);
            for (; exp.More(); exp.Next())
            {
                Shape cbs = exp.Current();
                bb.Add(rcb, cbs);
            }
            //
            if (containerType == ShapeType.Shell)
            {
                AlgoTools.OrientFacesOnShell(ref rcb);
            }
            //
            bb.Add(result, rcb);
        }
    }

    // Adds to theMapKeepBnd the boundary shapes of the area defined by shapes from the list.
    private static void CollectMaterialBoundaries(NCollection_List<Shape> ls, NCollection_Map<Shape> mapKeepBnd)
    {
        ShapeType type = ls.First().ShapeType();
        ShapeType typeSubsh = (type == ShapeType.Face ? ShapeType.Edge : ShapeType.Vertex);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapSubSh = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(ls);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            Exploration.MapShapesAndAncestors(s, typeSubsh, type, mapSubSh);
        }
        for (int i = 1; i <= mapSubSh.Extent(); i++)
        {
            // check if the subshape belongs to boundary of the area
            if (mapSubSh[i].Extent() == 1)
            {
                // add to theMapKeepBnd
                mapKeepBnd.Add(mapSubSh.FindKey(i));
            }
        }
    }

    private static ShapeType TypeToExplore(int dim)
    {
        ShapeType ret;
        //
        switch (dim)
        {
            case 0:
                ret = ShapeType.Vertex;
                break;
            case 1:
                ret = ShapeType.Edge;
                break;
            case 2:
                ret = ShapeType.Face;
                break;
            case 3:
                ret = ShapeType.Solid;
                break;
            default:
                ret = global::Kerf.Topology.ShapeType.Shape;
                break;
        }
        return ret;
    }
}
