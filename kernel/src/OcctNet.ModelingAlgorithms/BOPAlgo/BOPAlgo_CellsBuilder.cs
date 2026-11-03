// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The Cells Builder: splits the arguments with the General Fuse algorithm into parts (cells) and lets the caller
/// choose the parts of the result by the arguments they belong to (<see cref="AddToResult"/>,
/// <see cref="RemoveFromResult"/>), with materials whose internal boundaries can be removed
/// (<see cref="RemoveInternalBoundaries"/>), and makes containers of them (<see cref="MakeContainers"/>).
/// </summary>
/// <remarks>
/// <para>
/// The result is built step by step: each Add/Remove call changes it. <see cref="BOPAlgo_BuilderShape.Shape"/>
/// publishes (freezes) the current result; OCCT then adds the next parts into that same compound, changing the shape
/// the caller holds. The port adds them into a new compound holding the same parts instead (<c>// OCCT.NET</c>), so a
/// published result never changes and the next one is OCCT's.
/// </para>
/// <para>
/// <see cref="GetAllParts"/> is the compound of all the parts, unfrozen until published like the result.
/// <see cref="RemoveInternalBoundaries"/> runs ShapeUpgrade_UnifySameDomain on containers of the parts, which hold
/// the arguments' own sub-shapes where they were not split: with frozen arguments it copies them at its entry
/// (decision 766), so the arguments are never written.
/// </para>
/// </remarks>
public class BOPAlgo_CellsBuilder : BOPAlgo_Builder
{
    protected TopoDS_Shape myAllParts = new(); //!< All split parts of the arguments
    protected NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myIndex = new(TopTools_ShapeMapHasher.Instance); //!< Connection map from all splits parts to the argument shapes from which they were created
    protected NCollection_IntDataMap<NCollection_List<TopoDS_Shape>> myMaterials = new(100); //!< Map of assigned materials (material -> list of shape)
    protected NCollection_DataMap<TopoDS_Shape, int> myShapeMaterial = new(TopTools_ShapeMapHasher.Instance); //!< Map of assigned materials (shape -> material)
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapModified = new(TopTools_ShapeMapHasher.Instance); //!< Local modification map to track unification of the splits

    public BOPAlgo_CellsBuilder()
    {
    }

    /// <summary>Redefined method Clear - clears the contents.</summary>
    public override void Clear()
    {
        base.Clear();
        myIndex.Clear();
        myMaterials.Clear();
        myShapeMaterial.Clear();
        myMapModified.Clear();
    }

    /// <summary>All split parts of the arguments (frozen).</summary>
    public TopoDS_Shape GetAllParts()
    {
        myAllParts.TShape()?.FreezeGraph();
        return myAllParts;
    }

    /// <summary>All split parts of the arguments, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalGetAllParts() => myAllParts;

    protected override void PerformInternal1(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        // Avoid filling history after GF operation as later
        // in this method the result shape will be nullified
        bool isHistory = HasHistory();
        SetToFillHistory(false);
        // Perform splitting of the arguments
        using var aPS = new ProgressScope(theRange, "Performing MakeCells operation", 1);
        base.PerformInternal1(theFiller, aPS.Next());
        if (HasErrors())
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
        BRep_Builder aBB = new();
        // all split parts of the shapes
        TopoDS_Compound anAllParts = new();
        aBB.MakeCompound(ref anAllParts);
        //
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IntMap aMDims = new();
        //
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(myArguments);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();

            NCollection_List<TopoDS_Shape> aLSubS = new();
            BOPTools_AlgoTools.TreatCompound(aS, aLSubS);
            for (NCollection_List<TopoDS_Shape>.Iterator itSub = new(aLSubS); itSub.More(); itSub.Next())
            {
                TopoDS_Shape aSS = itSub.Value();
                int iDim = BOPTools_AlgoTools.Dimension(aSS);
                aMDims.Add(iDim);
                TopAbs_ShapeEnum aType = TypeToExplore(iDim);
                TopExp_Explorer aExp = new(aSS, aType);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aST = aExp.Current();
                    if (!myImages.Find(aST, out NCollection_List<TopoDS_Shape> pLSIm))
                    {
                        if (!myIndex.FindFromKey(aST, out NCollection_List<TopoDS_Shape> pLS))
                        {
                            pLS = new NCollection_List<TopoDS_Shape>();
                            myIndex.Add(aST, pLS);
                        }
                        pLS.Append(aS);
                        //
                        if (aMFence.Add(aST))
                        {
                            aBB.Add(anAllParts, aST);
                        }
                        //
                        continue;
                    }
                    //
                    NCollection_List<TopoDS_Shape>.Iterator aItIm = new(pLSIm);
                    for (; aItIm.More(); aItIm.Next())
                    {
                        TopoDS_Shape aSTIm = aItIm.Value();
                        //
                        if (!myIndex.FindFromKey(aSTIm, out NCollection_List<TopoDS_Shape> pLS))
                        {
                            pLS = new NCollection_List<TopoDS_Shape>();
                            myIndex.Add(aSTIm, pLS);
                        }
                        pLS.Append(aS);
                        //
                        if (aMFence.Add(aSTIm))
                        {
                            aBB.Add(anAllParts, aSTIm);
                        }
                    } // for (; aItIm.More(); aItIm.Next()) {
                } // for (; aExp.More(); aExp.Next()) {
            } // for (; itSub.More(); itSub.Next())
        } // for (; aIt.More(); aIt.Next()) {
        //
        myAllParts = anAllParts;
        //
        if (aMDims.Extent() == 1)
        {
            return;
        }
        //
        // for the multi-dimensional case
        // add sub-shapes of the splits into the <myIndex> map
        //
        int i, aNbS = myIndex.Extent();
        for (i = 1; i <= aNbS; ++i)
        {
            TopoDS_Shape aSP = myIndex.FindKey(i);
            NCollection_List<TopoDS_Shape> aLSOr = myIndex[i];
            //
            int iType = BOPTools_AlgoTools.Dimension(aSP);
            NCollection_IntMap.Iterator aItM = new(aMDims);
            for (; aItM.More(); aItM.Next())
            {
                int k = aItM.Key();
                if (k >= iType)
                {
                    continue;
                }
                //
                TopExp_Explorer aExp = new(aSP, TypeToExplore(k));
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aSS = aExp.Current();
                    if (!myIndex.FindFromKey(aSS, out NCollection_List<TopoDS_Shape> pLSSOr))
                    {
                        // C++ copies the list into the map
                        myIndex.Add(aSS, new NCollection_List<TopoDS_Shape>(aLSOr));
                        continue;
                    }
                    // add ancestors of the shape to the ancestors of the sub-shape
                    NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLSOr);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        TopoDS_Shape aSOr = aItLS.Value();
                        // provide uniqueness of the ancestors
                        NCollection_List<TopoDS_Shape>.Iterator aItLSS = new(pLSSOr);
                        for (; aItLSS.More(); aItLSS.Next())
                        {
                            if (aSOr.IsSame(aItLSS.Value()))
                            {
                                break;
                            }
                        }
                        //
                        if (!aItLSS.More())
                        {
                            pLSSOr.Append(aSOr);
                        }
                    }
                }
            }
        }
    }

    // OCCT.NET: frozen. OCCT adds parts into myShape in place; once Shape() has published it, the port adds into a new
    // compound holding the same parts (a published result never changes).
    private void UnfreezeResult()
    {
        if (myShape.TShape() is { } aTShape && aTShape.IsFrozen)
        {
            BRep_Builder aBB = new();
            TopoDS_Compound aC = new();
            aBB.MakeCompound(ref aC);
            for (TopoDS_Iterator aIt = new(myShape); aIt.More(); aIt.Next())
            {
                aBB.Add(aC, aIt.Value());
            }
            myShape = aC;
        }
    }

    /// <summary>
    /// Adds to the result the parts that belong to all the shapes of <paramref name="theLSToTake"/> and to none of
    /// <paramref name="theLSToAvoid"/>, with the material <paramref name="theMaterial"/> (0: none); with
    /// <paramref name="theUpdate"/> the internal boundaries between the parts of the same material are removed.
    /// </summary>
    public void AddToResult(NCollection_List<TopoDS_Shape> theLSToTake,
                            NCollection_List<TopoDS_Shape> theLSToAvoid,
                            int theMaterial = 0,
                            bool theUpdate = false)
    {
        // find parts
        NCollection_List<TopoDS_Shape> aParts = new();
        FindParts(theLSToTake, theLSToAvoid, aParts);
        if (aParts.IsEmpty())
        {
            return;
        }
        //
        // collect result parts to avoid multiple adding of the same parts
        NCollection_Map<TopoDS_Shape> aResParts = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Iterator aIt = new(myShape);
        for (; aIt.More(); aIt.Next())
        {
            aResParts.Add(aIt.Value());
        }
        //
        bool bChanged = false;
        // add parts to result
        NCollection_List<TopoDS_Shape>.Iterator aItLP = new(aParts);
        for (; aItLP.More(); aItLP.Next())
        {
            TopoDS_Shape aPart = aItLP.Value();
            // provide uniqueness of the parts
            if (aResParts.Add(aPart) && !myShapeMaterial.IsBound(aPart))
            {
                UnfreezeResult();
                new BRep_Builder().Add(myShape, aPart);
                bChanged = true;
            }
        }
        //
        // update the material
        if (theMaterial != 0)
        {
            NCollection_List<TopoDS_Shape> aLSP = new();
            aItLP.Initialize(aParts);
            for (; aItLP.More(); aItLP.Next())
            {
                TopoDS_Shape aPart = aItLP.Value();
                if (!myShapeMaterial.IsBound(aPart))
                {
                    myShapeMaterial.Bind(aPart, theMaterial);
                    aLSP.Append(aPart);
                }
            } // for (; aIt.More(); aIt.Next()) {
            //
            if (aLSP.Extent() != 0)
            {
                if (!myMaterials.Find(theMaterial, out NCollection_List<TopoDS_Shape> pLS))
                {
                    pLS = myMaterials.Bound(theMaterial, new NCollection_List<TopoDS_Shape>());
                }
                pLS.Append(aLSP);
            } // if (aLSP.Extent()) {
        } // if (theMaterial != 0) {
        //
        if (!theUpdate)
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
    /// Adds all the parts to the result, with the material <paramref name="theMaterial"/> (0: none); with
    /// <paramref name="theUpdate"/> the internal boundaries between them are removed.
    /// </summary>
    public void AddAllToResult(int theMaterial = 0, bool theUpdate = false)
    {
        myShapeMaterial.Clear();
        myMaterials.Clear();
        myMapModified.Clear();
        //
        myShape = myAllParts;
        //
        if (theMaterial != 0)
        {
            NCollection_List<TopoDS_Shape> pLSM = myMaterials.Bound(theMaterial, new NCollection_List<TopoDS_Shape>());
            //
            TopoDS_Iterator aIt = new(myAllParts);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aPart = aIt.Value();
                myShapeMaterial.Bind(aPart, theMaterial);
                pLSM.Append(aPart);
            }
        }
        //
        if (!theUpdate)
        {
            PrepareHistory(new ProgressRange());
        }
        else
        {
            RemoveInternalBoundaries();
        }
    }

    /// <summary>
    /// Removes from the result the parts that belong to all the shapes of <paramref name="theLSToTake"/> and to none
    /// of <paramref name="theLSToAvoid"/>.
    /// </summary>
    public void RemoveFromResult(NCollection_List<TopoDS_Shape> theLSToTake, NCollection_List<TopoDS_Shape> theLSToAvoid)
    {
        // find parts
        NCollection_List<TopoDS_Shape> aParts = new();
        FindParts(theLSToTake, theLSToAvoid, aParts);
        if (aParts.IsEmpty())
        {
            return;
        }
        //
        // collect parts into the map and remove parts from materials
        NCollection_Map<TopoDS_Shape> aPartsToRemove = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItP = new(aParts);
        for (; aItP.More(); aItP.Next())
        {
            TopoDS_Shape aPart = aItP.Value();
            aPartsToRemove.Add(aPart);
            //
            if (myShapeMaterial.Find(aPart, out int pMaterial))
            {
                if (myMaterials.Find(pMaterial, out NCollection_List<TopoDS_Shape> pLSM))
                {
                    NCollection_List<TopoDS_Shape>.Iterator aItM = new(pLSM);
                    for (; aItM.More(); aItM.Next())
                    {
                        if (aPart.IsSame(aItM.Value()))
                        {
                            pLSM.Remove(aItM);
                            break;
                        }
                    }
                }
                myShapeMaterial.UnBind(aPart);
            }
        }
        //
        BRep_Builder aBB = new();
        TopoDS_Compound aResult = new();
        aBB.MakeCompound(ref aResult);
        bool bChanged = false;
        //
        TopoDS_Iterator aIt = new(myShape);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            TopAbs_ShapeEnum aType = aS.ShapeType();
            if (aType != TopAbs_WIRE && aType != TopAbs_SHELL && aType != TopAbs_COMPSOLID)
            {
                // basic element
                if (aPartsToRemove.Contains(aS))
                {
                    bChanged = true;
                    continue;
                }
                aBB.Add(aResult, aS);
            }
            else
            {
                // container
                TopoDS_Compound aSC = new();
                aBB.MakeCompound(ref aSC);
                bool bSCNotEmpty = false;
                //
                TopoDS_Iterator aItSC = new(aS);
                for (; aItSC.More(); aItSC.Next())
                {
                    TopoDS_Shape aSS = aItSC.Value();
                    if (aPartsToRemove.Contains(aSS))
                    {
                        bChanged = true;
                        continue;
                    }
                    //
                    bSCNotEmpty = true;
                    aBB.Add(aSC, aSS);
                }
                //
                if (bSCNotEmpty)
                {
                    TopoDS_Shape aRes = aResult;
                    MakeTypedContainers(aSC, ref aRes);
                }
            }
        }
        //
        if (bChanged)
        {
            myShape = aResult;
            //
            PrepareHistory(new ProgressRange());
        }
    }

    /// <summary>Removes all the parts from the result.</summary>
    public void RemoveAllFromResult()
    {
        // empty compound
        TopoDS_Compound aC = new();
        new BRep_Builder().MakeCompound(ref aC);
        myShape = aC;
        //
        myMaterials.Clear();
        myShapeMaterial.Clear();
        myMapModified.Clear();
        //
        PrepareHistory(new ProgressRange());
    }

    /// <summary>Removes the internal boundaries between the parts of the same material.</summary>
    public void RemoveInternalBoundaries()
    {
        if (myMaterials.IsEmpty())
        {
            return;
        }
        //
        BRep_Builder aBB = new();
        TopoDS_Compound aResult = new();
        aBB.MakeCompound(ref aResult);
        //
        bool bChanged = false;
        // try to remove the internal boundaries between the
        // shapes of the same material
        NCollection_IntDataMap<NCollection_List<TopoDS_Shape>>.Iterator aItM = new(myMaterials);
        NCollection_List<TopoDS_Shape>[] aLSUnify = [new(), new()];
        NCollection_Map<TopoDS_Shape>[] aKeepMap = [new(TopTools_ShapeMapHasher.Instance), new(TopTools_ShapeMapHasher.Instance)];
        for (; aItM.More(); aItM.Next())
        {
            int iMaterial = aItM.Key();
            NCollection_List<TopoDS_Shape> aLS = aItM.ChangeValue();
            //
            if (aLS.IsEmpty())
            {
                continue;
            }
            //
            if (aLS.Extent() == 1)
            {
                TopAbs_ShapeEnum aType1 = aLS.First().ShapeType();
                if (aType1 != TopAbs_WIRE && aType1 != TopAbs_SHELL && aType1 != TopAbs_COMPSOLID)
                {
                    aBB.Add(aResult, aLS.First());
                    continue;
                }
            }
            //
            // check the shapes of the same material to be of the same type
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLS);
            TopAbs_ShapeEnum aType = aItLS.Value().ShapeType();
            for (aItLS.Next(); aItLS.More(); aItLS.Next())
            {
                if (aType != aItLS.Value().ShapeType())
                {
                    break;
                }
            }

            if (aItLS.More())
            {
                // add the warning
                TopoDS_Compound aMultiDimS = new();
                aBB.MakeCompound(ref aMultiDimS);
                aBB.Add(aMultiDimS, aLS.First());
                aBB.Add(aMultiDimS, aItLS.Value());
                AddWarning(new BOPAlgo_AlertRemovalOfIBForMDimShapes(aMultiDimS));
            }
            else
            {
                if (aType == TopAbs_EDGE || aType == TopAbs_FACE)
                {
                    // for edges and faces, just collect shapes to unify them later after exiting the loop;
                    // collect boundaries of shapes of current material in the keep map
                    int iType = (aType == TopAbs_EDGE ? 0 : 1);
                    CollectMaterialBoundaries(aLS, aKeepMap[iType]);
                    // save shapes to unify later
                    NCollection_List<TopoDS_Shape> aCopy = new(aLS);
                    aLSUnify[iType].Append(aCopy);
                    continue;
                }
                else
                {
                    // aType is Solid;
                    // remove internal faces between solids of the same material just now
                    NCollection_List<TopoDS_Shape> aLSNew = new();
                    if (RemoveInternals(aLS, aLSNew))
                    {
                        bChanged = true;
                        // update materials maps
                        for (aItLS.Initialize(aLSNew); aItLS.More(); aItLS.Next())
                        {
                            TopoDS_Shape aS = aItLS.Value();
                            myShapeMaterial.Bind(aS, iMaterial);
                        }
                        aLS.Assign(aLSNew);
                    }
                }
            }
            // add shapes to result (multidimensional and solids)
            for (aItLS.Initialize(aLS); aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aS = aItLS.Value();
                aBB.Add(aResult, aS);
            }
        }

        // remove internal boundaries for edges and faces
        for (int iType = 0; iType < 2; ++iType)
        {
            if (aLSUnify[iType].IsEmpty())
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> aLSN = new();
            if (RemoveInternals(aLSUnify[iType], aLSN, aKeepMap[iType]))
            {
                bChanged = true;
            }
            // add shapes to result ([unified] edges or faces)
            for (NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLSN); aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aS = aItLS.Value();
                aBB.Add(aResult, aS);
            }
        }
        //
        if (bChanged)
        {
            // add shapes without material into result
            TopoDS_Iterator aIt = new(myShape);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aS = aIt.Value();
                //
                if (myShapeMaterial.IsBound(aS))
                {
                    continue;
                }
                //
                // check if it is not a collection
                TopAbs_ShapeEnum aType = aS.ShapeType();
                if (aType != TopAbs_WIRE && aType != TopAbs_SHELL && aType != TopAbs_COMPSOLID)
                {
                    aBB.Add(aResult, aS);
                }
                else
                {
                    TopoDS_Compound aSC = new();
                    aBB.MakeCompound(ref aSC);
                    bool bSCEmpty = true, bSCChanged = false;
                    //
                    TopoDS_Iterator aItSC = new(aS);
                    for (; aItSC.More(); aItSC.Next())
                    {
                        TopoDS_Shape aSS = aItSC.Value();
                        if (!myShapeMaterial.IsBound(aSS))
                        {
                            aBB.Add(aSC, aSS);
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
                        TopoDS_Shape aRes = aResult;
                        MakeTypedContainers(aSC, ref aRes);
                    }
                    else
                    {
                        aBB.Add(aResult, aS);
                    }
                }
            }
            //
            myShape = aResult;
            //
            PrepareHistory(new ProgressRange());
        }
    }

    /// <summary>Finds the parts of the arguments to take (or remove).</summary>
    protected void FindParts(NCollection_List<TopoDS_Shape> theLSToTake,
                             NCollection_List<TopoDS_Shape> theLSToAvoid,
                             NCollection_List<TopoDS_Shape> theParts)
    {
        if (theLSToTake.IsEmpty())
        {
            return;
        }
        //
        // map shapes to avoid
        NCollection_Map<TopoDS_Shape> aMSToAvoid = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItArgs = new(theLSToAvoid);
        for (; aItArgs.More(); aItArgs.Next())
        {
            TopoDS_Shape aS = aItArgs.Value();
            aMSToAvoid.Add(aS);
        }
        //
        // map shapes to be taken
        NCollection_Map<TopoDS_Shape> aMSToTake = new(TopTools_ShapeMapHasher.Instance);
        aItArgs.Initialize(theLSToTake);
        for (; aItArgs.More(); aItArgs.Next())
        {
            TopoDS_Shape aS = aItArgs.Value();
            aMSToTake.Add(aS);
        }
        //
        int aNbS = aMSToTake.Extent();
        //
        // among the shapes to be taken into result, find any one
        // of minimal dimension
        int iDimMin = 10;
        TopoDS_Shape aSMin = new();
        //
        aItArgs.Initialize(theLSToTake);
        for (; aItArgs.More(); aItArgs.Next())
        {
            TopoDS_Shape aS = aItArgs.Value();
            int iDim = BOPTools_AlgoTools.Dimension(aS);
            if (iDim < iDimMin)
            {
                iDimMin = iDim;
                aSMin = aS;
            }
        }
        //
        // among the split parts of the shape of minimal dimension
        // look for the parts to be taken into result
        TopAbs_ShapeEnum aType = TypeToExplore(iDimMin);
        TopExp_Explorer aExp = new(aSMin, aType);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aST = aExp.Current();
            // get split parts of the shape
            NCollection_List<TopoDS_Shape> aLSTIm = new();
            if (!myImages.IsBound(aST))
            {
                aLSTIm.Append(aST);
            }
            else
            {
                aLSTIm.Assign(myImages.Find(aST));
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItIm = new(aLSTIm);
            for (; aItIm.More(); aItIm.Next())
            {
                TopoDS_Shape aPart = aItIm.Value();
                //
                if (!myIndex.Contains(aPart))
                {
                    continue;
                }
                //
                // get input shapes in which the split part is contained
                NCollection_List<TopoDS_Shape> aLS = myIndex.FindFromKey(aPart);
                if (aLS.Extent() < aNbS)
                {
                    continue;
                }
                //
                // check that input shapes containing the part should not be avoided
                NCollection_Map<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
                aItArgs.Initialize(aLS);
                for (; aItArgs.More(); aItArgs.Next())
                {
                    TopoDS_Shape aS = aItArgs.Value();
                    aMS.Add(aS);
                    if (aMSToAvoid.Contains(aS))
                    {
                        break;
                    }
                }
                //
                if (aItArgs.More())
                {
                    continue;
                }
                //
                // check that all shapes which should be taken contain the part
                aItArgs.Initialize(theLSToTake);
                for (; aItArgs.More(); aItArgs.Next())
                {
                    if (!aMS.Contains(aItArgs.Value()))
                    {
                        break;
                    }
                }
                //
                if (!aItArgs.More())
                {
                    theParts.Append(aPart);
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
        BRep_Builder aBB = new();
        TopoDS_Compound aResult = new();
        aBB.MakeCompound(ref aResult);
        //
        // basic elements of type EDGE, FACE and SOLID added into result
        NCollection_List<TopoDS_Shape>[] aLS = [new(), new(), new()];
        //
        TopoDS_Iterator aIt = new(myShape);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            //
            int iDim = BOPTools_AlgoTools.Dimension(aS);
            if (iDim <= 0)
            {
                aBB.Add(aResult, aS);
                continue;
            }
            //
            aLS[iDim - 1].Append(aS);
        }
        //
        for (int i = 0; i < 3; ++i)
        {
            if (aLS[i].IsEmpty())
            {
                continue;
            }
            //
            TopoDS_Compound aC = new();
            aBB.MakeCompound(ref aC);
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLS[i]);
            for (; aItLS.More(); aItLS.Next())
            {
                aBB.Add(aC, aItLS.Value());
            }
            //
            TopoDS_Shape aRes = aResult;
            MakeTypedContainers(aC, ref aRes);
        }
        myShape = aResult;
    }

    /// <summary>Removes the internal boundaries between the shapes of the list (unifies them).</summary>
    protected bool RemoveInternals(NCollection_List<TopoDS_Shape> theLS,
                                   NCollection_List<TopoDS_Shape> theLSNew,
                                   NCollection_Map<TopoDS_Shape>? theMapKeepBnd = null)
    {
        bool bRemoved = false;
        if (theLS.Extent() < 2)
        {
            theLSNew.Assign(theLS);
            return bRemoved;
        }
        //
        TopAbs_ShapeEnum aType = theLS.First().ShapeType();
        //
        if (aType == TopAbs_EDGE || aType == TopAbs_FACE)
        {
            //
            // make container
            BRep_Builder aBB = new();
            TopoDS_Shape aShape = new();
            //
            BOPTools_AlgoTools.MakeContainer((aType == TopAbs_FACE) ? TopAbs_SHELL : TopAbs_WIRE, ref aShape);
            //
            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLS); aIt.More(); aIt.Next())
            {
                TopoDS_Shape aS = aIt.Value();
                aBB.Add(aShape, aS);
            }
            //
            // Unify same domain
            bool bFaces, bEdges;
            //
            bFaces = (aType == TopAbs_FACE);
            bEdges = (aType == TopAbs_EDGE);
            ShapeUpgrade_UnifySameDomain anUnify = new(aShape, bEdges, bFaces);
            anUnify.KeepShapes(theMapKeepBnd ?? new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance));
            anUnify.Build();
            TopoDS_Shape aSNew = anUnify.InternalShape();
            //
            TopExp_Explorer aExp = new(aSNew, aType);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aSn = aExp.Current();
                theLSNew.Append(aSn);
            }
            //
            if (theLSNew.IsEmpty())
            {
                // add the warning
                if (bFaces)
                {
                    AddWarning(new BOPAlgo_AlertRemovalOfIBForFacesFailed(aShape));
                }
                else
                {
                    AddWarning(new BOPAlgo_AlertRemovalOfIBForEdgesFailed(aShape));
                }
                //
                theLSNew.Assign(theLS);
                return bRemoved;
            }
            //
            // fill map of modified shapes
            NCollection_IndexedMap<TopoDS_Shape> aMG = new(TopTools_ShapeMapHasher.Instance);
            int i, aNb;
            //
            TopExp.MapShapes(aShape, TopAbs_VERTEX, aMG);
            TopExp.MapShapes(aShape, TopAbs_EDGE, aMG);
            TopExp.MapShapes(aShape, TopAbs_FACE, aMG);
            //
            BRepTools_History aHistory = anUnify.InternalHistory()!;
            aNb = aMG.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                TopoDS_Shape aSS = aMG[i];
                bool hasMaterial = myShapeMaterial.Find(aSS, out int pMaterial);
                foreach (TopoDS_Shape aSU in aHistory.Modified(aSS))
                {
                    myMapModified.Bind(aSS, aSU);
                    bRemoved = true;
                    if (hasMaterial && !myShapeMaterial.IsBound(aSU))
                    {
                        myShapeMaterial.Bind(aSU, pMaterial);
                    }
                }
            }
        }
        else if (aType == TopAbs_SOLID)
        {
            BRep_Builder aBB = new();
            TopoDS_Compound aSolids = new();
            aBB.MakeCompound(ref aSolids);
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(theLS);
            for (; aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aSol = aItLS.Value();
                aBB.Add(aSolids, aSol);
            }
            //
            // Make connexity blocks of solids to create from each isolated block one solid.
            // It will allow attaching internal entities of the solids to new solid.
            NCollection_List<TopoDS_Shape> aLCB = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aSolids, TopAbs_FACE, TopAbs_SOLID, aLCB);
            //
            // for each block remove internal faces
            NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCB);
            for (; aItLCB.More(); aItLCB.Next())
            {
                TopoDS_Shape aCB = aItLCB.Value();
                //
                // Map faces and solids to find boundary faces that can be removed
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMFS = new(TopTools_ShapeMapHasher.Instance);
                // internal entities
                NCollection_List<TopoDS_Shape> aLSInt = new();
                //
                TopoDS_Iterator aItS = new(aCB);
                for (; aItS.More(); aItS.Next())
                {
                    TopoDS_Shape aSol = aItS.Value();
                    //
                    TopoDS_Iterator aItIS = new(aSol);
                    for (; aItIS.More(); aItIS.Next())
                    {
                        TopoDS_Shape aSI = aItIS.Value();
                        if (aSI.Orientation() == TopAbs_INTERNAL)
                        {
                            aLSInt.Append(aSI);
                        }
                        else
                        {
                            TopoDS_Iterator aItF = new(aSI);
                            for (; aItF.More(); aItF.Next())
                            {
                                TopoDS_Shape aF = aItF.Value();
                                if (!aDMFS.FindFromKey(aF, out NCollection_List<TopoDS_Shape> pLSols))
                                {
                                    pLSols = new NCollection_List<TopoDS_Shape>();
                                    aDMFS.Add(aF, pLSols);
                                }
                                pLSols.Append(aSol);
                            }
                        }
                    }
                }
                //
                // to build unified solid, select only faces attached to only one solid
                NCollection_List<TopoDS_Shape> aLFUnique = new();
                int i, aNb = aDMFS.Extent();
                for (i = 1; i <= aNb; ++i)
                {
                    if (aDMFS[i].Extent() == 1)
                    {
                        aLFUnique.Append(aDMFS.FindKey(i));
                    }
                }
                //
                if (aNb == aLFUnique.Extent())
                {
                    // no faces to remove
                    aItS.Initialize(aCB);
                    for (; aItS.More(); aItS.Next())
                    {
                        theLSNew.Append(aItS.Value());
                    }
                    continue;
                }
                //
                // build new solid
                BOPAlgo_BuilderSolid aBS = new();
                aBS.SetShapes(aLFUnique);
                aBS.Perform();
                //
                if (aBS.HasErrors() || aBS.Areas().Extent() != 1)
                {
                    // add the warning
                    {
                        TopoDS_Compound aUniqeFaces = new();
                        aBB.MakeCompound(ref aUniqeFaces);
                        NCollection_List<TopoDS_Shape>.Iterator aItLFUniqe = new(aLFUnique);
                        for (; aItLFUniqe.More(); aItLFUniqe.Next())
                        {
                            aBB.Add(aUniqeFaces, aItLFUniqe.Value());
                        }
                        //
                        AddWarning(new BOPAlgo_AlertRemovalOfIBForSolidsFailed(aUniqeFaces));
                    }
                    //
                    aItS.Initialize(aCB);
                    for (; aItS.More(); aItS.Next())
                    {
                        theLSNew.Append(aItS.Value());
                    }
                    continue;
                }
                //
                myReport.Merge(aBS.GetReport());
                //
                TopoDS_Shape aSNew = aBS.Areas().First();
                //
                // put all internal parts into new solid
                aSNew.Free(true);
                NCollection_List<TopoDS_Shape>.Iterator aItLSI = new(aLSInt);
                for (; aItLSI.More(); aItLSI.Next())
                {
                    aBB.Add(aSNew, aItLSI.Value());
                }
                aSNew.Free(false);
                //
                theLSNew.Append(aSNew);
                bRemoved = true;

                // Save information about the fuse of the solids into a history map
                aItS.Initialize(aCB);
                for (; aItS.More(); aItS.Next())
                {
                    myMapModified.Bind(aItS.Value(), aSNew);
                }
            }
        }
        return bRemoved;
    }

    /// <summary>The splits of the shape, with the local unification of the splits taken into account.</summary>
    protected override NCollection_List<TopoDS_Shape>? LocModified(in TopoDS_Shape theS)
    {
        // Get shape's modification coming from GF operation
        NCollection_List<TopoDS_Shape>? pLSp = base.LocModified(theS);
        if (myMapModified.IsEmpty())
        {
            // No local modifications
            return pLSp;
        }

        myHistShapes.Clear();

        // Check if the shape (or its splits) has participated in unification
        if (pLSp is null)
        {
            // No splits from GF operation.
            // Check if the shape has been unified with other shapes
            if (!myMapModified.Find(theS, out TopoDS_Shape pSU))
            {
                return null;
            }

            myHistShapes.Append(pSU);
        }
        else
        {
            NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
            // Process all GF splits and check them for local unification with other shapes
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(pLSp);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape pSp = aIt.Value();
                if (myMapModified.Find(pSp, out TopoDS_Shape pSU))
                {
                    pSp = pSU;
                }
                if (aMFence.Add(pSp))
                {
                    myHistShapes.Append(pSp);
                }
            }
        }
        return myHistShapes;
    }

    // Makes the typed containers (wires, shells, compsolids) of the connexity blocks of the parts and adds them to
    // theResult (OCCT's file-static MakeTypedContainers).
    private static void MakeTypedContainers(in TopoDS_Shape theSC, ref TopoDS_Shape theResult)
    {
        TopAbs_ShapeEnum aContainerType, aConnexityType, aPartType;
        //
        aPartType = TypeToExplore(BOPTools_AlgoTools.Dimension(theSC));
        switch (aPartType)
        {
            case TopAbs_EDGE:
            {
                aContainerType = TopAbs_WIRE;
                aConnexityType = TopAbs_VERTEX;
                break;
            }
            case TopAbs_FACE:
            {
                aContainerType = TopAbs_SHELL;
                aConnexityType = TopAbs_EDGE;
                break;
            }
            case TopAbs_SOLID:
            {
                aContainerType = TopAbs_COMPSOLID;
                aConnexityType = TopAbs_FACE;
                break;
            }
            default:
                return;
        }
        //
        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(theSC, aConnexityType, aPartType, aLCB);
        if (aLCB.IsEmpty())
        {
            return;
        }
        //
        BRep_Builder aBB = new();
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape>.Iterator aItCB = new();
        //
        aItCB.Initialize(aLCB);
        for (; aItCB.More(); aItCB.Next())
        {
            TopoDS_Shape aRCB = new();
            BOPTools_AlgoTools.MakeContainer(aContainerType, ref aRCB);
            //
            TopoDS_Shape aCB = aItCB.Value();
            aExp.Init(aCB, aPartType);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aCBS = aExp.Current();
                aBB.Add(aRCB, aCBS);
            }
            //
            if (aContainerType == TopAbs_SHELL)
            {
                BOPTools_AlgoTools.OrientFacesOnShell(ref aRCB);
            }
            //
            aBB.Add(theResult, aRCB);
        }
    }

    // Adds to theMapKeepBnd the boundary shapes of the area defined by shapes from the list.
    private static void CollectMaterialBoundaries(NCollection_List<TopoDS_Shape> theLS, NCollection_Map<TopoDS_Shape> theMapKeepBnd)
    {
        TopAbs_ShapeEnum aType = theLS.First().ShapeType();
        TopAbs_ShapeEnum aTypeSubsh = (aType == TopAbs_FACE ? TopAbs_EDGE : TopAbs_VERTEX);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapSubSh = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator anIt = new(theLS);
        for (; anIt.More(); anIt.Next())
        {
            TopoDS_Shape aS = anIt.Value();
            TopExp.MapShapesAndAncestors(aS, aTypeSubsh, aType, aMapSubSh);
        }
        for (int i = 1; i <= aMapSubSh.Extent(); i++)
        {
            // check if the subshape belongs to boundary of the area
            if (aMapSubSh[i].Extent() == 1)
            {
                // add to theMapKeepBnd
                theMapKeepBnd.Add(aMapSubSh.FindKey(i));
            }
        }
    }

    private static TopAbs_ShapeEnum TypeToExplore(int theDim)
    {
        TopAbs_ShapeEnum aRet;
        //
        switch (theDim)
        {
            case 0:
                aRet = TopAbs_VERTEX;
                break;
            case 1:
                aRet = TopAbs_EDGE;
                break;
            case 2:
                aRet = TopAbs_FACE;
                break;
            case 3:
                aRet = TopAbs_SOLID;
                break;
            default:
                aRet = TopAbs_SHAPE;
                break;
        }
        return aRet;
    }
}
