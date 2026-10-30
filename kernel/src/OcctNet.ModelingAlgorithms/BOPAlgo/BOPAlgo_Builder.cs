// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_Operation;
using static Occt.Message_Gravity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The General Fuse algorithm: builds the split parts of the arguments from the intersections computed by a pave
/// filler, and the history of the operation.
/// </summary>
/// <remarks>
/// <para>
/// The result is a compound of the split arguments, of the same dimension as the arguments, sharing their common
/// parts (same-domain vertices, edges, faces and solids are represented by one shape). The images map
/// (<see cref="Images"/>) gives the splits of every argument sub-shape, <see cref="Origins"/> the reverse, and
/// <see cref="ShapesSD"/> the same-domain shapes.
/// </para>
/// <para>
/// The class is partial over OCCT's files: this one, <c>_1</c> (vertices, edges, containers, compounds), <c>_2</c>
/// (faces), <c>_3</c> (solids) and <c>_4</c> (history). OCCT's builder deletes the pave filler it made itself
/// (<c>myEntryPoint == 1</c>); the port lets it go.
/// </para>
/// </remarks>
public partial class BOPAlgo_Builder : BOPAlgo_BuilderShape
{
    protected NCollection_List<TopoDS_Shape> myArguments = new();                                                     //!< Arguments of the operation
    protected NCollection_Map<TopoDS_Shape> myMapFence = new(TopTools_ShapeMapHasher.Instance);                         //!< Fence map providing the uniqueness of the shapes in the list of arguments
    protected BOPAlgo_PaveFiller? myPaveFiller;                                                                         //!< Pave Filler - algorithm for sub-shapes intersection
    protected BOPDS_DS? myDS;                                                                                           //!< Data Structure - holder of intersection information
    protected IntTools_Context? myContext;                                                                              //!< Context - tool for cashing heavy algorithms such as Projectors and Classifiers
    protected int myEntryPoint;                                                                                         //!< EntryPoint - controls the deletion of the PaveFiller, which could live longer than the Builder
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myImages = new(TopTools_ShapeMapHasher.Instance);  //!< Images - map of Images of the sub-shapes of arguments
    protected NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myShapesSD = new(TopTools_ShapeMapHasher.Instance);                  //!< ShapesSD - map of SD Shapes
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myOrigins = new(TopTools_ShapeMapHasher.Instance); //!< Origins - map of Origins, back map of Images
    protected NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myInParts = new(TopTools_ShapeMapHasher.Instance); //!< InParts - map of own and acquired IN faces of the arguments solids
    protected bool myNonDestructive;                                                                                    //!< Safe processing option allows avoiding modification of the input shapes
    protected BOPAlgo_GlueEnum myGlue;                                                                                  //!< Gluing option allows speeding up the intersection of the input shapes
    protected bool myCheckInverted;                                                                                     //!< Check inverted option allows disabling the check of input solids on inverted status

    public BOPAlgo_Builder()
    {
        myPaveFiller = null;
        myDS = null;
        myEntryPoint = 0;
        myNonDestructive = false;
        myGlue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
        myCheckInverted = true;
    }

    /// <summary>Clears the content of the algorithm.</summary>
    public override void Clear()
    {
        base.Clear();
        myArguments.Clear();
        myMapFence.Clear();
        myImages.Clear();
        myShapesSD.Clear();
        myOrigins.Clear();
        myInParts.Clear();
    }

    /// <summary>The pave filler used.</summary>
    internal BOPAlgo_PaveFiller? PPaveFiller() => myPaveFiller;

    /// <summary>The data structure used.</summary>
    internal BOPDS_DS? PDS() => myDS;

    /// <summary>The context used.</summary>
    internal IntTools_Context? Context() => myContext;

    //! @name Arguments

    /// <summary>Adds an argument (once).</summary>
    /// <remarks>
    /// As in OCCT, the fence of added arguments is cleared only by <see cref="Clear"/>, not by
    /// <see cref="SetArguments"/>: setting again arguments added before leaves them out (occt-issues.md, unit 6A).
    /// </remarks>
    public virtual void AddArgument(in TopoDS_Shape theShape)
    {
        if (myMapFence.Add(theShape))
        {
            myArguments.Append(theShape);
        }
    }

    /// <summary>Sets the arguments.</summary>
    public virtual void SetArguments(NCollection_List<TopoDS_Shape> theShapes)
    {
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        myArguments.Clear();
        // OCCT: myMapFence isn't cleared (occt-issues.md, unit 6A)
        //
        aIt.Initialize(theShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            AddArgument(aS);
        }
    }

    /// <summary>The arguments.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    //! @name Options

    /// <summary>Sets the flag that defines the mode of treatment: in non-destructive mode the arguments are not modified.</summary>
    public void SetNonDestructive(bool theFlag) => myNonDestructive = theFlag;

    /// <summary>The flag defining the mode of treatment.</summary>
    public bool NonDestructive() => myNonDestructive;

    /// <summary>Sets the glue option.</summary>
    public void SetGlue(BOPAlgo_GlueEnum theGlue) => myGlue = theGlue;

    /// <summary>The glue option.</summary>
    public BOPAlgo_GlueEnum Glue() => myGlue;

    /// <summary>Enables or disables the check of the input solids for being inverted.</summary>
    public void SetCheckInverted(bool theCheck) => myCheckInverted = theCheck;

    /// <summary>The flag of the check of the input solids for being inverted.</summary>
    public bool CheckInverted() => myCheckInverted;

    //! @name Performing the operation

    /// <summary>Intersects the arguments with a new pave filler and builds the result.</summary>
    public override void Perform(Message_ProgressRange theRange = default)
    {
        GetReport().Clear();
        //
        if (myEntryPoint == 1)
        {
            myPaveFiller = null;
        }
        //
        BOPAlgo_PaveFiller pPF = new();
        //
        pPF.SetArguments(myArguments);
        pPF.SetRunParallel(myRunParallel);
        using var aPS = new Message_ProgressScope(theRange, "Performing General Fuse operation", 10);
        pPF.SetFuzzyValue(myFuzzyValue);
        pPF.SetNonDestructive(myNonDestructive);
        pPF.SetGlue(myGlue);
        pPF.SetUseOBB(myUseOBB);
        //
        pPF.Perform(aPS.Next(9));
        //
        myEntryPoint = 1;
        PerformInternal(pPF, aPS.Next(1));
    }

    /// <summary>Builds the result from the intersections of <paramref name="theFiller"/>.</summary>
    public virtual void PerformWithFiller(BOPAlgo_PaveFiller theFiller, Message_ProgressRange theRange = default)
    {
        GetReport().Clear();
        myEntryPoint = 0;
        myNonDestructive = theFiller.NonDestructive();
        myFuzzyValue = theFiller.FuzzyValue();
        myGlue = theFiller.Glue();
        myUseOBB = theFiller.UseOBB();
        PerformInternal(theFiller, theRange);
    }

    //! @name BOPs on open solids

    /// <summary>
    /// Builds the result of a Boolean operation from the splits of the given solids (or compounds of solids), each
    /// group taken in the given state relatively the other (IN or OUT), as the General Fuse built them.
    /// </summary>
    /// <remarks>
    /// Works for open solids too, and keeps the internal parts of the solids where the operation keeps IN faces.
    /// Errors go to <paramref name="theReport"/> (the algorithm's own report when null).
    /// </remarks>
    public virtual void BuildBOP(NCollection_List<TopoDS_Shape> theObjects, TopAbs_State theObjState,
                                 NCollection_List<TopoDS_Shape> theTools, TopAbs_State theToolsState,
                                 Message_ProgressRange theRange, Message_Report? theReport = null)
    {
        if (HasErrors())
        {
            return;
        }

        // Report for the method
        Message_Report aReport = theReport is null ? myReport : theReport;

        if (myArguments.IsEmpty() || myShape.IsNull())
        {
            aReport.AddAlert(Message_Fail, new BOPAlgo_AlertBuilderFailed());
            return;
        }
        // Check the input data
        if ((theObjState != TopAbs_IN && theObjState != TopAbs_OUT)
            || (theToolsState != TopAbs_IN && theToolsState != TopAbs_OUT))
        {
            aReport.AddAlert(Message_Fail, new BOPAlgo_AlertBOPNotSet());
            return;
        }

        // Check input shapes
        bool hasObjects = !theObjects.IsEmpty();
        bool hasTools = !theTools.IsEmpty();
        if (!hasObjects && !hasTools)
        {
            aReport.AddAlert(Message_Fail, new BOPAlgo_AlertTooFewArguments());
            return;
        }

        // Check that all input solids are from the arguments
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aList = i == 0 ? theObjects : theTools;
            NCollection_List<TopoDS_Shape>.Iterator itLS = new(aList);
            for (; itLS.More(); itLS.Next())
            {
                TopoDS_Shape aS = itLS.Value();
                // Check if the shape belongs to the arguments of operation
                if (myDS!.Index(aS) < 0)
                {
                    aReport.AddAlert(Message_Fail, new BOPAlgo_AlertUnknownShape(aS));
                    return;
                }

                // Check if the shape is a solid or collection of them
                if (aS.ShapeType() != TopAbs_SOLID)
                {
                    NCollection_List<TopoDS_Shape> aLS = new();
                    NCollection_Map<TopoDS_Shape> aMFence0 = new(TopTools_ShapeMapHasher.Instance);
                    BOPTools_AlgoTools.TreatCompound(aS, aLS, aMFence0);

                    NCollection_List<TopoDS_Shape>.Iterator it = new(aLS);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Shape aSx = it.Value();
                        if (aSx.ShapeType() != TopAbs_SOLID && aSx.ShapeType() != TopAbs_COMPSOLID)
                        {
                            aReport.AddAlert(Message_Fail, new BOPAlgo_AlertUnsupportedType(aS));
                            return;
                        }
                    }
                }
            }
        }

        // Classification of the faces relatively solids has been made
        // on the stage of Solids splitting. All results are saved into
        // myInParts map, which connects the solids with its IN faces from
        // other arguments. All faces not contained in the list of IN faces
        // will be considered as OUT.

        // Prepare the maps of splits of solids faces with orientations (OCCT's default hasher: orientation counts)
        NCollection_IndexedMap<TopoDS_Shape> aMObjFacesOri = new(EqualityComparer<TopoDS_Shape>.Default),
                                             aMToolFacesOri = new(EqualityComparer<TopoDS_Shape>.Default);
        // Prepare the maps of splits of solids faces
        NCollection_IndexedMap<TopoDS_Shape> aMObjFaces = new(TopTools_ShapeMapHasher.Instance),
                                             aMToolFaces = new(TopTools_ShapeMapHasher.Instance);
        // Copy the list of IN faces of the solids into map
        NCollection_Map<TopoDS_Shape> anINObjects = new(TopTools_ShapeMapHasher.Instance),
                                      anINTools = new(TopTools_ShapeMapHasher.Instance);

        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<TopoDS_Shape> aList = i == 0 ? theObjects : theTools;
            NCollection_IndexedMap<TopoDS_Shape> aMapOri = i == 0 ? aMObjFacesOri : aMToolFacesOri;
            NCollection_IndexedMap<TopoDS_Shape> aMap = i == 0 ? aMObjFaces : aMToolFaces;
            NCollection_List<TopoDS_Shape>.Iterator itLS = new(aList);
            for (; itLS.More(); itLS.Next())
            {
                TopoDS_Shape aShape = itLS.Value();
                TopExp_Explorer expS = new(aShape, TopAbs_SOLID);
                for (; expS.More(); expS.Next())
                {
                    TopoDS_Shape aS = expS.Current();
                    TopExp_Explorer expF = new(aS, TopAbs_FACE);
                    for (; expF.More(); expF.Next())
                    {
                        TopoDS_Shape aF = expF.Current();
                        if (aF.Orientation() != TopAbs_FORWARD && aF.Orientation() != TopAbs_REVERSED)
                        {
                            continue;
                        }
                        if (myImages.Find(aF, out NCollection_List<TopoDS_Shape> pLFIm))
                        {
                            NCollection_List<TopoDS_Shape>.Iterator itLFIm = new(pLFIm);
                            for (; itLFIm.More(); itLFIm.Next())
                            {
                                TopoDS_Face aFImRef = TopoDS.Face(itLFIm.Value());
                                if (BOPTools_AlgoTools.IsSplitToReverse(aFImRef, aF, myContext))
                                {
                                    TopoDS_Face aFIm = aFImRef;
                                    aFIm.Reverse();
                                    aMapOri.Add(aFIm);
                                    aMap.Add(aFIm);
                                }
                                else
                                {
                                    aMapOri.Add(aFImRef);
                                    aMap.Add(aFImRef);
                                }
                            }
                        }
                        else
                        {
                            aMapOri.Add(aF);
                            aMap.Add(aF);
                        }
                    }

                    // Copy the list of IN faces into a map
                    if (myInParts.Find(aS, out NCollection_List<TopoDS_Shape> pLFIN))
                    {
                        NCollection_Map<TopoDS_Shape> anINMap = i == 0 ? anINObjects : anINTools;
                        NCollection_List<TopoDS_Shape>.Iterator itLFIn = new(pLFIN);
                        for (; itLFIn.More(); itLFIn.Next())
                        {
                            anINMap.Add(itLFIn.Value());
                        }
                    }
                }
            }
        }

        // Now we need to select all faces which will participate in
        // building of the resulting solids. The final set of faces
        // depends on the given states for the groups.
        bool isObjectsIN = theObjState == TopAbs_IN, isToolsIN = theToolsState == TopAbs_IN;

        // Shortcuts
        bool bAvoidIN = !isObjectsIN && !isToolsIN,  // avoid all in faces
             bAvoidINforBoth = isObjectsIN != isToolsIN; // avoid faces IN for both groups

        // Choose which SD faces are needed to be taken - equally or differently oriented faces
        bool isSameOriNeeded = theObjState == theToolsState;
        // Resulting faces
        NCollection_IndexedMap<TopoDS_Shape> aMResFacesOri = new(EqualityComparer<TopoDS_Shape>.Default);
        NCollection_Map<TopoDS_Shape> aMResFacesFence = new(TopTools_ShapeMapHasher.Instance);
        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance),
                                      aMFToAvoid = new(TopTools_ShapeMapHasher.Instance);
        // Oriented fence map
        NCollection_Map<TopoDS_Shape> aMFenceOri = new(EqualityComparer<TopoDS_Shape>.Default);

        for (int i = 0; i < 2; ++i)
        {
            NCollection_IndexedMap<TopoDS_Shape> aMap = i == 0 ? aMObjFacesOri : aMToolFacesOri;
            NCollection_IndexedMap<TopoDS_Shape> anOppositeMap = i == 0 ? aMToolFaces : aMObjFaces;
            NCollection_Map<TopoDS_Shape> anINMap = i == 0 ? anINObjects : anINTools;
            NCollection_Map<TopoDS_Shape> anOppositeINMap = i == 0 ? anINTools : anINObjects;
            bool bTakeIN = i == 0 ? isObjectsIN : isToolsIN;

            int aNbF = aMap.Extent();
            for (int j = 1; j <= aNbF; ++j)
            {
                TopoDS_Shape aFIm = aMap[j];

                bool isIN = anINMap.Contains(aFIm);
                bool isINOpposite = anOppositeINMap.Contains(aFIm);

                // Filtering for FUSE - avoid any IN faces
                if (bAvoidIN && (isIN || isINOpposite))
                {
                    continue;
                }

                // Filtering for CUT - avoid faces IN for both groups
                if (bAvoidINforBoth && isIN && isINOpposite)
                {
                    continue;
                }

                // Treatment of SD faces
                if (!aMFence.Add(aFIm))
                {
                    if (!anOppositeMap.Contains(aFIm))
                    {
                        // The face belongs to only one group
                        if (bTakeIN != isSameOriNeeded)
                        {
                            aMFToAvoid.Add(aFIm);
                        }
                    }
                    else
                    {
                        // The face belongs to both groups.
                        // Using its orientation decide if it is needed in the result or not.
                        bool isSameOri = !aMFenceOri.Add(aFIm);
                        if (isSameOriNeeded == isSameOri)
                        {
                            // Take the shape without classification
                            if (aMResFacesFence.Add(aFIm))
                            {
                                aMResFacesOri.Add(aFIm);
                            }
                        }
                        else
                        {
                            // Remove the face
                            aMFToAvoid.Add(aFIm);
                        }

                        continue;
                    }
                }
                if (!aMFenceOri.Add(aFIm))
                {
                    continue;
                }

                if (bTakeIN == isINOpposite)
                {
                    if (isIN)
                    {
                        aMResFacesOri.Add(aFIm);
                        aMResFacesOri.Add(aFIm.Reversed());
                    }
                    else if (bTakeIN && !isSameOriNeeded)
                    {
                        aMResFacesOri.Add(aFIm.Reversed());
                    }
                    else
                    {
                        aMResFacesOri.Add(aFIm);
                    }
                    aMResFacesFence.Add(aFIm);
                }
            }
        }

        // Remove the faces which has to be avoided
        NCollection_List<TopoDS_Shape> aResFaces = new();
        int aNbRF = aMResFacesOri.Extent();
        for (int i = 1; i <= aNbRF; ++i)
        {
            TopoDS_Shape aRF = aMResFacesOri[i];
            if (!aMFToAvoid.Contains(aRF))
            {
                aResFaces.Append(aRF);
            }
        }
        using var aPS = new Message_ProgressScope(theRange, null, 2);
        BRep_Builder aBB = new();

        // Try to build closed solids from the faces
        BOPAlgo_BuilderSolid aBS = new();
        aBS.SetShapes(aResFaces);
        aBS.SetRunParallel(myRunParallel);
        aBS.SetContext(myContext);
        aBS.SetFuzzyValue(myFuzzyValue);
        aBS.Perform(aPS.Next());

        // Resulting solids
        NCollection_List<TopoDS_Shape> aResSolids = new();

        aMFence.Clear();
        if (!aBS.HasErrors())
        {
            // If any, add solids into resulting compound
            NCollection_List<TopoDS_Shape>.Iterator itA = new(aBS.Areas());
            for (; itA.More(); itA.Next())
            {
                TopoDS_Shape aSolid = itA.Value();
                // The solid must contain at least one face
                // from either of objects or tools
                TopExp_Explorer expF = new(aSolid, TopAbs_FACE);
                for (; expF.More(); expF.Next())
                {
                    TopoDS_Shape aF = expF.Current();
                    if (aMObjFacesOri.Contains(aF) || aMToolFacesOri.Contains(aF))
                    {
                        break;
                    }
                }
                if (expF.More())
                {
                    aResSolids.Append(aSolid);
                    TopExp.MapShapes(aSolid, aMFence);
                }
            }
        }
        else
        {
            return;
        }

        // Collect unused faces
        TopoDS_Compound anUnUsedFaces = new();
        aBB.MakeCompound(ref anUnUsedFaces);

        NCollection_List<TopoDS_Shape>.Iterator itLF = new(aResFaces);
        for (; itLF.More(); itLF.Next())
        {
            if (aMFence.Add(itLF.Value()))
            {
                aBB.Add(anUnUsedFaces, itLF.Value());
            }
        }

        // Build blocks from the unused faces
        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(anUnUsedFaces, TopAbs_EDGE, TopAbs_FACE, aLCB);

        // Build solid from each block
        NCollection_List<TopoDS_Shape>.Iterator itCB = new(aLCB);
        for (; itCB.More(); itCB.Next())
        {
            TopoDS_Shape aCB = itCB.Value();
            TopoDS_Shell aShell0 = new();
            aBB.MakeShell(ref aShell0);
            TopoDS_Shape aShell = aShell0;
            // Add faces of the block to the shell
            TopExp_Explorer anExpF = new(aCB, TopAbs_FACE);
            for (; anExpF.More(); anExpF.Next())
            {
                aBB.Add(aShell, anExpF.Current());
            }

            BOPTools_AlgoTools.OrientFacesOnShell(ref aShell);
            // Make solid out of the shell
            TopoDS_Solid aSolid = new();
            aBB.MakeSolid(ref aSolid);
            aBB.Add(aSolid, aShell);
            // Add new solid to result
            aResSolids.Append(aSolid);
        }

        if (!bAvoidIN)
        {
            // Fill solids with internal parts coming with the solids
            NCollection_List<TopoDS_Shape> anInParts = new();
            for (int i = 0; i < 2; ++i)
            {
                NCollection_List<TopoDS_Shape> aList = i == 0 ? theObjects : theTools;
                NCollection_List<TopoDS_Shape>.Iterator itLS = new(aList);
                for (; itLS.More(); itLS.Next())
                {
                    TopExp_Explorer expS = new(itLS.Value(), TopAbs_SOLID);
                    for (; expS.More(); expS.Next())
                    {
                        TopoDS_Shape aS = expS.Current(); // Solid
                        for (TopoDS_Iterator it = new(aS); it.More(); it.Next())
                        {
                            TopoDS_Shape aSInt = it.Value();
                            if (aSInt.Orientation() == TopAbs_INTERNAL)
                            {
                                anInParts.Append(aSInt); // vertex or edge
                            }
                            else
                            {
                                // shell treatment
                                TopoDS_Iterator itInt = new(aSInt);
                                if (itInt.More() && itInt.Value().Orientation() == TopAbs_INTERNAL)
                                {
                                    anInParts.Append(aSInt);
                                }
                            }
                        }
                    }
                }
            }

            BOPAlgo_Tools.FillInternals(aResSolids, anInParts, myImages, myContext!);
        }

        // Combine solids into compound
        TopoDS_Compound aResult0 = new();
        aBB.MakeCompound(ref aResult0);
        TopoDS_Shape aResult = aResult0;

        NCollection_List<TopoDS_Shape>.Iterator itLS2 = new(aResSolids);
        for (; itLS2.More(); itLS2.Next())
        {
            aBB.Add(aResult, itLS2.Value());
        }

        myShape = aResult;
        PrepareHistory(aPS.Next());
    }

    /// <summary>
    /// Builds the result of a Boolean operation of the given type from the splits of the given solids (see the
    /// overload with states).
    /// </summary>
    public void BuildBOP(NCollection_List<TopoDS_Shape> theObjects, NCollection_List<TopoDS_Shape> theTools,
                         BOPAlgo_Operation theOperation, Message_ProgressRange theRange, Message_Report? theReport = null)
    {
        TopAbs_State anObjState, aToolsState;
        switch (theOperation)
        {
            case BOPAlgo_COMMON:
            {
                anObjState = TopAbs_IN;
                aToolsState = TopAbs_IN;
                break;
            }
            case BOPAlgo_FUSE:
            {
                anObjState = TopAbs_OUT;
                aToolsState = TopAbs_OUT;
                break;
            }
            case BOPAlgo_CUT:
            {
                anObjState = TopAbs_OUT;
                aToolsState = TopAbs_IN;
                break;
            }
            case BOPAlgo_CUT21:
            {
                anObjState = TopAbs_IN;
                aToolsState = TopAbs_OUT;
                break;
            }
            default:
            {
                anObjState = TopAbs_UNKNOWN;
                aToolsState = TopAbs_UNKNOWN;
                break;
            }
        }
        BuildBOP(theObjects, anObjState, theTools, aToolsState, theRange, theReport);
    }

    //! @name Images/Origins

    /// <summary>The map of images: the splits of the argument sub-shapes.</summary>
    public NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Images() => myImages;

    /// <summary>The map of origins: the argument sub-shapes each split comes from.</summary>
    public NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Origins() => myOrigins;

    /// <summary>The map of same-domain shapes: each shape to its representative.</summary>
    public NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> ShapesSD() => myShapesSD;

    //! @name Analyze progress of the operation

    protected enum BOPAlgo_PIOperation
    {
        PIOperation_TreatVertices = 0,
        PIOperation_TreatEdges,
        PIOperation_TreatWires,
        PIOperation_TreatFaces,
        PIOperation_TreatShells,
        PIOperation_TreatSolids,
        PIOperation_TreatCompsolids,
        PIOperation_TreatCompounds,
        PIOperation_FillHistory,
        PIOperation_PostTreat,
        PIOperation_Last,
    }

    /// <summary>The numbers of the shapes of each type the operation treats, for the progress steps.</summary>
    protected sealed class NbShapes
    {
        private readonly int[] myNbShapesArr = new int[8];

        public ref int NbVertices() => ref myNbShapesArr[0];

        public ref int NbEdges() => ref myNbShapesArr[1];

        public ref int NbWires() => ref myNbShapesArr[2];

        public ref int NbFaces() => ref myNbShapesArr[3];

        public ref int NbShells() => ref myNbShapesArr[4];

        public ref int NbSolids() => ref myNbShapesArr[5];

        public ref int NbCompsolids() => ref myNbShapesArr[6];

        public ref int NbCompounds() => ref myNbShapesArr[7];
    }

    /// <summary>Counts the shapes of each type the operation treats.</summary>
    protected NbShapes getNbShapes()
    {
        NbShapes aCounter = new();
        aCounter.NbVertices() = myDS!.ShapesSD().Extent();
        for (int i = 0; i < myDS.NbSourceShapes(); ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            switch (aSI.ShapeType())
            {
                case TopAbs_EDGE:
                {
                    if (myDS.HasPaveBlocks(i))
                    {
                        aCounter.NbEdges()++;
                    }
                    break;
                }
                case TopAbs_WIRE:
                    aCounter.NbWires()++;
                    break;
                case TopAbs_FACE:
                {
                    if (myDS.HasFaceInfo(i))
                    {
                        aCounter.NbFaces()++;
                    }
                    break;
                }
                case TopAbs_SHELL:
                    aCounter.NbShells()++;
                    break;
                case TopAbs_SOLID:
                    aCounter.NbSolids()++;
                    break;
                case TopAbs_COMPSOLID:
                    aCounter.NbCompsolids()++;
                    break;
                case TopAbs_COMPOUND:
                    aCounter.NbCompounds()++;
                    break;
                default:
                    break;
            }
        }
        return aCounter;
    }

    /// <summary>The steps of the constant operations.</summary>
    protected override void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
        // Fill in the constants:
        if (myFillHistory)
        {
            // for FillHistroty, which takes about 5% of the whole operation
            theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory, 0.05 * theWhole);
        }

        // and for PostTreat, which takes about 3% of the whole operation
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat, 0.03 * theWhole);
    }

    /// <summary>The steps of the operations depending on the numbers of shapes.</summary>
    protected override void fillPISteps(BOPAlgo_PISteps theSteps)
    {
        // Compute the rest of the operations - all depend on the number of sub-shapes of certain type
        NbShapes aNbShapes = getNbShapes();

        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices, aNbShapes.NbVertices());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges, aNbShapes.NbEdges());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatWires, aNbShapes.NbWires());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatFaces, 20 * aNbShapes.NbFaces());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatShells, aNbShapes.NbShells());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatSolids, 50 * aNbShapes.NbSolids());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompsolids, aNbShapes.NbCompsolids());
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompounds, aNbShapes.NbCompounds());
    }

    //! @name Methods for building the result

    /// <summary>Builds the result, catching the failures as <see cref="BOPAlgo_AlertBuilderFailed"/>.</summary>
    protected virtual void PerformInternal(BOPAlgo_PaveFiller theFiller, Message_ProgressRange theRange)
    {
        GetReport().Clear();
        //
        try
        {
            PerformInternal1(theFiller, theRange);
        }
        //
        catch (Standard_Failure)
        {
            AddError(new BOPAlgo_AlertBuilderFailed());
        }
    }

    /// <summary>Builds the result: the images of the vertices, edges, wires, faces, shells, solids and compounds.</summary>
    protected virtual void PerformInternal1(BOPAlgo_PaveFiller theFiller, Message_ProgressRange theRange)
    {
        myPaveFiller = theFiller;
        myDS = myPaveFiller.PDS();
        myContext = myPaveFiller.Context();
        myFuzzyValue = myPaveFiller.FuzzyValue();
        myNonDestructive = myPaveFiller.NonDestructive();
        //
        using var aPS = new Message_ProgressScope(theRange, "Building the result of General Fuse operation", 100);
        // 1. CheckData
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors())
        {
            return;
        }
        //
        BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        analyzeProgress(100.0, aSteps);
        // 3. Fill Images
        // 3.1 Vertice
        FillImagesVertices(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_VERTEX);
        if (HasErrors())
        {
            return;
        }
        // 3.2 Edges
        FillImagesEdges(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_EDGE);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.3 Wires
        FillImagesContainers(TopAbs_WIRE, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatWires)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_WIRE);
        if (HasErrors())
        {
            return;
        }

        // 3.4 Faces
        FillImagesFaces(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatFaces)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(TopAbs_FACE);
        if (HasErrors())
        {
            return;
        }
        // 3.5 Shells
        FillImagesContainers(TopAbs_SHELL, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatShells)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(TopAbs_SHELL);
        if (HasErrors())
        {
            return;
        }
        // 3.6 Solids
        FillImagesSolids(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatSolids)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(TopAbs_SOLID);
        if (HasErrors())
        {
            return;
        }
        // 3.7 CompSolids
        FillImagesContainers(TopAbs_COMPSOLID, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompsolids)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(TopAbs_COMPSOLID);
        if (HasErrors())
        {
            return;
        }

        // 3.8 Compounds
        FillImagesCompounds(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompounds)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(TopAbs_COMPOUND);
        if (HasErrors())
        {
            return;
        }
        //
        // 4 History
        PrepareHistory(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors())
        {
            return;
        }
        //
        // 5 Post-treatment
        PostTreat(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    //! @name Checking input arguments

    /// <summary>Checks the input data: at least two arguments and a pave filler.</summary>
    protected override void CheckData()
    {
        int aNb = myArguments.Extent();
        if (aNb < 2)
        {
            AddError(new BOPAlgo_AlertTooFewArguments()); // too few arguments to process
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>Checks that there is a pave filler and takes its warnings and errors.</summary>
    protected void CheckFiller()
    {
        if (myPaveFiller is null)
        {
            AddError(new BOPAlgo_AlertNoFiller());
            return;
        }
        GetReport().Merge(myPaveFiller.GetReport());
    }

    /// <summary>Makes the result an empty compound.</summary>
    protected virtual void Prepare()
    {
        BRep_Builder aBB = new();
        TopoDS_Compound aC = new();
        //
        // 1. myShape is empty compound
        aBB.MakeCompound(ref aC);
        myShape = aC;
    }

    //! @name Post treatment

    /// <summary>Corrects the tolerances of the result (not of the argument sub-shapes in the non-destructive mode).</summary>
    protected virtual void PostTreat(Message_ProgressRange theRange)
    {
        int i, aNbS;
        TopAbs_ShapeEnum aType;
        NCollection_IndexedMap<TopoDS_Shape> aMA = new(TopTools_ShapeMapHasher.Instance);
        if (myPaveFiller!.NonDestructive())
        {
            // MapToAvoid
            aNbS = myDS!.NbSourceShapes();
            for (i = 0; i < aNbS; ++i)
            {
                BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
                aType = aSI.ShapeType();
                if (aType == TopAbs_VERTEX || aType == TopAbs_EDGE || aType == TopAbs_FACE)
                {
                    TopoDS_Shape aS = aSI.Shape();
                    aMA.Add(aS);
                }
            }
        }
        //
        using var aPS = new Message_ProgressScope(theRange, "Post treatment of result shape", 2);
        BOPTools_AlgoTools.CorrectTolerances(myShape, aMA, 0.05, myRunParallel);
        aPS.Next();
        BOPTools_AlgoTools.CorrectShapeTolerances(myShape, aMA, myRunParallel);
    }
}
