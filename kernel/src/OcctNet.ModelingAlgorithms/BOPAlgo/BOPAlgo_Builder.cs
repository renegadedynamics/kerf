// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Occt.BOPAlgo_Operation;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    protected NCollection_List<Shape> myArguments = new();                                                     //!< Arguments of the operation
    protected NCollection_Map<Shape> myMapFence = new(ShapeHasher.Instance);                         //!< Fence map providing the uniqueness of the shapes in the list of arguments
    protected BOPAlgo_PaveFiller? myPaveFiller;                                                                         //!< Pave Filler - algorithm for sub-shapes intersection
    protected BOPDS_DS? myDS;                                                                                           //!< Data Structure - holder of intersection information
    protected IntTools_Context? myContext;                                                                              //!< Context - tool for cashing heavy algorithms such as Projectors and Classifiers
    protected int myEntryPoint;                                                                                         //!< EntryPoint - controls the deletion of the PaveFiller, which could live longer than the Builder
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myImages = new(ShapeHasher.Instance);  //!< Images - map of Images of the sub-shapes of arguments
    protected NCollection_DataMap<Shape, Shape> myShapesSD = new(ShapeHasher.Instance);                  //!< ShapesSD - map of SD Shapes
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myOrigins = new(ShapeHasher.Instance); //!< Origins - map of Origins, back map of Images
    protected NCollection_DataMap<Shape, NCollection_List<Shape>> myInParts = new(ShapeHasher.Instance); //!< InParts - map of own and acquired IN faces of the arguments solids
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
    public virtual void AddArgument(in Shape theShape)
    {
        if (myMapFence.Add(theShape))
        {
            myArguments.Append(theShape);
        }
    }

    /// <summary>Sets the arguments.</summary>
    public virtual void SetArguments(NCollection_List<Shape> theShapes)
    {
        NCollection_List<Shape>.Iterator aIt = new();
        //
        myArguments.Clear();
        // OCCT: myMapFence isn't cleared (occt-issues.md, unit 6A)
        //
        aIt.Initialize(theShapes);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            AddArgument(aS);
        }
    }

    /// <summary>The arguments.</summary>
    public NCollection_List<Shape> Arguments() => myArguments;

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
    public override void Perform(ProgressRange theRange = default)
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
        using var aPS = new ProgressScope(theRange, "Performing General Fuse operation", 10);
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
    public virtual void PerformWithFiller(BOPAlgo_PaveFiller theFiller, ProgressRange theRange = default)
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
    public virtual void BuildBOP(NCollection_List<Shape> theObjects, Containment theObjState,
                                 NCollection_List<Shape> theTools, Containment theToolsState,
                                 ProgressRange theRange, Report? theReport = null)
    {
        if (HasErrors())
        {
            return;
        }

        // Report for the method
        Report aReport = theReport is null ? myReport : theReport;

        if (myArguments.IsEmpty() || myShape.IsNull)
        {
            aReport.AddAlert(Fail, new BOPAlgo_AlertBuilderFailed());
            return;
        }
        // Check the input data
        if ((theObjState != In && theObjState != Out)
            || (theToolsState != In && theToolsState != Out))
        {
            aReport.AddAlert(Fail, new BOPAlgo_AlertBOPNotSet());
            return;
        }

        // Check input shapes
        bool hasObjects = !theObjects.IsEmpty();
        bool hasTools = !theTools.IsEmpty();
        if (!hasObjects && !hasTools)
        {
            aReport.AddAlert(Fail, new BOPAlgo_AlertTooFewArguments());
            return;
        }

        // Check that all input solids are from the arguments
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aList = i == 0 ? theObjects : theTools;
            NCollection_List<Shape>.Iterator itLS = new(aList);
            for (; itLS.More(); itLS.Next())
            {
                Shape aS = itLS.Value();
                // Check if the shape belongs to the arguments of operation
                if (myDS!.Index(aS) < 0)
                {
                    aReport.AddAlert(Fail, new BOPAlgo_AlertUnknownShape(aS));
                    return;
                }

                // Check if the shape is a solid or collection of them
                if (aS.ShapeType() != ShapeType.Solid)
                {
                    NCollection_List<Shape> aLS = new();
                    NCollection_Map<Shape> aMFence0 = new(ShapeHasher.Instance);
                    BOPTools_AlgoTools.TreatCompound(aS, aLS, aMFence0);

                    NCollection_List<Shape>.Iterator it = new(aLS);
                    for (; it.More(); it.Next())
                    {
                        Shape aSx = it.Value();
                        if (aSx.ShapeType() != ShapeType.Solid && aSx.ShapeType() != ShapeType.CompSolid)
                        {
                            aReport.AddAlert(Fail, new BOPAlgo_AlertUnsupportedType(aS));
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
        NCollection_IndexedMap<Shape> aMObjFacesOri = new(EqualityComparer<Shape>.Default),
                                             aMToolFacesOri = new(EqualityComparer<Shape>.Default);
        // Prepare the maps of splits of solids faces
        NCollection_IndexedMap<Shape> aMObjFaces = new(ShapeHasher.Instance),
                                             aMToolFaces = new(ShapeHasher.Instance);
        // Copy the list of IN faces of the solids into map
        NCollection_Map<Shape> anINObjects = new(ShapeHasher.Instance),
                                      anINTools = new(ShapeHasher.Instance);

        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<Shape> aList = i == 0 ? theObjects : theTools;
            NCollection_IndexedMap<Shape> aMapOri = i == 0 ? aMObjFacesOri : aMToolFacesOri;
            NCollection_IndexedMap<Shape> aMap = i == 0 ? aMObjFaces : aMToolFaces;
            NCollection_List<Shape>.Iterator itLS = new(aList);
            for (; itLS.More(); itLS.Next())
            {
                Shape aShape = itLS.Value();
                Explorer expS = new(aShape, ShapeType.Solid);
                for (; expS.More(); expS.Next())
                {
                    Shape aS = expS.Current();
                    Explorer expF = new(aS, ShapeType.Face);
                    for (; expF.More(); expF.Next())
                    {
                        Shape aF = expF.Current();
                        if (aF.Orientation() != Forward && aF.Orientation() != Reversed)
                        {
                            continue;
                        }
                        if (myImages.Find(aF, out NCollection_List<Shape> pLFIm))
                        {
                            NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                            for (; itLFIm.More(); itLFIm.Next())
                            {
                                Face aFImRef = ShapeCasts.Face(itLFIm.Value());
                                if (BOPTools_AlgoTools.IsSplitToReverse(aFImRef, aF, myContext))
                                {
                                    Face aFIm = aFImRef;
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
                    if (myInParts.Find(aS, out NCollection_List<Shape> pLFIN))
                    {
                        NCollection_Map<Shape> anINMap = i == 0 ? anINObjects : anINTools;
                        NCollection_List<Shape>.Iterator itLFIn = new(pLFIN);
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
        bool isObjectsIN = theObjState == In, isToolsIN = theToolsState == In;

        // Shortcuts
        bool bAvoidIN = !isObjectsIN && !isToolsIN,  // avoid all in faces
             bAvoidINforBoth = isObjectsIN != isToolsIN; // avoid faces IN for both groups

        // Choose which SD faces are needed to be taken - equally or differently oriented faces
        bool isSameOriNeeded = theObjState == theToolsState;
        // Resulting faces
        NCollection_IndexedMap<Shape> aMResFacesOri = new(EqualityComparer<Shape>.Default);
        NCollection_Map<Shape> aMResFacesFence = new(ShapeHasher.Instance);
        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance),
                                      aMFToAvoid = new(ShapeHasher.Instance);
        // Oriented fence map
        NCollection_Map<Shape> aMFenceOri = new(EqualityComparer<Shape>.Default);

        for (int i = 0; i < 2; ++i)
        {
            NCollection_IndexedMap<Shape> aMap = i == 0 ? aMObjFacesOri : aMToolFacesOri;
            NCollection_IndexedMap<Shape> anOppositeMap = i == 0 ? aMToolFaces : aMObjFaces;
            NCollection_Map<Shape> anINMap = i == 0 ? anINObjects : anINTools;
            NCollection_Map<Shape> anOppositeINMap = i == 0 ? anINTools : anINObjects;
            bool bTakeIN = i == 0 ? isObjectsIN : isToolsIN;

            int aNbF = aMap.Extent();
            for (int j = 1; j <= aNbF; ++j)
            {
                Shape aFIm = aMap[j];

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
        NCollection_List<Shape> aResFaces = new();
        int aNbRF = aMResFacesOri.Extent();
        for (int i = 1; i <= aNbRF; ++i)
        {
            Shape aRF = aMResFacesOri[i];
            if (!aMFToAvoid.Contains(aRF))
            {
                aResFaces.Append(aRF);
            }
        }
        using var aPS = new ProgressScope(theRange, null, 2);
        BrepBuilder aBB = new();

        // Try to build closed solids from the faces
        BOPAlgo_BuilderSolid aBS = new();
        aBS.SetShapes(aResFaces);
        aBS.SetRunParallel(myRunParallel);
        aBS.SetContext(myContext);
        aBS.SetFuzzyValue(myFuzzyValue);
        aBS.Perform(aPS.Next());

        // Resulting solids
        NCollection_List<Shape> aResSolids = new();

        aMFence.Clear();
        if (!aBS.HasErrors())
        {
            // If any, add solids into resulting compound
            NCollection_List<Shape>.Iterator itA = new(aBS.Areas());
            for (; itA.More(); itA.Next())
            {
                Shape aSolid = itA.Value();
                // The solid must contain at least one face
                // from either of objects or tools
                Explorer expF = new(aSolid, ShapeType.Face);
                for (; expF.More(); expF.Next())
                {
                    Shape aF = expF.Current();
                    if (aMObjFacesOri.Contains(aF) || aMToolFacesOri.Contains(aF))
                    {
                        break;
                    }
                }
                if (expF.More())
                {
                    aResSolids.Append(aSolid);
                    Exploration.MapShapes(aSolid, aMFence);
                }
            }
        }
        else
        {
            return;
        }

        // Collect unused faces
        Compound anUnUsedFaces = new();
        aBB.MakeCompound(ref anUnUsedFaces);

        NCollection_List<Shape>.Iterator itLF = new(aResFaces);
        for (; itLF.More(); itLF.Next())
        {
            if (aMFence.Add(itLF.Value()))
            {
                aBB.Add(anUnUsedFaces, itLF.Value());
            }
        }

        // Build blocks from the unused faces
        NCollection_List<Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(anUnUsedFaces, ShapeType.Edge, ShapeType.Face, aLCB);

        // Build solid from each block
        NCollection_List<Shape>.Iterator itCB = new(aLCB);
        for (; itCB.More(); itCB.Next())
        {
            Shape aCB = itCB.Value();
            Shell aShell0 = new();
            aBB.MakeShell(ref aShell0);
            Shape aShell = aShell0;
            // Add faces of the block to the shell
            Explorer anExpF = new(aCB, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                aBB.Add(aShell, anExpF.Current());
            }

            BOPTools_AlgoTools.OrientFacesOnShell(ref aShell);
            // Make solid out of the shell
            Solid aSolid = new();
            aBB.MakeSolid(ref aSolid);
            aBB.Add(aSolid, aShell);
            // Add new solid to result
            aResSolids.Append(aSolid);
        }

        if (!bAvoidIN)
        {
            // Fill solids with internal parts coming with the solids
            NCollection_List<Shape> anInParts = new();
            for (int i = 0; i < 2; ++i)
            {
                NCollection_List<Shape> aList = i == 0 ? theObjects : theTools;
                NCollection_List<Shape>.Iterator itLS = new(aList);
                for (; itLS.More(); itLS.Next())
                {
                    Explorer expS = new(itLS.Value(), ShapeType.Solid);
                    for (; expS.More(); expS.Next())
                    {
                        Shape aS = expS.Current(); // Solid
                        for (Iterator it = new(aS); it.More(); it.Next())
                        {
                            Shape aSInt = it.Value();
                            if (aSInt.Orientation() == Internal)
                            {
                                anInParts.Append(aSInt); // vertex or edge
                            }
                            else
                            {
                                // shell treatment
                                Iterator itInt = new(aSInt);
                                if (itInt.More() && itInt.Value().Orientation() == Internal)
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
        Compound aResult0 = new();
        aBB.MakeCompound(ref aResult0);
        Shape aResult = aResult0;

        NCollection_List<Shape>.Iterator itLS2 = new(aResSolids);
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
    public void BuildBOP(NCollection_List<Shape> theObjects, NCollection_List<Shape> theTools,
                         BOPAlgo_Operation theOperation, ProgressRange theRange, Report? theReport = null)
    {
        Containment anObjState, aToolsState;
        switch (theOperation)
        {
            case BOPAlgo_COMMON:
            {
                anObjState = In;
                aToolsState = In;
                break;
            }
            case BOPAlgo_FUSE:
            {
                anObjState = Out;
                aToolsState = Out;
                break;
            }
            case BOPAlgo_CUT:
            {
                anObjState = Out;
                aToolsState = In;
                break;
            }
            case BOPAlgo_CUT21:
            {
                anObjState = In;
                aToolsState = Out;
                break;
            }
            default:
            {
                anObjState = Unknown;
                aToolsState = Unknown;
                break;
            }
        }
        BuildBOP(theObjects, anObjState, theTools, aToolsState, theRange, theReport);
    }

    //! @name Images/Origins

    /// <summary>The map of images: the splits of the argument sub-shapes.</summary>
    public NCollection_DataMap<Shape, NCollection_List<Shape>> Images() => myImages;

    /// <summary>The map of origins: the argument sub-shapes each split comes from.</summary>
    public NCollection_DataMap<Shape, NCollection_List<Shape>> Origins() => myOrigins;

    /// <summary>The map of same-domain shapes: each shape to its representative.</summary>
    public NCollection_DataMap<Shape, Shape> ShapesSD() => myShapesSD;

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
                case ShapeType.Edge:
                {
                    if (myDS.HasPaveBlocks(i))
                    {
                        aCounter.NbEdges()++;
                    }
                    break;
                }
                case ShapeType.Wire:
                    aCounter.NbWires()++;
                    break;
                case ShapeType.Face:
                {
                    if (myDS.HasFaceInfo(i))
                    {
                        aCounter.NbFaces()++;
                    }
                    break;
                }
                case ShapeType.Shell:
                    aCounter.NbShells()++;
                    break;
                case ShapeType.Solid:
                    aCounter.NbSolids()++;
                    break;
                case ShapeType.CompSolid:
                    aCounter.NbCompsolids()++;
                    break;
                case ShapeType.Compound:
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
    protected virtual void PerformInternal(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        GetReport().Clear();
        //
        try
        {
            PerformInternal1(theFiller, theRange);
        }
        //
        catch (FailureException)
        {
            AddError(new BOPAlgo_AlertBuilderFailed());
        }
    }

    /// <summary>Builds the result: the images of the vertices, edges, wires, faces, shells, solids and compounds.</summary>
    protected virtual void PerformInternal1(BOPAlgo_PaveFiller theFiller, ProgressRange theRange)
    {
        myPaveFiller = theFiller;
        myDS = myPaveFiller.PDS();
        myContext = myPaveFiller.Context();
        myFuzzyValue = myPaveFiller.FuzzyValue();
        myNonDestructive = myPaveFiller.NonDestructive();
        //
        using var aPS = new ProgressScope(theRange, "Building the result of General Fuse operation", 100);
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
        BuildResult(ShapeType.Vertex);
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
        BuildResult(ShapeType.Edge);
        if (HasErrors())
        {
            return;
        }
        //
        // 3.3 Wires
        FillImagesContainers(ShapeType.Wire, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatWires)));
        if (HasErrors())
        {
            return;
        }
        //
        BuildResult(ShapeType.Wire);
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
        BuildResult(ShapeType.Face);
        if (HasErrors())
        {
            return;
        }
        // 3.5 Shells
        FillImagesContainers(ShapeType.Shell, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatShells)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(ShapeType.Shell);
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

        BuildResult(ShapeType.Solid);
        if (HasErrors())
        {
            return;
        }
        // 3.7 CompSolids
        FillImagesContainers(ShapeType.CompSolid, aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatCompsolids)));
        if (HasErrors())
        {
            return;
        }

        BuildResult(ShapeType.CompSolid);
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

        BuildResult(ShapeType.Compound);
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
        BrepBuilder aBB = new();
        Compound aC = new();
        //
        // 1. myShape is empty compound
        aBB.MakeCompound(ref aC);
        myShape = aC;
    }

    //! @name Post treatment

    /// <summary>Corrects the tolerances of the result (not of the argument sub-shapes in the non-destructive mode).</summary>
    protected virtual void PostTreat(ProgressRange theRange)
    {
        int i, aNbS;
        ShapeType aType;
        NCollection_IndexedMap<Shape> aMA = new(ShapeHasher.Instance);
        if (myPaveFiller!.NonDestructive())
        {
            // MapToAvoid
            aNbS = myDS!.NbSourceShapes();
            for (i = 0; i < aNbS; ++i)
            {
                BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
                aType = aSI.ShapeType();
                if (aType == ShapeType.Vertex || aType == ShapeType.Edge || aType == ShapeType.Face)
                {
                    Shape aS = aSI.Shape();
                    aMA.Add(aS);
                }
            }
        }
        //
        using var aPS = new ProgressScope(theRange, "Post treatment of result shape", 2);
        BOPTools_AlgoTools.CorrectTolerances(myShape, aMA, 0.05, myRunParallel);
        aPS.Next();
        BOPTools_AlgoTools.CorrectShapeTolerances(myShape, aMA, myRunParallel);
    }
}
