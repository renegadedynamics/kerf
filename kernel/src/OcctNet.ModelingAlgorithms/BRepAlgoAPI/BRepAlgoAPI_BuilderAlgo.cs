// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BuilderAlgo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BuilderAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;

namespace Occt;

/// <summary>
/// The API of the General Fuse algorithm: splits the arguments by each other and returns all the splits.
/// </summary>
/// <remarks>
/// <para>
/// Options besides the base class's: the non-destructive mode (the arguments aren't modified, copies are made where
/// needed; always so for frozen arguments, decision 577), the glue options, the check of input solids for being
/// inverted, and the collection of the history. Errors: <see cref="BOPAlgo_AlertTooFewArguments"/>,
/// <see cref="BOPAlgo_AlertIntersectionFailed"/>, <see cref="BOPAlgo_AlertBuilderFailed"/>; the intersection's and
/// the builder's warnings are collected in the report.
/// </para>
/// <para>
/// A working object (decision 579): <see cref="BRepBuilderAPI_MakeShape.Shape"/>, <c>Modified</c>,
/// <c>Generated</c>, <see cref="SectionEdges"/> and <see cref="History"/> publish, freezing the whole result first;
/// ported code that goes on building reads <see cref="BRepAlgoAPI_Algo.InternalShape"/>,
/// <see cref="InternalHistory"/> and <see cref="InternalSectionEdges"/>. The functional twins are in
/// <see cref="BRepAlgoAPI"/>.
/// </para>
/// </remarks>
public class BRepAlgoAPI_BuilderAlgo : BRepAlgoAPI_Algo
{
    // Inputs
    protected NCollection_List<TopoDS_Shape> myArguments = new(); //!< Arguments of the operation

    // Options
    protected bool myNonDestructive;  //!< Non-destructive mode management
    protected BOPAlgo_GlueEnum myGlue; //!< Gluing mode management
    protected bool myCheckInverted;   //!< Check for inverted solids management
    protected bool myFillHistory;     //!< Controls the history collection

    // Tools
    protected bool myIsIntersectionNeeded;         //!< Whether the intersection of the arguments is performed
    private protected BOPAlgo_PaveFiller? myDSFiller; //!< Intersection tool
    private protected BOPAlgo_Builder? myBuilder;     //!< Building tool
    protected BRepTools_History? myHistory;           //!< The history of the whole operation
    protected BRepTools_History? mySimplifierHistory; //!< The history of the result's simplification (unit 6P)

    /// <summary>An empty algorithm.</summary>
    public BRepAlgoAPI_BuilderAlgo()
    {
        myNonDestructive = false;
        myGlue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
        myCheckInverted = true;
        myFillHistory = true;
        myIsIntersectionNeeded = true;
        myDSFiller = null;
        myBuilder = null;
    }

    /// <summary>An algorithm using the intersections already computed by <paramref name="aPF"/>.</summary>
    public BRepAlgoAPI_BuilderAlgo(BOPAlgo_PaveFiller aPF)
    {
        myNonDestructive = false;
        myGlue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
        myCheckInverted = true;
        myFillHistory = true;
        myIsIntersectionNeeded = false;
        myBuilder = null;
        myDSFiller = aPF;
    }

    /// <summary>Sets the arguments (copies the list).</summary>
    public void SetArguments(NCollection_List<TopoDS_Shape> theLS) => myArguments = new NCollection_List<TopoDS_Shape>(theLS);

    /// <summary>The arguments.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    /// <summary>
    /// Sets the non-destructive mode: the arguments are not modified, a copy of a sub-shape is made in the result
    /// where it must change. Frozen arguments are always treated so (decision 577).
    /// </summary>
    public void SetNonDestructive(bool theFlag) => myNonDestructive = theFlag;

    public bool NonDestructive() => myNonDestructive;

    /// <summary>Sets the glue option, which speeds up the intersection of arguments that only touch.</summary>
    public void SetGlue(BOPAlgo_GlueEnum theGlue) => myGlue = theGlue;

    public BOPAlgo_GlueEnum Glue() => myGlue;

    /// <summary>Enables or disables the check of the input solids for being inverted.</summary>
    public void SetCheckInverted(bool theCheck) => myCheckInverted = theCheck;

    public bool CheckInverted() => myCheckInverted;

    /// <summary>Performs the algorithm.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        // Setting not done status
        NotDone();
        // Destroy the tools if necessary
        Clear();
        using var aPS = new Message_ProgressScope(theRange, "Performing General Fuse operation", 100);
        // If necessary perform intersection of the argument shapes
        IntersectShapes(myArguments, aPS.Next(70));
        if (HasErrors())
        {
            return;
        }
        // Initialization of the Building tool
        myBuilder = new BOPAlgo_Builder();
        // Set arguments to builder
        myBuilder.SetArguments(myArguments);
        // Build the result basing on intersection results
        BuildResult(aPS.Next(30));
    }

    /// <summary>
    /// Simplifies the result by unifying the tangential edges and faces (ShapeUpgrade_UnifySameDomain). The fuzzy value
    /// of the operation is the unifier's linear tolerance; the non-destructive mode is its safe input mode. The history
    /// of the simplification is merged into the history of the operation, so <c>Modified</c>, <c>Generated</c> and
    /// <see cref="IsDeleted"/> take it into account.
    /// </summary>
    /// <remarks>
    /// Runs on the unfrozen result (decision 579). The result shares the arguments' untouched sub-shapes, which are
    /// frozen: the unifier copies its input at the entry (decision 766), so the arguments are never written, whatever the
    /// non-destructive flag, and the sub-shapes it leaves unchanged are the result's (and the arguments') own.
    /// </remarks>
    /// <param name="theUnifyEdges">Controls the edges unification (true by default).</param>
    /// <param name="theUnifyFaces">Controls the faces unification (true by default).</param>
    public void SimplifyResult(bool theUnifyEdges = true, bool theUnifyFaces = true) =>
        SimplifyResult(theUnifyEdges, theUnifyFaces, Precision.Angular());

    /// <inheritdoc cref="SimplifyResult(bool, bool)"/>
    /// <param name="theUnifyEdges">Controls the edges unification.</param>
    /// <param name="theUnifyFaces">Controls the faces unification.</param>
    /// <param name="theAngularTol">Angular criteria for tangency of edges and faces (Precision::Angular() by default).</param>
    public void SimplifyResult(bool theUnifyEdges, bool theUnifyFaces, double theAngularTol)
    {
        if (HasErrors())
        {
            return;
        }

        if (!theUnifyEdges && !theUnifyFaces)
        {
            return;
        }

        // Simplification tool
        ShapeUpgrade_UnifySameDomain anUnifier = new(myShape, theUnifyEdges, theUnifyFaces, true);
        // Pass options
        anUnifier.SetLinearTolerance(myOptions.myFuzzyValue);
        anUnifier.SetAngularTolerance(theAngularTol);
        anUnifier.SetSafeInputMode(myNonDestructive);
        anUnifier.AllowInternalEdges(false);
        // Perform simplification
        anUnifier.Build();
        // Overwrite result with simplified shape
        myShape = anUnifier.InternalShape();
        // Keep simplification history
        mySimplifierHistory = anUnifier.InternalHistory();
        if (myFillHistory)
        {
            // Merge simplification history into result history
            myHistory!.Merge(mySimplifierHistory!);
        }
    }

    /// <summary>The algorithm's own list behind <c>Modified</c>: the splits of <paramref name="theS"/> in the result.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape theS)
    {
        myGenerated.Clear();
        if (myFillHistory && myHistory is not null)
        {
            foreach (TopoDS_Shape aS in myHistory.Modified(theS))
            {
                myGenerated.Append(aS);
            }
        }
        return myGenerated;
    }

    /// <summary>The algorithm's own list behind <c>Generated</c>: the shapes generated from <paramref name="theS"/>.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape theS)
    {
        myGenerated.Clear();
        if (myFillHistory && myHistory is not null)
        {
            foreach (TopoDS_Shape aS in myHistory.Generated(theS))
            {
                myGenerated.Append(aS);
            }
        }
        return myGenerated;
    }

    /// <summary>True if <paramref name="aS"/> has been removed: neither it nor a split of it is in the result.</summary>
    public override bool IsDeleted(in TopoDS_Shape aS) => myFillHistory && myHistory is not null && myHistory.IsRemoved(aS);

    /// <summary>True if any argument has been modified.</summary>
    public virtual bool HasModified() => myFillHistory && myHistory is not null && myHistory.HasModified();

    /// <summary>True if any argument has generated shapes.</summary>
    public virtual bool HasGenerated() => myFillHistory && myHistory is not null && myHistory.HasGenerated();

    /// <summary>True if any argument has been deleted.</summary>
    public virtual bool HasDeleted() => myFillHistory && myHistory is not null && myHistory.HasRemoved();

    /// <summary>Enables or disables the collection of the history.</summary>
    public void SetToFillHistory(bool theHistFlag) => myFillHistory = theHistFlag;

    /// <summary>True if the history is collected.</summary>
    public bool HasHistory() => myFillHistory;

    /// <summary>The section edges: the edges where the arguments intersect (frozen, with the whole result).</summary>
    public IBentoList<TopoDS_Shape> SectionEdges()
    {
        BeforePublishingLists();
        return PublishedList(InternalSectionEdges());
    }

    /// <summary>The algorithm's own list behind <see cref="SectionEdges"/>, unfrozen.</summary>
    internal NCollection_List<TopoDS_Shape> InternalSectionEdges()
    {
        myGenerated.Clear();
        if (myBuilder is null)
        {
            return myGenerated;
        }

        // Fence map to avoid duplicated section edges in the result list
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        // Intersection results
        BOPDS_DS pDS = myDSFiller!.PDS()!;
        // Iterate on all Face/Face interferences and take section edges
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = pDS.InterfFF();
        int aNbFF = aFFs.Length();
        for (int i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFFi = aFFs[i];
            // Section curves between pair of faces
            NCollection_DynamicArray<BOPDS_Curve> aSectionCurves = aFFi.Curves();
            int aNbC = aSectionCurves.Length();
            for (int j = 0; j < aNbC; ++j)
            {
                BOPDS_Curve aCurve = aSectionCurves[j];
                // Section edges created from the curve
                for (NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aCurve.PaveBlocks()); aItPB.More(); aItPB.Next())
                {
                    BOPDS_PaveBlock aPB = aItPB.Value();
                    TopoDS_Shape aSE = pDS.Shape(aPB.Edge());
                    if (!aMFence.Add(aSE))
                    {
                        continue;
                    }
                    // Take into account simplification of the result shape
                    if (mySimplifierHistory is not null)
                    {
                        if (mySimplifierHistory.IsRemoved(aSE))
                        {
                            continue;
                        }

                        IBentoList<TopoDS_Shape> aLSEIm = mySimplifierHistory.Modified(aSE);
                        if (aLSEIm.Count != 0)
                        {
                            foreach (TopoDS_Shape aSEIm in aLSEIm)
                            {
                                if (aMFence.Add(aSEIm))
                                {
                                    myGenerated.Append(aSEIm);
                                }
                            }
                        }
                        else
                        {
                            myGenerated.Append(aSE);
                        }
                    }
                    else
                    {
                        myGenerated.Append(aSE);
                    }
                }
            }
        }
        return myGenerated;
    }

    /// <summary>The intersection tool (a working object of the Boolean component).</summary>
    internal BOPAlgo_PaveFiller? DSFiller() => myDSFiller;

    /// <summary>The building tool.</summary>
    internal BOPAlgo_Builder? Builder() => myBuilder;

    /// <summary>
    /// The history of the operation, null if it isn't collected. The result is frozen first, so every shape the
    /// history lists is frozen.
    /// </summary>
    public BRepTools_History? History()
    {
        BeforePublishingLists();
        return InternalHistory();
    }

    /// <summary>OCCT's <c>History()</c>, without freezing the result.</summary>
    internal BRepTools_History? InternalHistory() => myFillHistory ? myHistory : null;

    /// <summary>Sets the options of the derived classes on the intersection tool; nothing here.</summary>
    protected virtual void SetAttributes()
    {
    }

    /// <summary>Intersects the given shapes with a new intersection tool, unless one was given.</summary>
    protected void IntersectShapes(NCollection_List<TopoDS_Shape> theArgs, Message_ProgressRange theRange)
    {
        if (!myIsIntersectionNeeded)
        {
            return;
        }

        // Create new Filler
        myDSFiller = new BOPAlgo_PaveFiller();
        // Set arguments for intersection
        myDSFiller.SetArguments(theArgs);
        // Set options for intersection
        myDSFiller.SetRunParallel(myOptions.myRunParallel);
        myDSFiller.SetFuzzyValue(myOptions.myFuzzyValue);
        myDSFiller.SetNonDestructive(myNonDestructive);
        myDSFiller.SetGlue(myGlue);
        myDSFiller.SetUseOBB(myOptions.myUseOBB);
        // Set Face/Face intersection options to the intersection algorithm
        SetAttributes();
        // Perform intersection
        myDSFiller.Perform(theRange);
        // Check for the errors during intersection
        GetReport().Merge(myDSFiller.GetReport());
    }

    /// <summary>Builds the result with the building tool from the intersections.</summary>
    protected void BuildResult(Message_ProgressRange theRange = default)
    {
        // Set options to the builder
        myBuilder!.SetRunParallel(myOptions.myRunParallel);
        myBuilder.SetCheckInverted(myCheckInverted);
        myBuilder.SetToFillHistory(myFillHistory);
        // Perform building of the result with pre-calculated intersections
        myBuilder.PerformWithFiller(myDSFiller!, theRange);
        // Merge the warnings of the Building part
        GetReport().Merge(myBuilder.GetReport());
        // Check for the errors
        if (myBuilder.HasErrors())
        {
            return;
        }
        // Set done status
        Done();
        // Get the result shape
        myShape = myBuilder.InternalShape();
        // Fill history
        if (myFillHistory)
        {
            myHistory = new BRepTools_History();
            myHistory.Merge(myBuilder.InternalHistory());
        }
    }

    /// <summary>Clears the algorithm from previous runs.</summary>
    public override void Clear()
    {
        base.Clear();
        if (myIsIntersectionNeeded)
        {
            myDSFiller = null;
        }
        myBuilder = null;
        myHistory = null;
        mySimplifierHistory = null;
    }

    // The lists published by Modified, Generated, SectionEdges and History hold sub-shapes of the result: the whole
    // result is frozen first (decision 414).
    private protected override void BeforePublishingLists() => myShape.TShape()?.FreezeGraph();
}
