// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.BOPAlgo_GlueEnum;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The intersection of the arguments of a Boolean operation (the "pave filler"): it fills the data structure
/// <see cref="BOPDS_DS"/> with the interferences of the arguments' sub-shapes, which the builders then use.
/// </summary>
/// <remarks>
/// <para>
/// The steps, in OCCT's order: the vertex/vertex, vertex/edge, edge/edge, vertex/face and edge/face interferences
/// (with the pave blocks of the edges updated after each), the repeated intersection of the vertices whose tolerance
/// grew, the forced edge/edge and edge/face interferences, the face/face intersections and their section curves,
/// the split edges, the blocks, the self-interference check, the p-curves and the degenerated edges.
/// </para>
/// <para>
/// A working object (plan section 6). Its intersection loops run through <see cref="BOPTools_Parallel"/> with a
/// context per worker; the result is the serial one at every degree of parallelism (decision 580).
/// </para>
/// <para>
/// <b>Non-destructive mode.</b> The filler turns non-destructive by itself when an argument is
/// <see cref="TopoDS_Shape.Locked()"/> (<c>BOPAlgo_PaveFiller_10</c>), and frozen shapes are locked, so a published
/// shape is never written (decision 577): an argument sub-shape that must change is replaced by a new one.
/// </para>
/// <para>
/// The allocator constructor goes; the protected typedefs are written out (<c>BOPAlgo_DataMapOfPaveBlockBndBox</c>
/// is <c>NCollection_DataMap&lt;BOPDS_PaveBlock, Bnd_Box&gt;</c>, <c>BOPAlgo_DataMapOfPaveBlockListOfInteger</c>
/// <c>NCollection_DataMap&lt;BOPDS_PaveBlock, NCollection_List&lt;int&gt;&gt;</c> and
/// <c>BOPAlgo_DataMapOfIntegerMapOfPaveBlock</c> <c>NCollection_DataMap&lt;int,
/// NCollection_Map&lt;BOPDS_PaveBlock&gt;&gt;</c>).
/// </para>
/// </remarks>
public partial class BOPAlgo_PaveFiller : BOPAlgo_Algo
{
    // List of operations to be supported by the Progress Indicator
    private enum BOPAlgo_PIOperation
    {
        PIOperation_Prepare = 0,
        PIOperation_PerformVV,
        PIOperation_PerformVE,
        PIOperation_PerformEE,
        PIOperation_PerformVF,
        PIOperation_PerformEF,
        PIOperation_RepeatIntersection,
        PIOperation_ForceInterfEE,
        PIOperation_ForceInterfEF,
        PIOperation_PerformFF,
        PIOperation_MakeSplitEdges,
        PIOperation_MakeBlocks,
        PIOperation_MakePCurves,
        PIOperation_ProcessDE,
        PIOperation_Last
    }

    /// <summary>A range of an edge and its minimal distance to another shape.</summary>
    protected struct EdgeRangeDistance
    {
        public double First;
        public double Last;
        public double Distance;

        public EdgeRangeDistance()
            : this(0.0, 0.0, RealLast())
        {
        }

        public EdgeRangeDistance(double theFirst, double theLast = 0.0, double theDistance = double.MaxValue)
        {
            First = theFirst;
            Last = theLast;
            Distance = theDistance;
        }
    }

    protected NCollection_List<TopoDS_Shape> myArguments = new();
    protected BOPDS_DS? myDS;
    protected BOPDS_Iterator? myIterator;
    protected IntTools_Context? myContext;
    protected BOPAlgo_SectionAttribute mySectionAttribute = new();
    protected bool myNonDestructive;
    protected bool myIsPrimary;
    protected bool myAvoidBuildPCurve;
    protected BOPAlgo_GlueEnum myGlue;

    //! Fence map of intersected faces and pave blocks
    protected NCollection_IntDataMap<NCollection_Map<BOPDS_PaveBlock>> myFPBDone = new();
    //! Sub-shapes with increased tolerance during the operation
    protected NCollection_IntMap myIncreasedSS = new();
    //! Vertices located close to E/E or E/F intersection points
    protected NCollection_IntMap myVertsToAvoidExtension = new();
    //! Map to store minimal distances between shapes
    protected NCollection_DataMap<BOPDS_Pair, NCollection_List<EdgeRangeDistance>> myDistances = new();

    public BOPAlgo_PaveFiller()
    {
        myDS = null;
        myIterator = null;
        myNonDestructive = false;
        myIsPrimary = true;
        myAvoidBuildPCurve = false;
        myGlue = BOPAlgo_GlueOff;
    }

    /// <summary>The data structure (filled by <see cref="Perform"/>).</summary>
    public BOPDS_DS DS() => myDS!;

    /// <summary>The data structure (OCCT's pointer form; null before <see cref="Perform"/>).</summary>
    public BOPDS_DS? PDS() => myDS;

    /// <summary>The iterator of interfering pairs.</summary>
    public BOPDS_Iterator? Iterator() => myIterator;

    /// <summary>Sets the arguments (copies the list).</summary>
    public void SetArguments(NCollection_List<TopoDS_Shape> theLS) => myArguments = new NCollection_List<TopoDS_Shape>(theLS);

    /// <summary>Adds an argument.</summary>
    public void AddArgument(in TopoDS_Shape theShape) => myArguments.Append(theShape);

    /// <summary>The arguments.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    /// <summary>The intersection context (null before <see cref="Perform"/>).</summary>
    public IntTools_Context? Context() => myContext;

    /// <summary>Sets the options of the section curves (approximation, p-curves).</summary>
    public void SetSectionAttribute(in BOPAlgo_SectionAttribute theSecAttr) => mySectionAttribute = theSecAttr;

    /// <summary>
    /// The non-destructive mode: the arguments are not modified (new shapes are made where needed). Locked (frozen)
    /// arguments turn it on whatever is set here.
    /// </summary>
    public void SetNonDestructive(bool theFlag) => myNonDestructive = theFlag;

    /// <summary>Whether the operation is non-destructive.</summary>
    public bool NonDestructive() => myNonDestructive;

    /// <summary>Sets the glue option.</summary>
    public void SetGlue(BOPAlgo_GlueEnum theGlue) => myGlue = theGlue;

    /// <summary>The glue option.</summary>
    public BOPAlgo_GlueEnum Glue() => myGlue;

    /// <summary>Disables the building of the p-curves of the section edges on planar faces.</summary>
    public void SetAvoidBuildPCurve(bool theValue) => myAvoidBuildPCurve = theValue;

    /// <summary>Whether the building of the p-curves on planar faces is avoided.</summary>
    public bool IsAvoidBuildPCurve() => myAvoidBuildPCurve;

    protected void SetIsPrimary(bool theFlag) => myIsPrimary = theFlag;

    protected bool IsPrimary() => myIsPrimary;

    public override void Clear()
    {
        base.Clear();
        myIterator = null;
        myDS = null;
        myIncreasedSS.Clear();
    }

    /// <summary>OCCT's initialisation: the checks, the data structure, the context, the iterator and the mode.</summary>
    protected virtual void Init(ProgressRange theRange)
    {
        if (myArguments.Extent() == 0)
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        using var aPS = new ProgressScope(theRange, "Initialization of Intersection algorithm", 1);
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(myArguments); aIt.More(); aIt.Next())
        {
            if (aIt.Value().IsNull())
            {
                AddError(new BOPAlgo_AlertNullInputShapes());
                return;
            }
        }
        //
        // 0 Clear
        Clear();
        //
        // 1.myDS
        myDS = new BOPDS_DS();
        myDS.SetArguments(myArguments);
        myDS.Init(myFuzzyValue);
        //
        // 2 myContext
        myContext = new IntTools_Context();
        //
        // 3.myIterator
        myIterator = new BOPDS_Iterator();
        myIterator.SetRunParallel(myRunParallel);
        myIterator.SetDS(myDS);
        myIterator.Prepare(myContext, myUseOBB, myFuzzyValue);
        //
        // 4 NonDestructive flag
        SetNonDestructive();
    }

    /// <summary>Performs the intersection; an exception becomes <see cref="BOPAlgo_AlertIntersectionFailed"/>.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        try
        {
            //
            PerformInternal(theRange);
        }
        //
        catch (FailureException)
        {
            AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }

    protected virtual void PerformInternal(ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, "Performing intersection of shapes", 100);

        Init(aPS.Next(5));
        if (HasErrors())
        {
            return;
        }

        // Compute steps of the PI
        BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        analyzeProgress(95, aSteps);
        //
        Prepare(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_Prepare)));
        if (HasErrors())
        {
            return;
        }
        // 00
        PerformVV(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVV)));
        if (HasErrors())
        {
            return;
        }
        // 01
        PerformVE(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVE)));
        if (HasErrors())
        {
            return;
        }
        //
        UpdatePaveBlocksWithSDVertices();
        // 11
        PerformEE(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEE)));
        if (HasErrors())
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        // 02
        PerformVF(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVF)));
        if (HasErrors())
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        // 12
        PerformEF(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEF)));
        if (HasErrors())
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();
        UpdateInterfsWithSDVertices();

        // Repeat Intersection with increased vertices
        RepeatIntersection(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_RepeatIntersection)));
        if (HasErrors())
        {
            return;
        }
        // Force intersection of edges after increase
        // of the tolerance values of their vertices
        ForceInterfEE(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEE)));
        if (HasErrors())
        {
            return;
        }
        // Force Edge/Face intersection after increase
        // of the tolerance values of their vertices
        ForceInterfEF(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEF)));
        if (HasErrors())
        {
            return;
        }
        //
        // 22
        PerformFF(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PerformFF)));
        if (HasErrors())
        {
            return;
        }
        //
        UpdateBlocksWithSharedVertices();
        //
        myDS!.RefineFaceInfoIn();
        //
        MakeSplitEdges(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakeSplitEdges)));
        if (HasErrors())
        {
            return;
        }
        //
        UpdatePaveBlocksWithSDVertices();
        //
        MakeBlocks(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakeBlocks)));
        if (HasErrors())
        {
            return;
        }
        //
        CheckSelfInterference();
        //
        UpdateInterfsWithSDVertices();
        myDS.ReleasePaveBlocks();
        myDS.RefineFaceInfoOn();
        //
        RemoveMicroEdges();
        //
        MakePCurves(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_MakePCurves)));
        if (HasErrors())
        {
            return;
        }
        //
        ProcessDE(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_ProcessDE)));
        if (HasErrors())
        {
            return;
        }
    }

    /// <summary>Repeats the intersection of the vertices whose tolerance has been increased.</summary>
    protected void RepeatIntersection(ProgressRange theRange)
    {
        // Find all vertices with increased tolerance
        NCollection_IntMap anExtraInterfMap = new();
        int aNbS = myDS!.NbSourceShapes();
        using var aPS = new ProgressScope(theRange, "Repeat intersection", 3);
        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_VERTEX)
            {
                continue;
            }
            // Check if the tolerance of the original vertex has been increased
            if (myIncreasedSS.Contains(i))
            {
                anExtraInterfMap.Add(i);
                continue;
            }

            // Check if the vertex created a new vertex with greater tolerance
            int nVSD = 0; // OCCT: uninitialised, read only when HasShapeSD writes it
            if (!myDS.HasShapeSD(i, ref nVSD))
            {
                continue;
            }

            if (myIncreasedSS.Contains(nVSD))
            {
                anExtraInterfMap.Add(i);
            }
        }

        if (anExtraInterfMap.IsEmpty())
        {
            return;
        }

        // Update iterator of pairs of shapes with interfering boxes
        myIterator!.IntersectExt(anExtraInterfMap);

        // Perform intersections with vertices

        PerformVV(aPS.Next());
        if (HasErrors())
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();

        PerformVE(aPS.Next());
        if (HasErrors())
        {
            return;
        }
        UpdatePaveBlocksWithSDVertices();

        PerformVF(aPS.Next());
        if (HasErrors())
        {
            return;
        }

        UpdatePaveBlocksWithSDVertices();
        UpdateInterfsWithSDVertices();
    }

    protected override void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
        if (!myNonDestructive)
        {
            theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_Prepare, 1 * theWhole / 100.0);
        }
    }

    protected override void fillPISteps(BOPAlgo_PISteps theSteps)
    {
        // Get number of all intersecting pairs
        int aVVSize = 0, aVESize = 0, aEESize = 0, aVFSize = 0, aEFSize = 0, aFFSize = 0;

        myIterator!.Initialize(TopAbs_VERTEX, TopAbs_VERTEX);
        aVVSize = myIterator.ExpectedLength();

        myIterator.Initialize(TopAbs_VERTEX, TopAbs_EDGE);
        aVESize = myIterator.ExpectedLength();

        myIterator.Initialize(TopAbs_EDGE, TopAbs_EDGE);
        aEESize = myIterator.ExpectedLength();

        myIterator.Initialize(TopAbs_VERTEX, TopAbs_FACE);
        aVFSize = myIterator.ExpectedLength();

        if (myGlue != BOPAlgo_GlueFull)
        {
            myIterator.Initialize(TopAbs_EDGE, TopAbs_FACE);
            aEFSize = myIterator.ExpectedLength();
        }

        myIterator.Initialize(TopAbs_FACE, TopAbs_FACE);
        aFFSize = myIterator.ExpectedLength();

        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVV, aVVSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVE, 2 * aVESize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEE, 5 * aEESize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformVF, 5 * aVFSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformEF, 10 * aEFSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_RepeatIntersection, 0.2 * (aVVSize + aVESize + aVFSize));
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEE, 2 * aEESize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ForceInterfEF, 2 * aEFSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PerformFF, (myGlue == BOPAlgo_GlueFull ? 1 : 30) * aFFSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakeSplitEdges, aEESize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakeBlocks, (myGlue == BOPAlgo_GlueFull ? 0 : 5) * aFFSize);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_MakePCurves, myAvoidBuildPCurve ? 0 : 0.2 * (aEESize + aEFSize));
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_ProcessDE, 0.1 * aEESize);
    }
}
