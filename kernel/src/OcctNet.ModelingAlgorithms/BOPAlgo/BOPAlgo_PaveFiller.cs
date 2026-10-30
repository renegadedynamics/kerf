// Unit 6A stub of OCCT's BOPAlgo_PaveFiller: the API the BRepAlgoAPI shell calls, replaced by unit 6E's port.
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The intersection of the arguments of a Boolean operation (the "pave filler"): it fills the data structure
/// <see cref="BOPDS_DS"/> with the interferences of the arguments' sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// <b>Unit 6A stub</b> (decision 638). <see cref="Init"/> is OCCT's: the argument checks, the data structure, the
/// context and the iterator of interfering pairs. The intersections themselves (OCCT's PerformVV to PerformFF, the
/// pave blocks, the section curves, the non-destructive check of <c>BOPAlgo_PaveFiller_10</c>) are unit 6E's: if the
/// iterator finds any pair of interfering sub-shapes, the stub adds <see cref="BOPAlgo_AlertIntersectionFailed"/>.
/// Arguments that don't interfere need no intersection, so the stub's data structure is the real one for them.
/// </para>
/// <para>
/// Unit 6E replaces this file with the port; the members below are what BRepAlgoAPI and the stub builders call,
/// and keep their signatures.
/// </para>
/// </remarks>
public class BOPAlgo_PaveFiller : BOPAlgo_Algo
{
    protected NCollection_List<TopoDS_Shape> myArguments = new();
    protected BOPDS_DS? myDS;
    protected BOPDS_Iterator? myIterator;
    protected IntTools_Context? myContext;
    protected BOPAlgo_SectionAttribute mySectionAttribute = new();
    protected bool myNonDestructive;
    protected bool myIsPrimary;
    protected bool myAvoidBuildPCurve;
    protected BOPAlgo_GlueEnum myGlue;

    public BOPAlgo_PaveFiller()
    {
        myIsPrimary = true;
        myAvoidBuildPCurve = false;
        myNonDestructive = false;
        myGlue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
    }

    /// <summary>The data structure (null before <see cref="Perform"/>).</summary>
    public BOPDS_DS DS() => myDS!;

    /// <summary>The data structure (OCCT's pointer form).</summary>
    public BOPDS_DS? PDS() => myDS;

    /// <summary>The iterator of interfering pairs.</summary>
    public BOPDS_Iterator? Iterator() => myIterator;

    /// <summary>Sets the arguments (copies the list).</summary>
    public void SetArguments(NCollection_List<TopoDS_Shape> theLS) => myArguments = new NCollection_List<TopoDS_Shape>(theLS);

    /// <summary>Adds an argument.</summary>
    public void AddArgument(in TopoDS_Shape theShape) => myArguments.Append(theShape);

    /// <summary>The arguments.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    /// <summary>The intersection context.</summary>
    public IntTools_Context? Context() => myContext;

    /// <summary>Sets the options of the section curves (approximation, p-curves).</summary>
    public void SetSectionAttribute(in BOPAlgo_SectionAttribute theSecAttr) => mySectionAttribute = theSecAttr;

    /// <summary>The non-destructive mode: the arguments are not modified (copies are made where needed).</summary>
    public void SetNonDestructive(bool theFlag) => myNonDestructive = theFlag;

    public bool NonDestructive() => myNonDestructive;

    /// <summary>Sets the glue option.</summary>
    public void SetGlue(BOPAlgo_GlueEnum theGlue) => myGlue = theGlue;

    public BOPAlgo_GlueEnum Glue() => myGlue;

    public void SetAvoidBuildPCurve(bool theValue) => myAvoidBuildPCurve = theValue;

    public bool IsAvoidBuildPCurve() => myAvoidBuildPCurve;

    public override void Clear()
    {
        base.Clear();
        myIterator = null;
        myDS = null;
        myContext = null;
    }

    /// <summary>Performs the intersection; an exception becomes <see cref="BOPAlgo_AlertIntersectionFailed"/>.</summary>
    public override void Perform(Message_ProgressRange theRange = default)
    {
        try
        {
            PerformInternal(theRange);
        }
        catch (Standard_Failure)
        {
            AddError(new BOPAlgo_AlertIntersectionFailed());
        }
    }

    protected virtual void PerformInternal(Message_ProgressRange theRange)
    {
        using var aPS = new Message_ProgressScope(theRange, "Performing intersection of shapes", 100);

        Init(aPS.Next(5));
        if (HasErrors())
        {
            return;
        }

        // Unit 6A stub: the intersections are unit 6E's. Without interfering pairs there is nothing to intersect.
        TopAbs_ShapeEnum[] aTypes = [TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_SOLID];
        for (int i = 0; i < aTypes.Length; ++i)
        {
            for (int j = i; j < aTypes.Length; ++j)
            {
                myIterator!.Initialize(aTypes[i], aTypes[j]);
                if (myIterator.More())
                {
                    AddError(new BOPAlgo_AlertIntersectionFailed());
                    return;
                }
            }
        }
    }

    /// <summary>OCCT's initialisation: the checks, the data structure, the context and the iterator.</summary>
    protected virtual void Init(Message_ProgressRange theRange)
    {
        if (myArguments.Extent() == 0)
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        //
        using var aPS = new Message_ProgressScope(theRange, "Initialization of Intersection algorithm", 1);
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
        // 4 NonDestructive flag: PaveFiller_10's check of locked (frozen) arguments is unit 6E's.
    }
}
