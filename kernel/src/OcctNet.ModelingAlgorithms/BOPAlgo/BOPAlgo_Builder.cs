// Unit 6A stub of OCCT's BOPAlgo_Builder: the API the BRepAlgoAPI shell calls, replaced by unit 6F's port.
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The General Fuse builder: builds the split arguments from the intersections of a pave filler.</summary>
/// <remarks>
/// <para>
/// <b>Unit 6A stub</b> (decision 638). The members are OCCT's (argument fence, options, <see cref="PerformWithFiller"/>,
/// <see cref="CheckData"/>, <see cref="CheckFiller"/>, <see cref="Prepare"/>, <see cref="PrepareHistory"/>), but
/// the images (FillImagesVertices to FillImagesCompounds) are unit 6F's. The stub's result is the one for arguments
/// that don't interfere, which the stub pave filler guarantees: no argument is split, so every image is the argument
/// itself, the result shares every argument, and the history records nothing modified or generated.
/// </para>
/// <para>
/// Unit 6F replaces this file with the port, keeping these members' signatures; <see cref="BuildStubResult"/>,
/// <see cref="LocModified"/> and <see cref="LocGenerated"/> become OCCT's.
/// </para>
/// </remarks>
public class BOPAlgo_Builder : BOPAlgo_BuilderShape
{
    protected NCollection_List<TopoDS_Shape> myArguments = new();
    protected NCollection_Map<TopoDS_Shape> myMapFence = new(TopTools_ShapeMapHasher.Instance);
    protected BOPAlgo_PaveFiller? myPaveFiller;
    protected BOPDS_DS? myDS;
    protected IntTools_Context? myContext;
    protected int myEntryPoint;
    protected bool myNonDestructive;
    protected BOPAlgo_GlueEnum myGlue;
    protected bool myCheckInverted;

    public BOPAlgo_Builder()
    {
        myEntryPoint = 0;
        myNonDestructive = false;
        myGlue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
        myCheckInverted = true;
    }

    public override void Clear()
    {
        base.Clear();
        myArguments.Clear();
        myMapFence.Clear();
    }

    /// <summary>The pave filler used.</summary>
    internal BOPAlgo_PaveFiller? PPaveFiller() => myPaveFiller;

    /// <summary>The data structure used.</summary>
    internal BOPDS_DS? PDS() => myDS;

    internal IntTools_Context? Context() => myContext;

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
        myArguments.Clear();
        // OCCT: myMapFence isn't cleared (occt-issues.md, unit 6A)
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(theShapes); aIt.More(); aIt.Next())
        {
            AddArgument(aIt.Value());
        }
    }

    /// <summary>The arguments.</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    public void SetNonDestructive(bool theFlag) => myNonDestructive = theFlag;

    public bool NonDestructive() => myNonDestructive;

    public void SetGlue(BOPAlgo_GlueEnum theGlue) => myGlue = theGlue;

    public BOPAlgo_GlueEnum Glue() => myGlue;

    /// <summary>Enables or disables the check of the input solids for being inverted.</summary>
    public void SetCheckInverted(bool theCheck) => myCheckInverted = theCheck;

    public bool CheckInverted() => myCheckInverted;

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

    protected virtual void PerformInternal(BOPAlgo_PaveFiller theFiller, Message_ProgressRange theRange)
    {
        GetReport().Clear();
        //
        try
        {
            PerformInternal1(theFiller, theRange);
        }
        catch (Standard_Failure)
        {
            AddError(new BOPAlgo_AlertBuilderFailed());
        }
    }

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
        // 3. Unit 6A stub: the images are the arguments themselves (unit 6F fills the images)
        BuildStubResult();
        if (HasErrors())
        {
            return;
        }
        //
        PrepareHistory(aPS.Next(10));
    }

    /// <summary>The stub's result: the arguments, which the stub filler guarantees aren't split (unit 6F replaces it).</summary>
    protected virtual void BuildStubResult()
    {
        BRep_Builder aBB = new();
        for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(myArguments); aIt.More(); aIt.Next())
        {
            aBB.Add(myShape, aIt.Value());
        }
    }

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

    /// <summary>The splits of <paramref name="theS"/> (null: the stub splits nothing).</summary>
    protected virtual NCollection_List<TopoDS_Shape>? LocModified(in TopoDS_Shape theS) => null;

    /// <summary>The shapes generated from <paramref name="theS"/> (none in the stub).</summary>
    protected virtual NCollection_List<TopoDS_Shape> LocGenerated(in TopoDS_Shape theS) => myHistShapes;

    /// <summary>OCCT's history of the result: modified, generated and deleted argument sub-shapes.</summary>
    protected void PrepareHistory(Message_ProgressRange theRange)
    {
        if (!HasHistory())
        {
            return;
        }

        // Initializing history tool
        myHistory = new BRepTools_History();

        // Map the result shape
        myMapShape.Clear();
        TopExp.MapShapes(myShape, myMapShape);

        // Among all input shapes find:
        // - Shapes that have been modified (split). Add the splits kept in the result
        //   shape as Modified from the shape;
        // - Shapes that have created new geometries (i.e. generated new shapes). Add
        //   the generated elements kept in the result shape as Generated from the shape;
        // - Shapes that have no trace in the result shape. Add them as Deleted
        //   during the operation.
        int aNbS = myDS!.NbSourceShapes();
        using var aPS = new Message_ProgressScope(theRange, "Preparing history information", aNbS);
        for (int i = 0; i < aNbS; ++i, aPS.Next())
        {
            TopoDS_Shape aS = myDS.Shape(i);

            // Check if History information is available for this kind of shape.
            if (!BRepTools_History.IsSupportedType(aS))
            {
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }

            bool isModified = false;

            // Check if the shape has any splits
            NCollection_List<TopoDS_Shape>? pLSp = LocModified(aS);
            if (pLSp is not null)
            {
                // Find all splits of the shape which are kept in the result
                for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(pLSp); aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aSp = aIt.Value();
                    // Check if the result shape contains the split
                    if (myMapShape.Contains(aSp))
                    {
                        // Add modified shape with proper orientation; the reversal check of other types
                        // (BOPTools_AlgoTools::IsSplitToReverse) comes with unit 6F
                        TopAbs_ShapeEnum aType = aSp.ShapeType();
                        if (aType == TopAbs_VERTEX || aType == TopAbs_SOLID)
                        {
                            aSp.Orientation(aS.Orientation());
                        }

                        myHistory.AddModified(aS, aSp);
                        isModified = true;
                    }
                }
            }

            // Check if the shape has Generated elements
            for (NCollection_List<TopoDS_Shape>.Iterator aIt = new(LocGenerated(aS)); aIt.More(); aIt.Next())
            {
                TopoDS_Shape aG = aIt.Value();
                if (myMapShape.Contains(aG))
                {
                    myHistory.AddGenerated(aS, aG);
                }
            }

            // Check if the shape has been deleted, i.e. it is not contained in the result
            // and has no Modified shapes.
            if (!isModified && !myMapShape.Contains(aS))
            {
                myHistory.Remove(aS);
            }
        }
    }
}
