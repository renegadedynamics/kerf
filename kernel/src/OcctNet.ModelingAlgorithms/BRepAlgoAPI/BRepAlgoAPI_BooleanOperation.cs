// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BooleanOperation.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BooleanOperation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using static Occt.BOPAlgo_Operation;

namespace Occt;

/// <summary>
/// The root of the Boolean operations on arbitrary shapes: the arguments are in two groups, objects and tools, and
/// the operation is FUSE (union), COMMON (intersection), CUT (subtraction) or SECTION.
/// </summary>
/// <remarks>
/// <para>
/// The rules: for FUSE all arguments have the same dimension; for CUT the tools' smallest dimension is at least the
/// objects' largest; COMMON and SECTION take any dimensions. Errors besides the base class's:
/// <see cref="BOPAlgo_AlertBOPNotSet"/> when the operation isn't set.
/// </para>
/// <para>
/// Left out: OCCT's debug dump of the arguments (<c>BRepAlgoAPI_DumpOper</c>, enabled by the CSF_DEBUG_BOP environment
/// variable), which writes files: a result never depends on the environment.
/// </para>
/// </remarks>
public class BRepAlgoAPI_BooleanOperation : BRepAlgoAPI_BuilderAlgo
{
    protected NCollection_List<Shape> myTools = new(); //!< Tool arguments of operation
    protected BOPAlgo_Operation myOperation;                  //!< Type of Boolean Operation

    /// <summary>An empty operation.</summary>
    public BRepAlgoAPI_BooleanOperation()
    {
        myOperation = BOPAlgo_UNKNOWN;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="thePF"/>.</summary>
    public BRepAlgoAPI_BooleanOperation(BOPAlgo_PaveFiller thePF)
        : base(thePF)
    {
        myOperation = BOPAlgo_UNKNOWN;
    }

    /// <summary>The operation on two arguments (obsolete in OCCT).</summary>
    protected BRepAlgoAPI_BooleanOperation(in Shape theS1, in Shape theS2, BOPAlgo_Operation theOp)
    {
        myOperation = theOp;
        myArguments.Append(theS1);
        myTools.Append(theS2);
    }

    /// <summary>The operation on two arguments with intersections already computed (obsolete in OCCT).</summary>
    protected BRepAlgoAPI_BooleanOperation(in Shape theS1, in Shape theS2, BOPAlgo_PaveFiller thePF,
                                           BOPAlgo_Operation theOp)
        : base(thePF)
    {
        myOperation = theOp;
        myArguments.Append(theS1);
        myTools.Append(theS2);
    }

    /// <summary>The first object (obsolete in OCCT).</summary>
    public Shape Shape1() => myArguments.First();

    /// <summary>The first tool (obsolete in OCCT).</summary>
    public Shape Shape2() => myTools.First();

    /// <summary>Sets the tools (copies the list).</summary>
    public void SetTools(NCollection_List<Shape> theLS) => myTools = new NCollection_List<Shape>(theLS);

    /// <summary>The tools.</summary>
    public NCollection_List<Shape> Tools() => myTools;

    /// <summary>Sets the type of the operation.</summary>
    public void SetOperation(BOPAlgo_Operation theBOP) => myOperation = theBOP;

    /// <summary>The type of the operation.</summary>
    public BOPAlgo_Operation Operation() => myOperation;

    /// <summary>Performs the Boolean operation.</summary>
    public override void Build(ProgressRange theRange = default)
    {
        // Set Not Done status by default
        NotDone();
        // Clear from previous runs
        Clear();
        // Check for availability of arguments and tools
        // Both should be present
        if (myArguments.IsEmpty() || myTools.IsEmpty())
        {
            AddError(new BOPAlgo_AlertTooFewArguments());
            return;
        }
        // Check if the operation is set
        if (myOperation == BOPAlgo_UNKNOWN)
        {
            AddError(new BOPAlgo_AlertBOPNotSet());
            return;
        }

        // (OCCT's CSF_DEBUG_BOP dump of the arguments is left out.)

        string aPSName;
        switch (myOperation)
        {
            case BOPAlgo_COMMON:
                aPSName = "Performing COMMON operation";
                break;
            case BOPAlgo_FUSE:
                aPSName = "Performing FUSE operation";
                break;
            case BOPAlgo_CUT:
            case BOPAlgo_CUT21:
                aPSName = "Performing CUT operation";
                break;
            case BOPAlgo_SECTION:
                aPSName = "Performing SECTION operation";
                break;
            default:
                return;
        }

        using var aPS = new ProgressScope(theRange, aPSName, myIsIntersectionNeeded ? 100 : 30);
        // If necessary perform intersection of the argument shapes
        if (myIsIntersectionNeeded)
        {
            // Combine Objects and Tools into a single list for intersection
            NCollection_List<Shape> aLArgs = new(myArguments);
            for (NCollection_List<Shape>.Iterator it = new(myTools); it.More(); it.Next())
            {
                aLArgs.Append(it.Value());
            }
            // Perform intersection
            IntersectShapes(aLArgs, aPS.Next(70));
            if (HasErrors())
            {
                return;
            }
        }
        // Builder Initialization
        if (myOperation == BOPAlgo_SECTION)
        {
            myBuilder = new BOPAlgo_Section();
            myBuilder.SetArguments(myDSFiller!.Arguments());
        }
        else
        {
            BOPAlgo_BOP aBOP = new();
            myBuilder = aBOP;
            myBuilder.SetArguments(myArguments);
            aBOP.SetTools(myTools);
            aBOP.SetOperation(myOperation);
        }
        // Build the result
        BuildResult(aPS.Next(30));
    }
}
