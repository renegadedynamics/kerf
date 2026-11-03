// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Check.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Check.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Schematiq.Collections;
using static Occt.BOPAlgo_CheckStatus;
using static Occt.BOPAlgo_Operation;

namespace Occt;

/// <summary>
/// Checks one shape, or two shapes for a Boolean operation, for problems that would make the operation fail: the
/// types of the shapes (for the operation), small edges, self-interference (<see cref="BOPAlgo_ArgumentAnalyzer"/>),
/// and topological validity (<see cref="BRepCheck_Analyzer"/>).
/// </summary>
/// <remarks>
/// OCCT's class derives from BOPAlgo_Options (fuzzy value, parallel mode, report), as here. The shapes are only read,
/// so frozen shapes are checked as they are. <see cref="Result"/> publishes the faults, whose shapes are frozen as
/// they are read (<see cref="BOPAlgo_CheckResult"/>); ported code reads <see cref="InternalResult"/>.
/// </remarks>
public class BRepAlgoAPI_Check : BOPAlgo_Options
{
    // Inputs
    protected TopoDS_Shape myS1 = new();             //!< The first shape
    protected TopoDS_Shape myS2 = new();             //!< The second shape
    protected bool myTestSE;                         //!< Flag defining whether to look for small edges in the given shapes or not
    protected bool myTestSI;                         //!< Flag defining whether to check the input edges on self-interference or not
    protected BOPAlgo_Operation myOperation;         //!< Type of Boolean operation for which the validity of input shapes should be checked

    // Results
    protected NCollection_List<BOPAlgo_CheckResult> myFaultyShapes = new(); //!< Found faulty shapes

    /// <summary>An empty check: give the shapes with <see cref="SetData(in TopoDS_Shape, bool, bool)"/>.</summary>
    public BRepAlgoAPI_Check()
    {
        myTestSE = true;
        myTestSI = true;
        myOperation = BOPAlgo_UNKNOWN;
    }

    /// <summary>Checks a single shape, on small edges and self-interference unless told not to.</summary>
    public BRepAlgoAPI_Check(in TopoDS_Shape theS, bool bTestSE = true, bool bTestSI = true,
                             ProgressRange theRange = default)
    {
        myS1 = theS;
        myTestSE = bTestSE;
        myTestSI = bTestSI;
        myOperation = BOPAlgo_UNKNOWN;
        Perform(theRange);
    }

    /// <summary>
    /// Checks a couple of shapes: each shape's validity, and the validity of their types for the Boolean operation
    /// of the given type.
    /// </summary>
    public BRepAlgoAPI_Check(in TopoDS_Shape theS1, in TopoDS_Shape theS2, BOPAlgo_Operation theOp = BOPAlgo_UNKNOWN,
                             bool bTestSE = true, bool bTestSI = true, ProgressRange theRange = default)
    {
        myS1 = theS1;
        myS2 = theS2;
        myTestSE = bTestSE;
        myTestSI = bTestSI;
        myOperation = theOp;
        Perform(theRange);
    }

    /// <summary>Initializes the algorithm with a single shape.</summary>
    public void SetData(in TopoDS_Shape theS, bool bTestSE = true, bool bTestSI = true)
    {
        myS1 = theS;
        myS2 = new TopoDS_Shape();
        myTestSE = bTestSE;
        myTestSI = bTestSI;
        myFaultyShapes.Clear();
    }

    /// <summary>Initializes the algorithm with a couple of shapes and the operation they are checked for.</summary>
    public void SetData(in TopoDS_Shape theS1, in TopoDS_Shape theS2, BOPAlgo_Operation theOp = BOPAlgo_UNKNOWN,
                        bool bTestSE = true, bool bTestSI = true)
    {
        myS1 = theS1;
        myS2 = theS2;
        myOperation = theOp;
        myTestSE = bTestSE;
        myTestSI = bTestSI;
        myFaultyShapes.Clear();
    }

    /// <summary>Performs the check.</summary>
    public void Perform(ProgressRange theRange = default)
    {
        // Check the incompatibility of shapes types, small edges and self-interference
        BOPAlgo_ArgumentAnalyzer anAnalyzer = new();
        // Set the shapes and options for the check
        anAnalyzer.SetShape1(myS1);
        anAnalyzer.SetShape2(myS2);
        anAnalyzer.OperationType() = myOperation;
        anAnalyzer.ArgumentTypeMode() = true;
        anAnalyzer.SmallEdgeMode() = myTestSE;
        anAnalyzer.SelfInterMode() = myTestSI;
        // Set options from BOPAlgo_Options
        anAnalyzer.SetRunParallel(myRunParallel);
        anAnalyzer.SetFuzzyValue(myFuzzyValue);
        // Perform the check
        using var aPS = new ProgressScope(theRange, "Checking shapes", 1);
        anAnalyzer.Perform(aPS.Next());
        if (UserBreak(aPS))
        {
            return;
        }
        // Get the results
        myFaultyShapes = new NCollection_List<BOPAlgo_CheckResult>(anAnalyzer.InternalGetCheckResult());

        // Check the topological validity of the shapes
        bool isValidS1 = !myS1.IsNull() ? new BRepCheck_Analyzer(myS1).IsValid() : true;

        bool isValidS2 = !myS2.IsNull() ? new BRepCheck_Analyzer(myS2).IsValid() : true;

        if (!isValidS1 || !isValidS2)
        {
            BOPAlgo_CheckResult aRes = new();
            aRes.SetCheckStatus(BOPAlgo_NotValid);
            if (!isValidS1)
            {
                aRes.SetShape1(myS1);
            }
            if (!isValidS2)
            {
                aRes.SetShape2(myS2);
            }
            myFaultyShapes.Append(aRes);
        }
    }

    /// <summary>True if no fault was found.</summary>
    public bool IsValid() => myFaultyShapes.IsEmpty();

    /// <summary>The faults found (their shapes frozen as they are read).</summary>
    public ISchematiqList<BOPAlgo_CheckResult> Result() => PackedList.Own([.. myFaultyShapes]);

    /// <summary>OCCT's <c>Result()</c>: the algorithm's own list, for ported code.</summary>
    internal NCollection_List<BOPAlgo_CheckResult> InternalResult() => myFaultyShapes;
}
