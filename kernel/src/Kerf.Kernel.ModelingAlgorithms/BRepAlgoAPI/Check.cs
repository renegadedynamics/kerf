// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Check.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Check.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Booleans.CheckStatus;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// Checks one shape, or two shapes for a Boolean operation, for problems that would make the operation fail: the
/// types of the shapes (for the operation), small edges, self-interference (<see cref="ArgumentAnalyzer"/>),
/// and topological validity (<see cref="Analyzer"/>).
/// </summary>
/// <remarks>
/// OCCT's class derives from BOPAlgo_Options (fuzzy value, parallel mode, report), as here. The shapes are only read,
/// so frozen shapes are checked as they are. <see cref="Result"/> publishes the faults, whose shapes are frozen as
/// they are read (<see cref="CheckResult"/>); ported code reads <see cref="InternalResult"/>.
/// </remarks>
public class Check : Options
{
    // Inputs
    protected Shape _s1 = new();             //!< The first shape
    protected Shape _s2 = new();             //!< The second shape
    protected bool _testSE;                         //!< Flag defining whether to look for small edges in the given shapes or not
    protected bool _testSI;                         //!< Flag defining whether to check the input edges on self-interference or not
    protected Operation _operation;         //!< Type of Boolean operation for which the validity of input shapes should be checked

    // Results
    protected NCollection_List<CheckResult> _faultyShapes = new(); //!< Found faulty shapes

    /// <summary>An empty check: give the shapes with <see cref="SetData(in Shape, bool, bool)"/>.</summary>
    public Check()
    {
        _testSE = true;
        _testSI = true;
        _operation = Operation.Unknown;
    }

    /// <summary>Checks a single shape, on small edges and self-interference unless told not to.</summary>
    public Check(in Shape s, bool bTestSE = true, bool bTestSI = true,
                             ProgressRange range = default)
    {
        _s1 = s;
        _testSE = bTestSE;
        _testSI = bTestSI;
        _operation = Operation.Unknown;
        Perform(range);
    }

    /// <summary>
    /// Checks a couple of shapes: each shape's validity, and the validity of their types for the Boolean operation
    /// of the given type.
    /// </summary>
    public Check(in Shape s1, in Shape s2, Operation op = Operation.Unknown,
                             bool bTestSE = true, bool bTestSI = true, ProgressRange range = default)
    {
        _s1 = s1;
        _s2 = s2;
        _testSE = bTestSE;
        _testSI = bTestSI;
        _operation = op;
        Perform(range);
    }

    /// <summary>Initializes the algorithm with a single shape.</summary>
    public void SetData(in Shape s, bool bTestSE = true, bool bTestSI = true)
    {
        _s1 = s;
        _s2 = new Shape();
        _testSE = bTestSE;
        _testSI = bTestSI;
        _faultyShapes.Clear();
    }

    /// <summary>Initializes the algorithm with a couple of shapes and the operation they are checked for.</summary>
    public void SetData(in Shape s1, in Shape s2, Operation op = Operation.Unknown,
                        bool bTestSE = true, bool bTestSI = true)
    {
        _s1 = s1;
        _s2 = s2;
        _operation = op;
        _testSE = bTestSE;
        _testSI = bTestSI;
        _faultyShapes.Clear();
    }

    /// <summary>Performs the check.</summary>
    public void Perform(ProgressRange range = default)
    {
        // Check the incompatibility of shapes types, small edges and self-interference
        ArgumentAnalyzer analyzer = new();
        // Set the shapes and options for the check
        analyzer.SetShape1(_s1);
        analyzer.SetShape2(_s2);
        analyzer.OperationType() = _operation;
        analyzer.ArgumentTypeMode() = true;
        analyzer.SmallEdgeMode() = _testSE;
        analyzer.SelfInterMode() = _testSI;
        // Set options from BOPAlgo_Options
        analyzer.SetRunParallel(_runParallel);
        analyzer.SetFuzzyValue(_fuzzyValue);
        // Perform the check
        using var ps = new ProgressScope(range, "Checking shapes", 1);
        analyzer.Perform(ps.Next());
        if (UserBreak(ps))
        {
            return;
        }
        // Get the results
        _faultyShapes = new NCollection_List<CheckResult>(analyzer.InternalGetCheckResult());

        // Check the topological validity of the shapes
        bool isValidS1 = !_s1.IsNull ? new Analyzer(_s1).IsValid() : true;

        bool isValidS2 = !_s2.IsNull ? new Analyzer(_s2).IsValid() : true;

        if (!isValidS1 || !isValidS2)
        {
            CheckResult res = new();
            res.SetCheckStatus(CheckStatus.NotValid);
            if (!isValidS1)
            {
                res.SetShape1(_s1);
            }
            if (!isValidS2)
            {
                res.SetShape2(_s2);
            }
            _faultyShapes.Append(res);
        }
    }

    /// <summary>True if no fault was found.</summary>
    public bool IsValid => _faultyShapes.IsEmpty();

    /// <summary>The faults found (their shapes frozen as they are read).</summary>
    public ISchematiqList<CheckResult> Result() => PackedList.Own([.. _faultyShapes]);

    /// <summary>OCCT's <c>Result()</c>: the algorithm's own list, for ported code.</summary>
    internal NCollection_List<CheckResult> InternalResult() => _faultyShapes;
}
