// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BooleanOperation.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BooleanOperation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// The root of the Boolean operations on arbitrary shapes: the arguments are in two groups, objects and tools, and
/// the operation is FUSE (union), COMMON (intersection), CUT (subtraction) or SECTION.
/// </summary>
/// <remarks>
/// <para>
/// The rules: for FUSE all arguments have the same dimension; for CUT the tools' smallest dimension is at least the
/// objects' largest; COMMON and SECTION take any dimensions. Errors besides the base class's:
/// <see cref="AlertBopNotSet"/> when the operation isn't set.
/// </para>
/// <para>
/// Left out: OCCT's debug dump of the arguments (<c>BRepAlgoAPI_DumpOper</c>, enabled by the CSF_DEBUG_BOP environment
/// variable), which writes files: a result never depends on the environment.
/// </para>
/// </remarks>
public class BooleanOperation : BuilderAlgo
{
    protected NCollection_List<Shape> _tools = new(); //!< Tool arguments of operation
    protected Operation _operation;                  //!< Type of Boolean Operation

    /// <summary>An empty operation.</summary>
    public BooleanOperation()
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Unknown;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="pf"/>.</summary>
    public BooleanOperation(PaveFiller pf)
        : base(pf)
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Unknown;
    }

    /// <summary>The operation on two arguments (obsolete in OCCT).</summary>
    protected BooleanOperation(in Shape s1, in Shape s2, Operation op)
    {
        _operation = op;
        _arguments.Append(s1);
        _tools.Append(s2);
    }

    /// <summary>The operation on two arguments with intersections already computed (obsolete in OCCT).</summary>
    protected BooleanOperation(in Shape s1, in Shape s2, PaveFiller pf,
                                           Operation op)
        : base(pf)
    {
        _operation = op;
        _arguments.Append(s1);
        _tools.Append(s2);
    }

    /// <summary>The first object (obsolete in OCCT).</summary>
    public Shape Shape1() => _arguments.First();

    /// <summary>The first tool (obsolete in OCCT).</summary>
    public Shape Shape2() => _tools.First();

    /// <summary>Sets the tools (copies the list).</summary>
    public void SetTools(NCollection_List<Shape> ls) => _tools = new NCollection_List<Shape>(ls);

    /// <summary>The tools.</summary>
    public NCollection_List<Shape> Tools() => _tools;

    /// <summary>Sets the type of the operation.</summary>
    public void SetOperation(Operation bop) => _operation = bop;

    /// <summary>The type of the operation.</summary>
    public Operation Operation() => _operation;

    /// <summary>Performs the Boolean operation.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Set Not Done status by default
        NotDone();
        // Clear from previous runs
        Clear();
        // Check for availability of arguments and tools
        // Both should be present
        if (_arguments.IsEmpty() || _tools.IsEmpty())
        {
            AddError(new AlertTooFewArguments());
            return;
        }
        // Check if the operation is set
        if (_operation == global::Kerf.Engine.Booleans.Operation.Unknown)
        {
            AddError(new AlertBopNotSet());
            return;
        }

        // (OCCT's CSF_DEBUG_BOP dump of the arguments is left out.)

        string psName;
        switch (_operation)
        {
            case global::Kerf.Engine.Booleans.Operation.Common:
                psName = "Performing COMMON operation";
                break;
            case global::Kerf.Engine.Booleans.Operation.Fuse:
                psName = "Performing FUSE operation";
                break;
            case global::Kerf.Engine.Booleans.Operation.Cut:
            case global::Kerf.Engine.Booleans.Operation.CUT21:
                psName = "Performing CUT operation";
                break;
            case global::Kerf.Engine.Booleans.Operation.Section:
                psName = "Performing SECTION operation";
                break;
            default:
                return;
        }

        using var ps = new ProgressScope(range, psName, _isIntersectionNeeded ? 100 : 30);
        // If necessary perform intersection of the argument shapes
        if (_isIntersectionNeeded)
        {
            // Combine Objects and Tools into a single list for intersection
            NCollection_List<Shape> lArgs = new(_arguments);
            for (NCollection_List<Shape>.Iterator it = new(_tools); it.More(); it.Next())
            {
                lArgs.Append(it.Value());
            }
            // Perform intersection
            IntersectShapes(lArgs, ps.Next(70));
            if (HasErrors)
            {
                return;
            }
        }
        // Builder Initialization
        if (_operation == global::Kerf.Engine.Booleans.Operation.Section)
        {
            _builder = new Section();
            _builder.SetArguments(_dsFiller!.Arguments());
        }
        else
        {
            Bop bop = new();
            _builder = bop;
            _builder.SetArguments(_arguments);
            bop.SetTools(_tools);
            bop.SetOperation(_operation);
        }
        // Build the result
        BuildResult(ps.Next(30));
    }
}
