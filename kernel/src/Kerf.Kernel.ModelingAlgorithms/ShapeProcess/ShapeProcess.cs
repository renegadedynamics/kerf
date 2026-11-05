// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Collections.Frozen;
using System.Collections.Immutable;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Healing.Process;

/// <summary>
/// Shape processing: the process-wide registry of operators, and the performer that runs a sequence of operators,
/// named in a resource file or given as flags, on a context.
/// </summary>
/// <remarks>
/// <para>
/// The registry (plan 2.3, decision 1021; unit 10I) has two layers. The defaults are Shape Healing's own operators,
/// which <see cref="OperLibrary.Init"/> publishes once per process, immutable from then on. An operator a caller
/// registers (<see cref="RegisterOperator"/>) is added to the current flow of execution, a setting of the
/// <see cref="OperationContext"/>: the operations of that flow (and the tasks it starts) see it; others don't. A name
/// can't be registered twice, as in OCCT. Neither layer takes a lock: OCCT's process-wide map had none either, and the
/// port's had one.
/// </para>
/// <para>
/// A <see cref="ShapeContext"/> whose shape holds frozen shapes runs the whole sequence on an exact copy
/// and gives back the caller's own sub-shapes where nothing changed (decision 1022): the operators write into the
/// shape they work on, as OCCT's do.
/// </para>
/// </remarks>
internal static class ShapeProcess
{
    /// <summary>Describes all available operations.</summary>
    public enum Operation : byte
    {
        First = 0, // First operation index.
        DirectFaces = First,
        SameParameter,
        SetTolerance,
        SplitAngle,
        BSplineRestriction,
        ElementaryToRevolution,
        SweptToElementary,
        SurfaceToBSpline,
        ToBezier,
        SplitContinuity,
        SplitClosedFaces,
        FixWireGaps,
        FixFaceSize,
        DropSmallSolids,
        DropSmallEdges,
        FixShape,
        SplitClosedEdges,
        SplitCommonVertex,
        Last = SplitCommonVertex, // Last operation index.
    }

    /// <summary>
    /// The set of operations to perform (OCCT's <c>std::bitset&lt;Operation::Last + 1&gt;</c>): a value; <see cref="With"/>
    /// gives a set with one more operation.
    /// </summary>
    public readonly record struct OperationsFlags
    {
        private readonly uint _bits;

        private OperationsFlags(uint bits) => _bits = bits;

        /// <summary>The set of the given operations.</summary>
        public OperationsFlags(params Operation[] operations)
        {
            foreach (Operation operation in operations)
            {
                _bits |= 1u << (int)operation;
            }
        }

        /// <summary>The set with <paramref name="operation"/> added (bitset's <c>set</c>).</summary>
        public OperationsFlags With(Operation operation) => new(_bits | (1u << (int)operation));

        /// <summary>True if the set holds the operation (bitset's <c>test</c>).</summary>
        public bool Test(Operation operation) => (_bits & (1u << (int)operation)) != 0;

        /// <summary>The number of operations in the set (bitset's <c>count</c>).</summary>
        public int Count() => System.Numerics.BitOperations.PopCount(_bits);
    }

    // The defaults: Shape Healing's operators, published once by OperLibrary.Init (null before)
    private static volatile FrozenDictionary<string, Operator>? s_defaultOperators;

    /// <summary>The operators registered in the current flow of execution, over the defaults.</summary>
    internal static readonly ContextKey<ImmutableDictionary<string, Operator>> OperatorsSetting =
        new("ShapeProcess.Operators", ImmutableDictionary.Create<string, Operator>(StringComparer.Ordinal));

    /// <summary>Publishes the default operators, once (<see cref="OperLibrary.Init"/>'s one-time initialisation).</summary>
    internal static void SetDefaultOperators(FrozenDictionary<string, Operator> operators) =>
        Interlocked.CompareExchange(ref s_defaultOperators, operators, null);

    // Simple RAII class to lock the scope of the current operation (OCCT's ScopeLock).
    private readonly struct ScopeLock : IDisposable
    {
        private readonly ProcessContext _context;

        public ScopeLock(ProcessContext context, string scopeName)
        {
            _context = context;
            _context.SetScope(scopeName);
        }

        public void Dispose() => _context.UnSetScope();
    }

    /// <summary>
    /// Registers an operator to make it visible for the performer in the current flow of execution (see the remarks);
    /// false if the name is taken.
    /// </summary>
    public static bool RegisterOperator(string name, Operator op)
    {
        if (FindOperator(name, out _))
        {
            // OCCT_DEBUG: a warning that the operator is already registered, left out
            return false;
        }
        OperationContext.Set(OperatorsSetting, OperatorsSetting.Value.Add(name, op));
        return true;
    }

    /// <summary>Finds an operator by its name: the current flow's, else Shape Healing's own.</summary>
    public static bool FindOperator(string name, out Operator? op)
    {
        if (OperatorsSetting.Value.TryGetValue(name, out op)
            || (s_defaultOperators is { } defaults && defaults.TryGetValue(name, out op)))
        {
            return op is not null;
        }
        // OCCT_DEBUG: an error that no operator has the name, left out
        return false;
    }

    /// <summary>
    /// Performs the sequence of operators <paramref name="seq"/> on the context: its resource file (and other data)
    /// must be loaded already, including the description of the sequence (<c>seq.exec.op</c>). True if an operator
    /// succeeded.
    /// </summary>
    public static bool Perform(ProcessContext context, string seq, ProgressRange progress = default)
    {
        using var sequenceScope = new ScopeLock(context, seq);

        // get description of the sequence
        string sequence = "";
        if (!context.GetString("exec.op", ref sequence))
        {
            // OCCT_DEBUG: an error message to std::cout, left out
            if (context.TraceLevel() > 0)
            {
                var smsg3 = new Msg("SP.Sequence.Warn.NoSeq"); // Sequence %s not found
                context.Messenger().Send(smsg3.Arg(seq).Get(), Warning);
            }
            return false;
        }
        var sequenceOfOperators = new NCollection_Sequence<string>();
        string oper;
        int i;
        for (i = 1; ; i++)
        {
            oper = TCollection_AsciiString.Token(sequence, " \t,;", i);
            if (oper.Length <= 0)
            {
                break;
            }
            sequenceOfOperators.Append(oper);
        }

        // put a message
        if (context.TraceLevel() >= 2)
        {
            var smsg0 = new Msg("SP.Sequence.Info.Seq"); // Sequence of operators: %s
            string Seq = "";
            for (int i1 = 1; i1 <= sequenceOfOperators.Length(); i1++)
            {
                if (i1 > 1)
                {
                    Seq += ",";
                }
                Seq += sequenceOfOperators.Value(i1);
            }
            smsg0.Arg(Seq);
            context.Messenger().Send(smsg0.Get(), Info);
        }

        // iterate on operators in the sequence
        bool isDone = false;
        var shapeContext = context as ShapeContext;
        shapeContext?.InternalBeginWrite();
        try
        {
            using var ps = new ProgressScope(progress, null, sequenceOfOperators.Length());
            for (i = 1; i <= sequenceOfOperators.Length() && ps.More(); i++)
            {
                oper = sequenceOfOperators.Value(i);
                ProgressRange range = ps.Next();

                if (context.TraceLevel() >= 2)
                {
                    var smsg5 = new Msg("SP.Sequence.Info.Operator"); // Operator %d/%d: %s
                    smsg5.Arg(i).Arg(sequenceOfOperators.Length()).Arg(oper);
                    context.Messenger().Send(smsg5.Get(), Alarm);
                }

                if (!FindOperator(oper, out Operator? op))
                {
                    if (context.TraceLevel() > 0)
                    {
                        var smsg1 = new Msg("SP.Sequence.Error.NoOp"); // Operator %s not found
                        context.Messenger().Send(smsg1.Arg(oper).Get(), Alarm);
                    }
                    continue;
                }

                using var operationScope = new ScopeLock(context, oper);
                try
                {
                    if (op!.Perform(context, range))
                    {
                        isDone = true;
                    }
                }
                catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                {
                    var smsg2 = new Msg("SP.Sequence.Error.Except"); // Operator %s failed with exception %s
                    smsg2.Arg(oper).Arg(What(exception));
                    context.Messenger().Send(smsg2.Get(), Alarm);
                }
            }
        }
        finally
        {
            shapeContext?.InternalEndWrite();
        }

        return isDone;
    }

    /// <summary>
    /// Performs the operations <paramref name="operations"/> on the context, in the order of
    /// <see cref="Operation"/>. Their parameters are read in the operation's scope ("FixShape.Tolerance3d", not
    /// "FromSTEP.FixShape.Tolerance3d"); unset parameters take their defaults. True if at least one operation was
    /// performed.
    /// </summary>
    public static bool Perform(ProcessContext? context, OperationsFlags operations,
                               ProgressRange progress = default)
    {
        if (context is null)
        {
            return false;
        }

        List<(string, Operator)> operators = GetOperators(operations);
        if (operators.Count == 0)
        {
            return false;
        }

        bool isAnySuccess = false;
        var shapeContext = context as ShapeContext;
        shapeContext?.InternalBeginWrite();
        try
        {
            using var progressScope = new ProgressScope(progress, null, operators.Count);
            foreach ((string operationName, Operator operation) in operators)
            {
                ProgressRange progressRange = progressScope.Next();
                using var operationScope = new ScopeLock(context, operationName); // Set operation scope.
                try
                {
                    isAnySuccess |= operation.Perform(context, progressRange);
                }
                catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                {
                    var message = new Msg("SP.Sequence.Error.Except"); // Operator %s failed with exception %s
                    message.Arg(operationName).Arg(What(exception));
                    context.Messenger().Send(message.Get(), Alarm);
                }
            }
        }
        finally
        {
            shapeContext?.InternalEndWrite();
        }
        return isAnySuccess;
    }

    // OCCT's Standard_Failure::what(): the message, empty when there is none. A NullReferenceException is natively an
    // access violation, which OCC_CATCH_SIGNALS turns into OSD_Exception_ACCESS_VIOLATION with Windows' text for a read
    // at address 0 (decisions 977, 1024)
    private static string What(Exception exception)
    {
        if (exception is NullReferenceException)
        {
            return "ACCESS VIOLATION at address 0x0000000000000000 during 'READ' operation";
        }
        string message = exception.Message;
        return message == $"Exception of type '{exception.GetType().FullName}' was thrown." ? "" : message;
    }

    /// <summary>The operation named <paramref name="name"/>, and true; (First, false) if no operation has the name.</summary>
    public static (Operation, bool) ToOperationFlag(string? name)
    {
        if (name is null)
        {
            return (Operation.First, false);
        }

        for (Operation operation = Operation.First; operation <= Operation.Last; ++operation)
        {
            string? operationName = ToOperationName(operation);
            if (operationName is not null && name == operationName)
            {
                return (operation, true);
            }
        }
        return (Operation.First, false);
    }

    // The registered operators of the flags, with their names, in the order of Operation.
    private static List<(string, Operator)> GetOperators(OperationsFlags flags)
    {
        var result = new List<(string, Operator)>();
        for (Operation operation = Operation.First; operation <= Operation.Last; ++operation)
        {
            if (flags.Test(operation))
            {
                string? operationName = ToOperationName(operation);
                if (operationName is null)
                {
                    continue;
                }
                if (FindOperator(operationName, out Operator? anOperator))
                {
                    result.Add((operationName, anOperator!));
                }
            }
        }
        return result;
    }

    private static string? ToOperationName(Operation operation) => operation switch
    {
        Operation.DirectFaces => "DirectFaces",
        Operation.SameParameter => "SameParameter",
        Operation.SetTolerance => "SetTolerance",
        Operation.SplitAngle => "SplitAngle",
        Operation.BSplineRestriction => "BSplineRestriction",
        Operation.ElementaryToRevolution => "ElementaryToRevolution",
        Operation.SweptToElementary => "SweptToElementary",
        Operation.SurfaceToBSpline => "SurfaceToBSpline",
        Operation.ToBezier => "ToBezier",
        Operation.SplitContinuity => "SplitContinuity",
        Operation.SplitClosedFaces => "SplitClosedFaces",
        Operation.FixWireGaps => "FixWireGaps",
        Operation.FixFaceSize => "FixFaceSize",
        Operation.DropSmallSolids => "DropSmallSolids",
        Operation.DropSmallEdges => "DropSmallEdges",
        Operation.FixShape => "FixShape",
        Operation.SplitClosedEdges => "SplitClosedEdges",
        Operation.SplitCommonVertex => "SplitCommonVertex",
        _ => null,
    };
}
