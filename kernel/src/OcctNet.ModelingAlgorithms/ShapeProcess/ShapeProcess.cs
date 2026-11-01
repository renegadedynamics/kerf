// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// Shape processing: the process-wide registry of operators, and the performer that runs a sequence of operators,
/// named in a resource file or given as flags, on a context.
/// </summary>
/// <remarks>
/// <para>
/// The registry is process-wide state with OCCT's semantics (plan 2.3, decision 1021): an operator registered once is
/// visible to every thread, and a name can't be registered twice. OCCT's map has no lock; the port's reads and writes
/// take one, so that registering while another thread performs can't corrupt it.
/// </para>
/// <para>
/// A <see cref="ShapeProcess_ShapeContext"/> whose shape holds frozen shapes runs the whole sequence on an exact copy
/// and gives back the caller's own sub-shapes where nothing changed (decision 1022): the operators write into the
/// shape they work on, as OCCT's do.
/// </para>
/// </remarks>
public static class ShapeProcess
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
        private readonly uint myBits;

        private OperationsFlags(uint theBits) => myBits = theBits;

        /// <summary>The set of the given operations.</summary>
        public OperationsFlags(params Operation[] theOperations)
        {
            foreach (Operation anOperation in theOperations)
            {
                myBits |= 1u << (int)anOperation;
            }
        }

        /// <summary>The set with <paramref name="theOperation"/> added (bitset's <c>set</c>).</summary>
        public OperationsFlags With(Operation theOperation) => new(myBits | (1u << (int)theOperation));

        /// <summary>True if the set holds the operation (bitset's <c>test</c>).</summary>
        public bool Test(Operation theOperation) => (myBits & (1u << (int)theOperation)) != 0;

        /// <summary>The number of operations in the set (bitset's <c>count</c>).</summary>
        public int Count() => System.Numerics.BitOperations.PopCount(myBits);
    }

    private static readonly Dictionary<string, ShapeProcess_Operator> aMapOfOperators = new(StringComparer.Ordinal);
    private static readonly Lock aMapOfOperatorsLock = new();

    // Simple RAII class to lock the scope of the current operation (OCCT's ScopeLock).
    private readonly struct ScopeLock : IDisposable
    {
        private readonly ShapeProcess_Context myContext;

        public ScopeLock(ShapeProcess_Context theContext, string theScopeName)
        {
            myContext = theContext;
            myContext.SetScope(theScopeName);
        }

        public void Dispose() => myContext.UnSetScope();
    }

    /// <summary>Registers an operator to make it visible for the performer; false if the name is taken.</summary>
    public static bool RegisterOperator(string name, ShapeProcess_Operator op)
    {
        lock (aMapOfOperatorsLock)
        {
            if (aMapOfOperators.ContainsKey(name))
            {
                // OCCT_DEBUG: a warning that the operator is already registered, left out
                return false;
            }
            aMapOfOperators.Add(name, op);
            return true;
        }
    }

    /// <summary>Finds an operator by its name.</summary>
    public static bool FindOperator(string name, out ShapeProcess_Operator? op)
    {
        lock (aMapOfOperatorsLock)
        {
            if (!aMapOfOperators.TryGetValue(name, out op))
            {
                // OCCT_DEBUG: an error that no operator has the name, left out
                return false;
            }
            return op is not null;
        }
    }

    /// <summary>
    /// Performs the sequence of operators <paramref name="seq"/> on the context: its resource file (and other data)
    /// must be loaded already, including the description of the sequence (<c>seq.exec.op</c>). True if an operator
    /// succeeded.
    /// </summary>
    public static bool Perform(ShapeProcess_Context context, string seq, Message_ProgressRange theProgress = default)
    {
        using var aSequenceScope = new ScopeLock(context, seq);

        // get description of the sequence
        string sequence = "";
        if (!context.GetString("exec.op", ref sequence))
        {
            // OCCT_DEBUG: an error message to std::cout, left out
            if (context.TraceLevel() > 0)
            {
                var SMSG3 = new Message_Msg("SP.Sequence.Warn.NoSeq"); // Sequence %s not found
                context.Messenger().Send(SMSG3.Arg(seq).Get(), Message_Warning);
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
            var SMSG0 = new Message_Msg("SP.Sequence.Info.Seq"); // Sequence of operators: %s
            string Seq = "";
            for (int i1 = 1; i1 <= sequenceOfOperators.Length(); i1++)
            {
                if (i1 > 1)
                {
                    Seq += ",";
                }
                Seq += sequenceOfOperators.Value(i1);
            }
            SMSG0.Arg(Seq);
            context.Messenger().Send(SMSG0.Get(), Message_Info);
        }

        // iterate on operators in the sequence
        bool isDone = false;
        var aShapeContext = context as ShapeProcess_ShapeContext;
        aShapeContext?.InternalBeginWrite();
        try
        {
            using var aPS = new Message_ProgressScope(theProgress, null, sequenceOfOperators.Length());
            for (i = 1; i <= sequenceOfOperators.Length() && aPS.More(); i++)
            {
                oper = sequenceOfOperators.Value(i);
                Message_ProgressRange aRange = aPS.Next();

                if (context.TraceLevel() >= 2)
                {
                    var SMSG5 = new Message_Msg("SP.Sequence.Info.Operator"); // Operator %d/%d: %s
                    SMSG5.Arg(i).Arg(sequenceOfOperators.Length()).Arg(oper);
                    context.Messenger().Send(SMSG5.Get(), Message_Alarm);
                }

                if (!FindOperator(oper, out ShapeProcess_Operator? op))
                {
                    if (context.TraceLevel() > 0)
                    {
                        var SMSG1 = new Message_Msg("SP.Sequence.Error.NoOp"); // Operator %s not found
                        context.Messenger().Send(SMSG1.Arg(oper).Get(), Message_Alarm);
                    }
                    continue;
                }

                using var anOperationScope = new ScopeLock(context, oper);
                try
                {
                    if (op!.Perform(context, aRange))
                    {
                        isDone = true;
                    }
                }
                catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                {
                    var SMSG2 = new Message_Msg("SP.Sequence.Error.Except"); // Operator %s failed with exception %s
                    SMSG2.Arg(oper).Arg(What(anException));
                    context.Messenger().Send(SMSG2.Get(), Message_Alarm);
                }
            }
        }
        finally
        {
            aShapeContext?.InternalEndWrite();
        }

        return isDone;
    }

    /// <summary>
    /// Performs the operations <paramref name="theOperations"/> on the context, in the order of
    /// <see cref="Operation"/>. Their parameters are read in the operation's scope ("FixShape.Tolerance3d", not
    /// "FromSTEP.FixShape.Tolerance3d"); unset parameters take their defaults. True if at least one operation was
    /// performed.
    /// </summary>
    public static bool Perform(ShapeProcess_Context? theContext, OperationsFlags theOperations,
                               Message_ProgressRange theProgress = default)
    {
        if (theContext is null)
        {
            return false;
        }

        List<(string, ShapeProcess_Operator)> anOperators = getOperators(theOperations);
        if (anOperators.Count == 0)
        {
            return false;
        }

        bool anIsAnySuccess = false;
        var aShapeContext = theContext as ShapeProcess_ShapeContext;
        aShapeContext?.InternalBeginWrite();
        try
        {
            using var aProgressScope = new Message_ProgressScope(theProgress, null, anOperators.Count);
            foreach ((string anOperationName, ShapeProcess_Operator anOperation) in anOperators)
            {
                Message_ProgressRange aProgressRange = aProgressScope.Next();
                using var anOperationScope = new ScopeLock(theContext, anOperationName); // Set operation scope.
                try
                {
                    anIsAnySuccess |= anOperation.Perform(theContext, aProgressRange);
                }
                catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                {
                    var aMessage = new Message_Msg("SP.Sequence.Error.Except"); // Operator %s failed with exception %s
                    aMessage.Arg(anOperationName).Arg(What(anException));
                    theContext.Messenger().Send(aMessage.Get(), Message_Alarm);
                }
            }
        }
        finally
        {
            aShapeContext?.InternalEndWrite();
        }
        return anIsAnySuccess;
    }

    // OCCT's Standard_Failure::what(): the message, empty when there is none. A NullReferenceException is natively an
    // access violation, which OCC_CATCH_SIGNALS turns into OSD_Exception_ACCESS_VIOLATION with Windows' text for a read
    // at address 0 (decisions 977, 1024)
    private static string What(Exception theException)
    {
        if (theException is NullReferenceException)
        {
            return "ACCESS VIOLATION at address 0x0000000000000000 during 'READ' operation";
        }
        string aMessage = theException.Message;
        return aMessage == $"Exception of type '{theException.GetType().FullName}' was thrown." ? "" : aMessage;
    }

    /// <summary>The operation named <paramref name="theName"/>, and true; (First, false) if no operation has the name.</summary>
    public static (Operation, bool) ToOperationFlag(string? theName)
    {
        if (theName is null)
        {
            return (Operation.First, false);
        }

        for (Operation anOperation = Operation.First; anOperation <= Operation.Last; ++anOperation)
        {
            string? anOperationName = toOperationName(anOperation);
            if (anOperationName is not null && theName == anOperationName)
            {
                return (anOperation, true);
            }
        }
        return (Operation.First, false);
    }

    // The registered operators of the flags, with their names, in the order of Operation.
    private static List<(string, ShapeProcess_Operator)> getOperators(OperationsFlags theFlags)
    {
        var aResult = new List<(string, ShapeProcess_Operator)>();
        for (Operation anOperation = Operation.First; anOperation <= Operation.Last; ++anOperation)
        {
            if (theFlags.Test(anOperation))
            {
                string? anOperationName = toOperationName(anOperation);
                if (anOperationName is null)
                {
                    continue;
                }
                if (FindOperator(anOperationName, out ShapeProcess_Operator? anOperator))
                {
                    aResult.Add((anOperationName, anOperator!));
                }
            }
        }
        return aResult;
    }

    private static string? toOperationName(Operation theOperation) => theOperation switch
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
