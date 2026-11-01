// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcessAPI/ShapeProcessAPI_ApplySequence.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcessAPI/ShapeProcessAPI_ApplySequence.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Applies a sequence of Shape Healing operators, described in a resource file, to a shape.
/// </summary>
/// <remarks>
/// The resource file <c>rscName</c> is OCCT's own of that name, compiled in (<c>IGES</c>, <c>STEP</c>), unless the
/// caller gives the directories to read it from (the optional arguments, an addition of the port: OCCT reads the
/// directories from the environment variables <c>CSF_rscNameDefaults</c> and <c>CSF_rscNameUserDefaults</c>; decision
/// 8703). The shape is never changed (decision 1022); <see cref="PrepareShape"/>'s result and <see cref="Map"/>'s values
/// are frozen. A working object (decision 21).
/// </remarks>
public class ShapeProcessAPI_ApplySequence
{
    private ShapeProcess_ShapeContext myContext;
    private readonly string mySeq;

    /// <summary>
    /// Creates an object for the resource file <paramref name="rscName"/> and the sequence <paramref name="seqName"/>
    /// (whose operators are listed in its resource <c>seqName.exec.op</c>), and registers the operators.
    /// </summary>
    public ShapeProcessAPI_ApplySequence(string rscName, string seqName = "", string? theDefaultsDirectory = null,
                                         string? theUserDefaultsDirectory = null)
    {
        myContext = new ShapeProcess_ShapeContext(rscName, "", theDefaultsDirectory, theUserDefaultsDirectory);
        myContext.SetDetalisation(TopAbs_FACE);
        string str = seqName;

        // initialize operators
        ShapeProcess_OperLibrary.Init();

        mySeq = str;
    }

    /// <summary>The context.</summary>
    public ShapeProcess_ShapeContext Context() => myContext;

    /// <summary>Sets the context (OCCT's <c>Context()</c> returns a modifiable reference).</summary>
    public void SetContext(ShapeProcess_ShapeContext theContext)
    {
        myContext = theContext;
    }

    /// <summary>
    /// Performs the sequence on the shape (if the resource file defines it) and returns the result, frozen. The
    /// other arguments are unused, as in OCCT.
    /// </summary>
    public TopoDS_Shape PrepareShape(in TopoDS_Shape shape, bool fillmap = false, TopAbs_ShapeEnum until = TopAbs_SHAPE,
                                     Message_ProgressRange theProgress = default)
    {
        TopoDS_Shape aResult = InternalPrepareShape(shape, theProgress);
        return shape.IsNull() ? aResult : myContext.Result();
    }

    /// <summary>
    /// <see cref="PrepareShape"/> without freezing the result, for ported code (Draw-lite runs it on a copy it owns).
    /// </summary>
    internal TopoDS_Shape InternalPrepareShape(in TopoDS_Shape shape, Message_ProgressRange theProgress = default)
    {
        if (shape.IsNull())
        {
            return shape;
        }
        Resource_Manager rsc = myContext.ResourceManager()!;
        myContext.Init(shape);

        string str = mySeq;
        str += ".exec.op";
        if (rsc.Find(str))
        {
            ShapeProcess.Perform(myContext, mySeq, theProgress);
        }

        return myContext.InternalResult();
    }

    /// <summary>Clears the history map (OCCT: does nothing).</summary>
    public void ClearMap()
    {
        // myContext->Map().Clear();
    }

    /// <summary>The history: each sub-shape modified, with what it became. Its values are frozen.</summary>
    public NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Map() => myContext.Map();

    /// <summary>Prints statistics on the history (shells and faces mapped, the preparation ratio) to the messenger.</summary>
    public void PrintPreparationResult()
    {
        ShapeProcess_ShapeContext.PrintPreparation(myContext.InternalMap(), myContext.Messenger());
    }
}
