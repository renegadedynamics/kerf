// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcessAPI/ShapeProcessAPI_ApplySequence.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcessAPI/ShapeProcessAPI_ApplySequence.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Process;

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
public class ApplySequence
{
    private ShapeContext _context;
    private readonly string _seq;

    /// <summary>
    /// Creates an object for the resource file <paramref name="rscName"/> and the sequence <paramref name="seqName"/>
    /// (whose operators are listed in its resource <c>seqName.exec.op</c>), and registers the operators.
    /// </summary>
    public ApplySequence(string rscName, string seqName = "", string? defaultsDirectory = null,
                                         string? userDefaultsDirectory = null)
    {
        _context = new ShapeContext(rscName, "", defaultsDirectory, userDefaultsDirectory);
        _context.SetDetalisation(ShapeType.Face);
        string str = seqName;

        // initialize operators
        OperLibrary.Init();

        _seq = str;
    }

    /// <summary>The context.</summary>
    public ShapeContext Context() => _context;

    /// <summary>Sets the context (OCCT's <c>Context()</c> returns a modifiable reference).</summary>
    public void SetContext(ShapeContext context)
    {
        _context = context;
    }

    /// <summary>
    /// Performs the sequence on the shape (if the resource file defines it) and returns the result, frozen. The
    /// other arguments are unused, as in OCCT.
    /// </summary>
    public Shape PrepareShape(in Shape shape, bool fillmap = false, ShapeType until = ShapeType.Shape,
                                     ProgressRange progress = default)
    {
        Shape result = InternalPrepareShape(shape, progress);
        return shape.IsNull ? result : _context.Result();
    }

    /// <summary>
    /// <see cref="PrepareShape"/> without freezing the result, for ported code (Draw-lite runs it on a copy it owns).
    /// </summary>
    internal Shape InternalPrepareShape(in Shape shape, ProgressRange progress = default)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        Manager rsc = _context.ResourceManager()!;
        _context.Init(shape);

        string str = _seq;
        str += ".exec.op";
        if (rsc.Find(str))
        {
            global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(_context, _seq, progress);
        }

        return _context.InternalResult();
    }

    /// <summary>Clears the history map (OCCT: does nothing).</summary>
    public void ClearMap()
    {
        // myContext->Map().Clear();
    }

    /// <summary>The history: each sub-shape modified, with what it became. Its values are frozen.</summary>
    public Dictionary<Shape, Shape> Map() => _context.Map();

    /// <summary>Prints statistics on the history (shells and faces mapped, the preparation ratio) to the messenger.</summary>
    public void PrintPreparationResult()
    {
        ShapeContext.PrintPreparation(_context.InternalMap(), _context.Messenger());
    }
}
