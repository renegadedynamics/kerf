// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMesh.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_IncrementalMesh.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Meshes a shape, keeping the meshes of its faces that are already fine enough ("incremental").
/// </summary>
/// <remarks>
/// <para>
/// <b>Writing into the shape</b> (decision 489). OCCT writes the triangulations into the faces of the shape it is
/// given, and the polygons into its edges.
/// <list type="bullet">
/// <item>On a shape whose graph holds a frozen TShape (any shape a caller holds), the port meshes it in a working
/// overlay of its own (unit 10H6, <see cref="WorkingOverlay"/>; its unfrozen TShapes, a compound a caller built,
/// copied first): the triangulations and polygons go to working copies of the faces and edges, and the result holds
/// new TShapes for them and the input's own sub-shapes wherever meshing changed nothing, frozen.
/// <see cref="DiscretRoot.Shape"/> then returns the meshed result and <see cref="History"/> maps the input's sub-shapes
/// to the result's. The input is never written; meshing a shape that is already meshed finely enough gives back the
/// input itself. The overlay is the mesher's own even inside another operation's, as the copy was: meshing gives a
/// result, it doesn't write into the operation's shapes.</item>
/// <item>On an unfrozen shape (ported code's), it meshes in place, as OCCT does; <see cref="History"/> is empty.</item>
/// </list>
/// So build123d.NET's <c>new BRepMesh_IncrementalMesh(copy, tol, rel, ang, true)</c> becomes
/// <c>new BRepMesh_IncrementalMesh(shape, tol, rel, ang, true).Shape()</c>. The functional twin is <see cref="Meshed(in Shape, in ToolParameters)"/>.
/// </para>
/// <para>
/// The default <see cref="Perform(ProgressRange)"/> runs a <see cref="MeshContext"/> with the parameters'
/// algorithm (Watson by default: <c>CSF_MeshAlgo</c> isn't read, decision 492);
/// <see cref="Perform(ToolContext, ProgressRange)"/> runs any context.
/// </para>
/// <para>
/// <b>Determinism</b> (decision 490). With <c>InParallel</c>, the edges and then the faces are meshed in parallel;
/// each loop body writes only its own edge or face, so the result is the serial one bit for bit, every run.
/// </para>
/// </remarks>
public class IncrementalMesh : DiscretRoot
{
    // Default flag to control parallelization for BRepMesh_IncrementalMesh tool returned for Mesh Factory
    // (process-wide in OCCT; a setting of the operation context, with an internal setter: decisions 415, 492, unit 10I).
    internal static readonly ContextKey<bool> ParallelDefaultSetting = new("BRepMesh_IncrementalMesh.ParallelDefault", false);

    protected ToolParameters _parameters = new();
    protected bool _modified;
    protected int _status;

    // OCCT.NET: the history of a frozen shape's meshing (decision 489)
    private History _history = new();
    private bool _toCopyAlways;

    /// <summary>An algorithm without a shape.</summary>
    public IncrementalMesh()
    {
        _modified = false;
        _status = (int)StatusModel.NoError;
    }

    /// <summary>
    /// Meshes the shape with the linear deflection (relative to each edge's size if <paramref name="isRelative"/>; the
    /// faces then take their edges' maximum) and the angular deflection, in parallel if <paramref name="isInParallel"/>.
    /// </summary>
    /// <exception cref="NumericErrorException">The deflection or the angle is too small.</exception>
    public IncrementalMesh(in Shape shape,
                                    double linDeflection,
                                    bool isRelative = false,
                                    double angDeflection = 0.5,
                                    bool isInParallel = false)
    {
        _modified = false;
        _status = (int)StatusModel.NoError;
        _parameters.Deflection = linDeflection;
        _parameters.Angle = angDeflection;
        _parameters.Relative = isRelative;
        _parameters.InParallel = isInParallel;

        _shape = shape;
        Perform();
    }

    /// <summary>Meshes the shape with the parameters.</summary>
    /// <exception cref="NumericErrorException">The deflection or the angle is too small.</exception>
    public IncrementalMesh(in Shape shape,
                                    in ToolParameters parameters,
                                    ProgressRange range = default)
    {
        _parameters = parameters;
        _shape = shape;
        Perform(range);
    }

    /// <summary>Meshes the shape.</summary>
    public override void Perform(ProgressRange range = default)
    {
        MeshContext context = new(_parameters.MeshAlgo);
        Perform(context, range);
    }

    /// <summary>Meshes the shape with a custom context.</summary>
    public void Perform(ToolContext context, ProgressRange range = default)
    {
        InitParameters();

        // OCCT.NET: a shape holding frozen TShapes is meshed in a working overlay (decision 489, unit 10H6)
        Shape input = Shape();
        _history = new History();
        Dictionary<TShape, TShape>? copies = null;
        Shape target = input;
        WorkingOverlay? overlay = null;
        if (!input.IsNull && (_toCopyAlways || !WorkingOverlay.IsWritableInPlace(input)))
        {
            target = WorkingOverlay.WithUnfrozenCopied(input, out copies);
            overlay = WorkingOverlay.Begin(target);
        }
        try
        {
            Perform(context, target, overlay, copies, range, finish: true);
        }
        finally
        {
            overlay?.Dispose();
        }
    }

    /// <summary>
    /// Meshes <paramref name="shape"/> in a working overlay of its own and gives what <paramref name="read"/> reads of
    /// the meshes while the overlay is open: the public API's <c>Mesher</c> (unit 10L3, decision 1349 question 59). The
    /// overlay is dropped unfinished, so nothing is published: the shape is never written, and no meshed copy of it is
    /// assembled. <paramref name="read"/> gets the shape the meshes are on: <paramref name="shape"/> itself, or, if
    /// it holds unfrozen TShapes, a copy of it with the same structure.
    /// </summary>
    /// <param name="shape">The shape to mesh.</param>
    /// <param name="parameters">The meshing parameters.</param>
    /// <param name="read">What to read of the meshes, given the shape they are on.</param>
    /// <param name="range">The progress range.</param>
    /// <param name="status">The status flags (<see cref="StatusModel"/>) met while meshing.</param>
    /// <returns>What <paramref name="read"/> returned, or default if the meshing was stopped.</returns>
    /// <exception cref="NumericErrorException">The deflection or the angle is too small.</exception>
    internal static T? ReadMeshed<T>(in Shape shape, in ToolParameters parameters, Func<Shape, T> read, ProgressRange range,
                                     out int status)
    {
        ArgumentNullException.ThrowIfNull(read);
        IncrementalMesh algo = new();
        algo._parameters = parameters;
        algo.SetShape(shape);
        algo.InitParameters();
        Shape target = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        using WorkingOverlay overlay = WorkingOverlay.Begin(target);
        algo.Perform(new MeshContext(algo._parameters.MeshAlgo), target, overlay, copies, range, finish: false);
        status = algo._status;
        return algo.IsDone ? read(target) : default;
    }

    // Meshes target (the input, or the input with its unfrozen TShapes copied) in the overlay, if any.
    private void Perform(ToolContext context, in Shape target, WorkingOverlay? overlay,
                         Dictionary<TShape, TShape>? copies, ProgressRange range, bool finish)
    {
        context.SetShape(target);
        context.ChangeParameters() = _parameters;
        context.ChangeParameters().CleanModel = false;

        using var ps = new ProgressScope(range, "Perform incmesh", 10);
        MeshBuilder incMesh = new(context);
        incMesh.Perform(ps.Next(9));
        if (!ps.More())
        {
            _status = (int)StatusModel.UserBreak;
            return;
        }
        _status = (int)StatusModel.NoError;
        ModelModel? model = context.GetModel();
        if (model is not null)
        {
            for (int faceIt = 0; faceIt < model.FacesNb(); ++faceIt)
            {
                FaceModel dFace = model.GetFace(faceIt);
                _status |= dFace.GetStatusMask();

                for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
                {
                    WireModel dWire = dFace.GetWire(wireIt);
                    _status |= dWire.GetStatusMask();
                }
            }
        }

        // OCCT.NET: publish the meshed shape, sharing what meshing left unchanged (unit 10H6)
        if (overlay is not null && finish)
        {
            _shape = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(target), copies);
            _history = overlay.History();
            _shape.TShape()?.FreezeGraph();
        }

        ps.Next(1);
        SetDone();
    }

    /// <summary>
    /// The functional form: the shape meshed with the parameters, frozen, sharing the input's sub-shapes wherever
    /// meshing changed nothing. The input is never written, frozen or not (decision 489).
    /// </summary>
    /// <exception cref="NumericErrorException">The deflection or the angle is too small.</exception>
    public static Shape Meshed(in Shape shape, in ToolParameters parameters)
        => Meshed(shape, parameters, out _);

    /// <summary>
    /// <see cref="Meshed(in Shape, in ToolParameters)"/>, with the history from the input's vertices,
    /// edges, faces and solids to the result's.
    /// </summary>
    public static Shape Meshed(in Shape shape, in ToolParameters parameters,
                                      out History history)
    {
        IncrementalMesh algo = new();
        algo._parameters = parameters;
        algo._toCopyAlways = true;
        algo.SetShape(shape);
        algo.Perform();
        history = algo.History();
        return algo.Shape();
    }

    // accessing to parameters

    /// <summary>The meshing parameters.</summary>
    public ToolParameters Parameters() => _parameters;

    /// <summary>The meshing parameters, for writing.</summary>
    public ref ToolParameters ChangeParameters() => ref _parameters;

    /// <summary>The modified flag (never set, as in OCCT).</summary>
    public bool IsModified => _modified;

    /// <summary>The status flags (<see cref="StatusModel"/>) met while meshing.</summary>
    public int GetStatusFlags() => _status;

    /// <summary>
    /// The history of the meshed copy (OCCT.NET, decision 489): the input's vertices, edges, faces and solids modified
    /// into the result's. Empty when the shape was meshed in place, or left as it was.
    /// </summary>
    public History History() => _history;

    // Initializes specific parameters
    private void InitParameters()
    {
        if (_parameters.Deflection < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new NumericErrorException(
                "BRepMesh_IncrementalMesh::initParameters : invalid parameter value");
        }

        if (_parameters.DeflectionInterior < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _parameters.DeflectionInterior = _parameters.Deflection;
        }

        if (_parameters.MinSize < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            _parameters.MinSize =
                Max(ToolParameters.RelMinSize()
                        * Min(_parameters.Deflection, _parameters.DeflectionInterior),
                    global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        if (_parameters.Angle < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            throw new NumericErrorException(
                "BRepMesh_IncrementalMesh::initParameters : invalid parameter value");
        }

        if (_parameters.AngleInterior < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            _parameters.AngleInterior = 2.0 * _parameters.Angle;
        }
    }

    // plugin API

    /// <summary>
    /// The plugin interface of the mesh factories: an algorithm for the shape with the deflections, in parallel if
    /// <see cref="IsParallelDefault"/>, not yet performed. Returns 0 (no error).
    /// </summary>
    public static int Discret(in Shape shape,
                              double deflection,
                              double angle,
                              out DiscretRoot algo)
    {
        IncrementalMesh anAlgo = new();
        anAlgo.ChangeParameters().Deflection = deflection;
        anAlgo.ChangeParameters().Angle = angle;
        anAlgo.ChangeParameters().InParallel = ParallelDefaultSetting.Value;
        anAlgo.SetShape(shape);
        algo = anAlgo;
        return 0; // no error
    }

    /// <summary>The parallel flag that <see cref="Discret"/> sets (for the mesh factories).</summary>
    public static bool IsParallelDefault => ParallelDefaultSetting.Value;

    /// <summary>
    /// Sets the parallel flag of <see cref="Discret"/> for the rest of the current flow of execution. Internal: only
    /// Draw-lite's <c>mpparallel</c> changes it (decision 492).
    /// </summary>
    internal static void SetParallelDefault(bool inParallel) => OperationContext.Set(ParallelDefaultSetting, inParallel);
}
