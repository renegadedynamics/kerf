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
/// <item>On a shape whose graph holds a frozen TShape (any shape a caller holds), the port meshes
/// <c>BRepTools.ExactCopyForWrite</c> of it; then <c>BRepTools.KeepUnchanged</c> gives back the input's own sub-shapes
/// wherever meshing changed nothing (decisions 416, 468), and the result is frozen. <see cref="DiscretRoot.Shape"/>
/// then returns the meshed result and <see cref="History"/> maps the input's sub-shapes to the result's. The input is
/// never written; meshing a shape that is already meshed finely enough gives back the input itself.</item>
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
    // (process-wide, with an internal setter: decisions 415, 492).
    private static volatile bool s_isInParallel = false;

    protected ToolParameters _parameters = new();
    protected bool _modified;
    protected int _status;

    // OCCT.NET: the copy for write (decision 489)
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

        // OCCT.NET: a shape holding frozen TShapes is meshed on an exact copy (decision 489)
        Shape input = Shape();
        bool isCopy = !input.IsNull && (_toCopyAlways || BrepTools.HoldsFrozen(input));
        Shape target = isCopy ? BrepTools.ExactCopyForWrite(input, out _) : input;
        _history = new History();

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

        // OCCT.NET: publish the meshed copy, sharing what meshing left unchanged
        if (isCopy)
        {
            _shape = BrepTools.KeepUnchanged(input, target, out _history);
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
        anAlgo.ChangeParameters().InParallel = s_isInParallel;
        anAlgo.SetShape(shape);
        algo = anAlgo;
        return 0; // no error
    }

    /// <summary>The parallel flag that <see cref="Discret"/> sets (for the mesh factories).</summary>
    public static bool IsParallelDefault => s_isInParallel;

    /// <summary>
    /// Sets the parallel flag of <see cref="Discret"/>. Internal: a process-wide setting that only Draw-lite's
    /// <c>mpparallel</c> changes (decision 492).
    /// </summary>
    internal static void SetParallelDefault(bool inParallel) => s_isInParallel = inParallel;
}
