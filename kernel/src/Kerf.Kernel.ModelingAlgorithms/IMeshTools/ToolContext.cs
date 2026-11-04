// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Occt;

namespace Kerf.Engine.Meshing.Tools;

/// <summary>
/// The context of the mesher: the discrete model and the tools that build and process it, with the meshing parameters.
/// </summary>
internal class ToolContext : ShapeModel
{
    private ModelBuilder? _modelBuilder;
    private ModelModel? _model;
    private ModelAlgo? _edgeDiscret;
    private ModelAlgo? _modelHealer;
    private ModelAlgo? _preProcessor;
    private ModelAlgo? _faceDiscret;
    private ModelAlgo? _postProcessor;
    private ToolParameters _parameters = new();

    /// <summary>A context without tools.</summary>
    public ToolContext()
    {
    }

    /// <summary>Builds the model with the model builder; true on success.</summary>
    public virtual bool BuildModel()
    {
        if (_modelBuilder is null)
        {
            return false;
        }

        _model = _modelBuilder.Perform(GetShape(), _parameters);

        return _model is not null;
    }

    /// <summary>Discretizes the model's edges with the edge discretizer; true on success.</summary>
    public virtual bool DiscretizeEdges()
    {
        if (_model is null || _edgeDiscret is null)
        {
            return false;
        }

        // Discretize edges of a model.
        return _edgeDiscret.Perform(_model, _parameters, new ProgressRange());
    }

    /// <summary>Heals the model built by <see cref="DiscretizeEdges"/> with the healer, if any; true on success.</summary>
    public virtual bool HealModel()
    {
        if (_model is null)
        {
            return false;
        }

        return _modelHealer is null
                 ? true
                 : _modelHealer.Perform(_model, _parameters, new ProgressRange());
    }

    /// <summary>
    /// Pre-processes the model with the pre-processor, if any (auxiliary work such as cleaning old triangulations from
    /// the shape); true on success.
    /// </summary>
    public virtual bool PreProcessModel()
    {
        if (_model is null)
        {
            return false;
        }

        return _preProcessor is null
                 ? true
                 : _preProcessor.Perform(_model, _parameters, new ProgressRange());
    }

    /// <summary>Meshes the model's faces with the face discretizer; true on success.</summary>
    public virtual bool DiscretizeFaces(ProgressRange range)
    {
        if (_model is null || _faceDiscret is null)
        {
            return false;
        }

        // Discretize faces of a model.
        return _faceDiscret.Perform(_model, _parameters, range);
    }

    /// <summary>Post-processes the model with the post-processor, if any; true on success.</summary>
    public virtual bool PostProcessModel()
    {
        if (_model is null)
        {
            return false;
        }

        return _postProcessor is null
                 ? true
                 : _postProcessor.Perform(_model, _parameters, new ProgressRange());
    }

    /// <summary>Cleans the temporary data: the model, if the parameters ask for it.</summary>
    public virtual void Clean()
    {
        if (_parameters.CleanModel)
        {
            _model = null;
        }
    }

    /// <summary>The tool building the discrete model.</summary>
    public ModelBuilder? GetModelBuilder() => _modelBuilder;

    /// <summary>Sets the tool building the discrete model.</summary>
    public void SetModelBuilder(ModelBuilder? builder) => _modelBuilder = builder;

    /// <summary>The tool discretizing the model's edges.</summary>
    public ModelAlgo? GetEdgeDiscret() => _edgeDiscret;

    /// <summary>Sets the tool discretizing the model's edges.</summary>
    public void SetEdgeDiscret(ModelAlgo? edgeDiscret) => _edgeDiscret = edgeDiscret;

    /// <summary>The tool healing the model.</summary>
    public ModelAlgo? GetModelHealer() => _modelHealer;

    /// <summary>Sets the tool healing the model.</summary>
    public void SetModelHealer(ModelAlgo? modelHealer) => _modelHealer = modelHealer;

    /// <summary>The pre-processing algorithm.</summary>
    public ModelAlgo? GetPreProcessor() => _preProcessor;

    /// <summary>Sets the pre-processing algorithm.</summary>
    public void SetPreProcessor(ModelAlgo? preProcessor) => _preProcessor = preProcessor;

    /// <summary>The meshing algorithm.</summary>
    public ModelAlgo? GetFaceDiscret() => _faceDiscret;

    /// <summary>Sets the meshing algorithm.</summary>
    public void SetFaceDiscret(ModelAlgo? faceDiscret) => _faceDiscret = faceDiscret;

    /// <summary>The post-processing algorithm.</summary>
    public ModelAlgo? GetPostProcessor() => _postProcessor;

    /// <summary>Sets the post-processing algorithm.</summary>
    public void SetPostProcessor(ModelAlgo? postProcessor) => _postProcessor = postProcessor;

    /// <summary>The meshing parameters.</summary>
    public ToolParameters GetParameters() => _parameters;

    /// <summary>The meshing parameters, for writing.</summary>
    public ref ToolParameters ChangeParameters() => ref _parameters;

    /// <summary>The discrete model of the shape.</summary>
    public ModelModel? GetModel() => _model;
}
