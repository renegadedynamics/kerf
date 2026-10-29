// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_Context.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The context of the mesher: the discrete model and the tools that build and process it, with the meshing parameters.
/// </summary>
public class IMeshTools_Context : IMeshData_Shape
{
    private IMeshTools_ModelBuilder? myModelBuilder;
    private IMeshData_Model? myModel;
    private IMeshTools_ModelAlgo? myEdgeDiscret;
    private IMeshTools_ModelAlgo? myModelHealer;
    private IMeshTools_ModelAlgo? myPreProcessor;
    private IMeshTools_ModelAlgo? myFaceDiscret;
    private IMeshTools_ModelAlgo? myPostProcessor;
    private IMeshTools_Parameters myParameters = new();

    /// <summary>A context without tools.</summary>
    public IMeshTools_Context()
    {
    }

    /// <summary>Builds the model with the model builder; true on success.</summary>
    public virtual bool BuildModel()
    {
        if (myModelBuilder is null)
        {
            return false;
        }

        myModel = myModelBuilder.Perform(GetShape(), myParameters);

        return myModel is not null;
    }

    /// <summary>Discretizes the model's edges with the edge discretizer; true on success.</summary>
    public virtual bool DiscretizeEdges()
    {
        if (myModel is null || myEdgeDiscret is null)
        {
            return false;
        }

        // Discretize edges of a model.
        return myEdgeDiscret.Perform(myModel, myParameters, new Message_ProgressRange());
    }

    /// <summary>Heals the model built by <see cref="DiscretizeEdges"/> with the healer, if any; true on success.</summary>
    public virtual bool HealModel()
    {
        if (myModel is null)
        {
            return false;
        }

        return myModelHealer is null
                 ? true
                 : myModelHealer.Perform(myModel, myParameters, new Message_ProgressRange());
    }

    /// <summary>
    /// Pre-processes the model with the pre-processor, if any (auxiliary work such as cleaning old triangulations from
    /// the shape); true on success.
    /// </summary>
    public virtual bool PreProcessModel()
    {
        if (myModel is null)
        {
            return false;
        }

        return myPreProcessor is null
                 ? true
                 : myPreProcessor.Perform(myModel, myParameters, new Message_ProgressRange());
    }

    /// <summary>Meshes the model's faces with the face discretizer; true on success.</summary>
    public virtual bool DiscretizeFaces(Message_ProgressRange theRange)
    {
        if (myModel is null || myFaceDiscret is null)
        {
            return false;
        }

        // Discretize faces of a model.
        return myFaceDiscret.Perform(myModel, myParameters, theRange);
    }

    /// <summary>Post-processes the model with the post-processor, if any; true on success.</summary>
    public virtual bool PostProcessModel()
    {
        if (myModel is null)
        {
            return false;
        }

        return myPostProcessor is null
                 ? true
                 : myPostProcessor.Perform(myModel, myParameters, new Message_ProgressRange());
    }

    /// <summary>Cleans the temporary data: the model, if the parameters ask for it.</summary>
    public virtual void Clean()
    {
        if (myParameters.CleanModel)
        {
            myModel = null;
        }
    }

    /// <summary>The tool building the discrete model.</summary>
    public IMeshTools_ModelBuilder? GetModelBuilder() => myModelBuilder;

    /// <summary>Sets the tool building the discrete model.</summary>
    public void SetModelBuilder(IMeshTools_ModelBuilder? theBuilder) => myModelBuilder = theBuilder;

    /// <summary>The tool discretizing the model's edges.</summary>
    public IMeshTools_ModelAlgo? GetEdgeDiscret() => myEdgeDiscret;

    /// <summary>Sets the tool discretizing the model's edges.</summary>
    public void SetEdgeDiscret(IMeshTools_ModelAlgo? theEdgeDiscret) => myEdgeDiscret = theEdgeDiscret;

    /// <summary>The tool healing the model.</summary>
    public IMeshTools_ModelAlgo? GetModelHealer() => myModelHealer;

    /// <summary>Sets the tool healing the model.</summary>
    public void SetModelHealer(IMeshTools_ModelAlgo? theModelHealer) => myModelHealer = theModelHealer;

    /// <summary>The pre-processing algorithm.</summary>
    public IMeshTools_ModelAlgo? GetPreProcessor() => myPreProcessor;

    /// <summary>Sets the pre-processing algorithm.</summary>
    public void SetPreProcessor(IMeshTools_ModelAlgo? thePreProcessor) => myPreProcessor = thePreProcessor;

    /// <summary>The meshing algorithm.</summary>
    public IMeshTools_ModelAlgo? GetFaceDiscret() => myFaceDiscret;

    /// <summary>Sets the meshing algorithm.</summary>
    public void SetFaceDiscret(IMeshTools_ModelAlgo? theFaceDiscret) => myFaceDiscret = theFaceDiscret;

    /// <summary>The post-processing algorithm.</summary>
    public IMeshTools_ModelAlgo? GetPostProcessor() => myPostProcessor;

    /// <summary>Sets the post-processing algorithm.</summary>
    public void SetPostProcessor(IMeshTools_ModelAlgo? thePostProcessor) => myPostProcessor = thePostProcessor;

    /// <summary>The meshing parameters.</summary>
    public IMeshTools_Parameters GetParameters() => myParameters;

    /// <summary>The meshing parameters, for writing.</summary>
    public ref IMeshTools_Parameters ChangeParameters() => ref myParameters;

    /// <summary>The discrete model of the shape.</summary>
    public IMeshData_Model? GetModel() => myModel;
}
