// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Status;

namespace Occt;

/// <summary>
/// Meshes each face of the shape that has no triangulation. A face that already has one keeps it if its deflection
/// fits the parameters; otherwise its triangulation is replaced.
/// </summary>
/// <remarks>
/// Statuses: <see cref="Message_Done1"/>, finished without errors; <see cref="Message_Fail1"/>, an invalid context;
/// <see cref="Message_Fail2"/>, an unexpected error; <see cref="Message_Fail3"/>, the edges couldn't be discretized;
/// <see cref="Message_Fail4"/>, the model couldn't be healed; <see cref="Message_Fail5"/>, pre-processing failed;
/// <see cref="Message_Fail6"/>, the faces couldn't be discretized; <see cref="Message_Fail7"/>, post-processing
/// failed; <see cref="Message_Fail8"/>, user break; <see cref="Message_Warn1"/>, the shape has nothing to mesh.
/// </remarks>
public class IMeshTools_MeshBuilder : Message_Algorithm
{
    private IMeshTools_Context? myContext;

    /// <summary>A builder without context.</summary>
    public IMeshTools_MeshBuilder()
    {
    }

    /// <summary>A builder with the given context.</summary>
    public IMeshTools_MeshBuilder(IMeshTools_Context? theContext)
    {
        myContext = theContext;
    }

    /// <summary>Sets the context.</summary>
    public void SetContext(IMeshTools_Context? theContext) => myContext = theContext;

    /// <summary>The context.</summary>
    public IMeshTools_Context? GetContext() => myContext;

    /// <summary>Meshes the shape of the context.</summary>
    public virtual void Perform(Message_ProgressRange theRange)
    {
        ClearStatus();

        IMeshTools_Context? aContext = GetContext();
        if (aContext is null)
        {
            SetStatus(Message_Fail1);
            return;
        }

        using var aPS = new Message_ProgressScope(theRange, "Mesh Perform", 10);

        if (aContext.BuildModel())
        {
            if (aContext.DiscretizeEdges())
            {
                if (aContext.HealModel())
                {
                    if (aContext.PreProcessModel())
                    {
                        if (aContext.DiscretizeFaces(aPS.Next(9)))
                        {
                            if (aContext.PostProcessModel())
                            {
                                SetStatus(Message_Done1);
                            }
                            else
                            {
                                SetStatus(Message_Fail7);
                            }
                        }
                        else
                        {
                            if (!aPS.More())
                            {
                                SetStatus(Message_Fail8);
                                aContext.Clean();
                                return;
                            }
                            SetStatus(Message_Fail6);
                        }
                    }
                    else
                    {
                        SetStatus(Message_Fail5);
                    }
                }
                else
                {
                    SetStatus(Message_Fail4);
                }
            }
            else
            {
                SetStatus(Message_Fail3);
            }
        }
        else
        {
            IMeshTools_ModelBuilder? aModelBuilder = aContext.GetModelBuilder();

            if (aModelBuilder is null)
            {
                SetStatus(Message_Fail1);
            }
            else
            {
                // Is null shape or another problem?
                SetStatus(aModelBuilder.GetStatus().IsSet(Message_Fail1) ? Message_Warn1 : Message_Fail2);
            }
        }

        aPS.Next(1);
        aContext.Clean();
    }
}
