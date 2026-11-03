// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Engine.Foundation.Status;

namespace Occt;

/// <summary>
/// Meshes each face of the shape that has no triangulation. A face that already has one keeps it if its deflection
/// fits the parameters; otherwise its triangulation is replaced.
/// </summary>
/// <remarks>
/// Statuses: <see cref="Done1"/>, finished without errors; <see cref="Fail1"/>, an invalid context;
/// <see cref="Fail2"/>, an unexpected error; <see cref="Fail3"/>, the edges couldn't be discretized;
/// <see cref="Fail4"/>, the model couldn't be healed; <see cref="Fail5"/>, pre-processing failed;
/// <see cref="Fail6"/>, the faces couldn't be discretized; <see cref="Fail7"/>, post-processing
/// failed; <see cref="Fail8"/>, user break; <see cref="Warn1"/>, the shape has nothing to mesh.
/// </remarks>
public class IMeshTools_MeshBuilder : Algorithm
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
    public virtual void Perform(ProgressRange theRange)
    {
        ClearStatus();

        IMeshTools_Context? aContext = GetContext();
        if (aContext is null)
        {
            SetStatus(Fail1);
            return;
        }

        using var aPS = new ProgressScope(theRange, "Mesh Perform", 10);

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
                                SetStatus(Done1);
                            }
                            else
                            {
                                SetStatus(Fail7);
                            }
                        }
                        else
                        {
                            if (!aPS.More())
                            {
                                SetStatus(Fail8);
                                aContext.Clean();
                                return;
                            }
                            SetStatus(Fail6);
                        }
                    }
                    else
                    {
                        SetStatus(Fail5);
                    }
                }
                else
                {
                    SetStatus(Fail4);
                }
            }
            else
            {
                SetStatus(Fail3);
            }
        }
        else
        {
            IMeshTools_ModelBuilder? aModelBuilder = aContext.GetModelBuilder();

            if (aModelBuilder is null)
            {
                SetStatus(Fail1);
            }
            else
            {
                // Is null shape or another problem?
                SetStatus(aModelBuilder.GetStatus().IsSet(Fail1) ? Warn1 : Fail2);
            }
        }

        aPS.Next(1);
        aContext.Clean();
    }
}
