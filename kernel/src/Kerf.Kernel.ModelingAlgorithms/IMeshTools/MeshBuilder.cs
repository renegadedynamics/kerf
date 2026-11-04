// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_MeshBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Foundation.Status;

namespace Kerf.Engine.Meshing.Tools;

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
internal class MeshBuilder : Algorithm
{
    private ToolContext? _context;

    /// <summary>A builder without context.</summary>
    public MeshBuilder()
    {
    }

    /// <summary>A builder with the given context.</summary>
    public MeshBuilder(ToolContext? context)
    {
        _context = context;
    }

    /// <summary>Sets the context.</summary>
    public void SetContext(ToolContext? context) => _context = context;

    /// <summary>The context.</summary>
    public ToolContext? GetContext() => _context;

    /// <summary>Meshes the shape of the context.</summary>
    public virtual void Perform(ProgressRange range)
    {
        ClearStatus();

        ToolContext? context = GetContext();
        if (context is null)
        {
            SetStatus(Fail1);
            return;
        }

        using var ps = new ProgressScope(range, "Mesh Perform", 10);

        if (context.BuildModel())
        {
            if (context.DiscretizeEdges())
            {
                if (context.HealModel())
                {
                    if (context.PreProcessModel())
                    {
                        if (context.DiscretizeFaces(ps.Next(9)))
                        {
                            if (context.PostProcessModel())
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
                            if (!ps.More())
                            {
                                SetStatus(Fail8);
                                context.Clean();
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
            ModelBuilder? modelBuilder = context.GetModelBuilder();

            if (modelBuilder is null)
            {
                SetStatus(Fail1);
            }
            else
            {
                // Is null shape or another problem?
                SetStatus(modelBuilder.GetStatus().IsSet(Fail1) ? Warn1 : Fail2);
            }
        }

        ps.Next(1);
        context.Clean();
    }
}
