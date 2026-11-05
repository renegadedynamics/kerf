// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_DuplicateCleaner.hxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_DuplicateCleaner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Tidy;

/// <summary>
/// Merges equal STEP entities in a work session and removes the duplicates. 'Equal' means equal names and exactly equal
/// values (<see cref="TidyHash"/>). The reducers merge, each for its type of entity, and the cleaner then removes what they replaced, and any
/// entity left with nothing referring to it and nothing it refers to.
/// </summary>
internal sealed class DuplicateCleaner(ExchangeWorkSession workSession)
{
    /// <summary>Merges every set of equal entities in the session's model, and removes the duplicates.</summary>
    public void Perform()
    {
        if (workSession.Model() is not StepModel model)
        {
            return;
        }

        var cartesianPointReducer = new CartesianPointReducer(workSession);
        var directionReducer = new DirectionReducer(workSession);
        var axis2Placement3dReducer = new Axis2Placement3dReducer(workSession);
        var vectorReducer = new VectorReducer(workSession);
        var lineReducer = new LineReducer(workSession);
        var planeReducer = new PlaneReducer(workSession);
        var circleReducer = new CircleReducer(workSession);

        for (int index = 1; index <= model.EntityCount; ++index)
        {
            Transient entity = model.Value(index)!;
            cartesianPointReducer.ProcessEntity(entity);
            directionReducer.ProcessEntity(entity);
            axis2Placement3dReducer.ProcessEntity(entity);
            vectorReducer.ProcessEntity(entity);
            lineReducer.ProcessEntity(entity);
            planeReducer.ProcessEntity(entity);
            circleReducer.ProcessEntity(entity);
        }

        var replacedEntities = new HashSet<Transient>(ReferenceEqualityComparer.Instance);
        cartesianPointReducer.Perform(replacedEntities);
        directionReducer.Perform(replacedEntities);
        axis2Placement3dReducer.Perform(replacedEntities);
        vectorReducer.Perform(replacedEntities);
        lineReducer.Perform(replacedEntities);
        planeReducer.Perform(replacedEntities);
        circleReducer.Perform(replacedEntities);

        RemoveEntities(replacedEntities);
    }

    private void RemoveEntities(HashSet<Transient> toRemove)
    {
        if (toRemove.Count == 0)
        {
            return;
        }

        var readModel = (StepModel)workSession.Model()!;
        StepModel intermediateModel = NewModelLike(readModel);
        for (int i = 1; i <= readModel.EntityCount; i++)
        {
            Transient entity = readModel.Value(i)!;
            if (!toRemove.Contains(entity))
            {
                intermediateModel.AddWithRefs(entity);
            }
        }

        workSession.SetModel(intermediateModel);
        workSession.ComputeGraph();

        // Clean hanged entities.
        StepModel newModel = NewModelLike(intermediateModel);
        EntityGraph graph = workSession.Graph();
        for (int i = 1; i <= intermediateModel.EntityCount; i++)
        {
            Transient entity = intermediateModel.Value(i)!;
            if (graph.Shareds(entity).NbEntities() > 0 || graph.Sharings(entity).NbEntities() > 0)
            {
                newModel.AddWithRefs(entity);
            }
        }

        workSession.SetModel(newModel);
        workSession.ComputeGraph();
    }

    // OCCT's new models leave the parameters and header behind. The parameters name the schema, so its file names the default
    // whichever was asked for; its Write makes a new header, but a stream written after this had none. Both go across
    // (occt-issues.md, unit 11N).
    private static StepModel NewModelLike(StepModel model)
    {
        var like = new StepModel { InternalParameters = model.InternalParameters };
        like.SetProtocol(model.Protocol());
        like.SetGTool(model.GTool());
        for (EntityIterator header = model.Header(); header.More(); header.Next())
        {
            like.AddHeaderEntity(header.Value()!);
        }
        return like;
    }
}
