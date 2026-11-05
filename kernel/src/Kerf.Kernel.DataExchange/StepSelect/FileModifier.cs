// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_FileModifier.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_FileModifier_0.cxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_ModelModifier.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_ModelModifier_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Framework.Selection;

using Kerf.Engine.Exchange.Step.Data;

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Selections;

/// <summary>A File Modifier of STEP: it acts on the StepWriter, once the model is produced.</summary>
internal abstract class FileModifier : GeneralModifier
{
    /// <summary>
    /// Perform the action specific to each class of File Modifier &lt;ctx&gt; is the ContextWrite, which brings : the
    /// model, the protocol, the file name, plus the object AppliedModifiers (not used here) and the CheckList Remark
    /// that the model has to be casted for specific access &lt;writer&gt; is the Writer and is specific to each norm,
    /// on which to act
    /// </summary>
    public abstract void Perform(ContextWrite ctx, StepWriter writer);

    /// <summary>Sets a File Modifier to keep the graph of dependences unchanges (because it works on the model already produced)</summary>
    protected FileModifier() : base(false)
    {
    }
}

/// <summary>A Model Modifier of STEP: it works on a StepData_StepModel.</summary>
internal abstract class ModelModifier : SelectionModifier
{
    /// <summary>
    /// The inherited Perform does the required cast (and refuses to go further if cast has failed) then calls the
    /// instantiated Performing
    /// </summary>
    public override void Perform(ContextModif ctx, InterfaceModel target, Protocol protocol,
                                 CopyTool tc)
    {
        ctx.TraceModifier(this);
        StepModel targ = target as StepModel;
        StepProtocol prot = protocol as StepProtocol;
        if (targ is null)
        {
            ctx.CCheck().AddFail("Model to Modify : unproper type");
            return;
        }
        PerformProtocol(ctx, targ, prot, tc);
    }

    /// <summary>
    /// Specific Perform with Protocol. It is defined to let the Protocol unused and to call Performing without
    /// Protocol (most current case). It can be redefined if specific action requires Protocol.
    /// </summary>
    public virtual void PerformProtocol(ContextModif ctx, StepModel target, StepProtocol protocol,
                                        CopyTool tc)
    {
        ctx.SetProtocol(protocol);
        Performing(ctx, target, tc);
    }

    /// <summary>
    /// Specific Perform, without Protocol. If Performing with Protocol is redefined, Performing without Protocol must
    /// though be defined to do nothing (not called, but demanded by the linker)
    /// </summary>
    public abstract void Performing(ContextModif ctx, StepModel target, CopyTool tc);

    /// <summary>Calls inherited Initialize, transmits to it the information &lt;maychangegraph&gt;</summary>
    protected ModelModifier(bool maychangegraph) : base(maychangegraph)
    {
    }
}
