// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_FileModifier.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_FileModifier_0.cxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_ModelModifier.hxx
// Port of: src/DataExchange/TKDESTEP/StepSelect/StepSelect_ModelModifier_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>A File Modifier of STEP: it acts on the StepWriter, once the model is produced.</summary>
public abstract class StepSelect_FileModifier : IFSelect_GeneralModifier
{
    /// <summary>
    /// Perform the action specific to each class of File Modifier &lt;ctx&gt; is the ContextWrite, which brings : the
    /// model, the protocol, the file name, plus the object AppliedModifiers (not used here) and the CheckList Remark
    /// that the model has to be casted for specific access &lt;writer&gt; is the Writer and is specific to each norm,
    /// on which to act
    /// </summary>
    public abstract void Perform(IFSelect_ContextWrite ctx, StepData_StepWriter writer);

    /// <summary>Sets a File Modifier to keep the graph of dependences unchanges (because it works on the model already produced)</summary>
    protected StepSelect_FileModifier() : base(false)
    {
    }
}

/// <summary>A Model Modifier of STEP: it works on a StepData_StepModel.</summary>
public abstract class StepSelect_ModelModifier : IFSelect_Modifier
{
    /// <summary>
    /// The inherited Perform does the required cast (and refuses to go further if cast has failed) then calls the
    /// instantiated Performing
    /// </summary>
    public override void Perform(IFSelect_ContextModif ctx, Interface_InterfaceModel target, Interface_Protocol protocol,
                                 Interface_CopyTool TC)
    {
        ctx.TraceModifier(this);
        StepData_StepModel targ = target as StepData_StepModel;
        StepData_Protocol prot = protocol as StepData_Protocol;
        if (targ is null)
        {
            ctx.CCheck().AddFail("Model to Modify : unproper type");
            return;
        }
        PerformProtocol(ctx, targ, prot, TC);
    }

    /// <summary>
    /// Specific Perform with Protocol. It is defined to let the Protocol unused and to call Performing without
    /// Protocol (most current case). It can be redefined if specific action requires Protocol.
    /// </summary>
    public virtual void PerformProtocol(IFSelect_ContextModif ctx, StepData_StepModel target, StepData_Protocol protocol,
                                        Interface_CopyTool TC)
    {
        ctx.SetProtocol(protocol);
        Performing(ctx, target, TC);
    }

    /// <summary>
    /// Specific Perform, without Protocol. If Performing with Protocol is redefined, Performing without Protocol must
    /// though be defined to do nothing (not called, but demanded by the linker)
    /// </summary>
    public abstract void Performing(IFSelect_ContextModif ctx, StepData_StepModel target, Interface_CopyTool TC);

    /// <summary>Calls inherited Initialize, transmits to it the information &lt;maychangegraph&gt;</summary>
    protected StepSelect_ModelModifier(bool maychangegraph) : base(maychangegraph)
    {
    }
}
