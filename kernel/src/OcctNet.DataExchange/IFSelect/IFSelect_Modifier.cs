// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Modifier.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Modifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class gives a frame for Actions which can work globally on a File once completely defined (i.e.
/// afterwards) Remark : if no Selection is set as criterium, the Modifier is set to work and should consider
/// all the content of the Model produced.
/// </summary>
public abstract class IFSelect_Modifier : IFSelect_GeneralModifier
{

    /// <summary>
    /// This deferred method defines the action specific to each class of Modifier. It is called by a
    /// ModelCopier, once the Model generated and filled. ModelCopier has already checked the criteria (Dispatch,
    /// Model Rank, Selection) before calling it. &lt;ctx&gt; detains information about original data and
    /// selection. The result of copying, on which modifications are to be done, is &lt;target&gt;. &lt;TC&gt;
    /// allows to run additional copies as required In case of Error, use methods CCheck from the ContextModif to
    /// acknowledge an entity Check or a Global Check with messages
    /// </summary>
    public abstract void Perform(IFSelect_ContextModif ctx, Interface_InterfaceModel? target, Interface_Protocol? protocol, Interface_CopyTool TC);

    /// <summary>
    /// Calls inherited Initialize, transmits to it the information &lt;maychangegraph&gt;
    /// </summary>
    protected IFSelect_Modifier(bool grf) : base(grf)
    {
    }
}
