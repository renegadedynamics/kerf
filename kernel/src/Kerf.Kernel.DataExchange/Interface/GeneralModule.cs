// Port of: src/DataExchange/TKXSBase/Interface/Interface_GeneralModule.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GeneralModule.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class defines general services, which must be provided for each type of Entity (i.e. of Transient
/// Object processed by an Interface) : Shared List, Check, Copy, Delete, Category To optimise processing
/// (e.g. firstly bind an Entity to a Module then calls Module), each recognized Entity Type corresponds to a
/// Case Number, determined by the Protocol each class of GeneralModule belongs to.
/// </summary>
internal abstract class GeneralModule : Transient
{

    /// <summary>
    /// Specific filling of the list of Entities shared by an Entity &lt;ent&gt;, according a Case Number
    /// &lt;CN&gt; (formerly computed by CaseNum), considered in the context of a Model &lt;model&gt; Default
    /// calls FillSharedCase (i.e., ignores the model) Can be redefined to use the model for working
    /// </summary>
    public virtual void FillShared(InterfaceModel? model, int casenum, Transient? ent, EntityIterator iter)
    {
        FillSharedCase(casenum, ent, iter);
    }

    /// <summary>
    /// Specific filling of the list of Entities shared by an Entity &lt;ent&gt;, according a Case Number
    /// &lt;CN&gt; (formerly computed by CaseNum). Can use the internal utility method Share, below
    /// </summary>
    public abstract void FillSharedCase(int cn, Transient? ent, EntityIterator iter);

    /// <summary>
    /// Adds an Entity to a Shared List (uses GetOneItem on &lt;iter&gt;)
    /// </summary>
    public void Share(EntityIterator iter, Transient? shared)
    {
        iter.GetOneItem(shared);
    }

    /// <summary>
    /// List the Implied References of &lt;ent&gt; considered in the context of a Model &lt;model&gt; : i.e. the
    /// Entities which are Referenced while not considered as Shared (not copied if &lt;ent&gt; is, references
    /// not renewed by CopyCase but by ImpliedCase, only if referenced Entities have been Copied too) FillShared
    /// + ListImplied give the complete list of References Default calls ListImpliedCase (i.e. ignores the model)
    /// Can be redefined to use the model for working
    /// </summary>
    public virtual void ListImplied(InterfaceModel? model, int casenum, Transient? ent, EntityIterator iter)
    {
        ListImpliedCase(casenum, ent, iter);
    }

    /// <summary>
    /// List the Implied References of &lt;ent&gt; (see above) are Referenced while not considered as Shared (not
    /// copied if &lt;ent&gt; is, references not renewed by CopyCase but by ImpliedCase, only if referenced
    /// Entities have been Copied too) FillSharedCase + ListImpliedCase give the complete list of Referenced
    /// Entities The provided default method does nothing (Implied References are specific of a little amount of
    /// Entity Classes).
    /// </summary>
    public virtual void ListImpliedCase(int cn, Transient? ent, EntityIterator iter)
    {

    }

    /// <summary>
    /// Specific Checking of an Entity &lt;ent&gt; Can check context queried through a ShareTool, as required
    /// </summary>
    public abstract void CheckCase(int cn, Transient? ent, ShareTool shares, ref FrameworkCheck? ach);

    /// <summary>
    /// Specific answer to the question "is Copy properly implemented" Remark that it should be in phase with the
    /// implementation of NewVoid+CopyCase/NewCopyCase Default returns always False, can be redefined
    /// </summary>
    public virtual bool CanCopy(int cn, Transient? ent)
    {
        return false;
    }

    /// <summary>
    /// Dispatches an entity Returns True if it works by copy, False if it just duplicates the starting Handle
    /// Dispatching means producing a new entity, image of the starting one, in order to be put into a new Model,
    /// this Model being itself the result of a dispatch from an original Model According to the cases, dispatch
    /// can either * just return &lt;entto&gt; as equating &lt;entfrom&gt; -&gt; the new model designates the
    /// starting entity : it is lighter, but the dispatched entity being shared might not be modified for
    /// dispatch * copy &lt;entfrom&gt; to &lt;entto&gt; by calling NewVoid+CopyCase (two steps) or NewCopiedCase
    /// (1) -&gt; the dispatched entity is a COPY, hence it can be modified The provided default just duplicates
    /// the handle without copying, then returns False. Can be redefined
    /// </summary>
    public virtual bool Dispatch(int cn, Transient? entfrom, out Transient? entto, CopyTool tc)
    {
        entto = entfrom;
        return false;
    }

    /// <summary>
    /// Creates a new void entity &lt;entto&gt; according to a Case Number This entity remains to be filled, by
    /// reading from a file or by copying from another entity of same type (see CopyCase)
    /// </summary>
    public abstract bool NewVoid(int cn, out Transient? entto);

    /// <summary>
    /// Specific Copy ("Deep") from &lt;entfrom&gt; to &lt;entto&gt; (same type) by using a CopyTool which
    /// provides its working Map. Use method Transferred from CopyTool to work
    /// </summary>
    public abstract void CopyCase(int cn, Transient? entfrom, Transient? entto, CopyTool tc);

    /// <summary>
    /// Specific operator (create+copy) defaulted to do nothing. It can be redefined : When it is not possible to
    /// work in two steps (NewVoid then CopyCase). This can occur when there is no default constructor : hence
    /// the result &lt;entto&gt; must be created with an effective definition. Remark : if NewCopiedCase is
    /// defined, CopyCase has nothing to do Returns True if it has produced something, false else
    /// </summary>
    public virtual bool NewCopiedCase(int cn, Transient? entfrom, ref Transient? entto, CopyTool tc)
    {
        return false;
    }

    /// <summary>
    /// Specific Copying of Implied References A Default is provided which does nothing (must current case !)
    /// Already copied references (by CopyFrom) must remain unchanged Use method Search from CopyTool to work
    /// </summary>
    public virtual void RenewImpliedCase(int cn, Transient? entfrom, Transient? entto, CopyTool tc)
    {

    }

    /// <summary>
    /// Prepares an entity to be deleted. What does it mean : Basically, any class of entity may define its own
    /// destructor By default, it does nothing but calling destructors on fields With the Memory Manager, it is
    /// useless to call destructor, it is done automatically when the Handle is nullified(cleared) BUT this is
    /// ineffective in looping structures (whatever these are "Implied" references or not). THUS : if no loop may
    /// appear in definitions, a class which inherits from TShared is correctly managed by automatic way BUT if
    /// there can be loops (or simply back pointers), they must be broken, for instance by clearing fields of one
    /// of the nodes The default does nothing, to be redefined if a loop can occur (Implied generally requires
    /// WhenDelete, but other cases can occur) Warning : &lt;dispatched&gt; tells if the entity to be deleted has
    /// been produced by Dispatch or not. Hence WhenDelete must be in coherence with Dispatch Dispatch can either
    /// copy or not. If it copies the entity, this one should be deleted If it doesn't (i.e. duplicates the
    /// handle) nothing to do If &lt;dispatch&gt; is False, normal deletion is to be performed
    /// </summary>
    public virtual void WhenDeleteCase(int cn, Transient? ent, bool dispatched)
    {

    }

    /// <summary>
    /// Returns a category number which characterizes an entity Category Numbers are managed by the class
    /// Category &lt;shares&gt; can be used to evaluate this number in the context Default returns 0 which means
    /// "unspecified"
    /// </summary>
    public virtual int CategoryNumber(int cn, Transient? ent, ShareTool shares)
    {
        return 0;
    }

    /// <summary>
    /// Determines if an entity brings a Name (or widerly, if a Name can be attached to it, through the ShareTool
    /// By default, returns a Null Handle (no name can be produced) Can be redefined Warning : While this string
    /// may be edited on the spot, if it is a read field, the returned value must be copied before.
    /// </summary>
    public virtual TCollection_HAsciiString? Name(int cn, Transient? ent, ShareTool shares)
    {
        TCollection_HAsciiString str = null;
        return str;
    }
}
