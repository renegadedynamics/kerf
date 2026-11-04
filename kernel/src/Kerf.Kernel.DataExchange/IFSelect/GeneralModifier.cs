// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_GeneralModifier.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_GeneralModifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This class gives a frame for Actions which modify the effect of a Dispatch, i.e. : By Selections and
/// Dispatches, an original Model can be split into one or more "target" Models : these Models contain
/// Entities copied from the original one (that is, a part of it). Basically, these dispatched Entities are
/// copied as identical to their original counterparts. Also the copied Models reproduce the Header of the
/// original one. Modifiers allow to change this copied content : this is the way to be used for any kind of
/// alterations, adaptations ... They are exploited by a ModelCopier, which firstly performs the copy
/// operation described by Dispatches, then invokes the Modifiers to work on the result. Each GeneralModifier
/// can be attached to : - all the Models produced - a Dispatch (it will be applied to all the Models
/// obtained from this Dispatch) designated by its Ident in a ShareOut - in addition, to a Selection
/// (facultative) : this adds a criterium, the Modifier is invoked on a produced Model only if this Model
/// contains an Entity copied from one of the Entities designated by this Selection. (for special Modifiers
/// from IFAdapt, while they must work on definite Entities, this Selection is mandatory to run) Remark :
/// this class has no action attached, it only provides a frame to work on criteria. Then, sub-classes will
/// define their kind of action, which can be applied at a precise step of the production of a File : see
/// Modifier, and in the package IFAdapt, EntityModifier and EntityCopier
/// </summary>
internal abstract class GeneralModifier : Transient
{

    /// <summary>
    /// Returns True if this modifier may change the graph of dependences (acknowledged at creation time)
    /// </summary>
    public bool MayChangeGraph()
    {
        return _thechgr;
    }

    /// <summary>
    /// Attaches to a Dispatch. If &lt;disp&gt; is Null, Resets it (to apply the Modifier on every Dispatch)
    /// </summary>
    public void SetDispatch(Dispatch? disp)
    {
        _thedisp = disp;
    }

    /// <summary>
    /// Returns the Dispatch to be matched, Null if not set
    /// </summary>
    public Dispatch? Dispatch()
    {
        return _thedisp;
    }

    /// <summary>
    /// Returns True if a Model obtained from the Dispatch &lt;disp&gt; is to be treated (apart from the
    /// Selection criterium) If Dispatch(me) is Null, returns True. Else, checks &lt;disp&gt;
    /// </summary>
    public bool Applies(Dispatch? disp)
    {
        if (_thedisp is null)
        {
            return true;
        }
        if (_thedisp != disp)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Sets a Selection : a Model is treated if it contains one or more Entities designated by the Selection
    /// </summary>
    public void SetSelection(AbstractSelection? sel)
    {
        _thesel = sel;
    }

    /// <summary>
    /// Resets the Selection : this criterium is not longer active
    /// </summary>
    public void ResetSelection()
    {
        _thesel = null;
    }

    /// <summary>
    /// Returns True if a Selection is set as an additional criterium
    /// </summary>
    public bool HasSelection
    {
        get
        {
            return _thesel is not null;
        }
    }

    /// <summary>
    /// Returns the Selection, or a Null Handle if not set
    /// </summary>
    public AbstractSelection? Selection()
    {
        return _thesel;
    }

    /// <summary>
    /// Returns a short text which defines the operation performed
    /// </summary>
    public abstract string Label();

    /// <summary>
    /// Sets the Modifier criteria to default Values (i.e. "always applies") &lt;maychangegraph&gt; must be
    /// provided at creation time, to : - False if this GeneralModifier surely lets the graph of dependencies
    /// unchanged (NO edition of any reference, BUT also NO entity added or replaced or removed) - True if there
    /// is a possibility of changing the graph of dependencies when this modifier is applied
    /// </summary>
    protected GeneralModifier(bool maychangegraph)
    {
        _thechgr = maychangegraph;
    }

    private AbstractSelection? _thesel;
    private Dispatch? _thedisp;
    private bool _thechgr;
}
