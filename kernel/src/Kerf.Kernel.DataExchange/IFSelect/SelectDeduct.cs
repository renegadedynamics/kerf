// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectDeduct.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectDeduct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectDeduct determines a list of Entities from an Input Selection, by a computation : Output list is
/// not obliged to be a sub-list of Input list (for more specific, see SelectExtract for filtered sub-lists,
/// and SelectExplore for recurcive exploration) A SelectDeduct may use an alternate input for one shot This
/// allows to use an already existing definition, by overloading the input selection by an alternate list,
/// already defined, for one use : If this alternate list is set, InputResult queries it instead of calling
/// the input selection, then clears it immediately
/// </summary>
internal abstract class SelectDeduct : AbstractSelection
{

    /// <summary>
    /// Defines or Changes the Input Selection
    /// </summary>
    public void SetInput(AbstractSelection? sel)
    {
        _thesel = sel;
    }

    /// <summary>
    /// Returns the Input Selection
    /// </summary>
    public AbstractSelection? Input()
    {
        return _thesel;
    }

    /// <summary>
    /// Returns True if the Input Selection is defined, False else
    /// </summary>
    public bool HasInput
    {
        get
        {
            return (_thesel is not null);
        }
    }

    /// <summary>
    /// Tells if an Alternate List has been set, i.e. : the Alternate Definition is present and set
    /// </summary>
    public bool HasAlternate()
    {
        if (_thealt is not null)
        {
            return _thealt.IsSet;
        }
        return false;
    }

    /// <summary>
    /// Returns the Alternate Definition It is returned modifiable, hence an already defined SelectPointed can be
    /// used But if it was not yet defined, it is created the first time It is exploited by InputResult
    /// </summary>
    public ref SelectPointed? Alternate()
    {
        if (_thealt is null)
        {
            _thealt = new SelectPointed();
        }
        return ref _thealt;
    }

    /// <summary>
    /// Returns the Result determined by Input Selection, as Unique if Input Selection is not defined, returns an
    /// empty list. If Alternate is set, InputResult takes its definition instead of calling the Input Selection,
    /// then clears it
    /// </summary>
    public EntityIterator InputResult(EntityGraph g)
    {
        EntityIterator res = new EntityIterator();
        if (_thealt is not null)
        {
            if (_thealt.IsSet)
            {
                res = _thealt.UniqueResult(g);
                _thealt.Clear();
                return res;
            }
        }
        if (_thesel is null)
        {
            return res;
        }
        return _thesel.UniqueResult(g);
    }

    /// <summary>
    /// Puts in an Iterator the Selections from which "me" depends This list contains one Selection : the
    /// InputSelection
    /// </summary>
    public override void FillIterator(SelectionIterator iter)
    {
        iter.AddItem(_thesel);
    }

    private AbstractSelection? _thesel;
    private SelectPointed? _thealt;
}
