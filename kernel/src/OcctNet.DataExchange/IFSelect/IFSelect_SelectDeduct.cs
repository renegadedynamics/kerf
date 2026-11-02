// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectDeduct.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectDeduct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A SelectDeduct determines a list of Entities from an Input Selection, by a computation : Output list is
/// not obliged to be a sub-list of Input list (for more specific, see SelectExtract for filtered sub-lists,
/// and SelectExplore for recurcive exploration) A SelectDeduct may use an alternate input for one shot This
/// allows to use an already existing definition, by overloading the input selection by an alternate list,
/// already defined, for one use : If this alternate list is set, InputResult queries it instead of calling
/// the input selection, then clears it immediately
/// </summary>
public abstract class IFSelect_SelectDeduct : IFSelect_Selection
{

    /// <summary>
    /// Defines or Changes the Input Selection
    /// </summary>
    public void SetInput(IFSelect_Selection? sel)
    {
        thesel = sel;
    }

    /// <summary>
    /// Returns the Input Selection
    /// </summary>
    public IFSelect_Selection? Input()
    {
        return thesel;
    }

    /// <summary>
    /// Returns True if the Input Selection is defined, False else
    /// </summary>
    public bool HasInput()
    {
        return (thesel is not null);
    }

    /// <summary>
    /// Tells if an Alternate List has been set, i.e. : the Alternate Definition is present and set
    /// </summary>
    public bool HasAlternate()
    {
        if (thealt is not null)
        {
            return thealt.IsSet();
        }
        return false;
    }

    /// <summary>
    /// Returns the Alternate Definition It is returned modifiable, hence an already defined SelectPointed can be
    /// used But if it was not yet defined, it is created the first time It is exploited by InputResult
    /// </summary>
    public ref IFSelect_SelectPointed? Alternate()
    {
        if (thealt is null)
        {
            thealt = new IFSelect_SelectPointed();
        }
        return ref thealt;
    }

    /// <summary>
    /// Returns the Result determined by Input Selection, as Unique if Input Selection is not defined, returns an
    /// empty list. If Alternate is set, InputResult takes its definition instead of calling the Input Selection,
    /// then clears it
    /// </summary>
    public Interface_EntityIterator InputResult(Interface_Graph G)
    {
        Interface_EntityIterator res = new Interface_EntityIterator();
        if (thealt is not null)
        {
            if (thealt.IsSet())
            {
                res = thealt.UniqueResult(G);
                thealt.Clear();
                return res;
            }
        }
        if (thesel is null)
        {
            return res;
        }
        return thesel.UniqueResult(G);
    }

    /// <summary>
    /// Puts in an Iterator the Selections from which "me" depends This list contains one Selection : the
    /// InputSelection
    /// </summary>
    public override void FillIterator(IFSelect_SelectionIterator iter)
    {
        iter.AddItem(thesel);
    }

    private IFSelect_Selection? thesel;
    private IFSelect_SelectPointed? thealt;
}
