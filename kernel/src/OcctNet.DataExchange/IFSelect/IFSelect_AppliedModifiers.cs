// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_AppliedModifiers.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_AppliedModifiers.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class allows to memorize and access to the modifiers which are to be applied to a file. To each
/// modifier, is bound a list of integers (optional) : if this list is absent, the modifier applies to all
/// the file. Else, it applies to the entities designated by these numbers in the produced file. To record a
/// modifier, and a possible list of entity numbers to be applied on: AddModif (amodifier); loop on AddNum
/// (anumber); To query it, Count gives the count of recorded modifiers, then for each one: Item (numodif,
/// amodifier, entcount); IsForAll () -&gt; can be called, if True, applies on the whole file for (i = 1; i
/// &lt;= entcount; i ++) nument = ItemNum (i); -&gt; return an entity number
/// </summary>
public class IFSelect_AppliedModifiers : Standard_Transient
{

    /// <summary>
    /// Creates an AppliedModifiers, ready to record up to &lt;nbmax&gt; modifiers, on a model of &lt;nbent&gt;
    /// entities
    /// </summary>
    public IFSelect_AppliedModifiers(int nbmax, int nbent)
    {
        thelists = new Interface_IntList(nbmax + 1);
        thenbent = nbent;
        theentcnt = 0;
    }

    /// <summary>
    /// Records a modifier. By default, it is to apply on all a produced file. Further calls to AddNum will
    /// restrict this. Returns True if done, False if too many modifiers are already recorded
    /// </summary>
    public bool AddModif(IFSelect_GeneralModifier? modif)
    {
        if (themodifs.Length() >= thelists.NbEntities())
        {
            return false;
        }
        themodifs.Append(modif);
        thelists.SetNumber(themodifs.Length());
        return true;
    }

    /// <summary>
    /// Adds a number of entity of the output file to be applied on. If a sequence of AddNum is called after
    /// AddModif, this Modifier will be applied on the list of designated entities. Else, it will be applied on
    /// all the file Returns True if done, False if no modifier has yet been added
    /// </summary>
    public bool AddNum(int nument)
    {
        thelists.Add(nument);
        return true;
    }

    /// <summary>
    /// Returns the count of recorded modifiers
    /// </summary>
    public int Count()
    {
        return themodifs.Length();
    }

    /// <summary>
    /// Returns the description for applied modifier n0 &lt;num&gt; : the modifier itself, and the count of
    /// entities to be applied on. If no specific list of number has been defined, returns the total count of
    /// entities of the file If this count is zero, then the modifier applies to all the file (see below). Else,
    /// the numbers are then queried by calls to ItemNum between 1 and &lt;entcount&gt; Returns True if OK, False
    /// if &lt;num&gt; is out of range
    /// </summary>
    public bool Item(int num, ref IFSelect_GeneralModifier? modif, ref int entcount)
    {
        if (num < 1 || num > themodifs.Length())
        {
            return false;
        }
        modif = themodifs.Value(num);
        thelists.SetNumber(num);
        theentcnt = thelists.Length();
        entcount = (theentcnt > 0 ? theentcnt : thenbent);
        return true;
    }

    /// <summary>
    /// Returns a numero of entity to be applied on, given its rank in the list. If no list is defined (i.e. for
    /// all the file), returns &lt;nument&gt; itself, to give all the entities of the file Returns 0 if
    /// &lt;nument&gt; out of range
    /// </summary>
    public int ItemNum(int nument)
    {
        return (theentcnt > 0 ? thelists.Value(nument) : nument);
    }

    /// <summary>
    /// Returns the list of entities to be applied on (see Item) as a HSequence (IsForAll produces the complete
    /// list of all the entity numbers of the file
    /// </summary>
    public NCollection_HSequence<int>? ItemList()
    {
        NCollection_HSequence<int> list = new NCollection_HSequence<int>();
        int i, nb = (theentcnt > 0 ? theentcnt : thenbent);
        for (i = 1; i <= nb; i++)
        {
            list.Append(ItemNum(i));
        }
        return list;
    }

    /// <summary>
    /// Returns True if the applied modifier queried by last call to Item is to be applied to all the produced
    /// file. Else, &lt;entcount&gt; returned by Item gives the count of entity numbers, each one is queried by
    /// ItemNum
    /// </summary>
    public bool IsForAll()
    {
        return (theentcnt == 0);
    }

    private NCollection_Sequence<IFSelect_GeneralModifier> themodifs = new NCollection_Sequence<IFSelect_GeneralModifier>();
    private Interface_IntList thelists = new Interface_IntList();
    private int thenbent;
    private int theentcnt;
}
