// Port of: src/DataExchange/TKXSBase/Interface/Interface_UndefinedContent.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_UndefinedContent.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Defines resources for an "Undefined Entity" : such an Entity is used to describe an Entity which complies
/// with the Norm, but of an Unknown Type : hence it is kept under a literal form (avoiding to loose data).
/// UndefinedContent offers a way to store a list of Parameters, as literals or references to other Entities
/// Each Interface must provide one "UndefinedEntity", which must have same basic description as all its
/// types of entities : the best way would be double inheritance : on the Entity Root of the Norm and on an
/// general "UndefinedEntity" While it is not possible to do so, the UndefinedEntity of each Interface can
/// define its own UndefinedEntity by INCLUDING (in a field) this UndefinedContent Hence, for that
/// UndefinedEntity, define a Constructor which creates this UndefinedContent, plus access methods to it (or
/// to its data, calling methods defined here). Finally, the Protocols of each norm have to Create and
/// Recognize Unknown Entities of this norm
/// </summary>
public class Interface_UndefinedContent : Standard_Transient
{
    private const int Content_TypeField = 31;
    private const int Content_LocalField = 7;
    private const int Content_LocalRef = 1;
    private const int Content_LocalShift = 5;
    private const int Content_NumberShift = 8;
    // This class provides basic services for defining entities
    // Unknown (this, for lack of double inheritance) : literal description

    /// <summary>
    /// Defines an empty UndefinedContent
    /// </summary>
    public Interface_UndefinedContent()
    {
        thenbparams = 0;
        thenbstr = 0;
    }

    /// <summary>
    /// Gives count of recorded parameters
    /// </summary>
    public int NbParams()
    {
        return thenbparams;
    }

    /// <summary>
    /// Gives count of Literal Parameters
    /// </summary>
    public int NbLiterals()
    {
        return thenbstr;
    }

    /// <summary>
    /// Returns data of a Parameter : its type, and the entity if it designates en entity ("ent") or its literal
    /// value else ("str") Returned value (Boolean) : True if it is an Entity, False else
    /// </summary>
    public bool ParamData(int num, out Interface_ParamType ptype, out Standard_Transient? ent, out TCollection_HAsciiString? val)
    {
        ent = null; // OCCT leaves the one not concerned as it was
        val = null;
        if (num < 1 || num > thenbparams)
        {
            throw new Standard_OutOfRange("Interface UndefinedContent : ParamData");
        }
        int desc = theparams.Value(num);
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        ptype = (Interface_ParamType)(desc & Content_TypeField);
        int adr = desc >> Content_NumberShift;
        if (local == Content_LocalRef)
        {
            ent = theentities.Value(adr);
        }
        else
        {
            val = thevalues.Value(adr);
        }
        return (local == Content_LocalRef);
    }

    /// <summary>
    /// Returns the ParamType of a Param, given its rank Error if num is not between 1 and NbParams
    /// </summary>
    public Interface_ParamType ParamType(int num)
    {
        return (Interface_ParamType)(theparams.Value(num) & Content_TypeField);
    }

    /// <summary>
    /// Returns True if a Parameter is recorded as an entity Error if num is not between 1 and NbParams
    /// </summary>
    public bool IsParamEntity(int num)
    {
        return (((theparams.Value(num) >> Content_LocalShift) & Content_LocalField) == Content_LocalRef);
    }

    /// <summary>
    /// Returns Entity corresponding to a Param, given its rank
    /// </summary>
    public Standard_Transient? ParamEntity(int num)
    {
        int desc = theparams.Value(num);
        if (((desc >> Content_LocalShift) & Content_LocalField) != Content_LocalRef)
        {
            throw new Interface_InterfaceError("UndefinedContent : Param is not Entity type");
        }
        return theentities.Value(desc >> Content_NumberShift);
    }

    /// <summary>
    /// Returns literal value of a Parameter, given its rank
    /// </summary>
    public TCollection_HAsciiString? ParamValue(int num)
    {
        int desc = theparams.Value(num);
        if (((desc >> Content_LocalShift) & Content_LocalField) != 0)
        {
            throw new Interface_InterfaceError("UndefinedContent : Param is not literal");
        }
        return thevalues.Value(desc >> Content_NumberShift);
    }

    /// <summary>
    /// Manages reservation for parameters (internal use) (nb : total count of parameters, nblit : count of
    /// literals)
    /// </summary>
    public void Reservate(int nb, int nblit)
    {
        // Reservation: If enlargement, copy old to new ...
        if (nb > thenbparams)
        { // Total reservation
            if (theparams is null)
            {
                theparams = new NCollection_HArray1<int>(1, nb);
            }
            else if (nb > theparams.Length())
            {
                int nbnew = 2 * thenbparams; // reserve a bit more
                if (nbnew < nb)
                {
                    nbnew = nb;
                }
                NCollection_HArray1<int> newparams = new NCollection_HArray1<int>(1, nbnew);
                for (int i = 1; i <= thenbparams; i++)
                {
                    newparams.SetValue(i, theparams.Value(i));
                }
                theparams = newparams;
            }
        }

        if (nblit > thenbstr)
        { // Literal reservation
            if (thevalues is null)
            {
                thevalues = new NCollection_HArray1<TCollection_HAsciiString>(1, nblit);
            }
            else if (nblit > thevalues.Length())
            {
                int nbnew = 2 * thenbstr; // reserve a bit more
                if (nbnew < nblit)
                {
                    nbnew = nblit;
                }
                NCollection_HArray1<TCollection_HAsciiString> newvalues =
                    new NCollection_HArray1<TCollection_HAsciiString>(1, nbnew);
                for (int i = 1; i <= thenbstr; i++)
                {
                    newvalues.SetValue(i, thevalues.Value(i));
                }
                thevalues = newvalues;
            }
        }
        //  Entities: Parameters - Literals. In fact, EntityList is dynamic
    }

    /// <summary>
    /// Adds a literal Parameter to the list
    /// </summary>
    public void AddLiteral(Interface_ParamType ptype, TCollection_HAsciiString? val)
    {
        Reservate(thenbparams + 1, thenbstr + 1);
        int desc = (int)(ptype);
        thenbstr++;
        thenbparams++;
        thevalues.SetValue(thenbstr, val);
        desc += (thenbstr << Content_NumberShift);
        theparams.SetValue(thenbparams, desc);
    }

    /// <summary>
    /// Adds a Parameter which references an Entity
    /// </summary>
    public void AddEntity(Interface_ParamType ptype, Standard_Transient? ent)
    {
        Reservate(thenbparams + 1, 0);
        int desc = (int)(ptype);
        theentities.Append(ent);
        desc += Content_LocalRef << Content_LocalShift; // "It is an Entity"
        thenbparams++; // Rank: thenbparams - thenbstr
        desc += ((thenbparams - thenbstr) << Content_NumberShift);
        theparams.SetValue(thenbparams, desc);
    }

    /// <summary>
    /// Removes a Parameter given its rank
    /// </summary>
    public void RemoveParam(int num)
    {
        int desc = theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        //    Remove an Entity
        if (c1ent)
        {
            theentities.Remove(rang);
            //    Remove a Literal
        }
        else
        { // thevalues.Remove(rang) but it is an array
            for (int i = rang + 1; i <= thenbstr; i++)
            {
                thevalues.SetValue(i - 1, thevalues.Value(i));
            }
            TCollection_HAsciiString nulstr = null;
            thevalues.SetValue(thenbstr, nulstr);
            thenbstr--;
        }
        //    Remove this parameter from the list (which is an array)
        int np; // svv Jan11 2000 : porting on DEC
        for (np = num + 1; np <= thenbparams; np++)
        {
            theparams.SetValue(np - 1, theparams.Value(np));
        }
        theparams.SetValue(thenbparams, 0);
        thenbparams--;
        //    Renumber, Entity or Literal, depending
        for (np = 1; np <= thenbparams; np++)
        {
            desc = theparams.Value(np);
            if (((desc >> Content_LocalShift) & Content_LocalField) == local
                    && (desc >> Content_NumberShift) > rang)
            {
                theparams.SetValue(np, desc - (1 << Content_NumberShift));
            }
        }
    }

    /// <summary>
    /// Sets a new value for the Parameter &lt;num&gt;, to a literal value (if it referenced formerly an Entity,
    /// this Entity is removed)
    /// </summary>
    public void SetLiteral(int num, Interface_ParamType ptype, TCollection_HAsciiString? val)
    {
        //  Change a parameter. If already literal, simple substitution
        //  If Entity, remove the entity and renumber the "Entity" Parameters
        int desc = theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (c1ent)
        {
            //  Entity: remove it and renumber the "Entity" type Parameters
            theentities.Remove(rang);
            for (int i = 1; i <= thenbparams; i++)
            {
                desc = theparams.Value(i);
                if (((desc >> Content_LocalShift) & Content_LocalField) == Content_LocalRef
                        && (desc >> Content_NumberShift) > rang)
                {
                    theparams.SetValue(i, desc - (1 << Content_NumberShift));
                }
            }
            //  And prepare arrival of an additional Literal
            Reservate(thenbparams, thenbstr + 1);
            thenbstr++;
            rang = thenbstr;
        }
        //  Put the new value in place and rebuild the Param descriptor
        thevalues.SetValue(rang, val);
        desc = (int)(ptype) + (rang << Content_NumberShift);
        theparams.SetValue(num, desc);
    }

    /// <summary>
    /// Sets a new value for the Parameter &lt;num&gt;, to reference an Entity. To simply change the Entity, see
    /// the variant below
    /// </summary>
    public void SetEntity(int num, Interface_ParamType ptype, Standard_Transient? ent)
    {
        //  Change a Parameter. If already Entity, simple substitution
        //  If Literal, remove its value and renumber the Literal parameters
        int desc = theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (!c1ent)
        {
            //  Literal: remove it and renumber the "Entity" type Parameters
            //  (Remove Literal but in an array)
            int i; // svv Jan11 2000 : porting on DEC
            for (i = rang + 1; i <= thenbstr; i++)
            {
                thevalues.SetValue(i - 1, thevalues.Value(i));
            }
            TCollection_HAsciiString nulstr = null;
            thevalues.SetValue(thenbstr, nulstr);

            for (i = 1; i <= thenbparams; i++)
            {
                desc = theparams.Value(i);
                if (((desc >> Content_LocalShift) & Content_LocalField) == 0
                        && (desc >> Content_NumberShift) > rang)
                {
                    theparams.SetValue(i, desc - (1 << Content_NumberShift));
                }
            }
            //  And prepare arrival of an additional Entity
            thenbstr--;
            rang = thenbparams - thenbstr;
            //  Put the new value in place and rebuild the Param descriptor
            theentities.Append(ent);
        }
        else
        {
            theentities.SetValue(rang, ent);
        }

        desc = (int)(ptype) + (Content_LocalRef << Content_LocalShift) + (rang << Content_NumberShift);
        theparams.SetValue(num, desc);
    }

    /// <summary>
    /// Changes the Entity referenced by the Parameter &lt;num&gt; (with same ParamType)
    /// </summary>
    public void SetEntity(int num, Standard_Transient? ent)
    {
        //  Change the Entity defined by a Parameter, all other things being equal,
        //  PROVIDED that it is already an "Entity" type Parameter
        int desc = theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (!c1ent)
        {
            throw new Interface_InterfaceError("UndefinedContent : SetEntity");
        }
        theentities.SetValue(rang, ent);
    }

    /// <summary>
    /// Returns globally the list of param entities. Note that it can be used as shared entity list for the
    /// UndefinedEntity
    /// </summary>
    public Interface_EntityList EntityList()
    {
        return theentities;
    }

    /// <summary>
    /// Copies contents of undefined entities; deigned to be called by GetFromAnother method from Undefined
    /// entity of each Interface (the basic operation is the same regardless the norm)
    /// </summary>
    public void GetFromAnother(Interface_UndefinedContent? other, Interface_CopyTool TC)
    {
        int nb = other.NbParams();
        theentities.Clear();
        thevalues = null;
        theparams = null;
        Reservate(nb, other.NbLiterals());

        Standard_Transient ent = null;
        for (int i = 1; i <= nb; i++)
        {
            Interface_ParamType ptype = default;
            TCollection_HAsciiString val = null;
            if (other.ParamData(i, out ptype, out ent, out val))
            {
                ent = TC.Transferred(ent);
                AddEntity(ptype, ent);
            }
            else
            {
                AddLiteral(ptype, val);
            }
        }
    }

    private int thenbparams;
    private int thenbstr;
    private NCollection_HArray1<int>? theparams;
    private NCollection_HArray1<TCollection_HAsciiString>? thevalues;
    private Interface_EntityList theentities = new Interface_EntityList();
}
