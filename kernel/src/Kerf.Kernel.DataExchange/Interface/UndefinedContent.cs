// Port of: src/DataExchange/TKXSBase/Interface/Interface_UndefinedContent.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_UndefinedContent.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

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
internal class UndefinedContent : Transient
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
    public UndefinedContent()
    {
        _thenbparams = 0;
        _thenbstr = 0;
    }

    /// <summary>
    /// Gives count of recorded parameters
    /// </summary>
    public int ParamCount
    {
        get
        {
            return _thenbparams;
        }
    }

    /// <summary>
    /// Gives count of Literal Parameters
    /// </summary>
    public int LiteralCount
    {
        get
        {
            return _thenbstr;
        }
    }

    /// <summary>
    /// Returns data of a Parameter : its type, and the entity if it designates en entity ("ent") or its literal
    /// value else ("str") Returned value (Boolean) : True if it is an Entity, False else
    /// </summary>
    public bool ParamData(int num, out ParamType ptype, out Transient? ent, out TCollection_HAsciiString? val)
    {
        ent = null; // OCCT leaves the one not concerned as it was
        val = null;
        if (num < 1 || num > _thenbparams)
        {
            throw new OutOfRangeException("Interface UndefinedContent : ParamData");
        }
        int desc = _theparams.Value(num);
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        ptype = (ParamType)(desc & Content_TypeField);
        int adr = desc >> Content_NumberShift;
        if (local == Content_LocalRef)
        {
            ent = _theentities.Value(adr);
        }
        else
        {
            val = _thevalues.Value(adr);
        }
        return (local == Content_LocalRef);
    }

    /// <summary>
    /// Returns the ParamType of a Param, given its rank Error if num is not between 1 and NbParams
    /// </summary>
    public ParamType ParamType(int num)
    {
        return (ParamType)(_theparams.Value(num) & Content_TypeField);
    }

    /// <summary>
    /// Returns True if a Parameter is recorded as an entity Error if num is not between 1 and NbParams
    /// </summary>
    public bool IsParamEntity(int num)
    {
        return (((_theparams.Value(num) >> Content_LocalShift) & Content_LocalField) == Content_LocalRef);
    }

    /// <summary>
    /// Returns Entity corresponding to a Param, given its rank
    /// </summary>
    public Transient? ParamEntity(int num)
    {
        int desc = _theparams.Value(num);
        if (((desc >> Content_LocalShift) & Content_LocalField) != Content_LocalRef)
        {
            throw new InterfaceErrorException("UndefinedContent : Param is not Entity type");
        }
        return _theentities.Value(desc >> Content_NumberShift);
    }

    /// <summary>
    /// Returns literal value of a Parameter, given its rank
    /// </summary>
    public TCollection_HAsciiString? ParamValue(int num)
    {
        int desc = _theparams.Value(num);
        if (((desc >> Content_LocalShift) & Content_LocalField) != 0)
        {
            throw new InterfaceErrorException("UndefinedContent : Param is not literal");
        }
        return _thevalues.Value(desc >> Content_NumberShift);
    }

    /// <summary>
    /// Manages reservation for parameters (internal use) (nb : total count of parameters, nblit : count of
    /// literals)
    /// </summary>
    public void Reservate(int nb, int nblit)
    {
        // Reservation: If enlargement, copy old to new ...
        if (nb > _thenbparams)
        { // Total reservation
            if (_theparams is null)
            {
                _theparams = new NCollection_HArray1<int>(1, nb);
            }
            else if (nb > _theparams.Length())
            {
                int nbnew = 2 * _thenbparams; // reserve a bit more
                if (nbnew < nb)
                {
                    nbnew = nb;
                }
                NCollection_HArray1<int> newparams = new NCollection_HArray1<int>(1, nbnew);
                for (int i = 1; i <= _thenbparams; i++)
                {
                    newparams.SetValue(i, _theparams.Value(i));
                }
                _theparams = newparams;
            }
        }

        if (nblit > _thenbstr)
        { // Literal reservation
            if (_thevalues is null)
            {
                _thevalues = new NCollection_HArray1<TCollection_HAsciiString>(1, nblit);
            }
            else if (nblit > _thevalues.Length())
            {
                int nbnew = 2 * _thenbstr; // reserve a bit more
                if (nbnew < nblit)
                {
                    nbnew = nblit;
                }
                NCollection_HArray1<TCollection_HAsciiString> newvalues =
                    new NCollection_HArray1<TCollection_HAsciiString>(1, nbnew);
                for (int i = 1; i <= _thenbstr; i++)
                {
                    newvalues.SetValue(i, _thevalues.Value(i));
                }
                _thevalues = newvalues;
            }
        }
        //  Entities: Parameters - Literals. In fact, EntityList is dynamic
    }

    /// <summary>
    /// Adds a literal Parameter to the list
    /// </summary>
    public void AddLiteral(ParamType ptype, TCollection_HAsciiString? val)
    {
        Reservate(_thenbparams + 1, _thenbstr + 1);
        int desc = (int)(ptype);
        _thenbstr++;
        _thenbparams++;
        _thevalues.SetValue(_thenbstr, val);
        desc += (_thenbstr << Content_NumberShift);
        _theparams.SetValue(_thenbparams, desc);
    }

    /// <summary>
    /// Adds a Parameter which references an Entity
    /// </summary>
    public void AddEntity(ParamType ptype, Transient? ent)
    {
        Reservate(_thenbparams + 1, 0);
        int desc = (int)(ptype);
        _theentities.Append(ent);
        desc += Content_LocalRef << Content_LocalShift; // "It is an Entity"
        _thenbparams++; // Rank: thenbparams - thenbstr
        desc += ((_thenbparams - _thenbstr) << Content_NumberShift);
        _theparams.SetValue(_thenbparams, desc);
    }

    /// <summary>
    /// Removes a Parameter given its rank
    /// </summary>
    public void RemoveParam(int num)
    {
        int desc = _theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        //    Remove an Entity
        if (c1ent)
        {
            _theentities.Remove(rang);
            //    Remove a Literal
        }
        else
        { // thevalues.Remove(rang) but it is an array
            for (int i = rang + 1; i <= _thenbstr; i++)
            {
                _thevalues.SetValue(i - 1, _thevalues.Value(i));
            }
            TCollection_HAsciiString nulstr = null;
            _thevalues.SetValue(_thenbstr, nulstr);
            _thenbstr--;
        }
        //    Remove this parameter from the list (which is an array)
        int np; // svv Jan11 2000 : porting on DEC
        for (np = num + 1; np <= _thenbparams; np++)
        {
            _theparams.SetValue(np - 1, _theparams.Value(np));
        }
        _theparams.SetValue(_thenbparams, 0);
        _thenbparams--;
        //    Renumber, Entity or Literal, depending
        for (np = 1; np <= _thenbparams; np++)
        {
            desc = _theparams.Value(np);
            if (((desc >> Content_LocalShift) & Content_LocalField) == local
                    && (desc >> Content_NumberShift) > rang)
            {
                _theparams.SetValue(np, desc - (1 << Content_NumberShift));
            }
        }
    }

    /// <summary>
    /// Sets a new value for the Parameter &lt;num&gt;, to a literal value (if it referenced formerly an Entity,
    /// this Entity is removed)
    /// </summary>
    public void SetLiteral(int num, ParamType ptype, TCollection_HAsciiString? val)
    {
        //  Change a parameter. If already literal, simple substitution
        //  If Entity, remove the entity and renumber the "Entity" Parameters
        int desc = _theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (c1ent)
        {
            //  Entity: remove it and renumber the "Entity" type Parameters
            _theentities.Remove(rang);
            for (int i = 1; i <= _thenbparams; i++)
            {
                desc = _theparams.Value(i);
                if (((desc >> Content_LocalShift) & Content_LocalField) == Content_LocalRef
                        && (desc >> Content_NumberShift) > rang)
                {
                    _theparams.SetValue(i, desc - (1 << Content_NumberShift));
                }
            }
            //  And prepare arrival of an additional Literal
            Reservate(_thenbparams, _thenbstr + 1);
            _thenbstr++;
            rang = _thenbstr;
        }
        //  Put the new value in place and rebuild the Param descriptor
        _thevalues.SetValue(rang, val);
        desc = (int)(ptype) + (rang << Content_NumberShift);
        _theparams.SetValue(num, desc);
    }

    /// <summary>
    /// Sets a new value for the Parameter &lt;num&gt;, to reference an Entity. To simply change the Entity, see
    /// the variant below
    /// </summary>
    public void SetEntity(int num, ParamType ptype, Transient? ent)
    {
        //  Change a Parameter. If already Entity, simple substitution
        //  If Literal, remove its value and renumber the Literal parameters
        int desc = _theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (!c1ent)
        {
            //  Literal: remove it and renumber the "Entity" type Parameters
            //  (Remove Literal but in an array)
            int i; // svv Jan11 2000 : porting on DEC
            for (i = rang + 1; i <= _thenbstr; i++)
            {
                _thevalues.SetValue(i - 1, _thevalues.Value(i));
            }
            TCollection_HAsciiString nulstr = null;
            _thevalues.SetValue(_thenbstr, nulstr);

            for (i = 1; i <= _thenbparams; i++)
            {
                desc = _theparams.Value(i);
                if (((desc >> Content_LocalShift) & Content_LocalField) == 0
                        && (desc >> Content_NumberShift) > rang)
                {
                    _theparams.SetValue(i, desc - (1 << Content_NumberShift));
                }
            }
            //  And prepare arrival of an additional Entity
            _thenbstr--;
            rang = _thenbparams - _thenbstr;
            //  Put the new value in place and rebuild the Param descriptor
            _theentities.Append(ent);
        }
        else
        {
            _theentities.SetValue(rang, ent);
        }

        desc = (int)(ptype) + (Content_LocalRef << Content_LocalShift) + (rang << Content_NumberShift);
        _theparams.SetValue(num, desc);
    }

    /// <summary>
    /// Changes the Entity referenced by the Parameter &lt;num&gt; (with same ParamType)
    /// </summary>
    public void SetEntity(int num, Transient? ent)
    {
        //  Change the Entity defined by a Parameter, all other things being equal,
        //  PROVIDED that it is already an "Entity" type Parameter
        int desc = _theparams.Value(num);
        int rang = desc >> Content_NumberShift;
        int local = ((desc >> Content_LocalShift) & Content_LocalField);
        bool c1ent = (local == Content_LocalRef);
        if (!c1ent)
        {
            throw new InterfaceErrorException("UndefinedContent : SetEntity");
        }
        _theentities.SetValue(rang, ent);
    }

    /// <summary>
    /// Returns globally the list of param entities. Note that it can be used as shared entity list for the
    /// UndefinedEntity
    /// </summary>
    public EntityList EntityList()
    {
        return _theentities;
    }

    /// <summary>
    /// Copies contents of undefined entities; deigned to be called by GetFromAnother method from Undefined
    /// entity of each Interface (the basic operation is the same regardless the norm)
    /// </summary>
    public void GetFromAnother(UndefinedContent? other, CopyTool tc)
    {
        int nb = other.ParamCount;
        _theentities.Clear();
        _thevalues = null;
        _theparams = null;
        Reservate(nb, other.LiteralCount);

        Transient ent = null;
        for (int i = 1; i <= nb; i++)
        {
            ParamType ptype = default;
            TCollection_HAsciiString val = null;
            if (other.ParamData(i, out ptype, out ent, out val))
            {
                ent = tc.Transferred(ent);
                AddEntity(ptype, ent);
            }
            else
            {
                AddLiteral(ptype, val);
            }
        }
    }

    private int _thenbparams;
    private int _thenbstr;
    private NCollection_HArray1<int>? _theparams;
    private NCollection_HArray1<TCollection_HAsciiString>? _thevalues;
    private EntityList _theentities = new EntityList();
}
