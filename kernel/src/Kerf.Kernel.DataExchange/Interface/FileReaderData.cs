// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileReaderData.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileReaderData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class defines services which permit to access Data issued from a File, in a form which does not
/// depend of physical format : thus, each Record has an attached ParamList (to be managed) and resulting
/// Entity. Each Interface defines its own FileReaderData : on one hand by defining deferred methods given
/// here, on the other hand by describing literal data and their accesses, with the help of basic classes
/// such as String, Array1OfString, etc... FileReaderData is used by a FileReaderTool, which is also specific
/// of each Norm, to read an InterfaceModel of the Norm FileReaderData inherits TShared to be accessed by
/// Handle : this allows FileReaderTool to define more easily the specific methods, and improves memory
/// management.
/// </summary>
internal abstract class FileReaderData : Transient
{
    //  Stores Data from a File (Preserved in Literal form)
    //  Each standard can use it as a base (literal parameter lists,
    //  associated entities) and add its own data to it.
    //  Works under the control of FileReaderTool

    /// <summary>
    /// Returns the count of registered records. That is, value given for Initialization (can be redefined).
    /// </summary>
    public virtual int RecordCount
    {
        get
        {
            return _thenumpar.Upper();
        }
    }

    /// <summary>
    /// Returns the count of entities. Depending of each norm, records can be Entities or SubParts (SubList in
    /// STEP, SubGroup in SET ...). NbEntities counts only Entities, not Subs Used for memory reservation in
    /// InterfaceModel Default implementation uses FindNextRecord Can be redefined into a more performant way
    /// </summary>
    public virtual int NbEntities()
    {
        int nb = 0;
        int num = 0;
        while ((num = FindNextRecord(num)) > 0)
        {
            nb++;
        }
        return nb;
    }

    /// <summary>
    /// Determines the record number defining an Entity following a given record number. Specific to each
    /// sub-class of FileReaderData. Returning zero means no record found
    /// </summary>
    public abstract int FindNextRecord(int num);

    /// <summary>
    /// attaches an empty ParamList to a Record
    /// </summary>
    public void InitParams(int num)
    {
        _thenumpar.SetValue(num, _theparams.ParamCount);
    }

    /// <summary>
    /// Adds a parameter to record no "num" and fills its fields (EntityNumber is optional) Warning :
    /// &lt;aval&gt; is assumed to be memory-managed elsewhere : it is NOT copied. This gives a best speed :
    /// strings remain stored in pages of characters
    /// </summary>

    /// <summary>
    /// Same as above, but gets a AsciiString from TCollection Remark that the content of the AsciiString is
    /// locally copied (because its content is most often lost after using)
    /// </summary>
    public void AddParam(int num, string aval, ParamType atype, int nument = 0)
    {
        _theparams.Append(aval, aval.Length, atype, nument);
    }

    /// <summary>
    /// Same as above, but gets a complete FileParameter Warning : Content of &lt;FP&gt; is NOT copied : its
    /// original address and space in memory are assumed to be managed elsewhere (see ParamSet)
    /// </summary>
    public void AddParam(int num, FileParameter fp)
    {
        _theparams.Append(fp);
    }

    /// <summary>
    /// Sets a new value for a parameter of a record, given by : num : record number; nump : parameter number in
    /// the record
    /// </summary>
    public void SetParam(int num, int nump, FileParameter fp)
    {
        _theparams.SetParam(_thenumpar[num - 1] + nump, fp);
    }

    /// <summary>
    /// Returns count of parameters attached to record "num" If &lt;num&gt; = 0, returns the total recorded count
    /// of parameters
    /// </summary>
    public int NbParams(int num)
    {
        if (num > 1)
        {
            return (_thenumpar[num] - _thenumpar[num - 1]);
        }
        else if (num == 1)
        {
            return _thenumpar[num];
        }
        else
        {
            return _theparams.ParamCount;
        }
    }

    /// <summary>
    /// Returns the complete ParamList of a record (read only) num = 0 to return the whole param list for the
    /// file
    /// </summary>
    public ParamList? Params(int num)
    {
        if (num == 0)
        {
            return _theparams.Params(0, 0); // complet
        }
        else if (num == 1)
        {
            return _theparams.Params(0, _thenumpar[1]);
        }
        else
        {
            return _theparams.Params(_thenumpar[num - 1] + 1, (_thenumpar[num] - _thenumpar[num - 1]));
        }
    }

    /// <summary>
    /// Returns parameter "nump" of record "num", as a complete FileParameter
    /// </summary>
    public FileParameter Param(int num, int nump)
    {
        // Kerf: OCCT's cache of the last record is process-wide (statics), active for the last file created only;
        // it is this object's own here (decision 1183), with the same results
        if (_thenm0 != num)
        if (_thenm0 != num)
        {
            _thenp0 = _thenumpar[num - 1];
            _thenm0 = num;
        }
        return _theparams.Param(_thenp0 + nump);
    }

    /// <summary>
    /// Same as above, but in order to be modified on place
    /// </summary>
    public ref FileParameter ChangeParam(int num, int nump)
    {
        if (_thenm0 != num)
        if (_thenm0 != num)
        {
            _thenp0 = _thenumpar[num - 1];
            _thenm0 = num;
        }
        return ref _theparams.ChangeParam(_thenp0 + nump);
    }

    /// <summary>
    /// Returns type of parameter "nump" of record "num" Returns literal value of parameter "nump" of record
    /// "num" was C++ : return const &amp;
    /// </summary>
    public ParamType ParamType(int num, int nump)
    {
        return Param(num, nump).ParamType();
    }

    /// <summary>
    /// Same as above, but as a CString was C++ : return const
    /// </summary>
    public string ParamCValue(int num, int nump)
    {
        return Param(num, nump).CValue();
    }

    /// <summary>
    /// Returns True if parameter "nump" of record "num" is defined (it is not if its type is ParamVoid)
    /// </summary>
    public bool IsParamDefined(int num, int nump)
    {
        return (Param(num, nump).ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid);
    }

    /// <summary>
    /// Returns record number of an entity referenced by a parameter of type Ident; 0 if no EntityNumber has been
    /// determined Note that it is used to reference Entities but also Sublists (sublists are not objects, but
    /// internal descriptions)
    /// </summary>
    public int ParamNumber(int num, int nump)
    {
        return Param(num, nump).EntityNumber();
    }

    /// <summary>
    /// Returns the StepEntity referenced by a parameter Error if none
    /// </summary>
    public Transient? ParamEntity(int num, int nump)
    {
        return BoundEntity(Param(num, nump).EntityNumber());
    }

    /// <summary>
    /// Returns the absolute rank of the beginning of a record (its list is from ParamFirstRank+1 to
    /// ParamFirstRank+NbParams)
    /// </summary>
    public int ParamFirstRank(int num)
    {
        return _thenumpar[num];
    }

    /// <summary>
    /// Returns the entity bound to a record, set by SetEntities
    /// </summary>
    public Transient? BoundEntity(int num)
    {
        if (num >= _theents.Lower() && num <= _theents.Upper())
        {
            return _theents[num];
        }
        else
        {
            return null; // OCCT's static null "dummy"
        }
    }

    /// <summary>
    /// Binds an entity to a record
    /// </summary>
    public void BindEntity(int num, Transient? ent)
    {
        _theents.SetValue(num,ent);
    }

    /// <summary>
    /// Sets the status "Error Load" on, to overside check fails &lt;val&gt; True : declares unloaded &lt;val&gt;
    /// False : declares loaded If not called before loading (see FileReaderTool), check fails give the status
    /// IsErrorLoad says if SetErrorLoad has been called by user ResetErrorLoad resets it (called by
    /// FileReaderTool) This allows to specify that the currently loaded entity remains unloaded (because of
    /// syntactic fail)
    /// </summary>
    public void SetErrorLoad(bool val)
    {
        _therrload = (val ? 1 : -1);
    }

    /// <summary>
    /// Returns True if the status "Error Load" has been set (to True or False)
    /// </summary>
    public bool IsErrorLoad
    {
        get
        {
            return (_therrload != 0);
        }
    }

    /// <summary>
    /// Returns the former value of status "Error Load" then resets it Used to read the status then ensure it is
    /// reset
    /// </summary>
    public bool ResetErrorLoad()
    {
        bool res = (_therrload > 0);
        _therrload = 0;
        return res;
    }

    /// <summary>
    /// Destructor (waiting for memory management)
    /// </summary>
    public void Destroy()
    {

    }


    /// <summary>
    /// Same spec.s as standard &lt;atof&gt; but 5 times faster
    /// </summary>
    public static double Fastof(string ligne)
    {
        return StreamUtilities.Strtod(ligne);
    }

    /// <summary>
    /// Initializes arrays of Entities and of ParamLists attached to registered records &lt;nbr&gt; must be the
    /// maximum number of records to get (no way to extend it at run-time) : count entities and sub-entities ...
    /// &lt;npar&gt; is the total count of parameters (if it is not exact, it will be extended as necessary)
    /// Hence, to each record can be bound an Entity and a list of Parameters. Each kind of FileReaderData can
    /// add other data, by having them in parallel (other arrays with same sizes) Else, it must manage binding
    /// between items and their data
    /// </summary>
    protected internal FileReaderData(int nbr, int npar)
    {
        _therrload = 0;
        _thenumpar = new NCollection_Array1<int>(0, nbr);
        _theents = new NCollection_Array1<Transient>(0, nbr);
        _theparams = new ParamSet(npar);
        _theparams.Reserve();
        _thenumpar.Init(0);
        _thenm0 = -1;
    }

    /// <summary>
    /// Returns a parameter given its absolute rank in the file in order to be consulted or modified in
    /// specilaized actions
    /// </summary>
    protected internal ref FileParameter ChangeParameter(int numpar)
    {
        return ref _theparams.ChangeParam(numpar);
    }

    /// <summary>
    /// For a given absolute rank of parameter, determines the record to which its belongs, and the parameter
    /// number for it
    /// </summary>
    protected internal void ParamPosition(int numpar, out int num, out int nump)
    {
        int nbe = _thenumpar.Upper();
        if (numpar <= 0)
        {
            num = nump = 0;
            return;
        }
        for (int i = 1; i <= nbe; i++)
        {
            if (_thenumpar[i] > numpar)
            {
                num = i;
                nump = numpar - _thenumpar[i] + 1;
                return;
            }
        }
        num = nbe;
        nump = numpar - _thenumpar[nbe] + 1;
    }

    private int _thenm0 = -1;
    private int _thenp0 = -1;
    private int _therrload;
    private ParamSet _theparams;
    private NCollection_Array1<int> _thenumpar;
    private NCollection_Array1<Transient?> _theents;
}
