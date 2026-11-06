// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckIterator.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Result of a Check operation (especially from InterfaceModel)
/// </summary>
internal class CheckIterator
{
    // OCCT's nulcheck(): one static empty check, returned for entities without a check. The port returns a new empty
    // check each time, so that a caller completing it can't change process-wide state (decision 1183).
    private static FrameworkCheck Nulcheck() => new FrameworkCheck();

    //=================================================================================================

    /// <summary>
    /// Creates an empty CheckIterator
    /// </summary>
    public CheckIterator()
    {
        Clear();
    }

    /// <summary>
    /// Creates a CheckIterator with a name (displayed by Print as a title)
    /// </summary>
    public CheckIterator(string name)
    {
        _thename = name;
        Clear();
    }

    /// <summary>
    /// Sets / Changes the name
    /// </summary>
    public void SetName(string name)
    {
        _thename = "";
        if (name.Length > 0)
        {
            _thename += name;
        }
    }

    /// <summary>
    /// Returns the recorded name (can be empty)
    /// </summary>
    public string Name()
    {
        return _thename;
    }

    /// <summary>
    /// Defines a Model, used to locate entities (not required, if it is absent, entities are simply less
    /// documented)
    /// </summary>
    public void SetModel(InterfaceModel? model)
    {
        _themod = model;
    }

    /// <summary>
    /// Returns the stored model (can be a null handle)
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themod;
    }

    /// <summary>
    /// Clears the list of checks
    /// </summary>
    public void Clear()
    {
        _thelist = new NCollection_HSequence<FrameworkCheck>();
        _thenums = new NCollection_HSequence<int>();
        _thecurr = new IntVal();
        _thecurr.CValue() = 1;
    }

    /// <summary>
    /// Merges another CheckIterator into &lt;me&gt;, i.e. adds each of its Checks. Content of &lt;other&gt;
    /// remains unchanged. Takes also the Model but not the Name
    /// </summary>
    public void Merge(CheckIterator other)
    {
        _themod = other.Model();
        for (other.Start(); other.More(); other.Next())
        {
            Add(other.Value(), other.Number());
        }
    }

    /// <summary>
    /// Adds a Check to the list to be iterated This Check is Accompanied by Entity Number in the Model (0 for
    /// Global Check or Entity unknown in the Model), if 0 and Model is recorded in &lt;me&gt;, it is computed
    /// </summary>
    public void Add(FrameworkCheck? ach, int num = 0)
    {
        //  Add <same num as the last> . accumulate Checks
        if (ach.WarningCount + ach.FailCount == 0)
        {
            return;
        }
        int nm = num;
        if (num <= 0 && ach.HasEntity)
        {
            if (_themod is not null)
            {
                nm = _themod.Number(ach.Entity());
                if (nm <= 0)
                {
                    nm = -1;
                }
            }
            else
            {
                nm = -1;
            }
        }
        if (nm >= 0 && nm <= -(_thecurr.Value()))
        {
            int i, numpos = 0, nb = _thelist.Length();
            for (i = nb; i > 0; i--)
            {
                if (_thenums.Value(i) == nm)
                {
                    numpos = i;
                    break;
                }
            }
            if (numpos > 0 && nm >= 0)
            {
                FrameworkCheck lch = _thelist.ChangeValue(numpos);
                lch.GetMessages(ach);
            }
            //  Normal case: add at end of list
            else
            {
                _thelist.Append(ach);
                _thenums.Append(nm);
            }
        }
        //  Not yet seen passed: no need to search
        else
        {
            _thelist.Append(ach);
            _thenums.Append(nm);
            _thecurr.CValue() = -nm;
        }
    }

    /// <summary>
    /// Returns the Check which was attached to an Entity given its Number in the Model. &lt;num&gt;=0 is for the
    /// Global Check. If no Check was recorded for this Number, returns an empty Check. Remark : Works apart from
    /// the iteration methods (no interference)
    /// </summary>
    public FrameworkCheck? Check(int num)
    {
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (num == _thenums.Value(i))
            {
                return _thelist.Value(i);
            }
        }
        return Nulcheck();
    }

    /// <summary>
    /// Returns the Check attached to an Entity If no Check was recorded for this Entity, returns an empty Check.
    /// Remark : Works apart from the iteration methods (no interference)
    /// </summary>
    public FrameworkCheck? Check(Transient? ent)
    {
        int num = -1;
        if (_themod is not null)
        {
            num = _themod.Number(ent);
        }
        if (num > 0)
        {
            return Check(num);
        }

        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (ent == _thelist.Value(i).Entity())
            {
                return _thelist.Value(i);
            }
        }
        return Nulcheck();
    }

    /// <summary>
    /// Returns the Check bound to an Entity Number (0 : Global) in order to be consulted or completed on the
    /// spot I.e. returns the Check if is already exists, or adds it then returns the new empty Check
    /// </summary>
    public ref FrameworkCheck? CCheck(int num)
    {
        int i, nb = _thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (num == _thenums.Value(i))
            {
                return ref _thelist.ChangeValue(i);
            }
        }
        FrameworkCheck ach = new FrameworkCheck();
        _thelist.Append(ach);
        _thenums.Append(num);
        return ref _thelist.ChangeValue(_thelist.Length());
    }

    /// <summary>
    /// Returns the Check bound to an Entity, in order to be consulted or completed on the spot I.e. returns the
    /// Check if is already exists, or adds it then returns the new empty Check
    /// </summary>
    public ref FrameworkCheck? CCheck(Transient? ent)
    {
        int num = -1;
        if (_themod is not null)
        {
            num = _themod.Number(ent);
        }
        if (num > 0)
        {
            return ref CCheck(num);
        }

        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (ent == _thelist.Value(i).Entity())
            {
                return ref _thelist.ChangeValue(i);
            }
        }

        FrameworkCheck ach = new FrameworkCheck();
        _thelist.Append(ach);
        _thenums.Append(num);
        return ref _thelist.ChangeValue(_thelist.Length());
    }

    /// <summary>
    /// Returns True if : no Fail has been recorded if &lt;failsonly&gt; is True, no Check at all if
    /// &lt;failsonly&gt; is False
    /// </summary>
    public bool IsEmpty(bool failsonly)
    {
        if (_thelist.IsEmpty())
        {
            return true;
        }
        if (!failsonly)
        {
            return false;
        }
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (_thelist.Value(i).HasFailed)
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Returns worst status among : OK, Warning, Fail
    /// </summary>
    public FrameworkCheckStatus Status()
    {
        FrameworkCheckStatus stat = FrameworkCheckStatus.CheckOK;
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck ach = _thelist.Value(i);
            if (ach.HasFailed)
            {
                return FrameworkCheckStatus.CheckFail;
            }
            if (ach.WarningCount > 0)
            {
                stat = FrameworkCheckStatus.CheckWarning;
            }
        }
        return stat;
    }

    /// <summary>
    /// Tells if this check list complies with a given status : OK (i.e. empty), Warning (at least one Warning,
    /// but no Fail), Fail (at least one), Message (not OK), NoFail, Any
    /// </summary>
    public bool Complies(FrameworkCheckStatus stat)
    {
        bool res = (stat == FrameworkCheckStatus.CheckNoFail);
        int nb = _thelist.Length();
        for (int i = 1; i <= nb; ++i)
        {
            FrameworkCheck ach = _thelist.Value(i);
            int nbf = ach.FailCount, nbw = ach.WarningCount;
            switch (stat)
            {
                case FrameworkCheckStatus.CheckOK: {
                    if (nbf + nbw > 0)
                    {
                        return false;
                    }
                    break;
                }
                case FrameworkCheckStatus.CheckWarning: {
                    if (nbf > 0)
                    {
                        return false;
                    }
                    if (nbw > 0)
                    {
                        res = true;
                    }
                    break;
                }
                case FrameworkCheckStatus.CheckFail: {
                    if (nbf > 0)
                    {
                        return true;
                    }
                    break;
                }
                case FrameworkCheckStatus.CheckAny: {
                    return true;
                }
                case FrameworkCheckStatus.CheckMessage: {
                    if (nbf + nbw > 0)
                    {
                        return true;
                    }
                    break;
                }
                case FrameworkCheckStatus.CheckNoFail: {
                    if (nbf > 0)
                    {
                        return false;
                    }
                    break;
                }
                default:
                    break;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns a CheckIterator which contains the checks which comply with a given status Each check is added
    /// completely (no split Warning/Fail)
    /// </summary>
    public CheckIterator Extract(FrameworkCheckStatus stat)
    {
        CheckIterator res = new CheckIterator();
        res.SetModel(_themod);
        res.SetName(_thename);
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck ach = _thelist.Value(i);
            int nbf = ach.FailCount, nbw = ach.WarningCount;
            bool prend = false;
            switch (stat)
            {
                case FrameworkCheckStatus.CheckOK:
                    prend = (nbf + nbw == 0);
                    break;
                case FrameworkCheckStatus.CheckWarning:
                    prend = (nbf == 0 && nbw > 0);
                    break;
                case FrameworkCheckStatus.CheckFail:
                    prend = (nbf > 0);
                    break;
                case FrameworkCheckStatus.CheckAny:
                    prend = true;
                    break;
                case FrameworkCheckStatus.CheckMessage:
                    prend = (nbf + nbw > 0);
                    break;
                case FrameworkCheckStatus.CheckNoFail:
                    prend = (nbf == 0);
                    break;
                default:
                    break;
            }
            if (prend)
            {
                res.Add(ach, _thenums.Value(i));
            }
        }
        return res;
    }

    /// <summary>
    /// Returns a CheckIterator which contains the check which comply with a message, plus some conditions as
    /// follows : &lt;incl&gt; = 0 : &lt;mess&gt; exactly matches one of the messages &lt;incl&gt; &lt; 0 :
    /// &lt;mess&gt; is contained by one of the messages &lt;incl&gt; &gt; 0 : &lt;mess&gt; contains one of the
    /// messages For &lt;status&gt; : for CheckWarning and CheckFail, considers only resp. Warning or Check
    /// messages. for CheckAny, considers all other values are ignored (answer will be false) Each Check which
    /// complies is entirely taken
    /// </summary>
    public CheckIterator Extract(string mess, int incl, FrameworkCheckStatus stat)
    {
        TCollection_HAsciiString str = new TCollection_HAsciiString(mess);
        CheckIterator res = new CheckIterator();
        res.SetModel(_themod);
        res.SetName(_thename);
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck ach = _thelist.Value(i);
            if (ach.Complies(str, incl, stat))
            {
                res.Add(ach, _thenums.Value(i));
            }
        }
        return res;
    }

    /// <summary>
    /// Removes the messages of all Checks, under these conditions : &lt;incl&gt; = 0 : &lt;mess&gt; exactly
    /// matches one of the messages &lt;incl&gt; &lt; 0 : &lt;mess&gt; is contained by one of the messages
    /// &lt;incl&gt; &gt; 0 : &lt;mess&gt; contains one of the messages For &lt;status&gt; : for CheckWarning and
    /// CheckFail, considers only resp. Warning or Check messages. for CheckAny, considers all other values are
    /// ignored (nothing is done) Returns True if at least one message has been removed, False else
    /// </summary>
    public bool Remove(string mess, int incl, FrameworkCheckStatus stat)
    {
        TCollection_HAsciiString str = new TCollection_HAsciiString(mess);
        bool res = false;
        int i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck ach = _thelist.ChangeValue(i);
            if (ach.Remove(str, incl, stat))
            {
                res = true;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns the list of entities concerned by a Check Only fails if &lt;failsonly&gt; is True, else all
    /// non-empty checks If &lt;global&gt; is true, adds the model for a global check Else, global check is
    /// ignored
    /// </summary>
    public NCollection_HSequence<Transient>? Checkeds(bool failsonly, bool global)
    {
        NCollection_HSequence<Transient> list = null;
        if (_themod is null)
        {
            return list;
        }
        list = new NCollection_HSequence<Transient>();
        int num, i, nb = _thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck chk = _thelist.Value(i);
            if (failsonly && !chk.HasFailed)
            {
                continue;
            }
            if (chk.WarningCount == 0)
            {
                continue;
            }
            num = _thenums.Value(i);
            if (num == 0 && global)
            {
                list.Append(_themod);
            }
            else if (num > 0)
            {
                list.Append(_themod.Value(num));
            }
        }
        return list;
    }

    /// <summary>
    /// Starts Iteration. Thus, it is possible to restart it Remark : an iteration may be done with a const
    /// Iterator While its content is modified (through a pointer), this allows to give it as a const argument to
    /// a function
    /// </summary>
    public void Start()
    {
        _thecurr.CValue() = 1;
    }

    /// <summary>
    /// Returns True if there are more Checks to get
    /// </summary>
    public bool More()
    {
        if (_thecurr.Value() < 0)
        {
            _thecurr.CValue() = 1;
        }
        return (_thecurr.Value() <= _thelist.Length());
    }

    /// <summary>
    /// Sets Iteration to next Item
    /// </summary>
    public void Next()
    {
        if (_thecurr.Value() < 0)
        {
            _thecurr.CValue() = 1;
        }
        _thecurr.CValue()++;
    }

    /// <summary>
    /// Returns Check currently Iterated It brings all other information (status, messages, ...) The Number of
    /// the Entity in the Model is given by Number below
    /// </summary>
    public FrameworkCheck? Value()
    {
        if (_thecurr.Value() > _thelist.Length())
        {
            throw new NoSuchObjectException("Interface Check Iterator : Value");
        }
        return _thelist.Value(_thecurr.Value());
    }

    /// <summary>
    /// Returns Number of Entity for the Check currently iterated or 0 for GlobalCheck
    /// </summary>
    public int Number()
    {
        if (_thecurr.Value() > _thenums.Length())
        {
            throw new NoSuchObjectException("Interface Check Iterator : Value");
        }
        return _thenums.Value(_thecurr.Value());
    }

    /// <summary>
    /// Prints the list of Checks with their attached Numbers If &lt;failsonly&gt; is True, prints only Fail
    /// messages If &lt;failsonly&gt; is False, prints all messages If &lt;final&gt; = 0 (D), prints also
    /// original messages if different If &lt;final&gt; &lt; 0, prints only original messages If &lt;final&gt;
    /// &gt; 0, prints only final messages It uses the recorded Model if it is defined Remark : Works apart from
    /// the iteration methods (no interference)
    /// </summary>
    public void Print(TextWriter s, bool failsonly, int final = 0)
    {
        Print(s, _themod, failsonly, final);
    }

    /// <summary>
    /// Works as Print without a model, but for entities which have no attached number (Number not positive),
    /// tries to compute this Number from &lt;model&gt; and displays "original" or "computed"
    /// </summary>
    public void Print(TextWriter s, InterfaceModel? model, bool failsonly, int final = 0)
    {
        bool titre = false;
        /*string mesnum;
            string mesnum0 = ":";
            string mesnum1 = " (original):";
            string mesnum2 = " (computed):"; */
        int i, nb = _thelist.Length(); //,j; svv #2
        bool yamod = model is not null;
        for (i = 1; i <= nb; i++)
        {
            FrameworkCheck ach = _thelist.Value(i);
            int nbw = 0, nbf = ach.FailCount;
            if (!failsonly)
            {
                nbw = ach.WarningCount;
            }
            if (nbf + nbw == 0)
            {
                continue;
            }
            Transient ent = ach.Entity();
            int nm0 = _thenums.Value(i);
            bool entnul = ent is null;
            int num = nm0;
            if (nm0 <= 0 && !entnul && yamod)
            {
                num = model.Number(ent);
            }
            if (nm0 <= 0 && entnul)
            {
                num = -1; // Global
            }
            //  mesnum = mesnum0;
            //    if (yamod) mesnum = (nm0 > 0 ? mesnum1 : mesnum2);

            if (!titre)
            {
                s.Write(" **  ");
                s.Write(Name());
                s.Write("  **");
                s.Write('\n');
            }
            titre = true;
            s.Write("Check:");
            if (nb > 9 && i < 10)
            {
                s.Write(" ");
            }
            if (nb > 99 && i < 100)
            {
                s.Write(" ");
            }
            s.Write(i);
            if (num < 0)
            {
                s.Write(" -- Global Check");
                s.Write('\n');
            }
            else if (num == 0)
            {
                s.Write(" -- Entity n0 ??:");
            }
            else
            {
                if (yamod)
                {
                    s.Write(" -- Entity (n0:id) ");
                    model.Print(ent, s);
                }
                else
                {
                    s.Write(" -- Entity n0 ");
                    s.Write(num);
                }
                //      S<<" -- Entity n0 "<<num<<mesnum;
                //      if (yamod) model.PrintLabel(ent,S);
            }
            if (num >= 0 && entnul)
            {
                s.Write(" (unknown Type)");
                s.Write('\n');
            }
            else if (num >= 0 && !entnul)
            {
                if (yamod)
                {
                    s.Write("   Type:");
                    s.Write(model.TypeName(ent));
                    s.Write('\n');
                }
                else
                {
                    s.Write("   Type:");
                    s.Write(ent.DynamicTypeName());
                    s.Write('\n');
                }
            }

            ach.Print(s, (failsonly ? 1 : 3));
        }
        //  if (nbch > 0)  S<<" ----  Checks : "<<nbch<<"  ----"<<std.endl;
    }

    /// <summary>
    /// Clears data of iteration
    /// </summary>
    public void Destroy()
    {
        _thecurr = null;
    }


    private NCollection_HSequence<FrameworkCheck>? _thelist;
    private NCollection_HSequence<int>? _thenums;
    private InterfaceModel? _themod;
    private string _thename = "";
    private IntVal? _thecurr;
}
