// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckIterator.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CheckIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// Result of a Check operation (especially from InterfaceModel)
/// </summary>
public class Interface_CheckIterator
{
    // OCCT's nulcheck(): one static empty check, returned for entities without a check. The port returns a new empty
    // check each time, so that a caller completing it can't change process-wide state (decision 1183).
    private static Interface_Check nulcheck() => new Interface_Check();

    //=================================================================================================

    /// <summary>
    /// Creates an empty CheckIterator
    /// </summary>
    public Interface_CheckIterator()
    {
        Clear();
    }

    /// <summary>
    /// Creates a CheckIterator with a name (displayed by Print as a title)
    /// </summary>
    public Interface_CheckIterator(string name)
    {
        thename = name;
        Clear();
    }

    /// <summary>
    /// Sets / Changes the name
    /// </summary>
    public void SetName(string name)
    {
        thename = "";
        if (name.Length > 0)
        {
            thename += name;
        }
    }

    /// <summary>
    /// Returns the recorded name (can be empty)
    /// </summary>
    public string Name()
    {
        return thename;
    }

    /// <summary>
    /// Defines a Model, used to locate entities (not required, if it is absent, entities are simply less
    /// documented)
    /// </summary>
    public void SetModel(Interface_InterfaceModel? model)
    {
        themod = model;
    }

    /// <summary>
    /// Returns the stored model (can be a null handle)
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themod;
    }

    /// <summary>
    /// Clears the list of checks
    /// </summary>
    public void Clear()
    {
        thelist = new NCollection_HSequence<Interface_Check>();
        thenums = new NCollection_HSequence<int>();
        thecurr = new Interface_IntVal();
        thecurr.CValue() = 1;
    }

    /// <summary>
    /// Merges another CheckIterator into &lt;me&gt;, i.e. adds each of its Checks. Content of &lt;other&gt;
    /// remains unchanged. Takes also the Model but not the Name
    /// </summary>
    public void Merge(Interface_CheckIterator other)
    {
        themod = other.Model();
        for (other.Start(); other.More(); other.Next())
        {
            Add(other.Value(), other.Number());
        }
    }

    /// <summary>
    /// Adds a Check to the list to be iterated This Check is Accompanied by Entity Number in the Model (0 for
    /// Global Check or Entity unknown in the Model), if 0 and Model is recorded in &lt;me&gt;, it is computed
    /// </summary>
    public void Add(Interface_Check? ach, int num = 0)
    {
        //  Add <same num as the last> . accumulate Checks
        if (ach.NbWarnings() + ach.NbFails() == 0)
        {
            return;
        }
        int nm = num;
        if (num <= 0 && ach.HasEntity())
        {
            if (themod is not null)
            {
                nm = themod.Number(ach.Entity());
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
        if (nm >= 0 && nm <= -(thecurr.Value()))
        {
            int i, numpos = 0, nb = thelist.Length();
            for (i = nb; i > 0; i--)
            {
                if (thenums.Value(i) == nm)
                {
                    numpos = i;
                    break;
                }
            }
            if (numpos > 0 && nm >= 0)
            {
                Interface_Check lch = thelist.ChangeValue(numpos);
                lch.GetMessages(ach);
            }
            //  Normal case: add at end of list
            else
            {
                thelist.Append(ach);
                thenums.Append(nm);
            }
        }
        //  Not yet seen passed: no need to search
        else
        {
            thelist.Append(ach);
            thenums.Append(nm);
            thecurr.CValue() = -nm;
        }
    }

    /// <summary>
    /// Returns the Check which was attached to an Entity given its Number in the Model. &lt;num&gt;=0 is for the
    /// Global Check. If no Check was recorded for this Number, returns an empty Check. Remark : Works apart from
    /// the iteration methods (no interference)
    /// </summary>
    public Interface_Check? Check(int num)
    {
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (num == thenums.Value(i))
            {
                return thelist.Value(i);
            }
        }
        return nulcheck();
    }

    /// <summary>
    /// Returns the Check attached to an Entity If no Check was recorded for this Entity, returns an empty Check.
    /// Remark : Works apart from the iteration methods (no interference)
    /// </summary>
    public Interface_Check? Check(Transient? ent)
    {
        int num = -1;
        if (themod is not null)
        {
            num = themod.Number(ent);
        }
        if (num > 0)
        {
            return Check(num);
        }

        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (ent == thelist.Value(i).Entity())
            {
                return thelist.Value(i);
            }
        }
        return nulcheck();
    }

    /// <summary>
    /// Returns the Check bound to an Entity Number (0 : Global) in order to be consulted or completed on the
    /// spot I.e. returns the Check if is already exists, or adds it then returns the new empty Check
    /// </summary>
    public ref Interface_Check? CCheck(int num)
    {
        int i, nb = thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (num == thenums.Value(i))
            {
                return ref thelist.ChangeValue(i);
            }
        }
        Interface_Check ach = new Interface_Check();
        thelist.Append(ach);
        thenums.Append(num);
        return ref thelist.ChangeValue(thelist.Length());
    }

    /// <summary>
    /// Returns the Check bound to an Entity, in order to be consulted or completed on the spot I.e. returns the
    /// Check if is already exists, or adds it then returns the new empty Check
    /// </summary>
    public ref Interface_Check? CCheck(Transient? ent)
    {
        int num = -1;
        if (themod is not null)
        {
            num = themod.Number(ent);
        }
        if (num > 0)
        {
            return ref CCheck(num);
        }

        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (ent == thelist.Value(i).Entity())
            {
                return ref thelist.ChangeValue(i);
            }
        }

        Interface_Check ach = new Interface_Check();
        thelist.Append(ach);
        thenums.Append(num);
        return ref thelist.ChangeValue(thelist.Length());
    }

    /// <summary>
    /// Returns True if : no Fail has been recorded if &lt;failsonly&gt; is True, no Check at all if
    /// &lt;failsonly&gt; is False
    /// </summary>
    public bool IsEmpty(bool failsonly)
    {
        if (thelist.IsEmpty())
        {
            return true;
        }
        if (!failsonly)
        {
            return false;
        }
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            if (thelist.Value(i).HasFailed())
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Returns worst status among : OK, Warning, Fail
    /// </summary>
    public Interface_CheckStatus Status()
    {
        Interface_CheckStatus stat = Interface_CheckOK;
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            Interface_Check ach = thelist.Value(i);
            if (ach.HasFailed())
            {
                return Interface_CheckFail;
            }
            if (ach.NbWarnings() > 0)
            {
                stat = Interface_CheckWarning;
            }
        }
        return stat;
    }

    /// <summary>
    /// Tells if this check list complies with a given status : OK (i.e. empty), Warning (at least one Warning,
    /// but no Fail), Fail (at least one), Message (not OK), NoFail, Any
    /// </summary>
    public bool Complies(Interface_CheckStatus stat)
    {
        bool res = (stat == Interface_CheckNoFail);
        int nb = thelist.Length();
        for (int i = 1; i <= nb; ++i)
        {
            Interface_Check ach = thelist.Value(i);
            int nbf = ach.NbFails(), nbw = ach.NbWarnings();
            switch (stat)
            {
                case Interface_CheckOK: {
                    if (nbf + nbw > 0)
                    {
                        return false;
                    }
                    break;
                }
                case Interface_CheckWarning: {
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
                case Interface_CheckFail: {
                    if (nbf > 0)
                    {
                        return true;
                    }
                    break;
                }
                case Interface_CheckAny: {
                    return true;
                }
                case Interface_CheckMessage: {
                    if (nbf + nbw > 0)
                    {
                        return true;
                    }
                    break;
                }
                case Interface_CheckNoFail: {
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
    public Interface_CheckIterator Extract(Interface_CheckStatus stat)
    {
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(themod);
        res.SetName(thename);
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            Interface_Check ach = thelist.Value(i);
            int nbf = ach.NbFails(), nbw = ach.NbWarnings();
            bool prend = false;
            switch (stat)
            {
                case Interface_CheckOK:
                    prend = (nbf + nbw == 0);
                    break;
                case Interface_CheckWarning:
                    prend = (nbf == 0 && nbw > 0);
                    break;
                case Interface_CheckFail:
                    prend = (nbf > 0);
                    break;
                case Interface_CheckAny:
                    prend = true;
                    break;
                case Interface_CheckMessage:
                    prend = (nbf + nbw > 0);
                    break;
                case Interface_CheckNoFail:
                    prend = (nbf == 0);
                    break;
                default:
                    break;
            }
            if (prend)
            {
                res.Add(ach, thenums.Value(i));
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
    public Interface_CheckIterator Extract(string mess, int incl, Interface_CheckStatus stat)
    {
        TCollection_HAsciiString str = new TCollection_HAsciiString(mess);
        Interface_CheckIterator res = new Interface_CheckIterator();
        res.SetModel(themod);
        res.SetName(thename);
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            Interface_Check ach = thelist.Value(i);
            if (ach.Complies(str, incl, stat))
            {
                res.Add(ach, thenums.Value(i));
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
    public bool Remove(string mess, int incl, Interface_CheckStatus stat)
    {
        TCollection_HAsciiString str = new TCollection_HAsciiString(mess);
        bool res = false;
        int i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            Interface_Check ach = thelist.ChangeValue(i);
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
        if (themod is null)
        {
            return list;
        }
        list = new NCollection_HSequence<Transient>();
        int num, i, nb = thelist.Length();
        for (i = 1; i <= nb; i++)
        {
            Interface_Check chk = thelist.Value(i);
            if (failsonly && !chk.HasFailed())
            {
                continue;
            }
            if (chk.NbWarnings() == 0)
            {
                continue;
            }
            num = thenums.Value(i);
            if (num == 0 && global)
            {
                list.Append(themod);
            }
            else if (num > 0)
            {
                list.Append(themod.Value(num));
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
        thecurr.CValue() = 1;
    }

    /// <summary>
    /// Returns True if there are more Checks to get
    /// </summary>
    public bool More()
    {
        if (thecurr.Value() < 0)
        {
            thecurr.CValue() = 1;
        }
        return (thecurr.Value() <= thelist.Length());
    }

    /// <summary>
    /// Sets Iteration to next Item
    /// </summary>
    public void Next()
    {
        if (thecurr.Value() < 0)
        {
            thecurr.CValue() = 1;
        }
        thecurr.CValue()++;
    }

    /// <summary>
    /// Returns Check currently Iterated It brings all other information (status, messages, ...) The Number of
    /// the Entity in the Model is given by Number below
    /// </summary>
    public Interface_Check? Value()
    {
        if (thecurr.Value() > thelist.Length())
        {
            throw new NoSuchObjectException("Interface Check Iterator : Value");
        }
        return thelist.Value(thecurr.Value());
    }

    /// <summary>
    /// Returns Number of Entity for the Check currently iterated or 0 for GlobalCheck
    /// </summary>
    public int Number()
    {
        if (thecurr.Value() > thenums.Length())
        {
            throw new NoSuchObjectException("Interface Check Iterator : Value");
        }
        return thenums.Value(thecurr.Value());
    }

    /// <summary>
    /// Prints the list of Checks with their attached Numbers If &lt;failsonly&gt; is True, prints only Fail
    /// messages If &lt;failsonly&gt; is False, prints all messages If &lt;final&gt; = 0 (D), prints also
    /// original messages if different If &lt;final&gt; &lt; 0, prints only original messages If &lt;final&gt;
    /// &gt; 0, prints only final messages It uses the recorded Model if it is defined Remark : Works apart from
    /// the iteration methods (no interference)
    /// </summary>
    public void Print(TextWriter S, bool failsonly, int final = 0)
    {
        Print(S, themod, failsonly, final);
    }

    /// <summary>
    /// Works as Print without a model, but for entities which have no attached number (Number not positive),
    /// tries to compute this Number from &lt;model&gt; and displays "original" or "computed"
    /// </summary>
    public void Print(TextWriter S, Interface_InterfaceModel? model, bool failsonly, int final = 0)
    {
        bool titre = false;
        /*string mesnum;
            string mesnum0 = ":";
            string mesnum1 = " (original):";
            string mesnum2 = " (computed):"; */
        int i, nb = thelist.Length(); //,j; svv #2
        bool yamod = model is not null;
        for (i = 1; i <= nb; i++)
        {
            Interface_Check ach = thelist.Value(i);
            int nbw = 0, nbf = ach.NbFails();
            if (!failsonly)
            {
                nbw = ach.NbWarnings();
            }
            if (nbf + nbw == 0)
            {
                continue;
            }
            Transient ent = ach.Entity();
            int nm0 = thenums.Value(i);
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
                S.Write(" **  ");
                S.Write(Name());
                S.Write("  **");
                S.Write('\n');
            }
            titre = true;
            S.Write("Check:");
            if (nb > 9 && i < 10)
            {
                S.Write(" ");
            }
            if (nb > 99 && i < 100)
            {
                S.Write(" ");
            }
            S.Write(i);
            if (num < 0)
            {
                S.Write(" -- Global Check");
                S.Write('\n');
            }
            else if (num == 0)
            {
                S.Write(" -- Entity n0 ??:");
            }
            else
            {
                if (yamod)
                {
                    S.Write(" -- Entity (n0:id) ");
                    model.Print(ent, S);
                }
                else
                {
                    S.Write(" -- Entity n0 ");
                    S.Write(num);
                }
                //      S<<" -- Entity n0 "<<num<<mesnum;
                //      if (yamod) model.PrintLabel(ent,S);
            }
            if (num >= 0 && entnul)
            {
                S.Write(" (unknown Type)");
                S.Write('\n');
            }
            else if (num >= 0 && !entnul)
            {
                if (yamod)
                {
                    S.Write("   Type:");
                    S.Write(model.TypeName(ent));
                    S.Write('\n');
                }
                else
                {
                    S.Write("   Type:");
                    S.Write(ent.DynamicTypeName());
                    S.Write('\n');
                }
            }

            ach.Print(S, (failsonly ? 1 : 3));
        }
        //  if (nbch > 0)  S<<" ----  Checks : "<<nbch<<"  ----"<<std.endl;
    }

    /// <summary>
    /// Clears data of iteration
    /// </summary>
    public void Destroy()
    {
        thecurr = null;
    }


    private NCollection_HSequence<Interface_Check>? thelist;
    private NCollection_HSequence<int>? thenums;
    private Interface_InterfaceModel? themod;
    private string thename = "";
    private Interface_IntVal? thecurr;
}
