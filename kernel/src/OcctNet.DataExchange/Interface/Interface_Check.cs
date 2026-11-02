// Port of: src/DataExchange/TKXSBase/Interface/Interface_Check.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Check.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// Defines a Check, as a list of Fail or Warning Messages under a literal form, which can be empty. A Check
/// can also bring an Entity, which is the Entity to which the messages apply (this Entity may be any
/// Transient Object). Messages can be stored in two forms : the definitive form (the only one by default),
/// and another form, the original form, which can be different if it contains values to be inserted
/// (integers, reals, strings) The original form can be more suitable for some operations such as counting
/// messages
/// </summary>
public class Interface_Check : Standard_Transient
{

    /// <summary>
    /// Allows definition of a Sequence. Used also for Global Check of an InterfaceModel (which stores global
    /// messages for file)
    /// </summary>
    public Interface_Check()
    {
    }

    /// <summary>
    /// Defines a Check on an Entity
    /// </summary>
    public Interface_Check(Standard_Transient? anentity)
    {
        theent = anentity;
    }

    /// <summary>
    /// New name for AddFail (Msg)
    /// </summary>
    public void SendFail(Message_Msg amsg)
    {
        AddFail(amsg);
    }

    /// <summary>
    /// Records a new Fail message
    /// </summary>
    public void AddFail(TCollection_HAsciiString? mess)
    {
        if (thefails is null)
        {
            thefails = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        if (thefailo is null)
        {
            thefailo = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        thefails.Append(mess);
        thefailo.Append(mess);
    }

    /// <summary>
    /// Records a new Fail message under two forms : final,original
    /// </summary>
    public void AddFail(TCollection_HAsciiString? mess, TCollection_HAsciiString? orig)
    {
        if (thefails is null)
        {
            thefails = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        if (thefailo is null)
        {
            thefailo = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        thefails.Append(mess);
        thefailo.Append(orig);
    }

    /// <summary>
    /// Records a new Fail message given as "error text" directly If &lt;orig&gt; is given, a distinct original
    /// form is recorded else (D), the original form equates &lt;amess&gt;
    /// </summary>
    public void AddFail(string amess, string orig = "")
    {
        if (amess.Length == 0)
        {
            return;
        }
        if (orig is null || orig.Length == 0)
        {
            AddFail(new TCollection_HAsciiString(amess));
        }
        else
        {
            AddFail(new TCollection_HAsciiString(amess), new TCollection_HAsciiString(orig));
        }
    }

    /// <summary>
    /// Records a new Fail from the definition of a Msg (Original+Value)
    /// </summary>
    public void AddFail(Message_Msg amsg)
    {
        if (amsg.IsEdited())
        {
            AddFail(new TCollection_HAsciiString((amsg.Value())),
                            new TCollection_HAsciiString((amsg.Original())));
        }
        else
        {
            AddFail(new TCollection_HAsciiString((amsg.Value())));
        }
    }

    /// <summary>
    /// Returns True if Check brings at least one Fail Message
    /// </summary>
    public bool HasFailed()
    {
        return (thefails is not null);
    }

    /// <summary>
    /// Returns count of recorded Fails
    /// </summary>
    public int NbFails()
    {
        return (thefails is null ? 0 : thefails.Length());
    }

    /// <summary>
    /// Returns Fail Message as a String Final form by default, Original form if &lt;final&gt; is False
    /// </summary>
    public TCollection_HAsciiString? Fail(int num, bool final = true)
    {
        if (thefails is null)
        {
            throw new Standard_OutOfRange();
        }
        return (final ? thefails.Value(num) : thefailo.Value(num));
    }

    /// <summary>
    /// Same as above, but returns a CString (to be printed ...) Final form by default, Original form if
    /// &lt;final&gt; is False
    /// </summary>
    public string CFail(int num, bool final = true)
    {
        return Fail(num, final).ToCString();
    }

    /// <summary>
    /// Returns the list of Fails, for a frontal-engine logic Final forms by default, Original forms if
    /// &lt;final&gt; is False Can be empty
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? Fails(bool final = true)
    {
        if (thefails is null)
        {
            return new NCollection_HSequence<TCollection_HAsciiString>();
        }
        return (final ? thefails : thefailo);
    }

    /// <summary>
    /// New name for AddWarning
    /// </summary>
    public void SendWarning(Message_Msg amsg)
    {
        AddWarning(amsg);
    }

    /// <summary>
    /// Records a new Warning message
    /// </summary>
    public void AddWarning(TCollection_HAsciiString? mess)
    {
        if (thewarns is null)
        {
            thewarns = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        if (thewarno is null)
        {
            thewarno = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        thewarns.Append(mess);
        thewarno.Append(mess);
    }

    /// <summary>
    /// Records a new Warning message under two forms : final,original
    /// </summary>
    public void AddWarning(TCollection_HAsciiString? mess, TCollection_HAsciiString? orig)
    {
        if (thewarns is null)
        {
            thewarns = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        if (thewarno is null)
        {
            thewarno = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        thewarns.Append(mess);
        thewarno.Append(orig);
    }

    /// <summary>
    /// Records a Warning message given as "warning message" directly If &lt;orig&gt; is given, a distinct
    /// original form is recorded else (D), the original form equates &lt;amess&gt;
    /// </summary>
    public void AddWarning(string amess, string orig = "")
    {
        if (amess.Length == 0)
        {
            return;
        }
        if (orig is null || orig.Length == 0)
        {
            AddWarning(new TCollection_HAsciiString(amess));
        }
        else
        {
            AddWarning(new TCollection_HAsciiString(amess), new TCollection_HAsciiString(orig));
        }
    }

    /// <summary>
    /// Records a new Warning from the definition of a Msg (Original+Value)
    /// </summary>
    public void AddWarning(Message_Msg amsg)
    {
        if (amsg.IsEdited())
        {
            AddWarning(new TCollection_HAsciiString((amsg.Value())),
                                  new TCollection_HAsciiString((amsg.Original())));
        }
        else
        {
            AddWarning(new TCollection_HAsciiString((amsg.Value())));
        }
    }

    /// <summary>
    /// Returns True if Check brings at least one Warning Message
    /// </summary>
    public bool HasWarnings()
    {
        return (thewarns is not null);
    }

    /// <summary>
    /// Returns count of recorded Warning messages
    /// </summary>
    public int NbWarnings()
    {
        return (thewarns is null ? 0 : thewarns.Length());
    }

    /// <summary>
    /// Returns Warning message as a String Final form by default, Original form if &lt;final&gt; is False
    /// </summary>
    public TCollection_HAsciiString? Warning(int num, bool final = true)
    {
        if (thewarns is null)
        {
            throw new Standard_OutOfRange();
        }
        return (final ? thewarns.Value(num) : thewarno.Value(num));
    }

    /// <summary>
    /// Same as above, but returns a CString (to be printed ...) Final form by default, Original form if
    /// &lt;final&gt; is False
    /// </summary>
    public string CWarning(int num, bool final = true)
    {
        return Warning(num, final).ToCString();
    }

    /// <summary>
    /// Returns the list of Warnings, for a frontal-engine logic Final forms by default, Original forms if
    /// &lt;final&gt; is False Can be empty
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? Warnings(bool final = true)
    {
        if (thewarns is null)
        {
            return new NCollection_HSequence<TCollection_HAsciiString>();
        }
        return (final ? thewarns : thewarno);
    }

    /// <summary>
    /// Records an information message This does not change the status of the Check
    /// </summary>
    public void SendMsg(Message_Msg amsg)
    {
        TCollection_HAsciiString mess = new TCollection_HAsciiString(amsg.Value());
        TCollection_HAsciiString orig = mess;
        if (amsg.IsEdited())
        {
            orig = new TCollection_HAsciiString(amsg.Original());
        }

        if (theinfos is null)
        {
            theinfos = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        if (theinfoo is null)
        {
            theinfoo = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        theinfos.Append(mess);
        theinfoo.Append(mess);
    }

    /// <summary>
    /// Returns the count of recorded information messages
    /// </summary>
    public int NbInfoMsgs()
    {
        return (theinfos is null ? 0 : theinfos.Length());
    }

    /// <summary>
    /// Returns information message as a String
    /// </summary>
    public TCollection_HAsciiString? InfoMsg(int num, bool final = true)
    {
        if (theinfos is null)
        {
            throw new Standard_OutOfRange();
        }
        return (final ? theinfos.Value(num) : theinfoo.Value(num));
    }

    /// <summary>
    /// Same as above, but returns a CString (to be printed ...) Final form by default, Original form if
    /// &lt;final&gt; is False
    /// </summary>
    public string CInfoMsg(int num, bool final = true)
    {
        return InfoMsg(num, final).ToCString();
    }

    /// <summary>
    /// Returns the list of Info Msg, for a frontal-engine logic Final forms by default, Original forms if
    /// &lt;final&gt; is False Can be empty
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? InfoMsgs(bool final = true)
    {
        if (theinfos is null)
        {
            return new NCollection_HSequence<TCollection_HAsciiString>();
        }
        return (final ? theinfos : theinfoo);
    }

    /// <summary>
    /// Returns the Check Status : OK, Warning or Fail
    /// </summary>
    public Interface_CheckStatus Status()
    {
        if (thefails is not null)
        {
            return Interface_CheckFail;
        }
        if (thewarns is not null)
        {
            return Interface_CheckWarning;
        }
        return Interface_CheckOK;
    }

    /// <summary>
    /// Tells if Check Status complies with a given one (i.e. also status for query)
    /// </summary>
    public bool Complies(Interface_CheckStatus status)
    {
        bool now = thewarns is null;
        bool nof = thefails is null;
        switch (status)
        {
            case Interface_CheckOK:
                return (nof && now);
            case Interface_CheckWarning:
                return (nof && !now);
            case Interface_CheckFail:
                return !nof;
            case Interface_CheckAny:
                return true;
            case Interface_CheckMessage:
                return (!nof || !now);
            case Interface_CheckNoFail:
                return nof;
            default:
                break;
        }
        return false;
    }

    /// <summary>
    /// Tells if a message is brought by a Check, as follows : &lt;incl&gt; = 0 : &lt;mess&gt; exactly matches
    /// one of the messages &lt;incl&gt; &lt; 0 : &lt;mess&gt; is contained by one of the messages &lt;incl&gt;
    /// &gt; 0 : &lt;mess&gt; contains one of the messages For &lt;status&gt; : for CheckWarning and CheckFail,
    /// considers only resp. Warning or Check messages. for CheckAny, considers all other values are ignored
    /// (answer will be false)
    /// </summary>
    public bool Complies(TCollection_HAsciiString? mess, int incl, Interface_CheckStatus status)
    {
        if (mess is null)
        {
            return false;
        }
        int lng = mess.Length();
        if (status == Interface_CheckWarning || status == Interface_CheckAny)
        {
            int i, nb = NbWarnings();
            for (i = nb; i > 0; i--)
            {
                TCollection_HAsciiString ames = Warning(i);
                if (incl == 0)
                {
                    if (mess.IsSameString(ames))
                    {
                        return true;
                    }
                }
                if (incl < 0)
                {
                    if (ames.Location(mess, 1, ames.Length()) > 0)
                    {
                        return true;
                    }
                }
                if (incl < 0)
                {
                    if (mess.Location(mess, 1, lng) > 0)
                    {
                        return true;
                    }
                }
            }
        }
        if (status == Interface_CheckFail || status == Interface_CheckAny)
        {
            int i, nb = NbWarnings(); // OCCT: the fails branch looks at the warnings (copied from above)
            for (i = nb; i > 0; i--)
            {
                TCollection_HAsciiString ames = Warning(i);
                if (incl == 0)
                {
                    if (mess.IsSameString(ames))
                    {
                        return true;
                    }
                }
                if (incl < 0)
                {
                    if (ames.Location(mess, 1, ames.Length()) > 0)
                    {
                        return true;
                    }
                }
                if (incl < 0)
                {
                    if (mess.Location(mess, 1, lng) > 0)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Returns True if a Check is devoted to an entity; else, it is global (for InterfaceModel's storing of
    /// global error messages)
    /// </summary>
    public bool HasEntity()
    {
        return (theent is not null);
    }

    /// <summary>
    /// Returns the entity on which the Check has been defined
    /// </summary>
    public Standard_Transient? Entity()
    {
        return theent;
    }

    /// <summary>
    /// Clears a check, in order to receive information from transfer (Messages and Entity)
    /// </summary>
    public void Clear()
    {
        thefails = null;
        thefailo = null;
        thewarns = null;
        thewarno = null;
        theinfos = null;
        theinfoo = null;
        theent = null;
    }

    /// <summary>
    /// Clears the Fail Messages (for instance to keep only Warnings)
    /// </summary>
    public void ClearFails()
    {
        thefails = null;
        thefailo = null;
    }

    /// <summary>
    /// Clears the Warning Messages (for instance to keep only Fails)
    /// </summary>
    public void ClearWarnings()
    {
        thewarns = null;
        thewarno = null;
    }

    /// <summary>
    /// Clears the Info Messages
    /// </summary>
    public void ClearInfoMsgs()
    {
        theinfos = null;
        theinfoo = null;
    }

    /// <summary>
    /// Removes the messages which comply with &lt;mess&gt;, as follows : &lt;incl&gt; = 0 : &lt;mess&gt; exactly
    /// matches one of the messages &lt;incl&gt; &lt; 0 : &lt;mess&gt; is contained by one of the messages
    /// &lt;incl&gt; &gt; 0 : &lt;mess&gt; contains one of the messages For &lt;status&gt; : for CheckWarning and
    /// CheckFail, considers only resp. Warning or Check messages. for CheckAny, considers all other values are
    /// ignored (nothing is done) Returns True if at least one message has been removed, False else
    /// </summary>
    public bool Remove(TCollection_HAsciiString? mess, int incl, Interface_CheckStatus status)
    {
        if (mess is null)
        {
            return false;
        }
        bool res = false;
        int lng = mess.Length();
        if (status == Interface_CheckWarning || status == Interface_CheckAny)
        {
            int i, nb = NbWarnings();
            for (i = nb; i > 0; i--)
            {
                bool rem = false;
                TCollection_HAsciiString ames = Warning(i);
                if (incl == 0)
                {
                    rem = mess.IsSameString(ames);
                }
                if (incl < 0)
                {
                    rem = (ames.Location(mess, 1, ames.Length()) > 0);
                }
                if (incl < 0)
                {
                    rem = (mess.Location(mess, 1, lng) > 0);
                }
                if (rem)
                {
                    thewarns.Remove(i);
                    thewarno.Remove(i);
                    res = rem;
                }
            }
        }
        if (status == Interface_CheckFail || status == Interface_CheckAny)
        {
            int i, nb = NbWarnings(); // OCCT: the fails branch looks at the warnings (copied from above)
            for (i = nb; i > 0; i--)
            {
                bool rem = false;
                TCollection_HAsciiString ames = Warning(i);
                if (incl == 0)
                {
                    rem = mess.IsSameString(ames);
                }
                if (incl < 0)
                {
                    rem = (ames.Location(mess, 1, ames.Length()) > 0);
                }
                if (incl < 0)
                {
                    rem = (mess.Location(mess, 1, lng) > 0);
                }
                if (rem)
                {
                    thefails.Remove(i);
                    thefailo.Remove(i);
                    res = rem;
                }
            }
        }
        return res;
    }

    /// <summary>
    /// Mends messages, according &lt;pref&gt; and &lt;num&gt; According to &lt;num&gt;, works on the whole list
    /// of Fails if = 0(D) or only one Fail message, given its rank If &lt;pref&gt; is empty, converts Fail(s) to
    /// Warning(s) Else, does the conversion but prefixes the new Warning(s) but &lt;pref&gt; followed by a
    /// semi-column Some reserved values of &lt;pref&gt; are : "FM" : standard prefix "Mended" (can be
    /// translated) "CF" : clears Fail(s) "CW" : clears Warning(s) : here, &lt;num&gt; refers to Warning list
    /// "CA" : clears all messages : here, &lt;num&gt; is ignored
    /// </summary>
    public bool Mend(string pref, int num = 0)
    {
        int i, n1 = num, n2 = num;
        if (pref is not null && CStringChar(pref, 2) == '\0')
        {
            if (CStringChar(pref, 0) == 'F' && CStringChar(pref, 1) == 'M')
            {
                return Mend("Mended", num);
            }
            if (CStringChar(pref, 0) == 'C' && CStringChar(pref, 1) == 'A')
            {
                Clear();
                return true;
            }
            if (CStringChar(pref, 0) == 'C' && CStringChar(pref, 1) == 'W')
            {
                if (num == 0)
                {
                    ClearWarnings();
                    return true;
                }
                if (num < 0 || num > NbWarnings())
                {
                    return false;
                }
                thewarns.Remove(num);
                thewarno.Remove(num);
                return true;
            }
            if (CStringChar(pref, 0) == 'C' && CStringChar(pref, 1) == 'F')
            {
                if (num == 0)
                {
                    ClearFails();
                    return true;
                }
                if (num < 0 || num > NbFails())
                {
                    return false;
                }
                thefails.Remove(num);
                thefailo.Remove(num);
                return true;
            }
        }
        //  Common case
        if (num == 0)
        {
            n1 = 1;
            n2 = NbFails();
            for (i = n2; i >= n1; i--)
            {
                Mend(pref, i);
            }
            return true;
        }
        else if (num < 0 || num > NbFails())
        {
            return false;
        }
        //  A message
        TCollection_HAsciiString strf = thefails.Value(num);
        TCollection_HAsciiString stro = thefailo.Value(num);
        if (pref is not null && pref.Length > 0)
        {
            strf.Insert(1, " : ");
            strf.Insert(1, pref);
            if (stro != strf)
            {
                stro.Insert(1, " : ");
                stro.Insert(1, pref);
            }
        }
        thefails.Remove(num);
        thefailo.Remove(num);
        if (stro == strf)
        {
            AddFail(strf);
        }
        else
        {
            AddFail(strf, stro);
        }
        return true;
    }

    /// <summary>
    /// Receives an entity result of a Transfer
    /// </summary>
    public void SetEntity(Standard_Transient? anentity)
    {
        theent = anentity;
    }

    /// <summary>
    /// same as SetEntity (old form kept for compatibility) Warning : Does nothing if Entity field is not yet
    /// clear
    /// </summary>
    public void GetEntity(Standard_Transient? anentity)
    {
        if (theent is null)
        {
            SetEntity(anentity);
        }
    }

    /// <summary>
    /// Copies messages stored in another Check, cumulating Does not regard other's Entity. Used to cumulate
    /// messages
    /// </summary>
    public void GetMessages(Interface_Check? other)
    {
        int nb, i;
        if ((nb = other.NbFails()) != 0)
        {
            if (thefails is null)
            {
                thefails = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            if (thefailo is null)
            {
                thefailo = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            for (i = 1; i <= nb; i++)
            {
                thefails.Append(other.Fail(i, true));
            }
            for (i = 1; i <= nb; i++)
            {
                thefailo.Append(other.Fail(i, false));
            }
        }
        if ((nb = other.NbWarnings()) != 0)
        {
            if (thewarns is null)
            {
                thewarns = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            if (thewarno is null)
            {
                thewarno = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            for (i = 1; i <= nb; i++)
            {
                thewarns.Append(other.Warning(i, true));
            }
            for (i = 1; i <= nb; i++)
            {
                thewarno.Append(other.Warning(i, false));
            }
        }
        if ((nb = other.NbInfoMsgs()) != 0)
        {
            if (theinfos is null)
            {
                theinfos = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            if (theinfoo is null)
            {
                theinfoo = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            for (i = 1; i <= nb; i++)
            {
                theinfos.Append(other.InfoMsg(i, true));
            }
            for (i = 1; i <= nb; i++)
            {
                theinfoo.Append(other.InfoMsg(i, false));
            }
        }
    }

    /// <summary>
    /// Copies messages converted into Warning messages If failsonly is true, only Fails are taken, and converted
    /// else, Warnings are taken too. Does not regard Entity Used to keep Fail messages as Warning, after a
    /// recovery
    /// </summary>
    public void GetAsWarning(Interface_Check? other, bool failsonly)
    {
        int nb, i;
        if ((nb = other.NbFails()) != 0)
        {
            if (thewarns is null)
            {
                thewarns = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            if (thewarno is null)
            {
                thewarno = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            for (i = 1; i <= nb; i++)
            {
                thewarns.Append(other.Fail(i, true));
            }
            for (i = 1; i <= nb; i++)
            {
                thewarno.Append(other.Fail(i, false));
            }
        }
        if ((nb = other.NbWarnings()) != 0 || !failsonly)
        {
            if (thewarns is null)
            {
                thewarns = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            if (thewarno is null)
            {
                thewarno = new NCollection_HSequence<TCollection_HAsciiString>();
            }
            for (i = 1; i <= nb; i++)
            {
                thewarns.Append(other.Warning(i, true));
            }
            for (i = 1; i <= nb; i++)
            {
                thewarno.Append(other.Warning(i, false));
            }
        }
    }

    /// <summary>
    /// Prints the messages of the check to an Messenger &lt;level&gt; = 1 : only fails &lt;level&gt; = 2 : fails
    /// and warnings &lt;level&gt; = 3 : all (fails, warnings, info msg) &lt;final&gt; : if positive (D) prints
    /// final values of messages if negative, prints originals if null, prints both forms
    /// </summary>
    public void Print(TextWriter S, int level, int final = 1)
    {
        int j, nb = NbFails();

        if (level >= 1)
        {
            nb = NbFails();
            //    if (nb > 0) S<<" -.   Fails    : "<<nb<<std.endl;
            for (j = 1; j <= nb; j++)
            {
                if (final >= 0)
                {
                    S.Write(CFail(j, true));
                    S.Write("\n");
                }
                else
                {
                    S.Write(CFail(j, false));
                    S.Write("\n");
                }
                //      if (final == 0)
                //	if (Fail(j,true) != Fail(j,false))
                //	  S<<"[original:"<<CFail(j,false)<<"]\n";
            }
        }

        if (level >= 2)
        {
            nb = NbWarnings();
            //    if (nb > 0) S<<" -.   Warnings : "<<nb<<std.endl;
            for (j = 1; j <= nb; j++)
            {
                if (final >= 0)
                {
                    S.Write(CWarning(j, true));
                    S.Write("\n");
                }
                else
                {
                    S.Write(CWarning(j, false));
                    S.Write("\n");
                }
                //      if (final == 0)
                //	if (Warning(j,true) != Warning(j,false))
                //	  S<<"[original:"<<CWarning(j,false)<<"]\n";
            }
        }

        //   InfoMsg : causal rule (user message)
        if (level >= 0)
        {
            nb = NbInfoMsgs();
            //    if (nb > 0) S<<" -.   Infos    : "<<nb<<std.endl;
            for (j = 1; j <= nb; j++)
            {
                if (final >= 0)
                {
                    S.Write(CInfoMsg(j, true));
                    S.Write("\n");
                }
                else
                {
                    S.Write(CInfoMsg(j, false));
                    S.Write("\n");
                }
                //      if (final == 0)
                //	if (InfoMsg(j,true) != InfoMsg(j,false))
                //	  S<<"[original:"<<CInfoMsg(j,false)<<"]\n";
            }
        }
    }

    /// <summary>
    /// Prints the messages of the check to the default trace file By default, according to the default standard
    /// level Else, according level (see method Print)
    /// </summary>
    public void Trace(int level = -1, int final = 1)
    {
        // clang-format off
        using Message_Messenger.StreamBuffer sout = Message.SendInfo(); // shouldn't it be changed to SendTrace()?
        // clang-format on
        var aText = new StringWriter();
        Print(aText, level, final);
        sout.Append(aText.ToString());
    }

    private NCollection_HSequence<TCollection_HAsciiString>? thefails;
    private NCollection_HSequence<TCollection_HAsciiString>? thefailo;
    private NCollection_HSequence<TCollection_HAsciiString>? thewarns;
    private NCollection_HSequence<TCollection_HAsciiString>? thewarno;
    private NCollection_HSequence<TCollection_HAsciiString>? theinfos;
    private NCollection_HSequence<TCollection_HAsciiString>? theinfoo;
    private Standard_Transient? theent;
}
