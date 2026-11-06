// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Binder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Binder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A Binder is an auxiliary object to Map the Result of the Transfer of a given Object : it records the Result of
/// the Unitary Transfer (Resulting Object), status of progress and error (if any) of the Process.
/// </summary>
/// <remarks>
/// The class Binder itself makes no definition for the Result : it is defined by sub-classes : it can be either
/// Simple or Multiple (see class MultipleBinder). The StatusResult controls whether the Result can still change: if
/// it is "Used", it cannot. A Binder can also bring a list of next results. A binder is a working object of the
/// transfer process (decision 21): its setters are public, as OCCT's.
/// </remarks>
internal abstract class Binder : Transient
{
    /// <summary>Sets fields at initial values.</summary>
    protected Binder()
    {
        _thestatus = StatusResult.StatusVoid;
        _theexecst = global::Kerf.Engine.Exchange.Framework.Transfer.StatusExec.StatusInitial;
        _thecheck = new FrameworkCheck();
    }

    /// <summary>
    /// Merges basic data (Check, ExecStatus) from another Binder but keeps its result. Used when a binder is replaced
    /// by another one, this allows to keep messages.
    /// </summary>
    public void Merge(Binder? other)
    {
        if (other is null)
        {
            return;
        }
        if ((int)_theexecst < (int)other.StatusExec())
        {
            _theexecst = other.StatusExec();
        }
        _thecheck.GetMessages(other.Check());
    }

    /// <summary>
    /// Returns True if a Binder has several results, either by itself or because it has next results. Can be defined
    /// by sub-classes.
    /// </summary>
    public virtual bool IsMultiple()
    {
        if (_thenextr is null)
        {
            return false;
        }
        if (!HasResult)
        {
            return _thenextr.IsMultiple();
        }

        Binder? next = _thenextr;
        while (next is not null)
        {
            if (next.HasResult)
            {
                return true;
            }
            next = next.NextResult();
        }
        return false;
    }

    /// <summary>Returns the Type which characterizes the Result (if known).</summary>
    public abstract Type? ResultType();

    /// <summary>Returns the Name of the Type which characterizes the Result. Can be returned even if ResultType itself is unknown.</summary>
    public abstract string ResultTypeName();

    /// <summary>Adds a next result (at the end of the list). Remark : this information is not processed by Merge.</summary>
    public void AddResult(Binder? next)
    {
        if (next == this || next is null)
        {
            return;
        }
        next.CutResult(this);
        if (_thenextr is null)
        {
            _thenextr = next;
        }
        else
        {
            // Modification of recursive to cycle
            Binder theBinder = _theendr is null ? _thenextr : _theendr;
            while (theBinder != next)
            {
                if (theBinder.NextResult() is null)
                {
                    theBinder.AddResult(next);
                    _theendr = next;
                    return;
                }
                else
                {
                    theBinder = theBinder.NextResult();
                }
            }
        }
    }

    // Called by AddResult, to keep unicity of each item in the list
    private void CutResult(Binder next)
    {
        if (_thenextr is null)
        {
            return;
        }
        if (_thenextr == next)
        {
            _thenextr = null;
            _theendr = null;
        }
        // else thenextr->CutResult (next);
        else
        {
            Binder currBinder = _thenextr, currNext;
            while (!((currNext = currBinder.NextResult()) == next))
            {
                if (currNext is null)
                {
                    return;
                }
                currBinder = currNext;
            }
            currBinder.CutResult(next);
        }
    }

    /// <summary>Returns the next result, Null if none.</summary>
    public Binder? NextResult()
    {
        return _thenextr;
    }

    /// <summary>
    /// Used to declare that a result is recorded for an individual transfer (works by setting StatusResult to
    /// Defined). To be called once a Result is really recorded.
    /// </summary>
    /// <exception cref="TransferFailureException">The result is already set and used.</exception>
    protected void SetResultPresent()
    {
        if (_thestatus == StatusResult.StatusUsed)
        {
            throw new TransferFailureException("Binder : SetResult, Result is Already Set and Used");
        }
        _theexecst = global::Kerf.Engine.Exchange.Framework.Transfer.StatusExec.StatusDone;
        _thestatus = StatusResult.StatusDefined;
    }

    /// <summary>Returns True if a Result is available (StatusResult = Defined).</summary>
    public bool HasResult
    {
        get
        {
            return _thestatus != StatusResult.StatusVoid;
        }
    }

    /// <summary>Declares that result is now used by another one, it means that it cannot be modified (by Rebind).</summary>
    public void SetAlreadyUsed()
    {
        if (_thestatus != StatusResult.StatusVoid)
        {
            _thestatus = StatusResult.StatusUsed;
        }
    }

    /// <summary>
    /// Returns status, which can be Initial (not yet done), Made (a result is recorded, not yet shared), Used (it is
    /// shared and cannot be modified).
    /// </summary>
    public StatusResult Status()
    {
        return _thestatus;
    }

    //  ############    Checks    ############

    /// <summary>Returns execution status.</summary>
    public StatusExec StatusExec()
    {
        return _theexecst;
    }

    /// <summary>Modifies execution status; called by TransferProcess only (for StatusError, rather use AddFail).</summary>
    public void SetStatusExec(StatusExec stat)
    {
        _theexecst = stat;
    }

    /// <summary>
    /// Used to declare an individual transfer as being erroneous (StatusExec is set to Error, <paramref name="mess"/>
    /// is added to Check's list of Fails). It is possible to record several messages of error.
    /// </summary>
    public void AddFail(string mess, string orig = "")
    {
        _theexecst = global::Kerf.Engine.Exchange.Framework.Transfer.StatusExec.StatusError;
        _thecheck.AddFail(mess, orig);
    }

    /// <summary>Used to attach a Warning Message to an individual Transfer. It has no effect on the Status.</summary>
    public void AddWarning(string mess, string orig = "")
    {
        //  theexecst = Transfer_StatusError;
        _thecheck.AddWarning(mess, orig);
    }

    /// <summary>Returns Check which stores Fail messages. Note that no Entity is associated in this Check.</summary>
    public FrameworkCheck Check()
    {
        return _thecheck;
    }

    /// <summary>Returns Check which stores Fail messages, in order to modify it (adding messages, or replacing it).</summary>
    public FrameworkCheck CCheck()
    {
        return _thecheck;
    }

    // OCCT's destructor unlinks long chains of next results iteratively, to avoid a deep recursive destruction: the
    // garbage collector has no such recursion, so it is left out.

    private StatusResult _thestatus;
    private StatusExec _theexecst;
    private readonly FrameworkCheck _thecheck;
    private Binder? _thenextr;
    private Binder? _theendr;
}
