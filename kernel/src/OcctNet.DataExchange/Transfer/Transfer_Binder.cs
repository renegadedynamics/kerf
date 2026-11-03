// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Binder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_Binder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Transfer_StatusExec;
using static Occt.Transfer_StatusResult;

namespace Occt;

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
public abstract class Transfer_Binder : Transient
{
    /// <summary>Sets fields at initial values.</summary>
    protected Transfer_Binder()
    {
        thestatus = Transfer_StatusResult.Transfer_StatusVoid;
        theexecst = Transfer_StatusExec.Transfer_StatusInitial;
        thecheck = new Interface_Check();
    }

    /// <summary>
    /// Merges basic data (Check, ExecStatus) from another Binder but keeps its result. Used when a binder is replaced
    /// by another one, this allows to keep messages.
    /// </summary>
    public void Merge(Transfer_Binder? other)
    {
        if (other is null)
        {
            return;
        }
        if ((int)theexecst < (int)other.StatusExec())
        {
            theexecst = other.StatusExec();
        }
        thecheck.GetMessages(other.Check());
    }

    /// <summary>
    /// Returns True if a Binder has several results, either by itself or because it has next results. Can be defined
    /// by sub-classes.
    /// </summary>
    public virtual bool IsMultiple()
    {
        if (thenextr is null)
        {
            return false;
        }
        if (!HasResult())
        {
            return thenextr.IsMultiple();
        }

        Transfer_Binder? next = thenextr;
        while (next is not null)
        {
            if (next.HasResult())
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
    public void AddResult(Transfer_Binder? next)
    {
        if (next == this || next is null)
        {
            return;
        }
        next.CutResult(this);
        if (thenextr is null)
        {
            thenextr = next;
        }
        else
        {
            // Modification of recursive to cycle
            Transfer_Binder theBinder = theendr is null ? thenextr : theendr;
            while (theBinder != next)
            {
                if (theBinder.NextResult() is null)
                {
                    theBinder.AddResult(next);
                    theendr = next;
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
    private void CutResult(Transfer_Binder next)
    {
        if (thenextr is null)
        {
            return;
        }
        if (thenextr == next)
        {
            thenextr = null;
            theendr = null;
        }
        // else thenextr->CutResult (next);
        else
        {
            Transfer_Binder currBinder = thenextr, currNext;
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
    public Transfer_Binder? NextResult()
    {
        return thenextr;
    }

    /// <summary>
    /// Used to declare that a result is recorded for an individual transfer (works by setting StatusResult to
    /// Defined). To be called once a Result is really recorded.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The result is already set and used.</exception>
    protected void SetResultPresent()
    {
        if (thestatus == Transfer_StatusResult.Transfer_StatusUsed)
        {
            throw new Transfer_TransferFailure("Binder : SetResult, Result is Already Set and Used");
        }
        theexecst = Transfer_StatusExec.Transfer_StatusDone;
        thestatus = Transfer_StatusResult.Transfer_StatusDefined;
    }

    /// <summary>Returns True if a Result is available (StatusResult = Defined).</summary>
    public bool HasResult()
    {
        return thestatus != Transfer_StatusResult.Transfer_StatusVoid;
    }

    /// <summary>Declares that result is now used by another one, it means that it cannot be modified (by Rebind).</summary>
    public void SetAlreadyUsed()
    {
        if (thestatus != Transfer_StatusResult.Transfer_StatusVoid)
        {
            thestatus = Transfer_StatusResult.Transfer_StatusUsed;
        }
    }

    /// <summary>
    /// Returns status, which can be Initial (not yet done), Made (a result is recorded, not yet shared), Used (it is
    /// shared and cannot be modified).
    /// </summary>
    public Transfer_StatusResult Status()
    {
        return thestatus;
    }

    //  ############    Checks    ############

    /// <summary>Returns execution status.</summary>
    public Transfer_StatusExec StatusExec()
    {
        return theexecst;
    }

    /// <summary>Modifies execution status; called by TransferProcess only (for StatusError, rather use AddFail).</summary>
    public void SetStatusExec(Transfer_StatusExec stat)
    {
        theexecst = stat;
    }

    /// <summary>
    /// Used to declare an individual transfer as being erroneous (StatusExec is set to Error, <paramref name="mess"/>
    /// is added to Check's list of Fails). It is possible to record several messages of error.
    /// </summary>
    public void AddFail(string mess, string orig = "")
    {
        theexecst = Transfer_StatusExec.Transfer_StatusError;
        thecheck.AddFail(mess, orig);
    }

    /// <summary>Used to attach a Warning Message to an individual Transfer. It has no effect on the Status.</summary>
    public void AddWarning(string mess, string orig = "")
    {
        //  theexecst = Transfer_StatusError;
        thecheck.AddWarning(mess, orig);
    }

    /// <summary>Returns Check which stores Fail messages. Note that no Entity is associated in this Check.</summary>
    public Interface_Check Check()
    {
        return thecheck;
    }

    /// <summary>Returns Check which stores Fail messages, in order to modify it (adding messages, or replacing it).</summary>
    public Interface_Check CCheck()
    {
        return thecheck;
    }

    // OCCT's destructor unlinks long chains of next results iteratively, to avoid a deep recursive destruction: the
    // garbage collector has no such recursion, so it is left out.

    private Transfer_StatusResult thestatus;
    private Transfer_StatusExec theexecst;
    private readonly Interface_Check thecheck;
    private Transfer_Binder? thenextr;
    private Transfer_Binder? theendr;
}
