// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_SignTransferStatus.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_SignTransferStatus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusExec;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusResult;

using static Kerf.Engine.Exchange.Framework.FrameworkCheckStatus;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This Signatures gives the Transfer Status of an entity, as recorded in a TransferProcess. It can be : -
/// Void : not recorded, or recorded as void with no message (attributes are not taken into account) -
/// Warning : no result, warning message(s), no fail - Fail : no result, fail messages (with or without
/// warning) - Result.. : result, no message (neither warning nor fail) Result.. i.e. Result:TypeName of the
/// result - Result../Warning : result, with warning but no fail - Result../Fail : result, with fail (.e. bad
/// result) - Fail on run : no result yet recorded, no message, but an exception occurred while recording the
/// result (this should not appear and indicates a programming error)
/// </summary>
public class SignTransferStatus : Signature
{
    // OCCT's file static themes() (a process-wide buffer for the returned text) is a local string in Value.

    // BinderStatus retourne une valeur :
    // 0 Binder Null.   1 void  2 Warning seul  3 Fail seul
    // 11 Resultat OK. 12 Resultat+Warning. 13 Resultat+Fail
    // 20 Abnormal (Interrupted)

    private static int BinderStatus(Binder? binder)
    {
        int stat = 0;
        if (binder is null)
        {
            return 0;
        }
        FrameworkCheckStatus cst = binder.Check().Status();
        StatusExec est = binder.StatusExec();
        bool res = binder.HasResult;
        if (est == StatusExec.StatusRun || est == StatusExec.StatusLoop)
        {
            return 20;
        }
        if (cst == FrameworkCheckStatus.CheckOK)
        {
            stat = (res ? 11 : 1);
        }
        else if (cst == FrameworkCheckStatus.CheckWarning)
        {
            stat = (res ? 12 : 2);
        }
        else if (cst == FrameworkCheckStatus.CheckFail)
        {
            stat = (res ? 13 : 3);
        }

        return stat;
    }

    /// <summary>
    /// Creates a SignTransferStatus, not initialised it gives nothing (empty string)
    /// </summary>
    public SignTransferStatus() : base("Transfer Status")
    {
    }

    /// <summary>
    /// Creates a SignTransferStatus, which will work on the current TransientProcess brought by the
    /// TransferReader (its MapReader)
    /// </summary>
    public SignTransferStatus(TransferReader? tr) : base("Transfer Status")
    {
        _tr = tr;
    }

    /// <summary>
    /// Sets a TransferReader to work
    /// </summary>
    public void SetReader(TransferReader? tr)
    {
        _tr = tr;
    }

    /// <summary>
    /// Sets a precise map to sign entities This definition oversedes the creation with a TransferReader
    /// </summary>
    public void SetMap(TransientProcess? tp)
    {
        _tp = tp;
    }

    /// <summary>
    /// Returns the TransientProcess used as precised one Returns a Null Handle for a creation from a
    /// TransferReader without any further setting
    /// </summary>
    public TransientProcess? Map()
    {
        return _tp;
    }

    /// <summary>
    /// Returns the Reader (if created with a Reader) Returns a Null Handle if not created with a Reader
    /// </summary>
    public TransferReader? Reader()
    {
        return _tr;
    }

    /// <summary>
    /// Returns the Signature for a Transient object, as its transfer status
    /// </summary>
    public override string Value(Transient? ent, InterfaceModel? model)
    {
        if (ent is null)
        {
            return "";
        }
        TransientProcess tp = _tp;
        if (tp is null && _tr is not null)
        {
            tp = _tr.TransientProcess();
        }
        if (tp is null)
        {
            return "";
        }

        Binder binder = tp.Find(ent);

        int stat = BinderStatus(binder);

        if (stat <= 1)
        {
            return "";
        }
        if (stat == 2)
        {
            return "Warning";
        }
        if (stat == 3)
        {
            return "Fail";
        }
        if (stat == 20)
        {
            return "Fail on run";
        }

        string themes = "";
        if (stat > 10)
        {
            //  Y a un resultat : donner son type
            Binder bnd = binder;
            bool hasres = false;
            while (bnd is not null)
            {
                if (bnd.Status() != StatusResult.StatusVoid)
                {
                    if (!hasres)
                    {
                        themes += ("Result:");
                    }
                    else
                    {
                        themes += (",");
                    }
                    themes += (bnd.ResultTypeName());
                    hasres = true;
                }
                bnd = bnd.NextResult();
            }
            //    if (stat == 11) Sprintf(themes,"Result:%s",binder.ResultTypeName());
            if (stat == 12)
            {
                themes += ("/Warning");
            }
            if (stat == 13)
            {
                themes += ("/Fail");
            }
        }
        return themes;
    }

    private TransferReader? _tr;
    private TransientProcess? _tp;
}
