// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferWriter.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// TransferWriter gives help to control transfer to write a file after having converted data from
/// Cascade/Imagine It works with a Controller (which itself can work with an Actor to Write) and a
/// FinderProcess. It records results and checks
/// </summary>
internal class TransferWriter : Transient
{

    /// <summary>
    /// Creates a TransferWriter, empty, ready to run with an empty FinderProcess (but no controller, etc)
    /// </summary>
    public TransferWriter()
    {
        _transferWriter = new FinderProcess();
        _transferMode = 0;
    }

    /// <summary>
    /// Returns the FinderProcess itself
    /// </summary>
    public FinderProcess? FinderProcess()
    {
        return _transferWriter;
    }

    /// <summary>
    /// Sets a new FinderProcess and forgets the former one
    /// </summary>
    public void SetFinderProcess(FinderProcess? fp)
    {
        _transferWriter = fp;
    }

    /// <summary>
    /// Returns the currently used Controller
    /// </summary>
    public Controller? Controller()
    {
        return _controller;
    }

    /// <summary>
    /// Sets a new Controller(), also sets a new FinderProcess
    /// </summary>
    public void SetController(Controller? ctl)
    {
        _controller = ctl;
        Clear(-1);
    }

    /// <summary>
    /// Clears recorded data according a mode 0 clears FinderProcess (results, checks) -1 create a new
    /// FinderProcess
    /// </summary>
    public void Clear(int mode)
    {
        if (mode < 0 || _transferWriter is null)
        {
            _transferWriter = new FinderProcess();
        }
        else
        {
            _transferWriter.Clear();
        }
    }

    /// <summary>
    /// Returns the current Transfer Mode (an Integer) It will be interpreted by the Controller to run Transfers
    /// This call form could be later replaced by more specific ones (parameters suited for each norm / transfer
    /// case)
    /// </summary>
    public int TransferMode()
    {
        return _transferMode;
    }

    /// <summary>
    /// Changes the Transfer Mode
    /// </summary>
    public void SetTransferMode(int mode)
    {
        _transferMode = mode;
    }

    /// <summary>
    /// Prints statistics on current Trace File, according what,mode See PrintStatsProcess for details
    /// </summary>
    public void PrintStats(int what, int mode = 0)
    {
        using Messenger.StreamBuffer sout = _transferWriter.Messenger().SendInfo();
        //  A ameliorer ... !
        sout.Append("\n*******************************************************************\n");
        sout.Append("******        Statistics on Transfer (Write)                 ******");
        sout.Append('\n');
        sout.Append("\n*******************************************************************\n");
        sout.Append("******        Transfer Mode = ");
        sout.Append(_transferMode);
        string modehelp = _controller.ModeWriteHelp(_transferMode);
        if (modehelp is not null && CStringChar(modehelp, 0) != '\0')
        {
            sout.Append("  I.E.  ");
            sout.Append(modehelp);
        }
        sout.Append("       ******");
        sout.Append('\n');
    }

    /// <summary>
    /// Tells if a transient object (from an application) is a valid candidate for a transfer to a model Asks the
    /// Controller (RecognizeWriteTransient) If &lt;obj&gt; is a HShape, calls RecognizeShape
    /// </summary>
    public bool RecognizeTransient(Transient? obj)
    {
        if (_controller is null)
        {
            return false;
        }
        Utils xu = new Utils();
        Shape sh = xu.BinderShape(obj);
        if (!sh.IsNull)
        {
            return RecognizeShape(sh);
        }
        return _controller.RecognizeWriteTransient(obj, _transferMode);
    }

    /// <summary>
    /// Transfers a Transient object (from an application) to a model of current norm, according to the last call
    /// to SetTransferMode Works by calling the Controller Returns status : =0 if OK, &gt;0 if error during
    /// transfer, &lt;0 if transfer badly initialised
    /// </summary>
    public ReturnStatus TransferWriteTransient(InterfaceModel? model, Transient? obj, ProgressRange progress = default)
    {
        ReturnStatus status = ReturnStatus.RetVoid;
        if (_controller is null)
        {
            return ReturnStatus.RetError;
        }
        if (model is null)
        {
            return ReturnStatus.RetVoid;
        }

        if (_transferWriter is null)
        {
            _transferWriter = new FinderProcess();
        }
        ActorOfFinderProcess nulact = null;
        _transferWriter.SetActor(nulact);
        using Messenger.StreamBuffer sout = _transferWriter.Messenger().SendInfo();
        try
        {
            PrintStats(_transferMode);
            sout.Append("******        Transferring Transient, CDL Type = ");
            sout.Append(obj.DynamicTypeName());
            sout.Append("   ******");
            sout.Append('\n');
            status = _controller.TransferWriteTransient(obj, _transferWriter, model, _transferMode, progress);
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
            sout.Append("****  ****  TransferWriteShape, EXCEPTION : ");
            sout.Append(ProcessForTransient.What(exception));
            sout.Append('\n');
            status = ReturnStatus.RetFail;
        }
        return status;
    }

    /// <summary>
    /// Tells if a Shape is valid for a transfer to a model Asks the Controller (RecognizeWriteShape)
    /// </summary>
    public bool RecognizeShape(Shape shape)
    {
        if (_controller is null)
        {
            return false;
        }
        if (shape.IsNull)
        {
            return false;
        }
        return _controller.RecognizeWriteShape(shape, _transferMode);
    }

    /// <summary>
    /// Transfers a Shape from CasCade to a model of current norm, according to the last call to SetTransferMode
    /// Works by calling the Controller Returns status : =0 if OK, &gt;0 if error during transfer, &lt;0 if
    /// transfer badly initialised
    /// </summary>
    public ReturnStatus TransferWriteShape(InterfaceModel? model, Shape shape, ProgressRange progress = default)
    {
        ReturnStatus status = ReturnStatus.RetVoid;
        if (_controller is null)
        {
            return ReturnStatus.RetError;
        }
        if (model is null)
        {
            return ReturnStatus.RetVoid;
        }

        Shape aShape = shape;

        if (_transferWriter is null)
        {
            _transferWriter = new FinderProcess();
        }
        //  effacer l actor : Controller s en charge
        ActorOfFinderProcess nulact = null;
        _transferWriter.SetActor(nulact);
        using Messenger.StreamBuffer sout = _transferWriter.Messenger().SendInfo();
        try
        {
            PrintStats(_transferMode);
            sout.Append("******        Transferring Shape, ShapeType = ");
            sout.Append((int)aShape.Type); // the enumeration's value, as C++ streams it
            sout.Append("                      ******");
            sout.Append('\n');
            status = _controller.TransferWriteShape(aShape, _transferWriter, model, _transferMode, progress);
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
            sout.Append("****  ****  TransferWriteShape, EXCEPTION : ");
            sout.Append(ProcessForTransient.What(exception));
            sout.Append('\n');
            status = ReturnStatus.RetFail;
        }
        return status;
    }

    /// <summary>
    /// Returns the check-list of last transfer (write), i.e. the check-list currently recorded in the
    /// FinderProcess
    /// </summary>
    public CheckIterator CheckList()
    {
        CheckIterator chl = new CheckIterator();
        if (_transferWriter is null)
        {
            return chl;
        }
        int i, nb = _transferWriter.MappedCount;
        for (i = 1; i <= nb; i++)
        {
            SimpleBinderOfTransient tb = _transferWriter.MapItem(i) as SimpleBinderOfTransient;
            if (tb is null)
            {
                continue;
            }
            FrameworkCheck ach = tb.Check();
            if (ach.FailCount == 0 || ach.WarningCount == 0) // OCCT: only checks with both fails and warnings
            {
                continue;
            }
            TransientMapper tm = _transferWriter.Mapped(i) as TransientMapper;
            if (tm is null)
            {
                ach.GetEntity(_transferWriter.Mapped(i));
            }
            else
            {
                ach.GetEntity(tm.Value());
            }
            chl.Add(ach);
        }
        return chl;
    }

    /// <summary>
    /// Returns the check-list of last transfer (write), but tries to bind to each check, the resulting entity in
    /// the model instead of keeping the original Mapper, whenever known
    /// </summary>
    public CheckIterator ResultCheckList(InterfaceModel? model)
    {
        CheckIterator chl = new CheckIterator();
        if (_transferWriter is null)
        {
            return chl;
        }
        int i, nb = _transferWriter.MappedCount;
        for (i = 1; i <= nb; i++)
        {
            SimpleBinderOfTransient tb = _transferWriter.MapItem(i) as SimpleBinderOfTransient;
            if (tb is null)
            {
                continue;
            }
            FrameworkCheck ach = tb.Check();
            if (ach.FailCount == 0 || ach.WarningCount == 0) // OCCT: only checks with both fails and warnings
            {
                continue;
            }
            Transient ent = tb.Result();
            if (ent is not null && model is not null)
            {
                chl.Add(ach, model.Number(ent));
            }
            else
            {
                chl.Add(ach, 0);
            }
        }
        return chl;
    }

    private Controller? _controller;
    private FinderProcess? _transferWriter;
    private int _transferMode;
}
