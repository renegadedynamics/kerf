// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferWriter.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferWriter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Topology;
using static Occt.IFSelect_ReturnStatus;

namespace Occt;

/// <summary>
/// TransferWriter gives help to control transfer to write a file after having converted data from
/// Cascade/Imagine It works with a Controller (which itself can work with an Actor to Write) and a
/// FinderProcess. It records results and checks
/// </summary>
public class XSControl_TransferWriter : Transient
{

    /// <summary>
    /// Creates a TransferWriter, empty, ready to run with an empty FinderProcess (but no controller, etc)
    /// </summary>
    public XSControl_TransferWriter()
    {
        myTransferWriter = new Transfer_FinderProcess();
        myTransferMode = 0;
    }

    /// <summary>
    /// Returns the FinderProcess itself
    /// </summary>
    public Transfer_FinderProcess? FinderProcess()
    {
        return myTransferWriter;
    }

    /// <summary>
    /// Sets a new FinderProcess and forgets the former one
    /// </summary>
    public void SetFinderProcess(Transfer_FinderProcess? theFP)
    {
        myTransferWriter = theFP;
    }

    /// <summary>
    /// Returns the currently used Controller
    /// </summary>
    public XSControl_Controller? Controller()
    {
        return myController;
    }

    /// <summary>
    /// Sets a new Controller(), also sets a new FinderProcess
    /// </summary>
    public void SetController(XSControl_Controller? theCtl)
    {
        myController = theCtl;
        Clear(-1);
    }

    /// <summary>
    /// Clears recorded data according a mode 0 clears FinderProcess (results, checks) -1 create a new
    /// FinderProcess
    /// </summary>
    public void Clear(int mode)
    {
        if (mode < 0 || myTransferWriter is null)
        {
            myTransferWriter = new Transfer_FinderProcess();
        }
        else
        {
            myTransferWriter.Clear();
        }
    }

    /// <summary>
    /// Returns the current Transfer Mode (an Integer) It will be interpreted by the Controller to run Transfers
    /// This call form could be later replaced by more specific ones (parameters suited for each norm / transfer
    /// case)
    /// </summary>
    public int TransferMode()
    {
        return myTransferMode;
    }

    /// <summary>
    /// Changes the Transfer Mode
    /// </summary>
    public void SetTransferMode(int theMode)
    {
        myTransferMode = theMode;
    }

    /// <summary>
    /// Prints statistics on current Trace File, according what,mode See PrintStatsProcess for details
    /// </summary>
    public void PrintStats(int theWhat, int theMode = 0)
    {
        using Messenger.StreamBuffer sout = myTransferWriter.Messenger().SendInfo();
        //  A ameliorer ... !
        sout.Append("\n*******************************************************************\n");
        sout.Append("******        Statistics on Transfer (Write)                 ******");
        sout.Append('\n');
        sout.Append("\n*******************************************************************\n");
        sout.Append("******        Transfer Mode = ");
        sout.Append(myTransferMode);
        string modehelp = myController.ModeWriteHelp(myTransferMode);
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
        if (myController is null)
        {
            return false;
        }
        XSControl_Utils xu = new XSControl_Utils();
        Shape sh = xu.BinderShape(obj);
        if (!sh.IsNull)
        {
            return RecognizeShape(sh);
        }
        return myController.RecognizeWriteTransient(obj, myTransferMode);
    }

    /// <summary>
    /// Transfers a Transient object (from an application) to a model of current norm, according to the last call
    /// to SetTransferMode Works by calling the Controller Returns status : =0 if OK, &gt;0 if error during
    /// transfer, &lt;0 if transfer badly initialised
    /// </summary>
    public IFSelect_ReturnStatus TransferWriteTransient(Interface_InterfaceModel? model, Transient? obj, ProgressRange theProgress = default)
    {
        IFSelect_ReturnStatus status = IFSelect_RetVoid;
        if (myController is null)
        {
            return IFSelect_RetError;
        }
        if (model is null)
        {
            return IFSelect_RetVoid;
        }

        if (myTransferWriter is null)
        {
            myTransferWriter = new Transfer_FinderProcess();
        }
        Transfer_ActorOfFinderProcess nulact = null;
        myTransferWriter.SetActor(nulact);
        using Messenger.StreamBuffer sout = myTransferWriter.Messenger().SendInfo();
        try
        {
            PrintStats(myTransferMode);
            sout.Append("******        Transferring Transient, CDL Type = ");
            sout.Append(obj.DynamicTypeName());
            sout.Append("   ******");
            sout.Append('\n');
            status = myController.TransferWriteTransient(obj, myTransferWriter, model, myTransferMode, theProgress);
        }
        catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
        {
            sout.Append("****  ****  TransferWriteShape, EXCEPTION : ");
            sout.Append(Transfer_ProcessForTransient.What(anException));
            sout.Append('\n');
            status = IFSelect_RetFail;
        }
        return status;
    }

    /// <summary>
    /// Tells if a Shape is valid for a transfer to a model Asks the Controller (RecognizeWriteShape)
    /// </summary>
    public bool RecognizeShape(Shape shape)
    {
        if (myController is null)
        {
            return false;
        }
        if (shape.IsNull)
        {
            return false;
        }
        return myController.RecognizeWriteShape(shape, myTransferMode);
    }

    /// <summary>
    /// Transfers a Shape from CasCade to a model of current norm, according to the last call to SetTransferMode
    /// Works by calling the Controller Returns status : =0 if OK, &gt;0 if error during transfer, &lt;0 if
    /// transfer badly initialised
    /// </summary>
    public IFSelect_ReturnStatus TransferWriteShape(Interface_InterfaceModel? theModel, Shape theShape, ProgressRange theProgress = default)
    {
        IFSelect_ReturnStatus status = IFSelect_RetVoid;
        if (myController is null)
        {
            return IFSelect_RetError;
        }
        if (theModel is null)
        {
            return IFSelect_RetVoid;
        }

        Shape aShape = theShape;

        if (myTransferWriter is null)
        {
            myTransferWriter = new Transfer_FinderProcess();
        }
        //  effacer l actor : Controller s en charge
        Transfer_ActorOfFinderProcess nulact = null;
        myTransferWriter.SetActor(nulact);
        using Messenger.StreamBuffer sout = myTransferWriter.Messenger().SendInfo();
        try
        {
            PrintStats(myTransferMode);
            sout.Append("******        Transferring Shape, ShapeType = ");
            sout.Append((int)aShape.ShapeType()); // the enumeration's value, as C++ streams it
            sout.Append("                      ******");
            sout.Append('\n');
            status = myController.TransferWriteShape(aShape, myTransferWriter, theModel, myTransferMode, theProgress);
        }
        catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
        {
            sout.Append("****  ****  TransferWriteShape, EXCEPTION : ");
            sout.Append(Transfer_ProcessForTransient.What(anException));
            sout.Append('\n');
            status = IFSelect_RetFail;
        }
        return status;
    }

    /// <summary>
    /// Returns the check-list of last transfer (write), i.e. the check-list currently recorded in the
    /// FinderProcess
    /// </summary>
    public Interface_CheckIterator CheckList()
    {
        Interface_CheckIterator chl = new Interface_CheckIterator();
        if (myTransferWriter is null)
        {
            return chl;
        }
        int i, nb = myTransferWriter.NbMapped();
        for (i = 1; i <= nb; i++)
        {
            Transfer_SimpleBinderOfTransient tb = myTransferWriter.MapItem(i) as Transfer_SimpleBinderOfTransient;
            if (tb is null)
            {
                continue;
            }
            Interface_Check ach = tb.Check();
            if (ach.NbFails() == 0 || ach.NbWarnings() == 0) // OCCT: only checks with both fails and warnings
            {
                continue;
            }
            Transfer_TransientMapper tm = myTransferWriter.Mapped(i) as Transfer_TransientMapper;
            if (tm is null)
            {
                ach.GetEntity(myTransferWriter.Mapped(i));
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
    public Interface_CheckIterator ResultCheckList(Interface_InterfaceModel? model)
    {
        Interface_CheckIterator chl = new Interface_CheckIterator();
        if (myTransferWriter is null)
        {
            return chl;
        }
        int i, nb = myTransferWriter.NbMapped();
        for (i = 1; i <= nb; i++)
        {
            Transfer_SimpleBinderOfTransient tb = myTransferWriter.MapItem(i) as Transfer_SimpleBinderOfTransient;
            if (tb is null)
            {
                continue;
            }
            Interface_Check ach = tb.Check();
            if (ach.NbFails() == 0 || ach.NbWarnings() == 0) // OCCT: only checks with both fails and warnings
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

    private XSControl_Controller? myController;
    private Transfer_FinderProcess? myTransferWriter;
    private int myTransferMode;
}
