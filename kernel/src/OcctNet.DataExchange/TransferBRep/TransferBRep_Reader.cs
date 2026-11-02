// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_Reader.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class offers a simple, easy to call, way of transferring data from interface files to Shapes from CasCade.
/// It must be specialized according to each norm/protocol, by : - defining how to read a file (specific method with
/// protocol) - definig transfer, by providing an Actor.
/// </summary>
/// <remarks>
/// The shapes a transfer gave are published (frozen) when the transfer ends (<see cref="EndTransfer"/>), so
/// <see cref="Shapes"/>, <see cref="Shape"/> and <see cref="OneShape"/> return frozen shapes (decision 1205).
/// </remarks>
public class TransferBRep_Reader
{
    /// <summary>Initializes a non-specialised Reader. Typically, for each norm or protocol, is will be required to define a specific Create to load a file and transfer it.</summary>
    public TransferBRep_Reader()
    {
        theDone = false;
        theFilest = 0;
        theNewpr = false;
        theShapes = new NCollection_HSequence<TopoDS_Shape>();
        theTransi = new NCollection_HSequence<Standard_Transient>();
    }

    /// <summary>Records the protocol to be used for read and transfer roots.</summary>
    public void SetProtocol(Interface_Protocol? protocol)
    {
        theProto = protocol;
    }

    /// <summary>Returns the recorded Protocol.</summary>
    public virtual Interface_Protocol? Protocol()
    {
        return theProto;
    }

    /// <summary>Records the actor to be used for transfers.</summary>
    public void SetActor(Transfer_ActorOfTransientProcess? actor)
    {
        theActor = actor;
    }

    /// <summary>Returns the recorded Actor.</summary>
    public virtual Transfer_ActorOfTransientProcess? Actor()
    {
        return theActor;
    }

    /// <summary>Sets File Status to be interpreted as follows : = 0 OK, &lt; 0 file not found, &gt; 0 read error, no Model could be created.</summary>
    public void SetFileStatus(int status)
    {
        theFilest = status;
    }

    /// <summary>Returns the File Status.</summary>
    public int FileStatus()
    {
        return theFilest;
    }

    /// <summary>Returns True if FileStatus is for FileNotFound.</summary>
    public bool FileNotFound()
    {
        return theFilest < 0;
    }

    /// <summary>Returns True if FileStatus is for Error during read (major error; for local error, see CheckModel).</summary>
    public bool SyntaxError()
    {
        return theFilest > 0;
    }

    /// <summary>Specifies a Model to work on. Also clears the result and Done status.</summary>
    public void SetModel(Interface_InterfaceModel? model)
    {
        theModel = model;
        Clear();
    }

    /// <summary>Returns the Model to be worked on.</summary>
    public Interface_InterfaceModel? Model()
    {
        return theModel;
    }

    /// <summary>clears the result and Done status. But not the Model.</summary>
    public void Clear()
    {
        theDone = false;
        theShapes.Clear();
        theTransi.Clear();
    }

    /// <summary>
    /// Checks the Model. Returns True if there is NO FAIL at all (regardless Warnings). If <paramref name="withprint"/>
    /// is True, also sends Checks on standard output.
    /// </summary>
    public bool CheckStatusModel(bool withprint)
    {
        var cht = new Interface_CheckTool(theModel, theProto);
        Interface_CheckIterator chl = cht.CompleteCheckList();
        if (withprint && theProc is not null && theProc.Messenger() is not null)
        {
            using Message_Messenger.StreamBuffer aBuffer = theProc.Messenger().SendInfo();
            using var aStream = new StringWriter(aBuffer.Stream());
            chl.Print(aStream, theModel, false);
        }
        return chl.IsEmpty(true);
    }

    /// <summary>Checks the Model (complete : syntax + semantic) and returns the produced Check List.</summary>
    public Interface_CheckIterator CheckListModel()
    {
        var cht = new Interface_CheckTool(theModel, theProto);
        Interface_CheckIterator chl = cht.CompleteCheckList();
        return chl;
    }

    /// <summary>
    /// Returns (by Reference, hence can be changed) the Mode for new Transfer : True (D) means that each new Transfer
    /// produces a new TransferProcess. Else, the existing Process is cleared and reused.
    /// </summary>
    public ref bool ModeNewTransfer()
    {
        return ref theNewpr;
    }

    /// <summary>
    /// Initializes the Reader for a Transfer (one,roots, or list). Also calls PrepareTransfer. Returns True when done,
    /// False if could not be done.
    /// </summary>
    public bool BeginTransfer()
    {
        theDone = false;
        if (theModel is null)
        {
            return false;
        }

        if (theNewpr || theProc is null)
        {
            theProc = new Transfer_TransientProcess(theModel.NbEntities());
        }
        else
        {
            theProc.Clear();
        }
        theProc.SetErrorHandle(true);
        theProc.SetModel(theModel);
        PrepareTransfer();
        theProc.SetActor(theActor);
        return true;
    }

    /// <summary>Ebds a Transfer (one, roots or list) by recording its result; the shapes are published (frozen).</summary>
    public void EndTransfer()
    {
        NCollection_HSequence<TopoDS_Shape> aShapes = TransferBRep.Shapes(theProc, true);
        for (int i = 1; i <= aShapes.Length(); i++)
        {
            aShapes.Value(i).TShape()?.FreezeGraph(); // OCCT.NET: the reader's results are published (decision 1205)
        }
        theShapes.Append(aShapes);
        int nb = theProc.NbRoots();
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient? ent = theProc.Root(i);
            Standard_Transient? res = theProc.FindTransient(ent);
            if (res is not null)
            {
                theTransi.Append(res);
            }
        }
        theDone = true;
    }

    /// <summary>
    /// Prepares the Transfer. Also can act on the Actor or change the TransientProcess if required. Should not set the
    /// Actor into the TransientProcess, it is done by caller. The provided default does nothing.
    /// </summary>
    public virtual void PrepareTransfer()
    {
    }

    /// <summary>
    /// Transfers all Root Entities which are recognized as Geom-Topol. The result will be a list of Shapes. This
    /// method calls user redefinable PrepareTransfer. Error messages are produced as the Transfer goes on.
    /// </summary>
    public virtual void TransferRoots(Message_ProgressRange theProgress = default)
    {
        Clear();
        if (!BeginTransfer())
        {
            return;
        }
        var TP = new Transfer_TransferOutput(theProc, theModel);

        TP.TransferRoots(theProto, theProgress);
        EndTransfer();
    }

    /// <summary>
    /// Transfers an Entity given its rank in the Model (Root or not). Returns True if it is recognized as Geom-Topol.
    /// (But it can have failed : see IsDone).
    /// </summary>
    public virtual bool Transfer(int num, Message_ProgressRange theProgress = default)
    {
        // OCCT: the former shapes aren't cleared (TransferRoots does): they accumulate
        if (!BeginTransfer())
        {
            return false;
        }
        if (num <= 0 || num > theModel.NbEntities())
        {
            return false;
        }
        Standard_Transient? ent = theModel.Value(num);
        var TP = new Transfer_TransferOutput(theProc, theModel);

        if (theProc.TraceLevel() > 1)
        {
            using Message_Messenger.StreamBuffer sout = theProc.Messenger().SendInfo();
            sout.Append("--  Transfer(Read) : ");
            using (var aStream = new StringWriter(sout.Stream()))
            {
                theModel.Print(ent, aStream);
            }
            sout.Append('\n');
        }
        TP.Transfer(ent, theProgress);
        theProc.SetRoot(ent);
        EndTransfer();
        return true;
    }

    /// <summary>
    /// Transfers a list of Entities (only the ones also in the Model). Remark : former result is cleared (OCCT's
    /// documentation; in fact only the process is: the former shapes stay).
    /// </summary>
    public virtual void TransferList(NCollection_HSequence<Standard_Transient>? list, Message_ProgressRange theProgress = default)
    {
        // OCCT: the former shapes aren't cleared, despite the documentation
        if (!BeginTransfer())
        {
            return;
        }
        if (list is null)
        {
            return;
        }
        var TP = new Transfer_TransferOutput(theProc, theModel);
        int i, nb = list.Length();
        using Message_Messenger.StreamBuffer sout = theProc.Messenger().SendInfo();

        if (theProc.TraceLevel() > 1)
        {
            sout.Append("--  Transfer(Read-List) : ").Append(nb).Append(" Items").Append('\n');
        }
        using var aPS = new Message_ProgressScope(theProgress, null, nb);
        for (i = 1; i <= nb && aPS.More(); i++)
        {
            Message_ProgressRange aRange = aPS.Next();
            Standard_Transient? ent = list.Value(i);
            if (theModel.Number(ent) == 0)
            {
                continue;
            }

            if (theProc.TraceLevel() > 1)
            {
                sout.Append("--  Transfer(Read-List), Item ").Append(i).Append(" : ");
                using (var aStream = new StringWriter(sout.Stream()))
                {
                    theModel.Print(ent, aStream);
                }
                sout.Append('\n');
            }
            TP.Transfer(ent, aRange);
            theProc.SetRoot(ent);
        }
        EndTransfer();
    }

    /// <summary>Returns True if the LAST Transfer/TransferRoots was a success.</summary>
    public bool IsDone()
    {
        return theDone;
    }

    //   ######    RESULTAT : SHAPES    ######

    /// <summary>Returns the count of produced Shapes (roots).</summary>
    public int NbShapes()
    {
        return theShapes.Length();
    }

    /// <summary>Returns the complete list of produced Shapes.</summary>
    public NCollection_HSequence<TopoDS_Shape> Shapes()
    {
        return theShapes;
    }

    /// <summary>Returns a Shape given its rank, by default the first one.</summary>
    public TopoDS_Shape Shape(int num = 1)
    {
        return theShapes.Value(num);
    }

    /// <summary>
    /// Returns a Shape produced from a given entity (if it was individually transferred or if an entity is a
    /// subpart of the one which was transferred). Returns a Null Shape if there is no result.
    /// </summary>
    public TopoDS_Shape ShapeResult(Standard_Transient? ent)
    {
        return TransferBRep.ShapeResult(theProc, ent);
    }

    /// <summary>
    /// Returns a unique Shape for the result : a void shape if no result; if the result is only one shape, this
    /// shape; else, a compound of the shapes (frozen).
    /// </summary>
    public TopoDS_Shape OneShape()
    {
        TopoDS_Shape res = new TopoDS_Shape();
        int nb = theShapes.Length();
        if (nb == 0)
        {
            return res;
        }
        else if (nb == 1)
        {
            return theShapes.Value(1);
        }
        else
        {
            TopoDS_Compound C = new TopoDS_Compound();
            var B = new BRep_Builder();
            B.MakeCompound(ref C);
            for (int i = 1; i <= nb; i++)
            {
                B.Add(C, theShapes.Value(i));
            }
            C.TShape()?.FreezeGraph(); // OCCT.NET: published
            return C;
        }
    }

    //   ######    RESULTAT : TRANSIENTS    ######

    /// <summary>Returns the count of produced Transient Results (roots).</summary>
    public int NbTransients()
    {
        return theTransi.Length();
    }

    /// <summary>Returns the complete list of produced Transient Results.</summary>
    public NCollection_HSequence<Standard_Transient> Transients()
    {
        return theTransi;
    }

    /// <summary>Returns a Transient Root Result, given its rank (by default the first one).</summary>
    public Standard_Transient? Transient(int num = 1)
    {
        return theTransi.Value(num);
    }

    //   ######    CHECKS    ######

    /// <summary>
    /// Checks the Result of last Transfer (individual or roots). Returns True if there is NO FAIL at all (regardless
    /// Warnings). If <paramref name="withprint"/> is True, also sends Checks on standard output.
    /// </summary>
    public bool CheckStatusResult(bool withprint)
    {
        var chl = new Interface_CheckIterator();
        if (theProc is not null)
        {
            chl = theProc.CheckList(false);
        }
        if (withprint && theProc is not null && theProc.Messenger() is not null)
        {
            using Message_Messenger.StreamBuffer aBuffer = theProc.Messenger().SendInfo();
            using var aStream = new StringWriter(aBuffer.Stream());
            chl.Print(aStream, theModel, false);
        }
        return chl.IsEmpty(true);
    }

    /// <summary>Checks the Result of last Transfer (individual or roots) and returns the produced list.</summary>
    public Interface_CheckIterator CheckListResult()
    {
        if (theProc is not null)
        {
            return theProc.CheckList(false);
        }
        var chbid = new Interface_CheckIterator();
        return chbid;
    }

    /// <summary>Returns the TransientProcess. It records information about the very last transfer done.</summary>
    public Transfer_TransientProcess? TransientProcess()
    {
        return theProc;
    }

    protected bool theDone;
    protected Transfer_TransientProcess? theProc;

    private Interface_Protocol? theProto;
    private Transfer_ActorOfTransientProcess? theActor;
    private Interface_InterfaceModel? theModel;
    private int theFilest;
    private bool theNewpr;
    private readonly NCollection_HSequence<TopoDS_Shape> theShapes;
    private readonly NCollection_HSequence<Standard_Transient> theTransi;
}
