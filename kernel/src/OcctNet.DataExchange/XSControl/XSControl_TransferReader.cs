// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferReader.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferReader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_PrintCount;

using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// A TransferReader performs, manages, handles results of, transfers done when reading a file (i.e. from
/// entities of an InterfaceModel, to objects for Imagine) Running is organised around basic tools :
/// TransientProcess and its Actor, results are Binders and CheckIterators. It implies control by a
/// Controller (which prepares the Actor as required) Getting results can be done directly on
/// TransientProcess, but these are immediate "last produced" results. Each transfer of an entity gives a
/// final result, but also possible intermediate data, and checks, which can be attached to sub-entities.
/// Hence, final results (which intermediates and checks) are recorded as ResultFromModel and can be queried
/// individually. Some more direct access are given for results which are Transient or Shapes
/// </summary>
public class XSControl_TransferReader : Standard_Transient
{
    //  ****    UTILITAIRE DE STATISTIQUES GENERALES

    // BinderStatus returns a value:
    // 0 Binder Null.   1 void  2 Warning only  3 Fail only
    // 11 Result OK. 12 Result+Warning. 13 Result+Fail
    // (OCCT writes the message into a char[250] buffer: here an out string)

    private static int BinderStatus(Transfer_Binder? binder, out string mess)
    {
        int stat = 0;
        mess = "";
        if (binder is null)
        {
            mess = "(no data recorded)";
            return 0;
        }
        Interface_CheckStatus cst = binder.Check().Status();
        if (cst == Interface_CheckOK)
        {
            stat = 11;
            if (binder.HasResult())
            {
                mess = Sprintf("%s", binder.ResultTypeName());
            }
            else
            {
                mess = "(no result)";
                stat = 1;
            }
        }
        else if (cst == Interface_CheckWarning)
        {
            stat = 12;
            if (binder.HasResult())
            {
                mess = Sprintf("%s  (+ warning)", binder.ResultTypeName());
            }
            else
            {
                mess = "(warning)";
                stat = 2;
            }
        }
        else if (cst == Interface_CheckFail)
        {
            stat = 13;
            if (binder.HasResult())
            {
                mess = Sprintf("%s  (+ FAIL)", binder.ResultTypeName());
            }
            else
            {
                mess = "(FAIL)";
                stat = 3;
            }
        }
        return stat;
    }

    private static void PrintPercent(Message_Messenger sout, string mess, int nb, int nl)
    {
        if (nb <= 0 || nl == 0)
        {
            return;
        }
        using Message_Messenger.StreamBuffer aSender = sout.SendInfo();
        aSender.Append("******      ").Append(mess).Append(": ");
        if (nb == nl)
        {
            aSender.Append("100 %").Append('\n');
        }
        else if (nb * 100 / nl == 0)
        {
            aSender.Append("< 1 %").Append('\n');
        }
        else
        {
            aSender.Append(nb * 100 / nl < 10 ? "  " : " ").Append(nb * 100 / nl).Append(" %").Append('\n');
        }
    }

    /// <summary>
    /// Creates a TransferReader, empty
    /// </summary>
    public XSControl_TransferReader()
    {
    }

    /// <summary>
    /// Sets a Controller. It is required to generate the Actor. Elsewhere, the Actor must be provided directly
    /// </summary>
    public void SetController(XSControl_Controller? control)
    {
        myController = control;
        myActor = null;
        Clear(-1);
    }

    /// <summary>
    /// Sets the Actor directly : this value will be used if the Controller is not set
    /// </summary>
    public void SetActor(Transfer_ActorOfTransientProcess? theActor)
    {
        myActor = theActor;
    }

    /// <summary>
    /// Returns the Actor, determined by the Controller, or if this one is unknown, directly set. Once it has
    /// been defined, it can then be edited.
    /// </summary>
    public Transfer_ActorOfTransientProcess? Actor()
    {
        if (myActor is null && myController is not null && myModel is not null)
        {
            myActor = myController.ActorRead(myModel);
        }
        return myActor;
    }

    /// <summary>
    /// Sets an InterfaceModel. This causes former results, computed from another one, to be lost (see also
    /// Clear)
    /// </summary>
    public void SetModel(Interface_InterfaceModel? model)
    {
        myModel = model;
        if (myTP is not null)
        {
            myTP.SetModel(model);
        }
    }

    /// <summary>
    /// Sets a Graph and its InterfaceModel (calls SetModel)
    /// </summary>
    public void SetGraph(Interface_HGraph? graph)
    {
        if (graph is null)
        {
            myModel = null;
        }
        else
        {
            myModel = graph.Graph().Model();
        }

        myGraph = graph;

        if (myTP is not null)
        {
            myTP.SetGraph(graph);
        }
    }

    /// <summary>
    /// Returns the currently set InterfaceModel
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return myModel;
    }

    /// <summary>
    /// Sets a Context : according to receiving appli, to be interpreted by the Actor
    /// </summary>
    public void SetContext(string name, Standard_Transient? ctx)
    {
        myContext.Bind(name, ctx);
    }

    /// <summary>
    /// Returns the Context attached to a name, if set and if it is Kind of the type, else a Null Handle Returns
    /// True if OK, False if no Context
    /// </summary>
    public bool GetContext(string name, Type? type, ref Standard_Transient? ctx)
    {
        if (myContext.IsEmpty())
        {
            return false;
        }
        if (!myContext.Find(name, out ctx))
        {
            ctx = null;
        }
        if (ctx is null)
        {
            return false;
        }
        if (type is null)
        {
            return true;
        }
        if (!ctx.IsKind(type))
        {
            ctx = null;
        }
        return ctx is not null;
    }

    /// <summary>
    /// Returns (modifiable) the whole definition of Context Rather for internal use (ex.: preparing and setting
    /// in once)
    /// </summary>
    public NCollection_DataMap<string, Standard_Transient>? Context()
    {
        return myContext;
    }

    /// <summary>
    /// Sets a new value for (loaded) file name
    /// </summary>
    /// <summary>
    /// The parameter set the reader reads its statics from (OCCT.NET: OCCT's process-wide statics, decision
    /// 1181); set by its work session.
    /// </summary>
    public Interface_StaticSet? Statics()
    {
        return myStatics;
    }

    /// <summary>Sets the parameter set the reader reads its statics from (OCCT.NET).</summary>
    public void SetStatics(Interface_StaticSet? theStatics)
    {
        myStatics = theStatics;
    }

    // C++'s assignment of a map: theTarget becomes a copy of theSource (OCCT.NET).
    internal static void AssignContext(NCollection_DataMap<string, Standard_Transient> theTarget,
                                       NCollection_DataMap<string, Standard_Transient> theSource)
    {
        if (ReferenceEquals(theTarget, theSource))
        {
            return;
        }
        theTarget.Clear();
        foreach (KeyValuePair<string, Standard_Transient> anItem in theSource)
        {
            theTarget.Bind(anItem.Key, anItem.Value);
        }
    }

    public void SetFileName(string theName)
    {
        myFileName = theName;
    }

    /// <summary>
    /// Returns actual value of file name
    /// </summary>
    public string FileName()
    {
        return myFileName;
    }

    /// <summary>
    /// Clears data, according mode : -1 all 0 nothing done +1 final results +2 working data (model, context,
    /// transfer process)
    /// </summary>
    public void Clear(int mode)
    {
        if ((mode & 1) != 0)
        {
            myResults.Clear();
            myShapeResult = null;
        }
        if ((mode & 2) != 0)
        {
            myModel = null;
            myGraph = null;
            myTP = null;
            myActor = null;
            myFileName = "";
        }
    }

    /// <summary>
    /// Returns the currently used TransientProcess It is computed from the model by TransferReadRoots, or by
    /// BeginTransferRead
    /// </summary>
    public Transfer_TransientProcess? TransientProcess()
    {
        return myTP;
    }

    /// <summary>
    /// Forces the TransientProcess Remark : it also changes the Model and the Actor, from those recorded in the
    /// new TransientProcess
    /// </summary>
    public void SetTransientProcess(Transfer_TransientProcess? theTP)
    {
        myTP = theTP;
    }

    /// <summary>
    /// Records a final result of transferring an entity This result is recorded as a ResultFromModel, taken from
    /// the TransientProcess Returns True if a result is available, False else
    /// </summary>
    public bool RecordResult(Standard_Transient? ent)
    {
        if (myModel is null || myTP is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        TCollection_HAsciiString lab = myModel.StringLabel(ent);

        Transfer_ResultFromModel res = new Transfer_ResultFromModel();
        res.Fill(myTP, ent);

        //   Case of Shape result : for main result, make HShape ...
        Transfer_Binder binder = res.MainResult().Binder();
        TransferBRep_ShapeBinder shb = binder as TransferBRep_ShapeBinder;
        if (shb is not null)
        {
            Transfer_SimpleBinderOfTransient trb = new Transfer_SimpleBinderOfTransient();
            trb.SetResult(new TopoDS_HShape(shb.Result()));
            trb.Merge(binder);
            res.MainResult().SetBinder(trb);
        }

        res.SetFileName(myFileName);
        myResults.Bind(num, res);
        return true;
    }

    /// <summary>
    /// Returns True if a final result is recorded for an entity Remark that it can bring no effective result if
    /// transfer has completely failed (FinalResult brings only fail messages ...)
    /// </summary>
    public bool IsRecorded(Standard_Transient? ent)
    {
        if (myModel is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!myResults.IsBound(num))
        {
            return false;
        }
        return (myResults.Find(num).DynamicType() == typeof(Transfer_ResultFromModel));
    }

    /// <summary>
    /// Returns True if a final result is recorded AND BRINGS AN EFFECTIVE RESULT (else, it brings only fail
    /// messages)
    /// </summary>
    public bool HasResult(Standard_Transient? ent)
    {
        if (myModel is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!myResults.IsBound(num))
        {
            return false;
        }
        Transfer_ResultFromModel fr = myResults.Find(num) as Transfer_ResultFromModel;
        if (fr is null)
        {
            return false;
        }
        return fr.HasResult();
    }

    /// <summary>
    /// Returns the list of entities to which a final result is attached (i.e. processed by RecordResult)
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? RecordedList()
    {
        NCollection_HSequence<Standard_Transient> li =
            new NCollection_HSequence<Standard_Transient>();
        if (myModel is null)
        {
            return li;
        }
        int i, nb = myModel.NbEntities();
        for (i = 1; i <= nb; i++)
        {
            if (myResults.IsBound(i))
            {
                if (myResults.Find(i) is not null)
                {
                    li.Append(myModel.Value(i));
                }
            }
        }
        return li;
    }

    /// <summary>
    /// Note that an entity has been required for transfer but no result at all is available (typically : case
    /// not implemented) It is not an error, but it gives a specific status : Skipped Returns True if done, False
    /// if &lt;ent&gt; is not in starting model
    /// </summary>
    public bool Skip(Standard_Transient? ent)
    {
        if (myModel is null || myTP is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        myResults.Bind(num, ent);
        return true;
    }

    /// <summary>
    /// Returns True if an entity is noted as skipped
    /// </summary>
    public bool IsSkipped(Standard_Transient? ent)
    {
        if (myModel is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!myResults.IsBound(num))
        {
            return false;
        }
        return (myResults.Find(num).DynamicType() != typeof(Transfer_ResultFromModel));
    }

    /// <summary>
    /// Returns True if an entity has been asked for transfert, hence it is marked, as : Recorded (a computation
    /// has ran, with or without an effective result), or Skipped (case ignored)
    /// </summary>
    public bool IsMarked(Standard_Transient? ent)
    {
        if (myModel is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!myResults.IsBound(num))
        {
            return false;
        }
        if (myResults.Find(num) is null)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Returns the final result recorded for an entity, as such
    /// </summary>
    public Transfer_ResultFromModel? FinalResult(Standard_Transient? ent)
    {
        Transfer_ResultFromModel res = null;
        if (myModel is null)
        {
            return res;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return res;
        }
        if (!myResults.IsBound(num))
        {
            return res;
        }
        res = (myResults.Find(num) as Transfer_ResultFromModel);
        return res;
    }

    /// <summary>
    /// Returns the label attached to an entity recorded for final, or an empty string if not recorded
    /// </summary>
    public string FinalEntityLabel(Standard_Transient? ent)
    {
        Transfer_ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return "";
        }
        return resu.MainLabel();
    }

    /// <summary>
    /// Returns the number attached to the entity recorded for final, or zero if not recorded (looks in the
    /// ResultFromModel)
    /// </summary>
    public int FinalEntityNumber(Standard_Transient? ent)
    {
        Transfer_ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return 0;
        }
        return resu.MainNumber();
    }

    /// <summary>
    /// Returns the final result recorded for a NUMBER of entity (internal use). Null if out of range
    /// </summary>
    public Transfer_ResultFromModel? ResultFromNumber(int num)
    {
        Transfer_ResultFromModel res = null;
        if (num < 1 || num > myModel.NbEntities())
        {
            return res;
        }
        if (!myResults.IsBound(num))
        {
            return res;
        }
        res = (myResults.Find(num) as Transfer_ResultFromModel);
        return res;
    }

    /// <summary>
    /// Returns the resulting object as a Transient Null Handle if no result or result not transient
    /// </summary>
    public Standard_Transient? TransientResult(Standard_Transient? ent)
    {
        Standard_Transient tres = null;
        Transfer_ResultFromModel res = FinalResult(ent);
        if (res is null)
        {
            return tres;
        }
        Transfer_ResultFromTransient mres = res.MainResult();
        if (mres is null)
        {
            return tres;
        }
        Transfer_SimpleBinderOfTransient bnd = mres.Binder() as Transfer_SimpleBinderOfTransient;
        if (bnd is null)
        {
            return tres;
        }
        if (!bnd.HasResult())
        {
            return tres;
        }
        return bnd.Result();
    }

    /// <summary>
    /// Returns the resulting object as a Shape Null Shape if no result or result not a shape
    /// </summary>
    public TopoDS_Shape ShapeResult(Standard_Transient? ent)
    {
        TopoDS_Shape tres = default; // DOIT RESTER null
        Transfer_ResultFromModel res = FinalResult(ent);
        if (res is null)
        {
            return tres;
        }
        Transfer_ResultFromTransient mres = res.MainResult();
        if (mres is null)
        {
            return tres;
        }
        XSControl_Utils xu = new XSControl_Utils();
        TopoDS_Shape sh = xu.BinderShape(mres.Binder());

        //   Ooh the ugly wart
        // OCCT.NET: the static is the reader's parameter set's (OCCT: the process-wide one; without a set, its
        // default 0.01); OCCT encodes the regularity into the recorded shape itself (ShapeFix::EncodeRegularity), as
        // the port does into a reader's working shape (unfrozen: STEPControl_ActorRead's, decision 1273), so that
        // results sharing it keep sharing it; a shape holding frozen shapes is encoded into a copy (BRepLib's
        // functional twin, decision 1224)
        double tolang = (myStatics is null ? 0.01 : myStatics.RVal("read.encoderegularity.angle"));
        if (tolang <= 0 || sh.IsNull())
        {
            return sh;
        }
        if (!BRepTools.HoldsFrozen(sh))
        {
            ShapeFix.EncodeRegularity(sh, tolang);
            return sh;
        }
        sh = BRepLib.RegularityEncoded(sh, tolang);
        return sh;
    }

    /// <summary>
    /// Clears recorded result for an entity, according mode &lt;mode&gt; = -1 : true, complete, clearing
    /// (erasing result) &lt;mode&gt; &gt;= 0 : simple "stripping", see ResultFromModel, in particular, 0 for
    /// simple internal strip, 10 for all but final result, 11 for all : just label, status and filename are kept
    /// Returns True when done, False if nothing was to clear
    /// </summary>
    public bool ClearResult(Standard_Transient? ent, int mode)
    {
        if (myModel is null)
        {
            return false;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!myResults.IsBound(num))
        {
            return false;
        }
        if (mode < 0)
        {
            myResults.ChangeFind(num) = null;
        }
        else
        {
            Transfer_ResultFromModel resu = myResults.Find(num) as Transfer_ResultFromModel;
            if (resu is null)
            {
                return false;
            }
            resu.Strip(mode);
        }
        return true;
    }

    /// <summary>
    /// Returns an entity from which a given result was produced. If &lt;mode&gt; = 0 (D), searches in last root
    /// transfers If &lt;mode&gt; = 1, searches in last (root &amp; sub) transfers If &lt;mode&gt; = 2, searches
    /// in root recorded results If &lt;mode&gt; = 3, searches in all (root &amp; sub) recordeds &lt;res&gt; can
    /// be, either a transient object (result itself) or a binder. For a binder of shape, calls
    /// EntityFromShapeResult Returns a Null Handle if &lt;res&gt; not recorded
    /// </summary>
    public Standard_Transient? EntityFromResult(Standard_Transient? res, int mode = 0)
    {
        Standard_Transient nulh = null;
        //  case of the shape
        XSControl_Utils xu = new XSControl_Utils();
        TopoDS_Shape sh = xu.BinderShape(res);
        if (!sh.IsNull())
        {
            return EntityFromShapeResult(sh, mode);
        }

        Transfer_Binder abinder = null;
        Transfer_Binder binder = res as Transfer_Binder;
        int i, j, nb;

        if (mode == 0 || mode == 1)
        {
            //  we look in the TransientProcess (Roots or all Mappeds)
            if (myTP is not null)
            {
                nb = (mode == 0 ? myTP.NbRoots() : myTP.NbMapped());
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? myModel.Number(myTP.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    abinder = myTP.MapItem(i);
                    if (abinder is null)
                    {
                        continue;
                    }
                    if (binder is not null)
                    {
                        if (binder == abinder)
                        {
                            return myTP.Mapped(i);
                        }
                        continue;
                    }
                    Transfer_SimpleBinderOfTransient trb = abinder as Transfer_SimpleBinderOfTransient;
                    if (trb is null)
                    {
                        continue;
                    }
                    if (trb.Result() == res)
                    {
                        return myTP.Mapped(i);
                    }
                }
            }
            return nulh; // Null
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    return nulh;
                }
                NCollection_HSequence<Standard_Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    Transfer_ResultFromTransient rft = list.Value(ir) as Transfer_ResultFromTransient;
                    if (rft is null)
                    {
                        continue;
                    }
                    if (rft.Binder() == binder)
                    {
                        return rft.Start();
                    }
                }
            }
        }

        //  other cases not yet implemented
        return nulh;
    }

    /// <summary>
    /// Returns an entity from which a given shape result was produced Returns a Null Handle if &lt;res&gt; not
    /// recorded or not a Shape
    /// </summary>
    public Standard_Transient? EntityFromShapeResult(TopoDS_Shape res, int mode = 0)
    {
        Standard_Transient nulh = null, samesh = null, partner = null;
        if (res.IsNull())
        {
            return nulh;
        }
        int i, j, nb;

        XSControl_Utils xu = new XSControl_Utils();
        if (mode == 0 || mode == 1 || mode == -1)
        {
            //  we look in the TransientProcess
            if (myTP is not null)
            {
                nb = (mode == 0 ? myTP.NbRoots() : myTP.NbMapped());
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? myModel.Number(myTP.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    Standard_Transient ent = myTP.Mapped(i);
                    TopoDS_Shape sh = TransferBRep.ShapeResult(myTP, ent);
                    if (!sh.IsNull())
                    {
                        if (sh == res)
                        {
                            return ent;
                        }
                        // lesser priorities : Same (always) or Partner (mode < 0)
                        if (sh.IsSame(res))
                        {
                            samesh = ent;
                        }
                        if (mode == -1 && sh.IsPartner(res))
                        {
                            partner = ent;
                        }
                    }
                }
            }
            //    Here, no true equality found. Lesser priorities: Same then Partner
            if (samesh is not null)
            {
                return samesh;
            }
            if (partner is not null)
            {
                return partner; // calculated if mode = -1
            }
            return nulh;
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    continue;
                }

                NCollection_HSequence<Standard_Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    Transfer_ResultFromTransient rft = list.Value(ir) as Transfer_ResultFromTransient;
                    if (rft is null)
                    {
                        continue;
                    }
                    TopoDS_Shape sh = xu.BinderShape(rft.Binder());
                    if (!sh.IsNull() && sh == res)
                    {
                        return rft.Start();
                    }
                }
            }
        }

        return nulh;
    }

    /// <summary>
    /// Returns the list of entities from which some shapes were produced : it corresponds to a loop on
    /// EntityFromShapeResult, but is optimised
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? EntitiesFromShapeList(NCollection_HSequence<TopoDS_Shape>? res, int mode = 0)
    {
        NCollection_HSequence<Standard_Transient> lt =
            new NCollection_HSequence<Standard_Transient>();
        if (res is null)
        {
            return lt;
        }
        NCollection_Map<TopoDS_Shape> shapes = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        //  We convert res to a map, for rapid presence test
        int i, j, nb = res.Length();
        if (nb == 0)
        {
            return lt;
        }
        for (i = 1; i <= nb; i++)
        {
            shapes.Add(res.Value(i));
        }

        //  Now, search and registration

        XSControl_Utils xu = new XSControl_Utils();
        if (mode == 0 || mode == 1)
        {
            //  we look in the TransientProcess
            if (myTP is not null)
            {
                nb = (mode == 0 ? myTP.NbRoots() : myTP.NbMapped());
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? myModel.Number(myTP.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    TopoDS_Shape sh = xu.BinderShape(myTP.MapItem(i));
                    if (!sh.IsNull() && shapes.Contains(sh))
                    {
                        lt.Append(myTP.Mapped(i));
                        j = nb; // skl (for looking for entities in checkbrep)
                    }
                }
            }
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    continue;
                }

                NCollection_HSequence<Standard_Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    Transfer_ResultFromTransient rft = list.Value(i) as Transfer_ResultFromTransient; // OCCT: i, not ir
                    if (rft is null)
                    {
                        continue;
                    }
                    TopoDS_Shape sh = xu.BinderShape(rft.Binder());
                    if (!sh.IsNull() && shapes.Contains(sh))
                    {
                        lt.Append(rft.Start());
                    }
                }
            }
        }

        return lt;
    }

    /// <summary>
    /// Returns the CheckList resulting from transferring &lt;ent&gt;, i.e. stored in its recorded form
    /// ResultFromModel (empty if transfer successful or not recorded ...) If &lt;ent&gt; is the Model, returns
    /// the complete cumulated check-list, &lt;level&gt; is ignored If &lt;ent&gt; is an entity of the Model,
    /// &lt;level&gt; applies as follows &lt;level&gt; : -1 for &lt;ent&gt; only, LAST transfer
    /// (TransientProcess) &lt;level&gt; : 0 for &lt;ent&gt; only (D) 1 for &lt;ent&gt; and its immediate
    /// subtransfers, if any 2 for &lt;ent&gt; and subtransferts at all levels
    /// </summary>
    public Interface_CheckIterator CheckList(Standard_Transient? ent, int level = 0)
    {
        Interface_CheckIterator chl = new Interface_CheckIterator();
        if (myModel is null || ent is null)
        {
            return chl;
        }
        //  Check-List COMPLETE ... the entire Model
        if (ent == myModel)
        {
            int i, nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = ResultFromNumber(i);
                if (rec is not null)
                {
                    Interface_CheckIterator chiter = rec.CheckList(false, 2);
                    chl.Merge(chiter);
                }
            }
        }
        //  Check-List on a LIST ...
        else if (ent is NCollection_HSequence<Standard_Transient> list)
        {
            int i, nb = list.Length();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = FinalResult(list.Value(i));
                if (rec is not null)
                {
                    Interface_CheckIterator chiter = rec.CheckList(false, level);
                    chl.Merge(chiter);
                }
            }
        }

        //  otherwise, Check-List on an entity: Last or FinalResult
        else if (level < 0)
        {
            if (myTP is null)
            {
                return chl;
            }
            chl.Add(myTP.Check(ent), myModel.Number(ent));
        }
        else
        {
            Transfer_ResultFromModel rec = FinalResult(ent);
            if (rec is null)
            {
                return chl;
            }
            chl = rec.CheckList(false, level); // manque level ...
        }
        if (ent == myModel)
        {
            chl.SetName("XSControl : CheckList complete Model");
        }
        else if (level < 0)
        {
            chl.SetName("XSControl : CheckList Last");
        }
        else if (level == 0)
        {
            chl.SetName("XSControl : CheckList Final Main");
        }
        else if (level == 1)
        {
            chl.SetName("XSControl : CheckList Final Main+Subs");
        }
        else if (level >= 2)
        {
            chl.SetName("XSControl : CheckList Final Complete");
        }
        return chl;
    }

    /// <summary>
    /// Returns True if an entity (with a final result) has checks : - failsonly = False : any kind of check
    /// message - failsonly = True : fails only Returns False if &lt;ent&gt; is not recorded
    /// </summary>
    public bool HasChecks(Standard_Transient? ent, bool failsonly)
    {
        Transfer_ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return false;
        }
        int stat = (int)resu.ComputeCheckStatus(false);
        if (stat == 0)
        {
            return false;
        }
        if (stat > 1)
        {
            return true;
        }
        return (!failsonly);
    }

    /// <summary>
    /// Returns the list of starting entities to which a given check status is attached, IN FINAL RESULTS
    /// &lt;ent&gt; can be an entity, or the model to query all entities Below, "entities" are, either
    /// &lt;ent&gt; plus its sub-transferred, or all the entities of the model &lt;check&gt; = -2 , all entities
    /// whatever the check (see result) &lt;check&gt; = -1 , entities with no fail (warning allowed)
    /// &lt;check&gt; = 0 , entities with no check at all &lt;check&gt; = 1 , entities with warning but no fail
    /// &lt;check&gt; = 2 , entities with fail &lt;result&gt; : if True, only entities with an attached result
    /// Remark : result True and check=0 will give an empty list
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? CheckedList(Standard_Transient? ent, Interface_CheckStatus withcheck = Interface_CheckAny, bool level = true)
    {
        NCollection_HSequence<Standard_Transient> res =
            new NCollection_HSequence<Standard_Transient>();
        if (ent is null)
        {
            return res;
        }

        if (ent == myModel)
        {
            int i, nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = ResultFromNumber(i);
                if (rec is not null)
                {
                    res.Append(rec.CheckedList(withcheck, level));
                }
            }
        }
        else if (ent is NCollection_HSequence<Standard_Transient> list)
        {
            int i, nb = list.Length();
            for (i = 1; i <= nb; i++)
            {
                Transfer_ResultFromModel rec = FinalResult(list.Value(i));
                if (rec is not null)
                {
                    res.Append(rec.CheckedList(withcheck, level));
                }
            }
        }
        else
        {
            Transfer_ResultFromModel rec = FinalResult(ent);
            if (rec is not null)
            {
                res = rec.CheckedList(withcheck, level);
            }
        }
        return res;
    }

    /// <summary>
    /// Defines a new TransferProcess for reading transfer Returns True if done, False if data are not properly
    /// defined (the Model, the Actor for Read)
    /// </summary>
    public bool BeginTransfer()
    {
        if (myModel is null)
        {
            return false;
        }
        if (Actor() is null)
        {
            return false;
        }
        myShapeResult = null;

        if (myTP is null)
        {
            myTP = new Transfer_TransientProcess(myModel.NbEntities());
        }

        Transfer_ActorOfTransientProcess actor = null;
        myTP.SetActor(actor); // -> Reset
        actor = Actor();
        myTP.SetActor(actor); // Set properly
        myTP.SetErrorHandle(true);
        NCollection_DataMap<string, Standard_Transient> aTPContext = myTP.Context();
        AssignContext(aTPContext, myContext); // C++'s map assignment: a copy
        return true;
    }

    /// <summary>
    /// Tells if an entity is recognized as a valid candidate for Transfer. Calls method Recognize from the Actor
    /// (if known)
    /// </summary>
    public bool Recognize(Standard_Transient? ent)
    {
        if (myActor is null)
        {
            return false;
        }
        return myActor.Recognize(ent);
    }

    /// <summary>
    /// Commands the transfer on reading for an entity to data for Imagine, using the selected Actor for Read
    /// Returns count of transferred entities, ok or with fails (0/1) If &lt;rec&gt; is True (D), the result is
    /// recorded by RecordResult
    /// </summary>
    public int TransferOne(Standard_Transient? ent, bool rec = true, Message_ProgressRange theProgress = default)
    {
        if (myActor is null || myModel is null)
        {
            return 0;
        }

        if (myTP is null)
        {
            if (!BeginTransfer())
            {
                return 0;
            }
        }

        using Message_Messenger.StreamBuffer sout = myTP.Messenger().SendInfo();
        int level = myTP.TraceLevel();

        Transfer_TransferOutput TP = new Transfer_TransferOutput(myTP, myModel);
        if (myGraph is null)
        {
            myTP.SetModel(myModel);
        }
        else
        {
            myTP.SetGraph(myGraph);
        }

        //  for the log-file
        if (level > 1)
        {
            int num = myModel.Number(ent);
            TCollection_HAsciiString lab = myModel.StringLabel(ent);
            sout.Append("\n*******************************************************************\n");
            sout.Append("******           Transferring one Entity                     ******");
            sout.Append('\n');
            if (lab is not null)
            {
                sout.Append("******    N0 in file : ");
                sout.Append(Interface_MSG.Blanks(num, 5));
                sout.Append(num);
                sout.Append("      Ident : ");
                sout.Append(lab.ToCString());
                sout.Append(Interface_MSG.Blanks(14 - lab.Length()));
                sout.Append("******\n");
            }
            sout.Append("******    Type : ");
            sout.Append(myModel.TypeName(ent, false));
            sout.Append(Interface_MSG.Blanks(44 - myModel.TypeName(ent, false).Length));
            sout.Append("******");
            sout.Append("\n*******************************************************************\n");
        }

        //  only difference between TransferRoots and TransferOne
        int res = 0;
        Standard_Transient obj = ent;
        TP.Transfer(obj, theProgress);
        if (theProgress.UserBreak())
        {
            return res;
        }
        myTP.SetRoot(obj);

        //  Result ...
        Transfer_Binder binder = myTP.Find(obj);
        if (binder is null)
        {
            return res;
        }
        if (rec)
        {
            RecordResult(obj);
        }

        if (!binder.HasResult())
        {
            return res;
        }
        res++;

        return res;
    }

    /// <summary>
    /// Commands the transfer on reading for a list of entities to data for Imagine, using the selected Actor for
    /// Read Returns count of transferred entities, ok or with fails (0/1) If &lt;rec&gt; is True (D), the
    /// results are recorded by RecordResult
    /// </summary>
    public int TransferList(NCollection_HSequence<Standard_Transient>? list, bool rec = true, Message_ProgressRange theProgress = default)
    {
        if (myActor is null || myModel is null)
        {
            return 0;
        }

        if (myTP is null)
        {
            if (!BeginTransfer())
            {
                return 0;
            }
        }

        int level = myTP.TraceLevel();

        Transfer_TransferOutput TP = new Transfer_TransferOutput(myTP, myModel);
        if (myGraph is null)
        {
            myTP.SetModel(myModel);
        }
        else
        {
            myTP.SetGraph(myGraph);
        }

        int i, nb = list.Length();

        //   For the log-file
        if (level > 0)
        {
            StringWriter sout = new StringWriter(); // OCCT: one stream buffer, sent at its end
            sout.Write("\n*******************************************************************\n");
            sout.Write("******           Transferring a list of ");
            sout.Write(Interface_MSG.Blanks(nb, 5));
            sout.Write(" Entities       ******");
            sout.Write('\n');
            sout.Write("\n*******************************************************************\n");

            IFSelect_SignatureList sl = new IFSelect_SignatureList();
            for (i = 1; i <= nb; i++)
            {
                sl.Add(list.Value(i), myModel.TypeName(list.Value(i), false));
            }
            sl.SetName("Entities to Transfer");
            sl.PrintCount(sout);
            sout.Write("\n*******************************************************************\n");
            myTP.Messenger().SendInfo().Append(sout.ToString()).Flush();
        }

        //  only difference between TransferRoots and TransferOne
        int res = 0;
        nb = list.Length();
        Standard_Transient obj = null;
        Message_ProgressScope aPS = new Message_ProgressScope(theProgress, null, nb);
        for (i = 1; i <= nb && aPS.More(); i++)
        {
            obj = list.Value(i);
            TP.Transfer(obj, aPS.Next());
            myTP.SetRoot(obj);

            //  Result ...
            Transfer_Binder binder = myTP.Find(obj);
            if (binder is null)
            {
                continue;
            }
            if (rec)
            {
                RecordResult(obj);
            }

            if (!binder.HasResult())
            {
                continue;
            }
            res++;
        }
        return res;
    }

    /// <summary>
    /// Transfers the content of the current Interface Model to data handled by Imagine, starting from its Roots
    /// (determined by the Graph &lt;G&gt;), using the selected Actor for Read Returns the count of performed
    /// root transfers (i.e. 0 if none) or -1 if no actor is defined
    /// </summary>
    public int TransferRoots(Interface_Graph G, Message_ProgressRange theProgress = default)
    {
        if (myModel != G.Model())
        {
            return -1;
        }
        if (!BeginTransfer())
        {
            return -1;
        }
        int level = myTP.TraceLevel();

        Transfer_TransferOutput TP = new Transfer_TransferOutput(myTP, myModel);
        if (myGraph is null)
        {
            myTP.SetModel(myModel);
        }
        else
        {
            myTP.SetGraph(myGraph);
        }

        //   For the log-file
        if (level > 0)
        {
            Interface_EntityIterator roots = G.RootEntities();
            int nb = roots.NbEntities();
            StringWriter sout = new StringWriter(); // OCCT: one stream buffer, sent at its end
            sout.Write("\n*******************************************************************\n");
            sout.Write("******           Transferring the ");
            sout.Write(Interface_MSG.Blanks(nb, 5));
            sout.Write(" Root Entities        ******");
            sout.Write('\n');
            sout.Write("\n*******************************************************************\n");
            IFSelect_SignatureList sl = new IFSelect_SignatureList();
            for (roots.Start(); roots.More(); roots.Next())
            {
                sl.Add(roots.Value(), myModel.TypeName(roots.Value(), false));
            }
            sl.SetName("Entities to Transfer");
            sl.PrintCount(sout);
            sout.Write("\n*******************************************************************\n");
            myTP.Messenger().SendInfo().Append(sout.ToString()).Flush();
        }

        TP.TransferRoots(G, theProgress);
        if (theProgress.UserBreak())
        {
            return -1;
        }

        //  The transferred entities are noted as "asmain"
        int i, n = myTP.NbMapped();
        for (i = 1; i <= n; i++)
        {
            Standard_Transient ent = myTP.Mapped(i);
            Transfer_Binder bnd = myTP.MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            if (!bnd.HasResult())
            {
                continue;
            }
            RecordResult(ent);
        }

        //  Result ... we carefully note the Shapes
        myShapeResult = TransferBRep.Shapes(myTP, true);
        // ????  Et ici, il faut alimenter Imagine ...
        return myShapeResult.Length();
    }

    /// <summary>
    /// Clears the results attached to an entity if &lt;ents&gt; equates the starting model, clears all results
    /// </summary>
    public void TransferClear(Standard_Transient? ent, int level = 0)
    {
        if (myTP is null)
        {
            return;
        }
        if (ent == myModel)
        {
            myTP.Clear();
            return;
        }

        myTP.RemoveResult(ent, level);
        ClearResult(ent, -1);
    }

    /// <summary>
    /// Prints statistics on current Trace File, according &lt;what&gt; and &lt;mode&gt;. See PrintStatsProcess
    /// for details
    /// </summary>
    public void PrintStats(TextWriter sout, int what, int mode = 0)
    {
        //  To be improved ... !
        sout.Write("\n*******************************************************************\n");
        sout.Write("******        Statistics on Transfer (Read)                  ******");
        sout.Write('\n');
        sout.Write("\n*******************************************************************\n");
        if (what > 10)
        {
            sout.Write(" ***  Not yet implemented");
            sout.Write('\n');
            return;
        }
        if (what < 10)
        {
            sout.Write("******        Data recorded on Last Transfer                 ******");
            sout.Write('\n');
            PrintStatsProcess(myTP, what, mode);
        }
        //  remaining what = 10: we list the roots of final results
        sout.Write("******        Final Results                                  ******");
        sout.Write('\n');
        if (myModel is null)
        {
            sout.Write("****    Model unknown");
            sout.Write('\n');
            return;
        }
        NCollection_HSequence<Standard_Transient> list = RecordedList();
        int i, nb = list.Length();
        IFSelect_SignatureList counter = null;
        if (mode > 2)
        {
            counter = new IFSelect_SignatureList(mode == 6);
        }
        IFSelect_PrintCount pcm = IFSelect_CountByItem;
        if (mode == 6)
        {
            pcm = IFSelect_ListByItem;
        }

        sout.Write("****    Nb Recorded : ");
        sout.Write(nb);
        sout.Write(" : entities n0s : ");
        for (i = 1; i <= nb; i++)
        {
            Standard_Transient ent = list.Value(i);
            if (mode == 0)
            {
                sout.Write("  ");
                sout.Write(myModel.Number(ent));
                continue;
            }
            if (mode == 1 || mode == 2)
            {
                sout.Write("[ ");
                sout.Write(Interface_MSG.Blanks(i, 6));
                sout.Write(" ]:");
                myModel.Print(ent, sout);
                sout.Write("  Type:");
                sout.Write(myModel.TypeName(ent, false));
            }
            if (mode >= 3 && mode <= 6)
            {
                counter.Add(ent, myModel.TypeName(ent, false));
            }
        }
        if (counter is not null)
        {
            counter.PrintList(sout, myModel, pcm);
        }

        sout.Write('\n');
    }

    /// <summary>
    /// Returns the CheckList resulting from last TransferRead i.e. from TransientProcess itself, recorded from
    /// last Clear
    /// </summary>
    public Interface_CheckIterator LastCheckList()
    {
        Interface_CheckIterator chl = new Interface_CheckIterator();
        if (myTP is not null)
        {
            chl = myTP.CheckList(false);
        }
        return chl;
    }

    /// <summary>
    /// Returns the list of entities recorded as lastly transferred i.e. from TransientProcess itself, recorded
    /// from last Clear If &lt;roots&gt; is True , considers only roots of transfer If &lt;roots&gt; is False,
    /// considers all entities bound with result
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? LastTransferList(bool roots)
    {
        NCollection_HSequence<Standard_Transient> li =
            new NCollection_HSequence<Standard_Transient>();
        if (myTP is null)
        {
            return li;
        }
        int i, j, nb = (roots ? myTP.NbRoots() : myTP.NbMapped());
        for (j = 1; j <= nb; j++)
        {
            i = (roots ? myModel.Number(myTP.Root(j)) : j);
            Transfer_Binder bnd = myTP.MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            if (!bnd.HasResult())
            {
                continue;
            }
            li.Append(myTP.Mapped(i));
        }
        return li;
    }

    /// <summary>
    /// Returns a list of result Shapes If &lt;rec&gt; is True , sees RecordedList If &lt;rec&gt; is False, sees
    /// LastTransferList (last ROOT transfers) For each one, if it is a Shape, it is cumulated to the list If no
    /// Shape is found, returns an empty Sequence
    /// </summary>
    public NCollection_HSequence<TopoDS_Shape>? ShapeResultList(bool rec)
    {
        if (!rec)
        {
            if (myShapeResult is null)
            {
                myShapeResult = TransferBRep.Shapes(myTP, true);
            }
            if (myShapeResult is null)
            {
                myShapeResult = new NCollection_HSequence<TopoDS_Shape>();
            }
        }
        else
        {
            if (myShapeResult is null)
            {
                myShapeResult = new NCollection_HSequence<TopoDS_Shape>();
            }
            if (myModel is null)
            {
                return myShapeResult;
            }
            NCollection_HSequence<Standard_Transient> li = RecordedList();
            myShapeResult = new NCollection_HSequence<TopoDS_Shape>();
            int i, nb = myModel.NbEntities();
            TopoDS_Shape sh;
            for (i = 1; i <= nb; i++)
            {
                sh = ShapeResult(myModel.Value(i));
                if (!sh.IsNull())
                {
                    myShapeResult.Append(sh);
                }
            }
        }
        return myShapeResult;
    }

    /// <summary>
    /// This routines prints statistics about a TransientProcess It can be called, by a TransferReader, or
    /// isolately Prints are done on the default trace file &lt;what&gt; defines what kind of statistics are to
    /// be printed : 0 : basic figures 1 : root results 2 : all recorded (roots, intermediate, checked entities)
    /// 3 : abnormal records 4 : check messages (warnings and fails) 5 : fail messages &lt;mode&gt; is used
    /// according &lt;what&gt; : &lt;what&gt; = 0 : &lt;mode&gt; is ignored &lt;what&gt; = 1,2,3 : &lt;mode&gt;
    /// as follows : 0 (D) : just lists numbers of concerned entities in the model 1 : for each entity, gives
    /// number,label, type and result type and/or status (fail/warning...) 2 : for each entity, gives maximal
    /// information (i.e. checks) 3 : counts per type of starting entity (class type) 4 : counts per result type
    /// and/or status 5 : counts per couple (starting type / result type/status) 6 : idem plus gives for each
    /// item, the list of numbers of entities in the starting model &lt;what&gt; = 4,5 : modes relays on an enum
    /// PrintCount : 0 (D) : ItemsByEntity (sequential list by entity) 1 : CountByItem 2 : ShortByItem (count + 5
    /// first numbers) 3 : ListByItem (count + entity numbers) 4 : EntitiesByItem (count + entity numbers and
    /// labels)
    /// </summary>
    public static void PrintStatsProcess(Transfer_TransientProcess? TP, int what, int mode = 0)
    {
        NCollection_HSequence<Standard_Transient> list = null; // null
        XSControl_TransferReader.PrintStatsOnList(TP, list, what, mode);
    }

    /// <summary>
    /// Works as PrintStatsProcess, but displays data only on the entities which are in &lt;list&gt; (filter)
    /// </summary>
    public static void PrintStatsOnList(Transfer_TransientProcess? TP, NCollection_HSequence<Standard_Transient>? list, int what, int mode = 0)
    {
        // OCCT: one stream buffer of TP's messenger (made before TP is tested), sent when the function returns
        if (TP is null)
        {
            return;
        }
        StringWriter sout = new StringWriter();
        try
        {
            PrintStatsOnList(TP, list, what, mode, sout);
        }
        finally
        {
            if (sout.GetStringBuilder().Length > 0)
            {
                TP.Messenger().SendInfo().Append(sout.ToString()).Flush();
            }
        }
    }

    internal static void PrintStatsOnList(Transfer_TransientProcess TP, NCollection_HSequence<Standard_Transient>? list,
                                         int what, int mode, TextWriter sout)
    {
        string mess = "";
        if (what == 0)
        {
            TP.PrintStats(0, sout);
            return;
        }

        sout.Write("\n*******************************************************************\n");
        sout.Write("******        Statistics on Transfer Process (Read)          ******");
        sout.Write('\n');
        if (what == 1)
        {
            sout.Write("******        Individual Transfers  (Roots)                  ******\n");
        }
        if (what == 2)
        {
            sout.Write("******        All recorded data about Transfer               ******\n");
        }
        if (what == 3)
        {
            sout.Write("******        Abnormal records                               ******\n");
        }
        if (what == 1 || what == 2 || what == 3)
        {
            if (mode == 0)
            {
                sout.Write("******        (n0s of recorded entities)                     ******\n");
            }
            if (mode == 1)
            {
                sout.Write("******        (per entity : type + result)                   ******\n");
            }
            if (mode == 2)
            {
                sout.Write("******        (per entity : type + result/status)            ******\n");
            }
            if (mode == 3)
            {
                sout.Write("******        (count per type of entity)                     ******\n");
            }
            if (mode == 4)
            {
                sout.Write("******        (count per type of result)                     ******\n");
            }
            if (mode == 5)
            {
                sout.Write("******   (count per couple entity-type / result-type/status) ******\n");
            }
            if (mode == 6)
            {
                sout.Write("******   (list per couple entity-type / result-type/status)  ******\n");
            }
        }
        if (what == 4)
        {
            sout.Write("******        Check messages                                 ******\n");
        }
        if (what == 5)
        {
            sout.Write("******        Fail  messages                                 ******\n");
        }
        sout.Write("*******************************************************************\n");

        //  Case what = 1,2,3: content of TP (binders)

        bool nolist = list is null;
        Interface_InterfaceModel model = TP.Model();
        if (what >= 1 && what <= 3)
        {

            int stat;
            int nbw = 0, nbf = 0, nbr = 0, nbrw = 0, nbrf = 0, nbnr = 0, nbi = 0;
            Transfer_IteratorOfProcessForTransient itrp = new Transfer_IteratorOfProcessForTransient(true);
            if (what == 1)
            {
                itrp = TP.RootResult(true);
            }
            if (what == 2)
            {
                itrp = TP.CompleteResult(true);
            }
            if (what == 3)
            {
                itrp = TP.AbnormalResult();
            }
            int i = 0, nb = itrp.Number();
            if (!nolist)
            {
                itrp.Filter(list);
            }
            int nl = itrp.Number(); // after filtering
            IFSelect_SignatureList counter = null;
            if (mode > 2)
            {
                counter = new IFSelect_SignatureList(mode == 6);
            }
            bool                notrec = (!nolist && mode > 2); // note the "no record"
            IFSelect_PrintCount pcm = IFSelect_CountByItem;
            if (mode == 6)
            {
                pcm = IFSelect_ListByItem;
            }

            sout.Write("****        Entities in Model   : ");
            sout.Write(model.NbEntities());
            sout.Write('\n');
            sout.Write("****        Nb Items (Transfer) : ");
            sout.Write(nb);
            sout.Write('\n');
            if (!nolist)
            {
                sout.Write("****        Nb Items (Listed)   : ");
                sout.Write(nl);
                sout.Write('\n');
            }

            for (itrp.Start(); itrp.More(); itrp.Next())
            {
                nbi++;
                Transfer_Binder binder = itrp.Value();
                Standard_Transient ent = itrp.Starting();
                if (binder is null)
                {
                    nbnr++;
                    if (notrec)
                    {
                        counter.Add(ent, "(not recorded)");
                    }
                    else if (mode == 1 || mode == 2)
                    {
                        sout.Write("[");
                        sout.Write(Interface_MSG.Blanks(nbi, 4));
                        sout.Write(nbi);
                        sout.Write(" ]:");
                        model.Print(ent, sout);
                        sout.Write("   ");
                        sout.Write(model.TypeName(ent, false));
                        sout.Write("  (not recorded)");
                        sout.Write('\n');
                        continue;
                    }
                }
                if (mode == 0)
                {
                    sout.Write("  ");
                    sout.Write(model.Number(ent));
                    continue;
                }
                if (mode != 3)
                {
                    stat = BinderStatus(binder, out mess);
                    // 0 Binder Null.   1 void  2 Warning only  3 Fail only
                    // 11 Result OK. 12 Result+Warning. 13 Result+Fail
                    if (stat == 2)
                    {
                        nbw++;
                    }
                    if (stat == 3)
                    {
                        nbf++;
                    }
                    if (stat == 11)
                    {
                        nbr++;
                    }
                    if (stat == 12)
                    {
                        nbrw++;
                    }
                    if (stat == 13)
                    {
                        nbrf++;
                    }
                }

                //  mode : 0 list num;  1 : num+label + type + result (abrege);  2 : complet
                if (mode == 1 || mode == 2)
                {
                    sout.Write("[");
                    sout.Write(Interface_MSG.Blanks(i, 4));
                    sout.Write(i);
                    sout.Write(" ]:");
                    model.Print(ent, sout);
                    sout.Write("   ");
                    sout.Write(model.TypeName(ent, false));
                    sout.Write("\tResult:");
                    sout.Write(mess);
                    sout.Write('\n');
                    if (mode == 1)
                    {
                        continue;
                    }

                    Interface_Check ch = binder.Check();
                    int newi, newnbw = ch.NbWarnings(), newnbf = ch.NbFails();

                    if (newnbw > 0)
                    {
                        sout.Write(" - Warnings : ");
                        sout.Write(newnbw);
                        sout.Write(":\n");
                        for (newi = 1; newi <= newnbw; newi++)
                        {
                            sout.Write(ch.CWarning(newi));
                            sout.Write('\n');
                        }
                    }
                    if (newnbf > 0)
                    {
                        sout.Write(" - Fails : ");
                        sout.Write(newnbf);
                        sout.Write(":\n");
                        for (newi = 1; newi <= newnbf; newi++)
                        {
                            sout.Write(ch.CFail(newi));
                            sout.Write('\n');
                        }
                    }
                    continue;
                }

                //  mode : 3, counts per type of starting entity (class type)
                //         4 : counts per result type and/or status
                //         5 : counts per couple (starting type / result type/status)
                //         6 : idem plus gives for each item, the list of numbers of
                //                  entities in the starting model
                if (mode >= 3 && mode <= 6)
                {
                    // IFSelect_PrintCount newpcm = IFSelect_CountByItem;
                    // if (mode == 6) newpcm = IFSelect_ListByItem;
                    if (mode == 3)
                    {
                        counter.Add(ent, model.TypeName(ent, false));
                    }
                    if (mode == 4)
                    {
                        counter.Add(ent, mess);
                    }
                    if (mode >= 5)
                    {
                        string mest = model.TypeName(ent, false);
                        mest += "\t-> ";
                        mest += mess;
                        // Sprintf(mest,"%s\t-> %s",model.TypeName(ent,false),mess);
                        counter.Add(ent, mest);
                    }
                }

                //    End of iteration
            }
            if (counter is not null)
            {
                counter.PrintList(sout, model, pcm);
            }
            else
            {
                sout.Write('\n');
            }
            //    Pourcentages
            if (mode != 3 && nbi > 0)
            {
                sout.Write("******        Percentages according Transfer Status          ******");
                sout.Write('\n');
                PrintPercent(TP.Messenger(), "Result          ", nbr + nbrw, nl);
                PrintPercent(TP.Messenger(), "Result + FAIL   ", nbrf, nl);
                PrintPercent(TP.Messenger(), "FAIL, no Result ", nbf, nl);
                PrintPercent(TP.Messenger(), "Just Warning    ", nbw, nl);
                PrintPercent(TP.Messenger(), "Nothing Recorded", nbnr, nl);
                // (OCCT's former percentages, commented out, left out)
            }
            return;
        }

        //  Case what = 4,5: check-list

        if (what == 4 || what == 5)
        {

            Interface_CheckIterator chl = TP.CheckList(false);
            chl.SetName("** TRANSFER READ CHECK **");
            if (mode == 0)
            {
                chl.Print(sout, model, (what == 5));
            }
            else
            {
                IFSelect_PrintCount pcm = IFSelect_CountByItem;
                if (mode == 2)
                {
                    pcm = IFSelect_ListByItem;
                }
                IFSelect_CheckCounter counter = new IFSelect_CheckCounter(true);
                counter.Analyse(chl, model, true, (what == 5));
                counter.PrintList(sout, model, pcm);
            }
        }
    }

    private XSControl_Controller? myController;
    private string myFileName = "";
    private Interface_InterfaceModel? myModel;
    private Interface_HGraph? myGraph;
    private NCollection_DataMap<string, Standard_Transient> myContext = new NCollection_DataMap<string, Standard_Transient>();
    private Transfer_ActorOfTransientProcess? myActor;
    private Transfer_TransientProcess? myTP;
    private NCollection_DataMap<int, Standard_Transient> myResults = new NCollection_DataMap<int, Standard_Transient>();
    private Interface_StaticSet? myStatics;
    private NCollection_HSequence<TopoDS_Shape>? myShapeResult;
}
