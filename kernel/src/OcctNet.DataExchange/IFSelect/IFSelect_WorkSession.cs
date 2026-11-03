// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_WorkSession.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_WorkSession.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using static Occt.IFSelect_ReturnStatus;
using static Occt.IFSelect_PrintCount;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// This class can be used to simply manage a process such as splitting a file, extracting a set of Entities
/// ... It allows to manage different types of Variables : Integer or Text Parameters, Selections,
/// Dispatches, in addition to a ShareOut. To each of these variables, a unique Integer Identifier is
/// attached. A Name can be attached too as desired.
/// </summary>
public class IFSelect_WorkSession : Transient
{
    private const int Flag_Incorrect = 2;
    //  (Bit Map n0 2)

    // OCCT's file statics "errhand" (process-wide: "only one at a time, but it goes so fast") and "bufstr" (a
    // buffer for the returned texts): the first is the session's own (myErrHand), the second a local string.

    //  ###########################################################################
    //  ....         Analysis of a CheckIterator in relation to a graph        ....

    private static void IFSelect_QueryProp(Interface_IntList list, char[] ana, int num, int quoi)
    {
        list.SetNumber(num);
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            if (i > 1)
            {
                list.SetNumber(num); // because recursive call + depth first
            }
            int n = list.Value(i);
            //    is there a need to propagate?
            //  1 W/place  2 F/place  3 Wprop 4Wprop+W/place  5Wprop+F/place
            //  6 Fprop  7 Fprop+W/place  8 Fprop+F/place
            char val = ana[n - 1];
            switch (val)
            {
                case ' ':
                    val = (quoi != 0 ? '3' : '6');
                    break;
                case '1':
                    val = (quoi != 0 ? '4' : '7');
                    break;
                case '2':
                    val = (quoi != 0 ? '5' : '8');
                    break;
                case '3':
                    val = (quoi != 0 ? ' ' : '6');
                    break;
                case '4':
                    val = (quoi != 0 ? ' ' : '7');
                    break;
                case '5':
                    val = (quoi != 0 ? ' ' : '8');
                    break;
                case '6':
                    val = ' ';
                    break;
                case '7':
                    val = ' ';
                    break;
                case '8':
                    val = ' ';
                    break;
                default:
                    val = ' ';
                    break;
            }
            if (val == ' ')
            {
                continue;
            }
            ana[n - 1] = val;
            IFSelect_QueryProp(list, ana, n, quoi);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Creates a Work Session It provides default, empty ShareOut and ModelCopier, which can be replaced (if
    /// required, should be done just after creation).
    /// </summary>
    public IFSelect_WorkSession()
    {
        theshareout = new IFSelect_ShareOut();
        theerrhand = myErrHand = true;
        thecopier = new IFSelect_ModelCopier();
        thecopier.SetShareOut(theshareout);
        thecheckdone = false;
        thegtool = new Interface_GTool();
        themodelstat = false;
        thestatics = new Interface_StaticSet();
        thestatics.Standards();
    }

    /// <summary>
    /// The parameter set of the session (OCCT.NET: OCCT's process-wide statics, decision 1181). A new session has
    /// its own set with the standard statics; the controllers declare theirs in it (XSControl_WorkSession).
    /// </summary>
    public Interface_StaticSet Statics()
    {
        return thestatics;
    }

    /// <summary>
    /// Gives the session a parameter set (for instance an interpreter's, or a reader's copy), in place of its own
    /// (OCCT.NET). The standard statics are declared in it if they are not.
    /// </summary>
    public virtual void SetStatics(Interface_StaticSet theStatics)
    {
        thestatics = theStatics ?? throw new ArgumentNullException(nameof(theStatics));
        thestatics.Standards();
    }

    /// <summary>
    /// Changes the Error Handler status (by default, it is not set)
    /// </summary>
    public void SetErrorHandle(bool toHandle)
    {
        theerrhand = myErrHand = toHandle;
    }

    /// <summary>
    /// Returns the Error Handler status
    /// </summary>
    public bool ErrorHandle()
    {
        return theerrhand;
    }

    /// <summary>
    /// Returns the ShareOut defined at creation time
    /// </summary>
    public IFSelect_ShareOut? ShareOut()
    {
        return theshareout;
    }

    /// <summary>
    /// Sets a new ShareOut. Fills Items which its content Warning : data from the former ShareOut are lost
    /// </summary>
    public void SetShareOut(IFSelect_ShareOut? shareout)
    {
        theshareout = shareout;
        thecopier.SetShareOut(theshareout);
        // ... should add Params, Dispatches, etc...
    }

    /// <summary>
    /// Set value of mode responsible for presence of selections after loading If mode set to true that different
    /// selections will be accessible after loading else selections will be not accessible after loading( for
    /// economy memory in applications)
    /// </summary>
    public void SetModeStat(bool theMode)
    {
        themodelstat = theMode;
    }

    /// <summary>
    /// Return value of mode defining of filling selection during loading
    /// </summary>
    public bool GetModeStat()
    {
        return themodelstat;
    }

    /// <summary>
    /// Sets a WorkLibrary, which will be used to Read and Write Files
    /// </summary>
    public void SetLibrary(IFSelect_WorkLibrary? theLib)
    {
        thelibrary = theLib;
    }

    /// <summary>
    /// Returns the WorkLibrary. Null Handle if not yet set should be C++ : return const &amp;
    /// </summary>
    public IFSelect_WorkLibrary? WorkLibrary()
    {
        return thelibrary;
    }

    /// <summary>
    /// Sets a Protocol, which will be used to determine Graphs, to Read and to Write Files
    /// </summary>
    public void SetProtocol(Interface_Protocol? protocol)
    {
        theprotocol = protocol;
        // OCCT also makes it the active protocol (Interface_Protocol::SetActive): left out (decision 1183)
        thegtool.SetProtocol(protocol);
    }

    /// <summary>
    /// Returns the Protocol. Null Handle if not yet set should be C++ : return const &amp;
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return theprotocol;
    }

    /// <summary>
    /// Sets a specific Signature to be the SignType, i.e. the Signature which will determine TypeName from the
    /// Model (basic function). It is recorded in the GTool This Signature is also set as "xst-sign-type"
    /// (reserved name)
    /// </summary>
    public void SetSignType(IFSelect_Signature? signtype)
    {
        thegtool.SetSignType(signtype);
        if (signtype is null)
        {
            thenames.UnBind("xst-sign-type");
        }
        else
        {
            thenames.Bind("xst-sign-type", signtype);
        }
    }

    /// <summary>
    /// Returns the current SignType
    /// </summary>
    public IFSelect_Signature? SignType()
    {
        return (thegtool.SignType() as IFSelect_Signature);
    }

    /// <summary>
    /// Returns True is a Model has been set
    /// </summary>
    public bool HasModel()
    {
        return (myModel is not null);
    }

    /// <summary>
    /// Sets a Model as input : this will be the Model from which the ShareOut will work if &lt;clearpointed&gt;
    /// is True (default) all SelectPointed items are cleared, else they must be managed by the caller Remark :
    /// SetModel clears the Graph, recomputes it if a Protocol is set and if the Model is not empty, of course
    /// </summary>
    public void SetModel(Interface_InterfaceModel? model, bool clearpointed = true)
    {
        if (myModel != model)
        {
            theloaded = "";
        }
        myModel = model;
        if (thegtool is not null)
        {
            thegtool.ClearEntities(); // smh#14 FRA62479
        }
        myModel.SetGTool(thegtool);

        thegraph = null;
        ComputeGraph(); // does something if Protocol present. Otherwise, does nothing
        ClearData(3); // Reset CheckList, to be redone
        thecheckrun.Clear();

        //  UPDATE of SelectPointed  I.E.  clear their content
        if (clearpointed)
        {
            ClearData(4);
        }
        ClearData(0);
    }

    /// <summary>
    /// Returns the Model of the Work Session (Null Handle if none) should be C++ : return const &amp;
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return myModel;
    }

    /// <summary>
    /// Stores the filename used for read for setting the model It is cleared by SetModel and ClearData(1)
    /// </summary>
    public void SetLoadedFile(string theFileName)
    {
        theloaded = theFileName ?? "";
    }

    /// <summary>
    /// Returns the filename used to load current model empty if unknown
    /// </summary>
    public string LoadedFile()
    {
        return theloaded;
    }

    /// <summary>
    /// Reads a file with the WorkLibrary (sets Model and LoadedFile) Returns a integer status which can be :
    /// RetDone if OK, RetVoid if no Protocol not defined, RetError for file not found, RetFail if fail during
    /// read
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="theProgress"/> (not in OCCT) is given to the library (plan 2.7).</remarks>
    public IFSelect_ReturnStatus ReadFile(string filename, ProgressRange theProgress = default)
    {
        if (thelibrary is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        if (theprotocol is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        Interface_InterfaceModel model = null;
        IFSelect_ReturnStatus status = IFSelect_ReturnStatus.IFSelect_RetVoid;
        try
        {
            int stat = thelibrary.ReadFile(filename, ref model, theprotocol, theProgress);
            if (stat == 0)
            {
                status = IFSelect_ReturnStatus.IFSelect_RetDone;
            }
            else if (stat < 0)
            {
                status = IFSelect_ReturnStatus.IFSelect_RetError;
            }
            else
            {
                status = IFSelect_ReturnStatus.IFSelect_RetFail;
            }
        }
        catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
        {
            using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
            sout.Append("    ****    ReadFile Interruption by Exception :   ****\n").Append(Transfer_ProcessForTransient.What(anException)).Append("\n    Abandon").Append('\n');
            status = IFSelect_ReturnStatus.IFSelect_RetFail;
        }
        if (status != IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            return status;
        }
        if (model is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        SetModel(model);
        SetLoadedFile(filename);
        return status;
    }

    /// <summary>
    /// Reads a file from stream with the WorkLibrary (sets Model and LoadedFile) Returns a integer status which
    /// can be : RetDone if OK, RetVoid if no Protocol not defined, RetError for file not found, RetFail if fail
    /// during read
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the stream is a .NET <see cref="Stream"/>; <paramref name="theProgress"/> (not in OCCT) is given to
    /// the library (plan 2.7).
    /// </remarks>
    public IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream, ProgressRange theProgress = default)
    {
        if (thelibrary is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        if (theprotocol is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        Interface_InterfaceModel model = null;
        IFSelect_ReturnStatus status = IFSelect_ReturnStatus.IFSelect_RetVoid;
        try
        {
            int stat = thelibrary.ReadStream(theName, theIStream, ref model, theprotocol, theProgress);
            if (stat == 0)
            {
                status = IFSelect_ReturnStatus.IFSelect_RetDone;
            }
            else if (stat < 0)
            {
                status = IFSelect_ReturnStatus.IFSelect_RetError;
            }
            else
            {
                status = IFSelect_ReturnStatus.IFSelect_RetFail;
            }
        }
        catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
        {
            using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
            sout.Append("    ****    ReadFile Interruption by Exception :   ****\n").Append(Transfer_ProcessForTransient.What(anException)).Append("\n    Abandon").Append('\n');
            status = IFSelect_ReturnStatus.IFSelect_RetFail;
        }
        if (status != IFSelect_ReturnStatus.IFSelect_RetDone)
        {
            return status;
        }
        if (model is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        SetModel(model);
        SetLoadedFile(theName);
        return status;
    }

    /// <summary>
    /// Returns the count of Entities stored in the Model, or 0
    /// </summary>
    public int NbStartingEntities()
    {
        return (myModel is null ? 0 : myModel.NbEntities());
    }

    /// <summary>
    /// Returns an Entity stored in the Model of the WorkSession (Null Handle is no Model or num out of range)
    /// </summary>
    public Transient? StartingEntity(int num)
    {
        Transient res = null; // Null by default
        if (myModel is null)
        {
            return res;
        }
        if (num < 1 || num > myModel.NbEntities())
        {
            return res;
        }
        return myModel.Value(num);
    }

    /// <summary>
    /// Returns the Number of an Entity in the Model (0 if no Model set or &lt;ent&gt; not in the Model)
    /// </summary>
    public int StartingNumber(Transient? ent)
    {
        return (myModel is null ? 0 : myModel.Number(ent));
    }

    /// <summary>
    /// From a given label in Model, returns the corresponding number Starts from first entity by Default, may
    /// start after a given number : this number may be given negative, its absolute value is then considered.
    /// Hence a loop on NumberFromLabel may be programmed (stop test is : returned value positive or null)
    /// Returns 0 if not found, &lt; 0 if more than one found (first found in negative). If &lt;val&gt; just
    /// gives an integer value, returns it
    /// </summary>
    public int NumberFromLabel(string val, int afternum = 0)
    {
        int i, cnt = 0, num = Atoi(val);
        if (num > 0 || myModel is null)
        {
            return num; // a direct n0 : won !
        }
        //  Otherwise, consider it as a label; to be treated in CaseNonSensitive ...
        if (num > myModel.NbEntities())
        {
            num = 0;
            return num;
        }
        bool exact = false;
        int after = (afternum >= 0 ? afternum : -afternum);
        for (i = myModel.NextNumberForLabel(val, after, exact); i != 0;
                  i = myModel.NextNumberForLabel(val, i, exact))
        {
            cnt++;
            if (num <= 0)
            {
                num = i;
            }
        }
        if (cnt == 1)
        {
            return num;
        }
        num = -num;

        return num;
    }

    /// <summary>
    /// Returns the label for &lt;ent&gt;, as the Model does If &lt;ent&gt; is not in the Model or if no Model is
    /// loaded, a Null Handle is returned
    /// </summary>
    public TCollection_HAsciiString? EntityLabel(Transient? ent)
    {
        if (myModel is null || ent is null)
        {
            return null;
        }
        if (!myModel.Contains(ent))
        {
            return null;
        }
        return myModel.StringLabel(ent);
    }

    /// <summary>
    /// Returns the Name of an Entity This Name is computed by the general service Name Returns a Null Handle if
    /// fails
    /// </summary>
    public TCollection_HAsciiString? EntityName(Transient? ent)
    {
        if (myModel is null || ent is null)
        {
            return null;
        }
        Interface_ShareTool sht = new Interface_ShareTool(thegraph.Graph());

        if (!thegtool.Select(ent, out Interface_GeneralModule module, out int CN))
        {
            return null;
        }
        return module.Name(CN, ent, sht);
    }

    /// <summary>
    /// Returns the Category Number determined for an entity it is computed by the class Category An unknown
    /// entity (number 0) gives a value -1
    /// </summary>
    public int CategoryNumber(Transient? ent)
    {
        return (myModel is null ? -1 : myModel.CategoryNumber(StartingNumber(ent)));
    }

    /// <summary>
    /// Returns the Category Name determined for an entity it is computed by the class Category Remark : an
    /// unknown entity gives an empty string
    /// </summary>
    public string CategoryName(Transient? ent)
    {
        return Interface_Category.Name(CategoryNumber(ent));
    }

    /// <summary>
    /// Returns the Validity Name determined for an entity it is computed by the class SignValidity Remark : an
    /// unknown entity gives an empty string
    /// </summary>
    public string ValidityName(Transient? ent)
    {
        return ((StartingNumber(ent) == 0) ? "" : IFSelect_SignValidity.CVal(ent, myModel));
    }

    /// <summary>
    /// Clears recorded data (not the items) according mode : 1 : all Data : Model, Graph, CheckList, + ClearData
    /// 4 2 : Graph and CheckList (they will then be recomputed later) 3 : CheckList (it will be recomputed by
    /// ComputeCheck) 4 : just content of SelectPointed and Counters Plus 0 : does nothing but called by SetModel
    /// ClearData is virtual, hence it can be redefined to clear other data of a specialised Work Session
    /// </summary>
    public virtual void ClearData(int mode)
    {
        switch (mode)
        {
            case 1: {
                theloaded = "";
                if (myModel is not null)
                {
                    myModel.Clear();
                    myModel = null;
                }
                ClearData(2);
                ClearData(4);
                thecheckrun.Clear();
                break;
            }
            case 2: {
                thegraph = null;
                thecheckdone = false;
                thecheckana = "";
                break;
            }
            case 3: {
                thecheckdone = false;
                break;
            }
            case 4: {
                //  UPDATE of SelectPointed  I.E.  clear their content
                //  AS WELL AS editors (in fact, the EditForm)
                //  Counters  I.E.  we erase their content (to reevaluate)
                NCollection_HSequence<int> list =
                    ItemIdents(typeof(IFSelect_SelectPointed));
                int nb = list.Length();
                int i; // svv #1
                for (i = 1; i <= nb; i++)
                {
                    IFSelect_SelectPointed sp = Item(list.Value(i)) as IFSelect_SelectPointed;
                    if (sp is not null)
                    {
                        sp.Clear();
                    }
                }
                list = ItemIdents(typeof(IFSelect_SignatureList));
                nb = list.Length();
                for (i = 1; i <= nb; i++)
                {
                    IFSelect_SignatureList sl = Item(list.Value(i)) as IFSelect_SignatureList;
                    if (sl is not null)
                    {
                        sl.Clear();
                    }
                    IFSelect_SignCounter sc = sl as IFSelect_SignCounter;
                    if (sc is not null)
                    {
                        sc.SetSelMode(-1);
                    }
                }
                list = ItemIdents(typeof(IFSelect_EditForm));
                nb = list.Length();
                for (i = 1; i <= nb; i++)
                {
                    IFSelect_EditForm edf = Item(list.Value(i)) as IFSelect_EditForm;
                    edf.ClearData();
                }
                theitems.Clear();
                break;
            }
            default:
                break;
        }
    }

    /// <summary>
    /// Computes the Graph used for Selections, Displays ... If a HGraph is already set, with same model as given
    /// by method Model, does nothing. Else, computes a new Graph. If &lt;enforce&gt; is given True, computes a
    /// new Graph anyway. Remark that a call to ClearGraph will cause ComputeGraph to really compute a new Graph
    /// Returns True if Graph is OK, False else (i.e. if no Protocol is set, or if Model is absent or empty).
    /// </summary>
    public bool ComputeGraph(bool enforce = false)
    {
        if (theprotocol is null)
        {
            return false;
        }
        if (myModel is null)
        {
            return false;
        }
        if (enforce)
        {
            thegraph = null;
        }
        if (thegraph is not null)
        {
            if (myModel.NbEntities() == thegraph.Graph().Size())
            {
                return true;
            }
            thegraph = null;
        }
        if (myModel.NbEntities() == 0)
        {
            return false;
        }
        //  We must calculate the graph for good
        thegraph = new Interface_HGraph(myModel, themodelstat);
        int nb = myModel.NbEntities();
        if (themodelstat)
        {
            int i; // svv #1
            for (i = 1; i <= nb; i++)
            {
                thegraph.CGraph().SetStatus(i, 0);
            }
            Interface_BitMap bm = thegraph.CGraph().CBitMap();
            bm.AddFlag();
            bm.SetFlagName(Flag_Incorrect, "Incorrect");
        }
        ComputeCheck();
        thecheckdone = true;
        if (themodelstat)
        {
            //  Calculation of categories, now memorized in the model
            Interface_Category categ = new Interface_Category(thegtool);
            Interface_ShareTool sht = new Interface_ShareTool(thegraph);
            int i = 1;
            for (; i <= nb; i++)
            {
                myModel.SetCategoryNumber(i, categ.CatNum(myModel.Value(i), sht));
            }
        }

        return true;
    }

    /// <summary>
    /// Returns the Computed Graph as HGraph (Null Handle if not set)
    /// </summary>
    public Interface_HGraph? HGraph()
    {
        ComputeGraph();
        return thegraph;
    }

    /// <summary>
    /// Returns the Computed Graph, for Read only
    /// </summary>
    public Interface_Graph Graph()
    {
        ComputeGraph();
        if (thegraph is null)
        {
            throw new DomainErrorException("IFSelect WorkSession : Graph not available");
        }
        return thegraph.Graph();
    }

    /// <summary>
    /// Returns the list of entities shared by &lt;ent&gt; (can be empty) Returns a null Handle if &lt;ent&gt; is
    /// unknown
    /// </summary>
    public NCollection_HSequence<Transient>? Shareds(Transient? ent)
    {
        NCollection_HSequence<Transient> list = null;
        if (!ComputeGraph())
        {
            return list;
        }
        if (StartingNumber(ent) == 0)
        {
            return list;
        }
        return thegraph.Graph().Shareds(ent).Content();
    }

    /// <summary>
    /// Returns the list of entities sharing &lt;ent&gt; (can be empty) Returns a null Handle if &lt;ent&gt; is
    /// unknown
    /// </summary>
    public NCollection_HSequence<Transient>? Sharings(Transient? ent)
    {
        NCollection_HSequence<Transient> list = null;
        if (!ComputeGraph())
        {
            return list;
        }
        if (StartingNumber(ent) == 0)
        {
            return list;
        }
        return thegraph.Graph().Sharings(ent).Content();
    }

    /// <summary>
    /// Returns True if a Model is defined and really loaded (not empty), a Protocol is set and a Graph has been
    /// computed. In this case, the WorkSession can start to work
    /// </summary>
    public bool IsLoaded()
    {
        if (theprotocol is null)
        {
            return false;
        }
        if (myModel is null)
        {
            return false;
        }
        if (myModel.NbEntities() == 0)
        {
            return false;
        }
        if (thegraph is null)
        {
            return false;
        }
        if (myModel.NbEntities() == thegraph.Graph().Size())
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Computes the CheckList for the Model currently loaded It can then be used for displays, queries ...
    /// Returns True if OK, False else (i.e. no Protocol set, or Model absent). If &lt;enforce&gt; is False,
    /// works only if not already done or if a new Model has been loaded from last call. Remark : computation is
    /// enforced by every call to SetModel or RunTransformer
    /// </summary>
    public bool ComputeCheck(bool enforce = false)
    {
        if (enforce)
        {
            thecheckdone = false;
        }
        if (thecheckdone)
        {
            return true;
        }
        if (!IsLoaded())
        {
            return false;
        }

        Interface_Graph CG = thegraph.CGraph();
        Interface_CheckTool cht = new Interface_CheckTool(thegraph);
        Interface_CheckIterator checklist = cht.VerifyCheckList();
        myModel.FillSemanticChecks(checklist, false);
        if (themodelstat)
        {
            //  And we update the Graph (BitMap) !  Flag Incorrect (STX + SEM)
            Interface_BitMap BM = CG.CBitMap();
            BM.Init(false, Flag_Incorrect);
            int num, nb = CG.Size();
            for (checklist.Start(); checklist.More(); checklist.Next())
            {
                Interface_Check chk = checklist.Value();
                if (!chk.HasFailed())
                {
                    continue;
                }
                num = checklist.Number();
                if (num > 0 && num <= nb)
                {
                    BM.SetTrue(num, Flag_Incorrect);
                }
            }
            for (num = 1; num <= nb; num++)
            {
                if (myModel.IsErrorEntity(num))
                {
                    BM.SetTrue(num, Flag_Incorrect);
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Returns the Check List for the Model currently loaded : &lt;complete&gt; = True : complete (syntactic
    /// &amp; semantic messages), computed if not yet done &lt;complete&gt; = False : only syntactic (check file
    /// form)
    /// </summary>
    public Interface_CheckIterator ModelCheckList(bool complete = true)
    {
        Interface_CheckIterator checks = new Interface_CheckIterator();
        if (!IsLoaded())
        {
            checks.CCheck(0).AddFail("DATA NOT AVAILABLE FOR CHECK");
            return checks;
        }
        Interface_CheckTool cht = new Interface_CheckTool(Graph());
        checks = (complete ? cht.CompleteCheckList() : cht.AnalyseCheckList());
        checks.SetName(complete ? "Model Complete Check List" : "Model Syntactic Check List");
        return checks;
    }

    /// <summary>
    /// Returns a Check for a single entity, under the form of a CheckIterator (this gives only one form for the
    /// user) if &lt;ent&gt; is Null or equates the current Model, it gives the Global Check, else the Check for
    /// the given entity &lt;complete&gt; as for ModelCheckList
    /// </summary>
    public Interface_CheckIterator CheckOne(Transient? ent, bool complete = true)
    {
        Interface_CheckIterator checks = new Interface_CheckIterator();
        checks.SetModel(myModel);
        if (!IsLoaded())
        {
            checks.CCheck(0).AddFail("DATA NOT AVAILABLE FOR CHECK");
            return checks;
        }
        int num = -1;
        if (ent is null || ent == myModel)
        {
            num = 0;
        }
        else
        {
            num = myModel.Number(ent);
        }

        Interface_Check ach = myModel.Check(num, true);
        if (complete)
        {
            ach.GetMessages(myModel.Check(num, false));
        }
        if (num > 0)
        {
            ach.SetEntity(ent);
        }
        checks.Add(ach, num);
        checks.SetName("Data Check (One Entity)");
        return checks;
    }

    /// <summary>
    /// Returns the Check List produced by the last execution of either : EvaluateFile(for Split), SendSplit,
    /// SendAll, SendSelected, RunTransformer-RunModifier Cleared by SetModel or ClearData(1) The field is
    /// protected, hence a specialized WorkSession may fill it
    /// </summary>
    public Interface_CheckIterator LastRunCheckList()
    {
        return thecheckrun;
    }

    /// <summary>
    /// Returns the Maximum Value for an Item Identifier. It can be greater to the count of known Items, because
    /// some can have been removed
    /// </summary>
    public int MaxIdent()
    {
        return theitems.Extent();
    }

    /// <summary>
    /// Returns an Item, given its Ident. Returns a Null Handle if no Item corresponds to this Ident.
    /// </summary>
    public Transient? Item(int id)
    {
        Transient res = null;
        if (id <= 0 || id > MaxIdent())
        {
            return res;
        }
        if (theitems.FindFromIndex(id) is null)
        {
            return res;
        }
        return theitems.FindKey(id);
    }

    /// <summary>
    /// Returns the Ident attached to an Item in the WorkSession, or Zero if it is unknown
    /// </summary>
    public int ItemIdent(Transient? item)
    {
        if (item is null)
        {
            return 0;
        }
        int id = theitems.FindIndex(item);
        if (id == 0)
        {
            return 0;
        }
        if (theitems.FindFromIndex(id) is null)
        {
            return 0;
        }
        return id;
    }

    /// <summary>
    /// Returns the Item which corresponds to a Variable, given its Name (whatever the type of this Item).
    /// Returns a Null Handle if this Name is not recorded
    /// </summary>
    public Transient? NamedItem(string name)
    {
        Transient res = null;
        if (CStringChar(name, 0) == '\0')
        {
            return res;
        }
        if (name[0] == '#')
        { // #nnn : not a name but an id number
            int id = Atoi(name.Substring(1));
            return Item(id);
        }
        if (!thenames.Find(name, out res))
        {
            res = null;
        }
        return res;
    }

    /// <summary>
    /// Same as above, but &lt;name&gt; is given through a Handle Especially useful with methods SelectionNames,
    /// etc...
    /// </summary>
    public Transient? NamedItem(TCollection_HAsciiString? name)
    {
        Transient res = null;
        if (name is not null)
        {
            res = NamedItem(name.ToCString());
        }
        return res;
    }

    /// <summary>
    /// Returns the Ident attached to a Name, 0 if name not recorded
    /// </summary>
    public int NameIdent(string name)
    {
        if (CStringChar(name, 0) == '\0')
        {
            return 0;
        }
        if (name[0] == '#')
        { // #nnn : not a name but an id number
            int id = Atoi(name.Substring(1));
            return id;
        }
        if (!thenames.Find(name, out Transient res))
        {
            return 0;
        }
        return ItemIdent(res);
    }

    /// <summary>
    /// Returns True if an Item of the WorkSession has an attached Name
    /// </summary>
    public bool HasName(Transient? item)
    {
        if (item is null)
        {
            return false;
        }
        int id = theitems.FindIndex(item);
        if (id == 0)
        {
            return false;
        }
        Transient att = theitems.FindFromIndex(id);
        if (att is null)
        {
            return false;
        }
        return att.IsKind(typeof(TCollection_HAsciiString));
    }

    /// <summary>
    /// Returns the Name attached to an Item as a Variable of this WorkSession. If &lt;item&gt; is Null or not
    /// recorded, returns an empty string.
    /// </summary>
    public TCollection_HAsciiString? Name(Transient? item)
    {
        TCollection_HAsciiString res = null;
        if (item is null)
        {
            return res;
        }
        int id = theitems.FindIndex(item);
        if (id == 0)
        {
            return res; // Null
        }
        Transient att = theitems.FindFromIndex(id);
        return (att as TCollection_HAsciiString);
    }

    /// <summary>
    /// Adds an Item and returns its attached Ident. Does nothing if &lt;item&gt; is already recorded (and
    /// returns its attached Ident) &lt;active&gt; if True commands call to SetActive (see below) Remark : the
    /// determined Ident is used if &lt;item&gt; is a Dispatch, to fill the ShareOut
    /// </summary>
    public int AddItem(Transient? item, bool active = true)
    {
        if (item is null)
        {
            return 0;
        }
        int id = theitems.FindIndex(item);
        if (id > 0)
        {
            ref Transient att = ref theitems.ChangeFromIndex(id);
            if (att is null)
            {
                att = item;
            }
        }
        else
        {
            id = theitems.Add(item, item);
        }

        //  Special cases : Dispatch,Modifier
        if (active)
        {
            SetActive(item, true);
        }
        return id;
    }

    /// <summary>
    /// Adds an Item with an attached Name. If the Name is already known in the WorkSession, the older item losts
    /// it Returns Ident if Done, 0 else, i.e. if &lt;item&gt; is null If &lt;name&gt; is empty, works as AddItem
    /// (i.e. with no name) If &lt;item&gt; is already known but with no attached Name, this method tries to
    /// attached a Name to it &lt;active&gt; if True commands call to SetActive (see below)
    /// </summary>
    public int AddNamedItem(string name, Transient? item, bool active = true)
    {
        if (item is null)
        {
            return 0;
        }
        if (CStringChar(name, 0) == '#' || CStringChar(name, 0) == '!')
        {
            return 0;
        }
        // #nnn : not a name but a number. !... : reserved (forbidden for a name)
        //   name already taken : we overwrite the old value
        if (CStringChar(name, 0) != '\0')
        {
            thenames.Bind(name, item);
        }

        int id = theitems.FindIndex(item);
        if (id > 0)
        {
            ref Transient att = ref theitems.ChangeFromIndex(id);
            if (att is null)
            {
                att = item;
            }
            if (CStringChar(name, 0) != '\0')
            {
                //      if (!att.IsKind(typeof(TCollection_HAsciiString))) overwriting allowed !
                att = new TCollection_HAsciiString(name);
            }
        }
        else if (CStringChar(name, 0) != '\0')
        {
            id = theitems.Add(item, new TCollection_HAsciiString(name));
        }
        else
        {
            id = theitems.Add(item, item);
        }

        //  Special cases : Dispatch,Modifier
        if (active)
        {
            SetActive(item, true);
        }
        return id;
    }

    /// <summary>
    /// Following the type of &lt;item&gt; : - Dispatch : Adds or Removes it in the ShareOut &amp; FileNaming -
    /// GeneralModifier : Adds or Removes it for final sending (i.e. in the ModelCopier) Returns True if it did
    /// something, False else (state unchanged)
    /// </summary>
    public bool SetActive(Transient? item, bool mode)
    {
        if (item.IsKind(typeof(IFSelect_Dispatch)))
        {
            IFSelect_Dispatch disp = item as IFSelect_Dispatch;
            int num = theshareout.DispatchRank(disp);
            if (num > theshareout.NbDispatches())
            {
                return false;
            }
            if (mode)
            {
                if (num > 0)
                {
                    return false;
                }
                theshareout.AddDispatch(disp);
                return true;
            }
            else
            {
                if (num <= theshareout.LastRun())
                {
                    return false;
                }
                theshareout.RemoveDispatch(num);
                SetFileRoot(disp, ""); // if onlynamed : also cleans ShareOut
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Removes an Item from the Session, given its Name Returns True if Done, False else (Name not recorded)
    /// (Applies only on Item which are Named)
    /// </summary>
    public bool RemoveNamedItem(string name)
    {
        Transient item = NamedItem(name);
        if (item is null)
        {
            return false;
        }
        if (!RemoveItem(item))
        {
            return false; // which takes care of everything
        }
        return true;
    }

    /// <summary>
    /// Removes a Name without removing the Item Returns True if Done, False else (Name not recorded)
    /// </summary>
    public bool RemoveName(string name)
    {
        Transient item = NamedItem(name);
        if (item is null)
        {
            return false;
        }
        theitems.Add(item, item); // remains but without name
        return thenames.UnBind(name);
    }

    /// <summary>
    /// Removes an Item given its Ident. Returns False if &lt;id&gt; is attached to no Item in the WorkSession.
    /// For a Named Item, also removes its Name.
    /// </summary>
    public bool RemoveItem(Transient? item)
    {
        if (item is null)
        {
            return false;
        }
        int id = theitems.FindIndex(item);
        if (id == 0)
        {
            return false;
        }
        ref Transient att = ref theitems.ChangeFromIndex(id);
        if (att is null)
        {
            return false; // already canceled
        }

        //  Special cases : Dispatch,Modifier
        theshareout.RemoveItem(item);

        //  Mark "Removed" in the Map (we cannot empty it)
        if (att.IsKind(typeof(TCollection_HAsciiString)))
        {
            if (!thenames.UnBind((att as TCollection_HAsciiString).ToCString()))
            {
                return false;
            }
        }
        att = null; // cf ChangeFromIndex
        //  id = theitems.Add(item,att);
        return true;
    }

    /// <summary>
    /// Clears all the recorded Items : Selections, Dispatches, Modifiers, and Strings &amp; IntParams, with
    /// their Idents &amp; Names. Remark that if a Model has been loaded, it is not cleared.
    /// </summary>
    public void ClearItems()
    {
        thenames.Clear();
        theitems.Clear();
        theshareout.Clear(false);
    }

    /// <summary>
    /// Returns a Label which illustrates the content of an Item, given its Ident. This Label is : - for a Text
    /// Parameter, "Text:&lt;text value&gt;" - for an Integer Parameter, "Integer:&lt;integer value&gt;" - for a
    /// Selection, a Dispatch or a Modifier, its Label (see these classes) - for any other kind of Variable, its
    /// cdl type
    /// </summary>
    public TCollection_HAsciiString? ItemLabel(int id)
    {
        TCollection_HAsciiString res = null;
        Transient var = Item(id);
        if (var is null)
        {
            return res;
        }
        TCollection_HAsciiString text = var as TCollection_HAsciiString;
        if (text is not null)
        {
            res = new TCollection_HAsciiString("Text:");
            res.AssignCat(text);
            return res;
        }
        IFSelect_IntParam intpar = var as IFSelect_IntParam;
        if (intpar is not null)
        {
            res = new TCollection_HAsciiString(intpar.Value());
            res.Insert(1, "Integer:");
            return res;
        }
        IFSelect_Selection sel = var as IFSelect_Selection;
        if (sel is not null)
        {
            res = new TCollection_HAsciiString("Selection:");
            res.AssignCat(sel.Label());
            return res;
        }
        IFSelect_GeneralModifier mod = var as IFSelect_GeneralModifier;
        if (mod is not null)
        {
            if (mod.IsKind(typeof(IFSelect_Modifier)))
            {
                res = new TCollection_HAsciiString("ModelModifier:");
            }
            else
            {
                res = new TCollection_HAsciiString("FileModifier:");
            }
            res.AssignCat(mod.Label());
            return res;
        }
        IFSelect_Dispatch disp = var as IFSelect_Dispatch;
        if (disp is not null)
        {
            res = new TCollection_HAsciiString("Dispatch:");
            res.AssignCat(disp.Label());
            return res;
        }
        // IFSelect_Transformer ("Transformer:") is left out (IFSelect's cut)
        IFSelect_SignatureList slc = var as IFSelect_SignatureList;
        if (slc is not null)
        {
            res = new TCollection_HAsciiString("Counter:");
            res.AssignCat(slc.Name());
            return res;
        }
        IFSelect_Signature sig = var as IFSelect_Signature;
        if (sig is not null)
        {
            res = new TCollection_HAsciiString("Signature:");
            res.AssignCat(sig.Name());
            return res;
        }
        IFSelect_EditForm edf = var as IFSelect_EditForm;
        if (edf is not null)
        {
            res = new TCollection_HAsciiString("EditForm:");
            res.AssignCat(edf.Label());
            return res;
        }
        IFSelect_Editor edt = var as IFSelect_Editor;
        if (edt is not null)
        {
            res = new TCollection_HAsciiString("Editor:");
            res.AssignCat(edt.Label());
            return res;
        }
        res = new TCollection_HAsciiString("VariableType:");
        res.AssignCat(var.DynamicTypeName());
        return res;
    }

    /// <summary>
    /// Fills a Sequence with the List of Idents attached to the Items of which Type complies with (IsKind)
    /// &lt;type&gt; (alphabetic order) Remark : &lt;type&gt; = TYPE(Standard_Transient) gives all the Idents
    /// which are suitable in the WorkSession
    /// </summary>
    public NCollection_HSequence<int>? ItemIdents(Type? type)
    {
        NCollection_HSequence<int> list = new NCollection_HSequence<int>();
        int nb = theitems.Extent();
        for (int i = 1; i <= nb; i++)
        {
            if (type is not null && theitems.FindKey(i).IsKind(type))
            {
                list.Append(i);
            }
        }
        return list;
    }

    /// <summary>
    /// Fills a Sequence with the list of the Names attached to Items of which Type complies with (IsKind)
    /// &lt;type&gt; (alphabetic order) Remark : &lt;type&gt; = TYPE(Standard_Transient) gives all the Names
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? ItemNames(Type? type)
    {
        NCollection_HSequence<TCollection_HAsciiString> list =
            new NCollection_HSequence<TCollection_HAsciiString>();
        foreach (KeyValuePair<string, Transient> IT in thenames)
        {
            if (type is not null && IT.Value.IsKind(type))
            {
                list.Append(new TCollection_HAsciiString(IT.Key));
            }
        }
        return list;
    }

    /// <summary>
    /// Fills a Sequence with the NAMES of the control items, of which the label matches &lt;label&gt; (contain
    /// it) : see NextIdentForLabel Search mode is fixed to "contained" If &lt;label&gt; is empty, returns all
    /// Names
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? ItemNamesForLabel(string label)
    {
        NCollection_HSequence<TCollection_HAsciiString> list =
            new NCollection_HSequence<TCollection_HAsciiString>();
        int i, nb = MaxIdent();
        for (i = 1; i <= nb; i++)
        {
            TCollection_HAsciiString lab = ItemLabel(i);
            Transient item = Item(i);
            if (lab is null)
            {
                continue;
            }
            if (CStringChar(label, 0) != '\0' && lab.Search(label) <= 0)
            {
                continue;
            }

            TCollection_HAsciiString nom = Name(Item(i));
            if (nom is null)
            {
                nom = new TCollection_HAsciiString(i);
                nom.Insert(1, '#');
            }
            else
            {
                nom = new TCollection_HAsciiString(nom);
            }
            list.Append(new TCollection_HAsciiString(lab));
        }
        return list;
    }

    /// <summary>
    /// For query by Label with possible iterations Searches the Ident of which Item has a Label which matches a
    /// given one, the search starts from an initial Ident. Returns the first found Ident which follows
    /// &lt;id&gt;, or ZERO The search must start with &lt;id&gt; = 0, it returns the next Ident which matches.
    /// To iterate, call again this method which this returned value as &lt;id&gt;. Once an Ident has been
    /// returned, the Item can be obtained by the method Item &lt;mode&gt; precises the required matching mode :
    /// - 0 (Default) : &lt;label&gt; must match exactly with the Item Label - 1 : &lt;label&gt; must match the
    /// exact beginning (the end is free) - 2 : &lt;label&gt; must be at least once wherever in the Item Label -
    /// other values are ignored
    /// </summary>
    public int NextIdentForLabel(string label, int id, int mode = 0)
    {
        int nb = MaxIdent();
        for (int i = id + 1; i <= nb; i++)
        {
            TCollection_HAsciiString lab = ItemLabel(i);
            if (lab is null)
            {
                continue;
            }
            switch (mode)
            {
                case 0:
                    if (lab.ToCString() == label)
                    {
                        return i;
                    }
                    break; // switch
                case 1:
                    if (lab.Search(label) == 1)
                    {
                        return i;
                    }
                    break;
                case 2:
                    if (lab.Search(label) > 0)
                    {
                        return i;
                    }
                    break;
                default:
                    break; // break from switch
            }
        }
        return 0; // here : not found
    }

    /// <summary>
    /// Creates a parameter as being bound to a Static If the Static is Integer, this creates an IntParam bound
    /// to it by its name. Else this creates a String which is the value of the Static. Returns a null handle if
    /// &lt;statname&gt; is unknown as a Static
    /// </summary>
    public Transient? NewParamFromStatic(string statname, string name = "")
    {
        Transient param = null;
        Interface_Static stat = thestatics.Static(statname);
        if (stat is null)
        {
            return param;
        }
        if (stat.Type() == Interface_ParamType.Interface_ParamInteger)
        {
            IFSelect_IntParam intpar = new IFSelect_IntParam();
            intpar.SetStaticName(statname, thestatics);
            param = intpar;
        }
        else
        {
            param = stat.HStringValue();
        }
        if (param is null)
        {
            return param;
        }
        if (AddNamedItem(name, param) == 0)
        {
            param = null;
        }
        return param;
    }

    /// <summary>
    /// Returns an IntParam, given its Ident in the Session Null result if &lt;id&gt; is not suitable for an
    /// IntParam (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_IntParam? IntParam(int id)
    {
        return (Item(id) as IFSelect_IntParam);
    }

    /// <summary>
    /// Returns Integer Value of an IntParam
    /// </summary>
    public int IntValue(IFSelect_IntParam? par)
    {
        if (par is not null)
        {
            return par.Value();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>
    /// Creates a new IntParam. A Name can be set (Optional) Returns the created IntParam, or a Null Handle in
    /// case of Failure (see AddItem/AddNamedItem)
    /// </summary>
    public IFSelect_IntParam? NewIntParam(string name = "")
    {
        IFSelect_IntParam intpar = new IFSelect_IntParam();
        if (AddNamedItem(name, intpar) == 0)
        {
            intpar = null;
        }
        return intpar;
    }

    /// <summary>
    /// Changes the Integer Value of an IntParam Returns True if Done, False if &lt;it&gt; is not in the
    /// WorkSession
    /// </summary>
    public bool SetIntValue(IFSelect_IntParam? par, int val)
    {
        if (ItemIdent(par) == 0)
        {
            return false;
        }
        par.SetValue(val);
        return true;
    }

    /// <summary>
    /// Returns a TextParam, given its Ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// TextParam (undefined, or defined for another kind of variable)
    /// </summary>
    public TCollection_HAsciiString? TextParam(int id)
    {
        return (Item(id) as TCollection_HAsciiString);
    }

    /// <summary>
    /// Returns Text Value of a TextParam (a String) or an empty string if &lt;it&gt; is not in the WorkSession
    /// </summary>
    public string TextValue(TCollection_HAsciiString? par)
    {
        if (par is not null)
        {
            return par.ToCString();
        }
        else
        {
            return "";
        }
    }

    /// <summary>
    /// Creates a new (empty) TextParam. A Name can be set (Optional) Returns the created TextParam (as an
    /// HAsciiString), or a Null Handle in case of Failure (see AddItem/AddNamedItem)
    /// </summary>
    public TCollection_HAsciiString? NewTextParam(string name = "")
    {
        TCollection_HAsciiString textpar = new TCollection_HAsciiString("");
        if (AddNamedItem(name, textpar) == 0)
        {
            textpar = null;
        }
        return textpar;
    }

    /// <summary>
    /// Changes the Text Value of a TextParam (an HAsciiString) Returns True if Done, False if &lt;it&gt; is not
    /// in the WorkSession
    /// </summary>
    public bool SetTextValue(TCollection_HAsciiString? par, string val)
    {
        if (ItemIdent(par) == 0)
        {
            return false;
        }
        par.Clear();
        par.AssignCat(val);
        return true;
    }

    /// <summary>
    /// Returns a Signature, given its Ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// Signature (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_Signature? Signature(int id)
    {
        return (Item(id) as IFSelect_Signature);
    }

    /// <summary>
    /// Returns the Value computed by a Signature for an Entity Returns an empty string if the entity does not
    /// belong to the loaded model
    /// </summary>
    public string SignValue(IFSelect_Signature? sign, Transient? ent)
    {
        if (sign is null || myModel is null)
        {
            return "";
        }
        if (StartingNumber(ent) == 0)
        {
            return "";
        }
        return sign.Value(ent, myModel);
    }

    /// <summary>
    /// Returns a Selection, given its Ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// Selection (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_Selection? Selection(int id)
    {
        return (Item(id) as IFSelect_Selection);
    }

    /// <summary>
    /// Evaluates the effect of a Selection applied on the input Model Returned Result remains empty if no input
    /// Model has been set
    /// </summary>
    public Interface_EntityIterator EvalSelection(IFSelect_Selection? sel)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        if (myErrHand)
        {
            myErrHand = false;
            try
            {
                iter = EvalSelection(sel); // normal call (therefore, code not duplicated)
            }
            catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
            {
                using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append("    ****    EvalSelection Interrupted by Exception :   ****\n");
                sout.Append(Transfer_ProcessForTransient.What(anException));
                sout.Append("\n    Abandon");
                sout.Append('\n');
            }
            myErrHand = theerrhand;
            return iter;
        }

        if (thegraph is null)
        {
            return iter;
        }
        iter = sel.UniqueResult(thegraph.Graph());
        return iter;
    }

    /// <summary>
    /// Returns the Selections which are source of Selection, given its rank in the List of Selections (see
    /// SelectionIterator) Returned value is empty if &lt;num&gt; is out of range or if &lt;sel&gt; is not in the
    /// WorkSession
    /// </summary>
    public IFSelect_SelectionIterator Sources(IFSelect_Selection? sel)
    {
        return new IFSelect_SelectionIterator(sel);
    }

    /// <summary>
    /// Returns the result of a Selection, computed by EvalSelection (see above) under the form of a HSequence
    /// (hence, it can be used by a frontal-engine logic). It can be empty Returns a Null Handle if &lt;sel&gt;
    /// is not in the WorkSession
    /// </summary>
    public NCollection_HSequence<Transient>? SelectionResult(IFSelect_Selection? sel)
    {
        NCollection_HSequence<Transient> res = null;
        if (myErrHand)
        {
            myErrHand = false;
            try
            {
                res = SelectionResult(sel); // appel normal (.code unique)
            }
            catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
            {
                using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append("    ****    SelectionResult Interrupted by Exception :   ****\n");
                sout.Append(Transfer_ProcessForTransient.What(anException));
                sout.Append("\n    Abandon");
                sout.Append('\n');
            }
            myErrHand = theerrhand;
            return res;
        }

        if (!IsLoaded())
        {
            Console.Out.Write(" ***  Data for Evaluation not available  ***");
            Console.Out.Write('\n');
            return new NCollection_HSequence<Transient>();
        }
        //  if (ItemIdent(sel) == 0)
        if (sel is null)
        {
            Console.Out.Write(" Selection :  Unknown");
            Console.Out.Write('\n');
            return res;
        } // std.cout<<Handle
        return EvalSelection(sel).Content();
    }

    /// <summary>
    /// Returns the result of a Selection, by forcing its input with a given list &lt;list&gt; (unless
    /// &lt;list&gt; is Null). RULES : &lt;list&gt; applies only for a SelectDeduct kind Selection : its Input is
    /// considered : if it is a SelectDeduct kind Selection, its Input is considered, etc... until an Input is
    /// not a Deduct/Extract : its result is replaced by &lt;list&gt; and all the chain of deductions is applied
    /// </summary>
    public NCollection_HSequence<Transient>? SelectionResultFromList(IFSelect_Selection? sel, NCollection_HSequence<Transient>? list)
    {
        if (list is null)
        {
            return SelectionResult(sel);
        }
        IFSelect_SelectDeduct deduct = sel as IFSelect_SelectDeduct;
        if (deduct is null)
        {
            return SelectionResult(sel);
        }

        //   We will search for the last deduction in the input chain
        IFSelect_Selection ssel, newinput;
        ssel = sel;
        int i, nb = MaxIdent();
        for (i = 1; i <= nb * 2; i++)
        {
            newinput = deduct.Input();
            deduct = (newinput as IFSelect_SelectDeduct);
            if (deduct is null)
            {
                break;
            }
            ssel = newinput;
        }

        //  we're there (finally, we should be)
        //  ssel is the last selection examined, deduct its downcast
        //  input its Input (null if sel is not a deduction)
        deduct = (ssel as IFSelect_SelectDeduct);

        deduct.Alternate().SetList(list);

        //   We execute then we clean
        NCollection_HSequence<Transient> res = SelectionResult(sel);
        ////  deduct.SetInput (newinput);
        return res;
    }

    /// <summary>
    /// Sets a Selection as input for an item, according its type : if &lt;item&gt; is a Dispatch : as Final
    /// Selection if &lt;item&gt; is a GeneralModifier (i.e. any kind of Modifier) : as Selection used to filter
    /// entities to modify &lt;sel&gt; Null causes this Selection to be nullified Returns False if &lt;item&gt;
    /// is not of a suitable type, or &lt;item&gt; or &lt;sel&gt; is not in the WorkSession
    /// </summary>
    public bool SetItemSelection(Transient? item, IFSelect_Selection? sel)
    {
        IFSelect_Dispatch disp = item as IFSelect_Dispatch;
        IFSelect_GeneralModifier modif = item as IFSelect_GeneralModifier;
        if (disp is not null)
        {
            if (ItemIdent(disp) == 0)
            {
                return false;
            }
            //  Selection Nulle : Annuler FinalSelection
            if (sel is not null && ItemIdent(sel) == 0)
            {
                return false;
            }
            disp.SetFinalSelection(sel);
            return true;
        }
        if (modif is not null)
        {
            if (ItemIdent(modif) == 0)
            {
                return false;
            }
            if (sel is not null && ItemIdent(sel) == 0)
            {
                return false;
            }
            //   Selection Nulle : Annuler Selection
            modif.SetSelection(sel);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Resets input Selection which was set by SetItemSelection Same conditions as for SetItemSelection Returns
    /// True if done, False if &lt;item&gt; is not in the WorkSession
    /// </summary>
    public bool ResetItemSelection(Transient? item)
    {
        IFSelect_Selection nulsel = null;
        return SetItemSelection(item, nulsel);
    }

    /// <summary>
    /// Returns the Selection of a Dispatch or a GeneralModifier. Returns a Null Handle if none is defined or
    /// &lt;item&gt; not good type
    /// </summary>
    public IFSelect_Selection? ItemSelection(Transient? item)
    {
        IFSelect_Selection sel = null;
        IFSelect_Dispatch disp = item as IFSelect_Dispatch;
        IFSelect_GeneralModifier modif = item as IFSelect_GeneralModifier;
        if (ItemIdent(disp) > 0)
        {
            return disp.FinalSelection();
        }
        if (ItemIdent(modif) > 0)
        {
            return modif.Selection();
        }
        return sel; // Null or unknown . Null
    }

    /// <summary>
    /// Returns a SignCounter from its ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// SignCounter (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_SignCounter? SignCounter(int id)
    {
        return (Item(id) as IFSelect_SignCounter);
    }

    /// <summary>
    /// Computes the content of a SignCounter when it is defined with a Selection, then returns True Returns
    /// False if the SignCounter is not defined with a Selection, or if its Selection Mode is inhibited
    /// &lt;forced&gt; to work around optimisations
    /// </summary>
    public bool ComputeCounter(IFSelect_SignCounter? counter, bool forced = false)
    {
        if (counter is null)
        {
            return false;
        }
        if (!ComputeGraph())
        {
            return false;
        }
        return counter.ComputeSelected(Graph(), forced);
    }

    /// <summary>
    /// Computes the content of a SignCounter from an input list If &lt;list&gt; is Null, uses internal
    /// definition of the Counter : a Selection, else the whole Model (recomputation forced) If &lt;clear&gt; is
    /// True (D), starts from scratch Else, cumulates computations
    /// </summary>
    public bool ComputeCounterFromList(IFSelect_SignCounter? counter, NCollection_HSequence<Transient>? list, bool clear = true)
    {
        if (counter is null)
        {
            return false;
        }
        if (clear)
        {
            counter.Clear();
        }
        if (list is null)
        {
            return ComputeCounter(counter, true);
        }
        counter.AddList(list, myModel);
        return true;
    }

    /// <summary>
    /// Returns the ordered list of dispatches stored by the ShareOut
    /// </summary>
    public NCollection_HSequence<int>? AppliedDispatches()
    {
        NCollection_HSequence<int> list = new NCollection_HSequence<int>();
        int nb = theshareout.NbDispatches();
        for (int i = 1; i <= nb; i++)
        {
            list.Append(ItemIdent(theshareout.Dispatch(i)));
        }
        return list;
    }

    /// <summary>
    /// Clears the list of Dispatches recorded by the ShareOut if &lt;only&gt; disp is True, tha's all. Else,
    /// clears also the lists of Modifiers recorded by the ShareOut
    /// </summary>
    public void ClearShareOut(bool onlydisp)
    {
        theshareout.Clear(onlydisp);
    }

    /// <summary>
    /// Returns a Dispatch, given its Ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// Dispatch (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_Dispatch? Dispatch(int id)
    {
        return (Item(id) as IFSelect_Dispatch);
    }

    /// <summary>
    /// Returns the rank of a Dispatch in the ShareOut, or 0 if &lt;disp&gt; is not in the ShareOut or not in the
    /// WorkSession
    /// </summary>
    public int DispatchRank(IFSelect_Dispatch? disp)
    {
        if (ItemIdent(disp) == 0)
        {
            return 0;
        }
        return theshareout.DispatchRank(disp);
    }

    /// <summary>
    /// Gives access to the complete ModelCopier
    /// </summary>
    public IFSelect_ModelCopier? ModelCopier()
    {
        return thecopier;
    }

    /// <summary>
    /// Sets a new ModelCopier. Fills Items which its content
    /// </summary>
    public void SetModelCopier(IFSelect_ModelCopier? copier)
    {
        thecopier = copier;
        thecopier.SetShareOut(theshareout);
    }

    /// <summary>
    /// Returns the count of Modifiers applied to final sending Model Modifiers if &lt;formodel&gt; is True, File
    /// Modifiers else (i.e. Modifiers which apply once the Models have been filled)
    /// </summary>
    public int NbFinalModifiers(bool formodel)
    {
        return theshareout.NbModifiers(formodel);
    }

    /// <summary>
    /// Fills a Sequence with a list of Idents, those attached to the Modifiers applied to final sending. Model
    /// Modifiers if &lt;formodel&gt; is True, File Modifiers else This list is given in the order in which they
    /// will be applied (which takes into account the Changes to Modifier Ranks)
    /// </summary>
    public NCollection_HSequence<int>? FinalModifierIdents(bool formodel)
    {
        //  return ItemIdents(typeof(IFSelect_Modifier));
        //  We give the list in ModelCopier order, which is authoritative
        NCollection_HSequence<int> list = new NCollection_HSequence<int>();
        int nbm = theshareout.NbModifiers(formodel);
        for (int i = 1; i <= nbm; i++)
        {
            list.Append(ItemIdent(theshareout.GeneralModifier(formodel, i)));
        }
        return list;
    }

    /// <summary>
    /// Returns a Modifier, given its Ident in the Session Null result if &lt;id&gt; is not suitable for a
    /// Modifier (undefined, or defined for another kind of variable)
    /// </summary>
    public IFSelect_GeneralModifier? GeneralModifier(int id)
    {
        return (Item(id) as IFSelect_GeneralModifier);
    }

    /// <summary>
    /// Returns a Model Modifier, given its Ident in the Session, i.e. typed as a Modifier (not simply a
    /// GeneralModifier) Null result if &lt;id&gt; is not suitable for a Modifier (undefined, or defined for
    /// another kind of variable)
    /// </summary>
    public IFSelect_Modifier? ModelModifier(int id)
    {
        return (Item(id) as IFSelect_Modifier);
    }

    /// <summary>
    /// Returns the Rank of a Modifier given its Ident. Model or File Modifier according its type (ModelModifier
    /// or not) Remember that Modifiers are applied sequentially following their Rank : first Model Modifiers
    /// then File Modifiers Rank is given by rank of call to AddItem and can be changed by ChangeModifierRank
    /// </summary>
    public int ModifierRank(IFSelect_GeneralModifier? modif)
    {
        if (ItemIdent(modif) == 0)
        {
            return 0;
        }
        return theshareout.ModifierRank(modif);
    }

    /// <summary>
    /// Changes the Rank of a Modifier in the Session : Model Modifiers if &lt;formodel&gt; is True, File
    /// Modifiers else the Modifier n0 &lt;before&gt; is put to n0 &lt;after&gt; Return True if Done, False if
    /// &lt;before&gt; or &lt;after&gt; out of range
    /// </summary>
    public bool ChangeModifierRank(bool formodel, int before, int after)
    {
        return theshareout.ChangeModifierRank(formodel, before, after);
    }

    /// <summary>
    /// Removes all the Modifiers active in the ModelCopier : they become inactive and they are removed from the
    /// Session
    /// </summary>
    public void ClearFinalModifiers()
    {
        NCollection_HSequence<int> list = FinalModifierIdents(true);
        int nb = list.Length();
        int i; // svv #1
        for (i = 1; i <= nb; i++)
        {
            RemoveItem(GeneralModifier(list.Value(i)));
        }
        list = FinalModifierIdents(false);
        nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            RemoveItem(GeneralModifier(list.Value(i)));
        }
    }

    /// <summary>
    /// Sets a GeneralModifier to be applied to an item : - item = ShareOut : applies for final sending (all
    /// dispatches) - item is a Dispatch : applies for this dispatch only Returns True if done, False if
    /// &lt;modif&gt; or &lt;item&gt; not in &lt;me&gt;
    /// </summary>
    public bool SetAppliedModifier(IFSelect_GeneralModifier? modif, Transient? item)
    {
        if (ItemIdent(modif) == 0)
        {
            return false;
        }

        if (item is null)
        {
            return false;
        }
        if (item == theshareout)
        {
            theshareout.AddModifier(modif, 0);
            return true;
        }
        if (item.IsKind(typeof(IFSelect_Dispatch)))
        {
            IFSelect_Dispatch disp = item as IFSelect_Dispatch;
            theshareout.AddModifier(modif, 0);
            modif.SetDispatch(disp);
            return true;
        }
        // An IFSelect_TransformStandard (its modifiers): left out with the class (IFSelect's cut)
        return false;
    }

    /// <summary>
    /// Resets a GeneralModifier to be applied Returns True if done, False if &lt;modif&gt; was not applied
    /// </summary>
    public bool ResetAppliedModifier(IFSelect_GeneralModifier? modif)
    {
        if (ItemIdent(modif) == 0)
        {
            return false;
        }

        return theshareout.RemoveItem(modif);
    }

    /// <summary>
    /// Returns the item on which a GeneralModifier is applied : the ShareOut, or a given Dispatch Returns a Null
    /// Handle if &lt;modif&gt; is not applied
    /// </summary>
    public Transient? UsesAppliedModifier(IFSelect_GeneralModifier? modif)
    {
        Transient res = null;
        if (ItemIdent(modif) == 0)
        {
            return res;
        }
        if (theshareout.ModifierRank(modif) == 0)
        {
            return res;
        }
        res = modif.Dispatch();
        if (res is null)
        {
            res = theshareout;
        }
        return res;
    }

    /// <summary>
    /// Returns the defined File Prefix. Null Handle if not defined
    /// </summary>
    public TCollection_HAsciiString? FilePrefix()
    {
        return theshareout.Prefix();
    }

    /// <summary>
    /// Returns the defined Default File Root. It is used for Dispatches which have no specific root attached.
    /// Null Handle if not defined
    /// </summary>
    public TCollection_HAsciiString? DefaultFileRoot()
    {
        return theshareout.DefaultRootName();
    }

    /// <summary>
    /// Returns the defined File Extension. Null Handle if not defined
    /// </summary>
    public TCollection_HAsciiString? FileExtension()
    {
        return theshareout.Extension();
    }

    /// <summary>
    /// Returns the File Root defined for a Dispatch. Null if no Root Name is defined for it (hence, no File will
    /// be produced)
    /// </summary>
    public TCollection_HAsciiString? FileRoot(IFSelect_Dispatch? disp)
    {
        return theshareout.RootName(theshareout.DispatchRank(disp));
    }

    /// <summary>
    /// Defines a File Prefix
    /// </summary>
    public void SetFilePrefix(string name)
    {
        theshareout.SetPrefix(new TCollection_HAsciiString(name));
    }

    /// <summary>
    /// Defines a Default File Root Name. Clears it is &lt;name&gt; = "" Returns True if OK, False if
    /// &lt;name&gt; already set for a Dispatch
    /// </summary>
    public bool SetDefaultFileRoot(string name)
    {
        TCollection_HAsciiString defrt = null;
        if (CStringChar(name, 0) != '\0')
        {
            defrt = new TCollection_HAsciiString(name);
        }
        return theshareout.SetDefaultRootName(defrt);
    }

    /// <summary>
    /// Defines a File Extension
    /// </summary>
    public void SetFileExtension(string name)
    {
        theshareout.SetExtension(new TCollection_HAsciiString(name));
    }

    /// <summary>
    /// Defines a Root for a Dispatch If &lt;name&gt; is empty, clears Root Name This has as effect to inhibit
    /// the production of File by &lt;disp&gt; Returns False if &lt;disp&gt; is not in the WorkSession or if a
    /// root name is already defined for it
    /// </summary>
    public bool SetFileRoot(IFSelect_Dispatch? disp, string namefile)
    {
        int id = ItemIdent(disp);
        if (id == 0)
        {
            return false;
        }
        int nd = theshareout.DispatchRank(disp);
        /* if (theonlynamed) {
                if (nd == 0 && namefile[0] != 0)
                    theshareout.AddDispatch(disp);
                else if (nd != 0 && namefile[0] == 0)
                    theshareout.RemoveDispatch (nd);
            } */
        if (nd == 0)
        {
            return false;
        }
        //  The order below prevented to change the root name on a given dispatch !
        //  if (theshareout.HasRootName(nd)) return false;
        TCollection_HAsciiString filename = null;
        if (CStringChar(namefile, 0) != '\0')
        {
            filename = new TCollection_HAsciiString(namefile);
        }
        return theshareout.SetRootName(nd, filename);
    }

    /// <summary>
    /// Extracts File Root Name from a given complete file name (uses OSD_Path)
    /// </summary>
    public string GiveFileRoot(string file)
    {
        // OSD_Path(file).Name(): the file name without its folder and extension
        if (file is null)
        {
            return file;
        }
        string bufstr = file;
        int sep = Math.Max(bufstr.LastIndexOf('/'), bufstr.LastIndexOf('\\'));
        if (sep >= 0)
        {
            bufstr = bufstr.Substring(sep + 1);
        }
        int dot = bufstr.LastIndexOf('.');
        if (dot >= 0)
        {
            bufstr = bufstr.Substring(0, dot);
        }
        return bufstr;
    }

    /// <summary>
    /// Completes a file name as required, with Prefix and Extension (if defined; for a non-defined item,
    /// completes nothing)
    /// </summary>
    public string GiveFileComplete(string file)
    {
        //  add if needed: Prefix; Extension
        string bufstr = file;
        int i, j = 0, nb = bufstr.Length;
        TCollection_HAsciiString ext = FileExtension();
        if (ext is not null)
        {
            char val0 = '\0';
            if (ext.Length() > 0)
            {
                val0 = ext.Value(1);
            }
            for (i = nb; i > 0; i--)
            {
                if (bufstr[i - 1] == val0)
                {
                    j = 1;
                    break;
                }
            }
            if (j == 0)
            {
                bufstr += ext.ToCString();
            }
        }
        TCollection_HAsciiString pre = FilePrefix();
        if (pre is not null)
        {
            char val1 = '\0';
            if (pre.Length() > 0)
            {
                val1 = pre.Value(pre.Length());
            }
            j = 0;
            for (i = nb; i > 0; i--)
            {
                if (bufstr[i - 1] == val1)
                {
                    j = 1;
                    break;
                }
            }
            if (j == 0)
            {
                bufstr = pre.ToCString() + bufstr;
            }
        }

        return bufstr;
    }

    /// <summary>
    /// Erases all stored data from the File Evaluation (i.e. ALL former naming information are lost)
    /// </summary>
    public void ClearFile()
    {
        thecopier.ClearResult();
        theshareout.ClearResult(true);
    }

    /// <summary>
    /// Returns the count of produced Models
    /// </summary>
    public int NbFiles()
    {
        return thecopier.NbFiles();
    }

    /// <summary>
    /// Returns a Model, given its rank in the Evaluation List
    /// </summary>
    public Interface_InterfaceModel? FileModel(int num)
    {
        Interface_InterfaceModel mod = null;
        if (num > 0 && num <= NbFiles())
        {
            mod = thecopier.FileModel(num);
        }
        return mod;
    }

    /// <summary>
    /// Returns the name of a file corresponding to a produced Model, given its rank in the Evaluation List
    /// </summary>
    public string FileName(int num)
    {
        string name = "";
        if (num > 0 && num <= NbFiles())
        {
            name = thecopier.FileName(num);
        }
        return name;
    }

    /// <summary>
    /// Commands file sending to clear the list of already sent files, commands to record a new one if
    /// &lt;record&gt; is True This list is managed by the ModelCopier when SendSplit is called It allows a
    /// global exploitation of the set of sent files
    /// </summary>
    public void BeginSentFiles(bool record)
    {
        thecopier.BeginSentFiles(theshareout, record);
    }

    /// <summary>
    /// Returns the list of recorded sent files, or a Null Handle is recording has not been enabled
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? SentFiles()
    {
        return thecopier.SentFiles();
    }

    /// <summary>
    /// Sends the starting Model into one file, without splitting, managing remaining data or anything else.
    /// &lt;computegraph&gt; true commands the Graph to be recomputed before sending : required when a Model is
    /// filled in several steps The Model and File Modifiers recorded to be applied on sending files are. Returns
    /// a status of execution : Done if OK, Void if no data available, Error if errors occurred (work library is
    /// not defined), errors during translation Fail if exception during translation is raised Stop if no disk
    /// space or disk, file is write protected Fills LastRunCheckList
    /// </summary>
    /// <remarks>
    /// OCCT.NET: <paramref name="theStream"/>, when given, is where the work library writes, in place of the file
    /// (decision 1225).
    /// </remarks>
    public IFSelect_ReturnStatus SendAll(string filename, bool computegraph = false, Stream? theStream = null)
    {
        ////...
        Interface_CheckIterator checks = new Interface_CheckIterator();
        if (!IsLoaded())
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        if (thelibrary is null)
        {
            checks.CCheck(0).AddFail("WorkLibrary undefined");
            thecheckrun = checks;
            return IFSelect_ReturnStatus.IFSelect_RetError;
        }

        if (myErrHand)
        {
            myErrHand = false; // OCCT: restored on an exception only, so later calls run unprotected
            try
            {
                ComputeGraph(computegraph);
                checks = thecopier.SendAll(filename, thegraph.Graph(), thelibrary, theprotocol, theStream);
            }
            catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
            {
                using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append("    ****    SendAll Interrupted by Exception :   ****\n");
                sout.Append(Transfer_ProcessForTransient.What(anException));
                sout.Append("\n    Abandon");
                sout.Append('\n');
                myErrHand = theerrhand;
                checks.CCheck(0).AddFail("Exception Raised -> Abandon");
                thecheckrun = checks;
                return IFSelect_ReturnStatus.IFSelect_RetFail;
            }
        }
        else
        {
            checks = thecopier.SendAll(filename, thegraph.Graph(), thelibrary, theprotocol, theStream);
        }
        Interface_Check aMainFail = checks.CCheck(0);
        if (aMainFail is not null && aMainFail.HasFailed())
        {
            return IFSelect_ReturnStatus.IFSelect_RetStop;
        }
        if (theloaded.Length == 0)
        {
            theloaded += filename;
        }
        thecheckrun = checks;
        if (checks.IsEmpty(true))
        {
            return IFSelect_ReturnStatus.IFSelect_RetDone;
        }
        return IFSelect_ReturnStatus.IFSelect_RetError;
    }

    /// <summary>
    /// Sends a part of the starting Model into one file, without splitting. But remaining data are managed.
    /// &lt;computegraph&gt; true commands the Graph to be recomputed before sending : required when a Model is
    /// filled in several steps The Model and File Modifiers recorded to be applied on sending files are. Returns
    /// a status : Done if OK, Fail if error during send, Error : WorkLibrary not defined, Void : selection list
    /// empty Fills LastRunCheckList
    /// </summary>
    /// <remarks>
    /// OCCT.NET: <paramref name="theStream"/>, when given, is where the work library writes, in place of the file
    /// (decision 1225).
    /// </remarks>
    public IFSelect_ReturnStatus SendSelected(string filename, IFSelect_Selection? sel, bool computegraph = false, Stream? theStream = null)
    {
        ////...
        if (!IsLoaded())
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        Interface_CheckIterator checks = new Interface_CheckIterator();
        if (thelibrary is null)
        {
            checks.CCheck(0).AddFail("WorkLibrary undefined");
            thecheckrun = checks;
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }

        if (myErrHand)
        {
            myErrHand = false; // OCCT: restored on an exception only, so later calls run unprotected
            try
            {
                ComputeGraph(computegraph);
                return SendSelected(filename, sel, false, theStream); // normal call
            }
            catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
            {
                using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append("    ****    SendSelected Interrupted by Exception :   ****\n");
                sout.Append(Transfer_ProcessForTransient.What(anException));
                sout.Append("\n    Abandon");
                sout.Append('\n');
                checks.CCheck(0).AddFail("Exception Raised -> Abandon");
                myErrHand = theerrhand;
                thecheckrun = checks;
                return IFSelect_ReturnStatus.IFSelect_RetFail;
            }
        }
        //  if (ItemIdent(sel) == 0) return 3;
        Interface_EntityIterator iter = sel.UniqueResult(thegraph.Graph());
        if (iter.NbEntities() == 0)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }

        checks = thecopier.SendSelected(filename, thegraph.Graph(), thelibrary, theprotocol, iter, theStream);
        thecopier.SetRemaining(thegraph.CGraph());
        thecheckrun = checks;
        if (checks.IsEmpty(true))
        {
            return IFSelect_ReturnStatus.IFSelect_RetDone;
        }
        return IFSelect_ReturnStatus.IFSelect_RetError;
    }

    /// <summary>
    /// Writes the current Interface Model globally to a File, and returns a write status which can be : Done OK,
    /// Fail file could not be written, Error no norm is selected Remark : It is a simple, one-file writing,
    /// other operations are available (such as splitting ...) which calls SendAll
    /// </summary>
    public IFSelect_ReturnStatus WriteFile(string filename)
    {
        if (WorkLibrary() is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        ComputeGraph(true);
        if (!IsLoaded())
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        return SendAll(filename);
    }

    /// <summary>
    /// Writes a sub-part of the current Interface Model to a File, as defined by a Selection &lt;sel&gt;,
    /// recomputes the Graph, and returns a write status which can be : Done OK, Fail file could not be written,
    /// Error no norm is selected Remark : It is a simple, one-file writing, other operations are available (such
    /// as splitting ...) which calls SendSelected
    /// </summary>
    public IFSelect_ReturnStatus WriteFile(string filename, IFSelect_Selection? sel)
    {
        if (WorkLibrary() is null || sel is null)
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        ComputeGraph(true);
        if (!IsLoaded())
        {
            return IFSelect_ReturnStatus.IFSelect_RetVoid;
        }
        return SendSelected(filename, sel);
    }

    /// <summary>
    /// Returns the count of Input Selections known for a Selection, or 0 if &lt;sel&gt; not in the WorkSession.
    /// This count is one for a SelectDeduct / SelectExtract kind, two for SelectControl kind, variable for a
    /// SelectCombine (Union/Intersection), zero else
    /// </summary>
    public int NbSources(IFSelect_Selection? sel)
    {
        if (ItemIdent(sel) == 0)
        {
            return 0;
        }
        if (sel.IsKind(typeof(IFSelect_SelectExtract))
                || sel.IsKind(typeof(IFSelect_SelectDeduct)))
        {
            return 1;
        }
        // IFSelect_SelectControl (2 sources) and IFSelect_SelectCombine (its inputs) are left out (IFSelect's cut)
        return 0;
    }

    /// <summary>
    /// Returns the &lt;num&gt;th Input Selection of a Selection (see NbSources). Returns a Null Handle if
    /// &lt;sel&gt; is not in the WorkSession or if &lt;num&gt; is out of the range &lt;1-NbSources&gt; To obtain
    /// more details, see the method Sources
    /// </summary>
    public IFSelect_Selection? Source(IFSelect_Selection? sel, int num = 1)
    {
        IFSelect_Selection sr = null;
        if (ItemIdent(sel) == 0)
        {
            return sr;
        }
        if (sel.IsKind(typeof(IFSelect_SelectExtract)))
        {
            sr = (sel as IFSelect_SelectExtract).Input();
        }
        else if (sel.IsKind(typeof(IFSelect_SelectDeduct)))
        {
            sr = (sel as IFSelect_SelectDeduct).Input();
        }
        // IFSelect_SelectControl and IFSelect_SelectCombine are left out (IFSelect's cut)
        return sr;
    }

    /// <summary>
    /// Returns True if &lt;sel&gt; a Reversed SelectExtract, False else
    /// </summary>
    public bool IsReversedSelectExtract(IFSelect_Selection? sel)
    {
        if (ItemIdent(sel) == 0)
        {
            return false;
        }
        IFSelect_SelectExtract sxt = sel as IFSelect_SelectExtract;
        if (sxt is null)
        {
            return false;
        }
        return (!sxt.IsDirect());
    }

    /// <summary>
    /// Toggles the Sense (Direct &lt;-&gt; Reversed) of a SelectExtract Returns True if Done, False if
    /// &lt;sel&gt; is not a SelectExtract or is not in the WorkSession
    /// </summary>
    public bool ToggleSelectExtract(IFSelect_Selection? sel)
    {
        if (ItemIdent(sel) == 0)
        {
            return false;
        }
        IFSelect_SelectExtract sxt = sel as IFSelect_SelectExtract;
        if (sxt is null)
        {
            return false;
        }
        sxt.SetDirect(!sxt.IsDirect());
        return true;
    }

    /// <summary>
    /// Sets an Input Selection (as &lt;input&gt;) to a SelectExtract or a SelectDeduct (as &lt;sel&gt;). Returns
    /// True if Done, False if &lt;sel&gt; is neither a SelectExtract nor a SelectDeduct, or not in the
    /// WorkSession
    /// </summary>
    public bool SetInputSelection(IFSelect_Selection? sel, IFSelect_Selection? inp)
    {
        if (ItemIdent(sel) == 0)
        {
            return false;
        }
        if (inp is not null && ItemIdent(inp) == 0)
        {
            return false;
        }
        IFSelect_SelectExtract sxt = sel as IFSelect_SelectExtract;
        if (sxt is not null)
        {
            sxt.SetInput(inp);
            return true;
        }
        IFSelect_SelectDeduct sdt = sel as IFSelect_SelectDeduct;
        if (sdt is not null)
        {
            sdt.SetInput(inp);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Creates a new Selection, of type SelectPointed, its content starts with &lt;list&gt;. A name must be
    /// given (can be empty)
    /// </summary>
    public IFSelect_Selection? NewSelectPointed(NCollection_HSequence<Transient>? list, string name)
    {
        IFSelect_SelectPointed sel = new IFSelect_SelectPointed();
        if (list is not null)
        {
            sel.AddList(list);
        }
        if (AddNamedItem(name, sel) == 0)
        {
            sel = null;
        }
        return sel;
    }

    /// <summary>
    /// Changes the content of a Selection of type SelectPointed According &lt;mode&gt; : 0 set &lt;list&gt; as
    /// new content (clear former) 1 : adds &lt;list&gt; to actual content -1 : removes &lt;list&gt; from actual
    /// content Returns True if done, False if &lt;sel&gt; is not a SelectPointed
    /// </summary>
    public bool SetSelectPointed(IFSelect_Selection? sel, NCollection_HSequence<Transient>? list, int mode)
    {
        IFSelect_SelectPointed sp = sel as IFSelect_SelectPointed;
        if (sp is null || list is null)
        {
            return false;
        }
        if (mode == 0)
        {
            sp.Clear();
        }
        if (mode >= 0)
        {
            sp.AddList(list);
        }
        else
        {
            sp.RemoveList(list);
        }
        return true;
    }

    /// <summary>
    /// Returns a Selection from a Name : - the name of a Selection : this Selection - the name of a Signature +
    /// criteria between (..) : a new Selection from this Signature - an entity or a list of entities : a new
    /// SelectPointed Else, returns a Null Handle
    /// </summary>
    public IFSelect_Selection? GiveSelection(string selname)
    {
        // OCCT copies the name into a char[500] buffer, cut at the first '(' and at the closing ')'
        char[] nomsel = new char[selname.Length + 1];
        int np = -1, nf = -1, nivp = 0;
        for (int n = 0; n < selname.Length && selname[n] != '\0'; n++)
        {
            nomsel[n] = selname[n];
            nomsel[n + 1] = '\0';
            if (selname[n] == '(')
            {
                np = n;
                nivp++;
            }
            if (selname[n] == ')')
            {
                nivp--;
                if (nivp <= 0)
                {
                    nf = n;
                }
            }
        }

        IFSelect_Selection sel = null;
        if (np >= 0)
        {
            nomsel[np] = '\0';
        }
        if (nf >= 0)
        {
            nomsel[nf] = '\0';
        }
        Transient item = NamedItem(CString(nomsel, 0));

        //  Parentheses? try Signature (later: parameterized Selection)
        //  NB: we count the levels of parentheses (reflected nesting)
        if (np > 0 && nf > 0)
        {
            IFSelect_SelectSignature selsign = null;
            int debsign = np + 1;

            IFSelect_Signature sign = item as IFSelect_Signature;
            IFSelect_SignCounter cnt = item as IFSelect_SignCounter;
            if (sign is not null)
            {
                selsign = new IFSelect_SelectSignature(sign, CString(nomsel, debsign), false);
            }
            else if (cnt is not null)
            {
                selsign = new IFSelect_SelectSignature(cnt, CString(nomsel, debsign), false);
            }
            else
            {
                Console.Out.Write(selname);
                Console.Out.Write(" : neither Signature nor Counter");
                Console.Out.Write('\n');
                return sel;
            }

            selsign.SetInput(new IFSelect_SelectModelEntities()); // by default
            sel = selsign;
        }

        else
        {
            sel = (item as IFSelect_Selection);
        }

        return sel;
    }

    // A C string held in a char array from an index: up to the first '\0' (OCCT.NET).
    private static string CString(char[] theBuffer, int theFrom)
    {
        int anEnd = theFrom;
        while (anEnd < theBuffer.Length && theBuffer[anEnd] != '\0')
        {
            anEnd++;
        }
        return new string(theBuffer, theFrom, anEnd - theFrom);
    }

    /// <summary>
    /// Determines a list of entities from an object : &lt;obj&gt; already HSequenceOfTransient : returned itself
    /// &lt;obj&gt; Selection : its Result of Evaluation is returned &lt;obj&gt; an entity of the Model : a
    /// HSequence which contains it else, an empty HSequence &lt;obj&gt; the Model it self : ALL its content (not
    /// only the roots)
    /// </summary>
    public NCollection_HSequence<Transient>? GiveList(Transient? obj)
    {
        //   Already a list
        NCollection_HSequence<Transient> list = obj as NCollection_HSequence<Transient>;
        if (list is not null)
        {
            return list;
        }

        //   Nothing at all: returns nothing at all
        if (obj is null)
        {
            return list;
        }

        //   A selection: its result (standard)
        IFSelect_Selection sel = obj as IFSelect_Selection;
        if (sel is not null)
        {
            Interface_EntityIterator iter = EvalSelection(sel);
            return iter.Content();
        }

        //   The model: its content
        list = new NCollection_HSequence<Transient>();
        if (obj == myModel)
        {
            int i, nb = myModel.NbEntities();
            for (i = 1; i <= nb; i++)
            {
                list.Append(myModel.Value(i));
            }
        }

        //   A model entity: this entity
        else if (StartingNumber(obj) > 0)
        {
            list.Append(obj);

            //   A Text: its interpretation
        }
        else
        {
            TCollection_HAsciiString str = obj as TCollection_HAsciiString;
            if (str is not null)
            {
                return GiveList(str.ToCString());
            }
        }

        //  If it's not all that: an empty list
        return list;
    }

    /// <summary>
    /// Computes a List of entities from two alphanums, first and second, as follows : if &lt;first&gt; is a
    /// Number or Label of an entity : this entity if &lt;first&gt; is a list of Numbers/Labels : the list of
    /// entities if &lt;first&gt; is the name of a Selection in &lt;WS&gt;, and &lt;second&gt; not defined, the
    /// standard result of this Selection else, let's consider "first second" : this whole phrase is split by
    /// blanks, as follows (RECURSIVE CALL) : - the leftest term is the final selection - the other terms define
    /// the result of the selection - and so on (the "leftest minus one" is a selection, of which the input is
    /// given by the remaining ...)
    /// </summary>
    public NCollection_HSequence<Transient>? GiveList(string first, string second = "")
    {
        NCollection_HSequence<Transient> list = null;
        if (first is null || CStringChar(first, 0) == '\0')
        {
            return list;
        }
        if (first[0] == ' ')
        {
            return GiveList(first.Substring(1), second);
        }
        if (second is not null && CStringChar(second, 0) == ' ')
        {
            return GiveList(first, second.Substring(1));
        }

        //   null list will be interpreted as SelectionResult (raw selection)
        //   otherwise as SelectionResultFromList
        if (second is not null && CStringChar(second, 0) != '\0')
        {
            list = GiveList(second, "");
        }

        list = GiveListFromList(first, list);
        return list;
    }

    /// <summary>
    /// Computes a List of entities from the model as follows &lt;first&gt; being a Selection or a combination of
    /// Selections, &lt;ent&gt; being an entity or a list of entities (as a HSequenceOfTransient) : the standard
    /// result of this selection applied to this list if &lt;ent&gt; is Null, the standard definition of the
    /// selection is used (which contains a default input selection) if &lt;selname&gt; is erroneous, a null
    /// handle is returned REMARK : selname is processed as &lt;first second&gt; of preceding GiveList
    /// </summary>
    public NCollection_HSequence<Transient>? GiveListFromList(string selname, Transient? ent)
    {
        NCollection_HSequence<Transient> list = null;
        int num;

        //   LIST DEFINED BY DEFAULT (in this case, the input list is ignored)
        if (CStringChar(selname, 0) == '(')
        {
            //  list of entities given one after another: (ID,ID,ID...)
            System.Text.StringBuilder entid = new System.Text.StringBuilder();
            int i;
            NCollection_Map<int> numap = new NCollection_Map<int>();
            list = new NCollection_HSequence<Transient>();
            for (i = 1; i < selname.Length && selname[i] != '\0'; i++)
            {
                if (selname[i] == ' ')
                {
                    continue;
                }
                if (selname[i] == ',' || selname[i] == ')')
                {
                    if (entid.Length == 0)
                    {
                        continue;
                    }
                    num = NumberFromLabel(entid.ToString());
                    entid.Clear();
                    if (num <= 0 || !numap.Add(num))
                    {
                        continue;
                    }
                    Transient anent = StartingEntity(num);
                    if (anent is not null)
                    {
                        list.Append(anent);
                    }
                    if (selname[i] == ')')
                    {
                        break;
                    }
                    continue;
                }
                entid.Append(selname[i]); // OCCT: a char[50] buffer, not checked
            }
            return list;
        }
        num = NumberFromLabel(selname);
        if (num > 0)
        {
            return GiveList(StartingEntity(num));
        }

        //  Other cases: is there an input list.
        //   Si OUI . SelectionResultFromList.  Si NON . SelectionResult
        //   If an isolated entity . we make it a list

        list = GiveList(ent); // ent null . list null otherwise interpreted

        //    Decomposition term1 term2 ...

        string nomsel = selname;
        int n, nb = -1;
        for (n = 0; n < selname.Length && selname[n] != '\0'; n++)
        {
            if (selname[n] == ' ')
            {
                nb = n;
                nomsel = selname.Substring(0, n);
                break;
            }
        }
        if (nomsel.Length == 0)
        {
            return list;
        }

        IFSelect_Selection sel = GiveSelection(nomsel);
        if (sel is null)
        {
            Console.Out.Write("Neither Entity Number/Label nor Selection :");
            Console.Out.Write(nomsel);
            Console.Out.Write('\n');
            return list;
        }

        if (nb > 0)
        {
            list = GiveListFromList(selname.Substring(nb + 1), list);
        }

        if (list is null)
        {
            list = SelectionResult(sel);
        }
        else
        {
            list = SelectionResultFromList(sel, list);
        }

        return list;
    }

    /// <summary>
    /// Combines two lists and returns the result, according to mode : &lt;mode&gt; &lt; 0 : entities in
    /// &lt;l1&gt; AND NOT in &lt;l2&gt; &lt;mode&gt; = 0 : entities in &lt;l1&gt; AND in &lt;l2&gt; &lt;mode&gt;
    /// &gt; 0 : entities in &lt;l1&gt; OR in &lt;l2&gt;
    /// </summary>
    public NCollection_HSequence<Transient>? GiveListCombined(NCollection_HSequence<Transient>? l1, NCollection_HSequence<Transient>? l2, int mode)
    {
        NCollection_HSequence<Transient> list = null; // OCCT: never created, so Append fails below
        if (l1 is null || l2 is null)
        {
            return list;
        }

        //  mode < 0 l1-l2  = 0 l1&l2  > 0 l1|l2 (l1+l2)
        NCollection_Map<Transient> numap = new NCollection_Map<Transient>();
        int i, n = l2.Length();
        for (i = n; i > 0; i--)
        {
            Transient ent = l2.Value(i);
            if (ent is null)
            {
                continue;
            }
            numap.Add(ent);
            if (mode > 0)
            {
                list.Append(ent);
            }
        }

        //  entities from l1 not already in l2
        n = l1.Length();
        for (i = n; i > 0; i--)
        {
            Transient ent = l1.Value(i);
            if (ent is null)
            {
                continue;
            }

            if (numap.Contains(ent))
            {
                //    in l1 and in l2
                if (mode == 0)
                {
                    list.Append(ent);
                }
            }
            else
            {
                //    in l1 but not in l2
                if (mode != 0)
                {
                    list.Append(ent);
                }
            }
        }

        list.Reverse();
        return list;
    }

    /// <summary>
    /// Loads data from a check iterator to query status on it
    /// </summary>
    public void QueryCheckList(Interface_CheckIterator chl)
    {
        if (!IsLoaded())
        {
            return;
        }
        int i, nb = myModel.NbEntities();
        thecheckana = new string(' ', nb + 1);
        char[] ana = thecheckana.ToCharArray();
        for (chl.Start(); chl.More(); chl.Next())
        {
            int num = chl.Number();
            Interface_Check ach = chl.Value();
            if (ach.HasFailed())
            {
                ana[num - 1] = '2';
            }
            else if (ach.HasWarnings())
            {
                ana[num - 1] = '1';
            }
        }
        //  analysis according to the graph ... codes : blc = nothing
        //  1 W/place  2 F/place  3 Wprop 4Wprop+W/place  5Wprop+F/place
        //  6 Fprop  7 Fprop+W/place  8 Fprop+F/place
        Interface_IntList list = new Interface_IntList(); // = thegraph.Graph().SharingNums(0);
                                                        //   two passes : first Warning, then Fail
        for (i = 1; i <= nb; i++)
        {
            char val = ana[i - 1];
            int quoi = -1;
            if (val == '1' || val == '4' || val == '7')
            {
                quoi = 0;
            }
            if (quoi >= 0)
            {
                IFSelect_QueryProp(list, ana, i, quoi);
            }
        }
        for (i = 1; i <= nb; i++)
        {
            char val = ana[i - 1];
            int quoi = -1;
            if (val == '2' || val == '5' || val == '8')
            {
                quoi = 1;
            }
            if (quoi >= 0)
            {
                IFSelect_QueryProp(list, ana, i, quoi);
            }
        }
        thecheckana = new string(ana);
    }

    /// <summary>
    /// Determines check status for an entity regarding last call to QueryCheckList : -1 : &lt;ent&gt; unknown in
    /// the model, ignored 0 : no check at all, immediate or inherited thru Graph 1 : immediate warning (no
    /// fail), no inherited check 2 : immediate fail, no inherited check +10 : idem but some inherited warning
    /// (no fail) +20 : idem but some inherited fail
    /// </summary>
    public int QueryCheckStatus(Transient? ent)
    {
        if (!IsLoaded())
        {
            return -1;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            return -1;
        }
        if (thecheckana.Length < num)
        {
            return -1;
        }
        char val = thecheckana[num - 1];
        //  codes : blc = rien . 0
        //  1 W/place . 1   2 F/place . 2
        //  3 Wprop . 10    4 Wprop+W/place . 11    5 Wprop+F/place . 12
        //  6 Fprop . 20    7 Fprop+W/place . 21    8 Fprop+F/place . 22
        if (val == ' ')
        {
            return 0;
        }
        if (val == '1')
        {
            return 1;
        }
        if (val == '2')
        {
            return 2;
        }
        if (val == '3')
        {
            return 10;
        }
        if (val == '4')
        {
            return 11;
        }
        if (val == '5')
        {
            return 12;
        }
        if (val == '6')
        {
            return 20;
        }
        if (val == '7')
        {
            return 21;
        }
        if (val == '8')
        {
            return 22;
        }
        return 0;
    }

    /// <summary>
    /// Determines if &lt;entdad&gt; is parent of &lt;entson&gt; (in the graph), returns : -1 if no; 0 if
    /// &lt;entdad&gt; = &lt;entson&gt; 1 if immediate parent, &gt; 1 if parent, gives count of steps
    /// </summary>
    public int QueryParent(Transient? entdad, Transient? entson)
    {
        int ndad = StartingNumber(entdad);
        int nson = StartingNumber(entson);
        if (ndad < 1 || nson < 1)
        {
            return -1;
        }
        if (ndad == nson)
        {
            return 0;
        }
        //  we will calculate: for each immediate parent, of <son>, status with <dad> + 1
        //  nb: not protected against loops ...
        NCollection_HSequence<Transient> list =
            thegraph.Graph().Sharings(entson).Content();
        if (list is null)
        {
            return -1;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            if (list.Value(i) == entdad)
            {
                return 1;
            }
            int stat = QueryParent(entdad, list.Value(i));
            if (stat >= 0)
            {
                return stat + 1;
            }
        }
        return -1; // not yet implemented ...
    }

    /// <summary>
    /// Sets a list of Parameters, i.e. TypedValue, to be handled through an Editor The two lists are parallel,
    /// if &lt;params&gt; is longer than &lt;uses&gt;, surnumeral parameters are for general use EditForms are
    /// created to handle these parameters (list, edit) on the basis of a ParamEditor xst-params-edit A use
    /// number dispatches the parameter to a given EditForm EditForms are defined as follows Name Use Means
    /// xst-params all All Parameters (complete list) xst-params-general 1 Generals xst-params-load 2 LoadFile
    /// (no Transfer) xst-params-send 3 SendFile (Write, no Transfer) xst-params-split 4 Split xst-param-read 5
    /// Transfer on Reading xst-param-write 6 Transfer on Writing
    /// </summary>
    public void SetParams(NCollection_DynamicArray<Transient> @params, NCollection_DynamicArray<int> uselist)
    {
        int i, nbp = @params.Length(), nbu = uselist.Length();
        IFSelect_ParamEditor editor =
            new IFSelect_ParamEditor(nbp + nbu + 50, "Parameter Editor");
        for (i = @params.Lower(); i <= @params.Upper(); i++)
        {
            Interface_TypedValue val = @params.Value(i) as Interface_TypedValue;
            if (val is null)
            {
                continue;
            }
            editor.AddValue(val);
        }
        AddNamedItem("xst-params-edit", editor);
        //  The EditForm
        IFSelect_EditForm paramsall = editor.Form(false);
        AddNamedItem("xst-params-all", paramsall);

        //  We tackle the partial EditForms
        NCollection_Sequence<int> listgen = new NCollection_Sequence<int>(), listload = new NCollection_Sequence<int>(),
            listsend = new NCollection_Sequence<int>(), listsplit = new NCollection_Sequence<int>(),
            listread = new NCollection_Sequence<int>(), listwrite = new NCollection_Sequence<int>();
        for (i = uselist.Lower(); i <= uselist.Upper(); i++)
        {
            int use = uselist.Value(i);
            switch (use)
            {
                case 1:
                    listgen.Append(i);
                    break;
                case 2:
                    listread.Append(i);
                    break;
                case 3:
                    listsend.Append(i);
                    break;
                case 4:
                    listsplit.Append(i);
                    break;
                case 5:
                    listread.Append(i);
                    break;
                case 6:
                    listwrite.Append(i);
                    break;
                default:
                    break;
            }
        }
        IFSelect_EditForm paramsgen =
            new IFSelect_EditForm(editor, listgen, false, true, "General Parameters");
        if (listgen.Length() > 0)
        {
            AddNamedItem("xst-params-general", paramsgen);
        }
        IFSelect_EditForm paramsload =
            new IFSelect_EditForm(editor, listload, false, true, "Loading Parameters");
        if (listload.Length() > 0)
        {
            AddNamedItem("xst-params-load", paramsload);
        }
        IFSelect_EditForm paramssend =
            new IFSelect_EditForm(editor, listsend, false, true, "Sending Parameters");
        if (listsend.Length() > 0)
        {
            AddNamedItem("xst-params-send", paramssend);
        }
        IFSelect_EditForm paramsplit =
            new IFSelect_EditForm(editor, listsplit, false, true, "Split Parameters");
        if (listsplit.Length() > 0)
        {
            AddNamedItem("xst-params-split", paramsplit);
        }
        IFSelect_EditForm paramsread =
            new IFSelect_EditForm(editor, listread, false, true, "Read(Transfer) Parameters");
        if (listread.Length() > 0)
        {
            AddNamedItem("xst-params-read", paramsread);
        }
        IFSelect_EditForm paramswrite =
            new IFSelect_EditForm(editor, listwrite, false, true, "Write(Transfer) Parameters");
        if (listwrite.Length() > 0)
        {
            AddNamedItem("xst-params-write", paramswrite);
        }
    }

    /// <summary>
    /// Traces the Statics attached to a given use number If &lt;use&gt; is given positive (normal), the trace is
    /// embedded with a header and a trailer If &lt;use&gt; is negative, just values are printed (this allows to
    /// make compositions) Remark : use number 5 commands use -2 to be traced Remark : use numbers 4 and 6
    /// command use -3 to be traced
    /// </summary>
    public void TraceStatics(int use, int mode = 0)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        if (use > 0)
        {
            if (mode == 0)
            {
                sout.Append("******************************************");
                sout.Append('\n');
            }
            if (use == 1)
            {
                if (mode == 0)
                {
                    sout.Append("*****      General  Parameters       *****");
                    sout.Append('\n');
                }
            }
            else if (use == 2)
            {
                if (mode == 0)
                {
                    sout.Append("*****            Load  File          *****");
                    sout.Append('\n');
                }
            }
            else if (use == 3)
            {
                if (mode == 0)
                {
                    sout.Append("*****            Write File          *****");
                    sout.Append('\n');
                }
            }
            else if (use == 4)
            {
                if (mode == 0)
                {
                    sout.Append("*****            Split File          *****");
                    sout.Append('\n');
                }
            }
            else if (use == 5)
            {
                if (mode == 0)
                {
                    sout.Append("*****        Transfer (Read)         *****");
                    sout.Append('\n');
                }
            }
            else if (use == 6)
            {
                if (mode == 0)
                {
                    sout.Append("*****        Transfer (Write)        *****");
                    sout.Append('\n');
                }
            }
            if (mode == 0)
            {
                sout.Append("******************************************");
                sout.Append('\n');
                sout.Append('\n');
            }
        }

        //    Echainements particuliers (use > 0)
        if (use == 5)
        {
            TraceStatics(-2, mode);
            if (mode == 0)
            {
                sout.Append('\n');
            }
        }
        else if (use == 4 || use == 6)
        {
            TraceStatics(-3, mode);
            if (mode == 0)
            {
                sout.Append('\n');
            }
        }

        //    Valeurs particulieres
        if (use == 1 || use == -1)
        { // General : trace
            if (mode == 0)
            {
                //      sout << "Trace Level   :
                //      "<<Message_PrinterOStream.Default().GetTraceLevel()<<std.endl;
            }
        }
        else if (use == 4 || use == -4)
        { // Split : Prefix & cie
            if (mode == 0)
            {
                TCollection_HAsciiString str = theshareout.Prefix();
                if (str is not null)
                {
                    sout.Append("Prefix        : ");
                    sout.Append(str);
                    sout.Append('\n');
                }
                else
                {
                    sout.Append("Prefix       not Defined");
                    sout.Append('\n');
                }
                str = theshareout.DefaultRootName();
                if (str is not null)
                {
                    sout.Append("Default Root  : ");
                    sout.Append(str);
                    sout.Append('\n');
                }
                else
                {
                    sout.Append("Default Root not Defined");
                    sout.Append('\n');
                }
                str = theshareout.Extension();
                if (str is not null)
                {
                    sout.Append("Extension     : ");
                    sout.Append(str);
                    sout.Append('\n');
                }
                else
                {
                    sout.Append("Extension    not defined");
                    sout.Append('\n');
                }
            }
        }

        //  LIST THE STATICS
        //  Go through the ParamEditor ...

        //    Fin
        if (use > 0)
        {
            if (mode == 0)
            {
                sout.Append("******************************************");
                sout.Append('\n');
                sout.Append('\n');
            }
        }
    }

    /// <summary>
    /// Lists the content of the Input Model (if there is one) According level : 0 -&gt; gives only count of
    /// Entities and Roots 1 -&gt; Lists also Roots; 2 -&gt; Lists all Entities (by TraceType) 3 -&gt; Performs a
    /// call to CheckList (Fails) and lists the result 4 -&gt; as 3 but all CheckList (Fails + Warnings) 5,6,7 :
    /// as 3 but resp. Count,List,Labels by Fail 8,9,10 : as 4 but resp. Count,List,Labels by message
    /// </summary>
    public void DumpModel(int level, TextWriter S)
    {
        if (!IsLoaded())
        {
            S.Write(" ***  Data for List not available  ***");
            S.Write('\n');
            return;
        }
        S.Write("\n        *****************************************************************\n");
        if (theloaded.Length > 0)
        {
            S.Write("        ********  Loaded File : ");
            S.Write(theloaded);
            S.Write(Interface_MSG.Blanks(32 - theloaded.Length));
            S.Write(" ********");
            S.Write('\n');
        }
        else
        {
            S.Write("        ********  No name for Loaded File");
            S.Write('\n');
        }
        if (level == 0)
        {
            S.Write("        ********  Short Dump of Header                           ********\n");
            S.Write("        *****************************************************************\n\n");
            myModel.DumpHeader(S);
            S.Write('\n');
        }

        int nbent = myModel.NbEntities();
        int nbr = 0;
        Interface_ShareFlags shar = new Interface_ShareFlags(thegraph.Graph());

        for (int i = 1; i <= nbent; i++)
        {
            if (!shar.IsShared(myModel.Value(i)))
            {
                nbr++;
            }
        }
        S.Write("        *****************************************************************\n");
        S.Write("        ********  Model : ");
        S.Write(nbent);
        S.Write(" Entities, of which ");
        S.Write(nbr);
        S.Write(" Root(s)\n");
        S.Write("        *****************************************************************\n");
        S.Write('\n');

        if (level <= 0)
        {
            return;
        }
        else if (level == 1)
        {
            S.Write("        ********  Root Entities  ********      ");
            ListEntities(shar.RootEntities(), 1, S);
        }
        else if (level == 2)
        {
            S.Write("        ********  Complete List  ********      ");
            ListEntities(myModel.Entities(), 1, S);
        }
        else if (level > 2)
        {
            IFSelect_PrintCount mode = IFSelect_PrintCount.IFSelect_ItemsByEntity;
            if (level == 5 || level == 8)
            {
                mode = IFSelect_PrintCount.IFSelect_CountByItem;
            }
            if (level == 6 || level == 9)
            {
                mode = IFSelect_PrintCount.IFSelect_ListByItem;
            }
            if (level == 7 || level == 10)
            {
                mode = IFSelect_PrintCount.IFSelect_EntitiesByItem;
            }
            PrintCheckList(S, ModelCheckList(), false, mode);
        }
        else
        {
            if (level == 3)
            {
                S.Write("        ********  Check Model (Fails)  ********");
                S.Write('\n');
            }
            else
            {
                S.Write("        ********  Check Model (Complete)  ********");
                S.Write('\n');
            }
            Interface_CheckTool CT = new Interface_CheckTool(Graph());
            Interface_CheckIterator C = new Interface_CheckIterator();
            if (theerrhand)
            {
                try
                {
                    if (level == 3)
                    {
                        C = CT.CheckList();
                    }
                    else
                    {
                        C = CT.CompleteCheckList();
                    }
                }
                catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
                {
                    using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                    sout.Append("    ****    DumpModel (Check) Interrupted by Exception    ****\n");
                    S.Write("  ** **  Exception Raised during Check !  ** **\n");
                    S.Write("  -->  what could be determined is listed");
                    S.Write('\n');
                }
            }
            else if (level == 3)
            {
                C = CT.CheckList();
            }
            else
            {
                C = CT.CompleteCheckList();
            }

            //  Check List: if empty (not requested), naturally passed
            try
            {
                C.Print(S, myModel, (level == 3));
            }
            catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
            {
                using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
                sout.Append("    ****    DumpModel Interrupted by Exception :   ****\n");
                sout.Append(Transfer_ProcessForTransient.What(anException));
                sout.Append("\n    Abandon");
                sout.Append('\n');
            }
        }
        S.Write('\n');
        S.Write("There are ");
        S.Write(nbent);
        S.Write(" Entities, of which ");
        S.Write(nbr);
        S.Write(" Root(s)");
        S.Write('\n');
    }

    /// <summary>
    /// Dumps the current Model (as inherited DumpModel), on currently defined Default Trace File (default is
    /// standard output)
    /// </summary>
    public void TraceDumpModel(int mode)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo(); // should it be changed to SendTrace()?
        StringWriter aText = new StringWriter();
        DumpModel(mode, aText);
        sout.Append(aText.ToString());
        //  if      (mode <= 4)  {  DumpModel (mode,sout);  return;  }

        //  else if (mode <= 7) PrintCheckList (ModelCheckList(),false, mode-5);
        //  else if (mode <=10) PrintCheckList (ModelCheckList(),true , mode-8);
    }

    /// <summary>
    /// Dumps a starting entity according to the current norm. To do this, it calls DumpEntity from WorkLibrary.
    /// &lt;level&gt; is to be interpreted for each norm : see specific classes of WorkLibrary for it. Generally,
    /// 0 if for very basic (only type ...), greater values give more and more details.
    /// </summary>
    public void DumpEntity(Transient? ent, int level, TextWriter S)
    {
        if (!IsLoaded())
        {
            S.Write(" ***  Data for List not available  ***");
            S.Write('\n');
            return;
        }
        int num = myModel.Number(ent);
        if (num == 0)
        {
            S.Write(" ***  Entity to Dump not in the Model  ***");
            S.Write('\n');
            return;
        }
        if (thelibrary is null)
        {
            S.Write(" ***  WorkLibrary not defined  ***");
            S.Write('\n');
            return;
        }
        S.Write("        ********  Dumping Entity n0 ");
        S.Write(num);
        S.Write(" level:");
        S.Write(level);
        S.Write("  ********");
        S.Write('\n');
        thelibrary.DumpEntity(myModel, theprotocol, ent, S, level);
    }

    /// <summary>
    /// Prints main information about an entity : its number, type, validity (and checks if any), category,
    /// shareds and sharings.. mutable because it can recompute checks as necessary
    /// </summary>
    public void PrintEntityStatus(Transient? ent, TextWriter S)
    {
        int i, nb;
        int num = StartingNumber(ent);
        if (num == 0)
        {
            Console.Out.Write(" --  PrintEntityStatus : unknown");
            Console.Out.Write('\n');
            return;
        }

        S.Write("  Ent. n0/id:   ");
        myModel.Print(ent, S);
        TCollection_HAsciiString hname = EntityName(ent);
        if (hname is not null && hname.Length() > 0)
        {
            S.Write("\tName:");
            S.Write(hname.ToCString());
        }
        S.Write('\n');
        IFSelect_Signature signtype = SignType();
        if (signtype is null)
        {
            S.Write("  Type(CDL):");
            S.Write(ent.DynamicTypeName());
            S.Write('\n');
        }
        else
        {
            S.Write("  Type:");
            S.Write(signtype.Value(ent, myModel));
            S.Write('\n');
        }
        S.Write("    Category : ");
        S.Write(CategoryName(ent));
        S.Write("    Validity : ");
        S.Write(ValidityName(ent));
        S.Write('\n');
        Interface_CheckIterator chl = CheckOne(ent);
        chl.Print(S, myModel, false, 0); // OCCT: Print(S, model, false, false), the last argument an integer

        NCollection_HSequence<Transient> list = Sharings(ent);
        if (list is null)
        {
            S.Write("  Root");
            S.Write('\n');
        }
        else
        {
            nb = list.Length();
            if (nb == 0)
            {
                S.Write("  Root");
            }
            else
            {
                S.Write("  Super-entities:");
                S.Write(nb);
                S.Write(" : (n0/id):");
            }
            for (i = 1; i <= nb; i++)
            {
                S.Write(" ");
                myModel.Print(list.Value(i), S);
            }
            S.Write('\n');
        }
        list = Shareds(ent);
        if (list is null)
        {
            S.Write("  No sub-entity");
            S.Write('\n');
        }
        else
        {
            nb = list.Length();
            if (nb == 0)
            {
                S.Write("  No sub-entity");
            }
            else
            {
                S.Write("  Sub-entities:");
                S.Write(nb);
                S.Write(" , i.e. (n0/id):");
            }
            for (i = 1; i <= nb; i++)
            {
                S.Write(" ");
                myModel.Print(list.Value(i), S);
            }
            S.Write('\n');
        }
    }

    /// <summary>
    /// Dumps an entity from the current Model as inherited DumpEntity on currently defined Default Trace File
    /// (&lt;level&gt; interpreted according to the Norm, see WorkLibrary)
    /// </summary>
    public void TraceDumpEntity(Transient? ent, int level)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo(); // should it be changed to SendTrace()?
        StringWriter aText = new StringWriter();
        DumpEntity(ent, level, aText);
        sout.Append(aText.ToString());
    }

    /// <summary>
    /// Prints a CheckIterator to the current Trace File, controlled with the current Model complete or fails
    /// only, according to &lt;failsonly&gt; &lt;mode&gt; defines the mode of printing 0 : sequential, according
    /// entities; else with a CheckCounter 1 : according messages, count of entities 2 : id but with list of
    /// entities, designated by their numbers 3 : as 2 but with labels of entities
    /// </summary>
    public void PrintCheckList(TextWriter S, Interface_CheckIterator checklist, bool failsonly, IFSelect_PrintCount mode)
    {
        //  mode : 0  comptage   1 n0s entites   2 n0s+id ents
        if (mode == IFSelect_PrintCount.IFSelect_ItemsByEntity)
        {
            checklist.Print(S, myModel, failsonly);
        }
        else
        {
            Interface_CheckIterator chks = checklist;
            IFSelect_CheckCounter counter =
                new IFSelect_CheckCounter((int)mode > 1 && mode != IFSelect_PrintCount.IFSelect_CountSummary);
            counter.Analyse(chks, myModel, true, failsonly);
            counter.PrintList(S, myModel, mode);
        }
    }

    /// <summary>
    /// Prints a SignatureList to the current Trace File, controlled with the current Model &lt;mode&gt; defines
    /// the mode of printing (see SignatureList)
    /// </summary>
    public void PrintSignatureList(TextWriter S, IFSelect_SignatureList? signlist, IFSelect_PrintCount mode)
    {
        if (signlist is null)
        {
            return;
        }
        signlist.PrintList(S, myModel, mode);
    }

    /// <summary>
    /// Internal method which displays an EntityIterator &lt;mode&gt; 0 gives short display (only entity numbers)
    /// 1 gives a more complete trace (1 line per Entity) (can be used each time a trace has to be output from a
    /// list) 2 gives a form suitable for givelist : (n1,n2,n3...)
    /// </summary>
    public void ListEntities(Interface_EntityIterator iter, int mmode, TextWriter sout)
    {
        int titre = 0;
        int mode = (mmode < 0 ? -mmode : mmode);
        if (mmode >= 0)
        {
            sout.Write(" List of ");
            sout.Write(iter.NbEntities());
            sout.Write(" Entities :");
            sout.Write('\n');
        }
        if (!IsLoaded())
        {
            sout.Write(" ***  Data for List not available  ***");
            sout.Write('\n');
            return;
        }
        Interface_ShareFlags tool = new Interface_ShareFlags(thegraph.Graph());

        try
        {
            int newcount = -1;
            int mods = 0;
            int cnt = 0;
            for (iter.Start(); iter.More(); iter.Next())
            {
                if (titre == 0 && mode == 1)
                {
                    sout.Write("Number/Id.           Category Validity    Type\n-----------          ----....");
                    sout.Write('\n');
                }
                //          123456789 123456789 123456  123456789 123456789 123456
                if (titre == 0 && mode == 0)
                {
                    sout.Write("  Keys : R Root   ? Unknown   * Unloaded");
                    sout.Write('\n');
                }
                if (titre == 0 && mode == 2)
                {
                    sout.Write("(");
                }
                titre = 1;
                Transient ent = iter.Value();
                int num = myModel.Number(ent);
                if (mode == 1)
                {
                    // n0 id (root?) category validity tracetype
                    sout.Write(Interface_MSG.Blanks(num, 6));
                    myModel.Print(ent, sout, 0);
                    if (!tool.IsShared(ent))
                    {
                        sout.Write(" #ROOT#");
                    }
                    else
                    {
                        sout.Write("       ");
                    }
                    int catnum = myModel.CategoryNumber(num);
                    if (catnum > 0)
                    {
                        sout.Write("  ");
                        sout.Write(Interface_Category.Name(catnum));
                    }
                    sout.Write("  (");
                    sout.Write(ValidityName(ent));
                    sout.Write(")  ");

                    sout.Write(" Type:");
                    sout.Write(myModel.TypeName(ent, false));
                    sout.Write('\n');
                }
                else if (mode == 2)
                {
                    newcount++;
                    if (newcount > 0)
                    {
                        sout.Write(",");
                    }
                    sout.Write(num);
                }
                else
                {
                    newcount++;
                    mods = 0;
                    cnt++;
                    if (newcount >= 10)
                    {
                        sout.Write('\n');
                        sout.Write("[");
                        sout.Write(cnt);
                        sout.Write("]:");
                        newcount = 1;
                    }
                    if (newcount > 0)
                    {
                        sout.Write("\t");
                    }
                    myModel.Print(ent, sout, 0);
                    if (!tool.IsShared(ent))
                    {
                        if (mods == 0)
                        {
                            sout.Write("(");
                        }
                        sout.Write("R");
                        mods++;
                    }
                    if (myModel.IsUnknownEntity(num))
                    {
                        sout.Write((mods == 0 ? '(' : ' '));
                        sout.Write("?");
                        mods++;
                    }
                    if (myModel.IsRedefinedContent(num))
                    {
                        sout.Write((mods == 0 ? '(' : ' '));
                        sout.Write("*");
                        mods++;
                    }
                    if (mods != 0)
                    {
                        sout.Write(")");
                        newcount++;
                    }
                }
            }
            if (mode == 0)
            {
                sout.Write('\n');
            }
            if (mode == 2)
            {
                sout.Write(")");
                sout.Write('\n');
            }
        }
        catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
        {
            sout.Write("    ****    ListEntities Interrupted by Exception :   ****\n");
            sout.Write(Transfer_ProcessForTransient.What(anException));
            sout.Write("\n    Abandon");
            sout.Write('\n');
        }
    }

    protected Interface_HGraph? thegraph;
    protected Interface_CheckIterator thecheckrun = new Interface_CheckIterator();
    protected NCollection_IndexedDataMap<Transient, Transient> theitems = new NCollection_IndexedDataMap<Transient, Transient>();
    protected NCollection_DataMap<string, Transient> thenames = new NCollection_DataMap<string, Transient>();
    private bool theerrhand;
    private bool myErrHand; // OCCT's file static "errhand", the session's own here
    private Interface_StaticSet thestatics;
    private IFSelect_ShareOut? theshareout;
    private IFSelect_WorkLibrary? thelibrary;
    private Interface_Protocol? theprotocol;
    private Interface_InterfaceModel? myModel;
    private string theloaded = "";
    private Interface_GTool? thegtool;
    private bool thecheckdone;
    private Interface_CheckIterator thechecklist = new Interface_CheckIterator(); // unused (OCCT)
    private string thecheckana = "";
    private IFSelect_ModelCopier? thecopier;
    // theoldel (the model before SetRemaining or SetModelContent) is left out with them (IFSelect's cut)
    private bool themodelstat;
}
