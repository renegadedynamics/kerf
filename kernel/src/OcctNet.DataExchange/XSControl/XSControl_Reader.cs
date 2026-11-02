// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Reader.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// A groundwork to convert a shape to data which complies with a particular norm. This data can be that of a
/// whole model or that of a specific list of entities in the model. You specify the list using a single
/// selection or a combination of selections. A selection is an operator which computes a list of entities
/// from a list given in input. To specify the input, you can use: - A predefined selection such as
/// "xst-transferrable-roots" - A filter based on a signature. A signature is an operator which returns a
/// string from an entity according to its type. For example: - "xst-type" (CDL) - "iges-level" -
/// "step-type". A filter can be based on a signature by giving a value to be matched by the string returned.
/// For example, "xst-type(Curve)". If no list is specified, the selection computes its list of entities from
/// the whole model. To use this class, you have to initialize the transfer norm first, as shown in the
/// example below. Example: Control_Reader reader; IFSelect_ReturnStatus status = reader.ReadFile
/// (filename.); When using IGESControl_Reader or STEPControl_Reader - as the above example shows - the
/// reader initializes the norm directly. Note that loading the file only stores the data. It does not
/// translate this data. Shapes are accumulated by successive transfers. The last shape is cleared by: -
/// ClearShapes which allows you to handle a new batch - TransferRoots which restarts the list of shapes from
/// scratch.
/// </summary>
public class XSControl_Reader
{

    /// <summary>
    /// Creates a Reader from scratch (creates an empty WorkSession) A WorkSession or a Controller must be
    /// provided before running
    /// </summary>
    public XSControl_Reader()
    {
        SetWS(new XSControl_WorkSession());
    }

    /// <summary>
    /// Creates a Reader from scratch, with a norm name which identifies a Controller
    /// </summary>
    public XSControl_Reader(string norm)
    {
        SetNorm(norm);
    }

    /// <summary>
    /// Creates a Reader from an already existing Session, with a Controller already set Virtual destructor
    /// </summary>
    public XSControl_Reader(XSControl_WorkSession? WS, bool scratch = true)
    {
        SetWS(WS, scratch);
    }

    /// <summary>
    /// Sets a specific norm to &lt;me&gt; Returns True if done, False if &lt;norm&gt; is not available
    /// </summary>
    public bool SetNorm(string norm)
    {
        if (thesession is null)
        {
            SetWS(new XSControl_WorkSession());
        }
        bool stat = thesession.SelectNorm(norm);
        if (stat)
        {
            thesession.InitTransferReader(0);
            thesession.InitTransferReader(4);
        }
        return stat;
    }

    /// <summary>
    /// Sets a specific session to &lt;me&gt;
    /// </summary>
    public void SetWS(XSControl_WorkSession? WS, bool scratch = true)
    {
        therootsta = false;
        theroots.Clear();
        thesession = WS;
        //  There must be a Controller ...  Otherwise we'll see later (after SetNorm)
        if (thesession.NormAdaptor() is null)
        {
            return;
        }
        Interface_InterfaceModel model = thesession.Model();
        if (scratch || model is null)
        {
            model = thesession.NewModel();
        }
        thesession.InitTransferReader(0);
        thesession.InitTransferReader(4);
    }

    /// <summary>
    /// Returns the session used in &lt;me&gt;
    /// </summary>
    public XSControl_WorkSession? WS()
    {
        return thesession;
    }

    /// <summary>
    /// Loads a file and returns the read status Zero for a Model which complies with the Controller
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="theProgress"/> (not in OCCT) is given to the session (plan 2.7).</remarks>
    public virtual IFSelect_ReturnStatus ReadFile(string filename, Message_ProgressRange theProgress = default)
    {
        IFSelect_ReturnStatus stat = thesession.ReadFile(filename, theProgress);
        thesession.InitTransferReader(4);
        return stat;
    }

    /// <summary>
    /// Loads a file from stream and returns the read status
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the stream is a .NET <see cref="Stream"/>; <paramref name="theProgress"/> (not in OCCT) is given to
    /// the session (plan 2.7).
    /// </remarks>
    public virtual IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream, Message_ProgressRange theProgress = default)
    {
        IFSelect_ReturnStatus stat = thesession.ReadStream(theName, theIStream, theProgress);
        thesession.InitTransferReader(4);
        return stat;
    }

    /// <summary>
    /// Returns the model. It can then be consulted (header, product)
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return thesession.Model();
    }

    /// <summary>
    /// Returns a list of entities from the IGES or STEP file according to the following rules: - if first and
    /// second are empty strings, the whole file is selected. - if first is an entity number or label, the entity
    /// referred to is selected. - if first is a list of entity numbers/labels separated by commas, the entities
    /// referred to are selected, - if first is the name of a selection in the worksession and second is not
    /// defined, the list contains the standard output for that selection. - if first is the name of a selection
    /// and second is defined, the criterion defined by second is applied to the result of the first selection. A
    /// selection is an operator which computes a list of entities from a list given in input according to its
    /// type. If no list is specified, the selection computes its list of entities from the whole model. A
    /// selection can be: - A predefined selection (xst-transferrable-mode) - A filter based on a signature A
    /// Signature is an operator which returns a string from an entity according to its type. For example: -
    /// "xst-type" (CDL) - "iges-level" - "step-type". For example, if you wanted to select only the
    /// advanced_faces in a STEP file you would use the following code: Example
    /// Reader.GiveList("xst-transferrable-roots","step-type(ADVANCED_FACE)"); Warning If the value given to
    /// second is incorrect, it will simply be ignored.
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? GiveList(string first = "", string second = "")
    {
        if (first is not null && CStringChar(first, 0) != '\0')
        {
            return thesession.GiveList(first, second);
        }

        NCollection_HSequence<Standard_Transient> list =
            new NCollection_HSequence<Standard_Transient>();
        int i, nbr = NbRootsForTransfer();
        for (i = 1; i <= nbr; i++)
        {
            list.Append(RootForTransfer(i));
        }
        return list;
    }

    /// <summary>
    /// Computes a List of entities from the model as follows &lt;first&gt; being a Selection, &lt;ent&gt; being
    /// an entity or a list of entities (as a HSequenceOfTransient) : the standard result of this selection
    /// applied to this list if &lt;first&gt; is erroneous, a null handle is returned
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? GiveList(string first, Standard_Transient? list)
    {
        return thesession.GiveListFromList(first, list);
    }


    /// <summary>
    /// Determines the list of root entities which are candidate for a transfer to a Shape, and returns the
    /// number of entities in the list
    /// </summary>
    public virtual int NbRootsForTransfer()
    {
        if (therootsta)
        {
            return theroots.Length();
        }
        therootsta = true;
        Interface_ShareFlags sf = new Interface_ShareFlags(thesession.Graph());
        int i, nbr = sf.NbRoots();
        for (i = 1; i <= nbr; i++)
        {
            //    on filtre les racines qu on sait transferer
            Standard_Transient start = sf.Root(i);
            if (thesession.TransferReader().Recognize(start))
            {
                theroots.Append(start);
            }
        }
        return theroots.Length();
    }

    /// <summary>
    /// Returns an IGES or STEP root entity for translation. The entity is identified by its rank in a list.
    /// </summary>
    public Standard_Transient? RootForTransfer(int num = 1)
    {
        Standard_Transient voidroot = null;
        int nbr = NbRootsForTransfer();
        if (num < 1 || num > nbr)
        {
            return voidroot;
        }
        return theroots.Value(num);
    }

    /// <summary>
    /// Translates a root identified by the rank num in the model. false is returned if no shape is produced.
    /// </summary>
    public bool TransferOneRoot(int num = 1, Message_ProgressRange theProgress = default)
    {
        return TransferEntity(RootForTransfer(num), theProgress);
    }

    /// <summary>
    /// Translates an IGES or STEP entity identified by the rank num in the model. false is returned if no shape
    /// is produced.
    /// </summary>
    public bool TransferOne(int num, Message_ProgressRange theProgress = default)
    {
        return TransferEntity(thesession.StartingEntity(num), theProgress);
    }

    /// <summary>
    /// Translates an IGES or STEP entity in the model. true is returned if a shape is produced; otherwise, false
    /// is returned.
    /// </summary>
    public bool TransferEntity(Standard_Transient? theStart, Message_ProgressRange theProgress = default)
    {
        if (theStart is null)
        {
            return false;
        }

        XSControl_TransferReader aTransferReader = thesession.TransferReader();
        aTransferReader.BeginTransfer();
        InitializeMissingParameters();
        if (aTransferReader.TransferOne(theStart, true, theProgress) == 0)
        {
            return false;
        }

        TopoDS_Shape aShape = aTransferReader.ShapeResult(theStart);
        // Null shapes are allowed intentionally.
        aShape.TShape()?.FreezeGraph(); // OCCT.NET: the reader publishes frozen shapes (plan 2.7)
        // SMH May 00: allow empty shapes (STEP CAX-IF, external references)
        theshapes.Append(aShape);
        return true;
    }

    /// <summary>
    /// Translates a list of entities. Returns the number of IGES or STEP entities that were successfully
    /// translated. The list can be produced with GiveList. Warning - This function does not clear the existing
    /// output shapes.
    /// </summary>
    public int TransferList(NCollection_HSequence<Standard_Transient>? theList, Message_ProgressRange theProgress = default)
    {
        if (theList is null)
        {
            return 0;
        }

        int aTransferredCount = 0;
        XSControl_TransferReader aTransferReader = thesession.TransferReader();
        aTransferReader.BeginTransfer();
        InitializeMissingParameters();
        ClearShapes();
        Message_ProgressScope aProgressScope = new Message_ProgressScope(theProgress, null, theList.Size());
        for (int i = 1; i <= theList.Size() && aProgressScope.More(); i++)
        {
            Standard_Transient aStart = theList.Value(i);
            if (aTransferReader.TransferOne(aStart, true, aProgressScope.Next()) == 0)
            {
                continue;
            }

            TopoDS_Shape aShape = aTransferReader.ShapeResult(aStart);
            // Null shapes are allowed intentionally.
            aShape.TShape()?.FreezeGraph(); // OCCT.NET: the reader publishes frozen shapes (plan 2.7)
            // SMH May 00: allow empty shapes (STEP CAX-IF, external references)
            theshapes.Append(aShape);
            ++aTransferredCount;
        }
        return aTransferredCount;
    }

    /// <summary>
    /// Translates all translatable roots and returns the number of successful translations. Warning - This
    /// function clears existing output shapes first.
    /// </summary>
    public int TransferRoots(Message_ProgressRange theProgress = default)
    {
        NbRootsForTransfer();

        int aTransferredCount = 0;
        XSControl_TransferReader aTransferReader = thesession.TransferReader();
        aTransferReader.BeginTransfer();
        InitializeMissingParameters();
        ClearShapes();
        Message_ProgressScope aProgressScope = new Message_ProgressScope(theProgress, "Root", theroots.Size());
        for (int i = 1; i <= theroots.Size() && aProgressScope.More(); i++)
        {
            Standard_Transient aStart = theroots.Value(i);
            if (aTransferReader.TransferOne(aStart, true, aProgressScope.Next()) == 0)
            {
                continue;
            }

            TopoDS_Shape aShape = aTransferReader.ShapeResult(aStart);
            // Null shapes are allowed intentionally.
            aShape.TShape()?.FreezeGraph(); // OCCT.NET: the reader publishes frozen shapes (plan 2.7)
            // SMH May 00: allow empty shapes (STEP CAX-IF, external references)
            theshapes.Append(aShape);
            ++aTransferredCount;
        }
        return aTransferredCount;
    }

    /// <summary>
    /// Clears the list of shapes that may have accumulated in calls to TransferOne or TransferRoot.C
    /// </summary>
    public void ClearShapes()
    {
        theshapes.Clear();
    }

    /// <summary>
    /// Returns the number of shapes produced by translation.
    /// </summary>
    public int NbShapes()
    {
        return theshapes.Length();
    }

    /// <summary>
    /// Returns the shape resulting from a translation and identified by the rank num. num equals 1 by default.
    /// In other words, the first shape resulting from the translation is returned.
    /// </summary>
    public TopoDS_Shape Shape(int num = 1)
    {
        return theshapes.Value(num);
    }

    /// <summary>
    /// Returns all of the results in a single shape which is: - a null shape if there are no results, - a shape
    /// if there is one result, - a compound containing the resulting shapes if there are more than one.
    /// </summary>
    public TopoDS_Shape OneShape()
    {
        if (theshapes.IsEmpty())
        {
            return default;
        }
        if (theshapes.Length() == 1)
        {
            return theshapes.First();
        }

        BRep_Builder aBuilder = new BRep_Builder();
        TopoDS_Compound aResult = new TopoDS_Compound();
        aBuilder.MakeCompound(ref aResult);
        bool aNonNullShapeAdded = false;
        foreach (TopoDS_Shape aCurrShape in theshapes)
        {
            if (!aCurrShape.IsNull())
            {
                aBuilder.Add(aResult, aCurrShape);
                aNonNullShapeAdded = true;
            }
        }
        if (!aNonNullShapeAdded)
        {
            return default;
        }
        aResult.TShape()?.FreezeGraph(); // OCCT.NET: published (the reader's result)
        return aResult;
    }

    /// <summary>
    /// Prints the check list attached to loaded data, on the Standard Trace File (starts at std::cout) All
    /// messages or fails only, according to &lt;failsonly&gt; mode = 0 : per entity, prints messages mode = 1 :
    /// per message, just gives count of entities per check mode = 2 : also gives entity numbers
    /// </summary>
    public void PrintCheckLoad(bool failsonly, IFSelect_PrintCount mode)
    {
        using Message_Messenger.StreamBuffer aBuffer = Message.SendInfo();
        StringWriter aText = new StringWriter();
        PrintCheckLoad(aText, failsonly, mode);
        aBuffer.Append(aText.ToString());
    }

    /// <summary>
    /// Prints the check list attached to loaded data.
    /// </summary>
    public void PrintCheckLoad(TextWriter theStream, bool failsonly, IFSelect_PrintCount mode)
    {
        thesession.PrintCheckList(theStream, thesession.ModelCheckList(), failsonly, mode);
    }

    /// <summary>
    /// Displays check results for the last translation of IGES or STEP entities to Open CASCADE entities. Only
    /// fail messages are displayed if failsonly is true. All messages are displayed if failsonly is false. mode
    /// determines the contents and the order of the messages according to the terms of the IFSelect_PrintCount
    /// enumeration.
    /// </summary>
    public void PrintCheckTransfer(bool failsonly, IFSelect_PrintCount mode)
    {
        using Message_Messenger.StreamBuffer aBuffer = Message.SendInfo();
        StringWriter aText = new StringWriter();
        PrintCheckTransfer(aText, failsonly, mode);
        aBuffer.Append(aText.ToString());
    }

    /// <summary>
    /// Displays check results for the last translation of IGES or STEP entities to Open CASCADE entities.
    /// </summary>
    public void PrintCheckTransfer(TextWriter theStream, bool failsonly, IFSelect_PrintCount mode)
    {
        thesession.PrintCheckList(theStream, thesession.TransferReader().LastCheckList(), failsonly, mode);
    }

    /// <summary>
    /// Displays the statistics for the last translation. what defines the kind of statistics that are displayed
    /// as follows: - 0 gives general statistics (number of translated roots, number of warnings, number of fail
    /// messages), - 1 gives root results, - 2 gives statistics for all checked entities, - 3 gives the list of
    /// translated entities, - 4 gives warning and fail messages, - 5 gives fail messages only. The use of mode
    /// depends on the value of what. If what is 0, mode is ignored. If what is 1, 2 or 3, mode defines the
    /// following: - 0 lists the numbers of IGES or STEP entities in the respective model - 1 gives the number,
    /// identifier, type and result type for each IGES or STEP entity and/or its status (fail, warning, etc.) - 2
    /// gives maximum information for each IGES or STEP entity (i.e. checks) - 3 gives the number of entities per
    /// type of IGES or STEP entity - 4 gives the number of IGES or STEP entities per result type and/or status -
    /// 5 gives the number of pairs (IGES or STEP or result type and status) - 6 gives the number of pairs (IGES
    /// or STEP or result type and status) AND the list of entity numbers in the IGES or STEP model. If what is 4
    /// or 5, mode defines the warning and fail messages as follows: - if mode is 0 all warnings and checks per
    /// entity are returned - if mode is 2 the list of entities per warning is returned. If mode is not set, only
    /// the list of all entities per warning is given.
    /// </summary>
    public void PrintStatsTransfer(int what, int mode = 0)
    {
        using Message_Messenger.StreamBuffer aBuffer = Message.SendInfo();
        StringWriter aText = new StringWriter();
        PrintStatsTransfer(aText, what, mode);
        aBuffer.Append(aText.ToString());
    }

    /// <summary>
    /// Displays the statistics for the last translation.
    /// </summary>
    public void PrintStatsTransfer(TextWriter theStream, int what, int mode = 0)
    {
        thesession.TransferReader().PrintStats(theStream, what, mode);
    }

    /// <summary>
    /// Gives statistics about Transfer
    /// </summary>
    public void GetStatsTransfer(NCollection_HSequence<Standard_Transient>? list, out int nbMapped, out int nbWithResult, out int nbWithFail)
    {
        Transfer_TransientProcess TP =
            thesession.TransferReader().TransientProcess();
        Transfer_IteratorOfProcessForTransient itrp = new Transfer_IteratorOfProcessForTransient(true);
        itrp = TP.CompleteResult(true);
        if (list is not null)
        {
            itrp.Filter(list);
        }
        nbMapped = nbWithFail = nbWithResult = 0;

        for (itrp.Start(); itrp.More(); itrp.Next())
        {
            Transfer_Binder binder = itrp.Value();
            nbMapped++;
            if (binder is null)
            {
                nbWithFail++;
            }
            else if (!binder.HasResult())
            {
                nbWithFail++;
            }
            else
            {
                Interface_CheckStatus cst = binder.Check().Status();
                if ((cst == Interface_CheckOK) || (cst == Interface_CheckWarning))
                {
                    nbWithResult++;
                }
                else
                {
                    nbWithFail++;
                }
            }
        }
    }

    /// <summary>
    /// Sets parameters for shape processing. @param theParameters the parameters for shape processing.
    /// </summary>
    /// <summary>
    /// Sets parameters for shape processing.
    /// </summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> theParameters)
    {
        Transfer_ActorOfTransientProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeFixParameters(theParameters);
        }
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from theParameters are copied to the internal map.
    /// Parameters from theAdditionalParameters are copied to the internal map if they are not present in
    /// theParameters.
    /// </summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters,
                                      NCollection_DataMap<string, string>? theAdditionalParameters = null)
    {
        Transfer_ActorOfTransientProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetShapeFixParameters(theParameters, theAdditionalParameters ?? new NCollection_DataMap<string, string>());
        }
    }

    /// <summary>
    /// Returns parameters for shape processing that was set by SetParameters() method. Returns the empty map if no
    /// parameters were set.
    /// </summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters()
    {
        Transfer_ActorOfTransientProcess anActor = GetActor();
        return anActor is null ? new NCollection_DataMap<string, string>() : anActor.GetShapeFixParameters();
    }

    /// <summary>
    /// Sets flags defining operations to be performed on shapes.
    /// </summary>
    public void SetShapeProcessFlags(ShapeProcess.OperationsFlags theFlags)
    {
        Transfer_ActorOfTransientProcess anActor = GetActor();
        if (anActor is not null)
        {
            anActor.SetProcessingFlags(theFlags);
        }
    }

    /// <summary>
    /// Returns flags defining operations to be performed on shapes, and whether they were set.
    /// </summary>
    public (ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        Transfer_ActorOfTransientProcess anActor = GetActor();
        return anActor is null ? default : anActor.GetProcessingFlags();
    }

    protected NCollection_Sequence<TopoDS_Shape> Shapes()
    {
        return theshapes;
    }

    /// <summary>
    /// Returns default parameters for shape fixing. This method should be implemented in the derived classes to
    /// return default parameters for shape fixing. @return Default parameters for shape fixing.
    /// </summary>
    protected virtual DE_ShapeFixParameters GetDefaultShapeFixParameters()
    {
        return new DE_ShapeFixParameters();
    }

    /// <summary>
    /// Returns default flags for shape processing. This method should be implemented in the derived classes to
    /// return default flags for shape processing. @return Default flags for shape processing.
    /// </summary>
    protected virtual ShapeProcess.OperationsFlags GetDefaultShapeProcessFlags()
    {
        return new ShapeProcess.OperationsFlags();
    }

    /// <summary>
    /// Returns the Actor for the Transfer of an Entity. This Actor is used by the Reader to perform the
    /// Transfer. @return the Actor for the Transfer of an Entity. May be nullptr.
    /// </summary>
    private Transfer_ActorOfTransientProcess? GetActor()
    {
        XSControl_WorkSession aSession = WS();
        if (aSession is null)
        {
            return null;
        }

        XSControl_TransferReader aReader = aSession.TransferReader();
        if (aReader is null)
        {
            return null;
        }

        return aReader.Actor();
    }

    /// <summary>
    /// If parameters haven't yet been provided, initializes them with default values provided by
    /// GetDefaultShapeFixParameters() method.
    /// </summary>
    private void InitializeMissingParameters()
    {
        if (GetShapeFixParameters().IsEmpty())
        {
            SetShapeFixParameters(GetDefaultShapeFixParameters());
        }
        if (!GetShapeProcessFlags().Item2)
        {
            SetShapeProcessFlags(GetDefaultShapeProcessFlags());
        }
    }

    protected bool therootsta;
    protected NCollection_Sequence<Standard_Transient> theroots = new NCollection_Sequence<Standard_Transient>();
    private XSControl_WorkSession? thesession;
    private NCollection_Sequence<TopoDS_Shape> theshapes = new NCollection_Sequence<TopoDS_Shape>();
}
