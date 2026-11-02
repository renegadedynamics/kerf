// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileReaderTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_FileReaderTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Message_Gravity;

namespace Occt;

/// <summary>
/// Defines services which are required to load an InterfaceModel from a File. Typically, it may firstly
/// transform a system file into a FileReaderData object, then work on it, not longer considering file
/// contents, to load an Interface Model. It may also work on a FileReaderData already loaded. FileReaderTool
/// provides, on one hand, some general services which are common to all read operations but can be
/// redefined, plus general actions to be performed specifically for each Norm, as deferred methods to
/// define. In particular, FileReaderTool defines the Interface's Unknown and Error entities
/// </summary>
public abstract class Interface_FileReaderTool
{

    // MGE 16/06/98
    // To use Msg class
    // To use TCollectionHAsciiString

    // Failure to recover error when reading file,
    // TypeMismatch for detailed error message (important special case)

    //  Manages the loading of a File, previously transformed into FileReaderData
    //  (of the right standard), into a Model

    //=================================================================================================

    /// <summary>
    /// Sets Data to a FileReaderData. Works with a Protocol
    /// </summary>
    public void SetData(Interface_FileReaderData? reader, Interface_Protocol? protocol)
    {
        thereader = reader;
        theproto = protocol;
    }

    /// <summary>
    /// Returns the Protocol given at creation time
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return theproto;
    }

    /// <summary>
    /// Returns the FileReaderData which is used to work
    /// </summary>
    public Interface_FileReaderData? Data()
    {
        return thereader;
    }

    /// <summary>
    /// Stores a Model. Used when the Model has been loaded
    /// </summary>
    public void SetModel(Interface_InterfaceModel? amodel)
    {
        themodel = amodel;
    }

    /// <summary>
    /// Returns the stored Model
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>
    /// Sets Messenger used for outputting messages
    /// </summary>
    public void SetMessenger(Message_Messenger? messenger)
    {
        if (messenger is null)
        {
            themessenger = Message.DefaultMessenger();
        }
        else
        {
            themessenger = messenger;
        }
    }

    /// <summary>
    /// Returns Messenger used for outputting messages. The returned object is guaranteed to be non-null; default
    /// is Message::Messenger().
    /// </summary>
    public Message_Messenger? Messenger()
    {
        return themessenger;
    }

    /// <summary>
    /// Sets trace level used for outputting messages - 0: no trace at all - 1: errors - 2: errors and warnings -
    /// 3: all messages Default is 1 : Errors traced
    /// </summary>
    public void SetTraceLevel(int tracelev)
    {
        thetrace = tracelev;
    }

    /// <summary>
    /// Returns trace level used for outputting messages.
    /// </summary>
    public int TraceLevel()
    {
        return thetrace;
    }

    /// <summary>
    /// Allows controlling whether exception raisings are handled If err is False, they are not (hence, dbx can
    /// take control) If err is True, they are, and they are traced (by putting on messenger Entity's Number and
    /// file record num) Default given at Model's creation time is True
    /// </summary>
    public void SetErrorHandle(bool err)
    {
        theerrhand = err;
    }

    /// <summary>
    /// Returns ErrorHandle flag
    /// </summary>
    public bool ErrorHandle()
    {
        return theerrhand;
    }

    /// <summary>
    /// Fills records with empty entities; once done, each entity can ask the FileReaderTool for any entity
    /// referenced through an identifier. Calls Recognize which is specific to each specific type of
    /// FileReaderTool
    /// </summary>
    public void SetEntities()
    {
        int num;
        thenbreps = 0;
        thenbrep0 = 0;

        for (num = thereader.FindNextRecord(0); num > 0; num = thereader.FindNextRecord(num))
        {
            Standard_Transient newent = null;
            Interface_Check ach = new Interface_Check();
            if (!Recognize(num, ref ach, ref newent))
            {
                newent = UnknownEntity();
                if (thereports is null)
                {
                    thereports =
                        new NCollection_HArray1<Standard_Transient>(1, thereader.NbRecords());
                }
                thenbreps++;
                thenbrep0++;
                thereports.SetValue(num, new Interface_ReportEntity(ach, newent));
            }
            else if ((ach.NbFails() + ach.NbWarnings() > 0) && newent is not null)
            {
                if (thereports is null)
                {
                    thereports =
                        new NCollection_HArray1<Standard_Transient>(1, thereader.NbRecords());
                }
                thenbreps++;
                thenbrep0++;
                thereports.SetValue(num, new Interface_ReportEntity(ach, newent));
            }
            thereader.BindEntity(num, newent);
        }
    }

    /// <summary>
    /// Recognizes a record, given its number. Specific to each Interface; called by SetEntities. It can call the
    /// basic method RecognizeByLib. Returns False if recognition has failed, True else. &lt;ach&gt; has not to
    /// be filled if simply Recognition has failed : it must record true error messages : RecognizeByLib can
    /// generate error messages if NewRead is called Note that it works thru a Recognizer (method Evaluate) which
    /// has to be memorized before starting
    /// </summary>
    public abstract bool Recognize(int num, ref Interface_Check? ach, ref Standard_Transient? ent);

    /// <summary>
    /// Recognizes a record with the help of Libraries. Can be used to implement the method Recognize.
    /// &lt;rlib&gt; is used to find Protocol and CaseNumber to apply &lt;glib&gt; performs the creation (by
    /// service NewVoid, or NewRead if NewVoid gave no result) &lt;ach&gt; is a check, which is transmitted to
    /// NewRead if it is called, gives a result but which is false &lt;ent&gt; is the result Returns False if
    /// recognition has failed, True else
    /// </summary>
    public bool RecognizeByLib(int num, Interface_GeneralLib glib, Interface_ReaderLib rlib, ref Interface_Check? ach, ref Standard_Transient? ent)
    {
        // Interface_GeneralModule gmod: declared and unused in OCCT
        Interface_ReaderModule rmod = null;
        Interface_Protocol proto = null;
        int CN = 0;
        //   Search in ReaderLib : Case recognition . CN , proto
        for (rlib.Start(); rlib.More(); rlib.Next())
        {
            rmod = rlib.Module();
            if (rmod is null)
            {
                continue;
            }
            CN = rmod.CaseNum(thereader, num);
            if (CN > 0)
            {
                proto = rlib.Protocol();
                break;
            }
        }
        if (CN <= 0 || proto is null)
        {
            return false;
        }
        //   Recalibrate in GeneralLib : Creation of empty entity
        Type typrot = proto.DynamicType();
        for (glib.Start(); glib.More(); glib.Next())
        {
            proto = glib.Protocol();
            if (proto is null)
            {
                continue;
            }
            if (proto.DynamicType() != typrot)
            {
                continue;
            }
            bool res = glib.Module().NewVoid(CN, out ent);
            if (res)
            {
                return res;
            }
            if (rmod is not null)
            {
                return rmod.NewRead(CN, thereader, num, ref ach, ref ent);
            }
            //    return res;
        }
        return false;
    }

    /// <summary>
    /// Provides an unknown entity, specific to the Interface called by SetEntities when Recognize has failed
    /// (Unknown alone) or by LoadModel when an Entity has caused a Fail on reading (to keep at least its literal
    /// description) Uses Protocol to do it
    /// </summary>
    public Standard_Transient? UnknownEntity()
    {
        return theproto.UnknownEntity();
    }

    /// <summary>
    /// Creates an empty Model of the norm. Uses Protocol to do it
    /// </summary>
    public Interface_InterfaceModel? NewModel()
    {
        return theproto.NewModel();
    }

    /// <summary>
    /// Reads and fills Entities from the FileReaderData set by SetData to an InterfaceModel. It enchains
    /// required operations, the specific ones correspond to deferred methods (below) to be defined for each
    /// Norm. It manages also error recovery and trace. Remark : it calls SetModel. It Can raise any error which
    /// can occur during a load operation, unless Error Handling is set. This method can also be redefined if
    /// judged necessary.
    /// </summary>
    public void LoadModel(Interface_InterfaceModel? amodel)
    {
        // MGE 16/06/98
        // Building of Messages
        //====================================
        Message_Messenger TF = Messenger();
        //====================================
        Interface_Check ach = new Interface_Check();

        SetModel(amodel);

        //  ..            Startup : Header Reading            ..
        if (theerrhand)
        {
            try
            {
                BeginRead(amodel); // according to the standard
            }
            catch (Standard_Failure)
            {
                // Sendinf of message : Internal error during the header reading
                if (TF is not null)
                {
                    Message_Msg Msg11 = new Message_Msg("XSTEP_11");
                    TF.Send(Msg11.Get(), Message_Info);
                }
            }
        }
        else
        {
            BeginRead(amodel); // selon la norme
        }

        //  ..            Reading Entities            ..

        amodel.Reservate(thereader.NbEntities());

        int num, num0 = thereader.FindNextRecord(0);
        num = num0;

        while (num > 0)
        {
            int ierr = 0; // error on analysis of an entity
            Standard_Transient anent = null;
            try
            {
                for (num = num0; num > 0; num = thereader.FindNextRecord(num))
                {
                    num0 = num;

                    //    Reading under crash protection
                    //    (also does AddEntity but not SetReportEntity)
                    anent = LoadedEntity(num);

                    //     Unprotected reading : useful for working with dbx
                    ////    else
                    ////      anent = LoadedEntity(num);

                    //   ..        End Reading        ..
                    if (anent is null)
                    {
                        // Sending of message : Number of ignored Null Entities
                        if (TF is not null)
                        {
                            Message_Msg Msg21 = new Message_Msg("XSTEP_21");
                            Msg21.Arg(amodel.NbEntities());
                            TF.Send(Msg21.Get(), Message_Info);
                        }
                        continue;
                    }
                    //      LoadedEntity does AddEntity BUT NOT SetReport (in block at the end)

                } // ---- end loop on entities
                num0 = 0; // nothing more
            } // ---- fin du try, le catch suit

            //   In case of UNFORESEEN error by the analysis, default recovery
            //   Warning : the recovery can itself crash ... (cf ierr)
            catch (Exception anException) when (anException is not OutOfMemoryException and not OperationCanceledException)
            {
                //      On the next pass, we will attack the next record
                // clang-format off
                num0 = thereader.FindNextRecord(num); //:g9 abv 28 May 98: tr8_as2_ug.stp - infinite cycle: (0);
                // clang-format on
                // OCCT: a system signal (OSD_Exception, OSD_Signal: an access violation...) gives ierr = 2; their .NET
                // counterparts are the exceptions that aren't Standard_Failure (decision 1183)
                if (anException is not Standard_Failure)
                {
                    ierr = 2;
                }
                //: abv 03Apr00: anent is actually a previous one:      if (anent is null)
                anent = thereader.BoundEntity(num);
                if (anent is null)
                {
                    if (thetrace > 0)
                    {
                        // Sending of message : Number of ignored Null Entities
                        if (TF is not null)
                        {

                            Message_Msg Msg21 = new Message_Msg("XSTEP_21");
                            Msg21.Arg(amodel.NbEntities() + 1);

                            TF.Send(Msg21.Get(), Message_Info);
                        }
                        continue;
                    }
                }
                /*Interface_Check*/ ach = new Interface_Check(anent);
                //: abv 03 Apr 00: trj3_s1-tc-214.stp: generate a message on exception
                Message_Msg Msg278 = new Message_Msg("XSTEP_278");
                Msg278.Arg(amodel.StringLabel(anent).ToCString());
                ach.SendFail(Msg278);

                if (ierr == 2)
                {
                    // Sending of message : reading of entity failed
                    if (TF is not null)
                    {
                        Message_Msg Msg22 = new Message_Msg("XSTEP_22");
                        Msg22.Arg(amodel.StringLabel(anent).ToCString());
                        TF.Send(Msg22.Get(), Message_Info);
                    }
                    return;
                }

                if (ierr == 0)
                {
                    // char mess[100]; svv #2
                    ierr = 1;
                    // what would be good here would be to recover the error text for ach ...
                    if (thetrace > 0)
                    {
                        // Sending of message : recovered entity
                        if (TF is not null)
                        {
                            Message_Msg Msg23 = new Message_Msg("XSTEP_23");
                            Msg23.Arg(num);
                            TF.Send(Msg23.Get(), Message_Info);
                        }
                    }

                    //  Finally, we load an Unknown Entity
                    thenbreps++;
                    Interface_ReportEntity rep = new Interface_ReportEntity(ach, anent);
                    Standard_Transient undef = UnknownEntity();
                    AnalyseRecord(num, undef, ref ach);
                    rep.SetContent(undef);

                    if (thereports is null)
                    {
                        thereports =
                            new NCollection_HArray1<Standard_Transient>(1, thereader.NbRecords());
                    }
                    thenbreps++;
                    thereports.SetValue(num, rep);
                    // if(isValid)
                    amodel.AddEntity(anent); // not done by LoadedEntity ...
                }
                else
                {
                    if (thetrace > 0)
                    {
                        // Sending of message : reading of entity failed
                        if (TF is not null)
                        {
                            Message_Msg Msg22 = new Message_Msg("XSTEP_22");
                            Msg22.Arg(amodel.StringLabel(anent).ToCString());
                            TF.Send(Msg22.Get(), Message_Info);
                        }
                    }
                    //  We keep <rep> as is : no additional file analysis,
                    //  But the eventual preliminary phase is preserved
                    //  (in particular, we keep trace of the Type read from the file, etc...)
                }
            } // ----- fin complete du try/catch
        } // ----- fin du while

        //  ..        Adding Reports, if any
        if (thereports is not null)
        {
            if (thetrace > 0)
            {
                // Sending of message : report
                if (TF is not null)
                {
                    Message_Msg Msg24 = new Message_Msg("XSTEP_24");
                    Msg24.Arg(thenbreps);
                    TF.Send(Msg24.Get(), Message_Info);
                }
            }
            amodel.Reservate(-thenbreps - 10);
            thenbreps = thereports.Upper();
            for (int nr = 1; nr <= thenbreps; nr++)
            {
                if (thereports.Value(nr) is null)
                {
                    continue;
                }
                Standard_Transient anent = thereader.BoundEntity(nr);
                Interface_ReportEntity rep =
                    (thereports.Value(nr) as Interface_ReportEntity);
                amodel.SetReportEntity(-amodel.Number(anent), rep);
            }
        }

        //   Conclusion : may do nothing : according to necessity
        if (theerrhand)
        {
            try
            {
                EndRead(amodel); // according to the standard
            }
            catch (Standard_Failure)
            {
                // Sendinf of message : Internal error during the header reading
                if (TF is not null)
                {
                    Message_Msg Msg11 = new Message_Msg("XSTEP_11");
                    TF.Send(Msg11.Get(), Message_Info);
                }
            }
        }
        else
        {
            EndRead(amodel); // selon la norme
        }
    }

    /// <summary>
    /// Reads, Fills and Returns one Entity read from a Record of the FileReaderData. This Method manages also
    /// case of Fail or Warning, by producing a ReportEntyty plus , for a Fail, a literal Content (as an
    /// UnknownEntity). Performs also Trace
    /// </summary>
    public Standard_Transient? LoadedEntity(int num)
    {
        Standard_Transient anent = thereader.BoundEntity(num);
        Interface_Check ach = new Interface_Check(anent);
        Interface_ReportEntity rep = null; // entite Report, s il y a lieu
        int irep = 0;
        // int nbe  = 0; svv #2
        if (thenbrep0 > 0)
        {
            rep = (thereports.Value(num) as Interface_ReportEntity);
            if (rep is not null)
            {
                irep = num;
                ach = rep.Check();
            }
        }

        //    Trace Unknown Entity
        if (thetrace >= 2 && theproto.IsUnknownEntity(anent))
        {
            Message_Messenger TF = Messenger();
            if (TF is not null)
            {
                Message_Msg Msg22 = new Message_Msg("XSTEP_22");
                // Sending of message : reading of entity failed
                Msg22.Arg(themodel.StringLabel(anent).String());
                TF.Send(Msg22.Get(), Message_Info);
            }
        }
        //  ..        Actual Loading : Standard Specific        ..
        AnalyseRecord(num, anent, ref ach);

        //  ..        Adding to the model the entity as is        ..
        //            WARNING, ReportEntity processed in block after Load
        themodel.AddEntity(anent);

        //   Error or Correction : We create a ReportEntity that memorizes the Check,
        //   the Entity, and in case of Error an UndefinedEntity for the Parameters

        //   We exploit here the IsLoadError flag: if it has been defined (true or false)
        //   it has priority over check fails. Otherwise, it's the fails that speak

        int nbf = ach.NbFails();
        int nbw = ach.NbWarnings();
        if (nbf + nbw > 0)
        {
            // int n0; svv #2
            themodel.NbEntities();
            rep = new Interface_ReportEntity(ach, anent);
            if (irep == 0)
            {
                if (thereports is null)
                {
                    thereports =
                        new NCollection_HArray1<Standard_Transient>(1, thereader.NbRecords());
                }
                irep = num;
                thenbreps++;
            }
            thereports.SetValue(irep, rep);

            if (thetrace >= 2 && Messenger() is not null)
            {
                using Message_Messenger.StreamBuffer sout = Messenger().SendInfo();
                var aText = new StringWriter();
                ach.Print(aText, 2);
                sout.Append(aText.ToString());
            }
        }

        //    Reloading ? if yes, in an UnknownEntity provided by the protocol
        if (thereader.IsErrorLoad())
        {
            nbf = (thereader.ResetErrorLoad() ? 1 : 0);
        }
        if (nbf > 0)
        {
            Standard_Transient undef = UnknownEntity();
            AnalyseRecord(num, undef, ref ach);
            rep.SetContent(undef);
        }

        //    Conclusion  (Unknown : treated externally because Raise treatment)
        ////  if (irep > 0) themodel.SetReportEntity (nbe,rep);  en bloc a la fin

        return anent;
    }

    /// <summary>
    /// Fills model's header; each Interface defines for its Model its own file header; this method fills it from
    /// FileReaderTool.+ It is called by AnalyseFile from InterfaceModel
    /// </summary>
    public abstract void BeginRead(Interface_InterfaceModel? amodel);

    /// <summary>
    /// Fills an Entity, given record no; specific to each Interface, called by AnalyseFile from InterfaceModel
    /// (which manages its calling arguments) To work, each Interface can define a method in its proper Transient
    /// class, like this (given as an example) : AnalyseRecord (me : mutable; FR : in out FileReaderTool; num :
    /// Integer; acheck : in out Check) returns Boolean; and call it from AnalyseRecord Returned Value : True if
    /// the entity could be loaded, False else (in case of syntactic fail)
    /// </summary>
    public abstract bool AnalyseRecord(int num, Standard_Transient? anent, ref Interface_Check? acheck);

    protected internal Interface_FileReaderTool()
    {
        themessenger = Message.DefaultMessenger();
        theerrhand = true;
        thetrace = 0;
        thenbrep0 = thenbreps = 0;
    }

    /// <summary>
    /// Ends file reading after reading all the entities default is doing nothing; redefinable as necessary
    /// </summary>
    public virtual void EndRead(Interface_InterfaceModel? amodel)
    {

    }

    /// <summary>
    /// Clear fields
    /// </summary>
    public void Clear()
    {
        theproto = null;
        thereader = null;
        themodel = null;
        thereports = null;
    }

    /// <summary>
    /// Constructor; sets default fields
    /// </summary>

    private Interface_Protocol? theproto;
    private Interface_FileReaderData? thereader;
    private Interface_InterfaceModel? themodel;
    private Message_Messenger? themessenger;
    private int thetrace;
    private bool theerrhand;
    private int thenbrep0;
    private int thenbreps;
    private NCollection_HArray1<Standard_Transient>? thereports;
}
