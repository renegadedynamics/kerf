// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ModelCopier.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ModelCopier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class performs the Copy operations involved by the description of a ShareOut (evaluated by a
/// ShareOutResult) plus, if there are, the Modifications on the results, with the help of Modifiers. Each
/// Modifier can work on one or more resulting packets, according to its criteria : it operates on a Model
/// once copied and filled with the content of the packet. Modifiers can be : - Model Modifiers, inheriting
/// from the specific class Modifier able to run on the content of a Model (header or entities), activated by
/// the ModelCopier itself - File Modifiers, inheriting directly from GeneralModifier, intended to be
/// activated under the control of a WorkLibrary, once the Model has been produced (i.e. to act on output
/// format, or other specific file features) The Copy operations can be : - immediately put to files : for
/// each packet, a Model is created and filled, then the file is output, at that's all - memorized : for each
/// packet, a Model is created and filled, it is memorized with the corresponding file name. it is possible
/// to query the result of memorization (list of produced Models and their file names) -&gt; it is also
/// possible to send it into the files : once files are written, the result is cleared In addition, a list of
/// really written files is managed : A first call to BeginSentFiles clears the list and commands, either to
/// begin a new list, or to stop recording it. A call to SentFiles returns the list (if recording has been
/// required) This list allows to globally exploit the set of produced files Remark : For operations which
/// concern specific Entities, see also in package IFAdapt : a sub-class of ModelCopier allows to work with
/// EntityModifier, in addition to Modifier itself which still applies to a whole copied Model.
/// </summary>
public class IFSelect_ModelCopier : Standard_Transient
{

    /// <summary>
    /// Creates an empty ModelCopier
    /// </summary>
    public IFSelect_ModelCopier()
    {
    }

    /// <summary>
    /// Sets the ShareOut, which is used to define Modifiers to apply
    /// </summary>
    public void SetShareOut(IFSelect_ShareOut? sho)
    {
        theshareout = sho;
    }

    /// <summary>
    /// Clears the list of produced Models
    /// </summary>
    public void ClearResult()
    {
        thefilemodels.Clear();
        thefilenames.Clear();
        theapplieds.Clear();
        theremain = null;
    }

    /// <summary>
    /// Records a new File to be sent, as a couple (Name as AsciiString, Content as InterfaceModel) Returns True
    /// if Done, False if &lt;filename&gt; is already attached to another File
    /// </summary>
    public bool AddFile(string filename, Interface_InterfaceModel? content)
    {
        int nb = thefilenames.Length();
        for (int i = 1; i <= nb; i++)
        {
            if (filename.Length == 0)
            {
                continue;
            }
            if (thefilenames.Value(i) == filename)
            {
                return false;
            }
        }
        IFSelect_AppliedModifiers nulapplied = null;
        thefilenames.Append(filename);
        thefilemodels.Append(content);
        theapplieds.Append(nulapplied);
        return true;
    }

    /// <summary>
    /// Changes the Name attached to a File which was formerly defined by a call to AddFile Returns True if Done,
    /// False else : if &lt;num&gt; out of range or if the new &lt;filename&gt; is already attached to another
    /// File Remark : Giving an empty File Name is equivalent to ClearFile
    /// </summary>
    public bool NameFile(int num, string filename)
    {
        int nb = thefilenames.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        for (int i = 1; i <= nb; i++)
        {
            if (filename.Length == 0)
            {
                continue;
            }
            if (thefilenames.Value(i) == filename)
            {
                return false;
            }
        }
        thefilenames.SetValue(num, filename);
        return true;
    }

    /// <summary>
    /// Clears the Name attached to a File which was formerly defined by a call to AddFile. This Clearing can be
    /// undone by a call to NameFile (with same &lt;num&gt;) Returns True if Done, False else : if &lt;num&gt; is
    /// out of range
    /// </summary>
    public bool ClearFile(int num)
    {
        int nb = thefilenames.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        thefilenames.SetValue(num, "");
        return true;
    }

    /// <summary>
    /// Sets a list of File Modifiers to be applied on a file
    /// </summary>
    public bool SetAppliedModifiers(int num, IFSelect_AppliedModifiers? applied)
    {
        int nb = theapplieds.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        theapplieds.SetValue(num, applied);
        return true;
    }

    /// <summary>
    /// Clears the list of File Modifiers to be applied on a file
    /// </summary>
    public bool ClearAppliedModifiers(int num)
    {
        int nb = theapplieds.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        theapplieds.SetValue(num, null);
        return true;
    }

    /// <summary>
    /// Sends the formerly defined results (see method Copy) to files, then clears it Remark : A Null File Name
    /// cause file to be not produced
    /// </summary>
    public Interface_CheckIterator SendCopied(IFSelect_WorkLibrary? WL, Interface_Protocol? protocol)
    {
        Message.SendInfo().Append("** WorkSession : Sending split data already copied").Append('\n').Flush();
        int nb = NbFiles();
        Interface_CheckIterator checks = new Interface_CheckIterator();
        if (nb > 0)
        {
            for (int i = 1; i <= nb; i++)
            {
                if (FileName(i).Length == 0)
                {
                    continue;
                }
                IFSelect_AppliedModifiers curapp = theapplieds.Value(i);
                IFSelect_ContextWrite ctx = new IFSelect_ContextWrite(FileModel(i), protocol, curapp, FileName(i));
                bool res = WL.WriteFile(ctx);
                Interface_CheckIterator checklst = ctx.CheckList();
                checks.Merge(checklst);
                //\t(FileName(i), FileModel(i),protocol,curapp,checks);
                //      if (!checks.IsEmpty(false)) {
                //\tsout<<"  **  On Sending File n0."<<i<<", Check Messages :  **"<<std.endl;
                //\tchecks.Print (sout,false);
                //      }
                if (!res)
                {
                    string mess = Sprintf("Split Send (WriteFile) abandon on file n0.%d", i);
                    checks.CCheck(0).AddFail(mess);
                    Message.SendInfo().Append("  **  Sending File n0.").Append(i).Append(" has failed, abandon  **").Append('\n').Flush();
                    return checks;
                }
                AddSentFile(FileName(i));
            }
            ClearResult();
        }
        checks.SetName("X-STEP WorkSession : Split Send (Copy+Write)");
        return checks;
    }

    /// <summary>
    /// Sends a model (defined in &lt;G&gt;) into one file, without managing remaining data, already sent files,
    /// etc. Applies the Model and File Modifiers. Returns True if well done, False else
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="theStream"/>, when given, is written in place of the file (decision 12706).</remarks>
    public Interface_CheckIterator SendAll(string filename, Interface_Graph G, IFSelect_WorkLibrary? WL, Interface_Protocol? protocol,
                                           Stream? theStream = null)
    {
        Interface_CheckIterator checks = new Interface_CheckIterator();
        checks.SetName("X-STEP WorkSession : Send All");
        Message.SendInfo().Append("** WorkSession : Sending all data").Append('\n').Flush();
        Interface_InterfaceModel model = G.Model();
        if (model is null || protocol is null || WL is null)
        {
            return checks;
        }

        Interface_CopyTool TC = new Interface_CopyTool(model, protocol);
        int i, nb = model.NbEntities();
        for (i = 1; i <= nb; i++)
        {
            TC.Bind(model.Value(i), model.Value(i));
        }

        Interface_EntityIterator pipo = new Interface_EntityIterator();
        Interface_InterfaceModel newmod = null;
        IFSelect_AppliedModifiers applied = null;
        CopiedModel(G, WL, protocol, pipo, filename, 0, 0, TC, ref newmod, ref applied, checks);

        IFSelect_ContextWrite ctx = new IFSelect_ContextWrite(model, protocol, applied, filename, theStream);
        bool res = WL.WriteFile(ctx);
        Interface_CheckIterator checklst = ctx.CheckList();
        checks.Merge(checklst);
        if (!res)
        {
            checks.CCheck(0).AddFail("SendAll (WriteFile) has failed");
        }
        //  if (!checks.IsEmpty(false)) {
        //    Message.SendWarning() <<
        //      "  **    SendAll has produced Check Messages :    **"<<std.endl;
        //    checks.Print (sout,model,false);
        //  }
        return checks;
    }

    /// <summary>
    /// Sends a part of a model into one file. Model is gotten from &lt;G&gt;, the part is defined in
    /// &lt;iter&gt;. Remaining data are managed and can be later be worked on. Returns True if well done, False
    /// else
    /// </summary>
    /// <remarks>OCCT.NET: <paramref name="theStream"/>, when given, is written in place of the file (decision 12706).</remarks>
    public Interface_CheckIterator SendSelected(string filename, Interface_Graph G, IFSelect_WorkLibrary? WL, Interface_Protocol? protocol,
                                                Interface_EntityIterator list, Stream? theStream = null)
    {
        Interface_CheckIterator checks = new Interface_CheckIterator();
        checks.SetName("X-STEP WorkSession : Send Selected");
        Message.SendInfo().Append("** WorkSession : Sending selected data").Append('\n').Flush();
        Interface_InterfaceModel original = G.Model();
        if (original is null || protocol is null || WL is null)
        {
            return checks;
        }
        Interface_InterfaceModel newmod = original.NewEmptyModel();
        Interface_CopyTool TC = new Interface_CopyTool(original, protocol);
        TC.FillModel(newmod); // for Header ...

        //  No copy : AddWithRefs plus Bind declaration
        Interface_GeneralLib lib = new Interface_GeneralLib(protocol);
        for (list.Start(); list.More(); list.Next())
        {
            newmod.AddWithRefs(list.Value(), lib);
        }
        int i, nb = newmod.NbEntities();
        for (i = 1; i <= nb; i++)
        {
            TC.Bind(newmod.Value(i), newmod.Value(i));
        }
        if (theremain is null)
        {
            theremain = new NCollection_HArray1<int>(0, G.Size());
            theremain.Init(0);
        }

        Interface_EntityIterator pipo = new Interface_EntityIterator();
        IFSelect_AppliedModifiers applied = null;
        CopiedModel(G, WL, protocol, pipo, filename, 0, 0, TC, ref newmod, ref applied, checks);
        //  Feed Remaining : copied entities are to be noted
        for (int ic = TC.LastCopiedAfter(0, out _, out _); ic > 0; ic = TC.LastCopiedAfter(ic, out _, out _))
        {
            if (ic <= theremain.Upper())
            {
                theremain.SetValue(ic, theremain.Value(ic) + 1);
            }
        }
        IFSelect_ContextWrite ctx = new IFSelect_ContextWrite(newmod, protocol, applied, filename, theStream);
        bool res = WL.WriteFile(ctx);
        Interface_CheckIterator checklst = ctx.CheckList();
        checks.Merge(checklst);
        if (!res)
        {
            checks.CCheck(0).AddFail("SendSelected (WriteFile) has failed");
        }
        //  if (!checks.IsEmpty(false)) {
        //    Message.SendWarning() <<
        //      "  **    SendSelected has produced Check Messages :    **"<<std.endl;
        //    checks.Print (sout,original,false);
        //  }
        return checks;
    }

    /// <summary>
    /// Produces a Model copied from the Remaining List as &lt;newmod&gt; &lt;newmod&gt; is a Null Handle if this
    /// list is empty &lt;WL&gt; performs the copy by using &lt;TC&gt; &lt;TC&gt; is assumed to have been defined
    /// with the starting model same as defined by &lt;G&gt;. Produces a model copied from the remaining list.
    /// @param[in] G the interface graph @param[in] WL the work library performing the copy @param[in,out] TC the
    /// copy tool @return the new model with remaining data, or null handle if empty
    /// </summary>
    public Interface_InterfaceModel? CopiedRemaining(Interface_Graph G, IFSelect_WorkLibrary? WL, Interface_CopyTool TC)
    {
        Interface_InterfaceModel original = G.Model();
        //  Interface_CopyTool TC(original,protocol);
        Interface_InterfaceModel newmod = original.NewEmptyModel();
        TC.Clear();
        Interface_EntityIterator tocopy = new Interface_EntityIterator();
        int nb = G.Size();
        theremain = new NCollection_HArray1<int>(0, nb + 1);
        theremain.Init(0);
        for (int i = 1; i <= nb; i++)
        {
            if (G.Status(i) == 0)
            {
                tocopy.AddItem(original.Value(i));
            }
            else
            {
                theremain.SetValue(i, -1); // ?? -1
            }
        }
        WL.CopyModel(original, newmod, tocopy, TC);

        if (newmod.NbEntities() == 0)
        {
            newmod = null;
        }
        else
        {
            //  WHAT FOLLOWS MUST NOT BE DELETED ! cf theremain
            for (int ic = TC.LastCopiedAfter(0, out _, out _); ic > 0; ic = TC.LastCopiedAfter(ic, out _, out _))
            {
                if (ic <= theremain.Upper())
                {
                    theremain.SetValue(ic, 1);
                }
            }
        //  some debugging prints (MISOPOINT) left out
        }

        return newmod;
    }

    /// <summary>
    /// Updates Graph status for remaining data, for each entity : - Entities just Sent to file or Copied (by
    /// CopiedRemaining) have their status set to 1 - the other keep their former status (1 for Send/Copied, 0
    /// for Remaining) These status are computed by Copying/Sending/CopiedRemaining Then, SetRemaining updates
    /// graph status, and mustr be called just after one of these method has been called Returns True if done,
    /// False if remaining info if not in phase which the Graph (not same counts of items)
    /// </summary>
    public bool SetRemaining(Interface_Graph CG)
    {
        int nb = CG.Size();
        if (theremain is null)
        {
            return (nb == 0);
        }
        if (nb != theremain.Upper())
        {
            return false;
        }
        for (int i = 1; i <= nb; i++)
        {
            if (CG.Status(i) >= 0)
            {
                CG.SetStatus(i, CG.Status(i) + theremain.Value(i));
            }
        }
        theremain.Init(0);
        return true;
    }

    /// <summary>
    /// Returns the count of Files produced, i.e. the count of Models memorized (produced by the mmethod Copy)
    /// with their file names
    /// </summary>
    public int NbFiles()
    {
        return thefilemodels.Length();
    }

    /// <summary>
    /// Returns the File Name for a file given its rank It is empty after a call to ClearFile on same &lt;num&gt;
    /// </summary>
    public string FileName(int num)
    {
        return thefilenames.Value(num);
    }

    /// <summary>
    /// Returns the content of a file before sending, under the form of an InterfaceModel, given its rank
    /// </summary>
    public Interface_InterfaceModel? FileModel(int num)
    {
        return thefilemodels.Value(num);
    }

    /// <summary>
    /// Returns the list of File Modifiers to be applied on a file when it will be sent, as computed by
    /// CopiedModel : If it is a null handle, no File Modifier has to be applied.
    /// </summary>
    public IFSelect_AppliedModifiers? AppliedModifiers(int num)
    {
        return theapplieds.Value(num);
    }

    /// <summary>
    /// Begins a sequence of recording the really sent files &lt;sho&gt; : the default file numbering is cleared
    /// If &lt;record&gt; is False, clears the list and stops recording If &lt;record&gt; is True, clears the
    /// list and commands recording Creation time corresponds to "stop recording"
    /// </summary>
    public void BeginSentFiles(IFSelect_ShareOut? sho, bool record)
    {
        thesentfiles = null;
        if (record)
        {
            thesentfiles = new NCollection_HSequence<TCollection_HAsciiString>();
        }
        //  and default file numbering : held by ShareOut
        if (sho is null)
        {
            return;
        }
        int lastrun = sho.LastRun();
        sho.ClearResult(true);
        sho.SetLastRun(lastrun); // we are only interested in the numbers
    }

    /// <summary>
    /// Adds the name of a just sent file, if BeginSentFiles has commanded recording; else does nothing It is
    /// called by methods SendCopied Sending
    /// </summary>
    public void AddSentFile(string filename)
    {
        if (thesentfiles is not null)
        {
            thesentfiles.Append(new TCollection_HAsciiString(filename));
        }
    }

    /// <summary>
    /// Returns the list of recorded names of sent files. Can be empty (if no file has been sent). Returns a Null
    /// Handle if BeginSentFiles has stopped recording.
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? SentFiles()
    {
        return thesentfiles;
    }

    /// <summary>
    /// Performs the Copy of a unitary Packet Input parameters are : &lt;G&gt; is the graph which defines the
    /// starting entities, it contains the original InterfaceModel &lt;WL&gt; performs the copy by using
    /// &lt;TC&gt; &lt;protocol&gt; is the used protocol (can be useful for Modifiers) &lt;topcopy&gt; is the
    /// list of Entities which are the Roots of the packet to be copied &lt;filename&gt; is the name of the file
    /// which will receive it &lt;dispid&gt; is the Identifier of the Dispatch which have produced this packet,
    /// &lt;numod&gt; is the rank of the packet for this Dispatch &lt;TC&gt; is a CopyTool, which performs the
    /// copy Returned values (as arguments) are : &lt;newmod&gt; is the result of the copy, as a new
    /// InterfaceModel on which Model Modifiers have already been applied (if there are) &lt;applied&gt;
    /// determines the File Modifiers which remain to be applied (when the file itself will be output) : for each
    /// File Modifier recorded in &lt;me&gt;, &lt;applied&gt;'s Value is : - Null if this Modifier has not to be
    /// applied - an empty list if this Modifier has to be applied without distinguishing specific entities - a
    /// list of numbers of entities in &lt;model&gt; if this Modifier concerns particularly these entities (which
    /// are the results of copying the result of its input selection) &lt;checks&gt; is the produced Check List
    /// (by Modifiers as required) Warning : File Modifiers are evaluated at the time of Copy itself If their
    /// list is changed between this Copy and the Sending itself of the file, these changes are ignored
    /// </summary>
    protected void CopiedModel(Interface_Graph G, IFSelect_WorkLibrary? WL, Interface_Protocol? protocol, Interface_EntityIterator tocopy, string filename, int dispnum, int numod, Interface_CopyTool TC, ref Interface_InterfaceModel? newmod, ref IFSelect_AppliedModifiers? applied, Interface_CheckIterator checks)
    {
        //  ...  First "standard" part : filling the model  ...
        //  We create the Model, we fill it with Entities, and with the starting Header

        //  WARNING : dispnum = 0  means take original model, copy nothing
        //                             and also : no Dispatch (bulk sending)

        applied = null;
        Interface_InterfaceModel original = G.Model();
        if (dispnum > 0)
        {
            newmod = original.NewEmptyModel();
            TC.Clear();
            WL.CopyModel(original, newmod, tocopy, TC);

            //  Feed Remaining : copied entities are to be noted
            for (int ic = TC.LastCopiedAfter(0, out _, out _); ic > 0; ic = TC.LastCopiedAfter(ic, out _, out _))
            {
                if (ic <= theremain.Upper())
                {
                    theremain.SetValue(ic, theremain.Value(ic) + 1);
                }
            }
        }
        else if (newmod is null)
        {
            newmod = original;
        }

        //  ...  Then : We take into account the Model Modifiers  ...
        int nbmod = 0;
        if (theshareout is not null)
        {
            nbmod = theshareout.NbModifiers(true);
        }
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= nbmod; i++)
        {
            IFSelect_Modifier unmod = theshareout.ModelModifier(i);

            //    First,  Dispatch/Packet criterion
            if (dispnum > 0)
            {
                if (!unmod.Applies(theshareout.Dispatch(dispnum)))
                {
                    continue;
                }
            }
            IFSelect_ContextModif ctx = new IFSelect_ContextModif(G, TC, filename);
            //    Then, the Selection
            IFSelect_Selection sel = unmod.Selection();
            if (sel is not null)
            {
                Interface_EntityIterator entiter = sel.UniqueResult(G);
                ctx.Select(entiter);
            }
            if (ctx.IsForNone())
            {
                continue;
            }
            unmod.Perform(ctx, newmod, protocol, TC);
            Interface_CheckIterator checklst = ctx.CheckList();
            checks.Merge(checklst);

            //    Should we record errors in newmod ? good question
            //    if (!checks.IsEmpty(false)) {
            //      Message.SendWarning() <<
            //        " Messages on Copied Model n0 "<<numod<<", Dispatch Rank "<<dispnum<<std.endl;
            //      checks.Print(sout,newmod,false);
            //    }
        }

        //  ...  Then the File Modifiers : in fact, we record them  ...
        nbmod = 0;
        if (theshareout is not null)
        {
            nbmod = theshareout.NbModifiers(false);
        }
        if (nbmod == 0)
        {
            return;
        }
        applied = new IFSelect_AppliedModifiers(nbmod, newmod.NbEntities());
        for (i = 1; i <= nbmod; i++)
        {
            IFSelect_GeneralModifier unmod = theshareout.GeneralModifier(false, i);

            //    First,  Dispatch/Packet criterion
            if (dispnum > 0)
            {
                if (!unmod.Applies(theshareout.Dispatch(dispnum)))
                {
                    continue;
                }
            }
            //    Then, the Selection
            IFSelect_Selection sel = unmod.Selection();
            if (sel is null)
            {
                applied.AddModif(unmod); // empty . we take all
            }
            else
            {
                Interface_EntityIterator list = sel.UniqueResult(G);
                //    Entities designated by the Selection and Copied ?
                //    . if there is at least one, the Modifier applies, otherwise it is rejected
                //    . and this list is exploitable by the Modifier ...
                for (list.Start(); list.More(); list.Next())
                {
                    if (TC.Search(list.Value(), out Standard_Transient newent))
                    {
                        applied.AddNum(newmod.Number(newent));
                    }
                }
            }
        }
    }

    private NCollection_Sequence<Interface_InterfaceModel> thefilemodels = new NCollection_Sequence<Interface_InterfaceModel>();
    private NCollection_Sequence<string> thefilenames = new NCollection_Sequence<string>();
    private NCollection_Sequence<IFSelect_AppliedModifiers> theapplieds = new NCollection_Sequence<IFSelect_AppliedModifiers>();
    private IFSelect_ShareOut? theshareout;
    private NCollection_HArray1<int>? theremain;
    private NCollection_HSequence<TCollection_HAsciiString>? thesentfiles;
}
