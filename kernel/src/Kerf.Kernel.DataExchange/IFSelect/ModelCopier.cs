// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ModelCopier.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ModelCopier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

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
public class ModelCopier : Transient
{

    /// <summary>
    /// Creates an empty ModelCopier
    /// </summary>
    public ModelCopier()
    {
    }

    /// <summary>
    /// Sets the ShareOut, which is used to define Modifiers to apply
    /// </summary>
    public void SetShareOut(ShareOut? sho)
    {
        _theshareout = sho;
    }

    /// <summary>
    /// Clears the list of produced Models
    /// </summary>
    public void ClearResult()
    {
        _thefilemodels.Clear();
        _thefilenames.Clear();
        _theapplieds.Clear();
        _theremain = null;
    }

    /// <summary>
    /// Records a new File to be sent, as a couple (Name as AsciiString, Content as InterfaceModel) Returns True
    /// if Done, False if &lt;filename&gt; is already attached to another File
    /// </summary>
    public bool AddFile(string filename, InterfaceModel? content)
    {
        int nb = _thefilenames.Length();
        for (int i = 1; i <= nb; i++)
        {
            if (filename.Length == 0)
            {
                continue;
            }
            if (_thefilenames.Value(i) == filename)
            {
                return false;
            }
        }
        AppliedModifiers nulapplied = null;
        _thefilenames.Append(filename);
        _thefilemodels.Append(content);
        _theapplieds.Append(nulapplied);
        return true;
    }

    /// <summary>
    /// Changes the Name attached to a File which was formerly defined by a call to AddFile Returns True if Done,
    /// False else : if &lt;num&gt; out of range or if the new &lt;filename&gt; is already attached to another
    /// File Remark : Giving an empty File Name is equivalent to ClearFile
    /// </summary>
    public bool NameFile(int num, string filename)
    {
        int nb = _thefilenames.Length();
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
            if (_thefilenames.Value(i) == filename)
            {
                return false;
            }
        }
        _thefilenames.SetValue(num, filename);
        return true;
    }

    /// <summary>
    /// Clears the Name attached to a File which was formerly defined by a call to AddFile. This Clearing can be
    /// undone by a call to NameFile (with same &lt;num&gt;) Returns True if Done, False else : if &lt;num&gt; is
    /// out of range
    /// </summary>
    public bool ClearFile(int num)
    {
        int nb = _thefilenames.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        _thefilenames.SetValue(num, "");
        return true;
    }

    /// <summary>
    /// Sets a list of File Modifiers to be applied on a file
    /// </summary>
    public bool SetAppliedModifiers(int num, AppliedModifiers? applied)
    {
        int nb = _theapplieds.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        _theapplieds.SetValue(num, applied);
        return true;
    }

    /// <summary>
    /// Clears the list of File Modifiers to be applied on a file
    /// </summary>
    public bool ClearAppliedModifiers(int num)
    {
        int nb = _theapplieds.Length();
        if (num <= 0 || num > nb)
        {
            return false;
        }
        _theapplieds.SetValue(num, null);
        return true;
    }

    /// <summary>
    /// Sends the formerly defined results (see method Copy) to files, then clears it Remark : A Null File Name
    /// cause file to be not produced
    /// </summary>
    public CheckIterator SendCopied(WorkLibrary? wl, Protocol? protocol)
    {
        global::Kerf.Engine.Foundation.Message.SendInfo().Append("** WorkSession : Sending split data already copied").Append('\n').Flush();
        int nb = FileCount;
        CheckIterator checks = new CheckIterator();
        if (nb > 0)
        {
            for (int i = 1; i <= nb; i++)
            {
                if (FileName(i).Length == 0)
                {
                    continue;
                }
                AppliedModifiers curapp = _theapplieds.Value(i);
                ContextWrite ctx = new ContextWrite(FileModel(i), protocol, curapp, FileName(i));
                bool res = wl.WriteFile(ctx);
                CheckIterator checklst = ctx.CheckList();
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
                    global::Kerf.Engine.Foundation.Message.SendInfo().Append("  **  Sending File n0.").Append(i).Append(" has failed, abandon  **").Append('\n').Flush();
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
    /// <remarks>OCCT.NET: <paramref name="stream"/>, when given, is written in place of the file (decision 1225).</remarks>
    public CheckIterator SendAll(string filename, EntityGraph g, WorkLibrary? wl, Protocol? protocol,
                                           Stream? stream = null)
    {
        CheckIterator checks = new CheckIterator();
        checks.SetName("X-STEP WorkSession : Send All");
        global::Kerf.Engine.Foundation.Message.SendInfo().Append("** WorkSession : Sending all data").Append('\n').Flush();
        InterfaceModel model = g.Model();
        if (model is null || protocol is null || wl is null)
        {
            return checks;
        }

        CopyTool tc = new CopyTool(model, protocol);
        int i, nb = model.EntityCount;
        for (i = 1; i <= nb; i++)
        {
            tc.Bind(model.Value(i), model.Value(i));
        }

        EntityIterator pipo = new EntityIterator();
        InterfaceModel newmod = null;
        AppliedModifiers applied = null;
        CopiedModel(g, wl, protocol, pipo, filename, 0, 0, tc, ref newmod, ref applied, checks);

        ContextWrite ctx = new ContextWrite(model, protocol, applied, filename, stream);
        bool res = wl.WriteFile(ctx);
        CheckIterator checklst = ctx.CheckList();
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
    /// <remarks>OCCT.NET: <paramref name="stream"/>, when given, is written in place of the file (decision 1225).</remarks>
    public CheckIterator SendSelected(string filename, EntityGraph g, WorkLibrary? wl, Protocol? protocol,
                                                EntityIterator list, Stream? stream = null)
    {
        CheckIterator checks = new CheckIterator();
        checks.SetName("X-STEP WorkSession : Send Selected");
        global::Kerf.Engine.Foundation.Message.SendInfo().Append("** WorkSession : Sending selected data").Append('\n').Flush();
        InterfaceModel original = g.Model();
        if (original is null || protocol is null || wl is null)
        {
            return checks;
        }
        InterfaceModel newmod = original.NewEmptyModel();
        CopyTool tc = new CopyTool(original, protocol);
        tc.FillModel(newmod); // for Header ...

        //  No copy : AddWithRefs plus Bind declaration
        GeneralLib lib = new GeneralLib(protocol);
        for (list.Start(); list.More(); list.Next())
        {
            newmod.AddWithRefs(list.Value(), lib);
        }
        int i, nb = newmod.EntityCount;
        for (i = 1; i <= nb; i++)
        {
            tc.Bind(newmod.Value(i), newmod.Value(i));
        }
        if (_theremain is null)
        {
            _theremain = new NCollection_HArray1<int>(0, g.Size());
            _theremain.Init(0);
        }

        EntityIterator pipo = new EntityIterator();
        AppliedModifiers applied = null;
        CopiedModel(g, wl, protocol, pipo, filename, 0, 0, tc, ref newmod, ref applied, checks);
        //  Feed Remaining : copied entities are to be noted
        for (int ic = tc.LastCopiedAfter(0, out _, out _); ic > 0; ic = tc.LastCopiedAfter(ic, out _, out _))
        {
            if (ic <= _theremain.Upper())
            {
                _theremain.SetValue(ic, _theremain.Value(ic) + 1);
            }
        }
        ContextWrite ctx = new ContextWrite(newmod, protocol, applied, filename, stream);
        bool res = wl.WriteFile(ctx);
        CheckIterator checklst = ctx.CheckList();
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
    public InterfaceModel? CopiedRemaining(EntityGraph g, WorkLibrary? wl, CopyTool tc)
    {
        InterfaceModel original = g.Model();
        //  Interface_CopyTool TC(original,protocol);
        InterfaceModel newmod = original.NewEmptyModel();
        tc.Clear();
        EntityIterator tocopy = new EntityIterator();
        int nb = g.Size();
        _theremain = new NCollection_HArray1<int>(0, nb + 1);
        _theremain.Init(0);
        for (int i = 1; i <= nb; i++)
        {
            if (g.Status(i) == 0)
            {
                tocopy.AddItem(original.Value(i));
            }
            else
            {
                _theremain.SetValue(i, -1); // ?? -1
            }
        }
        wl.CopyModel(original, newmod, tocopy, tc);

        if (newmod.EntityCount == 0)
        {
            newmod = null;
        }
        else
        {
            //  WHAT FOLLOWS MUST NOT BE DELETED ! cf theremain
            for (int ic = tc.LastCopiedAfter(0, out _, out _); ic > 0; ic = tc.LastCopiedAfter(ic, out _, out _))
            {
                if (ic <= _theremain.Upper())
                {
                    _theremain.SetValue(ic, 1);
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
    public bool SetRemaining(EntityGraph cg)
    {
        int nb = cg.Size();
        if (_theremain is null)
        {
            return (nb == 0);
        }
        if (nb != _theremain.Upper())
        {
            return false;
        }
        for (int i = 1; i <= nb; i++)
        {
            if (cg.Status(i) >= 0)
            {
                cg.SetStatus(i, cg.Status(i) + _theremain.Value(i));
            }
        }
        _theremain.Init(0);
        return true;
    }

    /// <summary>
    /// Returns the count of Files produced, i.e. the count of Models memorized (produced by the mmethod Copy)
    /// with their file names
    /// </summary>
    public int FileCount
    {
        get
        {
            return _thefilemodels.Length();
        }
    }

    /// <summary>
    /// Returns the File Name for a file given its rank It is empty after a call to ClearFile on same &lt;num&gt;
    /// </summary>
    public string FileName(int num)
    {
        return _thefilenames.Value(num);
    }

    /// <summary>
    /// Returns the content of a file before sending, under the form of an InterfaceModel, given its rank
    /// </summary>
    public InterfaceModel? FileModel(int num)
    {
        return _thefilemodels.Value(num);
    }

    /// <summary>
    /// Returns the list of File Modifiers to be applied on a file when it will be sent, as computed by
    /// CopiedModel : If it is a null handle, no File Modifier has to be applied.
    /// </summary>
    public AppliedModifiers? AppliedModifiers(int num)
    {
        return _theapplieds.Value(num);
    }

    /// <summary>
    /// Begins a sequence of recording the really sent files &lt;sho&gt; : the default file numbering is cleared
    /// If &lt;record&gt; is False, clears the list and stops recording If &lt;record&gt; is True, clears the
    /// list and commands recording Creation time corresponds to "stop recording"
    /// </summary>
    public void BeginSentFiles(ShareOut? sho, bool record)
    {
        _thesentfiles = null;
        if (record)
        {
            _thesentfiles = new NCollection_HSequence<TCollection_HAsciiString>();
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
        if (_thesentfiles is not null)
        {
            _thesentfiles.Append(new TCollection_HAsciiString(filename));
        }
    }

    /// <summary>
    /// Returns the list of recorded names of sent files. Can be empty (if no file has been sent). Returns a Null
    /// Handle if BeginSentFiles has stopped recording.
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? SentFiles()
    {
        return _thesentfiles;
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
    protected void CopiedModel(EntityGraph g, WorkLibrary? wl, Protocol? protocol, EntityIterator tocopy, string filename, int dispnum, int numod, CopyTool tc, ref InterfaceModel? newmod, ref AppliedModifiers? applied, CheckIterator checks)
    {
        //  ...  First "standard" part : filling the model  ...
        //  We create the Model, we fill it with Entities, and with the starting Header

        //  WARNING : dispnum = 0  means take original model, copy nothing
        //                             and also : no Dispatch (bulk sending)

        applied = null;
        InterfaceModel original = g.Model();
        if (dispnum > 0)
        {
            newmod = original.NewEmptyModel();
            tc.Clear();
            wl.CopyModel(original, newmod, tocopy, tc);

            //  Feed Remaining : copied entities are to be noted
            for (int ic = tc.LastCopiedAfter(0, out _, out _); ic > 0; ic = tc.LastCopiedAfter(ic, out _, out _))
            {
                if (ic <= _theremain.Upper())
                {
                    _theremain.SetValue(ic, _theremain.Value(ic) + 1);
                }
            }
        }
        else if (newmod is null)
        {
            newmod = original;
        }

        //  ...  Then : We take into account the Model Modifiers  ...
        int nbmod = 0;
        if (_theshareout is not null)
        {
            nbmod = _theshareout.NbModifiers(true);
        }
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= nbmod; i++)
        {
            SelectionModifier unmod = _theshareout.ModelModifier(i);

            //    First,  Dispatch/Packet criterion
            if (dispnum > 0)
            {
                if (!unmod.Applies(_theshareout.Dispatch(dispnum)))
                {
                    continue;
                }
            }
            ContextModif ctx = new ContextModif(g, tc, filename);
            //    Then, the Selection
            AbstractSelection sel = unmod.Selection();
            if (sel is not null)
            {
                EntityIterator entiter = sel.UniqueResult(g);
                ctx.Select(entiter);
            }
            if (ctx.IsForNone())
            {
                continue;
            }
            unmod.Perform(ctx, newmod, protocol, tc);
            CheckIterator checklst = ctx.CheckList();
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
        if (_theshareout is not null)
        {
            nbmod = _theshareout.NbModifiers(false);
        }
        if (nbmod == 0)
        {
            return;
        }
        applied = new AppliedModifiers(nbmod, newmod.EntityCount);
        for (i = 1; i <= nbmod; i++)
        {
            GeneralModifier unmod = _theshareout.GeneralModifier(false, i);

            //    First,  Dispatch/Packet criterion
            if (dispnum > 0)
            {
                if (!unmod.Applies(_theshareout.Dispatch(dispnum)))
                {
                    continue;
                }
            }
            //    Then, the Selection
            AbstractSelection sel = unmod.Selection();
            if (sel is null)
            {
                applied.AddModif(unmod); // empty . we take all
            }
            else
            {
                EntityIterator list = sel.UniqueResult(g);
                //    Entities designated by the Selection and Copied ?
                //    . if there is at least one, the Modifier applies, otherwise it is rejected
                //    . and this list is exploitable by the Modifier ...
                for (list.Start(); list.More(); list.Next())
                {
                    if (tc.Search(list.Value(), out Transient newent))
                    {
                        applied.AddNum(newmod.Number(newent));
                    }
                }
            }
        }
    }

    private NCollection_Sequence<InterfaceModel> _thefilemodels = new NCollection_Sequence<InterfaceModel>();
    private NCollection_Sequence<string> _thefilenames = new NCollection_Sequence<string>();
    private NCollection_Sequence<AppliedModifiers> _theapplieds = new NCollection_Sequence<AppliedModifiers>();
    private ShareOut? _theshareout;
    private NCollection_HArray1<int>? _theremain;
    private NCollection_HSequence<TCollection_HAsciiString>? _thesentfiles;
}
