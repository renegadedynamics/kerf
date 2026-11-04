// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_WorkLibrary.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_WorkLibrary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This class defines the (empty) frame which can be used to enrich a XSTEP set with new capabilities In
/// particular, a specific WorkLibrary must give the way for Reading a File into a Model, and Writing a Model
/// to a File Thus, it is possible to define several Work Libraries for each norm, but recommended to define
/// one general class for each one : this general class will define the Read and Write methods. Also a Dump
/// service is provided, it can produce, according the norm, either a parcel of a file for an entity, or any
/// other kind of information relevant for the norm,
/// </summary>
internal abstract class WorkLibrary : Transient
{

    /// <summary>
    /// Gives the way to Read a File and transfer it to a Model &lt;mod&gt; is the resulting Model, which has to
    /// be created by this method. In case of error, &lt;mod&gt; must be returned Null Return value is a status
    /// with free values. Simply, 0 is for "Execution OK" The Protocol can be used to work (e.g. create the
    /// Model, read and recognize the Entities)
    /// </summary>
    /// <remarks>
    /// OCCT.NET: <paramref name="progress"/> (not in OCCT) lets a library report and cancel the read (plan 2.7).
    /// </remarks>
    public abstract int ReadFile(string name, ref InterfaceModel? model, Protocol? protocol,
                                 ProgressRange progress = default);

    /// <summary>
    /// Interface to read a data from the specified stream. @param model is the resulting Model, which has to be
    /// created by this method. In case of error, model must be returned Null Return value is a status: 0 - OK, 1
    /// - read failure, -1 - stream failure. Default implementation returns 1 (error).
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the stream is a .NET <see cref="Stream"/>; <paramref name="progress"/> (not in OCCT) lets a
    /// library report and cancel the read (plan 2.7).
    /// </remarks>
    public virtual int ReadStream(string name, Stream iStream, ref InterfaceModel? model, Protocol? protocol,
                                  ProgressRange progress = default)
    {
        return 1;
    }

    /// <summary>
    /// Gives the way to Write a File from a Model. &lt;ctx&gt; contains all necessary information : the model,
    /// the protocol, the file name, and the list of File Modifiers to be applied, also with restricted list of
    /// selected entities for each one, if required. In return, it brings the produced check-list The WorkLibrary
    /// has to query &lt;applied&gt; to get then run the ContextWrite by looping like this (example) : for (numap
    /// = 1; numap &lt;= ctx.NbModifiers(); numap ++) { ctx.SetModifier (numap); cast ctx.FileModifier() to
    /// specific type -&gt; variable filemod if (!filemod.IsNull()) filemod-&gt;Perform (ctx,writer); filemod
    /// then works with ctx. It can, either act on the model itself (for instance on its header), or iterate on
    /// selected entities (Start/Next/More/Value) it can call AddFail or AddWarning, as necessary }
    /// </summary>
    public abstract bool WriteFile(ContextWrite ctx);

    /// <summary>
    /// Performs the copy of entities from an original model to a new one. It must also copy headers if any.
    /// Returns True when done. The provided default works by copying the individual entities designated in the
    /// list, by using the general service class CopyTool. It can be redefined for a norm which, either
    /// implements Copy by another way (do not forget to Bind each copied result with its original entity in TC)
    /// and returns True, or does not know how to copy and returns False
    /// </summary>
    public virtual bool CopyModel(InterfaceModel? original, InterfaceModel? newmodel, EntityIterator list, CopyTool tc)
    {
        for (list.Start(); list.More(); list.Next())
        {
            tc.TransferEntity(list.Value());
        }

        tc.FillModel(newmodel);

        return true;
    }

    /// <summary>
    /// Gives the way of dumping an entity under a form comprehensive for each norm. &lt;model&gt; helps to
    /// identify, number ... entities. &lt;level&gt; is to be interpreted for each norm (because of the formats
    /// which can be very different)
    /// </summary>
    public abstract void DumpEntity(InterfaceModel? model, Protocol? protocol, Transient? entity, TextWriter s, int level);

    /// <summary>
    /// Calls deferred DumpEntity with the recorded default level
    /// </summary>
    public void DumpEntity(InterfaceModel? model, Protocol? protocol, Transient? entity, TextWriter s)
    {
        if (_thelevhlp is null)
        {
            DumpEntity(model, protocol, entity, s, 0);
        }
        else
        {
            DumpEntity(model, protocol, entity, s, _thelevdef);
        }
    }

    /// <summary>
    /// Records a default level and a maximum value for level level for DumpEntity can go between 0 and
    /// &lt;max&gt; default value will be &lt;def&gt;
    /// </summary>
    public void SetDumpLevels(int def, int max)
    {
        _thelevdef = def;
        _thelevhlp = null;
        if (max >= 0)
        {
            _thelevhlp = new NCollection_HArray1<TCollection_HAsciiString>(0, max);
        }
    }

    /// <summary>
    /// Returns the recorded default and maximum dump levels If none was recorded, max is returned negative, def
    /// as zero
    /// </summary>
    public void DumpLevels(out int def, out int max)
    {
        def = _thelevdef;
        if (_thelevhlp is null)
        {
            def = 0;
            max = -1;
        }
        else
        {
            max = _thelevhlp.Upper();
        }
    }

    /// <summary>
    /// Records a short line of help for a level (0 - max)
    /// </summary>
    public void SetDumpHelp(int level, string help)
    {
        if (_thelevhlp is null)
        {
            return;
        }
        if (level < 0 || level > _thelevhlp.Upper())
        {
            return;
        }
        TCollection_HAsciiString str = new TCollection_HAsciiString(help);
        _thelevhlp.SetValue(level, str);
    }

    /// <summary>
    /// Returns the help line recorded for &lt;level&gt;, or an empty string
    /// </summary>
    public string DumpHelp(int level)
    {
        if (_thelevhlp is null)
        {
            return "";
        }
        if (level < 0 || level > _thelevhlp.Upper())
        {
            return "";
        }
        TCollection_HAsciiString str = _thelevhlp.Value(level);
        if (str is null)
        {
            return "";
        }
        return str.ToCString();
    }

    /// <summary>
    /// Required to initialise fields
    /// </summary>
    protected WorkLibrary()
    {
        _thelevdef = 0;
    }

    private int _thelevdef;
    private NCollection_HArray1<TCollection_HAsciiString>? _thelevhlp;
}
