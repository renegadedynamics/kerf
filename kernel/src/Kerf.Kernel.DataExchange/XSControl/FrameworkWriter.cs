// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Writer.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class gives a simple way to create then write a Model compliant to a given norm, from a Shape The
/// model can then be edited by tools by other appropriate tools
/// </summary>
internal class FrameworkWriter
{

    /// <summary>
    /// Creates a Writer from scratch
    /// </summary>
    public FrameworkWriter()
    {
        SetWS(new ExchangeWorkSession());
    }

    /// <summary>
    /// Creates a Writer from scratch, with a norm name which identifie a Controller
    /// </summary>
    public FrameworkWriter(string norm)
    {
        SetNorm(norm);
    }

    /// <summary>
    /// Creates a Writer from an already existing Session If &lt;scratch&gt; is True (D), clears already recorded
    /// data
    /// </summary>
    public FrameworkWriter(ExchangeWorkSession? ws, bool scratch = true)
    {
        SetWS(ws, scratch);
    }

    /// <summary>
    /// Sets a specific norm to &lt;me&gt; Returns True if done, False if &lt;norm&gt; is not available
    /// </summary>
    public bool SetNorm(string norm)
    {
        if (_thesession is null)
        {
            SetWS(new ExchangeWorkSession());
        }
        bool sess = _thesession.SelectNorm(norm);
        InterfaceModel model = Model(); //: i1 gka 03.04.99 BUC60301
        return sess;
    }

    /// <summary>
    /// Sets a specific session to &lt;me&gt;
    /// </summary>
    public void SetWS(ExchangeWorkSession? ws, bool scratch = true)
    {
        _thesession = ws;
        //  A controller must be defined ...
        _thesession.InitTransferReader(0);
        InterfaceModel model = Model(scratch);
    }

    /// <summary>
    /// Returns the session used in &lt;me&gt;
    /// </summary>
    public ExchangeWorkSession? WS()
    {
        return _thesession;
    }

    /// <summary>
    /// Returns the produced model. Produces a new one if not yet done or if &lt;newone&gt; is True This method
    /// allows for instance to edit product or header data before writing
    /// </summary>
    public InterfaceModel? Model(bool newone = false)
    {
        InterfaceModel model = _thesession.Model();
        if (newone || model is null)
        {
            model = _thesession.NewModel();
        }
        return model;
    }

    /// <summary>
    /// Transfers a Shape according to the mode
    /// </summary>
    public ReturnStatus TransferShape(Shape sh, int mode = 0, ProgressRange progress = default)
    {
        _thesession.TransferWriter().SetTransferMode(mode);
        return _thesession.TransferWriteShape(sh, true, progress);
    }

    /// <summary>
    /// Writes the produced model
    /// </summary>
    public ReturnStatus WriteFile(string filename)
    {
        return _thesession.SendAll(filename);
    }

    /// <summary>
    /// Writes the produced model to a stream (OCCT.NET: OCCT writes only to files through the session's work
    /// library; the stream goes to the library through IFSelect_ContextWrite, decision 1225).
    /// </summary>
    public ReturnStatus WriteStream(Stream stream, string name = "")
    {
        return _thesession.SendAll(name, false, stream);
    }

    /// <summary>
    /// Prints Statistics about Transfer
    /// </summary>
    public void PrintStatsTransfer(int what, int mode = 0)
    {
        _thesession.TransferWriter().PrintStats(what, mode);
    }

    private ExchangeWorkSession? _thesession;
}
