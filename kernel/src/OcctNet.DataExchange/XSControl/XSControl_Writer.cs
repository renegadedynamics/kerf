// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Writer.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class gives a simple way to create then write a Model compliant to a given norm, from a Shape The
/// model can then be edited by tools by other appropriate tools
/// </summary>
public class XSControl_Writer
{

    /// <summary>
    /// Creates a Writer from scratch
    /// </summary>
    public XSControl_Writer()
    {
        SetWS(new XSControl_WorkSession());
    }

    /// <summary>
    /// Creates a Writer from scratch, with a norm name which identifie a Controller
    /// </summary>
    public XSControl_Writer(string norm)
    {
        SetNorm(norm);
    }

    /// <summary>
    /// Creates a Writer from an already existing Session If &lt;scratch&gt; is True (D), clears already recorded
    /// data
    /// </summary>
    public XSControl_Writer(XSControl_WorkSession? WS, bool scratch = true)
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
        bool sess = thesession.SelectNorm(norm);
        Interface_InterfaceModel model = Model(); //: i1 gka 03.04.99 BUC60301
        return sess;
    }

    /// <summary>
    /// Sets a specific session to &lt;me&gt;
    /// </summary>
    public void SetWS(XSControl_WorkSession? WS, bool scratch = true)
    {
        thesession = WS;
        //  A controller must be defined ...
        thesession.InitTransferReader(0);
        Interface_InterfaceModel model = Model(scratch);
    }

    /// <summary>
    /// Returns the session used in &lt;me&gt;
    /// </summary>
    public XSControl_WorkSession? WS()
    {
        return thesession;
    }

    /// <summary>
    /// Returns the produced model. Produces a new one if not yet done or if &lt;newone&gt; is True This method
    /// allows for instance to edit product or header data before writing
    /// </summary>
    public Interface_InterfaceModel? Model(bool newone = false)
    {
        Interface_InterfaceModel model = thesession.Model();
        if (newone || model is null)
        {
            model = thesession.NewModel();
        }
        return model;
    }

    /// <summary>
    /// Transfers a Shape according to the mode
    /// </summary>
    public IFSelect_ReturnStatus TransferShape(TopoDS_Shape sh, int mode = 0, Message_ProgressRange theProgress = default)
    {
        thesession.TransferWriter().SetTransferMode(mode);
        return thesession.TransferWriteShape(sh, true, theProgress);
    }

    /// <summary>
    /// Writes the produced model
    /// </summary>
    public IFSelect_ReturnStatus WriteFile(string filename)
    {
        return thesession.SendAll(filename);
    }

    /// <summary>
    /// Writes the produced model to a stream (OCCT.NET: OCCT writes only to files through the session's work
    /// library; the stream goes to the library through IFSelect_ContextWrite, decision 12706).
    /// </summary>
    public IFSelect_ReturnStatus WriteStream(Stream theStream, string theName = "")
    {
        return thesession.SendAll(theName, false, theStream);
    }

    /// <summary>
    /// Prints Statistics about Transfer
    /// </summary>
    public void PrintStatsTransfer(int what, int mode = 0)
    {
        thesession.TransferWriter().PrintStats(what, mode);
    }

    private XSControl_WorkSession? thesession;
}
