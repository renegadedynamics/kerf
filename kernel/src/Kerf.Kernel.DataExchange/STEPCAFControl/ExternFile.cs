// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.cxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Documents;

/// <summary>
/// Auxiliary class serving as container for data resulting from translation of external file
/// </summary>
/// <remarks>Kerf: the label is an <see cref="Label"/> in place of a <c>TDF_Label</c> (decision 1231).</remarks>
internal class ExternFile : Transient
{
    /// <summary>Creates an empty structure</summary>
    public ExternFile()
    {
        _loadStatus = ReturnStatus.RetVoid;
        _transferStatus = false;
        _writeStatus = ReturnStatus.RetVoid;
    }

    public void SetWS(ExchangeWorkSession ws) => _ws = ws;

    public ExchangeWorkSession GetWS() => _ws;

    public void SetLoadStatus(ReturnStatus stat) => _loadStatus = stat;

    public ReturnStatus GetLoadStatus() => _loadStatus;

    public void SetTransferStatus(bool isok) => _transferStatus = isok;

    public bool GetTransferStatus() => _transferStatus;

    public void SetWriteStatus(ReturnStatus stat) => _writeStatus = stat;

    public ReturnStatus GetWriteStatus() => _writeStatus;

    public void SetName(TCollection_HAsciiString name) => _name = name;

    public TCollection_HAsciiString GetName() => _name;

    public void SetLabel(in Label label) => _label = label;

    public Label GetLabel() => _label;

    private ExchangeWorkSession _ws;
    private ReturnStatus _loadStatus;
    private bool _transferStatus;
    private ReturnStatus _writeStatus;
    private TCollection_HAsciiString _name;
    private Label _label;
}
