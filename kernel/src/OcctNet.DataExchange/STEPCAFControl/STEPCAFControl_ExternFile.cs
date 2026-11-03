// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.cxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_ExternFile.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Auxiliary class serving as container for data resulting from translation of external file
/// </summary>
/// <remarks>OCCT.NET: the label is an <see cref="XdeDoc_Label"/> in place of a <c>TDF_Label</c> (decision 1231).</remarks>
public class STEPCAFControl_ExternFile : Transient
{
    /// <summary>Creates an empty structure</summary>
    public STEPCAFControl_ExternFile()
    {
        myLoadStatus = IFSelect_ReturnStatus.IFSelect_RetVoid;
        myTransferStatus = false;
        myWriteStatus = IFSelect_ReturnStatus.IFSelect_RetVoid;
    }

    public void SetWS(XSControl_WorkSession WS) => myWS = WS;

    public XSControl_WorkSession GetWS() => myWS;

    public void SetLoadStatus(IFSelect_ReturnStatus stat) => myLoadStatus = stat;

    public IFSelect_ReturnStatus GetLoadStatus() => myLoadStatus;

    public void SetTransferStatus(bool isok) => myTransferStatus = isok;

    public bool GetTransferStatus() => myTransferStatus;

    public void SetWriteStatus(IFSelect_ReturnStatus stat) => myWriteStatus = stat;

    public IFSelect_ReturnStatus GetWriteStatus() => myWriteStatus;

    public void SetName(TCollection_HAsciiString name) => myName = name;

    public TCollection_HAsciiString GetName() => myName;

    public void SetLabel(in XdeDoc_Label Label) => myLabel = Label;

    public XdeDoc_Label GetLabel() => myLabel;

    private XSControl_WorkSession myWS;
    private IFSelect_ReturnStatus myLoadStatus;
    private bool myTransferStatus;
    private IFSelect_ReturnStatus myWriteStatus;
    private TCollection_HAsciiString myName;
    private XdeDoc_Label myLabel;
}
