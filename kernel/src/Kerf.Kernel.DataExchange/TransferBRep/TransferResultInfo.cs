// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_TransferResultInfo.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_TransferResultInfo.lxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_TransferResultInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// Data structure for storing information on transfer result: the counts of result, result + warning(s), result +
/// fail(s), result + warning(s) + fail(s), no result, no result + warning(s), no result + fail(s), no result +
/// warning(s) + fail(s). The counts are modifiable through their <c>ref</c> accessors, as OCCT's <c>int&amp;</c>.
/// </summary>
internal class TransferResultInfo : Transient
{
    /// <summary>Creates object with all fields nullified.</summary>
    public TransferResultInfo()
    {
        Clear();
    }

    /// <summary>Resets all the fields.</summary>
    public void Clear()
    {
        _nr = _nrw = _nrf = _nrwf = _r = _rw = _rf = _rwf = 0;
    }

    public ref int Result() => ref _r;

    public ref int ResultWarning() => ref _rw;

    public ref int ResultFail() => ref _rf;

    public ref int ResultWarningFail() => ref _rwf;

    public ref int NoResult() => ref _nr;

    public ref int NoResultWarning() => ref _nrw;

    public ref int NoResultFail() => ref _nrf;

    public ref int NoResultWarningFail() => ref _nrwf;

    private int _r;
    private int _rw;
    private int _rf;
    private int _rwf;
    private int _nr;
    private int _nrw;
    private int _nrf;
    private int _nrwf;
}
