// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfFinderProcess.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfFinderProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// The actor of a <see cref="FinderProcess"/> (the original class was renamed; compatibility only), with the
/// parameters and the operations of the shape processing its transfers do. ModeTrans : a simple way of transmitting a
/// transfer mode from a user. To be interpreted for each norm.
/// </summary>
public class ActorOfFinderProcess : ActorOfProcessForFinder
{
    public ActorOfFinderProcess()
    {
        _themodetrans = 0;
    }

    /// <summary>Returns the Transfer Mode, modifiable.</summary>
    public ref int ModeTrans()
    {
        return ref _themodetrans;
    }

    public override Binder? Transferring(Finder? ent, ProcessForFinder? tp,
                                                  ProgressRange progress = default)
    {
        return Transfer(ent, tp as FinderProcess, progress);
    }

    public virtual Binder? Transfer(Finder? fnd, FinderProcess? fp,
                                             ProgressRange progress = default)
    {
        TransientMapper? tm = fnd as TransientMapper;
        if (tm is null)
        {
            return NullResult();
        }
        Transient? res = TransferTransient(tm.Value(), fp, progress);
        if (res is null)
        {
            return NullResult();
        }
        return TransientResult(res);
    }

    public virtual Transient? TransferTransient(Transient? ent, FinderProcess? fp,
                                                         ProgressRange progress = default)
    {
        Transient? nulres = null;
        return nulres;
    }

    /// <summary>Sets parameters for shape processing (a copy of <paramref name="parameters"/>).</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> parameters)
    {
        _shapeProcParams = new NCollection_DataMap<string, string>(parameters);
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from <paramref name="parameters"/> are copied to the
    /// internal map. Parameters from <paramref name="additionalParameters"/> are copied to the internal map if they
    /// are not present in <paramref name="parameters"/>.
    /// </summary>
    public void SetShapeFixParameters(in ShapeFixParameters parameters,
                                      NCollection_DataMap<string, string>? additionalParameters = null)
    {
        ShapeProcessor.SetShapeFixParameters(parameters,
                                                    additionalParameters ?? new NCollection_DataMap<string, string>(),
                                                    _shapeProcParams);
    }

    /// <summary>
    /// Returns parameters for shape processing that was set by SetParameters() method; an empty map if no parameters
    /// were set. A copy: OCCT returns a const reference, which its callers copy before changing it.
    /// </summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters()
    {
        return new NCollection_DataMap<string, string>(_shapeProcParams);
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags flags)
    {
        _shapeProcFlags = (flags, true);
    }

    /// <summary>
    /// Returns flags defining operations to be performed on shapes: the flags, and whether they were set (OCCT's
    /// <c>std::pair</c>, <c>XSAlgo_ShapeProcessor::ProcessingFlags</c>).
    /// </summary>
    public (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        return _shapeProcFlags;
    }

    protected int _themodetrans;

    private NCollection_DataMap<string, string> _shapeProcParams = new(StringComparer.Ordinal); // Parameters for shape processing.
    private (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) _shapeProcFlags; // Flags defining operations to be performed on shapes.
}
