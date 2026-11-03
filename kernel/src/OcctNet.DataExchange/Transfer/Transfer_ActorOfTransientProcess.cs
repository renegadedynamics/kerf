// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfTransientProcess.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfTransientProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The actor of a <see cref="Transfer_TransientProcess"/> (the original class was renamed; compatibility only), with
/// the parameters and the operations of the shape processing its transfers do.
/// </summary>
public class Transfer_ActorOfTransientProcess : Transfer_ActorOfProcessForTransient
{
    public Transfer_ActorOfTransientProcess()
    {
    }

    public override Transfer_Binder? Transferring(Transient? ent, Transfer_ProcessForTransient? TP,
                                                  ProgressRange theProgress = default)
    {
        return Transfer(ent, TP as Transfer_TransientProcess, theProgress);
    }

    public virtual Transfer_Binder? Transfer(Transient? start, Transfer_TransientProcess? TP,
                                             ProgressRange theProgress = default)
    {
        Transient? res = TransferTransient(start, TP, theProgress);
        if (res is null)
        {
            return NullResult();
        }
        return TransientResult(res);
    }

    public virtual Transient? TransferTransient(Transient? ent, Transfer_TransientProcess? TP,
                                                         ProgressRange theProgress = default)
    {
        Transient? nulres = null;
        return nulres;
    }

    /// <summary>Sets parameters for shape processing (a copy of <paramref name="theParameters"/>).</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> theParameters)
    {
        myShapeProcParams = new NCollection_DataMap<string, string>(theParameters);
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from <paramref name="theParameters"/> are copied to the
    /// internal map. Parameters from <paramref name="theAdditionalParameters"/> are copied to the internal map if they
    /// are not present in <paramref name="theParameters"/>.
    /// </summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters,
                                      NCollection_DataMap<string, string>? theAdditionalParameters = null)
    {
        XSAlgo_ShapeProcessor.SetShapeFixParameters(theParameters,
                                                    theAdditionalParameters ?? new NCollection_DataMap<string, string>(),
                                                    myShapeProcParams);
    }

    /// <summary>
    /// Returns parameters for shape processing that was set by SetParameters() method; an empty map if no parameters
    /// were set. A copy: OCCT returns a const reference, which its callers copy before changing it.
    /// </summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters()
    {
        return new NCollection_DataMap<string, string>(myShapeProcParams);
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetProcessingFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags theFlags)
    {
        myShapeProcFlags = (theFlags, true);
    }

    /// <summary>
    /// Returns flags defining operations to be performed on shapes: the flags, and whether they were set (OCCT's
    /// <c>std::pair</c>, <c>XSAlgo_ShapeProcessor::ProcessingFlags</c>).
    /// </summary>
    public (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) GetProcessingFlags()
    {
        return myShapeProcFlags;
    }

    private NCollection_DataMap<string, string> myShapeProcParams = new(StringComparer.Ordinal); // Parameters for shape processing.
    private (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) myShapeProcFlags; // Flags defining operations to be performed on shapes.
}
