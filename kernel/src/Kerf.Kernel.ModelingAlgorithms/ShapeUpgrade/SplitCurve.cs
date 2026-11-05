// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// The root of the tools splitting a curve: the splitting values (the curve's first and last parameters and the
/// values between them), computed by <see cref="Compute"/> and used by <see cref="Build"/>.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class SplitCurve : Transient
{
    /// <summary>The splitting values, the first and last parameters included.</summary>
    protected NCollection_HSequence<double>? _splitValues;
    /// <summary>The number of resulting curves.</summary>
    protected int _nbCurves;
    /// <summary>The status.</summary>
    protected int _status;

    /// <summary>Empty constructor.</summary>
    public SplitCurve()
    {
        _nbCurves = 0;
        _status = 0;
    }

    /// <summary>Initializes with the curve's first and last parameters.</summary>
    public void Init(double first, double last)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _nbCurves = 1;
        // mySplitValues.Clear();
        _splitValues = new NCollection_HSequence<double>();
        _splitValues.Append(first);
        _splitValues.Append(last);
    }

    /// <summary>
    /// Sets the parameters where splitting has to be done: those strictly inside the current intervals (by more than
    /// PConfusion) are inserted, in order.
    /// </summary>
    public void SetSplitValues(NCollection_HSequence<double>? splitValues)
    {
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        if (splitValues is null)
        {
            return;
        }
        if (splitValues.Length() == 0)
        {
            return;
        }

        double first = _splitValues!.Value(1), last = _splitValues.Value(_splitValues.Length());
        int i = 1;
        int len = splitValues.Length();
        for (int k = 2; k <= _splitValues.Length(); k++)
        {
            last = _splitValues.Value(k);
            for (; i <= len; i++)
            {
                if ((first + precision) >= splitValues.Value(i))
                {
                    continue;
                }
                if ((last - precision) <= splitValues.Value(i))
                {
                    break;
                }
                _splitValues.InsertBefore(k++, splitValues.Value(i));
            }
            first = last;
        }
    }

    /// <summary>
    /// Builds the resulting curves (segments of the curve for B-splines and Beziers when <paramref name="segment"/>,
    /// trimmed curves otherwise); the base class only resets the status.
    /// </summary>
    public virtual void Build(bool segment)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Returns the splitting values, the first and last parameters of the curve included.</summary>
    public NCollection_HSequence<double>? SplitValues() => _splitValues;

    /// <summary>Calculates the splitting values; the base class only resets the status.</summary>
    public virtual void Compute()
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Computes the splitting values (<see cref="Compute"/>), then builds the curves (<see cref="Build"/>).</summary>
    public void Perform(bool segment = true)
    {
        Compute();
        // if ( ! mySplitValues.IsNull() )
        //   SetSplitValues(mySplitValues);
        Build(segment);
    }

    /// <summary>
    /// Returns the status: OK (nothing to split), DONE1 (split values given or computed), DONE2 (splitting required,
    /// a single segment), DONE3 (the curve was modified), FAIL1 (an error occurred).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);
}
