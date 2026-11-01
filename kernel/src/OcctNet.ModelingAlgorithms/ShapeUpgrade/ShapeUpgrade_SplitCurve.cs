// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// The root of the tools splitting a curve: the splitting values (the curve's first and last parameters and the
/// values between them), computed by <see cref="Compute"/> and used by <see cref="Build"/>.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_SplitCurve : Standard_Transient
{
    /// <summary>The splitting values, the first and last parameters included.</summary>
    protected NCollection_HSequence<double>? mySplitValues;
    /// <summary>The number of resulting curves.</summary>
    protected int myNbCurves;
    /// <summary>The status.</summary>
    protected int myStatus;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_SplitCurve()
    {
        myNbCurves = 0;
        myStatus = 0;
    }

    /// <summary>Initializes with the curve's first and last parameters.</summary>
    public void Init(double First, double Last)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myNbCurves = 1;
        // mySplitValues.Clear();
        mySplitValues = new NCollection_HSequence<double>();
        mySplitValues.Append(First);
        mySplitValues.Append(Last);
    }

    /// <summary>
    /// Sets the parameters where splitting has to be done: those strictly inside the current intervals (by more than
    /// PConfusion) are inserted, in order.
    /// </summary>
    public void SetSplitValues(NCollection_HSequence<double>? SplitValues)
    {
        double precision = Occt.Precision.PConfusion();
        if (SplitValues is null)
        {
            return;
        }
        if (SplitValues.Length() == 0)
        {
            return;
        }

        double First = mySplitValues!.Value(1), Last = mySplitValues.Value(mySplitValues.Length());
        int i = 1;
        int len = SplitValues.Length();
        for (int k = 2; k <= mySplitValues.Length(); k++)
        {
            Last = mySplitValues.Value(k);
            for (; i <= len; i++)
            {
                if ((First + precision) >= SplitValues.Value(i))
                {
                    continue;
                }
                if ((Last - precision) <= SplitValues.Value(i))
                {
                    break;
                }
                mySplitValues.InsertBefore(k++, SplitValues.Value(i));
            }
            First = Last;
        }
    }

    /// <summary>
    /// Builds the resulting curves (segments of the curve for B-splines and Beziers when <paramref name="Segment"/>,
    /// trimmed curves otherwise); the base class only resets the status.
    /// </summary>
    public virtual void Build(bool Segment)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Returns the splitting values, the first and last parameters of the curve included.</summary>
    public NCollection_HSequence<double>? SplitValues() => mySplitValues;

    /// <summary>Calculates the splitting values; the base class only resets the status.</summary>
    public virtual void Compute()
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Computes the splitting values (<see cref="Compute"/>), then builds the curves (<see cref="Build"/>).</summary>
    public void Perform(bool Segment = true)
    {
        Compute();
        // if ( ! mySplitValues.IsNull() )
        //   SetSplitValues(mySplitValues);
        Build(Segment);
    }

    /// <summary>
    /// Returns the status: OK (nothing to split), DONE1 (split values given or computed), DONE2 (splitting required,
    /// a single segment), DONE3 (the curve was modified), FAIL1 (an error occurred).
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);
}
