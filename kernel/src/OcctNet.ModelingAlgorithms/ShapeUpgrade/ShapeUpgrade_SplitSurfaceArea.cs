// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Splits a surface into a number of parts of about the same area: into strips along its longer direction, or, in
/// the "splitting into squares" mode, into parts close to squares in 2D (or into fixed numbers of U and V parts).
/// </summary>
/// <remarks>A working object (decision 21). OCCT's modifiable reference <see cref="NbParts"/> returns <c>ref</c>.</remarks>
public class ShapeUpgrade_SplitSurfaceArea : ShapeUpgrade_SplitSurface
{
    private int myNbParts;
    private int myUnbSplit;
    private int myVnbSplit;
    private bool myIsSplittingIntoSquares;

    /// <summary>Empty constructor (one part).</summary>
    public ShapeUpgrade_SplitSurfaceArea()
    {
        myNbParts = 1;
        myUnbSplit = myVnbSplit = -1;
        myIsSplittingIntoSquares = false;
    }

    /// <summary>The number of parts to split into.</summary>
    public ref int NbParts() => ref myNbParts;

    /// <summary>
    /// Sets the splitting mode: "splitting into squares" splits the face into about <see cref="NbParts"/> parts close
    /// to squares in 2D.
    /// </summary>
    public void SetSplittingIntoSquares(bool theIsSplittingIntoSquares)
    {
        myIsSplittingIntoSquares = theIsSplittingIntoSquares;
    }

    /// <summary>Sets fixed numbers of splits in U and V (only in the "splitting into squares" mode).</summary>
    public void SetNumbersUVSplits(int theNbUsplits, int theNbVsplits)
    {
        myUnbSplit = theNbUsplits;
        myVnbSplit = theNbVsplits;
        if (myUnbSplit > 0 && myVnbSplit > 0)
        {
            myNbParts = myUnbSplit * myVnbSplit;
        }
    }

    /// <summary>Computes the U and V splitting values.</summary>
    public override void Compute(bool Segment = true)
    {
        if (myNbParts <= 1)
        {
            return;
        }

        var ads = new GeomAdaptor_Surface(mySurface!,
                                          myUSplitValues!.Value(1),
                                          myUSplitValues.Value(2),
                                          myVSplitValues!.Value(1),
                                          myVSplitValues.Value(2));
        double aKoefU = ads.UResolution(1.0);
        double aKoefV = ads.VResolution(1.0);
        if (aKoefU == 0)
        {
            aKoefU = 1.0;
        }
        if (aKoefV == 0)
        {
            aKoefV = 1.0;
        }
        double aUSize = Math.Abs(myUSplitValues.Value(2) - myUSplitValues.Value(1)) / aKoefU;
        double aVSize = Math.Abs(myVSplitValues.Value(2) - myVSplitValues.Value(1)) / aKoefV;
        double aNbUV = aUSize / aVSize;
        NCollection_HSequence<double> aFirstSplit = (aNbUV < 1.0 ? myVSplitValues : myUSplitValues);
        NCollection_HSequence<double> aSecondSplit = (aNbUV < 1.0 ? myUSplitValues : myVSplitValues);
        bool anIsUFirst = (aNbUV > 1.0);
        if (aNbUV < 1)
        {
            aNbUV = 1.0 / aNbUV;
        }

        bool anIsFixedUVnbSplits = (myUnbSplit > 0 && myVnbSplit > 0);
        int nbSplitF, nbSplitS;
        if (myIsSplittingIntoSquares && myNbParts > 0)
        {
            if (!anIsFixedUVnbSplits) //(myUnbSplit <= 0 || myVnbSplit <= 0)
            {
                double aSquareSize = Math.Sqrt(myArea / myNbParts);
                myUnbSplit = (int)(myUsize / aSquareSize);
                myVnbSplit = (int)(myVsize / aSquareSize);
                if (myUnbSplit == 0)
                {
                    myUnbSplit = 1;
                }
                if (myVnbSplit == 0)
                {
                    myVnbSplit = 1;
                }
            }

            if (anIsUFirst)
            {
                nbSplitF = myUnbSplit;
                nbSplitS = myVnbSplit;
            }
            else
            {
                nbSplitF = myVnbSplit;
                nbSplitS = myUnbSplit;
            }
        }
        else
        {
            nbSplitF = (aNbUV >= myNbParts ? myNbParts : RealToInt(Math.Ceiling(Math.Sqrt(myNbParts * Math.Ceiling(aNbUV)))));
            nbSplitS = (aNbUV >= myNbParts ? 0 : RealToInt(Math.Ceiling((double)myNbParts / (double)nbSplitF)));
        }
        if (nbSplitS == 1 && !anIsFixedUVnbSplits)
        {
            nbSplitS++;
        }
        if (nbSplitF == 0)
        {
            return;
        }
        double aStep = (aFirstSplit.Value(2) - aFirstSplit.Value(1)) / nbSplitF;
        double aPrevPar = aFirstSplit.Value(1);
        int i = 1;
        for (; i < nbSplitF; i++)
        {
            double aNextPar = aPrevPar + aStep;
            aFirstSplit.InsertBefore(i + 1, aNextPar);
            aPrevPar = aNextPar;
        }

        if (nbSplitS != 0)
        {
            aStep = (aSecondSplit.Value(2) - aSecondSplit.Value(1)) / nbSplitS;
            aPrevPar = aSecondSplit.Value(1);
            for (i = 1; i < nbSplitS; i++)
            {
                double aNextPar = aPrevPar + aStep;
                aSecondSplit.InsertBefore(i + 1, aNextPar);
                aPrevPar = aNextPar;
            }
        }
    }
}
