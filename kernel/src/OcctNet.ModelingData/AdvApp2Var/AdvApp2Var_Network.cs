// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Network.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Network.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The grid of patches of <see cref="AdvApp2Var_ApproxAFunc2Var"/>: patch (i, j) covers
/// [UParameter(i), UParameter(i + 1)] x [VParameter(j), VParameter(j + 1)].
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 1903). The constructor copies the
/// sequences, as OCCT's does; the patches are shared.
/// </remarks>
public class AdvApp2Var_Network
{
    private readonly NCollection_Sequence<AdvApp2Var_Patch> myNet;
    private readonly NCollection_Sequence<double> myUParameters;
    private readonly NCollection_Sequence<double> myVParameters;

    public AdvApp2Var_Network()
    {
        myNet = new NCollection_Sequence<AdvApp2Var_Patch>();
        myUParameters = new NCollection_Sequence<double>();
        myVParameters = new NCollection_Sequence<double>();
    }

    public AdvApp2Var_Network(NCollection_Sequence<AdvApp2Var_Patch> Net,
                              NCollection_Sequence<double> TheU,
                              NCollection_Sequence<double> TheV)
    {
        myNet = new NCollection_Sequence<AdvApp2Var_Patch>(Net);
        myUParameters = new NCollection_Sequence<double>(TheU);
        myVParameters = new NCollection_Sequence<double>(TheV);
    }

    /// <summary>
    /// The index of the first patch not approximated in <paramref name="theIndex"/>; false if every patch is.
    /// </summary>
    /// <remarks><paramref name="theIndex"/> is left unchanged when the result is false, as in OCCT.</remarks>
    public bool FirstNotApprox(ref int theIndex)
    {
        int anIndex = 1;
        foreach (AdvApp2Var_Patch aPatch in myNet)
        {
            if (!aPatch.IsApproximated())
            {
                theIndex = anIndex;
                return true;
            }
            ++anIndex;
        }
        return false;
    }

    /// <summary>The patch of index <paramref name="Index"/> (row by row, U fastest).</summary>
    public AdvApp2Var_Patch ChangePatch(int Index) => myNet.Value(Index);

    /// <summary>C++: <c>operator()(Index)</c>.</summary>
    public AdvApp2Var_Patch this[int Index] => ChangePatch(Index);

    public void UpdateInU(double CuttingValue)
    {
        // Insert the new cutting parameter.
        int i = 1;
        while (myUParameters.Value(i) < CuttingValue)
        {
            i++;
        }
        myUParameters.InsertBefore(i, CuttingValue);

        int aNbPatchInU = myUParameters.Length() - 1;
        for (int j = 1; j < myVParameters.Length(); j++)
        {
            // Modify the patch impacted by the cut.
            int aPatchIndex = aNbPatchInU * (j - 1) + i - 1;
            AdvApp2Var_Patch aPat = myNet.Value(aPatchIndex);
            aPat.ChangeDomain(aPat.U0(), CuttingValue, aPat.V0(), aPat.V1());
            aPat.ResetApprox();

            // Insert the right-side patch.
            AdvApp2Var_Patch aNewPat = new(CuttingValue,
                                           myUParameters.Value(i + 1),
                                           myVParameters.Value(j),
                                           myVParameters.Value(j + 1),
                                           aPat.UOrder(),
                                           aPat.VOrder());
            aNewPat.ResetApprox();
            myNet.InsertAfter(aPatchIndex, aNewPat);
        }
    }

    public void UpdateInV(double CuttingValue)
    {
        // Insert the new cutting parameter.
        int j = 1;
        while (myVParameters.Value(j) < CuttingValue)
        {
            j++;
        }
        myVParameters.InsertBefore(j, CuttingValue);

        int aNbPatchInU = myUParameters.Length() - 1;

        // Modify the patches affected by the cut.
        for (int i = 1; i <= aNbPatchInU; i++)
        {
            int aPatchIndex = aNbPatchInU * (j - 2) + i;
            AdvApp2Var_Patch aPatch = myNet.Value(aPatchIndex);
            aPatch.ChangeDomain(aPatch.U0(), aPatch.U1(), aPatch.V0(), CuttingValue);
            aPatch.ResetApprox();
        }

        // Insert the top patches.
        for (int i = 1; i <= aNbPatchInU; i++)
        {
            int aPatchIndex = aNbPatchInU * (j - 1) + i - 1;
            AdvApp2Var_Patch aSourcePatch = myNet.Value(aNbPatchInU * (j - 2) + i);
            AdvApp2Var_Patch aNewPat = new(myUParameters.Value(i),
                                           myUParameters.Value(i + 1),
                                           CuttingValue,
                                           myVParameters.Value(j + 1),
                                           aSourcePatch.UOrder(),
                                           aSourcePatch.VOrder());
            aNewPat.ResetApprox();
            myNet.InsertAfter(aPatchIndex, aNewPat);
        }
    }

    /// <summary>
    /// Gives every patch the largest numbers of coefficients in U and V of all the patches (at least those of the
    /// continuity orders <paramref name="iu"/> and <paramref name="iv"/>), returned in <paramref name="ncfu"/> and
    /// <paramref name="ncfv"/>.
    /// </summary>
    public void SameDegree(int iu, int iv, out int ncfu, out int ncfv)
    {
        //  Compute the max coefficients, initialized according to the continuity order
        ncfu = 2 * iu + 2;
        ncfv = 2 * iv + 2;
        foreach (AdvApp2Var_Patch aPat in myNet)
        {
            ncfu = Max(ncfu, aPat.NbCoeffInU());
            ncfv = Max(ncfv, aPat.NbCoeffInV());
        }

        //  Increase the number of coefficients
        foreach (AdvApp2Var_Patch aPat in myNet)
        {
            aPat.ChangeNbCoeff(ncfu, ncfv);
        }
    }

    public int NbPatch() => myNet.Length();

    public int NbPatchInU() => myUParameters.Length() - 1;

    public int NbPatchInV() => myVParameters.Length() - 1;

    public double UParameter(int Index) => myUParameters.Value(Index);

    public double VParameter(int Index) => myVParameters.Value(Index);

    /// <summary>The patch (<paramref name="UIndex"/>, <paramref name="VIndex"/>).</summary>
    public AdvApp2Var_Patch Patch(int UIndex, int VIndex) =>
        myNet.Value((VIndex - 1) * (myUParameters.Length() - 1) + UIndex);

    /// <summary>C++: <c>operator()(UIndex, VIndex)</c>.</summary>
    public AdvApp2Var_Patch this[int UIndex, int VIndex] => Patch(UIndex, VIndex);
}
