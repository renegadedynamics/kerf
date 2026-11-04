// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Network.hxx
// Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Network.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Advanced;

/// <summary>
/// The grid of patches of <see cref="ApproxAFunc2Var"/>: patch (i, j) covers
/// [UParameter(i), UParameter(i + 1)] x [VParameter(j), VParameter(j + 1)].
/// </summary>
/// <remarks>
/// A working object of the approximation, with OCCT's public mutators (decision 194). The constructor copies the
/// sequences, as OCCT's does; the patches are shared.
/// </remarks>
internal class Network
{
    private readonly NCollection_Sequence<Patch> _net;
    private readonly NCollection_Sequence<double> _uParameters;
    private readonly NCollection_Sequence<double> _vParameters;

    public Network()
    {
        _net = new NCollection_Sequence<Patch>();
        _uParameters = new NCollection_Sequence<double>();
        _vParameters = new NCollection_Sequence<double>();
    }

    public Network(NCollection_Sequence<Patch> net,
                              NCollection_Sequence<double> theU,
                              NCollection_Sequence<double> theV)
    {
        _net = new NCollection_Sequence<Patch>(net);
        _uParameters = new NCollection_Sequence<double>(theU);
        _vParameters = new NCollection_Sequence<double>(theV);
    }

    /// <summary>
    /// The index of the first patch not approximated in <paramref name="index"/>; false if every patch is.
    /// </summary>
    /// <remarks><paramref name="index"/> is left unchanged when the result is false, as in OCCT.</remarks>
    public bool FirstNotApprox(ref int index)
    {
        int anIndex = 1;
        foreach (Patch patch in _net)
        {
            if (!patch.IsApproximated)
            {
                index = anIndex;
                return true;
            }
            ++anIndex;
        }
        return false;
    }

    /// <summary>The patch of index <paramref name="index"/> (row by row, U fastest).</summary>
    public Patch ChangePatch(int index) => _net.Value(index);

    /// <summary>C++: <c>operator()(Index)</c>.</summary>
    public Patch this[int index] => ChangePatch(index);

    public void UpdateInU(double cuttingValue)
    {
        // Insert the new cutting parameter.
        int i = 1;
        while (_uParameters.Value(i) < cuttingValue)
        {
            i++;
        }
        _uParameters.InsertBefore(i, cuttingValue);

        int nbPatchInU = _uParameters.Length() - 1;
        for (int j = 1; j < _vParameters.Length(); j++)
        {
            // Modify the patch impacted by the cut.
            int patchIndex = nbPatchInU * (j - 1) + i - 1;
            Patch pat = _net.Value(patchIndex);
            pat.ChangeDomain(pat.U0(), cuttingValue, pat.V0(), pat.V1());
            pat.ResetApprox();

            // Insert the right-side patch.
            Patch newPat = new(cuttingValue,
                                           _uParameters.Value(i + 1),
                                           _vParameters.Value(j),
                                           _vParameters.Value(j + 1),
                                           pat.UOrder(),
                                           pat.VOrder());
            newPat.ResetApprox();
            _net.InsertAfter(patchIndex, newPat);
        }
    }

    public void UpdateInV(double cuttingValue)
    {
        // Insert the new cutting parameter.
        int j = 1;
        while (_vParameters.Value(j) < cuttingValue)
        {
            j++;
        }
        _vParameters.InsertBefore(j, cuttingValue);

        int nbPatchInU = _uParameters.Length() - 1;

        // Modify the patches affected by the cut.
        for (int i = 1; i <= nbPatchInU; i++)
        {
            int patchIndex = nbPatchInU * (j - 2) + i;
            Patch patch = _net.Value(patchIndex);
            patch.ChangeDomain(patch.U0(), patch.U1(), patch.V0(), cuttingValue);
            patch.ResetApprox();
        }

        // Insert the top patches.
        for (int i = 1; i <= nbPatchInU; i++)
        {
            int patchIndex = nbPatchInU * (j - 1) + i - 1;
            Patch sourcePatch = _net.Value(nbPatchInU * (j - 2) + i);
            Patch newPat = new(_uParameters.Value(i),
                                           _uParameters.Value(i + 1),
                                           cuttingValue,
                                           _vParameters.Value(j + 1),
                                           sourcePatch.UOrder(),
                                           sourcePatch.VOrder());
            newPat.ResetApprox();
            _net.InsertAfter(patchIndex, newPat);
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
        foreach (Patch pat in _net)
        {
            ncfu = Max(ncfu, pat.CoeffInUCount);
            ncfv = Max(ncfv, pat.CoeffInVCount);
        }

        //  Increase the number of coefficients
        foreach (Patch pat in _net)
        {
            pat.ChangeNbCoeff(ncfu, ncfv);
        }
    }

    public int PatchCount => _net.Length();

    public int PatchInUCount => _uParameters.Length() - 1;

    public int PatchInVCount => _vParameters.Length() - 1;

    public double UParameter(int index) => _uParameters.Value(index);

    public double VParameter(int index) => _vParameters.Value(index);

    /// <summary>The patch (<paramref name="uIndex"/>, <paramref name="vIndex"/>).</summary>
    public Patch Patch(int uIndex, int vIndex) =>
        _net.Value((vIndex - 1) * (_uParameters.Length() - 1) + uIndex);

    /// <summary>C++: <c>operator()(UIndex, VIndex)</c>.</summary>
    public Patch this[int uIndex, int vIndex] => Patch(uIndex, vIndex);
}
