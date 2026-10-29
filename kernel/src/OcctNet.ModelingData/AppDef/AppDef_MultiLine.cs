// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiLine.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Lpa, le 3/12/91

using System.Globalization;

namespace Occt;

/// <summary>
/// The ordered set of points approximated by AppDef: n MultiPointConstraints, each holding the corresponding
/// points (of dimension 2 or 3) of the lines approximated in parallel.
/// </summary>
/// <remarks>
/// <code>
/// P1______P2_____P3______P4________........_____PNbMult
/// Q1______Q2_____Q3______Q4________........_____QNbMult
/// R1______R2_____R3______R4________........_____RNbMult
/// </code>
/// (P1, Q1, ..., R1), ..., (Pn, Qn, ..., Rn) are the MultiPointConstraints, approximated in this order.
/// A class (decision 1601): OCCT copies it by value, but its data are a handle, so a copy shares the
/// MultiPointConstraints, as a C# reference does. <see cref="SetValue"/> is public (decision 21): callers fill a
/// MultiLine before approximating it.
/// </remarks>
public class AppDef_MultiLine
{
    protected NCollection_HArray1<AppDef_MultiPointConstraint>? tabMult;

    /// <summary>An undefined MultiLine.</summary>
    public AppDef_MultiLine() { }

    /// <summary>
    /// A MultiLine of <paramref name="NbMult"/> MultiPointConstraints, to be set with <see cref="SetValue"/>.
    /// Throws Standard_ConstructionError if <paramref name="NbMult"/> is negative.
    /// </summary>
    public AppDef_MultiLine(int NbMult)
    {
        if (NbMult < 0)
        {
            throw new Standard_ConstructionError();
        }

        tabMult = new NCollection_HArray1<AppDef_MultiPointConstraint>(1, NbMult);
    }

    /// <summary>A MultiLine of the MultiPointConstraints <paramref name="tabMultiP"/> (shared, as C++ copies share their arrays).</summary>
    public AppDef_MultiLine(NCollection_Array1<AppDef_MultiPointConstraint> tabMultiP)
    {
        tabMult = new NCollection_HArray1<AppDef_MultiPointConstraint>(1, tabMultiP.Length());
        int i, Lower = tabMultiP.Lower();
        for (i = 1; i <= tabMultiP.Length(); i++)
        {
            tabMult.SetValue(i, tabMultiP.Value(Lower + i - 1));
        }
    }

    /// <summary>A MultiLine of one line of 3d points, without tangents.</summary>
    public AppDef_MultiLine(NCollection_Array1<gp_Pnt> tabP3d)
    {
        tabMult = new NCollection_HArray1<AppDef_MultiPointConstraint>(1, tabP3d.Length());
        int i, Lower = tabP3d.Lower();
        for (i = 1; i <= tabP3d.Length(); i++)
        {
            AppDef_MultiPointConstraint MP = new(1, 0);
            MP.SetPoint(1, tabP3d[Lower + i - 1]);
            tabMult.SetValue(i, MP);
        }
    }

    /// <summary>A MultiLine of one line of 2d points, without tangents.</summary>
    public AppDef_MultiLine(NCollection_Array1<gp_Pnt2d> tabP2d)
    {
        tabMult = new NCollection_HArray1<AppDef_MultiPointConstraint>(1, tabP2d.Length());
        int i, Lower = tabP2d.Lower();
        for (i = 1; i <= tabP2d.Length(); i++)
        {
            AppDef_MultiPointConstraint MP = new(0, 1);
            MP.SetPoint2d(1, tabP2d[Lower + i - 1]);
            tabMult.SetValue(i, MP);
        }
    }

    /// <summary>The number of MultiPointConstraints.</summary>
    public int NbMultiPoints() => tabMult!.Length();

    /// <summary>The number of points (3d and 2d) of each MultiPointConstraint.</summary>
    public int NbPoints() => tabMult!.Value(1).NbPoints() + tabMult.Value(1).NbPoints2d();

    // AppDef_MultiLine.hxx declares SetParameter, but OCCT never defines it: left out.

    /// <summary>
    /// Sets the MultiPointConstraint of range <paramref name="Index"/>. Throws Standard_OutOfRange unless
    /// <paramref name="Index"/> is in 1..NbMultiPoints.
    /// </summary>
    public void SetValue(int Index, AppDef_MultiPointConstraint MPoint)
    {
        if ((Index <= 0) || (Index > tabMult!.Length()))
        {
            throw new Standard_OutOfRange();
        }
        tabMult.SetValue(Index, MPoint);
    }

    /// <summary>
    /// The MultiPointConstraint of range <paramref name="Index"/>: the stored one, whose arrays a C++ copy
    /// shares too. Throws Standard_OutOfRange unless <paramref name="Index"/> is in 1..NbMultiPoints.
    /// </summary>
    public AppDef_MultiPointConstraint Value(int Index)
    {
        if ((Index <= 0) || (Index > tabMult!.Length()))
        {
            throw new Standard_OutOfRange();
        }
        return tabMult.Value(Index);
    }

    /// <summary>Writes the number of MultiPointConstraints.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("AppDef_MultiLine dump:" + '\n');
        o.Write("It contains " + tabMult!.Length().ToString(CultureInfo.InvariantCulture) + " MultiPointConstraint" + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
