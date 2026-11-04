// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiLine.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Lpa, le 3/12/91

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Approximation;

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
/// A class (decision 201): OCCT copies it by value, but its data are a handle, so a copy shares the
/// MultiPointConstraints, as a C# reference does. <see cref="SetValue"/> is public (decision 21): callers fill a
/// MultiLine before approximating it.
/// </remarks>
internal class MultiLine
{
    protected NCollection_HArray1<MultiPointConstraint>? _tabMult;

    /// <summary>An undefined MultiLine.</summary>
    public MultiLine() { }

    /// <summary>
    /// A MultiLine of <paramref name="nbMult"/> MultiPointConstraints, to be set with <see cref="SetValue"/>.
    /// Throws Standard_ConstructionError if <paramref name="nbMult"/> is negative.
    /// </summary>
    public MultiLine(int nbMult)
    {
        if (nbMult < 0)
        {
            throw new ConstructionErrorException();
        }

        _tabMult = new NCollection_HArray1<MultiPointConstraint>(1, nbMult);
    }

    /// <summary>A MultiLine of the MultiPointConstraints <paramref name="tabMultiP"/> (shared, as C++ copies share their arrays).</summary>
    public MultiLine(NCollection_Array1<MultiPointConstraint> tabMultiP)
    {
        _tabMult = new NCollection_HArray1<MultiPointConstraint>(1, tabMultiP.Length());
        int i, lower = tabMultiP.Lower();
        for (i = 1; i <= tabMultiP.Length(); i++)
        {
            _tabMult.SetValue(i, tabMultiP.Value(lower + i - 1));
        }
    }

    /// <summary>A MultiLine of one line of 3d points, without tangents.</summary>
    public MultiLine(NCollection_Array1<Point3D> tabP3d)
    {
        _tabMult = new NCollection_HArray1<MultiPointConstraint>(1, tabP3d.Length());
        int i, lower = tabP3d.Lower();
        for (i = 1; i <= tabP3d.Length(); i++)
        {
            MultiPointConstraint mp = new(1, 0);
            mp.SetPoint(1, tabP3d[lower + i - 1]);
            _tabMult.SetValue(i, mp);
        }
    }

    /// <summary>A MultiLine of one line of 2d points, without tangents.</summary>
    public MultiLine(NCollection_Array1<Point2D> tabP2d)
    {
        _tabMult = new NCollection_HArray1<MultiPointConstraint>(1, tabP2d.Length());
        int i, lower = tabP2d.Lower();
        for (i = 1; i <= tabP2d.Length(); i++)
        {
            MultiPointConstraint mp = new(0, 1);
            mp.SetPoint2d(1, tabP2d[lower + i - 1]);
            _tabMult.SetValue(i, mp);
        }
    }

    /// <summary>The number of MultiPointConstraints.</summary>
    public int MultiPointCount => _tabMult!.Length();

    /// <summary>The number of points (3d and 2d) of each MultiPointConstraint.</summary>
    public int PointCount => _tabMult!.Value(1).PointCount + _tabMult.Value(1).Points2dCount;

    // AppDef_MultiLine.hxx declares SetParameter, but OCCT never defines it: left out.

    /// <summary>
    /// Sets the MultiPointConstraint of range <paramref name="index"/>. Throws Standard_OutOfRange unless
    /// <paramref name="index"/> is in 1..NbMultiPoints.
    /// </summary>
    public void SetValue(int index, MultiPointConstraint mPoint)
    {
        if ((index <= 0) || (index > _tabMult!.Length()))
        {
            throw new OutOfRangeException();
        }
        _tabMult.SetValue(index, mPoint);
    }

    /// <summary>
    /// The MultiPointConstraint of range <paramref name="index"/>: the stored one, whose arrays a C++ copy
    /// shares too. Throws Standard_OutOfRange unless <paramref name="index"/> is in 1..NbMultiPoints.
    /// </summary>
    public MultiPointConstraint Value(int index)
    {
        if ((index <= 0) || (index > _tabMult!.Length()))
        {
            throw new OutOfRangeException();
        }
        return _tabMult.Value(index);
    }

    /// <summary>Writes the number of MultiPointConstraints.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("AppDef_MultiLine dump:" + '\n');
        o.Write("It contains " + _tabMult!.Length().ToString(CultureInfo.InvariantCulture) + " MultiPointConstraint" + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
