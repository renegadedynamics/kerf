// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurAndInf.hxx
// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurAndInf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.LProp_CIType;

namespace Occt;

/// <summary>
/// The parameters of a 2D or 3D curve where its curvature is extreme and where it inflects, in increasing order.
/// </summary>
/// <remarks>A working object, filled by <see cref="GeomLProp_CurAndInf2d"/>: its methods are public, as in OCCT.</remarks>
public class LProp_CurAndInf
{
    private readonly NCollection_Sequence<double> theParams = new();
    private readonly NCollection_Sequence<LProp_CIType> theTypes = new();

    public LProp_CurAndInf()
    {
    }

    /// <summary>Adds an inflection point at the parameter, keeping the parameters sorted.</summary>
    public void AddInflection(double Param)
    {
        if (theParams.IsEmpty())
        {
            theParams.Append(Param);
            theTypes.Append(LProp_Inflection);
            return;
        }

        if (Param > theParams.Last())
        {
            theParams.Append(Param);
            theTypes.Append(LProp_Inflection);
            return;
        }

        for (int i = 1; i <= theParams.Length(); i++)
        {
            if (theParams.Value(i) > Param)
            {
                theParams.InsertBefore(i, Param);
                theTypes.InsertBefore(i, LProp_Inflection);
                break;
            }
        }
    }

    /// <summary>
    /// Adds an extremum of curvature at the parameter (a minimum of the radius of curvature if
    /// <paramref name="IsMin"/>), keeping the parameters sorted.
    /// </summary>
    public void AddExtCur(double Param, bool IsMin)
    {
        LProp_CIType TypePoint;
        if (IsMin)
        {
            TypePoint = LProp_MinCur;
        }
        else
        {
            TypePoint = LProp_MaxCur;
        }

        if (theParams.IsEmpty())
        {
            theParams.Append(Param);
            theTypes.Append(TypePoint);
            return;
        }

        if (Param > theParams.Last())
        {
            theParams.Append(Param);
            theTypes.Append(TypePoint);
            return;
        }

        for (int i = 1; i <= theParams.Length(); i++)
        {
            if (theParams.Value(i) > Param)
            {
                theParams.InsertBefore(i, Param);
                theTypes.InsertBefore(i, TypePoint);
                break;
            }
        }
    }

    public void Clear()
    {
        theParams.Clear();
        theTypes.Clear();
    }

    public bool IsEmpty()
    {
        return theParams.IsEmpty();
    }

    /// <summary>The number of points, stored by increasing parameter.</summary>
    public int NbPoints()
    {
        return theParams.Length();
    }

    /// <summary>The parameter of the Nth point; throws Standard_OutOfRange if N is not in [1, NbPoints()].</summary>
    public double Parameter(int N)
    {
        if (N < 1 || N > NbPoints())
        {
            throw new Standard_OutOfRange();
        }

        return theParams.Value(N);
    }

    /// <summary>
    /// The type of the Nth point: MinCur at a minimum of the radius of curvature, MaxCur at a maximum, Inflection at
    /// an inflection point. Throws Standard_OutOfRange if N is not in [1, NbPoints()].
    /// </summary>
    public LProp_CIType Type(int N)
    {
        if (N < 1 || N > NbPoints())
        {
            throw new Standard_OutOfRange();
        }

        return theTypes.Value(N);
    }
}
