// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Intf_PIType;

namespace Occt;

/// <summary>
/// An intersection point between polygons and polyhedra: its location, and on each argument its type (vertex,
/// edge, face), its addresses in the argument and its parameter.
/// </summary>
/// <remarks>
/// OCCT's default is all zeros (Intf_EXTERNAL), as <c>default(Intf_SectionPoint)</c>. OCCT's <c>operator==</c> is
/// <see cref="IsEqual"/>, which compares the logical information only; <c>==</c>, <c>Equals</c> and
/// <c>GetHashCode</c> compare every field exactly (decision 3403). <see cref="Merge"/> is an internal mutator.
/// OCCT's <c>Dump</c> prints only in a debug build and is left out.
/// </remarks>
public partial struct Intf_SectionPoint : IEquatable<Intf_SectionPoint>
{
    private gp_Pnt myPnt;
    private Intf_PIType DimenObje;
    private int IndexO1;
    private int IndexO2;
    private double ParamObje;
    private Intf_PIType DimenTool;
    private int IndexT1;
    private int IndexT2;
    private double ParamTool;
    private double Incide;

    public Intf_SectionPoint()
    {
        myPnt = new gp_Pnt(0.0, 0.0, 0.0);
        DimenObje = Intf_EXTERNAL;
        IndexO1 = 0;
        IndexO2 = 0;
        ParamObje = 0.0;
        DimenTool = Intf_EXTERNAL;
        IndexT1 = 0;
        IndexT2 = 0;
        ParamTool = 0.0;
        Incide = 0.0;
    }

    /// <summary>
    /// A section point with the respective dimensions (vertex, edge or face) of the concerned arguments and their
    /// addresses in the topological structure.
    /// </summary>
    public Intf_SectionPoint(in gp_Pnt Where,
                             Intf_PIType Dim1,
                             int Addr1,
                             int Addr2,
                             double Param1,
                             Intf_PIType Dim2,
                             int Addr3,
                             int Addr4,
                             double Param2,
                             double Incid)
    {
        myPnt = Where;
        DimenObje = Dim1;
        IndexO1 = Addr1;
        IndexO2 = Addr2;
        ParamObje = Param1;
        DimenTool = Dim2;
        IndexT1 = Addr3;
        IndexT2 = Addr4;
        ParamTool = Param2;
        Incide = Incid;
    }

    /// <summary>
    /// A 2D section point with the respective dimensions (vertex or edge) of the concerned arguments and their
    /// addresses in the topological structure.
    /// </summary>
    public Intf_SectionPoint(in gp_Pnt2d Where,
                             Intf_PIType Dim1,
                             int Addr1,
                             double Param1,
                             Intf_PIType Dim2,
                             int Addr2,
                             double Param2,
                             double Incid)
    {
        myPnt = new gp_Pnt(Where.X(), Where.Y(), 0.0);
        DimenObje = Dim1;
        IndexO1 = 0;
        IndexO2 = Addr1;
        ParamObje = Param1;
        DimenTool = Dim2;
        IndexT1 = 0;
        IndexT2 = Addr2;
        ParamTool = Param2;
        Incide = Incid;
    }

    /// <summary>The location of the section point.</summary>
    public readonly gp_Pnt Pnt() => myPnt;

    /// <summary>The cumulated parameter of the section point on the first element.</summary>
    public readonly double ParamOnFirst() => (IndexO2 - 1) + ParamObje;

    /// <summary>The cumulated parameter of the section point on the second element.</summary>
    public readonly double ParamOnSecond() => (IndexT2 - 1) + ParamTool;

    /// <summary>The type of the section point on the first element.</summary>
    public readonly Intf_PIType TypeOnFirst() => DimenObje;

    /// <summary>The type of the section point on the second element.</summary>
    public readonly Intf_PIType TypeOnSecond() => DimenTool;

    public readonly void InfoFirst(out Intf_PIType Dim, out int Add1, out int Add2, out double Param)
    {
        Dim = DimenObje;
        Add1 = IndexO1;
        Add2 = IndexO2;
        Param = ParamObje;
    }

    /// <summary>The data about the first argument of the interference.</summary>
    public readonly void InfoFirst(out Intf_PIType Dim, out int Add, out double Param)
    {
        Dim = DimenObje;
        Add = IndexO2;
        Param = ParamObje;
    }

    public readonly void InfoSecond(out Intf_PIType Dim, out int Add1, out int Add2, out double Param)
    {
        Dim = DimenTool;
        Add1 = IndexT1;
        Add2 = IndexT2;
        Param = ParamTool;
    }

    /// <summary>The data about the second argument of the interference.</summary>
    public readonly void InfoSecond(out Intf_PIType Dim, out int Add, out double Param)
    {
        Dim = DimenTool;
        Add = IndexT2;
        Param = ParamTool;
    }

    /// <summary>
    /// The incidence at this section point: the cosine between the two triangles. The best incidence is 0 (PI/2),
    /// the worst 1 (null angle).
    /// </summary>
    public readonly double Incidence() => Incide;

    /// <summary>True if the two section points have the same logical information (types and addresses).</summary>
    public readonly bool IsEqual(in Intf_SectionPoint Other)
        => (DimenObje == Other.DimenObje && IndexO1 == Other.IndexO1 && IndexO2 == Other.IndexO2
            && DimenTool == Other.DimenTool && IndexT1 == Other.IndexT1 && IndexT2 == Other.IndexT2);

    /// <summary>True if the two section points are on the same edge of the first or the second element.</summary>
    public readonly bool IsOnSameEdge(in Intf_SectionPoint Other)
    {
        bool isOn = false;
        if (DimenObje == Intf_EDGE)
        {
            if (Other.DimenObje == Intf_EDGE)
            {
                isOn = (IndexO1 == Other.IndexO1 && IndexO2 == Other.IndexO2);
            }
            else if (Other.DimenObje == Intf_VERTEX)
            {
                isOn = (IndexO1 == Other.IndexO1 || IndexO2 == Other.IndexO1);
            }
        }
        else if (DimenObje == Intf_VERTEX)
        {
            if (Other.DimenObje == Intf_EDGE)
            {
                isOn = (IndexO1 == Other.IndexO1 || IndexO1 == Other.IndexO2);
            }
            else if (Other.DimenObje == Intf_VERTEX)
            {
                // OCCT: compares the second argument's addresses (IndexT1) on the first argument's vertices
                isOn = (IndexT1 == Other.IndexT1);
            }
        }

        if (!isOn)
        {
            if (DimenTool == Intf_EDGE)
            {
                if (Other.DimenTool == Intf_EDGE)
                {
                    isOn = (IndexT1 == Other.IndexT1 && IndexT2 == Other.IndexT2);
                }
                else if (Other.DimenTool == Intf_VERTEX)
                {
                    isOn = (IndexT1 == Other.IndexT1 || IndexT2 == Other.IndexT1);
                }
            }
            else if (DimenTool == Intf_VERTEX)
            {
                if (Other.DimenTool == Intf_EDGE)
                {
                    isOn = (IndexT1 == Other.IndexT1 || IndexT1 == Other.IndexT2);
                }
                else if (Other.DimenTool == Intf_VERTEX)
                {
                    isOn = (IndexT1 == Other.IndexT1);
                }
            }
        }

        return isOn;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Merges two section points: both get this point's location, and on each argument the data of the point of
    /// higher dimension.
    /// </summary>
    internal void Merge(ref Intf_SectionPoint Other)
    {
        Other.myPnt = myPnt;
        if (DimenObje >= Other.DimenObje)
        {
            Other.DimenObje = DimenObje;
            Other.IndexO1 = IndexO1;
            Other.IndexO2 = IndexO2;
            Other.ParamObje = ParamObje;
        }
        else
        {
            DimenObje = Other.DimenObje;
            IndexO1 = Other.IndexO1;
            IndexO2 = Other.IndexO2;
            ParamObje = Other.ParamObje;
        }

        if (DimenTool >= Other.DimenTool)
        {
            Other.DimenTool = DimenTool;
            Other.IndexT1 = IndexT1;
            Other.IndexT2 = IndexT2;
            Other.ParamTool = ParamTool;
        }
        else
        {
            DimenTool = Other.DimenTool;
            IndexT1 = Other.IndexT1;
            IndexT2 = Other.IndexT2;
            ParamTool = Other.ParamTool;
        }
    }

    public readonly bool Equals(Intf_SectionPoint other)
        => myPnt.Equals(other.myPnt) && DimenObje == other.DimenObje && IndexO1 == other.IndexO1
           && IndexO2 == other.IndexO2 && ParamObje.Equals(other.ParamObje) && DimenTool == other.DimenTool
           && IndexT1 == other.IndexT1 && IndexT2 == other.IndexT2 && ParamTool.Equals(other.ParamTool)
           && Incide.Equals(other.Incide);

    public override readonly bool Equals(object? obj) => obj is Intf_SectionPoint other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(myPnt);
        aHash.Add(DimenObje);
        aHash.Add(IndexO1);
        aHash.Add(IndexO2);
        aHash.Add(ParamObje);
        aHash.Add(DimenTool);
        aHash.Add(IndexT1);
        aHash.Add(IndexT2);
        aHash.Add(ParamTool);
        aHash.Add(Incide);
        return aHash.ToHashCode();
    }

    public static bool operator ==(in Intf_SectionPoint theLeft, in Intf_SectionPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Intf_SectionPoint theLeft, in Intf_SectionPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"Intf_SectionPoint({DimenObje} {IndexO1} {IndexO2} {ParamObje.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, "
           + $"{DimenTool} {IndexT1} {IndexT2} {ParamTool.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, "
           + gp.Format("at", myPnt.X(), myPnt.Y(), myPnt.Z(), Incide) + ")";
}
