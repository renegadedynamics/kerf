// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Interf.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An interference between two shapes: their indices in the data structure and the index of the new shape it made.
/// The root of the interference classes.
/// </summary>
/// <remarks>
/// The interference classes are classes, working objects held in the data structure's vectors and changed in place
/// (decision 631): <c>NCollection_DynamicArray&lt;T&gt;.Appended()</c> appends null for a class, so ported code
/// writes <c>aVVs.Append(new BOPDS_InterfVV())</c> where OCCT writes <c>aVVs.Appended()</c>. OCCT's
/// <c>std::optional&lt;int&gt; GetIndexNew()</c> is <c>int?</c>.
/// </remarks>
public abstract class BOPDS_Interf
{
    protected int myIndex1;
    protected int myIndex2;
    protected int myIndexNew;

    protected BOPDS_Interf()
    {
        myIndex1 = -1;
        myIndex2 = -1;
        myIndexNew = -1;
    }

    /// <summary>Sets the indices of the interfering shapes.</summary>
    internal void SetIndices(int theIndex1, int theIndex2)
    {
        myIndex1 = theIndex1;
        myIndex2 = theIndex2;
    }

    /// <summary>The indices of the interfering shapes.</summary>
    public void Indices(out int theIndex1, out int theIndex2)
    {
        theIndex1 = myIndex1;
        theIndex2 = myIndex2;
    }

    internal void SetIndex1(int theIndex) => myIndex1 = theIndex;

    internal void SetIndex2(int theIndex) => myIndex2 = theIndex;

    public int Index1() => myIndex1;

    public int Index2() => myIndex2;

    /// <summary>The index of the other shape than <paramref name="theI"/>, or -1 if neither is <paramref name="theI"/>.</summary>
    public int OppositeIndex(int theI)
    {
        if (theI == myIndex1)
        {
            return myIndex2;
        }
        else if (theI == myIndex2)
        {
            return myIndex1;
        }
        else
        {
            return -1;
        }
    }

    /// <summary>True if the interference involves the shape.</summary>
    public bool Contains(int theIndex) => myIndex1 == theIndex || myIndex2 == theIndex;

    internal void SetIndexNew(int theIndex) => myIndexNew = theIndex;

    /// <summary>The index of the new shape.</summary>
    public int IndexNew() => myIndexNew;

    /// <summary>True if the interference has a new shape; its index then.</summary>
    public bool HasIndexNew(out int theIndex)
    {
        theIndex = myIndexNew;
        return myIndexNew >= 0;
    }

    /// <summary>True if the interference has a new shape (an index other than -1).</summary>
    public bool HasIndexNew() => myIndexNew != -1;

    /// <summary>The index of the new shape, or null (OCCT's <c>std::optional</c>).</summary>
    public int? GetIndexNew() => myIndexNew != -1 ? myIndexNew : null;
}

/// <summary>A vertex/vertex interference.</summary>
public class BOPDS_InterfVV : BOPDS_Interf
{
}

/// <summary>A vertex/edge interference, with the vertex's parameter on the edge's curve.</summary>
public class BOPDS_InterfVE : BOPDS_Interf
{
    protected double myParameter;

    public BOPDS_InterfVE()
    {
        myParameter = 0.0;
    }

    internal void SetParameter(double theT) => myParameter = theT;

    /// <summary>The parameter of the vertex's point on the edge's curve.</summary>
    public double Parameter() => myParameter;
}

/// <summary>A vertex/face interference, with the vertex's parameters on the face's surface.</summary>
public class BOPDS_InterfVF : BOPDS_Interf
{
    protected double myU;
    protected double myV;

    public BOPDS_InterfVF()
    {
        myU = 0.0;
        myV = 0.0;
    }

    internal void SetUV(double theU, double theV)
    {
        myU = theU;
        myV = theV;
    }

    /// <summary>The parameters of the vertex's point on the face's surface.</summary>
    public void UV(out double theU, out double theV)
    {
        theU = myU;
        theV = myV;
    }
}

/// <summary>An edge/edge interference, with its common part.</summary>
public class BOPDS_InterfEE : BOPDS_Interf
{
    protected IntTools_CommonPrt myCommonPart = new();

    /// <summary>Sets the common part (OCCT's assignment: see <see cref="IntTools_CommonPrt"/>).</summary>
    internal void SetCommonPart(IntTools_CommonPrt theCP) => myCommonPart.Assign(theCP);

    /// <summary>The common part (the interference's own object; read only for callers).</summary>
    public IntTools_CommonPrt CommonPart() => myCommonPart;
}

/// <summary>An edge/face interference, with its common part.</summary>
public class BOPDS_InterfEF : BOPDS_Interf
{
    protected IntTools_CommonPrt myCommonPart = new();

    /// <summary>Sets the common part (OCCT's assignment: see <see cref="IntTools_CommonPrt"/>).</summary>
    internal void SetCommonPart(IntTools_CommonPrt theCP) => myCommonPart.Assign(theCP);

    /// <summary>The common part (the interference's own object; read only for callers).</summary>
    public IntTools_CommonPrt CommonPart() => myCommonPart;
}

/// <summary>A face/face interference: the intersection curves and points, and whether the faces are tangent.</summary>
public class BOPDS_InterfFF : BOPDS_Interf
{
    protected bool myTangentFaces;
    protected NCollection_DynamicArray<BOPDS_Curve> myCurves = new(0);
    protected NCollection_DynamicArray<BOPDS_Point> myPoints = new(0);

    public BOPDS_InterfFF()
    {
        myTangentFaces = false;
    }

    /// <summary>
    /// Sets the expected numbers of curves and points. OCCT sets the vectors' allocation increments; the port's
    /// vectors grow by themselves, so this does nothing.
    /// </summary>
    internal void Init(int theNbCurves, int theNbPoints)
    {
    }

    internal void SetTangentFaces(bool theFlag) => myTangentFaces = theFlag;

    /// <summary>True if the faces are tangent.</summary>
    public bool TangentFaces() => myTangentFaces;

    /// <summary>The intersection curves (the interference's own vector; read only for callers).</summary>
    public NCollection_DynamicArray<BOPDS_Curve> Curves() => myCurves;

    internal NCollection_DynamicArray<BOPDS_Curve> ChangeCurves() => myCurves;

    /// <summary>The intersection points (the interference's own vector; read only for callers).</summary>
    public NCollection_DynamicArray<BOPDS_Point> Points() => myPoints;

    internal NCollection_DynamicArray<BOPDS_Point> ChangePoints() => myPoints;
}

/// <summary>A vertex/solid interference.</summary>
public class BOPDS_InterfVZ : BOPDS_Interf
{
}

/// <summary>An edge/solid interference.</summary>
public class BOPDS_InterfEZ : BOPDS_Interf
{
}

/// <summary>A face/solid interference.</summary>
public class BOPDS_InterfFZ : BOPDS_Interf
{
}

/// <summary>A solid/solid interference.</summary>
public class BOPDS_InterfZZ : BOPDS_Interf
{
}
