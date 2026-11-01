// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopOpeBRepDS_Config;

namespace Occt;

/// <summary>
/// The data the data structure keeps for a shape: its interferences, its same-domain shapes with their reference,
/// orientation and index, its ancestor rank, and whether it is kept.
/// </summary>
/// <remarks>Storage of <see cref="TopOpeBRepDS_DataStructure"/> (a class, so its map gives it for writing).</remarks>
public sealed class TopOpeBRepDS_ShapeData
{
    internal readonly NCollection_List<TopOpeBRepDS_Interference> myInterferences = new();
    internal readonly NCollection_List<TopoDS_Shape> mySameDomain = new();
    internal int mySameDomainRef;
    internal TopOpeBRepDS_Config mySameDomainOri;
    internal int mySameDomainInd;
    internal TopAbs_Orientation myOrientation;
    internal bool myOrientationDef;
    internal int myAncestorRank;
    private bool myKeep;

    /// <summary>Empty data: no interferences, no same domain, kept.</summary>
    public TopOpeBRepDS_ShapeData()
    {
        mySameDomainRef = 0;
        mySameDomainOri = TopOpeBRepDS_UNSHGEOMETRY;
        mySameDomainInd = 0;
        myOrientation = TopAbs_FORWARD;
        myOrientationDef = false;
        myAncestorRank = 0;
        myKeep = true;
    }

    /// <summary>The interferences.</summary>
    public NCollection_List<TopOpeBRepDS_Interference> Interferences() => myInterferences;

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeInterferences() => myInterferences;

    /// <summary>Whether the shape is kept.</summary>
    public bool Keep() => myKeep;

    internal void ChangeKeep(bool b)
    {
        myKeep = b;
    }
}
