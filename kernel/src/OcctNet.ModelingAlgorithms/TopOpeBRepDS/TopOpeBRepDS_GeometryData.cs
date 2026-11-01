// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_GeometryData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_GeometryData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The list of interferences attached to a geometry of the data structure (base of the surface, curve and point data).</summary>
/// <remarks>Storage of <see cref="TopOpeBRepDS_DataStructure"/>; a working object.</remarks>
public class TopOpeBRepDS_GeometryData
{
    private readonly NCollection_List<TopOpeBRepDS_Interference> myInterferences = new();

    /// <summary>An empty list of interferences.</summary>
    public TopOpeBRepDS_GeometryData()
    {
    }

    /// <summary>A copy of <paramref name="Other"/>'s list (the interferences themselves are shared).</summary>
    public TopOpeBRepDS_GeometryData(TopOpeBRepDS_GeometryData Other)
    {
        Assign(Other);
    }

    internal void Assign(TopOpeBRepDS_GeometryData Other)
    {
        myInterferences.Clear();
        NCollection_List<TopOpeBRepDS_Interference>.Iterator anIt = new(Other.myInterferences);
        for (; anIt.More(); anIt.Next())
        {
            myInterferences.Append(anIt.Value());
        }
    }

    /// <summary>The interferences.</summary>
    /// <remarks>The list itself, which the data structure's builders change; callers outside OCCT.NET only read it.</remarks>
    public NCollection_List<TopOpeBRepDS_Interference> Interferences() => myInterferences;

    internal NCollection_List<TopOpeBRepDS_Interference> ChangeInterferences() => myInterferences;

    internal void AddInterference(TopOpeBRepDS_Interference I)
    {
        myInterferences.Append(I);
    }
}
