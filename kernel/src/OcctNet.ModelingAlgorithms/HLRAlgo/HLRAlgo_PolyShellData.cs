// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyShellData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyShellData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The face data of a shell for the polygonal hidden-line removal, its hiding faces and its segments.</summary>
/// <remarks>A working object (a transient), as OCCT's.</remarks>
public class HLRAlgo_PolyShellData : Standard_Transient
{
    /// <summary>The shell's min-max indices (plain data).</summary>
    public struct ShellIndices
    {
        public int Min;
        public int Max;
    }

    private ShellIndices myIndices;
    private readonly NCollection_Array1<HLRAlgo_PolyData?> myPolyg;
    private NCollection_Array1<HLRAlgo_PolyData?> myHPolHi;
    private readonly NCollection_List<HLRAlgo_BiPoint> mySegList = new();

    public HLRAlgo_PolyShellData(int nbFace)
    {
        myPolyg = new NCollection_Array1<HLRAlgo_PolyData?>(1, nbFace);
        myHPolHi = new NCollection_Array1<HLRAlgo_PolyData?>();
    }

    /// <summary>Adds the segments' projected points and the faces' hiding nodes to <paramref name="theBox"/>.</summary>
    public void UpdateGlobalMinMax(ref Bnd_Box theBox)
    {
        for (NCollection_List<HLRAlgo_BiPoint>.Iterator it = new(mySegList); it.More(); it.Next())
        {
            HLRAlgo_BiPoint.PointsT aPoints = it.Value().PointsValue();
            theBox.Update(aPoints.PntP1.X(), aPoints.PntP1.Y(), aPoints.PntP1.Z());
            theBox.Update(aPoints.PntP2.X(), aPoints.PntP2.Y(), aPoints.PntP2.Z());
        }
        for (int i = myPolyg.Lower(); i <= myPolyg.Upper(); i++)
        {
            HLRAlgo_PolyData aPd = myPolyg[i]!;
            aPd.UpdateGlobalMinMax(ref theBox);
        }
    }

    /// <summary>Sizes the array of hiding faces to <paramref name="nbHiding"/> (empty if none).</summary>
    public void UpdateHiding(int nbHiding)
    {
        if (nbHiding > 0)
        {
            myHPolHi.Resize(1, nbHiding, false);
        }
        else
        {
            myHPolHi = new NCollection_Array1<HLRAlgo_PolyData?>();
        }
    }

    public bool Hiding() => !myHPolHi.IsEmpty();

    public NCollection_Array1<HLRAlgo_PolyData?> PolyData() => myPolyg;

    public NCollection_Array1<HLRAlgo_PolyData?> HidingPolyData() => myHPolHi;

    public NCollection_List<HLRAlgo_BiPoint> Edges() => mySegList;

    public ref ShellIndices Indices() => ref myIndices;
}
