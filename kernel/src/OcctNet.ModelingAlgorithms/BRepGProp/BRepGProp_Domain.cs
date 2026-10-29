// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Arc iterator over the boundary of a face: the edges of the face, forward, skipping INTERNAL and EXTERNAL edges after
/// the first, in an undigested order.
/// </summary>
/// <remarks>A working object, as OCCT's (an explorer): its iteration methods change it.</remarks>
public sealed class BRepGProp_Domain
{
    private readonly TopExp_Explorer myExplorer;

    /// <summary>An empty domain.</summary>
    public BRepGProp_Domain() => myExplorer = new TopExp_Explorer();

    /// <summary>The domain of the face.</summary>
    public BRepGProp_Domain(in TopoDS_Face F) => myExplorer = new TopExp_Explorer(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE);

    /// <summary>Initializes the domain with the face.</summary>
    public void Init(in TopoDS_Face F) => myExplorer.Init(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE);

    /// <summary>True if there is another arc of curve in the list.</summary>
    public bool More() => myExplorer.More();

    /// <summary>Restarts the exploration of the face already set.</summary>
    public void Init() => myExplorer.ReInit();

    /// <summary>The current edge.</summary>
    public TopoDS_Edge Value() => TopoDS.Edge(myExplorer.Current());

    /// <summary>Moves to the next FORWARD or REVERSED edge.</summary>
    public void Next()
    {
        // skip INTERNAL and EXTERNAL edges
        myExplorer.Next();
        while (myExplorer.More())
        {
            TopAbs_Orientation Or = myExplorer.Current().Orientation();
            if ((Or == TopAbs_FORWARD) || (Or == TopAbs_REVERSED))
            {
                return;
            }

            myExplorer.Next();
        }
    }
}
