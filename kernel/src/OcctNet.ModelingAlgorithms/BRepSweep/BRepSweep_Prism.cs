// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds the linear sweep of a shape: a prism along a vector, or a semi-infinite or infinite prism along a direction.</summary>
internal sealed class BRepSweep_Prism
{
    private readonly BRepSweep_Translation myTranslation;

    /// <summary>The prism of base <paramref name="S"/> and vector <paramref name="V"/>; with <paramref name="Copy"/> S is copied.</summary>
    public BRepSweep_Prism(in TopoDS_Shape S, in Vector3D V, bool Copy = false, bool Canonize = true)
    {
        myTranslation = new BRepSweep_Translation(S, NumShape(), Location(V), V, Copy, Canonize);
        Standard_ConstructionError_Raise_if(V.Magnitude() <= global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Prism::Constructor");
    }

    /// <summary>A semi-infinite (<paramref name="Inf"/> false) or infinite prism of base <paramref name="S"/>.</summary>
    public BRepSweep_Prism(in TopoDS_Shape S, in Direction3D D, bool Inf = true, bool Copy = false, bool Canonize = true)
    {
        myTranslation = new BRepSweep_Translation(S, NumShape(Inf), Location(new Vector3D(D)), new Vector3D(D), Copy, Canonize);
    }

    public TopoDS_Shape Shape() => myTranslation.Shape();

    public TopoDS_Shape Shape(in TopoDS_Shape aGenS) => myTranslation.Shape(aGenS);

    public TopoDS_Shape FirstShape() => myTranslation.FirstShape();

    public TopoDS_Shape FirstShape(in TopoDS_Shape aGenS) => myTranslation.FirstShape(aGenS);

    public TopoDS_Shape LastShape() => myTranslation.LastShape();

    public TopoDS_Shape LastShape(in TopoDS_Shape aGenS) => myTranslation.LastShape(aGenS);

    public Vector3D Vec() => myTranslation.Vec();

    private static Sweep_NumShape NumShape() => new(2, TopAbs_EDGE);

    private static Sweep_NumShape NumShape(bool Inf)
    {
        var N = new Sweep_NumShape();
        if (Inf)
        {
            N.Init(0, TopAbs_EDGE, false, true, true);
        }
        else
        {
            N.Init(1, TopAbs_EDGE, false, false, true);
        }
        return N;
    }

    private static Placement Location(in Vector3D V)
    {
        var gpt = new Transform3D();
        gpt.SetTranslation(V);
        var L = new Placement(gpt);
        return L;
    }

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in TopoDS_Shape aGenS) => myTranslation.IsUsed(aGenS);

    /// <summary>True if the shape generated from aGenS is used in the result shape.</summary>
    public bool GenIsUsed(in TopoDS_Shape aGenS) => myTranslation.GenIsUsed(aGenS);
}
