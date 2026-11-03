// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Builds the linear sweep of a shape: a prism along a vector, or a semi-infinite or infinite prism along a direction.</summary>
internal sealed class BRepSweep_Prism
{
    private readonly BRepSweep_Translation myTranslation;

    /// <summary>The prism of base <paramref name="S"/> and vector <paramref name="V"/>; with <paramref name="Copy"/> S is copied.</summary>
    public BRepSweep_Prism(in Shape S, in Vector3D V, bool Copy = false, bool Canonize = true)
    {
        myTranslation = new BRepSweep_Translation(S, NumShape(), Location(V), V, Copy, Canonize);
        Standard_ConstructionError_Raise_if(V.Magnitude() <= global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Prism::Constructor");
    }

    /// <summary>A semi-infinite (<paramref name="Inf"/> false) or infinite prism of base <paramref name="S"/>.</summary>
    public BRepSweep_Prism(in Shape S, in Direction3D D, bool Inf = true, bool Copy = false, bool Canonize = true)
    {
        myTranslation = new BRepSweep_Translation(S, NumShape(Inf), Location(new Vector3D(D)), new Vector3D(D), Copy, Canonize);
    }

    public Shape Shape() => myTranslation.Shape();

    public Shape Shape(in Shape aGenS) => myTranslation.Shape(aGenS);

    public Shape FirstShape() => myTranslation.FirstShape();

    public Shape FirstShape(in Shape aGenS) => myTranslation.FirstShape(aGenS);

    public Shape LastShape() => myTranslation.LastShape();

    public Shape LastShape(in Shape aGenS) => myTranslation.LastShape(aGenS);

    public Vector3D Vec() => myTranslation.Vec();

    private static Sweep_NumShape NumShape() => new(2, ShapeType.Edge);

    private static Sweep_NumShape NumShape(bool Inf)
    {
        var N = new Sweep_NumShape();
        if (Inf)
        {
            N.Init(0, ShapeType.Edge, false, true, true);
        }
        else
        {
            N.Init(1, ShapeType.Edge, false, false, true);
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
    public bool IsUsed(in Shape aGenS) => myTranslation.IsUsed(aGenS);

    /// <summary>True if the shape generated from aGenS is used in the result shape.</summary>
    public bool GenIsUsed(in Shape aGenS) => myTranslation.GenIsUsed(aGenS);
}
