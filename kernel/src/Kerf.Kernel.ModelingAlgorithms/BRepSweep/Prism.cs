// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Prism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Builds the linear sweep of a shape: a prism along a vector, or a semi-infinite or infinite prism along a direction.</summary>
internal sealed class Prism
{
    private readonly Translation _translation;

    /// <summary>The prism of base <paramref name="s"/> and vector <paramref name="v"/>; with <paramref name="copy"/> S is copied.</summary>
    public Prism(in Shape s, in Vector3D v, bool copy = false, bool canonize = true)
    {
        _translation = new Translation(s, NumShape(), Location(v), v, copy, canonize);
        Standard_ConstructionError_Raise_if(v.Magnitude <= global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Prism::Constructor");
    }

    /// <summary>A semi-infinite (<paramref name="inf"/> false) or infinite prism of base <paramref name="s"/>.</summary>
    public Prism(in Shape s, in Direction3D d, bool inf = true, bool copy = false, bool canonize = true)
    {
        _translation = new Translation(s, NumShape(inf), Location(new Vector3D(d)), new Vector3D(d), copy, canonize);
    }

    public Shape Shape() => _translation.Shape();

    public Shape Shape(in Shape aGenS) => _translation.Shape(aGenS);

    public Shape FirstShape() => _translation.FirstShape();

    public Shape FirstShape(in Shape aGenS) => _translation.FirstShape(aGenS);

    public Shape LastShape() => _translation.LastShape();

    public Shape LastShape(in Shape aGenS) => _translation.LastShape(aGenS);

    public Vector3D Vec() => _translation.Vec();

    private static NumShape NumShape() => new(2, ShapeType.Edge);

    private static NumShape NumShape(bool inf)
    {
        var n = new NumShape();
        if (inf)
        {
            n.Init(0, ShapeType.Edge, false, true, true);
        }
        else
        {
            n.Init(1, ShapeType.Edge, false, false, true);
        }
        return n;
    }

    private static Placement Location(in Vector3D v)
    {
        var gpt = new Transform3D();
        gpt = gpt.WithTranslation(v);
        var l = new Placement(gpt);
        return l;
    }

    /// <summary>True if the initial shape aGenS is used in the result shape.</summary>
    public bool IsUsed(in Shape aGenS) => _translation.IsUsed(aGenS);

    /// <summary>True if the shape generated from aGenS is used in the result shape.</summary>
    public bool GenIsUsed(in Shape aGenS) => _translation.GenIsUsed(aGenS);
}
