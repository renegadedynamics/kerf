// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>What the draft modification knows of a face: its new surface and the face it was added with.</summary>
/// <remarks>A value type in C++; a class in the port (decision 1033), as <see cref="Draft_EdgeInfo"/>.</remarks>
public sealed class Draft_FaceInfo
{
    private bool myNewGeom;
    private Surface? myGeom;
    private Face myRootFace = new();
    private Face myF1 = new();
    private Face myF2 = new();
    private Curve3D? myCurv;

    // OCCT leaves myNewGeom uninitialised in the default constructor; false here.
    public Draft_FaceInfo()
    {
    }

    public Draft_FaceInfo(Surface? S, bool HasNewGeometry)
    {
        myNewGeom = HasNewGeometry;
        if (S is TrimmedSurface T)
        {
            myGeom = T.BasisSurface();
        }
        else
        {
            myGeom = S;
        }
    }

    public void RootFace(in Face F) => myRootFace = F;

    public void Add(in Face F)
    {
        if (myF1.IsNull)
        {
            myF1 = F;
        }
        else if (myF2.IsNull)
        {
            myF2 = F;
        }
    }

    public Face FirstFace() => myF1;

    public Face SecondFace() => myF2;

    public bool NewGeometry() => myNewGeom;

    public Surface? Geometry() => myGeom;

    public ref Surface? ChangeGeometry() => ref myGeom;

    public Curve3D? Curve() => myCurv;

    public ref Curve3D? ChangeCurve() => ref myCurv;

    public Face RootFace() => myRootFace;
}
