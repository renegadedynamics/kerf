// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_FaceInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>What the draft modification knows of a face: its new surface and the face it was added with.</summary>
/// <remarks>A value type in C++; a class in the port (decision 1033), as <see cref="Draft_EdgeInfo"/>.</remarks>
public sealed class Draft_FaceInfo
{
    private bool myNewGeom;
    private Geom_Surface? myGeom;
    private TopoDS_Face myRootFace = new();
    private TopoDS_Face myF1 = new();
    private TopoDS_Face myF2 = new();
    private Geom_Curve? myCurv;

    // OCCT leaves myNewGeom uninitialised in the default constructor; false here.
    public Draft_FaceInfo()
    {
    }

    public Draft_FaceInfo(Geom_Surface? S, bool HasNewGeometry)
    {
        myNewGeom = HasNewGeometry;
        if (S is Geom_RectangularTrimmedSurface T)
        {
            myGeom = T.BasisSurface();
        }
        else
        {
            myGeom = S;
        }
    }

    public void RootFace(in TopoDS_Face F) => myRootFace = F;

    public void Add(in TopoDS_Face F)
    {
        if (myF1.IsNull())
        {
            myF1 = F;
        }
        else if (myF2.IsNull())
        {
            myF2 = F;
        }
    }

    public TopoDS_Face FirstFace() => myF1;

    public TopoDS_Face SecondFace() => myF2;

    public bool NewGeometry() => myNewGeom;

    public Geom_Surface? Geometry() => myGeom;

    public ref Geom_Surface? ChangeGeometry() => ref myGeom;

    public Geom_Curve? Curve() => myCurv;

    public ref Geom_Curve? ChangeCurve() => ref myCurv;

    public TopoDS_Face RootFace() => myRootFace;
}
