// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>What the draft modification knows of an edge: its new curve, its two faces and their new pcurves.</summary>
/// <remarks>
/// A value type in C++, held by value in Draft_Modification's map; a class in the port (decision 1033): the
/// modification creates one per edge and changes it only through the map, as OCCT's <c>ChangeFromKey</c> does.
/// </remarks>
public sealed class Draft_EdgeInfo
{
    private bool myNewGeom;
    private Geom_Curve? myGeom;
    private TopoDS_Face myFirstF = new();
    private TopoDS_Face mySeconF = new();
    private Geom2d_Curve? myFirstPC;
    private Geom2d_Curve? mySeconPC;
    private TopoDS_Face myRootFace = new();
    private bool myTgt;
    private Point3D myPt = new();
    private double myTol;

    public Draft_EdgeInfo()
    {
        myNewGeom = false;
        myTgt = false;
        myTol = 0;
    }

    public Draft_EdgeInfo(bool HasNewGeometry)
    {
        myNewGeom = HasNewGeometry;
        myTgt = false;
        myTol = 0;
    }

    public void Add(in TopoDS_Face F)
    {
        if (myFirstF.IsNull())
        {
            myFirstF = F;
        }
        else if (!myFirstF.IsSame(F) && mySeconF.IsNull())
        {
            mySeconF = F;
        }
        myTol = Max(myTol, BRep_Tool.Tolerance(F));
    }

    public void RootFace(in TopoDS_Face F) => myRootFace = F;

    public void Tangent(in Point3D P)
    {
        myTgt = true;
        myPt = P;
    }

    public bool IsTangent(out Point3D P)
    {
        P = myPt;
        return myTgt;
    }

    public bool NewGeometry() => myNewGeom;

    public void SetNewGeometry(bool NewGeom) => myNewGeom = NewGeom;

    public Geom_Curve? Geometry() => myGeom;

    public TopoDS_Face FirstFace() => myFirstF;

    public TopoDS_Face SecondFace() => mySeconF;

    public Geom2d_Curve? FirstPC() => myFirstPC;

    public Geom2d_Curve? SecondPC() => mySeconPC;

    public ref Geom_Curve? ChangeGeometry() => ref myGeom;

    public ref Geom2d_Curve? ChangeFirstPC() => ref myFirstPC;

    public ref Geom2d_Curve? ChangeSecondPC() => ref mySeconPC;

    public TopoDS_Face RootFace() => myRootFace;

    public void Tolerance(double tol) => myTol = tol;

    public double Tolerance() => myTol;
}
