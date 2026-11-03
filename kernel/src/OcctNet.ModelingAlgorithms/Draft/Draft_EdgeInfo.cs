// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_EdgeInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>What the draft modification knows of an edge: its new curve, its two faces and their new pcurves.</summary>
/// <remarks>
/// A value type in C++, held by value in Draft_Modification's map; a class in the port (decision 1033): the
/// modification creates one per edge and changes it only through the map, as OCCT's <c>ChangeFromKey</c> does.
/// </remarks>
public sealed class Draft_EdgeInfo
{
    private bool myNewGeom;
    private Curve3D? myGeom;
    private Face myFirstF = new();
    private Face mySeconF = new();
    private Curve2D? myFirstPC;
    private Curve2D? mySeconPC;
    private Face myRootFace = new();
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

    public void Add(in Face F)
    {
        if (myFirstF.IsNull)
        {
            myFirstF = F;
        }
        else if (!myFirstF.IsSame(F) && mySeconF.IsNull)
        {
            mySeconF = F;
        }
        myTol = Max(myTol, Tool.Tolerance(F));
    }

    public void RootFace(in Face F) => myRootFace = F;

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

    public Curve3D? Geometry() => myGeom;

    public Face FirstFace() => myFirstF;

    public Face SecondFace() => mySeconF;

    public Curve2D? FirstPC() => myFirstPC;

    public Curve2D? SecondPC() => mySeconPC;

    public ref Curve3D? ChangeGeometry() => ref myGeom;

    public ref Curve2D? ChangeFirstPC() => ref myFirstPC;

    public ref Curve2D? ChangeSecondPC() => ref mySeconPC;

    public Face RootFace() => myRootFace;

    public void Tolerance(double tol) => myTol = tol;

    public double Tolerance() => myTol;
}
