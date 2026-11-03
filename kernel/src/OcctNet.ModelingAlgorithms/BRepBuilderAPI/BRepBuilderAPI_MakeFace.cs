// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeFace.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_FaceError;
using static Occt.BRepLib_FaceError;

namespace Occt;

/// <summary>Builds a face from a surface (gp or Geom), with or without bounds, from a planar wire, from a surface and a wire, or from a face and a wire to add (a hole).</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeFace"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// <para>OCCT.NET: where the maker had to copy frozen sub-shapes of its input to update them (decision 417), the
/// copies are what OCCT's in-place result holds, and <see cref="BRepBuilderAPI_MakeShape.Modified"/> answers them for
/// the input's sub-shapes (OCCT's Modified, which would answer nothing, has nothing to report since it changes them in
/// place).</para>
/// </remarks>
public class BRepBuilderAPI_MakeFace : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeFace myMakeFace;

    // OCCT's "if (myMakeFace.IsDone()) { Done(); myShape = myMakeFace.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeFace.IsDone())
        {
            Done();
            myShape = myMakeFace.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeFace()
    {
        myMakeFace = new BRepLib_MakeFace();
    }

    public BRepBuilderAPI_MakeFace(in Face F)
    {
        myMakeFace = new BRepLib_MakeFace(F);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Plane3D P)
    {
        myMakeFace = new BRepLib_MakeFace(P);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cylinder C)
    {
        myMakeFace = new BRepLib_MakeFace(C);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cone C)
    {
        myMakeFace = new BRepLib_MakeFace(C);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Sphere S)
    {
        myMakeFace = new BRepLib_MakeFace(S);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Torus T)
    {
        myMakeFace = new BRepLib_MakeFace(T);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(Surface S, double TolDegen)
    {
        myMakeFace = new BRepLib_MakeFace(S, TolDegen);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Plane3D P, double UMin, double UMax, double VMin, double VMax)
    {
        myMakeFace = new BRepLib_MakeFace(P, UMin, UMax, VMin, VMax);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cylinder C, double UMin, double UMax, double VMin, double VMax)
    {
        myMakeFace = new BRepLib_MakeFace(C, UMin, UMax, VMin, VMax);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cone C, double UMin, double UMax, double VMin, double VMax)
    {
        myMakeFace = new BRepLib_MakeFace(C, UMin, UMax, VMin, VMax);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Sphere S, double UMin, double UMax, double VMin, double VMax)
    {
        myMakeFace = new BRepLib_MakeFace(S, UMin, UMax, VMin, VMax);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Torus T, double UMin, double UMax, double VMin, double VMax)
    {
        myMakeFace = new BRepLib_MakeFace(T, UMin, UMax, VMin, VMax);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(Surface S, double UMin, double UMax, double VMin, double VMax, double TolDegen)
    {
        myMakeFace = new BRepLib_MakeFace(S, UMin, UMax, VMin, VMax, TolDegen);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Wire W, bool OnlyPlane = false)
    {
        myMakeFace = new BRepLib_MakeFace(W, OnlyPlane);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Plane3D P, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(P, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cylinder C, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(C, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Cone C, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(C, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Sphere S, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(S, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Torus T, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(T, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(Surface S, in Wire W, bool Inside = true)
    {
        myMakeFace = new BRepLib_MakeFace(S, W, Inside);
        TakeResult();
    }

    public BRepBuilderAPI_MakeFace(in Face F, in Wire W)
    {
        myMakeFace = new BRepLib_MakeFace(F, W);
        TakeResult();
    }

    public void Init(in Face F)
    {
        myMakeFace.Init(F);
        TakeResult();
    }

    public void Init(Surface S, bool Bound, double TolDegen)
    {
        myMakeFace.Init(S, Bound, TolDegen);
        TakeResult();
    }

    public void Init(Surface SS, double Um, double UM, double Vm, double VM, double TolDegen)
    {
        myMakeFace.Init(SS, Um, UM, Vm, VM, TolDegen);
        TakeResult();
    }

    public void Add(in Wire W)
    {
        myMakeFace.Add(W);
        TakeResult();
    }


    /// <summary>The face (frozen). As in OCCT, it doesn't check <see cref="IsDone"/>.</summary>
    public Face Face()
    {
        return myMakeFace.Face();
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace()
    {
        return myMakeFace.InternalFace();
    }

    /// <summary>The face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Face(BRepBuilderAPI_MakeFace theMaker) => theMaker.Face();

    public override bool IsDone()
    {
        return myMakeFace.IsDone();
    }

    /// <summary>The construction status.</summary>
    public BRepBuilderAPI_FaceError Error()
    {
        switch (myMakeFace.Error())
        {
            case BRepLib_FaceDone:
                return BRepBuilderAPI_FaceDone;
            case BRepLib_NoFace:
                return BRepBuilderAPI_NoFace;
            case BRepLib_NotPlanar:
                return BRepBuilderAPI_NotPlanar;
            case BRepLib_CurveProjectionFailed:
                return BRepBuilderAPI_CurveProjectionFailed;
            case BRepLib_ParametersOutOfRange:
                return BRepBuilderAPI_ParametersOutOfRange;
        }

        // Portage WNT
        return BRepBuilderAPI_FaceDone;
    }

    // OCCT.NET: the replacements of frozen input sub-shapes by the copies the maker updated (decision 417).
    internal override NCollection_List<Shape> InternalModified(in Shape S)
    {
        myGenerated.Clear();
        History? aHistory = myMakeFace.InternalHistory();
        if (aHistory is not null)
        {
            foreach (Shape aModified in aHistory.Modified(S))
            {
                myGenerated.Append(aModified);
            }
        }
        return myGenerated;
    }
}
