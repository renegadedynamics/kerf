// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeFace.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Api.FaceError;
using static Kerf.Engine.Construction.Library.LibraryFaceError;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a face from a surface (gp or Geom), with or without bounds, from a planar wire, from a surface and a wire, or from a face and a wire to add (a hole).</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeFace"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// <para>OCCT.NET: where the maker had to copy frozen sub-shapes of its input to update them (decision 417), the
/// copies are what OCCT's in-place result holds, and <see cref="MakeShape.Modified"/> answers them for
/// the input's sub-shapes (OCCT's Modified, which would answer nothing, has nothing to report since it changes them in
/// place).</para>
/// </remarks>
internal class MakeFace : MakeShape
{
    private readonly LibraryMakeFace _makeFace;

    // OCCT's "if (myMakeFace.IsDone()) { Done(); myShape = myMakeFace.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeFace.IsDone)
        {
            Done();
            _shape = _makeFace.InternalShape();
        }
    }

    public MakeFace()
    {
        _makeFace = new LibraryMakeFace();
    }

    public MakeFace(in Face f)
    {
        _makeFace = new LibraryMakeFace(f);
        TakeResult();
    }

    public MakeFace(in Plane3D p)
    {
        _makeFace = new LibraryMakeFace(p);
        TakeResult();
    }

    public MakeFace(in Cylinder3D c)
    {
        _makeFace = new LibraryMakeFace(c);
        TakeResult();
    }

    public MakeFace(in Cone3D c)
    {
        _makeFace = new LibraryMakeFace(c);
        TakeResult();
    }

    public MakeFace(in Sphere3D s)
    {
        _makeFace = new LibraryMakeFace(s);
        TakeResult();
    }

    public MakeFace(in Torus3D t)
    {
        _makeFace = new LibraryMakeFace(t);
        TakeResult();
    }

    public MakeFace(Surface s, double tolDegen)
    {
        _makeFace = new LibraryMakeFace(s, tolDegen);
        TakeResult();
    }

    public MakeFace(in Plane3D p, double uMin, double uMax, double vMin, double vMax)
    {
        _makeFace = new LibraryMakeFace(p, uMin, uMax, vMin, vMax);
        TakeResult();
    }

    public MakeFace(in Cylinder3D c, double uMin, double uMax, double vMin, double vMax)
    {
        _makeFace = new LibraryMakeFace(c, uMin, uMax, vMin, vMax);
        TakeResult();
    }

    public MakeFace(in Cone3D c, double uMin, double uMax, double vMin, double vMax)
    {
        _makeFace = new LibraryMakeFace(c, uMin, uMax, vMin, vMax);
        TakeResult();
    }

    public MakeFace(in Sphere3D s, double uMin, double uMax, double vMin, double vMax)
    {
        _makeFace = new LibraryMakeFace(s, uMin, uMax, vMin, vMax);
        TakeResult();
    }

    public MakeFace(in Torus3D t, double uMin, double uMax, double vMin, double vMax)
    {
        _makeFace = new LibraryMakeFace(t, uMin, uMax, vMin, vMax);
        TakeResult();
    }

    public MakeFace(Surface s, double uMin, double uMax, double vMin, double vMax, double tolDegen)
    {
        _makeFace = new LibraryMakeFace(s, uMin, uMax, vMin, vMax, tolDegen);
        TakeResult();
    }

    public MakeFace(in Wire w, bool onlyPlane = false)
    {
        _makeFace = new LibraryMakeFace(w, onlyPlane);
        TakeResult();
    }

    public MakeFace(in Plane3D p, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(p, w, inside);
        TakeResult();
    }

    public MakeFace(in Cylinder3D c, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(c, w, inside);
        TakeResult();
    }

    public MakeFace(in Cone3D c, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(c, w, inside);
        TakeResult();
    }

    public MakeFace(in Sphere3D s, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(s, w, inside);
        TakeResult();
    }

    public MakeFace(in Torus3D t, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(t, w, inside);
        TakeResult();
    }

    public MakeFace(Surface s, in Wire w, bool inside = true)
    {
        _makeFace = new LibraryMakeFace(s, w, inside);
        TakeResult();
    }

    public MakeFace(in Face f, in Wire w)
    {
        _makeFace = new LibraryMakeFace(f, w);
        TakeResult();
    }

    public void Init(in Face f)
    {
        _makeFace.Init(f);
        TakeResult();
    }

    public void Init(Surface s, bool bound, double tolDegen)
    {
        _makeFace.Init(s, bound, tolDegen);
        TakeResult();
    }

    public void Init(Surface ss, double Um, double um, double Vm, double vm, double tolDegen)
    {
        _makeFace.Init(ss, Um, um, Vm, vm, tolDegen);
        TakeResult();
    }

    public void Add(in Wire w)
    {
        _makeFace.Add(w);
        TakeResult();
    }


    /// <summary>The face (frozen). As in OCCT, it doesn't check <see cref="IsDone"/>.</summary>
    public Face Face()
    {
        return _makeFace.Face();
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace()
    {
        return _makeFace.InternalFace();
    }

    /// <summary>The face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Face(MakeFace maker) => maker.Face();

    public override bool IsDone
    {
        get
        {
            return _makeFace.IsDone;
        }
    }

    /// <summary>The construction status.</summary>
    public FaceError Error()
    {
        switch (_makeFace.Error())
        {
            case LibraryFaceError.FaceDone:
                return FaceError.FaceDone;
            case LibraryFaceError.NoFace:
                return FaceError.NoFace;
            case LibraryFaceError.NotPlanar:
                return FaceError.NotPlanar;
            case LibraryFaceError.CurveProjectionFailed:
                return FaceError.CurveProjectionFailed;
            case LibraryFaceError.ParametersOutOfRange:
                return FaceError.ParametersOutOfRange;
        }

        // Portage WNT
        return FaceError.FaceDone;
    }

    // OCCT.NET: the replacements of frozen input sub-shapes by the copies the maker updated (decision 417).
    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        History? history = _makeFace.InternalHistory();
        if (history is not null)
        {
            foreach (Shape modified in history.Modified(s))
            {
                _generated.Append(modified);
            }
        }
        return _generated;
    }
}
