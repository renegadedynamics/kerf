// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve2d.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The pcurve of an edge on a face, seen as a 2D curve.</summary>
/// <remarks>A working object (decision 214).</remarks>
public class BRepAdaptor_Curve2d : Geom2dAdaptor_Curve
{
    private TopoDS_Edge myEdge = new();
    private TopoDS_Face myFace = new();

    /// <summary>An uninitialised curve.</summary>
    public BRepAdaptor_Curve2d()
    {
    }

    /// <summary>The pcurve of the edge on the face.</summary>
    public BRepAdaptor_Curve2d(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Initialize(E, F);
    }

    /// <summary>A shallow copy: the curve is shared, the evaluation caches are not.</summary>
    public override Adaptor2d_Curve2d ShallowCopy()
    {
        BRepAdaptor_Curve2d aCopy = new BRepAdaptor_Curve2d();

        // OCCT copies the curve, its type, range and evaluation data by hand ("cache will be rebuilt on demand");
        // the port does it with Geom2dAdaptor_Curve's own shallow copy
        ShallowCopyTo(aCopy);
        aCopy.myEdge = myEdge;
        aCopy.myFace = myFace;

        return aCopy;
    }

    /// <summary>Loads the pcurve of the edge on the face (nothing is loaded if there is none).</summary>
    public void Initialize(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myEdge = E;
        myFace = F;
        Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(E, F, out double aFirs, out double aLast);
        if (aPCurve is not null)
        {
            Load(aPCurve, aFirs, aLast);
        }
    }

    /// <summary>The edge.</summary>
    public TopoDS_Edge Edge() => myEdge;

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => myFace;
}
