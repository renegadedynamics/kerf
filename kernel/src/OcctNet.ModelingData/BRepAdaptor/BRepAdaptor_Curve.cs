// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An edge of the BRep topology seen as a 3D curve, placed by the edge's location. The edge's 3D curve is used if it
/// has one, otherwise its first curve on a surface; a curve on a given face can be asked for explicitly.
/// </summary>
/// <remarks>
/// Most methods come from <see cref="GeomAdaptor_TransformedCurve"/>, which applies the location. A working object
/// (decision 214).
/// </remarks>
public class BRepAdaptor_Curve : GeomAdaptor_TransformedCurve
{
    private TopoDS_Edge myEdge = new();

    /// <summary>An undefined curve, with no edge loaded.</summary>
    public BRepAdaptor_Curve()
    {
    }

    /// <summary>The geometry of the edge.</summary>
    /// <exception cref="Standard_NullObject">The edge has neither a 3D curve nor a curve on a surface.</exception>
    public BRepAdaptor_Curve(in TopoDS_Edge E)
    {
        Initialize(E);
    }

    /// <summary>The geometry of the edge, computed from its pcurve on the face.</summary>
    public BRepAdaptor_Curve(in TopoDS_Edge E, in TopoDS_Face F)
    {
        Initialize(E, F);
    }

    /// <summary>A shallow copy of the adaptor (with shallow copies of the inner adaptors).</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        BRepAdaptor_Curve aCopy = new BRepAdaptor_Curve();

        Adaptor3d_Curve aCurve = myCurve.ShallowCopy();
        GeomAdaptor_Curve aGeomCurve = (GeomAdaptor_Curve)aCurve;
        aCopy.myCurve = aGeomCurve;

        if (myConSurf is not null)
        {
            aCopy.myConSurf = (Adaptor3d_CurveOnSurface)myConSurf.ShallowCopy();
        }
        aCopy.myTrsf = myTrsf;
        aCopy.myEdge = myEdge;

        return aCopy;
    }

    /// <summary>Unloads the edge.</summary>
    public void Reset()
    {
        myCurve.Reset();
        myConSurf = null;
        myEdge.Nullify();
        myTrsf = new gp_Trsf();
    }

    /// <summary>Loads the geometry of the edge.</summary>
    /// <exception cref="Standard_NullObject">The edge has neither a 3D curve nor a curve on a surface.</exception>
    public void Initialize(in TopoDS_Edge E)
    {
        myConSurf = null;
        myEdge = E;

        Geom_Curve? C = BRep_Tool.Curve(E, out TopLoc_Location L, out double pf, out double pl);

        if (C is not null)
        {
            myCurve.Load(C, pf, pl);
        }
        else
        {
            BRep_Tool.CurveOnSurface(E, out Geom2d_Curve? PC, out Geom_Surface? S, out L, out pf, out pl);
            if (PC is not null)
            {
                GeomAdaptor_Surface HS = new GeomAdaptor_Surface();
                HS.Load(S!);
                Geom2dAdaptor_Curve HC = new Geom2dAdaptor_Curve();
                HC.Load(PC, pf, pl);
                myConSurf = new Adaptor3d_CurveOnSurface();
                myConSurf.Load(HC, HS);
            }
            else
            {
                throw new Standard_NullObject("BRepAdaptor_Curve::No geometry");
            }
        }
        myTrsf = L.Transformation();
    }

    /// <summary>Loads the geometry of the edge, computed from its pcurve on the face.</summary>
    public void Initialize(in TopoDS_Edge E, in TopoDS_Face F)
    {
        myConSurf = null;

        myEdge = E;
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);
        Geom2d_Curve? PC = BRep_Tool.CurveOnSurface(E, F, out double pf, out double pl);

        GeomAdaptor_Surface HS = new GeomAdaptor_Surface();
        HS.Load(S!);
        Geom2dAdaptor_Curve HC = new Geom2dAdaptor_Curve();
        HC.Load(PC!, pf, pl);
        myConSurf = new Adaptor3d_CurveOnSurface();
        myConSurf.Load(HC, HS);

        myTrsf = L.Transformation();
    }

    /// <summary>The edge.</summary>
    public TopoDS_Edge Edge() => myEdge;

    /// <summary>The edge's tolerance.</summary>
    public double Tolerance() => BRep_Tool.Tolerance(myEdge);

    /// <summary>The curve between First and Last, keeping the edge and the transformation.</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        // Make a copy of this to keep the transformation.
        // OCCT loads the range into its own adaptor, copies itself and restores the range; the port builds the copy
        // directly, with its own adaptors (the copy shares no cache, decision 214).
        BRepAdaptor_Curve res = new BRepAdaptor_Curve();
        if (myConSurf is null)
        {
            Geom_Curve? C = myCurve.Curve();
            res.myCurve.Load(C!, First, Last);
        }
        else
        {
            res.myConSurf = (Adaptor3d_CurveOnSurface)myConSurf.Trim(First, Last, Tol);
        }
        res.myTrsf = myTrsf;
        res.myEdge = myEdge;
        return res;
    }
}
