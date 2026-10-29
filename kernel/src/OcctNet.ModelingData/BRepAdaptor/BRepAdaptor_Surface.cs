// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A face of the BRep topology seen as a 3D surface: the face's surface, placed by its location. With restriction (the
/// default) the parameter range is the UV bounds of the face's boundary.
/// </summary>
/// <remarks>Most methods come from <see cref="GeomAdaptor_TransformedSurface"/>, which applies the location.</remarks>
public class BRepAdaptor_Surface : GeomAdaptor_TransformedSurface
{
    private TopoDS_Face myFace;

    /// <summary>An undefined surface, with no face loaded.</summary>
    public BRepAdaptor_Surface() => myFace = new TopoDS_Face();

    /// <summary>The surface of <paramref name="F"/>, over the face's UV bounds if <paramref name="R"/>.</summary>
    public BRepAdaptor_Surface(in TopoDS_Face F, bool R = true)
    {
        myFace = new TopoDS_Face();
        Initialize(F, R);
    }

    /// <summary>A shallow copy of the adaptor.</summary>
    public override Adaptor3d_Surface ShallowCopy()
    {
        BRepAdaptor_Surface aCopy = new BRepAdaptor_Surface();

        Adaptor3d_Surface aSurface = mySurf.ShallowCopy();
        GeomAdaptor_Surface aGeomSurface = (GeomAdaptor_Surface)aSurface;
        aCopy.mySurf = aGeomSurface;

        aCopy.myTrsf = myTrsf;
        aCopy.myFace = myFace;
        // OCCT copies the optional adaptor by value (sharing its cache); the port gives the copy its own, as
        // GeomAdaptor_TransformedSurface.ShallowCopy does
        aCopy.myTransformedAdaptor = (GeomAdaptor_Surface?)myTransformedAdaptor?.ShallowCopy();

        return aCopy;
    }

    /// <summary>Loads the surface of <paramref name="F"/>, over the face's UV bounds if <paramref name="Restriction"/>.</summary>
    public void Initialize(in TopoDS_Face F, bool Restriction = true)
    {
        if (F.IsNull())
        {
            return;
        }

        myFace = F;
        Geom_Surface? aSurface = BRep_Tool.Surface(F, out TopLoc_Location L);
        if (aSurface is null)
        {
            return;
        }

        if (Restriction)
        {
            BRepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
            Load(aSurface, umin, umax, vmin, vmax, L.Transformation());
        }
        else
        {
            Load(aSurface, L.Transformation());
        }
    }

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => myFace;

    /// <summary>The face's tolerance.</summary>
    public double Tolerance() => BRep_Tool.Tolerance(myFace);
}
