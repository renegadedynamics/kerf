// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>
/// A face of the BRep topology seen as a 3D surface: the face's surface, placed by its location. With restriction (the
/// default) the parameter range is the UV bounds of the face's boundary.
/// </summary>
/// <remarks>Most methods come from <see cref="TransformedSurface"/>, which applies the location.</remarks>
internal class FaceSurfaceAdaptor : TransformedSurface
{
    private Face _face;

    /// <summary>An undefined surface, with no face loaded.</summary>
    public FaceSurfaceAdaptor() => _face = new Face();

    /// <summary>The surface of <paramref name="f"/>, over the face's UV bounds if <paramref name="r"/>.</summary>
    public FaceSurfaceAdaptor(in Face f, bool r = true)
    {
        _face = new Face();
        Initialize(f, r);
    }

    /// <summary>A shallow copy of the adaptor.</summary>
    public override SurfaceAdaptorBase ShallowCopy()
    {
        FaceSurfaceAdaptor copy = new FaceSurfaceAdaptor();

        SurfaceAdaptorBase surface = _surf.ShallowCopy();
        SurfaceAdaptor geomSurface = (SurfaceAdaptor)surface;
        copy._surf = geomSurface;

        copy._trsf = _trsf;
        copy._face = _face;
        // OCCT copies the optional adaptor by value (sharing its cache); the port gives the copy its own, as
        // GeomAdaptor_TransformedSurface.ShallowCopy does
        copy._transformedAdaptor = (SurfaceAdaptor?)_transformedAdaptor?.ShallowCopy();

        return copy;
    }

    /// <summary>Loads the surface of <paramref name="f"/>, over the face's UV bounds if <paramref name="restriction"/>.</summary>
    public void Initialize(in Face f, bool restriction = true)
    {
        if (f.IsNull)
        {
            return;
        }

        _face = f;
        Surface? surface = Tool.Surface(f, out Placement l);
        if (surface is null)
        {
            return;
        }

        if (restriction)
        {
            BrepTools.UVBounds(f, out double umin, out double umax, out double vmin, out double vmax);
            Load(surface, umin, umax, vmin, vmax, l.Transformation);
        }
        else
        {
            Load(surface, l.Transformation);
        }
    }

    /// <summary>The face.</summary>
    public Face Face() => _face;

    /// <summary>The face's tolerance.</summary>
    public double Tolerance() => Tool.Tolerance(_face);
}
