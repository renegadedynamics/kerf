// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Vars.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Vars.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Defines a receptacle for externally defined variables, each one has a name I.E. a WorkSession for XSTEP
/// is generally used inside a context, which brings variables, especially shapes and geometries. For
/// instance DRAW or an application engine This class provides a common form for this. It also provides a
/// default implementation (locally recorded variables in a dictionary), but which is aimed to be redefined
/// </summary>
public class XSControl_Vars : Standard_Transient
{

    public XSControl_Vars()
    {
    }

    public virtual void Set(string name, Standard_Transient? val)
    {
        thevars.Bind(name, val);
    }

    public virtual Standard_Transient? Get(string name)
    {
        if (!thevars.Find(name, out Standard_Transient val))
        {
            val = null;
        }
        return val;
    }

    public virtual Geom_Geometry? GetGeom(string name)
    {
        return (Get(name) as Geom_Geometry);
    }

    public virtual Geom2d_Curve? GetCurve2d(string name)
    {
        return (Get(name) as Geom2d_Curve);
    }

    public virtual Geom_Curve? GetCurve(string name)
    {
        return (Get(name) as Geom_Curve);
    }

    public virtual Geom_Surface? GetSurface(string name)
    {
        return (Get(name) as Geom_Surface);
    }

    public virtual void SetPoint(string name, gp_Pnt val)
    {
        Set(name, new Geom_CartesianPoint(val));
    }

    public virtual void SetPoint2d(string name, gp_Pnt2d val)
    {
        Set(name, new Geom2d_CartesianPoint(val));
    }

    public virtual bool GetPoint(string name, ref gp_Pnt pnt)
    {
        Geom_CartesianPoint val = Get(name) as Geom_CartesianPoint;
        if (val is null)
        {
            return false;
        }
        pnt = val.Pnt();
        return true;
    }

    public virtual bool GetPoint2d(string name, ref gp_Pnt2d pnt)
    {
        Geom2d_CartesianPoint val = Get(name) as Geom2d_CartesianPoint;
        if (val is null)
        {
            return false;
        }
        pnt = val.Pnt2d();
        return true;
    }

    public virtual void SetShape(string name, TopoDS_Shape val)
    {
        Set(name, new TopoDS_HShape(val));
    }

    public virtual TopoDS_Shape GetShape(string name)
    {
        TopoDS_Shape sh = default;
        TopoDS_HShape val = Get(name) as TopoDS_HShape;
        if (val is not null)
        {
            sh = val.Shape();
        }
        return sh;
    }

    private NCollection_DataMap<string, Standard_Transient> thevars = new NCollection_DataMap<string, Standard_Transient>();
}
