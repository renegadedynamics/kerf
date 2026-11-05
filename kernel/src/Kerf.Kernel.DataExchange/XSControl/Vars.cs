// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Vars.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Vars.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Defines a receptacle for externally defined variables, each one has a name I.E. a WorkSession for XSTEP
/// is generally used inside a context, which brings variables, especially shapes and geometries. For
/// instance DRAW or an application engine This class provides a common form for this. It also provides a
/// default implementation (locally recorded variables in a dictionary), but which is aimed to be redefined
/// </summary>
internal class Vars : Transient
{

    public Vars()
    {
    }

    public virtual void Set(string name, Transient? val)
    {
        _thevars[name] = val;
    }

    public virtual Transient? Get(string name)
    {
        if (!_thevars.TryGetValue(name, out Transient val))
        {
            val = null;
        }
        return val;
    }

    public virtual Geometry3D? GetGeom(string name)
    {
        return (Get(name) as Geometry3D);
    }

    public virtual Curve2D? GetCurve2d(string name)
    {
        return (Get(name) as Curve2D);
    }

    public virtual Curve3D? GetCurve(string name)
    {
        return (Get(name) as Curve3D);
    }

    public virtual Surface? GetSurface(string name)
    {
        return (Get(name) as Surface);
    }

    public virtual void SetPoint(string name, Point3D val)
    {
        Set(name, new CartesianPoint(val));
    }

    public virtual void SetPoint2d(string name, Point2D val)
    {
        Set(name, new CartesianPoint2(val));
    }

    public virtual bool GetPoint(string name, ref Point3D pnt)
    {
        CartesianPoint val = Get(name) as CartesianPoint;
        if (val is null)
        {
            return false;
        }
        pnt = val.Pnt();
        return true;
    }

    public virtual bool GetPoint2d(string name, ref Point2D pnt)
    {
        CartesianPoint2 val = Get(name) as CartesianPoint2;
        if (val is null)
        {
            return false;
        }
        pnt = val.Pnt2d();
        return true;
    }

    public virtual void SetShape(string name, Shape val)
    {
        Set(name, new TopoDS_HShape(val));
    }

    public virtual Shape GetShape(string name)
    {
        Shape sh = default;
        TopoDS_HShape val = Get(name) as TopoDS_HShape;
        if (val is not null)
        {
            sh = val.Shape();
        }
        return sh;
    }

    private Dictionary<string, Transient> _thevars = new Dictionary<string, Transient>();
}
