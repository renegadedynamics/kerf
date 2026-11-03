// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.lxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The points of a MultiPoint: first the 3d points, then the 2d points. They are the poles of the curves of a
/// MultiCurve, or the data points of a MultiLine.
/// </summary>
/// <remarks>
/// A class (decision 141): OCCT's copies share the point arrays, which are handles, so a C# reference behaves as
/// a C++ copy does. The point counts never change after construction. <see cref="MultiPoint(MultiPoint)"/>
/// is the C++ copy constructor, sharing the arrays. The setters stay public, as in OCCT: MultiPoints are the
/// input data callers build (AppDef_MultiPointConstraint derives from this class).
/// </remarks>
public class MultiPoint
{
    protected NCollection_HArray1<Point3D>? _ttabPoint;
    protected NCollection_HArray1<Point2D>? _ttabPoint2d;
    protected int _nbP;
    protected int _nbP2d;

    /// <summary>An indefinite MultiPoint.</summary>
    public MultiPoint()
    {
        _nbP = 0;
        _nbP2d = 0;
    }

    /// <summary>
    /// <paramref name="nbPoles"/> 3d points and <paramref name="nbPoles2d"/> 2d points, to be set with
    /// <see cref="SetPoint"/> and <see cref="SetPoint2d"/>.
    /// </summary>
    public MultiPoint(int nbPoles, int nbPoles2d)
    {
        _nbP = nbPoles;
        _nbP2d = nbPoles2d;
        if (_nbP != 0)
        {
            NCollection_HArray1<Point3D> tab3d = new(1, nbPoles);
            _ttabPoint = tab3d;
        }
        if (_nbP2d != 0)
        {
            NCollection_HArray1<Point2D> tab2d = new(1, nbPoles2d);
            _ttabPoint2d = tab2d;
        }
    }

    /// <summary>A MultiPoint of 3d points only (copied).</summary>
    public MultiPoint(NCollection_Array1<Point3D> tabP)
    {
        _nbP2d = 0;
        _nbP = tabP.Length();
        NCollection_HArray1<Point3D> tab3d = new(1, _nbP);
        _ttabPoint = tab3d;
        int lower = tabP.Lower();
        NCollection_Array1<Point3D> p3d = TabPoint.ChangeArray1();
        for (int i = 1; i <= tabP.Length(); i++)
        {
            p3d.SetValue(i, tabP.Value(lower + i - 1));
        }
    }

    /// <summary>A MultiPoint of 2d points only (copied).</summary>
    public MultiPoint(NCollection_Array1<Point2D> tabP2d)
    {
        _nbP = 0;
        _nbP2d = tabP2d.Length();
        NCollection_HArray1<Point2D> tab2d = new(1, _nbP2d);
        _ttabPoint2d = tab2d;
        int lower = tabP2d.Lower();
        NCollection_Array1<Point2D> p2d = TabPoint2d.ChangeArray1();
        for (int i = 1; i <= _nbP2d; i++)
        {
            p2d.SetValue(i, tabP2d.Value(lower + i - 1));
        }
    }

    /// <summary>A MultiPoint of the 3d points <paramref name="tabP"/> and the 2d points <paramref name="tabP2d"/> (copied).</summary>
    public MultiPoint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Point2D> tabP2d)
    {
        _nbP = tabP.Length();
        _nbP2d = tabP2d.Length();
        NCollection_HArray1<Point3D> t3d = new(1, _nbP);
        _ttabPoint = t3d;
        NCollection_HArray1<Point2D> t2d = new(1, _nbP2d);
        _ttabPoint2d = t2d;
        NCollection_Array1<Point3D> p3d = TabPoint.ChangeArray1();
        int i, lower = tabP.Lower();
        for (i = 1; i <= _nbP; i++)
        {
            p3d.SetValue(i, tabP.Value(lower + i - 1));
        }
        lower = tabP2d.Lower();
        NCollection_Array1<Point2D> p2d = TabPoint2d.ChangeArray1();
        for (i = 1; i <= _nbP2d; i++)
        {
            p2d.SetValue(i, tabP2d.Value(lower + i - 1));
        }
    }

    /// <summary>OCCT's copy constructor: the copy shares the point arrays of <paramref name="other"/>.</summary>
    public MultiPoint(MultiPoint other)
    {
        _ttabPoint = other._ttabPoint;
        _ttabPoint2d = other._ttabPoint2d;
        _nbP = other._nbP;
        _nbP2d = other._nbP2d;
    }

    // OCCT's TabPoint and TabPoint2d macros (down-casts of the handles)
    private NCollection_HArray1<Point3D> TabPoint => _ttabPoint!;

    private NCollection_HArray1<Point2D> TabPoint2d => _ttabPoint2d!;

    /// <summary>The number of 3d points.</summary>
    public int PointCount => _nbP;

    /// <summary>The number of 2d points.</summary>
    public int Points2dCount => _nbP2d;

    /// <summary>The dimension (3 or 2) of the point of range <paramref name="index"/>.</summary>
    public int Dimension(int index)
    {
        if (index < 0 || index > (_nbP + _nbP2d))
        {
            throw new OutOfRangeException();
        }
        if (index <= _nbP)
        {
            return 3;
        }
        else
        {
            return 2;
        }
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy, newz = z + dz * oldz to the 3d point of range
    /// <paramref name="cuIndex"/>.
    /// </summary>
    public void Transform(int cuIndex, double x, double dx, double y, double dy, double z, double dz)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        Point3D p, newP = new();
        p = Point(cuIndex);
        newP.SetCoord(x + p.X() * dx, y + p.Y() * dy, z + p.Z() * dz);
        TabPoint.SetValue(cuIndex, newP);
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy to the 2d point of range <paramref name="cuIndex"/>.
    /// </summary>
    public void Transform2d(int cuIndex, double x, double dx, double y, double dy)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        Point2D p, newP = new();
        p = Point2d(cuIndex);
        newP.SetCoord(x + p.X() * dx, y + p.Y() * dy);
        SetPoint2d(cuIndex, newP);
    }

    /// <summary>Sets the 3d point of range <paramref name="index"/> (1 to NbPoints).</summary>
    public void SetPoint(int index, in Point3D point)
    {
        Standard_OutOfRange_Raise_if((index <= 0) || (index > _nbP), "AppParCurves_MultiPoint::SetPoint() - wrong index");
        TabPoint.SetValue(index, point);
    }

    /// <summary>The 3d point of range <paramref name="index"/> (1 to NbPoints).</summary>
    public Point3D Point(int index)
    {
        Standard_OutOfRange_Raise_if((index <= 0) || (index > _nbP), "AppParCurves_MultiPoint::Point() - wrong index");
        return TabPoint.Value(index);
    }

    /// <summary>Sets the 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public void SetPoint2d(int index, in Point2D point)
    {
        Standard_OutOfRange_Raise_if((index <= _nbP) || (index > _nbP + _nbP2d),
                                     "AppParCurves_MultiPoint::SetPoint2d() - wrong index");
        TabPoint2d.SetValue(index - _nbP, point);
    }

    /// <summary>The 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public Point2D Point2d(int index)
    {
        Standard_OutOfRange_Raise_if((index <= _nbP) || (index > _nbP + _nbP2d),
                                     "AppParCurves_MultiPoint::Point2d() - wrong index");
        return TabPoint2d.Value(index - _nbP);
    }

    /// <summary>Writes the points.</summary>
    public virtual void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiPoint dump:" + '\n');
        int nbPnts3D = PointCount, nbPnts2D = Points2dCount;
        o.Write("It contains " + Int(nbPnts3D) + " 3d points and " + Int(nbPnts2D) + " 2d points." + '\n');
        if (nbPnts3D > 0)
        {
            for (int i = TabPoint.Lower(); i <= TabPoint.Upper(); i++)
            {
                o.Write("3D-Point #" + Int(i) + '\n');
                o.Write(" Pole x = " + Real(TabPoint.Value(i).X()) + '\n');
                o.Write(" Pole y = " + Real(TabPoint.Value(i).Y()) + '\n');
                o.Write(" Pole z = " + Real(TabPoint.Value(i).Z()) + '\n');
            }
        }
        if (nbPnts2D > 0)
        {
            for (int i = TabPoint2d.Lower(); i <= TabPoint2d.Upper(); i++)
            {
                o.Write("2D-Point #" + Int(i) + '\n');
                o.Write(" Pole x = " + Real(TabPoint2d.Value(i).X()) + '\n');
                o.Write(" Pole y = " + Real(TabPoint2d.Value(i).Y()) + '\n');
            }
        }
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    // C++ stream formatting: OS << int, OS << double (six significant digits, as printf's %g)
    internal static string Int(int value) => value.ToString(CultureInfo.InvariantCulture);

    internal static string Real(double value) => global::Kerf.Engine.Meshing.Poly.Real(value);
}
