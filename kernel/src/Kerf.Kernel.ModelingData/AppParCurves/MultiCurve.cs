// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiCurve.hxx
// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_MultiCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Approximation;

/// <summary>
/// A set of n Bezier curves of the same degree approximating a MultiLine, stored as its poles: NbPoles
/// MultiPoints, each holding the pole of every curve (3d curves first, then 2d curves).
/// </summary>
/// <remarks>
/// A class, as <see cref="MultiPoint"/> (decision 141): OCCT's copies share the array of
/// MultiPoints, which is a handle. <see cref="MultiCurve(MultiCurve)"/> is the C++ copy
/// constructor (and the slicing copy of a MultiBSpCurve), sharing the poles. <see cref="Value(int)"/> returns the
/// stored MultiPoint (OCCT returns a const reference; shared: don't modify).
/// </remarks>
internal class MultiCurve
{
    protected NCollection_HArray1<MultiPoint>? _tabPoint;

    /// <summary>An indefinite MultiCurve.</summary>
    public MultiCurve() { }

    /// <summary>A MultiCurve of <paramref name="nbPol"/> MultiPoints, to be set with <see cref="SetValue"/>.</summary>
    public MultiCurve(int nbPol)
    {
        _tabPoint = NewPoints(nbPol);
    }

    /// <summary>A MultiCurve of the MultiPoints <paramref name="tabMU"/>, which must all have NbCurves points.</summary>
    public MultiCurve(NCollection_Array1<MultiPoint> tabMU)
    {
        _tabPoint = new NCollection_HArray1<MultiPoint>(1, tabMU.Length());
        int i, lower = tabMU.Lower();
        for (i = 1; i <= tabMU.Length(); i++)
        {
            _tabPoint.SetValue(i, tabMU.Value(lower + i - 1));
        }
    }

    /// <summary>OCCT's copy constructor: the copy shares the poles of <paramref name="other"/>.</summary>
    public MultiCurve(MultiCurve other)
    {
        _tabPoint = other._tabPoint;
    }

    // C++ default-constructs the elements of the array: empty MultiPoints, not nulls
    private static NCollection_HArray1<MultiPoint> NewPoints(int nbPoles)
    {
        NCollection_HArray1<MultiPoint> tab = new(1, nbPoles);
        for (int i = 1; i <= nbPoles; i++)
        {
            tab[i] = new MultiPoint();
        }
        return tab;
    }

    /// <summary>Gives the MultiCurve <paramref name="nbPoles"/> new, empty MultiPoints.</summary>
    public void SetNbPoles(int nbPoles)
    {
        _tabPoint = NewPoints(nbPoles);
    }

    /// <summary>Sets the MultiPoint of range <paramref name="index"/> (stored, not copied, as C++ stores a sharing copy).</summary>
    public void SetValue(int index, MultiPoint mPoint)
    {
        if ((index <= 0) || (index > _tabPoint!.Length()))
        {
            throw new OutOfRangeException();
        }
        _tabPoint.SetValue(index, mPoint);
    }

    /// <summary>The number of curves.</summary>
    public int NbCurves()
    {
        if (_tabPoint is null)
        {
            return 0;
        }
        MultiPoint mp = _tabPoint.Value(1);
        return mp.PointCount + mp.Points2dCount;
    }

    /// <summary>The number of poles of each curve.</summary>
    public virtual int NbPoles()
    {
        if (_tabPoint is null)
        {
            return 0;
        }
        return _tabPoint.Length();
    }

    /// <summary>The degree of the curves.</summary>
    public virtual int Degree() => _tabPoint!.Length() - 1;

    /// <summary>The dimension (3 or 2) of the curve of range <paramref name="cuIndex"/>.</summary>
    public int Dimension(int cuIndex)
    {
        int lo = _tabPoint!.Lower();
        int nb = _tabPoint.Value(lo).PointCount + _tabPoint.Value(lo).Points2dCount;
        if ((cuIndex <= 0) || (cuIndex > nb))
        {
            throw new OutOfRangeException();
        }
        return _tabPoint.Value(lo).Dimension(cuIndex);
    }

    /// <summary>Fills <paramref name="tabPnt"/> (from 1) with the poles of the 3d curve <paramref name="cuIndex"/>.</summary>
    public void Curve(int cuIndex, NCollection_Array1<Point3D> tabPnt)
    {
        if ((cuIndex <= 0))
        {
            throw new OutOfRangeException();
        }
        for (int i = 1; i <= _tabPoint!.Length(); i++)
        {
            tabPnt[i] = _tabPoint.Value(i).Point(cuIndex);
        }
    }

    /// <summary>Fills <paramref name="tabPnt2d"/> (from 1) with the poles of the 2d curve <paramref name="cuIndex"/>.</summary>
    public void Curve(int cuIndex, NCollection_Array1<Point2D> tabPnt2d)
    {
        if ((cuIndex <= 0))
        {
            throw new OutOfRangeException();
        }
        for (int i = 1; i <= _tabPoint!.Length(); i++)
        {
            tabPnt2d[i] = _tabPoint.Value(i).Point2d(cuIndex);
        }
    }

    /// <summary>The MultiPoint of range <paramref name="index"/> (shared: don't modify).</summary>
    public MultiPoint Value(int index)
    {
        if ((index <= 0) || (index > _tabPoint!.Length()))
        {
            throw new OutOfRangeException();
        }
        return _tabPoint.Value(index);
    }

    /// <summary>The pole <paramref name="nieme"/> of the 3d curve <paramref name="cuIndex"/>.</summary>
    public Point3D Pole(int cuIndex, int nieme)
    {
        // OCCT: && where || is meant, so the check only fires when both indexes are non-positive
        if ((cuIndex <= 0) && nieme <= 0)
        {
            throw new OutOfRangeException();
        }
        return _tabPoint!.Value(nieme).Point(cuIndex);
    }

    /// <summary>The pole <paramref name="nieme"/> of the 2d curve <paramref name="cuIndex"/>.</summary>
    public Point2D Pole2d(int cuIndex, int nieme)
    {
        // OCCT: && where || is meant (see Pole)
        if ((cuIndex <= 0) && nieme <= 0)
        {
            throw new OutOfRangeException();
        }
        return _tabPoint!.Value(nieme).Point2d(cuIndex);
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy, newz = z + dz * oldz to the poles of the 3d curve
    /// <paramref name="cuIndex"/>.
    /// </summary>
    public void Transform(int cuIndex, double x, double dx, double y, double dy, double z, double dz)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        for (int i = 1; i <= _tabPoint!.Length(); i++)
        {
            _tabPoint[i].Transform(cuIndex, x, dx, y, dy, z, dz);
        }
    }

    /// <summary>
    /// Applies newx = x + dx * oldx, newy = y + dy * oldy to the poles of the 2d curve <paramref name="cuIndex"/>.
    /// </summary>
    public void Transform2d(int cuIndex, double x, double dx, double y, double dy)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        for (int i = 1; i <= _tabPoint!.Length(); i++)
        {
            _tabPoint[i].Transform2d(cuIndex, x, dx, y, dy);
        }
    }

    /// <summary>The point at <paramref name="u"/> of the 3d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void Value(int cuIndex, double u, out Point3D pt)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPoles = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPoles[i] = _tabPoint.Value(i).Point(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, tabPoles, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt);
    }

    /// <summary>The point at <paramref name="u"/> of the 2d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void Value(int cuIndex, double u, out Point2D pt)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPole = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPole[i] = _tabPoint.Value(i).Point2d(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, tabPole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> of the 3d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void D1(int cuIndex, double u, out Point3D pt, out Vector3D v1)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPole = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPole[i] = _tabPoint.Value(i).Point(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, tabPole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt, out v1);
    }

    /// <summary>The point and two derivatives at <paramref name="u"/> of the 3d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void D2(int cuIndex, double u, out Point3D pt, out Vector3D v1, out Vector3D v2)
    {
        if (Dimension(cuIndex) != 3)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point3D> tabPole = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPole[i] = _tabPoint.Value(i).Point(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, tabPole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt, out v1, out v2);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> of the 2d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void D1(int cuIndex, double u, out Point2D pt, out Vector2D v1)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPole = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPole[i] = _tabPoint.Value(i).Point2d(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, tabPole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt, out v1);
    }

    /// <summary>The point and two derivatives at <paramref name="u"/> of the 2d Bezier curve <paramref name="cuIndex"/>.</summary>
    public virtual void D2(int cuIndex, double u, out Point2D pt, out Vector2D v1, out Vector2D v2)
    {
        if (Dimension(cuIndex) != 2)
        {
            throw new OutOfRangeException();
        }
        NCollection_Array1<Point2D> tabPole = new(1, _tabPoint!.Length());
        for (int i = 1; i <= _tabPoint.Length(); i++)
        {
            tabPole[i] = _tabPoint.Value(i).Point2d(cuIndex);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u, tabPole, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt, out v1, out v2);
    }

    /// <summary>Writes the number of curves and their degree (OCCT's pole listing is commented out).</summary>
    public virtual void Dump(TextWriter o)
    {
        o.Write("AppParCurves_MultiCurve dump:" + '\n');
        o.Write(" It contains " + MultiPoint.Int(NbCurves()) + " Bezier curves of degree "
                + MultiPoint.Int(_tabPoint!.Length() - 1) + '\n');
        o.Write(" The poles are: " + '\n');
    }

    /// <summary>What <see cref="Dump"/> writes.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
