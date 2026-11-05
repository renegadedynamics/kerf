// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Connexion.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Connexion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A connexion links two lines of items (points or curves) in a set of lines: two points and their
/// parameters on the lines.</summary>
/// <remarks>A working object of MAT2d_MiniPath and MAT2d_Circuit (decision 21): the setters are internal. Dump (to
/// std::cout) is left out.</remarks>
internal sealed class Connexion : Transient
{
    private int _lineA;
    private int _lineB;
    private int _itemA;
    private int _itemB;
    private double _distance;
    private double _parameterOnA;
    private double _parameterOnB;
    private Point2D _pointA;
    private Point2D _pointB;

    public Connexion()
    {
        _lineA = 0;
        _lineB = 0;
        _itemA = 0;
        _itemB = 0;
        _distance = 0.0;
        _parameterOnA = 0.0;
        _parameterOnB = 0.0;
        _pointA = new Point2D();
        _pointB = new Point2D();
    }

    public Connexion(int lineA, int lineB, int itemA, int itemB, double distance, double parameterOnA,
        double parameterOnB, in Point2D pointA, in Point2D pointB)
    {
        _lineA = lineA;
        _lineB = lineB;
        _itemA = itemA;
        _itemB = itemB;
        _distance = distance;
        _parameterOnA = parameterOnA;
        _parameterOnB = parameterOnB;
        _pointA = pointA;
        _pointB = pointB;
    }

    /// <summary>The index of the first line.</summary>
    public int IndexFirstLine() => _lineA;

    /// <summary>The index of the second line.</summary>
    public int IndexSecondLine() => _lineB;

    /// <summary>The index of the item on the first line.</summary>
    public int IndexItemOnFirst() => _itemA;

    /// <summary>The index of the item on the second line.</summary>
    public int IndexItemOnSecond() => _itemB;

    /// <summary>The parameter of the point on the first line.</summary>
    public double ParameterOnFirst() => _parameterOnA;

    /// <summary>The parameter of the point on the second line.</summary>
    public double ParameterOnSecond() => _parameterOnB;

    /// <summary>The point on the first line.</summary>
    public Point2D PointOnFirst() => _pointA;

    /// <summary>The point on the second line.</summary>
    public Point2D PointOnSecond() => _pointB;

    /// <summary>The distance between the two points.</summary>
    public double Distance() => _distance;

    internal void IndexFirstLine(int anIndex) => _lineA = anIndex;

    internal void IndexSecondLine(int anIndex) => _lineB = anIndex;

    internal void IndexItemOnFirst(int anIndex) => _itemA = anIndex;

    internal void IndexItemOnSecond(int anIndex) => _itemB = anIndex;

    internal void ParameterOnFirst(double aParameter) => _parameterOnA = aParameter;

    internal void ParameterOnSecond(double aParameter) => _parameterOnB = aParameter;

    internal void PointOnFirst(in Point2D aPoint) => _pointA = aPoint;

    internal void PointOnSecond(in Point2D aPoint) => _pointB = aPoint;

    internal void Distance(double d) => _distance = d;

    /// <summary>The reverse connexion: the first point is the second point and conversely.</summary>
    public Connexion Reverse()
        => new(_lineB, _lineA, _itemB, _itemA, _distance, _parameterOnB, _parameterOnA, _pointB, _pointA);

    /// <summary>True if the first point is on the same line as the first point of C2 and after it on the line.
    /// Sense is 1 if C2 is on the left of its first line, else -1.</summary>
    public bool IsAfter(Connexion c2, double sense)
    {
        if (_lineA != c2.IndexFirstLine())
        {
            return false;
        }
        if (_itemA > c2.IndexItemOnFirst())
        {
            return true;
        }
        else if (_itemA == c2.IndexItemOnFirst())
        {
            if (_parameterOnA > c2.ParameterOnFirst())
            {
                return true;
            }
            else if (_parameterOnA == c2.ParameterOnFirst())
            {
                Vector2D vect1 = new(c2.PointOnFirst(), c2.PointOnSecond());
                Vector2D vect2 = new(_pointA, _pointB);
                if ((vect1 ^ vect2) * sense > 0)
                {
                    return true;
                }
            }
        }
        return false;
    }
}
