// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Connexion.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Connexion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>A connexion links two lines of items (points or curves) in a set of lines: two points and their
/// parameters on the lines.</summary>
/// <remarks>A working object of MAT2d_MiniPath and MAT2d_Circuit (decision 21): the setters are internal. Dump (to
/// std::cout) is left out.</remarks>
public sealed class MAT2d_Connexion : Transient
{
    private int lineA;
    private int lineB;
    private int itemA;
    private int itemB;
    private double distance;
    private double parameterOnA;
    private double parameterOnB;
    private Point2D pointA;
    private Point2D pointB;

    public MAT2d_Connexion()
    {
        lineA = 0;
        lineB = 0;
        itemA = 0;
        itemB = 0;
        distance = 0.0;
        parameterOnA = 0.0;
        parameterOnB = 0.0;
        pointA = new Point2D();
        pointB = new Point2D();
    }

    public MAT2d_Connexion(int LineA, int LineB, int ItemA, int ItemB, double Distance, double ParameterOnA,
        double ParameterOnB, in Point2D PointA, in Point2D PointB)
    {
        lineA = LineA;
        lineB = LineB;
        itemA = ItemA;
        itemB = ItemB;
        distance = Distance;
        parameterOnA = ParameterOnA;
        parameterOnB = ParameterOnB;
        pointA = PointA;
        pointB = PointB;
    }

    /// <summary>The index of the first line.</summary>
    public int IndexFirstLine() => lineA;

    /// <summary>The index of the second line.</summary>
    public int IndexSecondLine() => lineB;

    /// <summary>The index of the item on the first line.</summary>
    public int IndexItemOnFirst() => itemA;

    /// <summary>The index of the item on the second line.</summary>
    public int IndexItemOnSecond() => itemB;

    /// <summary>The parameter of the point on the first line.</summary>
    public double ParameterOnFirst() => parameterOnA;

    /// <summary>The parameter of the point on the second line.</summary>
    public double ParameterOnSecond() => parameterOnB;

    /// <summary>The point on the first line.</summary>
    public Point2D PointOnFirst() => pointA;

    /// <summary>The point on the second line.</summary>
    public Point2D PointOnSecond() => pointB;

    /// <summary>The distance between the two points.</summary>
    public double Distance() => distance;

    internal void IndexFirstLine(int anIndex) => lineA = anIndex;

    internal void IndexSecondLine(int anIndex) => lineB = anIndex;

    internal void IndexItemOnFirst(int anIndex) => itemA = anIndex;

    internal void IndexItemOnSecond(int anIndex) => itemB = anIndex;

    internal void ParameterOnFirst(double aParameter) => parameterOnA = aParameter;

    internal void ParameterOnSecond(double aParameter) => parameterOnB = aParameter;

    internal void PointOnFirst(in Point2D aPoint) => pointA = aPoint;

    internal void PointOnSecond(in Point2D aPoint) => pointB = aPoint;

    internal void Distance(double d) => distance = d;

    /// <summary>The reverse connexion: the first point is the second point and conversely.</summary>
    public MAT2d_Connexion Reverse()
        => new(lineB, lineA, itemB, itemA, distance, parameterOnB, parameterOnA, pointB, pointA);

    /// <summary>True if the first point is on the same line as the first point of C2 and after it on the line.
    /// Sense is 1 if C2 is on the left of its first line, else -1.</summary>
    public bool IsAfter(MAT2d_Connexion C2, double Sense)
    {
        if (lineA != C2.IndexFirstLine())
        {
            return false;
        }
        if (itemA > C2.IndexItemOnFirst())
        {
            return true;
        }
        else if (itemA == C2.IndexItemOnFirst())
        {
            if (parameterOnA > C2.ParameterOnFirst())
            {
                return true;
            }
            else if (parameterOnA == C2.ParameterOnFirst())
            {
                Vector2D Vect1 = new(C2.PointOnFirst(), C2.PointOnSecond());
                Vector2D Vect2 = new(pointA, pointB);
                if ((Vect1 ^ Vect2) * Sense > 0)
                {
                    return true;
                }
            }
        }
        return false;
    }
}
