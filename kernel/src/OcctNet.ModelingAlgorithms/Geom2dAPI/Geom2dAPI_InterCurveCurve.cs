// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_InterCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The intersections between two 2D curves, or the self-intersections of one: points where the curves cross, and
/// segments where they are tangent along a portion.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117). It keeps copies of the curves, as OCCT does, for <see cref="Segment"/>.
/// </remarks>
public class Geom2dAPI_InterCurveCurve
{
    private bool myIsDone;
    private Geom2d_Curve? myCurve1;
    private Geom2d_Curve? myCurve2;
    private Geom2dInt_GInter myIntersector = new();

    /// <summary>An empty intersector; use <see cref="Init(Geom2d_Curve, Geom2d_Curve, double)"/>.</summary>
    public Geom2dAPI_InterCurveCurve()
    {
        myIsDone = false;
    }

    /// <summary>The intersections between <paramref name="C1"/> and <paramref name="C2"/>.</summary>
    public Geom2dAPI_InterCurveCurve(Geom2d_Curve C1, Geom2d_Curve C2, double Tol = 1.0e-6)
    {
        Init(C1, C2, Tol);
    }

    /// <summary>
    /// The self-intersections of <paramref name="C1"/>. <paramref name="Tol"/> is the precision of the points and,
    /// for tangential intersections, the size of the segments.
    /// </summary>
    public Geom2dAPI_InterCurveCurve(Geom2d_Curve C1, double Tol = 1.0e-6)
    {
        Init(C1, Tol);
    }

    /// <summary>Computes the intersections between <paramref name="C1"/> and <paramref name="C2"/>.</summary>
    /// <exception cref="NullObjectException">A curve is null.</exception>
    public void Init(Geom2d_Curve C1, Geom2d_Curve C2, double Tol = 1.0e-6)
    {
        Standard_NullObject_Raise_if(C1 is null, "Geom2dAPI_InterCurveCurve::Init - C1 is null");
        Standard_NullObject_Raise_if(C2 is null, "Geom2dAPI_InterCurveCurve::Init - C2 is null");
        myCurve1 = C1!.Copy() as Geom2d_Curve;
        myCurve2 = C2!.Copy() as Geom2d_Curve;

        Geom2dAdaptor_Curve AC1 = new(C1);
        Geom2dAdaptor_Curve AC2 = new(C2);
        myIntersector = new Geom2dInt_GInter(AC1, AC2, Tol, Tol);
        myIsDone = myIntersector.IsDone();
    }

    /// <summary>Computes the self-intersections of <paramref name="C1"/>.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public void Init(Geom2d_Curve C1, double Tol = 1.0e-6)
    {
        Standard_NullObject_Raise_if(C1 is null, "Geom2dAPI_InterCurveCurve::Init - C1 is null");
        myCurve1 = C1!.Copy() as Geom2d_Curve;
        myCurve2 = null;

        Geom2dAdaptor_Curve AC1 = new(C1);
        myIntersector = new Geom2dInt_GInter(AC1, Tol, Tol);
        myIsDone = myIntersector.IsDone();
    }

    /// <summary>The number of intersection points (cross intersections); 0 if none were found.</summary>
    public int NbPoints()
    {
        if (myIsDone)
        {
            return myIntersector.NbPoints();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The intersection point of index <paramref name="Index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="Index"/> is not in [1, NbPoints].</exception>
    public Point2D Point(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoints(), "Geom2dAPI_InterCurveCurve::Point");

        return (myIntersector.Point(Index)).Value();
    }

    /// <summary>The number of tangential intersections (segments); 0 if none were found.</summary>
    public int NbSegments()
    {
        if (myIsDone)
        {
            return myIntersector.NbSegments();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>
    /// The intersection segment of index <paramref name="theIndex"/>, as the portions of the first and the second
    /// curve it covers (both of the one curve for a self-intersection).
    /// </summary>
    /// <remarks>
    /// If the segment is opposite, the second portion runs from the parameter of the segment's last point to that
    /// of its first point.
    /// </remarks>
    /// <exception cref="OutOfRangeException"><paramref name="theIndex"/> is not in [1, NbSegments].</exception>
    /// <exception cref="NullObjectException">The algorithm was not initialised.</exception>
    public void Segment(int theIndex, out Geom2d_Curve theCurve1, out Geom2d_Curve theCurve2)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > NbSegments(), "Geom2dAPI_InterCurveCurve::Segment");

        Standard_NullObject_Raise_if(myCurve1 is null, "Geom2dAPI_InterCurveCurve::Segment");

        double aU1, aU2, aV1, aV2;
        aU1 = myCurve1!.FirstParameter();
        aU2 = myCurve1.LastParameter();
        if (myCurve2 is null)
        {
            aV1 = aU1;
            aV2 = aU2;
        }
        else
        {
            aV1 = myCurve2.FirstParameter();
            aV2 = myCurve2.LastParameter();
        }

        IntRes2d_IntersectionSegment aSeg = myIntersector.Segment(theIndex);
        bool isOpposite = aSeg.IsOpposite();

        if (aSeg.HasFirstPoint())
        {
            IntRes2d_IntersectionPoint anIPF = aSeg.FirstPoint();
            aU1 = anIPF.ParamOnFirst();

            if (isOpposite)
            {
                aV2 = anIPF.ParamOnSecond();
            }
            else
            {
                aV1 = anIPF.ParamOnSecond();
            }
        }

        if (aSeg.HasLastPoint())
        {
            IntRes2d_IntersectionPoint anIPL = aSeg.LastPoint();
            aU2 = anIPL.ParamOnFirst();

            if (isOpposite)
            {
                aV1 = anIPL.ParamOnSecond();
            }
            else
            {
                aV2 = anIPL.ParamOnSecond();
            }
        }

        theCurve1 = new Geom2d_TrimmedCurve(myCurve1, aU1, aU2);
        if (myCurve2 is null)
        {
            theCurve2 = new Geom2d_TrimmedCurve(myCurve1, aV1, aV2);
        }
        else
        {
            theCurve2 = new Geom2d_TrimmedCurve(myCurve2, aV1, aV2);
        }
    }

    /// <summary>The intersection algorithm used.</summary>
    public Geom2dInt_GInter Intersector() => myIntersector;
}
