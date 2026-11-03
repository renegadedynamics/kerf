// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepIntCurveSurface/BRepIntCurveSurface_Inter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepIntCurveSurface/BRepIntCurveSurface_Inter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The intersection of a curve with the faces of a shape: its points IN or ON the faces, face by face, iterated with
/// <see cref="More"/> and <see cref="Next"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). To intersect several curves with one shape, <see cref="Load"/> it once and
/// <see cref="Init(CurveAdaptor)"/> each curve; the faces' boxes are computed once. It only reads the shape.
/// </remarks>
public class BRepIntCurveSurface_Inter
{
    private double myTolerance;
    private CurveAdaptor? myCurve;
    private readonly IntCurveSurface_HInter myIntcs = new();
    private int myCurrentindex;
    private int myCurrentnbpoints;
    private readonly BRepTopAdaptor_TopolTool myFastClass;
    private Containment myCurrentstate;
    private double myCurrentU;
    private double myCurrentV;
    private Box myCurveBox = new();
    private int myIndFace;
    private readonly NCollection_Sequence<Shape> myFaces = new();
    private NCollection_HArray1<Box>? myFaceBoxes;

    /// <summary>An empty intersector.</summary>
    public BRepIntCurveSurface_Inter()
    {
        myFastClass = new BRepTopAdaptor_TopolTool();
        Clear();
    }

    /// <summary>Loads the shape and the curve, with the tolerance <paramref name="theTol"/> of the classification.</summary>
    public void Init(in Shape theShape, CurveAdaptor theCurve, double theTol)
    {
        Load(theShape, theTol);
        Init(theCurve);
    }

    /// <summary>Loads the shape and the line, with the tolerance <paramref name="theTol"/> of the classification.</summary>
    public void Init(in Shape theShape, in Line3D theLine, double theTol)
    {
        var geomline = new LineCurve3D(theLine);
        var aCurve = new CurveAdaptor(geomline);
        Load(theShape, theTol);
        Init(aCurve);
    }

    /// <summary>Resets the iteration.</summary>
    protected void Clear()
    {
        myCurrentindex = 0;
        myCurrentnbpoints = 0;
        myIndFace = 0;
        myCurrentstate = Unknown;
        myCurrentU = 0;
        myCurrentV = 0;
    }

    /// <summary>Loads the shape, with the tolerance <paramref name="theTol"/> of the classification.</summary>
    public void Load(in Shape theShape, double theTol)
    {
        Clear();
        myFaces.Clear();
        myFaceBoxes = null;
        myTolerance = theTol;
        var explorer = new Explorer(theShape, global::Kerf.Topology.ShapeType.Face);
        for (; explorer.More(); explorer.Next())
        {
            myFaces.Append(explorer.Current());
        }
    }

    /// <summary>Intersects the curve with the loaded shape and moves to the first point.</summary>
    /// <remarks>The intersector keeps a copy of the adaptor (OCCT's), sharing its curve.</remarks>
    public void Init(CurveAdaptor theCurve)
    {
        Clear();
        myCurveBox.SetVoid();
        double aFirst = theCurve.FirstParameter();
        double aLast = theCurve.LastParameter();
        myCurve = (CurveAdaptor)theCurve.ShallowCopy();
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aFirst) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(aLast))
        {
            Add3dCurve.Add(myCurve, 0.0, ref myCurveBox);
        }

        Find();
    }

    /// <summary>True if there is a current point.</summary>
    public bool More() => (myIndFace <= myFaces.Length());

    /// <summary>Moves to the next point.</summary>
    public void Next()
    {
        if (myCurrentnbpoints != 0)
        {
            myCurrentindex++;
        }

        Find();
    }

    /// <summary>Finds the next point IN or ON a face, from the current one.</summary>
    protected void Find()
    {
        if (myCurrentnbpoints != 0 && myCurrentindex <= myCurrentnbpoints && FindPoint())
        {
            return;
        }

        myCurrentnbpoints = 0;
        myCurrentindex = 0;

        int i = myIndFace + 1;
        for (; i <= myFaces.Length(); i++)
        {
            Shape aCurface = myFaces[i];
            myFaceBoxes ??= new NCollection_HArray1<Box>(1, myFaces.Length());

            ref Box aFaceBox = ref myFaceBoxes[i];
            if (aFaceBox.IsVoid)
            {
                BRepBndLib.Add(aCurface, ref aFaceBox);
                aFaceBox.SetGap(myTolerance); // Precision::Confusion());
            }

            bool isOut = (myCurve!.GetType() == Line
                              ? aFaceBox.IsOut(myCurve.Line())
                              : (!myCurveBox.IsVoid ? aFaceBox.IsOut(myCurveBox) : false));
            if (isOut)
            {
                continue;
            }

            var aSurfForFastClass = new FaceSurfaceAdaptor(ShapeCasts.Face(aCurface));
            myIntcs.Perform(myCurve, aSurfForFastClass);
            myCurrentnbpoints = myIntcs.NbPoints();
            if (myCurrentnbpoints == 0)
            {
                continue;
            }

            myFastClass.Initialize((SurfaceAdaptorBase)aSurfForFastClass);
            myIndFace = i;
            if (FindPoint())
            {
                return;
            }

            myCurrentnbpoints = 0;
        }

        if (myCurrentnbpoints == 0 && i > myFaces.Length())
        {
            myIndFace = i;
            return;
        }
    }

    /// <summary>Finds the first point from the current index that is IN or ON the current face.</summary>
    protected bool FindPoint()
    {
        int j = (myCurrentindex == 0 ? 1 : myCurrentindex);

        for (; j <= myCurrentnbpoints; j++)
        {
            double anU = myIntcs.Point(j).U();
            double aV = myIntcs.Point(j).V();

            Point2D Puv = new(anU, aV);

            myCurrentstate = myFastClass.Classify(Puv, myTolerance);
            if (myCurrentstate == On || myCurrentstate == In)
            {
                myCurrentindex = j;
                myCurrentU = anU;
                myCurrentV = aV;
                return true;
            }
        }

        return false;
    }

    /// <summary>The current point.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public IntCurveSurface_IntersectionPoint Point()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        IntCurveSurface_IntersectionPoint ICPS = myIntcs.Point(myCurrentindex);
        return (new IntCurveSurface_IntersectionPoint(ICPS.Pnt(),
                                                      myCurrentU, // ICPS.U(),
                                                      myCurrentV, // ICPS.V(),
                                                      ICPS.W(),
                                                      ICPS.Transition()));
        //-- return(myIntcs.Point(myCurrentindex));
    }

    /// <summary>The U parameter of the current point on the current face.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double U()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(myIntcs.Point(myCurrentindex).U());
        return (myCurrentU);
    }

    /// <summary>The V parameter of the current point on the current face.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double V()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(myIntcs.Point(myCurrentindex).V());
        return (myCurrentV);
    }

    /// <summary>The parameter of the current point on the curve.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public double W()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        return (myIntcs.Point(myCurrentindex).W());
    }

    /// <summary>The state of the current point in the current face: IN or ON.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public Containment State()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        //-- return(classifier.State());
        return (myCurrentstate);
    }

    /// <summary>The transition of the curve on the surface at the current point (IN, OUT or TANGENT).</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public IntCurveSurface_TransitionOnCurve Transition()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        return (myIntcs.Point(myCurrentindex).Transition());
    }

    /// <summary>The current face.</summary>
    public Face Face() => (ShapeCasts.Face(myFaces.Value(myIndFace)));

    /// <summary>The current point.</summary>
    /// <exception cref="NotDoneException">There is no current point.</exception>
    public Point3D Pnt()
    {
        if (myCurrentindex == 0)
        {
            throw new NotDoneException();
        }

        return (myIntcs.Point(myCurrentindex).Pnt());
    }
}
