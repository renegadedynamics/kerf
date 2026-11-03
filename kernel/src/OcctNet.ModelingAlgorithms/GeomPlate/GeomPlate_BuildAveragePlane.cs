// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildAveragePlane.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_BuildAveragePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The average plane of a set of points (or, when the points are aligned, their line), and the UV box of the points
/// in it; and the half-space test GeomPlate_BuildPlateSurface uses to choose a plane from the normals of its
/// constraints.
/// </summary>
/// <remarks>
/// Computed in the constructor. It keeps the caller's points, read only. <see cref="Plane"/> and <see cref="Line"/>
/// freeze the geometry they return (the same object each time).
/// </remarks>
public class GeomPlate_BuildAveragePlane
{
    private readonly NCollection_HArray1<Point3D> myPts;
    private double myUmax;
    private double myVmax;
    private double myVmin;
    private double myUmin;
    private readonly Geom_Plane? myPlane;
    private readonly double myTol;
    private readonly Geom_Line? myLine;
    private Vector3D myOX;
    private Vector3D myOY;
    private Point3D myG;
    private readonly int myNbBoundPoints;

    /// <summary>
    /// The average plane of <paramref name="Pts"/>, the first <paramref name="NbBoundPoints"/> of which are on the
    /// boundary.
    /// </summary>
    /// <param name="Pts">The points.</param>
    /// <param name="NbBoundPoints">The number of boundary points, first in <paramref name="Pts"/>.</param>
    /// <param name="Tol">The tolerance of the inertia axes.</param>
    /// <param name="POption">1: the plane through the barycentre with the normal found; 2: the axes of inertia of
    /// the points projected on it.</param>
    /// <param name="NOption">1: the normal of the axes of inertia (or of the boundary polygon if they differ by more
    /// than 60 degrees); 2: the normal of the boundary polygon.</param>
    public GeomPlate_BuildAveragePlane(NCollection_HArray1<Point3D> Pts, int NbBoundPoints, double Tol, int POption,
                                       int NOption)
    {
        myPts = Pts;
        myTol = Tol;
        myNbBoundPoints = NbBoundPoints;
        Vector3D OZ = DefPlan(NOption);

        if (OZ.SquareMagnitude() > 0)
        {
            if (POption == 1)
            {
                myPlane = new Geom_Plane(myG, new Direction3D(OZ));
                myOX = new Vector3D(myPlane.Pln().XAxis().Direction());
                myOY = new Vector3D(myPlane.Pln().YAxis().Direction());
            }
            else
            {
                BasePlan(OZ);
                Direction3D NDir = new Direction3D(myOX ^ myOY);
                Direction3D UDir = new Direction3D(myOX);
                CoordinateSystem triedre = new CoordinateSystem(myG, NDir, UDir);
                myPlane = new Geom_Plane(triedre);
            }
            int i, nb = myPts.Length();
            Plane3D P = myPlane.Pln();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(P, myG, out myUmax, out myVmax);
            myUmin = myUmax;
            myVmin = myVmax;
            double U = 0, V = 0;
            for (i = 1; i <= nb; i++)
            {
                global::Kerf.Engine.Geometry.ElSLib.Parameters(P, myPts.Value(i), out U, out V);
                if (myUmax < U)
                {
                    myUmax = U;
                }
                if (myUmin > U)
                {
                    myUmin = U;
                }
                if (myVmax < V)
                {
                    myVmax = V;
                }
                if (myVmin > V)
                {
                    myVmin = V;
                }
            }
        }

        if (IsLine())
        {
            myLine = new Geom_Line(myG, new Direction3D(myOX));
        }
    }

    /// <summary>
    /// The plane through the axis of inertia of <paramref name="Pts"/> whose normal makes the smallest largest angle
    /// with <paramref name="Normals"/>.
    /// </summary>
    public GeomPlate_BuildAveragePlane(NCollection_Sequence<Vector3D> Normals, NCollection_HArray1<Point3D> Pts)
    {
        myPts = Pts;
        int i, j, k, n, m;

        Vector3D BestVec = new Vector3D();
        int NN = Normals.Length();

        if (NN == 1)
        {
            BestVec = Normals.Value(1);
        }
        else if (NN == 2)
        {
            BestVec = Normals.Value(1) + Normals.Value(2);
            double aSqMagn = BestVec.SquareMagnitude();
            if (aSqMagn < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                double aSq1 = Normals.Value(1).SquareMagnitude(), aSq2 = Normals.Value(2).SquareMagnitude();

                if (aSq1 > aSq2)
                {
                    BestVec = Normals.Value(1).Normalized();
                }
                else
                {
                    BestVec = Normals.Value(2).Normalized();
                }
            }
            else
            {
                BestVec.Divide(Math.Sqrt(aSqMagn));
            }
        }
        else // the common case
        {
            double MaxAngle = 0.0;
            for (i = 1; i <= NN - 1; i++)
            {
                for (j = i + 1; j <= NN; j++)
                {
                    double Angle = Normals.Value(i).Angle(Normals.Value(j));
                    if (Angle > MaxAngle)
                    {
                        MaxAngle = Angle;
                    }
                }
            }
            MaxAngle *= 1.2;
            MaxAngle /= 2.0;
            int Nint = 50;

            NCollection_Array1<Vector3D> OptVec = new NCollection_Array1<Vector3D>(1, NN * (NN - 1) / 2);
            NCollection_Array1<double> OptScal = new NCollection_Array1<double>(1, NN * (NN - 1) / 2);
            Vector3D Vec, Vec1;
            Direction3D Cross1, Cross2;

            k = 1;
            for (i = 1; i <= NN - 1; i++)
            {
                for (j = i + 1; j <= NN; j++, k++)
                {
                    OptScal[k] = RealFirst();

                    double Step = MaxAngle / Nint;
                    Vec = Normals.Value(i) + Normals.Value(j);

                    double aSqMagn = Vec.SquareMagnitude();

                    if (aSqMagn < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        continue;
                    }

                    Vec.Divide(Math.Sqrt(aSqMagn));

                    Cross1 = new Direction3D(Normals.Value(i) ^ Normals.Value(j));
                    Cross2 = new Direction3D(Vec ^ new Vector3D(Cross1));
                    Axis3D Axe = new Axis3D(new Point3D(0, 0, 0), Cross2);

                    Vec1 = Vec.Rotated(Axe, -MaxAngle);

                    for (n = 0; n <= 2 * Nint; n++)
                    {
                        Vec1.Rotate(Axe, Step);
                        double minScal = RealLast();
                        for (m = 1; m <= NN; m++)
                        {
                            double Scal = Vec1 * Normals.Value(m);
                            if (Scal < minScal)
                            {
                                minScal = Scal;
                            }
                        }
                        if (minScal > OptScal[k])
                        {
                            OptScal[k] = minScal;
                            OptVec[k] = Vec1;
                        }
                    }
                } // for i, for j
            }
            // Find maximum among all maximums

            double BestScal = RealFirst();
            int Index = 0;
            for (k = 1; k <= OptScal.Length(); k++)
            {
                if (OptScal[k] > BestScal)
                {
                    BestScal = OptScal[k];
                    Index = k;
                }
            }
            BestVec = OptVec[Index];
        }

        // Making the plane myPlane
        Frame3D Axe2;
        bool IsSingular;
        NCollection_Array1<Point3D> PtsArray = new NCollection_Array1<Point3D>(1, myPts.Length());
        for (i = 1; i <= myPts.Length(); i++)
        {
            PtsArray[i] = myPts.Value(i);
        }
        GeomLib.AxeOfInertia(PtsArray, out Axe2, out IsSingular);
        Direction3D BestDir = new Direction3D(BestVec);
        Direction3D XDir = BestDir ^ Axe2.XDirection();
        XDir ^= BestDir;

        CoordinateSystem Axe3 = new CoordinateSystem(Axe2.Location(), BestDir, XDir);
        myPlane = new Geom_Plane(Axe3);

        // Initializing myUmin, myVmin, myUmax, myVmax
        Plane3D Pln = myPlane.Pln();
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Pln, Axe2.Location(), out myUmax, out myVmax);
        myUmin = myUmax;
        myVmin = myVmax;
        double U, V;
        for (i = 1; i <= myPts.Length(); i++)
        {
            Vector3D aVec = new Vector3D(Pln.Location(), myPts.Value(i));
            Vector3D NormVec = new Vector3D(Pln.Axis().Direction());
            NormVec = (aVec * NormVec) * NormVec;

            global::Kerf.Engine.Geometry.ElSLib.Parameters(Pln, myPts.Value(i).Translated(-NormVec), out U, out V);
            if (U > myUmax)
            {
                myUmax = U;
            }
            if (U < myUmin)
            {
                myUmin = U;
            }
            if (V > myVmax)
            {
                myVmax = V;
            }
            if (V < myVmin)
            {
                myVmin = V;
            }
        }
        // Initializing myOX, myOY
        myOX = new Vector3D(myPlane.Pln().XAxis().Direction());
        myOY = new Vector3D(myPlane.Pln().YAxis().Direction());
    }

    /// <summary>The average plane (frozen).</summary>
    /// <exception cref="NoSuchObjectException">The points are aligned (<see cref="IsLine"/>).</exception>
    public Geom_Plane? Plane()
    {
        Standard_NoSuchObject_Raise_if(
            IsLine(),
            "Cannot use the function 'GeomPlate_BuildAveragePlane::Plane()', the Object is a 'Geom_Line'");
        myPlane?.Freeze();
        return myPlane;
    }

    /// <summary>The bounds of the points' parameters in the plane.</summary>
    public void MinMaxBox(out double Umin, out double Umax, out double Vmin, out double Vmax)
    {
        Umax = myUmax;
        Umin = myUmin;
        Vmax = myVmax;
        Vmin = myVmin;
    }

    private Vector3D DefPlan(int NOption)
    {
        Point3D GB = new Point3D();
        Vector3D A = new Vector3D(), B = new Vector3D(), C = new Vector3D(), D;
        Vector3D OZ = new Vector3D();
        int i, nb = myPts.Length();
        GB.SetCoord(0.0, 0.0, 0.0);
        for (i = 1; i <= nb; i++)
        {
            GB.SetCoord(1, (GB.Coord(1) + myPts.Value(i).Coord(1)));
            GB.SetCoord(2, (GB.Coord(2) + myPts.Value(i).Coord(2)));
            GB.SetCoord(3, (GB.Coord(3) + myPts.Value(i).Coord(3)));
        }
        myG.SetCoord(1, (GB.Coord(1) / nb));
        myG.SetCoord(2, (GB.Coord(2) / nb));
        myG.SetCoord(3, (GB.Coord(3) / nb));

        if (NOption == 1)
        {
            Frame3D Axe;
            bool IsSingular;
            GeomLib.AxeOfInertia(myPts.Array1(), out Axe, out IsSingular, myTol);

            myOX = new Vector3D(Axe.XDirection());
            myOY = new Vector3D(Axe.YDirection());

            OZ = new Vector3D(Axe.Direction());

            if (myNbBoundPoints != 0 && myPts.Length() != myNbBoundPoints)
            {
                A.SetCoord(0.0, 0.0, 0.0);
                for (i = 3; i <= myNbBoundPoints; i++)
                {
                    B.SetCoord(myPts.Value(i - 1).Coord(1) - myPts.Value(1).Coord(1),
                               myPts.Value(i - 1).Coord(2) - myPts.Value(1).Coord(2),
                               myPts.Value(i - 1).Coord(3) - myPts.Value(1).Coord(3));
                    C.SetCoord(myPts.Value(i).Coord(1) - myPts.Value(1).Coord(1),
                               myPts.Value(i).Coord(2) - myPts.Value(1).Coord(2),
                               myPts.Value(i).Coord(3) - myPts.Value(1).Coord(3));
                    D = B ^ C;
                    A.SetCoord(1, A.Coord(1) + D.Coord(1));
                    A.SetCoord(2, A.Coord(2) + D.Coord(2));
                    A.SetCoord(3, A.Coord(3) + D.Coord(3));
                }
                Vector3D OZ1 = A;
                double theAngle = OZ.Angle(OZ1);
                if (theAngle > M_PI / 2)
                {
                    theAngle = M_PI - theAngle;
                }
                if (theAngle > M_PI / 3)
                {
                    OZ = OZ1;
                }
            }
        }

        else if (NOption == 2)
        {
            A.SetCoord(0.0, 0.0, 0.0);
            for (i = 3; i <= myNbBoundPoints; i++)
            {
                B.SetCoord(myPts.Value(i - 1).Coord(1) - myPts.Value(1).Coord(1),
                           myPts.Value(i - 1).Coord(2) - myPts.Value(1).Coord(2),
                           myPts.Value(i - 1).Coord(3) - myPts.Value(1).Coord(3));
                C.SetCoord(myPts.Value(i).Coord(1) - myPts.Value(1).Coord(1),
                           myPts.Value(i).Coord(2) - myPts.Value(1).Coord(2),
                           myPts.Value(i).Coord(3) - myPts.Value(1).Coord(3));
                D = B ^ C;
                A.SetCoord(1, A.Coord(1) + D.Coord(1));
                A.SetCoord(2, A.Coord(2) + D.Coord(2));
                A.SetCoord(3, A.Coord(3) + D.Coord(3));
            }
            OZ = A;
        }
        return OZ;
    }

    private void BasePlan(in Vector3D OZ)
    {
        Matrix M = new Matrix(1, 3, 1, 3);
        M.Init(0.0);
        Vector3D Proj = new Vector3D();
        int i, nb = myPts.Length();
        double scal;

        for (i = 1; i <= nb; i++)
        {
            Proj.SetCoord(1, myPts.Value(i).Coord(1) - myG.Coord(1));
            Proj.SetCoord(2, myPts.Value(i).Coord(2) - myG.Coord(2));
            Proj.SetCoord(3, myPts.Value(i).Coord(3) - myG.Coord(3));
            scal = Proj.Coord(1) * OZ.Coord(1) + Proj.Coord(2) * OZ.Coord(2) + Proj.Coord(3) * OZ.Coord(3);
            scal /= OZ.Coord(1) * OZ.Coord(1) + OZ.Coord(2) * OZ.Coord(2) + OZ.Coord(3) * OZ.Coord(3);
            Proj.SetCoord(1, Proj.Coord(1) - scal * OZ.Coord(1));
            Proj.SetCoord(2, Proj.Coord(2) - scal * OZ.Coord(2));
            Proj.SetCoord(3, Proj.Coord(3) - scal * OZ.Coord(3));
            M[1, 1] += Proj.Coord(1) * Proj.Coord(1);
            M[2, 2] += Proj.Coord(2) * Proj.Coord(2);
            M[3, 3] += Proj.Coord(3) * Proj.Coord(3);
            M[1, 2] += Proj.Coord(1) * Proj.Coord(2);
            M[1, 3] += Proj.Coord(1) * Proj.Coord(3);
            M[2, 3] += Proj.Coord(2) * Proj.Coord(3);
        }
        M[2, 1] = M[1, 2];
        M[3, 1] = M[1, 3];
        M[3, 2] = M[2, 3];
        Jacobi J = new Jacobi(M);
        double n1, n2, n3;
        math_Vector V1 = new math_Vector(1, 3), V2 = new math_Vector(1, 3), V3 = new math_Vector(1, 3);
        n1 = J.Value(1);
        n2 = J.Value(2);
        n3 = J.Value(3);

        double r1 = Min(Min(n1, n2), n3), r2;
        int m1, m2, m3;
        if (r1 == n1)
        {
            m1 = 1;
            r2 = Min(n2, n3);
            if (r2 == n2)
            {
                m2 = 2;
                m3 = 3;
            }
            else
            {
                m2 = 3;
                m3 = 2;
            }
        }
        else
        {
            if (r1 == n2)
            {
                m1 = 2;
                r2 = Min(n1, n3);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 3;
                }
                else
                {
                    m2 = 3;
                    m3 = 1;
                }
            }
            else
            {
                m1 = 3;
                r2 = Min(n1, n2);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 2;
                }
                else
                {
                    m2 = 2;
                    m3 = 1;
                }
            }
        }
        J.Vector(m1, V1);
        J.Vector(m2, V2);
        J.Vector(m3, V3);

        if (((Math.Abs(n1) <= myTol) && (Math.Abs(n2) <= myTol))
            || ((Math.Abs(n2) <= myTol) && (Math.Abs(n3) <= myTol))
            || ((Math.Abs(n1) <= myTol) && (Math.Abs(n3) <= myTol)))
        {
            myOX.SetCoord(V3[1], V3[2], V3[3]);
            myOY = OZ ^ myOX;
        }
        else
        {
            myOX.SetCoord(V3[1], V3[2], V3[3]);
            myOY.SetCoord(V2[1], V2[2], V2[3]);
        }
    }

    /// <summary>The line of the points (frozen).</summary>
    /// <exception cref="NoSuchObjectException">The points aren't aligned (<see cref="IsPlane"/>).</exception>
    public Geom_Line? Line()
    {
        Standard_NoSuchObject_Raise_if(
            IsPlane(),
            "Cannot use the function 'GeomPlate_BuildAveragePlane::Line()', the Object is a 'Geom_Plane'");
        myLine?.Freeze();
        return myLine;
    }

    public bool IsPlane()
    {
        Vector3D OZ = myOX ^ myOY;
        return OZ.SquareMagnitude() != 0;
    }

    public bool IsLine()
    {
        Vector3D OZ = myOX ^ myOY;
        return OZ.SquareMagnitude() == 0;
    }

    /// <summary>
    /// Adds <paramref name="NewNormals"/> to <paramref name="Normals"/> if all of them still lie in a common
    /// half-space, updating the boundary pairs <paramref name="Bset"/>; otherwise leaves both unchanged and returns
    /// false.
    /// </summary>
    public static bool HalfSpace(NCollection_Sequence<Vector3D> NewNormals, NCollection_Sequence<Vector3D> Normals,
                                 NCollection_Sequence<GeomPlate_Aij> Bset, double LinTol, double AngTol)
    {
        double SquareTol = LinTol * LinTol;

        NCollection_Sequence<Vector3D> SaveNormals;
        NCollection_Sequence<GeomPlate_Aij> SaveBset;
        // 1
        SaveNormals = new NCollection_Sequence<Vector3D>(Normals);
        SaveBset = new NCollection_Sequence<GeomPlate_Aij>(Bset);

        Vector3D Cross, NullVec = new Vector3D(0, 0, 0);
        NCollection_Sequence<GeomPlate_Aij> B1set = new NCollection_Sequence<GeomPlate_Aij>(),
                                            B2set = new NCollection_Sequence<GeomPlate_Aij>();
        int i, j, k;

        i = 1;
        if (Normals.IsEmpty())
        {
            if (NewNormals.Length() == 1)
            {
                Normals.Append(NewNormals.Last());
                return true;
            }
            // 2
            Cross = NewNormals.Value(1) ^ NewNormals.Value(2);
            if (Cross.SquareMagnitude() <= SquareTol)
            {
                return false;
            }

            Cross.Normalize();
            Bset.Append(new GeomPlate_Aij(1, 2, Cross));
            Bset.Append(new GeomPlate_Aij(2, 1, -Cross));
            Normals.Append(NewNormals.Value(1));
            Normals.Append(NewNormals.Value(2));
            i = 3;
        }

        for (; i <= NewNormals.Length(); i++)
        {
            // 3
            double Scal;
            for (j = 1; j <= Bset.Length(); j++)
            {
                if ((Scal = Bset.Value(j).Vec * NewNormals.Value(i)) >= -LinTol)
                {
                    B2set.Append(Bset.Value(j));
                }
            }

            int ii = Normals.Length() + 1;
            for (j = 1; j <= ii - 1; j++)
            {
                if (Normals.Value(j).SquareMagnitude() == 0.0)
                {
                    continue;
                }
                // 4
                Cross = NewNormals.Value(i) ^ Normals.Value(j);
                if (Cross.SquareMagnitude() <= SquareTol)
                {
                    Normals.Assign(SaveNormals);
                    Bset.Assign(SaveBset);
                    return false;
                }
                Cross.Normalize();
                bool isNew = true;
                for (k = 1; k <= B2set.Length(); k++)
                {
                    if (B2set.Value(k).Vec.IsOpposite(-Cross, AngTol)) // if (B2set(k).Vec.IsEqual( Cross, LinTol, AngTol ))
                    {
                        Vector3D Cross1, Cross2;
                        int ind1 = B2set.Value(k).Ind1, ind2 = B2set.Value(k).Ind2;
                        if (ind1 == ii || ind2 == ii)
                        {
                            isNew = false;
                            break;
                        }
                        Cross1 = Normals.Value(ind1) ^ NewNormals.Value(i);
                        Cross2 = Normals.Value(ind2) ^ NewNormals.Value(i);
                        if (Cross1.SquareMagnitude() <= SquareTol || Cross2.SquareMagnitude() <= SquareTol)
                        {
                            Normals.Assign(SaveNormals);
                            Bset.Assign(SaveBset);
                            return false;
                        }
                        if (Cross1.IsOpposite(Cross2, AngTol))
                        {
                            Cross2 = Normals.Value(ind1) ^ Normals.Value(ind2);
                            if (Cross1.IsOpposite(Cross2, AngTol))
                            {
                                Normals.Assign(SaveNormals);
                                Bset.Assign(SaveBset);
                                return false;
                            }
                        }
                        else
                        {
                            if (NewNormals.Value(i).Angle(Normals.Value(ind1)) > NewNormals.Value(i).Angle(Normals.Value(ind2)))
                            {
                                B2set[k].Ind2 = ind1;
                                B2set[k].Ind1 = ii;
                            }
                            else
                            {
                                B2set[k].Ind1 = ii;
                            }
                        }
                        isNew = false;
                        break;
                    }
                }
                if (isNew)
                {
                    B1set.Append(new GeomPlate_Aij(ii, j, Cross));
                }

                Cross.Reverse();
                isNew = true;
                for (k = 1; k <= B2set.Length(); k++)
                {
                    if (B2set.Value(k).Vec.IsOpposite(-Cross, AngTol)) // if (B2set(k).Vec.IsEqual( Cross, LinTol, AngTol ))
                    {
                        Vector3D Cross1, Cross2;
                        int ind1 = B2set.Value(k).Ind1, ind2 = B2set.Value(k).Ind2;
                        if (ind1 == ii || ind2 == ii)
                        {
                            isNew = false;
                            break;
                        }
                        Cross1 = Normals.Value(ind1) ^ NewNormals.Value(i);
                        Cross2 = Normals.Value(ind2) ^ NewNormals.Value(i);
                        if (Cross1.SquareMagnitude() <= SquareTol || Cross2.SquareMagnitude() <= SquareTol)
                        {
                            Normals.Assign(SaveNormals);
                            Bset.Assign(SaveBset);
                            return false;
                        }
                        if (Cross1.IsOpposite(Cross2, AngTol))
                        {
                            Cross2 = Normals.Value(ind1) ^ Normals.Value(ind2);
                            if (Cross1.IsOpposite(Cross2, AngTol))
                            {
                                Normals.Assign(SaveNormals);
                                Bset.Assign(SaveBset);
                                return false;
                            }
                        }
                        else
                        {
                            if (NewNormals.Value(i).Angle(Normals.Value(ind1)) > NewNormals.Value(i).Angle(Normals.Value(ind2)))
                            {
                                B2set[k].Ind2 = ind1;
                                B2set[k].Ind1 = ii;
                            }
                            else
                            {
                                B2set[k].Ind1 = ii;
                            }
                        }
                        isNew = false;
                        break;
                    }
                }
                if (isNew)
                {
                    B1set.Append(new GeomPlate_Aij(ii, j, Cross));
                }
            }

            // 5
            for (j = 1; j <= B1set.Length(); j++)
            {
                bool isGEnull = true;
                for (k = 1; k <= Normals.Length(); k++)
                {
                    if (Normals.Value(k).SquareMagnitude() == 0.0)
                    {
                        continue;
                    }
                    if (B1set.Value(j).Vec * Normals.Value(k) < -LinTol)
                    {
                        isGEnull = false;
                        break;
                    }
                }
                if (isGEnull)
                {
                    B2set.Append(B1set.Value(j));
                }
            }

            // 6
            if (B2set.IsEmpty())
            {
                Normals.Assign(SaveNormals);
                Bset.Assign(SaveBset);
                return false;
            }

            // 7
            Bset.Assign(B2set);
            B2set.Clear();
            B1set.Clear();
            Normals.Append(NewNormals.Value(i));

            // 8
            for (j = 1; j <= Normals.Length(); j++)
            {
                if (Normals.Value(j).SquareMagnitude() == 0.0)
                {
                    continue;
                }
                bool isFound = false;
                for (k = 1; k <= Bset.Length(); k++)
                {
                    if (j == Bset.Value(k).Ind1 || j == Bset.Value(k).Ind2)
                    {
                        isFound = true;
                        break;
                    }
                }
                if (!isFound)
                {
                    Normals[j] = NullVec;
                }
            }
        }

        return true;
    }
}
