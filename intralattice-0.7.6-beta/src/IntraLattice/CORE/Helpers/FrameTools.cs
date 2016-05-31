using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;
using Rhino.DocObjects;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry.Intersect;
using Grasshopper;
using Rhino.Collections;
using IntraLattice.CORE.Data;


// Summary:     This class contains a set of static methods used by the frame components.
// ======================================================================================
// Author(s):   Aidan Kurtz (http://aidankurtz.com)

namespace IntraLattice.CORE.Helpers
{
    public class FrameTools
    {
        /// <summary>
        /// Removes duplicate/invalid/tiny curves and outputs the cleaned list.
        /// </summary>
        public static List<Curve> CleanNetwork(List<Curve> inputStruts, double tol)
        {
            var nodes = new Point3dList();
            var nodePairs = new List<IndexPair>();

            return CleanNetwork(inputStruts, tol, out nodes, out nodePairs);
        }
        /// <summary>
        /// Removes duplicate/invalid/tiny curves and outputs the cleaned list, and a list of unique nodes.
        /// </summary>
        public static List<Curve> CleanNetwork(List<Curve> inputStruts, double tol, out Point3dList nodes)
        {
            nodes = new Point3dList();
            var nodePairs = new List<IndexPair>();

            return CleanNetwork(inputStruts, tol, out nodes, out nodePairs);
        }
        /// <summary>
        /// Removes duplicate/invalid/tiny curves and outputs the cleaned list, a list of unique nodes and a list of node pairs.
        /// </summary>
        public static List<Curve> CleanNetwork(List<Curve> inputStruts, double tol, out Point3dList nodes, out List<IndexPair> nodePairs)
        {
            nodes = new Point3dList();
            nodePairs = new List<IndexPair>();

            var struts = new List<Curve>();

            // Loop over list of struts
            for (int i = 0; i < inputStruts.Count; i++)
            {
                Curve strut = inputStruts[i];
                // Unitize domain
                strut.Domain = new Interval(0, 1);
                // Minimum strut length (if user defined very small tolerance, use 100*rhinotolerance)
                double minLength = Math.Max(tol, 100*RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
                // If strut is invalid, ignore it
                if (strut == null || !strut.IsValid || strut.IsShort(minLength))
                {
                    continue;
                }

                Point3d[] pts = new Point3d[2] { strut.PointAtStart, strut.PointAtEnd };
                List<int> nodeIndices = new List<int>();
                // Loop over end points of strut
                // Check if node is already in nodes list, if so, we find its index instead of creating a new node
                for (int j = 0; j < 2; j++)
                {
                    Point3d pt = pts[j];
                    // Find closest node to current pt
                    int closestIndex = nodes.ClosestIndex(pt); 

                    // If node already exists (within tolerance), set the index
                    if (nodes.Count != 0 && pt.EpsilonEquals(nodes[closestIndex], tol))
                    {
                        nodeIndices.Add(closestIndex);
                    }
                    // If node doesn't exist
                    else
                    {
                        // Update lookup list
                        nodes.Add(pt);
                        nodeIndices.Add(nodes.Count - 1);
                    }
                }

                // We must ignore duplicate struts
                bool isDuplicate = false;
                IndexPair nodePair = new IndexPair(nodeIndices[0], nodeIndices[1]);

                int dupIndex = nodePairs.IndexOf(nodePair);
                // dupIndex equals -1 if nodePair not found, i.e. if it doesn't equal -1, a match was found
                if (nodePairs.Count != 0 && dupIndex != -1)
                {
                    // Check the curve midpoint to make sure it's a duplicate
                    Curve testStrut = struts[dupIndex];
                    Point3d ptA = strut.PointAt(0.5);
                    Point3d ptB = testStrut.PointAt(0.5);
                    if (ptA.EpsilonEquals(ptB, tol)) isDuplicate = true;
                }

                // So we only create the strut if it doesn't exist yet (check nodePairLookup list)
                if (!isDuplicate)
                {
                    // Update the lookup list
                    nodePairs.Add(nodePair);
                    strut.Domain = new Interval(0, 1);
                    struts.Add(strut);
                }
            }

            return struts;
        }

        /// <summary>
        /// Validates a GeometryBase design space as a brep or a mesh.
        /// </summary>
        public static int ValidateSpace(ref GeometryBase designSpace)
        {
            // Types: 0-invalid, 1-brep, 2-mesh, 3-solid surface
            int type = 0;

            if (designSpace.ObjectType == ObjectType.Brep)
            {
                type = 1;
            }
            else if (designSpace.ObjectType == ObjectType.Mesh && ((Mesh)designSpace).IsClosed)
            {
                type = 2;
            }
            else if (designSpace.ObjectType == ObjectType.Surface && ((Surface)designSpace).IsSolid)
            {
                type = 3;
            }

            return type;
        }

        /// <summary>
        /// Determines if a point is inside a geometry. (Brep, Mesh or closed Surface)
        /// </summary>
        public static bool IsPointInside(GeometryBase geometry, Point3d testPoint, int spaceType, double tol, bool strictlyIn)
        {
            bool isInside = false;

            switch (spaceType)
            {
                // Brep design space
                case 1:
                    isInside = ((Brep)geometry).IsPointInside(testPoint, tol, strictlyIn);
                    break;
                // Mesh design space
                case 2:
                    isInside = ((Mesh)geometry).IsPointInside(testPoint, tol, strictlyIn);
                    break;
                // Solid surface design space (must be converted to brep)
                case 3:
                    isInside = ((Surface)geometry).ToBrep().IsPointInside(testPoint, tol, strictlyIn);
                    break;
            }

            return isInside;
        }

        /// <summary>
        /// Computes the distance of a point to a given geometry. (Brep, Mesh or closed Surface)
        /// </summary>
        public static double DistanceTo(GeometryBase geometry, Point3d testPoint, int spaceType)
        {
            double distanceTo = 0;
            Point3d closestPoint;

            switch (spaceType)
            {
                // Brep design space
                case 1:
                    closestPoint = ((Brep)geometry).ClosestPoint(testPoint);
                    distanceTo = testPoint.DistanceTo(closestPoint);
                    break;
                // Mesh design space
                case 2:
                    closestPoint = ((Mesh)geometry).ClosestPoint(testPoint);
                    distanceTo = testPoint.DistanceTo(closestPoint);
                    break;
                // Solid surface design space (must be converted to brep)
                case 3:
                    closestPoint = ((Surface)geometry).ToBrep().ClosestPoint(testPoint);
                    distanceTo = testPoint.DistanceTo(closestPoint);
                    break;
            }

            return distanceTo;
        }

        ///<summary
        ///Calls the Voronoi GH Component on a collection of points
        ///</summary>

        public static DataTree<object> ConstructVoro(List<Point3d> pts, BoundingBox Bbox)
        {
            var cs = new TriangulationComponents.Component_VolumeVoronoi();
            //add the points list (input 0)
            var points = cs.Params.Input[0] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Point>;
            points.PersistentData.ClearData();
            var box = cs.Params.Input[1] as Grasshopper.Kernel.GH_PersistentGeometryParam<GH_Box>;
            for (int i = 0; i < (int)pts.Count; i++)
            {
                points.PersistentData.Append(new GH_Point(pts[i]));
            }

            box.PersistentData.Append(new GH_Box(Bbox));
            
            cs.ExpireSolution(true);


            //add to a dummy document so we can read outputs
            var doc = new Grasshopper.Kernel.GH_Document();
            doc.AddObject(cs, false);


            //Initialize temporary brep storage
            //List<> temp = new List <object>();

            cs.Params.Output[0].CollectData();
            cs.Params.Output[1].CollectData();
            DataTree<object> tempA = new DataTree<object>();
            //DataTree<object> tempB = new DataTree<object>();
           

            //Loop thru points, storing each branch in a temporary tree
            for (int j = pts.Count - 1; j >= 0; j--)
            {
                GH_Path pth = new GH_Path(j);
               
                tempA.Add(cs.Params.Output[0].VolatileData.get_Branch(0)[j], pth);  //this function returns a branch of a data tree
               
            }
            
            DataTree<object> rawStruts = tempA;
            
            //remove that component
            doc.RemoveObject(cs.Attributes, false);

            //Set output data 
            return rawStruts;
          }

        public static List<GH_Brep> SolidIntersect(DataTree<object> voroBrep, Brep space)
        {
            var cs = new SurfaceComponents.SolidComponents.Component_BooleanIntersection();
            //add the points list (input 0)
            var geo1 = cs.Params.Input[0] as Grasshopper.Kernel.GH_PersistentGeometryParam<Grasshopper.Kernel.Types.GH_Brep>;
            geo1.PersistentData.ClearData();
            var geo2 = cs.Params.Input[1] as Grasshopper.Kernel.GH_PersistentGeometryParam<GH_Brep>;

            for (int i = 0; i < voroBrep.Branch(0).Count; i++)
            {
                geo1.PersistentData.Append((GH_Brep)voroBrep.Branch(0)[i]);
            }


            geo2.PersistentData.Append(new GH_Brep(space));

            cs.ExpireSolution(true);



            //add to a dummy document so we can read outputs
            var doc = new Grasshopper.Kernel.GH_Document();
            doc.AddObject(cs, false);

            cs.Params.Output[0].CollectData();
            
            var brepTree = cs.Params.Output[0].VolatileData;
            System.Collections.Generic.List<GH_Brep> brepList = new List<GH_Brep>();
            List<GH_Brep> brepResult = new List<GH_Brep>();
         
            for (int i = 0; i < brepTree.PathCount; i++)
            {
                brepList.AddRange((System.Collections.Generic.List<Grasshopper.Kernel.Types.GH_Brep>) cs.Params.Output[0].VolatileData.get_Branch(i));
             }
            
            foreach (GH_Brep entry in brepList)
            {
                brepResult.Add(entry);
            }


            doc.RemoveObject(cs.Attributes, false);

            //Set output data
            return brepResult;
        }
    }
}